//! Updates the drone's maburd over the maintenance lane (`maint::Hub`), with no
//! PC: the GS carries the drone files and drives the drone's shell.
//!
//! 1. video off; read the drone's maburd, config and what runs;
//! 2. upload maburd, the config (the drone's own, with the keys a release sets
//!    for every drone forced as `tools/drone-install.py` does), S96mabur and
//!    the swap script to /tmp/mabur-up, check each by sha256;
//! 3. start `mabur-swap.sh trial` detached: the old maburd stops and the new
//!    one runs from RAM, flash untouched;
//! 4. reach the drone through the NEW maburd (its shell's parent is
//!    /tmp/mabur-up/maburd) and see video for a while: then commit, which
//!    deletes the old binary and writes the new files to flash;
//! 5. reach the drone through the maburd on flash, with the new sha.
//!
//! Anything wrong before step 4's commit leaves flash as it was: the swap
//! script starts the old maburd again by itself.
//!
//! An app built with `PAL_MABURD` carries its release's maburd (`embedded`)
//! and keeps the drone on it (`spawn_auto`): whenever a drone is heard, it
//! reads the drone's maburd and installs its own when they differ, older or
//! newer.
//!
//! A drone with no cable must come back from a build that passes the trial
//! and fails after a reboot: the commit keeps the last proven set on flash and
//! marks the new one unproven, `PROBE` is the proof that clears the mark, and
//! the swap script's boot guard restores the kept set when no GS comes
//! (release/drone/mabur-swap.sh).

use std::path::{Path, PathBuf};
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use sha2::{Digest, Sha256};

use crate::maint::{Hub, State};
use crate::Shared;

pub const UP: &str = "/tmp/mabur-up";
const SWAP_SH: &[u8] = include_bytes!("../../../release/drone/mabur-swap.sh");

/// Keys an update sets to the shipped value whatever the drone's file holds;
/// "uncalibrated" ones only on a drone whose `radio.power_mode` is not
/// "offset". The same list as `FORCED` in `tools/drone-install.py` (a test
/// compares them).
pub const FORCED: &[(&str, &str, bool)] = &[
    ("venc", "min_iqp", false),
    ("ampdu", "max_num", false),
    ("radio", "wall_margin_db", false),
    ("air_clock", "efficiency_20", true),
    ("air_clock", "efficiency_40", true),
];

const RELEASE_TOML: &[u8] = include_bytes!("../../../release/drone/mabur.toml");
const RELEASE_S96: &[u8] = include_bytes!("../../../release/drone/S96mabur");

/// The drone side of a release, or of a custom build (`custom`).
#[derive(Clone)]
pub struct Payload {
    pub maburd: Vec<u8>,
    /// A release: the shipped mabur.toml, the source of the forced values and
    /// the config of a drone that has none. Custom: the config to install as it
    /// is, or empty to keep the drone's own.
    pub toml: Vec<u8>,
    pub s96: Vec<u8>,
    pub custom: bool,
    /// Custom: a config was asked for that changes what the GS finds
    /// the drone by (`LINK_KEYS`), with the file `ALLOW_LINK_CHANGE`.
    pub allow_link_change: bool,
}

/// The keys a GS finds and reaches the drone by. A custom mabur.toml that
/// changes one is refused unless asked for: a GS that does not follow loses
/// the drone until its trial or its boot guard brings the old set back.
const LINK_KEYS: [(&str, &str); 2] = [("link", "key_file"), ("radio", "channels")];

/// In the payload directory: install a custom mabur.toml that changes `LINK_KEYS`.
pub const ALLOW_LINK_CHANGE: &str = "allow-link-change";

/// The ed25519 key a payload directory's contents must be signed with; the
/// private half stays with whoever builds them.
const PAYLOAD_KEY: [u8; 32] = [
    0xc8, 0x71, 0xff, 0x3d, 0xc9, 0xce, 0x15, 0x8e, 0x9c, 0xec, 0x19, 0x90, 0xe4, 0xdd, 0x28, 0x87,
    0x5d, 0x59, 0xa3, 0x5e, 0xcb, 0x8c, 0x03, 0xd7, 0xa6, 0x6a, 0xb4, 0x72, 0xa3, 0x4f, 0x35, 0x96,
];

/// In the payload directory: the hex signature of `payload_message`.
pub const PAYLOAD_SIG: &str = "payload.sig";
/// In the payload directory: the hex signature of `TOOLS_MSG`, which shows
/// the extra tools in the UI.
pub const TOOLS_SIG: &str = "tools.sig";
pub const TOOLS_MSG: &[u8] = b"palianytsia tools v1\n";

/// What a payload signature covers: every file that goes to the drone and
/// the link-change marker, by sha256 ("-" for a file that is not there).
pub fn payload_message(
    maburd: &[u8],
    toml: Option<&[u8]>,
    s96: Option<&[u8]>,
    allow: bool,
) -> String {
    let h = |b: Option<&[u8]>| b.map_or_else(|| "-".to_string(), sha256_hex);
    format!(
        "palianytsia payload v1\nmaburd {}\nmabur.toml {}\nS96mabur {}\nallow-link-change {}\n",
        sha256_hex(maburd),
        h(toml),
        h(s96),
        u8::from(allow)
    )
}

/// `sig_hex`, the file's text, is `key`'s ed25519 signature of `msg`.
fn signed(key: &[u8; 32], msg: &[u8], sig_hex: &[u8]) -> bool {
    let t = String::from_utf8_lossy(sig_hex);
    let t = t.trim();
    if t.len() != 128 || !t.is_ascii() {
        return false;
    }
    let mut sig = [0u8; 64];
    for (i, b) in sig.iter_mut().enumerate() {
        match u8::from_str_radix(&t[2 * i..2 * i + 2], 16) {
            Ok(v) => *b = v,
            Err(_) => return false,
        }
    }
    ed25519_dalek::VerifyingKey::from_bytes(key).is_ok_and(|k| {
        k.verify_strict(msg, &ed25519_dalek::Signature::from_bytes(&sig))
            .is_ok()
    })
}

/// The `LINK_KEYS` whose values differ between two configs, spaces aside.
fn link_changes(from: &str, to: &str) -> Vec<String> {
    let v = |t: &str, s: &str, k: &str| shipped_value(t, s, k).map(|v| v.replace(' ', ""));
    LINK_KEYS
        .iter()
        .filter(|(s, k)| v(from, s, k) != v(to, s, k))
        .map(|(s, k)| {
            let show = |t: &str| shipped_value(t, s, k).unwrap_or_else(|| "unset".into());
            format!("{s}.{k} {} to {}", show(from), show(to))
        })
        .collect()
}

impl Payload {
    /// `dir` holds maburd, mabur.toml and S96mabur (a release's `drone/`).
    pub fn from_dir(dir: &Path) -> Result<Self, String> {
        let rd = |n: &str| {
            std::fs::read(dir.join(n)).map_err(|e| format!("{}: {e}", dir.join(n).display()))
        };
        Ok(Self {
            maburd: rd("maburd")?,
            toml: rd("mabur.toml")?,
            s96: rd("S96mabur")?,
            custom: false,
            allow_link_change: false,
        })
    }

    /// A custom build in `dir`: maburd, and optionally mabur.toml
    /// (installed as it is) and S96mabur (else the release's), signed with
    /// `PAYLOAD_KEY` in `PAYLOAD_SIG`. None when the directory has no maburd.
    pub fn custom_from_dir(dir: &Path) -> Result<Option<Self>, String> {
        Self::custom_from_dir_with(dir, &PAYLOAD_KEY)
    }

    fn custom_from_dir_with(dir: &Path, key: &[u8; 32]) -> Result<Option<Self>, String> {
        let rd = |n: &str| match std::fs::read(dir.join(n)) {
            Ok(b) => Ok(Some(b)),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
            Err(e) => Err(format!("{}: {e}", dir.join(n).display())),
        };
        let Some(maburd) = rd("maburd")? else {
            return Ok(None);
        };
        // A wrong file would only be refused by the drone's trial, a minute
        // of video later.
        if !maburd.starts_with(b"\x7fELF") {
            return Err(format!(
                "{} is not an ELF binary",
                dir.join("maburd").display()
            ));
        }
        let toml = rd("mabur.toml")?;
        let s96 = rd("S96mabur")?;
        let allow = dir.join(ALLOW_LINK_CHANGE).exists();
        let msg = payload_message(&maburd, toml.as_deref(), s96.as_deref(), allow);
        let sig = rd(PAYLOAD_SIG)?.unwrap_or_default();
        if !signed(key, msg.as_bytes(), &sig) {
            return Err(format!(
                "the build in {} carries no valid signature",
                dir.display()
            ));
        }
        Ok(Some(Self {
            maburd,
            toml: toml.unwrap_or_default(),
            s96: s96.unwrap_or_else(|| RELEASE_S96.to_vec()),
            custom: true,
            allow_link_change: allow,
        }))
    }

    /// The config to install on a drone whose /etc/mabur.toml is `drone`, and
    /// a line per change.
    fn config_for(&self, drone: &str) -> Result<(String, Vec<String>), String> {
        if self.custom {
            if !self.toml.is_empty() {
                let t = String::from_utf8_lossy(&self.toml).into_owned();
                let link = if drone.trim().is_empty() {
                    vec![]
                } else {
                    link_changes(drone, &t)
                };
                if !link.is_empty() && !self.allow_link_change {
                    return Err(format!(
                        "the custom mabur.toml changes {}: a GS that does not follow loses the drone. \
                         To install it anyway, put a file named {ALLOW_LINK_CHANGE} beside it; \
                         without a GS reaching it, \
                         the drone's trial or boot guard brings the old set back",
                        link.join(", ")
                    ));
                }
                let ch = if t == drone {
                    vec![]
                } else {
                    vec!["replaced by the custom mabur.toml".to_string()]
                };
                return Ok((t, ch));
            }
            if drone.trim().is_empty() {
                return Err(
                    "the drone has no /etc/mabur.toml: put one beside the custom maburd"
                        .to_string(),
                );
            }
            return Ok((drone.to_string(), vec![]));
        }
        let base = if drone.trim().is_empty() {
            String::from_utf8_lossy(&self.toml).into_owned()
        } else {
            drone.to_string()
        };
        Ok(force_keys(&base, &String::from_utf8_lossy(&self.toml)))
    }

    /// The config a drone must hold besides the maburd to count as current:
    /// a custom payload's own, None when the drone's config is kept.
    fn exact_config(&self) -> Option<String> {
        (self.custom && !self.toml.is_empty())
            .then(|| String::from_utf8_lossy(&self.toml).into_owned())
    }
}

/// The drone files this build carries, or None for a build without
/// `PAL_MABURD` (it updates no drone unless given a payload directory). The config and init script are the release's own
/// (`release/drone`).
pub fn embedded() -> Option<Payload> {
    const MABURD: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/maburd"));
    (!MABURD.is_empty()).then(|| Payload {
        maburd: MABURD.to_vec(),
        toml: RELEASE_TOML.to_vec(),
        s96: RELEASE_S96.to_vec(),
        custom: false,
        allow_link_change: false,
    })
}

/// How long the payload directory's files must stay unchanged before they are
/// taken: a build or a copy is not taken half-written.
pub const STILL: Duration = Duration::from_secs(2);

type Stamp = Vec<Option<(u64, SystemTime)>>;
/// A custom payload, its maburd's sha256 and a number that changes with each
/// load.
type CustomBuild = (Payload, String, u64);

/// The payload directory, watched: its payload once the files are still.
pub struct PayloadDir {
    dir: PathBuf,
    still: Duration,
    stamp: Stamp,
    since: Instant,
    /// The build of `stamp`; None until loaded.
    loaded: Option<Result<Option<CustomBuild>, String>>,
    loads: u64,
    key: [u8; 32],
}

impl PayloadDir {
    pub fn new(dir: PathBuf) -> Self {
        Self::with(dir, STILL, PAYLOAD_KEY)
    }

    fn with(dir: PathBuf, still: Duration, key: [u8; 32]) -> Self {
        Self {
            dir,
            still,
            stamp: Vec::new(),
            since: Instant::now(),
            loaded: None,
            loads: 0,
            key,
        }
    }

    /// The extra tools show: the directory holds a valid `TOOLS_SIG`.
    pub fn tools(&self) -> bool {
        std::fs::read(self.dir.join(TOOLS_SIG)).is_ok_and(|s| signed(&self.key, TOOLS_MSG, &s))
    }

    pub fn dir(&self) -> &Path {
        &self.dir
    }

    fn stamp_now(&self) -> Stamp {
        [
            "maburd",
            "mabur.toml",
            "S96mabur",
            ALLOW_LINK_CHANGE,
            PAYLOAD_SIG,
        ]
        .iter()
        .map(|n| {
            std::fs::metadata(self.dir.join(n))
                .ok()
                .map(|m| (m.len(), m.modified().unwrap_or(SystemTime::UNIX_EPOCH)))
        })
        .collect()
    }

    /// Ok(None): no maburd in the directory (custom payload off). Err: the files
    /// are changing, or the maburd is no binary.
    pub fn poll(&mut self) -> Result<Option<(&Payload, &str, u64)>, String> {
        let s = self.stamp_now();
        if s != self.stamp {
            self.stamp = s;
            self.since = Instant::now();
            self.loaded = None;
        }
        if self.stamp[0].is_none() {
            return Ok(None);
        }
        if self.since.elapsed() < self.still {
            return Err(format!(
                "the custom maburd is being written; it is taken once unchanged for {} s",
                self.still.as_secs()
            ));
        }
        if self.loaded.is_none() {
            self.loads += 1;
            let n = self.loads;
            self.loaded = Some(
                Payload::custom_from_dir_with(&self.dir, &self.key).map(|p| {
                    p.map(|p| (sha256_hex(&p.maburd), p))
                        .map(|(s, p)| (p, s, n))
                }),
            );
        }
        match self.loaded.as_ref().expect("loaded above") {
            Ok(Some((p, sha, n))) => Ok(Some((p, sha.as_str(), *n))),
            Ok(None) => Ok(None),
            Err(e) => Err(e.clone()),
        }
    }
}

pub fn sha256_hex(b: &[u8]) -> String {
    Sha256::digest(b)
        .iter()
        .map(|x| format!("{x:02x}"))
        .collect()
}

fn header_of(line: &str) -> Option<&str> {
    let t = line.trim();
    let t = t.strip_prefix('[')?;
    let end = t.find(']')?;
    let rest = t[end + 1..].trim();
    (rest.is_empty() || rest.starts_with('#')).then(|| t[..end].trim())
}

/// (everything before the value, the value) of `key = value  # comment`.
fn key_line<'a>(line: &'a str, key: &str) -> Option<(&'a str, &'a str)> {
    let ind = line.len() - line.trim_start().len();
    let after = line[ind..].strip_prefix(key)?.trim_start();
    let after_eq = after.strip_prefix('=')?;
    let v_start = line.len() - after_eq.trim_start().len();
    let v_all = &line[v_start..];
    let v = v_all.find('#').map_or(v_all, |i| &v_all[..i]).trim_end();
    Some((&line[..v_start], v))
}

/// The raw TOML value of section.key in `text`.
pub fn shipped_value(text: &str, section: &str, key: &str) -> Option<String> {
    let mut inside = false;
    for line in text.lines() {
        if line.trim_start().starts_with('[') {
            inside = header_of(line) == Some(section);
            continue;
        }
        if inside {
            if let Some((_, v)) = key_line(line, key) {
                return Some(v.trim().to_string());
            }
        }
    }
    None
}

/// Sets each FORCED key to the shipped value: in place (the comment stays),
/// added under its section, or added with a new section at the end. Returns
/// the text and a line per change.
pub fn force_keys(text: &str, shipped: &str) -> (String, Vec<String>) {
    let calibrated = shipped_value(text, "radio", "power_mode").as_deref() == Some("\"offset\"");
    let mut text = text.to_string();
    let mut changes = Vec::new();
    for &(section, key, uncal_only) in FORCED {
        if uncal_only && calibrated {
            continue;
        }
        let Some(value) = shipped_value(shipped, section, key) else {
            continue;
        };
        let mut lines: Vec<String> = text.split_inclusive('\n').map(str::to_string).collect();
        let (mut inside, mut header, mut done) = (false, None, false);
        for (i, line) in lines.iter_mut().enumerate() {
            let bare = line.trim_end_matches(['\r', '\n']).to_string();
            if bare.trim_start().starts_with('[') {
                inside = header_of(&bare) == Some(section);
                if inside && header.is_none() {
                    header = Some(i);
                }
                continue;
            }
            if !inside {
                continue;
            }
            if let Some((head, old)) = key_line(&bare, key) {
                done = true;
                if old.starts_with('[') && !old.ends_with(']') {
                    changes.push(format!(
                        "{section}.{key} spans several lines; left as it is"
                    ));
                } else if old != value {
                    let tail = &bare[head.len() + old.len()..];
                    let eol = line[bare.len()..].to_string();
                    changes.push(format!("{section}.{key} {old} set to {value}"));
                    *line = format!("{head}{value}{tail}{eol}");
                }
                break;
            }
        }
        if !done {
            match header {
                Some(h) => lines.insert(h + 1, format!("{key} = {value}\n")),
                None => {
                    if lines.last().is_some_and(|l| !l.ends_with('\n')) {
                        lines.last_mut().unwrap().push('\n');
                    }
                    lines.push(format!("[{section}]\n{key} = {value}\n"));
                }
            }
            changes.push(format!("{section}.{key} added at {value}"));
        }
        text = lines.concat();
    }
    (text, changes)
}

/// What the caller sees while an update runs.
pub trait Progress {
    /// A new step, in words.
    fn step(&mut self, what: &str);
    /// Bytes of the current upload the drone has.
    fn bytes(&mut self, done: u64, total: u64);
}

pub struct Report {
    pub old_sha: String,
    pub new_sha: String,
    pub config_changes: Vec<String>,
    pub secs: f64,
}

/// Why a command on the drone gave no result.
#[derive(Clone, Debug, PartialEq)]
pub enum ExecError {
    /// No answer to the OPEN: no drone in reach, or a maburd without the
    /// maintenance lane.
    NoAnswer,
    /// The drone refused or ended the session (`maint::STATUS_*`).
    Closed(i16),
    Other(String),
}

impl std::fmt::Display for ExecError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            Self::NoAnswer => write!(f, "the drone does not answer"),
            Self::Closed(st) if *st == pal_wire::maint::STATUS_ARMED => {
                write!(f, "the drone is armed")
            }
            Self::Closed(st) => write!(f, "the drone refused or ended the session (status {st})"),
            Self::Other(e) => f.write_str(e),
        }
    }
}

impl From<ExecError> for String {
    fn from(e: ExecError) -> String {
        e.to_string()
    }
}

/// One command on the drone: (exit status, output). `input` goes to its
/// stdin. Err when the drone does not answer within `answer` or the command
/// does not end within `limit`, or the session is refused.
pub fn exec(
    hub: &Hub,
    cmd: &str,
    input: &[u8],
    answer: Duration,
    limit: Duration,
    on_bytes: impl FnMut(u64),
) -> Result<(i16, Vec<u8>), ExecError> {
    exec_until_stall(hub, cmd, input, answer, limit, limit, on_bytes)
}

/// [`exec`] that also gives up once `stall` passes with no stdin byte acked
/// and no output byte read: a session whose link went away (a card that
/// dropped off the bus, a link restart) ends there instead of at `limit`.
fn exec_until_stall(
    hub: &Hub,
    cmd: &str,
    input: &[u8],
    answer: Duration,
    limit: Duration,
    stall: Duration,
    mut on_bytes: impl FnMut(u64),
) -> Result<(i16, Vec<u8>), ExecError> {
    let id = hub.open(cmd);
    let t0 = Instant::now();
    let (mut off, mut out) = (0usize, Vec::new());
    let (mut seen, mut moved) = ((0usize, 0usize), Instant::now());
    let r = loop {
        if off < input.len() {
            off += hub.write(id, &input[off..]);
        }
        if off == input.len() {
            hub.finish(id);
        }
        hub.read(id, &mut out);
        let Some(i) = hub.info(id) else {
            break Err(ExecError::Other("session lost".to_string()));
        };
        let acked = off - hub.unacked(id);
        on_bytes(acked as u64);
        if (acked, out.len()) != seen {
            seen = (acked, out.len());
            moved = Instant::now();
        }
        match i.state {
            State::Exited(st) => {
                hub.read(id, &mut out);
                break Ok((st, out));
            }
            State::Closed(st) => break Err(ExecError::Closed(st)),
            State::Opening if t0.elapsed() > answer => break Err(ExecError::NoAnswer),
            State::Opening => {}
            _ if moved.elapsed() > stall => {
                break Err(ExecError::Other(format!(
                    "`{cmd}` made no progress for {} s",
                    stall.as_secs()
                )))
            }
            _ if t0.elapsed() > limit => {
                break Err(ExecError::Other(format!(
                    "`{cmd}` did not end in {} s",
                    limit.as_secs()
                )))
            }
            _ => {}
        }
        std::thread::sleep(Duration::from_millis(5));
    };
    hub.close(id);
    r
}

/// The drone as one probe sees it.
#[derive(Debug, Default)]
struct Probe {
    /// What the shell's parent runs: the maburd that answered.
    exe: String,
    sha: String,
    state: String,
    config: String,
    /// The U-Boot `soc` variable (what tools/drone-install.py checks too):
    /// "ssc338q" on the SigmaStar drones, "hi3516cv610" on the HiSi camera.
    soc: String,
    /// The radio die maburd brought up ("RTL8822E", "RTL8812A"), empty when
    /// nothing on the drone names it (`radio_of`).
    radio: String,
}

/// The radio die from the probe's three sources, most direct first: the file
/// maburd writes at start (`/tmp/mabur-radio`, `chip=RTL8822E generation=...`),
/// the `maburd radio caps: RTL8822E jaguar3 ...` line of its log (every maburd
/// since the caps line, but the log's rotation can drop it), and a USB id only
/// one family uses. A USB id is no proof in general: `0bda:881a` ships on the
/// RTL8812AU (Jaguar1) and on other dies, and devourer picks the driver by the
/// chip-id register, not by the id.
fn radio_of(file: &str, caps: &str, usb: &str) -> String {
    let file_chip = file
        .split_whitespace()
        .find_map(|t| t.strip_prefix("chip="))
        .filter(|c| *c != "?");
    let caps_chip = caps
        .split_once("maburd radio caps:")
        .and_then(|(_, rest)| rest.split_whitespace().next())
        .filter(|c| *c != "?");
    let usb_chip = usb
        .split_whitespace()
        .any(|id| id.eq_ignore_ascii_case("0bda:a81a"))
        .then_some("RTL8822E");
    file_chip
        .or(caps_chip)
        .or(usb_chip)
        .unwrap_or("")
        .to_string()
}

/// The drone radios Palianytsia supports, as devourer builds their drivers in
/// or out (DEVOURER_JAGUAR3_8822E, DEVOURER_JAGUAR1). Any other die (an
/// RTL8812CU's RTL8822C among them) gets no update.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Radio {
    /// RTL8822E (RTL8812EU modules): Jaguar3, the 8822E variant.
    R8822e,
    /// RTL8812A/8811A/8821A (RTL8812AU): Jaguar1.
    Jaguar1,
}

impl Radio {
    const ALL: [Self; 2] = [Self::R8822e, Self::Jaguar1];

    /// The driver for the die maburd names in its caps line (devourer's
    /// AdapterCaps chip_name).
    fn of_chip(chip: &str) -> Option<Self> {
        match chip {
            "RTL8822E" => Some(Self::R8822e),
            "RTL8812A" | "RTL8811A" | "RTL8821A" => Some(Self::Jaguar1),
            _ => None,
        }
    }

    /// Text devourer leaves in a build that has this driver, and the refusal it
    /// compiles in when the driver is left out (WiFiDriver.cpp). Both must
    /// agree, so a renamed string reads as "not carried", never as carried.
    fn marks(self) -> (&'static [u8], &'static [u8]) {
        match self {
            Self::R8822e => (
                b"config_phydm_switch_channel_8822e",
                b"RTL8822E (chip-id 0x17) support not compiled in",
            ),
            Self::Jaguar1 => (b"15RtlJaguarDevice", b"Jaguar1 support not compiled in"),
        }
    }

    /// The drivers this maburd carries.
    pub fn in_maburd(bin: &[u8]) -> Vec<Self> {
        let has = |n: &[u8]| bin.windows(n.len()).any(|w| w == n);
        Self::ALL
            .into_iter()
            .filter(|r| {
                let (with, without) = r.marks();
                has(with) && !has(without)
            })
            .collect()
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::R8822e => "RTL8822E",
            Self::Jaguar1 => "RTL8812AU (Jaguar1)",
        }
    }
}

/// What a maburd is built for: its SoC family and, on SigmaStar, the radio
/// drivers it carries.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Build {
    pub platform: Option<Platform>,
    pub radios: Vec<Radio>,
}

impl Build {
    pub fn of_maburd(bin: &[u8]) -> Self {
        Self {
            platform: Platform::of_maburd(bin),
            radios: Radio::in_maburd(bin),
        }
    }
}

/// The SoC families maburd is built for. A build runs on one only: the
/// SigmaStar maburd is glibc and opens libmi_*, the Hi3516CV610 one is musl
/// and links the HiSi MPP (libss_mpi*); the wrong one does not even start.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Platform {
    Star6e,
    Cv610,
}

impl Platform {
    fn of_soc(soc: &str) -> Option<Self> {
        let s = soc.trim();
        if s.starts_with("ssc33") || s.starts_with("ssc30k") {
            Some(Self::Star6e)
        } else if s.starts_with("hi3516cv61") {
            Some(Self::Cv610)
        } else {
            None
        }
    }

    /// From the libraries the binary names.
    pub fn of_maburd(bin: &[u8]) -> Option<Self> {
        let has = |n: &[u8]| bin.windows(n.len()).any(|w| w == n);
        match (has(b"libmi_sys.so"), has(b"libss_mpi")) {
            (true, false) => Some(Self::Star6e),
            (false, true) => Some(Self::Cv610),
            _ => None,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Star6e => "SigmaStar",
            Self::Cv610 => "Hi3516CV610",
        }
    }
}

/// Whether a maburd built for `build` runs on the drone of `p`; the reason
/// when not. A drone or a build of no known family gets nothing: a wrong
/// binary would only fail its trial, but it cuts the video for a minute. On
/// SigmaStar the radio counts
/// too: a build without the drone's radio driver cannot open the card, and a
/// drone whose radio nothing names gets nothing either.
fn fits(p: &Probe, build: &Build) -> Result<(), String> {
    fits_platform(p, build.platform)?;
    if build.platform != Some(Platform::Star6e) {
        return Ok(());
    }
    let carried = || {
        build
            .radios
            .iter()
            .map(|r| r.name())
            .collect::<Vec<_>>()
            .join(", ")
    };
    match Radio::of_chip(&p.radio) {
        Some(r) if build.radios.contains(&r) => Ok(()),
        Some(r) => Err(format!(
            "this maburd has no driver for the drone's radio {} (it carries {})",
            r.name(),
            carried()
        )),
        None if p.radio.is_empty() => Err(
            "the drone's radio is unknown (no /tmp/mabur-radio, no caps line in its log, \
             no USB id of one family): restart the drone's maburd so it names its card"
                .into(),
        ),
        None => Err(format!(
            "the drone's radio {} is none this app's maburd can carry",
            p.radio
        )),
    }
}

fn fits_platform(p: &Probe, build: Option<Platform>) -> Result<(), String> {
    match (Platform::of_soc(&p.soc), build) {
        (Some(d), Some(b)) if d == b => Ok(()),
        (Some(d), Some(b)) => Err(format!(
            "this maburd is built for {}, the drone is {} ({})",
            b.name(),
            d.name(),
            p.soc
        )),
        (None, _) => Err(format!(
            "the drone's SoC ({:?}) is none maburd is built for",
            p.soc
        )),
        (_, None) => {
            Err("this maburd names neither libmi_sys nor libss_mpi: its platform is unknown".into())
        }
    }
}

/// Also the proof the swap script's boot guard waits for: this shell runs
/// under the installed maburd in a boot later than its commit, so the flag
/// that marks it unproven goes (release/drone/mabur-swap.sh).
const PROBE: &str = "e=$(readlink /proc/$PPID/exe); f=/etc/mabur.unproven; \
    [ \"$e\" = /usr/bin/maburd ] && [ -f $f ] && \
    [ \"$(cat $f)\" != \"$(cat /proc/sys/kernel/random/boot_id)\" ] && rm -f $f; \
    echo \"exe=$e\"; \
    echo \"sha=$(sha256sum /usr/bin/maburd 2>/dev/null | cut -d' ' -f1)\"; \
    echo \"state=$(cat /tmp/mabur-up/state 2>/dev/null)\"; \
    echo \"soc=$(fw_printenv -n soc 2>/dev/null)\"; \
    echo \"radio=$(head -n 1 /tmp/mabur-radio 2>/dev/null)\"; \
    echo \"caps=$(grep -h -m 1 'maburd radio caps:' /tmp/mabur.log /tmp/mabur.log.prev 2>/dev/null | head -n 1)\"; \
    echo \"usb=$(for d in /sys/bus/usb/devices/*; do [ -f $d/idProduct ] && printf '%s:%s ' $(cat $d/idVendor) $(cat $d/idProduct); done)\"; \
    echo ---; cat /etc/mabur.toml";

fn probe(hub: &Hub, answer: Duration) -> Result<Probe, ExecError> {
    let (st, out) = exec(hub, PROBE, &[], answer, Duration::from_secs(10), |_| {})?;
    if st != 0 {
        return Err(ExecError::Other(format!("probe exited {st}")));
    }
    let text = String::from_utf8_lossy(&out);
    let (head, config) = text.split_once("---\n").unwrap_or((&text, ""));
    Ok(parse_probe(head, config))
}

fn parse_probe(head: &str, config: &str) -> Probe {
    let mut p = Probe {
        config: config.to_string(),
        ..Probe::default()
    };
    let (mut file, mut caps, mut usb) = ("", "", "");
    for l in head.lines() {
        if let Some((k, v)) = l.split_once('=') {
            match k {
                "exe" => p.exe = v.to_string(),
                "sha" => p.sha = v.to_string(),
                "state" => p.state = v.to_string(),
                "soc" => p.soc = v.to_string(),
                "radio" => file = v,
                "caps" => caps = v,
                "usb" => usb = v,
                _ => {}
            }
        }
    }
    p.radio = radio_of(file, caps, usb);
    p
}

/// Probes until `ok` holds, the swap script reports a rollback, or `limit` passes.
fn wait_for(hub: &Hub, limit: Duration, ok: impl Fn(&Probe) -> bool) -> Result<Probe, String> {
    let t0 = Instant::now();
    while t0.elapsed() < limit {
        if let Ok(p) = probe(hub, Duration::from_secs(3)) {
            if ok(&p) {
                return Ok(p);
            }
            if matches!(
                p.state.as_str(),
                "rollback" | "restored" | "commit_failed" | "aborted_armed"
            ) {
                let log = exec(
                    hub,
                    "tail -n 25 /tmp/mabur-up/swap.log",
                    &[],
                    Duration::from_secs(3),
                    Duration::from_secs(5),
                    |_| {},
                )
                .map(|(_, o)| String::from_utf8_lossy(&o).into_owned())
                .unwrap_or_default();
                return Err(format!("the drone reports {}:\n{log}", p.state));
            }
        }
        std::thread::sleep(Duration::from_millis(200));
    }
    Err(format!(
        "no answer as expected within {} s",
        limit.as_secs()
    ))
}

/// Runs the whole update; `link_ok` says whether video is coming in now,
/// `may_go` whether the drone may restart maburd (`fc_not_armed`): asked
/// before the upload, before the trial and before the commit.
pub fn run(
    hub: &Hub,
    pl: &Payload,
    link_ok: &dyn Fn() -> bool,
    may_go: &dyn Fn() -> Result<(), String>,
    prog: &mut dyn Progress,
) -> Result<Report, String> {
    let t0 = Instant::now();
    let new_sha = sha256_hex(&pl.maburd);
    prog.step("reading the drone");
    let now = probe(hub, Duration::from_secs(5))?;
    if now.exe != "/usr/bin/maburd" {
        return Err(format!(
            "the drone runs {} (not the installed maburd): finish or undo that first",
            now.exe
        ));
    }
    fits(&now, &Build::of_maburd(&pl.maburd))?;
    let (config, config_changes) = pl.config_for(&now.config)?;
    if now.sha == new_sha && config == now.config {
        return Err("the drone already runs this maburd with this config".to_string());
    }
    may_go()?;
    hub.set_video_off(true);
    let r = upload_and_swap(hub, pl, &config, &new_sha, link_ok, may_go, prog);
    hub.set_video_off(false);
    r?;
    Ok(Report {
        old_sha: now.sha,
        new_sha,
        config_changes,
        secs: t0.elapsed().as_secs_f64(),
    })
}

/// An upload ends this long after its last acked byte.
const UPLOAD_STALL: Duration = Duration::from_secs(20);

/// The whole-upload bound: 4 kB/s plus 30 s. A Quest moved a 1445636 B maburd
/// in 85.6 s (16.9 kB/s, 2026-11-15); the old 20 kB/s bound gave it 102 s.
fn upload_limit(total: u64) -> Duration {
    Duration::from_secs(30 + total / 4_000)
}

fn upload_and_swap(
    hub: &Hub,
    pl: &Payload,
    config: &str,
    new_sha: &str,
    link_ok: &dyn Fn() -> bool,
    may_go: &dyn Fn() -> Result<(), String>,
    prog: &mut dyn Progress,
) -> Result<(), String> {
    let files: [(&str, &[u8]); 4] = [
        ("maburd", &pl.maburd),
        ("mabur.toml", config.as_bytes()),
        ("S96mabur", &pl.s96),
        ("swap.sh", SWAP_SH),
    ];
    exec(
        hub,
        &format!("rm -rf {UP}; mkdir -p {UP}"),
        &[],
        Duration::from_secs(5),
        Duration::from_secs(10),
        |_| {},
    )?;
    for (name, data) in files {
        prog.step(&format!("uploading {name}"));
        let total = data.len() as u64;
        let (st, _) = exec_until_stall(
            hub,
            &format!("cat > {UP}/{name}"),
            data,
            Duration::from_secs(5),
            upload_limit(total),
            UPLOAD_STALL,
            |d| prog.bytes(d, total),
        )?;
        if st != 0 {
            return Err(format!("writing {UP}/{name} on the drone failed ({st})"));
        }
    }
    prog.step("checking the uploads");
    let (_, out) = exec(
        hub,
        &format!("cd {UP} && sha256sum maburd mabur.toml S96mabur swap.sh"),
        &[],
        Duration::from_secs(5),
        Duration::from_secs(20),
        |_| {},
    )?;
    let got = String::from_utf8_lossy(&out);
    for (name, data) in files {
        let want = sha256_hex(data);
        if !got
            .lines()
            .any(|l| l.starts_with(&want) && l.ends_with(name))
        {
            return Err(format!(
                "{name} arrived corrupt on the drone; nothing changed"
            ));
        }
    }
    may_go()?;
    prog.step("starting the new maburd (trial, flash untouched)");
    // The old maburd carries this shell and stops seconds after the script
    // starts, so the answer can be lost on the way; the probes below find
    // out whether the trial runs. A refusal means it never started.
    if let Err(e @ ExecError::Closed(_)) = exec(
        hub,
        &format!("chmod 755 {UP}/maburd {UP}/swap.sh && setsid {UP}/swap.sh trial </dev/null >/dev/null 2>&1 & echo started"),
        &[],
        Duration::from_secs(5),
        Duration::from_secs(10),
        |_| {},
    ) {
        return Err(e.into());
    }
    // The new maburd starts with video on; the lease would turn it off again.
    hub.set_video_off(false);
    let trial_exe = format!("{UP}/maburd");
    wait_for(hub, Duration::from_secs(60), |p| {
        p.exe == trial_exe && p.state == "trial"
    })?;
    prog.step("checking video from the new maburd");
    let t = Instant::now();
    let mut good_since: Option<Instant> = None;
    loop {
        if link_ok() {
            if good_since.get_or_insert_with(Instant::now).elapsed() >= Duration::from_secs(3) {
                break;
            }
        } else {
            good_since = None;
        }
        if t.elapsed() > Duration::from_secs(40) {
            return Err(
                "no steady video from the new maburd; the drone restores the old one by itself"
                    .to_string(),
            );
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    may_go().map_err(|e| {
        format!("{e}: no commit; the drone goes back to its maburd once it is disarmed")
    })?;
    prog.step("committing: the old maburd is deleted, the new one written to flash");
    exec(
        hub,
        &format!("touch {UP}/commit && echo ok"),
        &[],
        Duration::from_secs(5),
        Duration::from_secs(10),
        |_| {},
    )?;
    wait_for(hub, Duration::from_secs(90), |p| {
        p.exe == "/usr/bin/maburd" && p.sha == new_sha && p.state == "committed"
    })?;
    prog.step("done");
    Ok(())
}

/// A `link_ok` for `run`: decoded frames rose within the last 500 ms.
pub fn video_flows(sh: &Shared) -> impl Fn() -> bool + '_ {
    let last = std::cell::Cell::new((sh.frames.load(Ordering::Relaxed), Instant::now()));
    move || {
        let n = sh.frames.load(Ordering::Relaxed);
        let (seen, at) = last.get();
        if n != seen {
            last.set((n, Instant::now()));
            return true;
        }
        at.elapsed() < Duration::from_millis(500)
    }
}

/// A `may_go` for `run`: no update while the flight controller's last
/// HEARTBEAT says armed. No heartbeat (no FC, as on the camera, or MSP
/// telemetry off) does not hold it: the GS meets the drone on the ground
/// before a flight and updates it then (Ihor, 2026-11-14). The drone's own
/// swap script holds the restart on its "armed" word the same way.
pub fn fc_not_armed(sh: &Shared) -> impl Fn() -> Result<(), String> + '_ {
    move || match sh.fc_armed() {
        Some(true) => Err("the drone is armed".to_string()),
        _ => Ok(()),
    }
}

/// What the automatic update does after a probe.
#[derive(Debug, PartialEq)]
enum Next {
    /// Probe again after this long; the state names why.
    Wait(Duration, &'static str),
    /// Nothing to do until the drone is heard anew.
    Settled(&'static str),
    /// The drone's maburd differs and this app already tried once: the
    /// failure stays on screen; trying again on every contact would cut the
    /// video for a minute each time.
    Keep,
    Update,
}

/// `want`: the maburd's sha256, and the config the drone must hold too (a custom
/// payload's own; None keeps the drone's). `build`: the platform that maburd
/// is for. `fc`: the flight controller's arm state, None when not heard.
fn decide(
    p: &Result<Probe, ExecError>,
    want: (&str, Option<&str>),
    build: &Build,
    tried: bool,
    fc: Option<bool>,
) -> Next {
    let (want, config) = want;
    match p {
        Err(ExecError::NoAnswer) => Next::Wait(Duration::from_secs(20), "no_lane"),
        Err(ExecError::Closed(st)) if *st == pal_wire::maint::STATUS_ARMED => {
            Next::Wait(Duration::from_secs(3), "armed")
        }
        Err(_) => Next::Wait(Duration::from_secs(10), "error"),
        // A swap in progress (another GS, or one this app lost): the drone
        // ends it by itself within the swap script's commit wait.
        Ok(p) if p.exe != "/usr/bin/maburd" => Next::Wait(Duration::from_secs(10), "busy"),
        Ok(p) if p.sha == want && config.is_none_or(|c| c == p.config) => Next::Settled("current"),
        Ok(p) if fits(p, build).is_err() => Next::Settled("other_platform"),
        Ok(_) if tried => Next::Keep,
        Ok(_) if fc == Some(true) => Next::Wait(Duration::from_secs(3), "armed"),
        Ok(_) => Next::Update,
    }
}

/// (drone, wanted) pairs this process is trying or has tried, across link
/// restarts (a Quest restarts its link when a card comes or goes): a pair
/// whose try reached the trial is not tried again; one that failed before
/// it goes back out of here ([`try_again`]). A new custom build makes a new pair.
static TRIED: std::sync::Mutex<Vec<String>> = std::sync::Mutex::new(Vec::new());

/// One entry per failed try of a TRIED key, for [`try_again`].
static FAILED: std::sync::Mutex<Vec<String>> = std::sync::Mutex::new(Vec::new());

/// Tries of one (drone, wanted) pair when each failed before the trial.
const TRIES_BEFORE_TRIAL: usize = 3;

/// Whether a try that failed at `step`, the `fails`th failure of its pair,
/// is tried again. Up to the trial start the drone is untouched (it still
/// runs its maburd; the uploads go to /tmp), so a lost link or a slow lane is
/// worth another go. From the trial on, the drone took part: a broken
/// build is not pushed at it again.
fn try_again(step: &str, fails: usize) -> bool {
    let untouched = step == "reading the drone"
        || step.starts_with("uploading ")
        || step == "checking the uploads";
    untouched && fails < TRIES_BEFORE_TRIAL
}

/// How soon a failed try that [`try_again`] allows runs again.
const RETRY_AFTER: Duration = Duration::from_secs(10);

/// How often a pair this process already tried is probed again: another
/// link session's thread may still hold it, or give it back for a retry.
const TRIED_RECHECK: Duration = Duration::from_secs(30);

/// How long "updated" stays on screen.
const UPDATED_SHOW: Duration = Duration::from_secs(15);

/// Keeps the drone on this app's maburd while the link runs: a thread that
/// probes the drone whenever it is heard (a session, or Telems in another
/// RC version: the lane works across versions) and runs the update when the
/// drone's maburd differs. A maburd in `custom` (`PayloadDir`) overrides the app's
/// own. Its state goes to `Shared::update`, and from there to the UI stats
/// line. No-op for a build without a payload and without a payload directory.
pub fn spawn_auto(sh: Arc<Shared>, custom: Option<PathBuf>) {
    let emb = embedded();
    if emb.is_none() && custom.is_none() {
        return;
    }
    if let Some(d) = &custom {
        log::info!("drone update: payload directory {}", d.display());
    }
    let r = std::thread::Builder::new()
        .name("pal-dupdate".into())
        .spawn(move || auto_loop(&sh, emb, custom.map(PayloadDir::new)));
    if let Err(e) = r {
        log::warn!("drone update thread: {e}");
    }
}

struct AutoProgress<'a> {
    sh: &'a Shared,
    base: serde_json::Value,
    step: String,
}

impl Progress for AutoProgress<'_> {
    fn step(&mut self, what: &str) {
        self.step = what.to_string();
        let mut v = self.base.clone();
        v["state"] = "updating".into();
        v["step"] = what.into();
        self.sh.set_update(v);
        self.sh
            .event("drone_update", serde_json::json!({ "step": what }));
    }
    fn bytes(&mut self, done: u64, total: u64) {
        let mut v = self.base.clone();
        v["state"] = "updating".into();
        v["step"] = self.step.as_str().into();
        v["done"] = done.into();
        v["total"] = total.into();
        self.sh.set_update(v);
    }
}

fn short(sha: &str) -> &str {
    &sha[..sha.len().min(12)]
}

/// The TRIED entry of updating the drone of `p` to `want` (plus `config`).
fn tried_key(p: &Probe, want: &str, config: Option<&str>) -> String {
    match config {
        Some(c) => format!(
            "{} {} > {want} {}",
            p.sha,
            sha256_hex(p.config.as_bytes()),
            sha256_hex(c.as_bytes())
        ),
        None => format!("{} > {want}", p.sha),
    }
}

fn auto_loop(sh: &Shared, emb: Option<Payload>, mut custom: Option<PayloadDir>) {
    let emb_sha = emb.as_ref().map(|p| sha256_hex(&p.maburd));
    // The payload in use: 0 the app's own, else the PayloadDir load number.
    let mut using: Option<u64> = None;
    let mut wait_shown = String::new();
    let (mut was_heard, mut settled) = (false, false);
    let mut next = Instant::now();
    let mut updated_at: Option<Instant> = None;
    while !sh.quit.load(Ordering::Relaxed) && !sh.core_done.load(Ordering::Acquire) {
        sh.tools_on.store(
            custom.as_ref().is_some_and(PayloadDir::tools),
            Ordering::Relaxed,
        );
        let picked = match custom.as_mut().map(PayloadDir::poll) {
            Some(Ok(Some(x))) => Ok(Some(x)),
            Some(Err(e)) => Err(e),
            _ => Ok(emb.as_ref().zip(emb_sha.as_deref()).map(|(p, s)| (p, s, 0))),
        };
        let (pl, want, id) = match picked {
            Ok(Some(x)) => x,
            Ok(None) => {
                // Custom payload off in a build with no maburd of its own.
                if using.take().is_some() {
                    *sh.update.lock().unwrap_or_else(|e| e.into_inner()) = None;
                }
                std::thread::sleep(Duration::from_millis(250));
                continue;
            }
            Err(e) => {
                if e != wait_shown {
                    sh.set_update(
                        serde_json::json!({ "state": "custom_wait", "custom": true, "error": e }),
                    );
                    wait_shown = e;
                }
                using = None;
                std::thread::sleep(Duration::from_millis(250));
                continue;
            }
        };
        wait_shown.clear();
        let base = |state: &str| {
            let mut v = serde_json::json!({ "state": state, "app": short(want) });
            if pl.custom {
                v["custom"] = true.into();
            }
            v
        };
        if using != Some(id) {
            using = Some(id);
            settled = false;
            next = Instant::now();
            updated_at = None;
            sh.set_update(base("waiting"));
        }
        if updated_at.is_some_and(|t| t.elapsed() > UPDATED_SHOW) {
            updated_at = None;
            let mut v = base("current");
            v["drone"] = short(want).into();
            sh.set_update(v);
        }
        let heard = sh.drone_heard.load(Ordering::Relaxed);
        if heard && !was_heard {
            settled = false;
            next = Instant::now();
        }
        was_heard = heard;
        if !heard || settled || Instant::now() < next {
            std::thread::sleep(Duration::from_millis(250));
            continue;
        }
        let config = pl.exact_config();
        let p = probe(&sh.maint, Duration::from_secs(3));
        // Asking for a link change is a new try of the same build.
        let key = p.as_ref().ok().map(|p| {
            tried_key(p, want, config.as_deref()) + if pl.allow_link_change { " +link" } else { "" }
        });
        let tried = key
            .as_ref()
            .is_some_and(|k| TRIED.lock().unwrap_or_else(|e| e.into_inner()).contains(k));
        let mut v = base("");
        if let Ok(p) = &p {
            v["drone"] = short(&p.sha).into();
            if p.state == "guard_restored" {
                v["fallback"] = true.into();
            }
        }
        let build = Build::of_maburd(&pl.maburd);
        match decide(&p, (want, config.as_deref()), &build, tried, sh.fc_armed()) {
            Next::Wait(d, state) => {
                v["state"] = state.into();
                if let Err(e) = &p {
                    v["error"] = e.to_string().into();
                }
                sh.set_update(v);
                next = Instant::now() + d;
            }
            Next::Settled(state) => {
                v["state"] = state.into();
                if let Some(Err(e)) = p.as_ref().ok().map(|p| fits(p, &build)) {
                    v["error"] = e.into();
                }
                sh.set_update(v);
                settled = true;
            }
            Next::Keep => next = Instant::now() + TRIED_RECHECK,
            Next::Update => {
                let from = p.map(|p| p.sha).unwrap_or_default();
                TRIED
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .extend(key.clone());
                sh.event(
                    "drone_update",
                    serde_json::json!({ "from": from, "to": want, "custom": pl.custom }),
                );
                let mut retry = false;
                let mut prog = AutoProgress {
                    sh,
                    base: v.clone(),
                    step: String::new(),
                };
                match run(
                    &sh.maint,
                    pl,
                    &video_flows(sh),
                    &fc_not_armed(sh),
                    &mut prog,
                ) {
                    Ok(r) => {
                        v["state"] = "updated".into();
                        v["secs"] = r.secs.into();
                        sh.event(
                            "drone_update",
                            serde_json::json!({ "done_s": r.secs, "config": r.config_changes }),
                        );
                        updated_at = Some(Instant::now());
                    }
                    Err(e) => {
                        log::warn!("drone update: {e}");
                        let fails = {
                            let mut f = FAILED.lock().unwrap_or_else(|e| e.into_inner());
                            f.extend(key.clone());
                            key.as_ref()
                                .map_or(0, |k| f.iter().filter(|x| *x == k).count())
                        };
                        let again = key.is_some() && try_again(&prog.step, fails);
                        v["state"] = "failed".into();
                        if again {
                            TRIED
                                .lock()
                                .unwrap_or_else(|e| e.into_inner())
                                .retain(|k| Some(k) != key.as_ref());
                            v["error"] =
                                format!("{e}; trying again in {} s", RETRY_AFTER.as_secs()).into();
                            next = Instant::now() + RETRY_AFTER;
                            retry = true;
                        } else {
                            v["error"] = e.as_str().into();
                        }
                        sh.event(
                            "drone_update",
                            serde_json::json!({ "failed": e, "again": again }),
                        );
                    }
                }
                sh.set_update(v);
                settled = !retry;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SHIPPED: &str =
        "[venc]\nmin_iqp = 42   # floor\n[ampdu]\nmax_num = 0\n[radio]\nwall_margin_db = 6.0\n\
        [air_clock]\nefficiency_20 = [1, 2]\nefficiency_40 = [3, 4]\n";

    #[test]
    fn a_try_that_failed_before_the_trial_runs_again_up_to_three_times() {
        for step in [
            "reading the drone",
            "uploading maburd",
            "uploading swap.sh",
            "checking the uploads",
        ] {
            assert!(try_again(step, 1), "{step}");
            assert!(try_again(step, 2), "{step}");
            assert!(!try_again(step, 3), "{step}");
        }
        for step in [
            "starting the new maburd (trial, flash untouched)",
            "checking video from the new maburd",
            "committing: the old maburd is deleted, the new one written to flash",
            "",
        ] {
            assert!(!try_again(step, 1), "{step}");
        }
    }

    #[test]
    fn the_upload_bound_leaves_a_slow_quest_lane_room() {
        // 2026-11-15: 1445636 B in 85.6 s; the old bound was 102 s.
        assert!(upload_limit(1_445_636) >= Duration::from_secs(3 * 86));
        assert_eq!(upload_limit(0), Duration::from_secs(30));
    }

    #[test]
    fn force_keys_replaces_in_place_adds_missing_and_skips_calibrated_tables() {
        let drone = "[radio]\npower_mode = \"offset\"\nwall_margin_db = 1.0  # mine\n[venc]\nfps = 60\nmin_iqp = 30 # old\n";
        let (t, ch) = force_keys(drone, SHIPPED);
        assert!(t.contains("wall_margin_db = 6.0  # mine\n"), "{t}");
        assert!(t.contains("min_iqp = 42 # old\n"), "{t}");
        assert!(t.contains("[ampdu]\nmax_num = 0\n"), "{t}");
        assert!(
            !t.contains("efficiency_20"),
            "calibrated: its own tables stay"
        );
        assert_eq!(ch.len(), 3, "{ch:?}");
        let (t2, ch2) = force_keys(&t, SHIPPED);
        assert_eq!((t2, ch2.len()), (t, 0), "a second run changes nothing");
        let (u, _) = force_keys("[venc]\nfps = 60", SHIPPED);
        assert!(
            u.contains("efficiency_40 = [3, 4]")
                && u.starts_with("[venc]\nmin_iqp = 42\nfps = 60\n"),
            "{u}"
        );
    }

    #[test]
    fn the_automatic_update_installs_any_other_maburd_of_the_drones_platform_once_unless_armed() {
        let drone = |exe: &str, sha: &str| {
            Ok(Probe {
                exe: exe.into(),
                sha: sha.into(),
                soc: "ssc338q".into(),
                radio: "RTL8822E".into(),
                ..Probe::default()
            })
        };
        let on_flash = "/usr/bin/maburd";
        let star = &Build {
            platform: Some(Platform::Star6e),
            radios: vec![Radio::R8822e],
        };
        let down = Some(false);
        assert_eq!(
            decide(&drone(on_flash, "aa"), ("aa", None), star, false, None),
            Next::Settled("current")
        );
        assert_eq!(
            decide(&drone(on_flash, "bb"), ("aa", None), star, false, down),
            Next::Update,
            "older or newer"
        );
        assert_eq!(
            decide(&drone(on_flash, "bb"), ("aa", None), star, false, None),
            Next::Update,
            "no FC heard (none, or MSP telemetry off) does not hold it"
        );
        assert_eq!(
            decide(&drone(on_flash, "bb"), ("aa", None), star, true, down),
            Next::Keep
        );
        assert!(matches!(
            decide(
                &drone(on_flash, "bb"),
                ("aa", None),
                star,
                false,
                Some(true)
            ),
            Next::Wait(_, "armed")
        ));
        assert_eq!(
            decide(
                &drone(on_flash, "bb"),
                ("aa", None),
                &Build {
                    platform: Some(Platform::Cv610),
                    radios: vec![],
                },
                false,
                down
            ),
            Next::Settled("other_platform"),
            "a camera build never goes on a SigmaStar drone"
        );
        assert_eq!(
            decide(
                &drone(on_flash, "bb"),
                ("aa", None),
                &Build {
                    platform: None,
                    radios: vec![Radio::R8822e],
                },
                false,
                down
            ),
            Next::Settled("other_platform"),
            "nor a build of no known platform"
        );
        assert!(matches!(
            decide(
                &drone("/tmp/mabur-up/maburd", "bb"),
                ("aa", None),
                star,
                false,
                down
            ),
            Next::Wait(_, "busy")
        ));
        let armed = Err(ExecError::Closed(pal_wire::maint::STATUS_ARMED));
        assert!(matches!(
            decide(&armed, ("aa", None), star, false, down),
            Next::Wait(_, "armed")
        ));
        assert!(matches!(
            decide(&Err(ExecError::NoAnswer), ("aa", None), star, false, down),
            Next::Wait(_, "no_lane")
        ));
        // A custom payload with its own config: a drone with the right maburd
        // and another config is not current.
        let mut p = drone(on_flash, "aa");
        if let Ok(p) = &mut p {
            p.config = "x = 1\n".into();
        }
        assert_eq!(
            decide(&p, ("aa", Some("x = 1\n")), star, false, None),
            Next::Settled("current")
        );
        assert_eq!(
            decide(&p, ("aa", Some("x = 2\n")), star, false, down),
            Next::Update
        );
    }

    #[test]
    fn a_maburd_goes_only_to_a_drone_of_its_platform() {
        let star: &[u8] = b"\x7fELF..libstdc++.so.6\0libmi_sys.so\0libmi_venc.so\0";
        let cam: &[u8] = b"\x7fELF..libss_mpi.so\0libss_mpi_isp.so\0libot_osal.so\0";
        assert_eq!(Platform::of_maburd(star), Some(Platform::Star6e));
        assert_eq!(Platform::of_maburd(cam), Some(Platform::Cv610));
        assert_eq!(Platform::of_maburd(b"\x7fELF test binary"), None);
        let on = |soc: &str| Probe {
            soc: soc.into(),
            ..Probe::default()
        };
        assert_eq!(
            fits_platform(&on("ssc338q"), Some(Platform::Star6e)),
            Ok(())
        );
        assert_eq!(
            fits_platform(&on("hi3516cv610\n"), Some(Platform::Cv610)),
            Ok(())
        );
        let e = fits_platform(&on("hi3516cv610"), Some(Platform::Star6e)).unwrap_err();
        assert!(
            e.contains("built for SigmaStar") && e.contains("Hi3516CV610"),
            "{e}"
        );
        assert!(
            fits_platform(&on(""), Some(Platform::Star6e)).is_err(),
            "SoC unread"
        );
        assert!(fits_platform(&on("ssc338q"), None).is_err());
    }

    // The two drones of the bench: the quad (RTL8812EU, 0bda:a81a) and the
    // wing (RTL8812AU, 0bda:881a). What a probe printed, from an older maburd
    // with no radio file and a rotated log, and from one with both.
    const QUAD_BARE: &str = "exe=/usr/bin/maburd\nsha=aa\nstate=\nsoc=ssc338q\nradio=\ncaps=\n\
        usb=1d6b:0002 0bda:a81a \n";
    const WING_BARE: &str = "exe=/usr/bin/maburd\nsha=aa\nstate=\nsoc=ssc338q\nradio=\ncaps=\n\
        usb=1d6b:0002 0bda:881a \n";
    const WING_CAPS: &str = "exe=/usr/bin/maburd\nsha=aa\nstate=\nsoc=ssc338q\nradio=\n\
        caps=maburd radio caps: RTL8812A jaguar1 2x2 bw=3 tune5g=5170-5835 fast_retune=0 \
        sensors fa=1 igi=1 nhm=0 floor=0\nusb=1d6b:0002 0bda:881a \n";

    fn maburd_with(radios: &[Radio]) -> Vec<u8> {
        let mut b = b"\x7fELF..libstdc++.so.6\0libmi_sys.so\0".to_vec();
        for r in Radio::ALL {
            let (with, without) = r.marks();
            b.extend_from_slice(if radios.contains(&r) { with } else { without });
            b.push(0);
        }
        b
    }

    #[test]
    fn the_probe_names_the_radio_by_file_then_log_then_a_one_family_usb_id() {
        assert_eq!(parse_probe(QUAD_BARE, "").radio, "RTL8822E");
        assert_eq!(parse_probe(WING_BARE, "").radio, "", "881a is no proof");
        assert_eq!(parse_probe(WING_CAPS, "").radio, "RTL8812A");
        let both = WING_CAPS.replace("radio=\n", "radio=chip=RTL8822E generation=jaguar3\n");
        assert_eq!(parse_probe(&both, "").radio, "RTL8822E", "the file wins");
        assert_eq!(radio_of("chip=? generation=?", "", "0bda:a81a"), "RTL8822E");
        let p = parse_probe(QUAD_BARE, "[radio]\n");
        assert_eq!(
            (p.soc.as_str(), p.config.as_str()),
            ("ssc338q", "[radio]\n")
        );
    }

    #[test]
    fn a_maburd_names_the_radio_drivers_it_carries() {
        let all = [Radio::R8822e, Radio::Jaguar1];
        assert_eq!(Radio::in_maburd(&maburd_with(&all)), all.to_vec());
        assert_eq!(
            Radio::in_maburd(&maburd_with(&[Radio::R8822e])),
            vec![Radio::R8822e]
        );
        // Both marks (a renamed refusal string would read the same way): not carried.
        let mut b = maburd_with(&[Radio::Jaguar1]);
        b.extend_from_slice(Radio::Jaguar1.marks().1);
        assert_eq!(Radio::in_maburd(&b), vec![]);
    }

    #[test]
    fn an_8822e_only_maburd_never_goes_to_the_wing_and_a_two_radio_one_goes_to_both() {
        let e_only = Build::of_maburd(&maburd_with(&[Radio::R8822e]));
        let both_radios = Build::of_maburd(&maburd_with(&Radio::ALL));
        assert_eq!(e_only.platform, Some(Platform::Star6e));
        let quad = parse_probe(QUAD_BARE, "");
        let wing = parse_probe(WING_BARE, "");
        let wing_named = parse_probe(WING_CAPS, "");
        assert_eq!(fits(&quad, &e_only), Ok(()));
        assert_eq!(fits(&quad, &both_radios), Ok(()));
        let e = fits(&wing, &e_only).unwrap_err();
        assert!(e.contains("radio is unknown"), "{e}");
        let e = fits(&wing_named, &e_only).unwrap_err();
        assert!(e.contains("no driver") && e.contains("RTL8812AU"), "{e}");
        assert_eq!(fits(&wing_named, &both_radios), Ok(()));
        assert!(
            fits(&wing, &both_radios).is_err(),
            "an unnamed radio gets nothing"
        );
        // An RTL8812CU drone is not supported: nothing goes to it.
        let cu = parse_probe(
            &WING_CAPS.replace("RTL8812A jaguar1", "RTL8822C jaguar3"),
            "",
        );
        let e = fits(&cu, &both_radios).unwrap_err();
        assert!(e.contains("RTL8822C"), "{e}");
        // Through the decision the automatic update takes.
        let down = Some(false);
        assert_eq!(
            decide(&Ok(wing_named), ("bb", None), &e_only, false, down),
            Next::Settled("other_platform")
        );
        assert_eq!(
            decide(&Ok(quad), ("bb", None), &e_only, false, down),
            Next::Update
        );
        // A camera build carries no Realtek driver and needs none.
        let cam = Build {
            platform: Some(Platform::Cv610),
            radios: vec![],
        };
        let p = Probe {
            soc: "hi3516cv610".into(),
            ..Probe::default()
        };
        assert_eq!(fits(&p, &cam), Ok(()));
    }

    fn payload_dir(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("pal-custom-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    const TEST_SEED: [u8; 32] = [7; 32];

    fn test_key() -> [u8; 32] {
        ed25519_dalek::SigningKey::from_bytes(&TEST_SEED)
            .verifying_key()
            .to_bytes()
    }

    fn sign_hex(seed: &[u8; 32], msg: &[u8]) -> String {
        use ed25519_dalek::Signer;
        ed25519_dalek::SigningKey::from_bytes(seed)
            .sign(msg)
            .to_bytes()
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect::<String>()
            + "\n"
    }

    /// Signs the set in `d` with the test key, as the private script does.
    fn sign_dir(d: &Path) {
        let rd = |n: &str| std::fs::read(d.join(n)).ok();
        let Some(m) = rd("maburd") else { return };
        let msg = payload_message(
            &m,
            rd("mabur.toml").as_deref(),
            rd("S96mabur").as_deref(),
            d.join(ALLOW_LINK_CHANGE).exists(),
        );
        std::fs::write(d.join(PAYLOAD_SIG), sign_hex(&TEST_SEED, msg.as_bytes())).unwrap();
    }

    /// The set in `d`, signed first.
    fn load(d: &Path) -> Result<Option<Payload>, String> {
        sign_dir(d);
        Payload::custom_from_dir_with(d, &test_key())
    }

    #[test]
    fn a_payload_is_taken_only_signed_and_the_signature_covers_every_file() {
        let d = payload_dir("sig");
        let k = test_key();
        std::fs::write(d.join("maburd"), b"\x7fELF build").unwrap();
        let e = Payload::custom_from_dir_with(&d, &k).err().unwrap();
        assert!(e.contains("no valid signature"), "{e}");
        sign_dir(&d);
        assert!(Payload::custom_from_dir_with(&d, &k).unwrap().is_some());
        assert!(
            Payload::custom_from_dir(&d).is_err(),
            "the test key is not the build's key"
        );
        // Each file the signature names, changed or added after signing.
        for (f, body) in [
            ("maburd", &b"\x7fELF other"[..]),
            ("mabur.toml", b"a = 1\n"),
            ("S96mabur", b"#!/bin/sh\n"),
            (ALLOW_LINK_CHANGE, b""),
        ] {
            sign_dir(&d);
            std::fs::write(d.join(f), body).unwrap();
            assert!(
                Payload::custom_from_dir_with(&d, &k).is_err(),
                "{f} changed"
            );
            sign_dir(&d);
            assert!(Payload::custom_from_dir_with(&d, &k).is_ok(), "{f} signed");
        }
        let other = sign_hex(&[9; 32], b"x");
        std::fs::write(d.join(PAYLOAD_SIG), other).unwrap();
        assert!(
            Payload::custom_from_dir_with(&d, &k).is_err(),
            "another key"
        );
        std::fs::write(d.join(PAYLOAD_SIG), "zz".repeat(64)).unwrap();
        assert!(Payload::custom_from_dir_with(&d, &k).is_err(), "not hex");

        // The tools: a signature of their own message, by the same key.
        let w = PayloadDir::with(d.clone(), STILL, k);
        assert!(!w.tools());
        std::fs::write(d.join(TOOLS_SIG), sign_hex(&[9; 32], TOOLS_MSG)).unwrap();
        assert!(!w.tools(), "another key");
        std::fs::write(d.join(TOOLS_SIG), sign_hex(&TEST_SEED, b"tools")).unwrap();
        assert!(!w.tools(), "another message");
        std::fs::write(d.join(TOOLS_SIG), sign_hex(&TEST_SEED, TOOLS_MSG)).unwrap();
        assert!(w.tools());
        assert!(!PayloadDir::new(d.clone()).tools(), "not the build's key");
        let _ = std::fs::remove_dir_all(&d);
    }

    /// A directory the private script filled (PAL_PAYLOAD_DIR): its build and
    /// tools pass with the key this build carries.
    #[test]
    #[ignore]
    fn a_directory_signed_by_the_script_passes() {
        let d = PathBuf::from(std::env::var("PAL_PAYLOAD_DIR").unwrap());
        assert!(Payload::custom_from_dir(&d).unwrap().is_some());
        assert!(PayloadDir::new(d).tools());
    }

    #[test]
    fn a_payload_directory_gives_its_maburd_its_config_if_any_and_the_release_init_script() {
        let d = payload_dir("payload");
        assert!(load(&d).unwrap().is_none(), "no maburd: off");
        std::fs::write(d.join("maburd"), "#!/bin/sh\n").unwrap();
        let e = load(&d).err().unwrap();
        assert!(e.contains("not an ELF"), "{e}");
        std::fs::write(d.join("maburd"), b"\x7fELF custom build").unwrap();
        let p = load(&d).unwrap().unwrap();
        assert!(p.custom && p.toml.is_empty());
        assert_eq!(p.s96, RELEASE_S96);
        assert_eq!(p.exact_config(), None);
        let drone = "[radio]\nwall_margin_db = 1.0\n";
        assert_eq!(
            p.config_for(drone).unwrap(),
            (drone.to_string(), vec![]),
            "the drone's config as it is: no forced keys"
        );
        assert!(p.config_for("").is_err(), "nothing to keep");

        std::fs::write(d.join("mabur.toml"), "[venc]\nfps = 90\n").unwrap();
        std::fs::write(d.join("S96mabur"), "#!/bin/sh\n# custom\n").unwrap();
        let p = load(&d).unwrap().unwrap();
        let (t, ch) = p.config_for(drone).unwrap();
        assert_eq!((t.as_str(), ch.len()), ("[venc]\nfps = 90\n", 1));
        assert_eq!(p.exact_config().as_deref(), Some("[venc]\nfps = 90\n"));
        assert_eq!(p.s96, b"#!/bin/sh\n# custom\n");

        // A custom config that moves the key or the channel set is refused
        // unless asked for.
        let linked = "[radio]\nchannels = [40, 64]\n[link]\nkey_file = \"/etc/mabur.key\"\n";
        std::fs::write(
            d.join("mabur.toml"),
            "[radio]\nchannels = [40,64]   # same set\n[link]\nkey_file = \"/etc/mabur.key\"\nx = 1\n",
        )
        .unwrap();
        let p = load(&d).unwrap().unwrap();
        assert!(
            p.config_for(linked).is_ok(),
            "spaces and comments aside, the same"
        );
        std::fs::write(
            d.join("mabur.toml"),
            "[radio]\nchannels = [149]\n[link]\nkey_file = \"/etc/other.key\"\n",
        )
        .unwrap();
        let p = load(&d).unwrap().unwrap();
        let e = p.config_for(linked).unwrap_err();
        assert!(
            e.contains("link.key_file \"/etc/mabur.key\" to \"/etc/other.key\"")
                && e.contains("radio.channels [40, 64] to [149]")
                && e.contains(ALLOW_LINK_CHANGE),
            "{e}"
        );
        assert!(
            p.config_for("").is_ok(),
            "a drone with no config has nothing to lose"
        );
        std::fs::write(d.join(ALLOW_LINK_CHANGE), "").unwrap();
        let p = load(&d).unwrap().unwrap();
        assert!(p.allow_link_change && p.config_for(linked).is_ok());

        // A release payload still forces its keys.
        let rel = Payload {
            custom: false,
            toml: SHIPPED.as_bytes().to_vec(),
            ..p.clone()
        };
        let (t, _) = rel.config_for(drone).unwrap();
        assert!(t.contains("wall_margin_db = 6.0"), "{t}");
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn the_payload_directory_is_taken_only_once_its_files_stay_unchanged() {
        let d = payload_dir("watch");
        let mut w = PayloadDir::with(d.clone(), Duration::from_millis(300), test_key());
        assert!(matches!(w.poll(), Ok(None)), "empty: custom payload off");
        std::fs::write(d.join("maburd"), b"\x7fELF one").unwrap();
        sign_dir(&d);
        assert!(w.poll().is_err(), "just written");
        std::thread::sleep(Duration::from_millis(350));
        let (sha1, n1) = {
            let (p, sha, n) = w.poll().unwrap().unwrap();
            assert_eq!(p.maburd, b"\x7fELF one");
            (sha.to_string(), n)
        };
        assert_eq!(w.poll().unwrap().unwrap().2, n1, "loaded once");
        std::fs::write(d.join("maburd"), b"\x7fELF two, longer").unwrap();
        sign_dir(&d);
        assert!(w.poll().is_err(), "a new build waits again");
        std::thread::sleep(Duration::from_millis(350));
        let (p, sha2, n2) = w.poll().unwrap().unwrap();
        assert_eq!(p.maburd, b"\x7fELF two, longer");
        assert!(sha2 != sha1 && n2 != n1);
        std::fs::remove_file(d.join("maburd")).unwrap();
        assert!(
            matches!(w.poll(), Ok(None)),
            "removed: back to the app's own"
        );
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_new_custom_build_is_a_new_try() {
        let p = Probe {
            sha: "old".into(),
            config: "a = 1\n".into(),
            ..Probe::default()
        };
        assert_ne!(tried_key(&p, "one", None), tried_key(&p, "two", None));
        assert_ne!(
            tried_key(&p, "one", Some("b = 1\n")),
            tried_key(&p, "one", Some("b = 2\n"))
        );
    }

    #[test]
    fn forced_list_matches_drone_install_py() {
        let py = include_str!("../../../tools/drone-install.py");
        let start = py.find("FORCED = (").unwrap();
        let block = &py[start..start + py[start..].find("\n)").unwrap()];
        let rows: Vec<(String, String, bool)> = block
            .lines()
            .filter_map(|l| {
                let l = l
                    .trim()
                    .trim_end_matches(',')
                    .strip_prefix('(')?
                    .strip_suffix(')')?;
                let p: Vec<&str> = l.split(',').map(|x| x.trim().trim_matches('"')).collect();
                Some((p[0].to_string(), p[1].to_string(), p[2] == "uncalibrated"))
            })
            .collect();
        let ours: Vec<(String, String, bool)> = FORCED
            .iter()
            .map(|(s, k, u)| (s.to_string(), k.to_string(), *u))
            .collect();
        assert_eq!(rows, ours);
    }

    /// The swap script on the host, its paths moved into a temp dir: a fake
    /// init script and a fake maburd stand in. The fake maburd writes the arm
    /// file from `fc` (the FC's word as the test sets it), as maburd does
    /// from the FC's reports; `fc` starts out disarmed.
    fn swap_world(name: &str) -> std::path::PathBuf {
        let d = std::env::temp_dir().join(format!("pal-swap-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(d.join("up")).unwrap();
        let w = |p: &str, s: &str| {
            let p = d.join(p);
            std::fs::write(&p, s).unwrap();
            std::process::Command::new("chmod")
                .arg("755")
                .arg(&p)
                .status()
                .unwrap();
        };
        w("bin", "old binary\n");
        w("conf", "old = 1\n");
        // The init script logs its calls; "start" is all a test needs.
        w(
            "init",
            &format!("#!/bin/sh\necho \"$1\" >> {}/init.calls\n", d.display()),
        );
        w(
            "up/maburd",
            &format!(
                "#!/bin/sh\nwhile true; do cp {0}/fc {0}/arm.tmp 2>/dev/null && mv {0}/arm.tmp {0}/arm; sleep 0.1; done\n",
                d.display()
            ),
        );
        std::fs::write(d.join("fc"), "disarmed\n").unwrap();
        w("up/mabur.toml", "new = 1\n");
        w("up/S96mabur", "#!/bin/sh\necho new init\n");
        std::fs::write(d.join("boot_id"), "boot-1\n").unwrap();
        d
    }

    const SWAP_PATH: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../release/drone/mabur-swap.sh"
    );

    /// The swap script with `arg`, every path in the world `d`.
    fn swap_cmd(d: &std::path::Path, arg: &str) -> std::process::Command {
        let mut c = std::process::Command::new("sh");
        c.arg(SWAP_PATH)
            .arg(arg)
            .env("UP", d.join("up"))
            .env("BIN", d.join("bin"))
            .env("CONF", d.join("conf"))
            .env("INIT", d.join("init"))
            .env("STOP_WAIT", "2")
            .env("START_DELAY", "0")
            .env("NAME", "pal-swap-test-no-such-process")
            .env("ARM", d.join("arm"))
            .env("LKG", d.join("lkg"))
            .env("FLAG", d.join("unproven"))
            .env("GUARD", d.join("guard"))
            .env("BOOT_ID", d.join("boot_id"))
            .env("LKG_WAIT", "30")
            .env("GUARD_POLL", "1");
        c
    }

    fn swap(d: &std::path::Path, wait: u32) -> std::process::Child {
        swap_cmd(d, "trial")
            .env("COMMIT_WAIT", wait.to_string())
            .spawn()
            .unwrap()
    }

    /// A trial of what is in `up/`, committed.
    fn commit(d: &std::path::Path) {
        let mut c = swap(d, 30);
        wait_state(d, "trial");
        std::fs::write(d.join("up/commit"), "").unwrap();
        assert!(c.wait().unwrap().success());
        assert_eq!(state(d), "committed");
    }

    fn read(d: &std::path::Path, p: &str) -> String {
        std::fs::read_to_string(d.join(p)).unwrap_or_default()
    }

    fn gunzip(p: &std::path::Path) -> Vec<u8> {
        std::process::Command::new("gunzip")
            .arg("-c")
            .arg(p)
            .output()
            .unwrap()
            .stdout
    }

    fn state(d: &std::path::Path) -> String {
        std::fs::read_to_string(d.join("up/state"))
            .unwrap_or_default()
            .trim()
            .to_string()
    }

    fn wait_state(d: &std::path::Path, want: &str) {
        let t = Instant::now();
        while state(d) != want && t.elapsed() < Duration::from_secs(10) {
            std::thread::sleep(Duration::from_millis(50));
        }
        assert_eq!(state(d), want);
    }

    fn init_calls(d: &std::path::Path) -> String {
        std::fs::read_to_string(d.join("init.calls")).unwrap_or_default()
    }

    /// maburd's shell inherits its descriptors (devourer's flock on the card
    /// among them); the new maburd and the init loop must not.
    #[test]
    fn swap_script_does_not_pass_inherited_descriptors_on() {
        let d = swap_world("fds");
        let lock = d.join("devourer-usb-1-1.lock");
        let dump = |who: &str| format!("ls -l /proc/$$/fd > {}/{who}.fds", d.display());
        std::fs::write(
            d.join("up/maburd"),
            format!(
                "#!/bin/sh\n{}\nwhile true; do cp {1}/fc {1}/arm.tmp 2>/dev/null && mv {1}/arm.tmp {1}/arm; sleep 0.1; done\n",
                dump("maburd"),
                d.display()
            ),
        )
        .unwrap();
        std::fs::write(
            d.join("init"),
            format!(
                "#!/bin/sh\n{}\necho \"$1\" >> {}/init.calls\n",
                dump("init"),
                d.display()
            ),
        )
        .unwrap();
        let base = swap_cmd(&d, "trial");
        let mut c = std::process::Command::new("sh");
        c.arg("-c")
            .arg(format!(
                "exec 7<>{}; exec sh {SWAP_PATH} trial",
                lock.display()
            ))
            .env("COMMIT_WAIT", "30");
        for (k, v) in base.get_envs() {
            if let Some(v) = v {
                c.env(k, v);
            }
        }
        let mut c = c.spawn().unwrap();
        wait_state(&d, "trial");
        let t = Instant::now();
        while read(&d, "maburd.fds").is_empty() && t.elapsed() < Duration::from_secs(5) {
            std::thread::sleep(Duration::from_millis(50));
        }
        std::fs::write(d.join("up/commit"), "").unwrap();
        assert!(c.wait().unwrap().success());
        assert_eq!(state(&d), "committed");
        for who in ["maburd", "init"] {
            let fds = read(&d, &format!("{who}.fds"));
            assert!(!fds.is_empty(), "{who} listed no descriptors");
            assert!(
                !fds.contains("devourer-usb"),
                "{who} inherited the card lock:\n{fds}"
            );
        }
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn swap_script_commits_on_the_commit_file() {
        let d = swap_world("commit");
        let mut c = swap(&d, 30);
        let t = Instant::now();
        while state(&d) != "trial" && t.elapsed() < Duration::from_secs(10) {
            std::thread::sleep(Duration::from_millis(50));
        }
        assert_eq!(state(&d), "trial");
        assert_eq!(
            std::fs::read_to_string(d.join("bin")).unwrap(),
            "old binary\n",
            "flash untouched in trial"
        );
        std::fs::write(d.join("up/commit"), "").unwrap();
        assert!(c.wait().unwrap().success());
        assert_eq!(state(&d), "committed");
        assert_eq!(
            std::fs::read(d.join("bin")).unwrap(),
            std::fs::read(d.join("up/maburd")).unwrap()
        );
        assert_eq!(
            std::fs::read_to_string(d.join("conf")).unwrap(),
            "new = 1\n"
        );
        assert_eq!(
            std::fs::read_to_string(d.join("init")).unwrap(),
            "#!/bin/sh\necho new init\n"
        );
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn swap_script_restores_the_old_maburd_without_a_commit_or_when_the_new_one_exits() {
        let d = swap_world("rollback");
        assert!(swap(&d, 2).wait().unwrap().success());
        assert_eq!(state(&d), "restored");
        assert_eq!(
            std::fs::read_to_string(d.join("bin")).unwrap(),
            "old binary\n"
        );
        assert_eq!(
            std::fs::read_to_string(d.join("conf")).unwrap(),
            "old = 1\n"
        );
        let calls = std::fs::read_to_string(d.join("init.calls")).unwrap();
        assert_eq!(calls, "stop\nstart\n");
        std::fs::write(d.join("up/maburd"), "#!/bin/sh\nexit 1\n").unwrap();
        std::fs::remove_file(d.join("init.calls")).unwrap();
        assert!(swap(&d, 30).wait().unwrap().success());
        assert_eq!(state(&d), "restored");
        let log = std::fs::read_to_string(d.join("up/swap.log")).unwrap();
        assert!(log.contains("the new maburd exited (1)"), "{log}");
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn swap_script_does_not_start_a_trial_on_an_armed_drone() {
        let d = swap_world("aborted");
        std::fs::write(d.join("arm"), "armed\n").unwrap();
        assert!(swap(&d, 30).wait().unwrap().success());
        assert_eq!(state(&d), "aborted_armed");
        assert_eq!(init_calls(&d), "", "the old maburd was never stopped");
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn swap_script_restarts_maburd_only_on_a_drone_not_armed() {
        // No commit in time while armed: the new maburd keeps flying.
        let d = swap_world("held");
        std::fs::write(d.join("fc"), "armed\n").unwrap();
        let mut c = swap(&d, 1);
        wait_state(&d, "trial_held");
        std::thread::sleep(Duration::from_millis(1500));
        assert_eq!(state(&d), "trial_held");
        assert_eq!(init_calls(&d), "stop\n", "no restart while armed");
        std::fs::write(d.join("fc"), "disarmed\n").unwrap();
        assert!(c.wait().unwrap().success());
        assert_eq!(state(&d), "restored");
        assert_eq!(init_calls(&d), "stop\nstart\n");

        // A commit while armed waits for the disarm; flash stays untouched.
        let d2 = swap_world("commit_held");
        std::fs::write(d2.join("fc"), "armed\n").unwrap();
        let mut c = swap(&d2, 30);
        wait_state(&d2, "trial");
        // The commit comes once the new maburd has spoken, as the GS's does
        // after seconds of its video: a file not yet written is no word.
        let t = Instant::now();
        while !d2.join("arm").exists() && t.elapsed() < Duration::from_secs(10) {
            std::thread::sleep(Duration::from_millis(50));
        }
        std::fs::write(d2.join("up/commit"), "").unwrap();
        wait_state(&d2, "commit_held");
        std::thread::sleep(Duration::from_millis(1500));
        assert_eq!(state(&d2), "commit_held");
        assert_eq!(
            std::fs::read_to_string(d2.join("bin")).unwrap(),
            "old binary\n"
        );
        std::fs::write(d2.join("fc"), "disarmed\n").unwrap();
        assert!(c.wait().unwrap().success());
        assert_eq!(state(&d2), "committed");
        let _ = std::fs::remove_dir_all(&d);
        let _ = std::fs::remove_dir_all(&d2);
    }

    #[test]
    fn swap_script_commits_on_a_drone_with_no_arm_report() {
        // A camera with no FC says "unknown" for good: that held the commit
        // forever before (Hi3516CV610, 2026-11-14).
        let d = swap_world("unknown");
        std::fs::write(d.join("fc"), "unknown\n").unwrap();
        let mut c = swap(&d, 30);
        wait_state(&d, "trial");
        let t = Instant::now();
        while !d.join("arm").exists() && t.elapsed() < Duration::from_secs(10) {
            std::thread::sleep(Duration::from_millis(50));
        }
        std::fs::write(d.join("up/commit"), "").unwrap();
        // Bounded: a script that waits for "disarmed" never ends here.
        let t = Instant::now();
        let st = loop {
            if let Some(st) = c.try_wait().unwrap() {
                break st;
            }
            if t.elapsed() > Duration::from_secs(60) {
                let _ = c.kill();
                panic!("no commit in 60 s, state {}", state(&d));
            }
            std::thread::sleep(Duration::from_millis(50));
        };
        assert!(st.success());
        assert_eq!(state(&d), "committed");
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_commit_to_a_maburd_without_the_arm_word_goes_ahead() {
        // The release maburd before the word (v0.14) writes no arm file; the
        // old one's file says disarmed until the trial removes it.
        let d = swap_world("no_word");
        std::fs::write(
            d.join("up/maburd"),
            "#!/bin/sh\nwhile true; do sleep 1; done\n",
        )
        .unwrap();
        std::fs::write(d.join("arm"), "disarmed\n").unwrap();
        let mut c = swap(&d, 30);
        wait_state(&d, "trial");
        std::fs::write(d.join("up/commit"), "").unwrap();
        // The bench hang sat in commit_held for good; the commit itself may
        // take long here (its sync waits on whatever the host is writing).
        let t = Instant::now();
        while matches!(state(&d).as_str(), "trial" | "commit_held")
            && t.elapsed() < Duration::from_secs(10)
        {
            std::thread::sleep(Duration::from_millis(50));
        }
        let held = state(&d);
        if held == "commit_held" {
            let _ = c.kill();
            let _ = c.wait();
        }
        assert_ne!(
            held, "commit_held",
            "the commit waits for a word that never comes"
        );
        assert!(c.wait().unwrap().success());
        assert_eq!(state(&d), "committed");
        assert_eq!(read(&d, "bin"), read(&d, "up/maburd"));
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_commit_keeps_the_proven_set_and_marks_the_new_one_unproven() {
        let d = swap_world("lkg");
        let old_init = read(&d, "init");
        commit(&d);
        assert_eq!(gunzip(&d.join("lkg/maburd.gz")), b"old binary\n");
        assert_eq!(read(&d, "lkg/mabur.toml"), "old = 1\n");
        assert_eq!(read(&d, "lkg/S96mabur"), old_init);
        assert_eq!(read(&d, "lkg/sha").trim(), sha256_hex(b"old binary\n"));
        assert_eq!(
            read(&d, "unproven"),
            "boot-1\n",
            "the new set waits for a GS after a reboot"
        );
        assert_eq!(
            std::fs::read(d.join("guard")).unwrap(),
            std::fs::read(SWAP_PATH).unwrap(),
            "the script itself is the boot guard"
        );
        // A second build in the same boot: the outgoing one was never proven,
        // so the fallback stays the cable-installed set.
        let build2 = read(&d, "up/maburd") + "# build 2\n";
        std::fs::write(d.join("up/maburd"), build2).unwrap();
        commit(&d);
        assert_eq!(gunzip(&d.join("lkg/maburd.gz")), b"old binary\n");
        assert!(read(&d, "up/swap.log").contains("the outgoing maburd is unproven"));
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_new_set_that_does_not_fit_beside_the_fallback_wins_the_room() {
        let d = swap_world("full");
        // An overlay with room for the new maburd only once the fallback is
        // gone: a cp onto `bin` comes out short while `lkg` exists.
        std::fs::create_dir_all(d.join("path")).unwrap();
        let cp = d.join("path/cp");
        std::fs::write(
            &cp,
            format!(
                "#!/bin/sh\nif [ \"$2\" = {0}/bin ] && [ -d {0}/lkg ]; then head -c 4 \"$1\" > \"$2\"; exit 1; fi\nexec /bin/cp \"$@\"\n",
                d.display()
            ),
        )
        .unwrap();
        std::process::Command::new("chmod")
            .arg("755")
            .arg(&cp)
            .status()
            .unwrap();
        let path = format!(
            "{}:{}",
            d.join("path").display(),
            std::env::var("PATH").unwrap()
        );
        let mut c = swap_cmd(&d, "trial")
            .env("COMMIT_WAIT", "30")
            .env("PATH", path)
            .spawn()
            .unwrap();
        wait_state(&d, "trial");
        std::fs::write(d.join("up/commit"), "").unwrap();
        assert!(c.wait().unwrap().success());
        assert_eq!(state(&d), "committed");
        assert_eq!(read(&d, "bin"), read(&d, "up/maburd"));
        assert!(!d.join("lkg").exists());
        assert!(read(&d, "up/swap.log").contains("the fallback goes"));
        // No fallback: the guard has nothing to put back and leaves the build.
        std::fs::write(d.join("boot_id"), "boot-2\n").unwrap();
        assert!(swap_cmd(&d, "guard")
            .env("LKG_WAIT", "0")
            .status()
            .unwrap()
            .success());
        assert_eq!(read(&d, "bin"), read(&d, "up/maburd"));
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn the_boot_guard_puts_the_fallback_back_when_no_gs_comes_after_a_reboot() {
        let d = swap_world("guard");
        let old_init = read(&d, "init");
        commit(&d);
        // In the boot that committed, the trial is the proof: nothing to do.
        assert!(swap_cmd(&d, "guard")
            .env("LKG_WAIT", "0")
            .status()
            .unwrap()
            .success());
        assert_eq!(read(&d, "bin"), read(&d, "up/maburd"));
        std::fs::write(d.join("boot_id"), "boot-2\n").unwrap();
        // The FC armed: it holds past LKG_WAIT.
        std::fs::write(d.join("arm"), "armed\n").unwrap();
        let mut g = swap_cmd(&d, "guard").env("LKG_WAIT", "1").spawn().unwrap();
        std::thread::sleep(Duration::from_millis(2500));
        assert!(g.try_wait().unwrap().is_none(), "no restart while armed");
        assert_eq!(read(&d, "bin"), read(&d, "up/maburd"));
        std::fs::write(d.join("arm"), "disarmed\n").unwrap();
        assert!(g.wait().unwrap().success());
        assert_eq!(state(&d), "guard_restored");
        assert_eq!(read(&d, "bin"), "old binary\n");
        assert_eq!(read(&d, "conf"), "old = 1\n");
        assert_eq!(read(&d, "init"), old_init);
        assert!(!d.join("unproven").exists(), "the fallback is proven");
        assert!(init_calls(&d).ends_with("start\n"), "{}", init_calls(&d));
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn the_boot_guard_stands_aside_for_a_gs_or_a_new_install() {
        let d = swap_world("guard_aside");
        commit(&d);
        std::fs::write(d.join("boot_id"), "boot-2\n").unwrap();
        // A GS's probe deletes the flag.
        let mut g = swap_cmd(&d, "guard").spawn().unwrap();
        std::thread::sleep(Duration::from_millis(500));
        std::fs::remove_file(d.join("unproven")).unwrap();
        assert!(g.wait().unwrap().success());
        assert!(read(&d, "up/swap.log").contains("a GS reached the drone"));
        assert_eq!(read(&d, "bin"), read(&d, "up/maburd"));
        // Someone replaces maburd (a GS whose probe knows no flag, a cable).
        std::fs::write(d.join("unproven"), "boot-1\n").unwrap();
        let mut g = swap_cmd(&d, "guard").env("LKG_WAIT", "3").spawn().unwrap();
        std::thread::sleep(Duration::from_millis(500));
        std::fs::remove_file(d.join("bin")).unwrap();
        std::fs::write(d.join("bin"), "other install\n").unwrap();
        assert!(g.wait().unwrap().success());
        assert!(read(&d, "up/swap.log").contains("maburd replaced meanwhile"));
        assert_eq!(read(&d, "bin"), "other install\n");
        let _ = std::fs::remove_dir_all(&d);
    }

    /// The probe's own text, run by sh as the drone would: this test is the
    /// maburd the shell's parent is, the flag and boot id live in a temp dir.
    #[test]
    fn the_probe_proves_the_installed_maburd_only_in_a_later_boot() {
        let d = swap_world("probe");
        let me = std::env::current_exe().unwrap();
        let flag = d.join("unproven");
        let at = |p: &str| d.join(p).to_str().unwrap().to_string();
        let moved = PROBE
            .replace("/etc/mabur.unproven", &at("unproven"))
            .replace("/proc/sys/kernel/random/boot_id", &at("boot_id"))
            .replace("/etc/mabur.toml", &at("conf"));
        let mine = moved.replace(
            "[ \"$e\" = /usr/bin/maburd ]",
            &format!("[ \"$e\" = {} ]", me.display()),
        );
        assert_ne!(mine, moved);
        let run = |cmd: &str| {
            let o = std::process::Command::new("sh")
                .arg("-c")
                .arg(cmd)
                .output()
                .unwrap();
            assert!(o.status.success(), "{o:?}");
            String::from_utf8_lossy(&o.stdout).into_owned()
        };
        std::fs::write(&flag, "boot-1\n").unwrap();
        let out = run(&mine);
        assert!(out.contains(&format!("exe={}", me.display())), "{out}");
        assert!(out.ends_with("---\nold = 1\n"), "{out}");
        assert!(
            flag.exists(),
            "same boot as the commit: the trial was the proof"
        );
        std::fs::write(d.join("boot_id"), "boot-2\n").unwrap();
        run(&moved);
        assert!(
            flag.exists(),
            "a shell under another program (a trial from /tmp) proves nothing"
        );
        run(&mine);
        assert!(
            !flag.exists(),
            "a later boot, reached under the installed maburd"
        );
        run(&mine);
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn only_an_fc_heartbeat_saying_armed_holds_an_update() {
        let sh = Shared::default();
        let go = fc_not_armed(&sh);
        assert_eq!(go(), Ok(()), "nothing heard: no FC, or MSP telemetry off");
        let hb = |base_mode: u8, sys: u8| {
            let mut pl = [0u8; 9];
            pl[6] = base_mode;
            pal_wire::mavlink::pack_v2(pal_wire::mavlink::MSG_HEARTBEAT, sys, 1, 0, &pl)
        };
        sh.note_mav(&hb(128 | 1, 1));
        assert_eq!(go(), Err("the drone is armed".to_string()));
        sh.note_mav(&hb(1, 191));
        assert_eq!(
            go(),
            Err("the drone is armed".to_string()),
            "maburd's own component says nothing of the FC"
        );
        sh.note_mav(&hb(1, 1));
        assert_eq!(go(), Ok(()));
        *sh.fc_arm.lock().unwrap() = Some((true, Instant::now() - crate::FC_ARM_STALE));
        assert_eq!(go(), Ok(()), "a stale armed heartbeat holds nothing either");
    }
}
