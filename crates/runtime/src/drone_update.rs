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

use std::time::{Duration, Instant};

use sha2::{Digest, Sha256};

use crate::maint::{Hub, State};

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

/// The drone side of a release.
#[derive(Clone)]
pub struct Payload {
    pub maburd: Vec<u8>,
    /// The shipped mabur.toml: the source of the forced values, and the
    /// config of a drone that has none.
    pub toml: Vec<u8>,
    pub s96: Vec<u8>,
}

impl Payload {
    /// `dir` holds maburd, mabur.toml and S96mabur (a release's `drone/`).
    pub fn from_dir(dir: &std::path::Path) -> Result<Self, String> {
        let rd = |n: &str| {
            std::fs::read(dir.join(n)).map_err(|e| format!("{}: {e}", dir.join(n).display()))
        };
        Ok(Self {
            maburd: rd("maburd")?,
            toml: rd("mabur.toml")?,
            s96: rd("S96mabur")?,
        })
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

/// One command on the drone: (exit status, output). `input` goes to its
/// stdin. Err when the drone does not answer within `answer` or the command
/// does not end within `limit`, or the session is refused.
pub fn exec(
    hub: &Hub,
    cmd: &str,
    input: &[u8],
    answer: Duration,
    limit: Duration,
    mut on_bytes: impl FnMut(u64),
) -> Result<(i16, Vec<u8>), String> {
    let id = hub.open(cmd);
    let t0 = Instant::now();
    let (mut off, mut out) = (0usize, Vec::new());
    let r = loop {
        if off < input.len() {
            off += hub.write(id, &input[off..]);
        }
        if off == input.len() {
            hub.finish(id);
        }
        hub.read(id, &mut out);
        let Some(i) = hub.info(id) else {
            break Err("session lost".to_string());
        };
        on_bytes((off - hub.unacked(id)) as u64);
        match i.state {
            State::Exited(st) => {
                hub.read(id, &mut out);
                break Ok((st, out));
            }
            State::Closed(st) => {
                break Err(format!(
                    "the drone refused or ended the session (status {st})"
                ))
            }
            State::Opening if t0.elapsed() > answer => {
                break Err("the drone does not answer".to_string())
            }
            _ if t0.elapsed() > limit => {
                break Err(format!("`{cmd}` did not end in {} s", limit.as_secs()))
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
}

const PROBE: &str = "echo \"exe=$(readlink /proc/$PPID/exe)\"; \
    echo \"sha=$(sha256sum /usr/bin/maburd 2>/dev/null | cut -d' ' -f1)\"; \
    echo \"state=$(cat /tmp/mabur-up/state 2>/dev/null)\"; echo ---; cat /etc/mabur.toml";

fn probe(hub: &Hub, answer: Duration) -> Result<Probe, String> {
    let (st, out) = exec(hub, PROBE, &[], answer, Duration::from_secs(10), |_| {})?;
    if st != 0 {
        return Err(format!("probe exited {st}"));
    }
    let text = String::from_utf8_lossy(&out);
    let (head, config) = text.split_once("---\n").unwrap_or((&text, ""));
    let mut p = Probe {
        config: config.to_string(),
        ..Probe::default()
    };
    for l in head.lines() {
        if let Some((k, v)) = l.split_once('=') {
            match k {
                "exe" => p.exe = v.to_string(),
                "sha" => p.sha = v.to_string(),
                "state" => p.state = v.to_string(),
                _ => {}
            }
        }
    }
    Ok(p)
}

/// Probes until `ok` holds, the swap script reports a rollback, or `limit` passes.
fn wait_for(hub: &Hub, limit: Duration, ok: impl Fn(&Probe) -> bool) -> Result<Probe, String> {
    let t0 = Instant::now();
    while t0.elapsed() < limit {
        if let Ok(p) = probe(hub, Duration::from_secs(3)) {
            if ok(&p) {
                return Ok(p);
            }
            if matches!(p.state.as_str(), "rollback" | "restored" | "commit_failed") {
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

/// Runs the whole update; `link_ok` says whether video is coming in now.
pub fn run(
    hub: &Hub,
    pl: &Payload,
    link_ok: &dyn Fn() -> bool,
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
    let base = if now.config.trim().is_empty() {
        String::from_utf8_lossy(&pl.toml).into_owned()
    } else {
        now.config.clone()
    };
    let (config, config_changes) = force_keys(&base, &String::from_utf8_lossy(&pl.toml));
    if now.sha == new_sha && config == now.config {
        return Err("the drone already runs this maburd with this config".to_string());
    }
    hub.set_video_off(true);
    let r = upload_and_swap(hub, pl, &config, &new_sha, link_ok, prog);
    hub.set_video_off(false);
    r?;
    Ok(Report {
        old_sha: now.sha,
        new_sha,
        config_changes,
        secs: t0.elapsed().as_secs_f64(),
    })
}

fn upload_and_swap(
    hub: &Hub,
    pl: &Payload,
    config: &str,
    new_sha: &str,
    link_ok: &dyn Fn() -> bool,
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
        let limit = Duration::from_secs(30 + total / 20_000);
        let (st, _) = exec(
            hub,
            &format!("cat > {UP}/{name}"),
            data,
            Duration::from_secs(5),
            limit,
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
    prog.step("starting the new maburd (trial, flash untouched)");
    exec(
        hub,
        &format!("chmod 755 {UP}/maburd {UP}/swap.sh && setsid {UP}/swap.sh trial </dev/null >/dev/null 2>&1 & echo started"),
        &[],
        Duration::from_secs(5),
        Duration::from_secs(10),
        |_| {},
    )?;
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

#[cfg(test)]
mod tests {
    use super::*;

    const SHIPPED: &str =
        "[venc]\nmin_iqp = 42   # floor\n[ampdu]\nmax_num = 0\n[radio]\nwall_margin_db = 6.0\n\
        [air_clock]\nefficiency_20 = [1, 2]\nefficiency_40 = [3, 4]\n";

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
    /// init script and a fake maburd (a shell loop) stand in.
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
        w("up/maburd", "#!/bin/sh\nwhile true; do sleep 1; done\n");
        w("up/mabur.toml", "new = 1\n");
        w("up/S96mabur", "#!/bin/sh\necho new init\n");
        d
    }

    fn swap(d: &std::path::Path, wait: u32) -> std::process::Child {
        std::process::Command::new("sh")
            .arg(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/../../release/drone/mabur-swap.sh"
            ))
            .arg("trial")
            .env("UP", d.join("up"))
            .env("BIN", d.join("bin"))
            .env("CONF", d.join("conf"))
            .env("INIT", d.join("init"))
            .env("COMMIT_WAIT", wait.to_string())
            .env("STOP_WAIT", "2")
            .env("NAME", "pal-swap-test-no-such-process")
            .spawn()
            .unwrap()
    }

    fn state(d: &std::path::Path) -> String {
        std::fs::read_to_string(d.join("up/state"))
            .unwrap_or_default()
            .trim()
            .to_string()
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
}
