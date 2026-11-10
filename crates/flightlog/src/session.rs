//! One flight-log directory per session (spec section 7):
//! `fl-NNNN_<yyyymmdd-HHMM>/` with `stats.jsonl`, `events.jsonl`, `au.log`
//! and an optional `bodies.bin` capture. Every file opens with a format
//! marker line carrying a version.
//!
//! Events are derived from consecutive stats lines (session, rung, MCS,
//! width, channel, key mismatch, drone state, video resets), so both cores
//! produce them without a hook into either; the host adds its own (USB
//! detach, capture cap) through `event`.
//!
//! The first event, `build`, says what wrote the session: version, commit,
//! pins, app, core, mode and the config's sha256. Card chips follow as a
//! `cards` event once the cards are open. A `panic.txt` the panic hook left
//! in the root is reported once, in the next session, as `panic_prev`.

use std::fs::{self, File};
use std::io::{self, BufRead, BufReader, BufWriter, Read, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicPtr, Ordering};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use pal_core::{Au, RxBody};
use serde_json::{json, Map, Value};
use sha2::{Digest, Sha256};

use crate::capture;

pub const STATS_FORMAT: &str = "pal-stats";
pub const EVENTS_FORMAT: &str = "pal-events";
pub const AULOG_MARKER: &str = "aulog 1";
pub const VERSION: u32 = 1;
/// In the log root: what the panic hook appends to. The next session moves
/// it into its own directory as `PANIC_PREV_FILE`.
pub const PANIC_FILE: &str = "panic.txt";
pub const PANIC_PREV_FILE: &str = "panic_prev.txt";
/// Bytes of a previous panic.txt copied into the `panic_prev` event.
const PANIC_PREV_MAX: u64 = 16 << 10;
/// Events a session writes whether or not a drone was ever heard.
pub const BOOKKEEPING: &[&str] = &["build", "cards"];

/// What runs the session, for its `build` event.
pub struct Run<'a> {
    /// "pal", "pal-desktop", "pal-phone", "pal-quest".
    pub app: &'a str,
    /// "rust" or "ref".
    pub core: &'a str,
    /// "gs" or "spotter".
    pub mode: &'a str,
    /// The config text the core was built from.
    pub config: Option<&'a [u8]>,
}

pub fn sha256_hex(b: &[u8]) -> String {
    Sha256::digest(b)
        .iter()
        .map(|x| format!("{x:02x}"))
        .collect()
}

/// The `build` event's fields.
pub fn build_record(run: &Run) -> Value {
    json!({
        "version": pal_build::VERSION,
        "commit": pal_build::GIT_SHA,
        "dirty": pal_build::GIT_DIRTY,
        "mabur": pal_build::MABUR_PIN,
        "devourer": pal_build::DEVOURER_PIN,
        "app": run.app,
        "core": run.core,
        "mode": run.mode,
        "config_sha256": run.config.map(sha256_hex),
    })
}

/// The directory name of the session this process opened last, for the
/// panic hook, which must not take a lock the panicking thread may hold.
static LAST_OPENED: AtomicPtr<String> = AtomicPtr::new(std::ptr::null_mut());

pub fn last_opened() -> Option<&'static str> {
    // SAFETY: null or from Box::into_raw in set_last_opened, never freed.
    unsafe { LAST_OPENED.load(Ordering::Acquire).as_ref() }.map(String::as_str)
}

fn set_last_opened(name: String) {
    // The previous name leaks (tens of bytes a session): a panic hook on
    // another thread may be reading it.
    LAST_OPENED.store(Box::into_raw(Box::new(name)), Ordering::Release);
}

/// Moves a panic.txt from `root` into `dir` and returns its text (the first
/// PANIC_PREV_MAX bytes). Moved, or removed when the move fails, so the
/// next session does not report it again.
fn take_panic(root: &Path, dir: &Path) -> Option<String> {
    let src = root.join(PANIC_FILE);
    let mut text = Vec::new();
    File::open(&src)
        .ok()?
        .take(PANIC_PREV_MAX)
        .read_to_end(&mut text)
        .ok()?;
    if fs::rename(&src, dir.join(PANIC_PREV_FILE)).is_err() {
        let _ = fs::remove_file(&src);
    }
    (!text.is_empty()).then(|| String::from_utf8_lossy(&text).into_owned())
}

/// A line of events.jsonl holding one of the BOOKKEEPING events.
pub fn is_bookkeeping(line: &str) -> bool {
    serde_json::from_str::<Value>(line)
        .ok()
        .and_then(|v| v.get("ev").and_then(Value::as_str).map(str::to_string))
        .is_some_and(|e| BOOKKEEPING.contains(&e.as_str()))
}

pub(crate) const CAPTURE_FILE: &str = "bodies.bin";
pub(crate) const AU_FILE: &str = "au.log";
pub(crate) const EVENTS_FILE: &str = "events.jsonl";
pub(crate) const STATS_FILE: &str = "stats.jsonl";
/// mabur's loss-episode log, same name and text as its GS writes
/// (gs/src/fec_log.cpp), so mabur's flightreport.py reads it.
pub(crate) const FEC_FILE: &str = "fec.log";
const FEC_FORMAT: &str = "feclog 4";
/// Lines `Session::create` writes before any data.
pub(crate) const AU_HEADER_LINES: usize = 2;
pub(crate) const EVENTS_HEADER_LINES: usize = 1;

/// Stats keys whose change is an event, in the order they are reported.
const WATCHED: &[&str] = &[
    "session",
    "key_mismatch",
    "drone_state",
    "rung",
    "mcs",
    "bw",
    "resets",
];

/// UTC civil date from days since 1970-01-01 (Howard Hinnant's algorithm).
fn civil(days: i64) -> (i64, u32, u32) {
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    (yoe + era * 400 + i64::from(m <= 2), m, d)
}

/// `yyyymmdd-HHMM` in UTC.
pub fn stamp(t: SystemTime) -> String {
    let s = t.duration_since(UNIX_EPOCH).map_or(0, |d| d.as_secs()) as i64;
    let (y, m, d) = civil(s.div_euclid(86_400));
    let sod = s.rem_euclid(86_400);
    format!("{y:04}{m:02}{d:02}-{:02}{:02}", sod / 3600, sod % 3600 / 60)
}

/// The number in a session directory's name; `{:04}` grows past four digits.
pub(crate) fn session_index(name: &str) -> Option<u32> {
    let s = name.strip_prefix("fl-")?;
    let end = s.find(|c: char| !c.is_ascii_digit()).unwrap_or(s.len());
    s[..end].parse().ok()
}

/// Next free `fl-NNNN` index under `root`.
fn next_index(root: &Path) -> u32 {
    let Ok(rd) = fs::read_dir(root) else { return 1 };
    rd.filter_map(|e| e.ok())
        .filter_map(|e| session_index(&e.file_name().into_string().ok()?))
        .max()
        .map_or(1, |n| n.saturating_add(1))
}

/// Where a line file's text goes: a buffered file, or what a test puts
/// around it (`Session::wrap_writers`).
pub type LineWriter = Box<dyn Write + Send>;

pub struct Session {
    dir: PathBuf,
    stats: LineWriter,
    events: LineWriter,
    au: LineWriter,
    fec: LineWriter,
    capture: Option<capture::Writer<BufWriter<File>>>,
    capture_cap: u64,
    last: Option<Map<String, Value>>,
    /// `channel` as last logged, and a new value seen once: the stat is
    /// card 0's tuning, so a scout dwell on card 0 shows for one line.
    channel: Option<Value>,
    channel_seen: Option<Value>,
    /// The first write error. From then on nothing is written: a writer
    /// that failed may hold half a record, and retrying it on every call
    /// would cost the core thread a syscall per AU on a full disk.
    failed: Option<io::Error>,
    /// The longest single write call (a line, a capture record, a flush)
    /// since the last stats row, in microseconds; the row carries it as
    /// `log_max_us`. The calls run on the core thread, which also runs FEC,
    /// AU delivery, NACKs and the uplink: a buffer flush into a busy
    /// filesystem stalls all four, and this is what shows how long.
    call_max_us: u64,
    /// The same since the last `take_call_max_us` (the latency report).
    lat_call_max_us: u64,
}

fn write_event(w: &mut impl Write, t_us: u64, ev: &str, detail: Value) -> io::Result<()> {
    let mut m = Map::new();
    m.insert("t_us".into(), json!(t_us));
    m.insert("ev".into(), json!(ev));
    if let Value::Object(d) = detail {
        m.extend(d);
    }
    writeln!(w, "{}", Value::Object(m))
}

fn create(dir: &Path, name: &str) -> io::Result<LineWriter> {
    Ok(Box::new(BufWriter::with_capacity(
        1 << 16,
        File::create(dir.join(name))?,
    )))
}

impl Session {
    /// `capture_cap`: bodies.bin byte cap, 0 = no capture.
    pub fn create(root: &Path, now: SystemTime, capture_cap: u64, run: &Run) -> io::Result<Self> {
        fs::create_dir_all(root)?;
        let dir = root.join(format!("fl-{:04}_{}", next_index(root), stamp(now)));
        fs::create_dir(&dir)?;
        let mut stats = create(&dir, STATS_FILE)?;
        writeln!(stats, "{}", json!({"format": STATS_FORMAT, "v": VERSION}))?;
        let mut events = create(&dir, EVENTS_FILE)?;
        writeln!(events, "{}", json!({"format": EVENTS_FORMAT, "v": VERSION}))?;
        write_event(&mut events, 0, "build", build_record(run))?;
        if let Some(text) = take_panic(root, &dir) {
            write_event(&mut events, 0, "panic_prev", json!({ "text": text }))?;
        }
        let mut au = create(&dir, AU_FILE)?;
        writeln!(au, "{AULOG_MARKER}")?;
        writeln!(au, "# t_us sid bytes flags complete salvaged pts_us")?;
        let mut fec = create(&dir, FEC_FILE)?;
        writeln!(fec, "{FEC_FORMAT}")?;
        // A release build aborts on a panic without dropping the writers:
        // the headers and the build event at least reach the disk.
        for w in [&mut stats, &mut events, &mut au, &mut fec] {
            w.flush()?;
        }
        let capture = if capture_cap > 0 {
            Some(capture::Writer::new(BufWriter::with_capacity(
                1 << 20,
                File::create(dir.join(CAPTURE_FILE))?,
            ))?)
        } else {
            None
        };
        if let Some(n) = dir.file_name().and_then(|n| n.to_str()) {
            set_last_opened(n.to_string());
        }
        Ok(Self {
            dir,
            stats,
            events,
            au,
            fec,
            capture,
            capture_cap,
            last: None,
            channel: None,
            channel_seen: None,
            failed: None,
            call_max_us: 0,
            lat_call_max_us: 0,
        })
    }

    pub fn dir(&self) -> &Path {
        &self.dir
    }

    /// The bodies.bin byte cap the session was created with; 0 = no capture.
    pub fn capture_cap(&self) -> u64 {
        self.capture_cap
    }

    /// The write error that stopped the log, if one did.
    pub fn write_error(&self) -> Option<&io::Error> {
        self.failed.as_ref()
    }

    /// Puts `f`'s writer around each line file's (stats, events, au, fec), e.g.
    /// one that fails after some bytes.
    pub fn wrap_writers(&mut self, mut f: impl FnMut(&'static str, LineWriter) -> LineWriter) {
        for (name, w) in [
            (STATS_FILE, &mut self.stats),
            (EVENTS_FILE, &mut self.events),
            (AU_FILE, &mut self.au),
            (FEC_FILE, &mut self.fec),
        ] {
            let inner = std::mem::replace(w, Box::new(io::sink()));
            *w = f(name, inner);
        }
    }

    /// The longest write call since the previous take, in microseconds.
    pub fn take_call_max_us(&mut self) -> u64 {
        std::mem::take(&mut self.lat_call_max_us)
    }

    /// Runs one write unless the log has stopped; latches its error and
    /// times the call.
    fn guard(&mut self, w: impl FnOnce(&mut Self) -> io::Result<()>) -> io::Result<()> {
        if let Some(e) = &self.failed {
            return Err(e.kind().into());
        }
        let t0 = Instant::now();
        let r =
            w(self).inspect_err(|e| self.failed = Some(io::Error::new(e.kind(), e.to_string())));
        let us = t0.elapsed().as_micros() as u64;
        self.call_max_us = self.call_max_us.max(us);
        self.lat_call_max_us = self.lat_call_max_us.max(us);
        r
    }

    /// One `stats_json` line, stamped; changes of the watched keys since
    /// the previous line become events.
    pub fn stats(&mut self, t_us: u64, line: &str) -> io::Result<()> {
        self.guard(|s| s.write_stats(t_us, line))
    }

    fn write_stats(&mut self, t_us: u64, line: &str) -> io::Result<()> {
        let Ok(Value::Object(mut m)) = serde_json::from_str::<Value>(line) else {
            return self.write_event(t_us, "bad_stats", json!({ "line": line }));
        };
        let ch = m.get("channel").cloned();
        if self.last.is_none() {
            self.channel = ch.clone();
        } else if ch == self.channel {
            self.channel_seen = None;
        } else if ch == self.channel_seen {
            // Two lines in a row: a hop or a pick, not a dwell.
            let from = std::mem::replace(&mut self.channel, ch.clone());
            self.channel_seen = None;
            self.write_event(t_us, "channel", json!({"from": from, "to": ch}))?;
        } else {
            self.channel_seen = ch;
        }
        if let Some(prev) = self.last.take() {
            for k in WATCHED {
                let (a, b) = (prev.get(*k), m.get(*k));
                if a != b {
                    self.write_event(t_us, k, json!({"from": a, "to": b}))?;
                }
            }
        }
        m.insert("t_us".into(), json!(t_us));
        // This row's own write counts towards the next one.
        m.insert(
            "log_max_us".into(),
            json!(std::mem::take(&mut self.call_max_us)),
        );
        writeln!(self.stats, "{}", serde_json::to_string(&m)?)?;
        self.last = Some(m); // t_us is not watched
        Ok(())
    }

    pub fn event(&mut self, t_us: u64, ev: &str, detail: Value) -> io::Result<()> {
        self.guard(|s| s.write_event(t_us, ev, detail))
    }

    fn write_event(&mut self, t_us: u64, ev: &str, detail: Value) -> io::Result<()> {
        write_event(&mut self.events, t_us, ev, detail)
    }

    /// IDR request path: `idr_req` per epoch bump, `idr_heal` per clearing IRAP.
    pub fn refresh_events(&mut self, evs: &[pal_core::refresh::Event]) -> io::Result<()> {
        if evs.is_empty() {
            return Ok(());
        }
        self.guard(|s| s.write_refresh_events(evs))
    }

    /// Loss episodes from `GsCore::take_fec_rows`, already in fec.log text.
    pub fn fec_rows(&mut self, rows: &[String]) -> io::Result<()> {
        if rows.is_empty() {
            return Ok(());
        }
        self.guard(|s| {
            for r in rows {
                writeln!(s.fec, "{r}")?;
            }
            Ok(())
        })
    }

    fn write_refresh_events(&mut self, evs: &[pal_core::refresh::Event]) -> io::Result<()> {
        for e in evs {
            // fec.log's `I` / `H` lines (FecLog::request / heal) beside the
            // episodes they answer.
            if e.kind == 'H' {
                writeln!(self.fec, "H {} {} {}", e.t_ms, e.heal_ms, e.retries)?;
            } else {
                writeln!(self.fec, "I {} {} {}", e.t_ms, e.reason.as_str(), e.epoch)?;
            }
            let (ev, d) = if e.kind == 'H' {
                (
                    "idr_heal",
                    json!({"reason": e.reason.as_str(), "epoch": e.epoch, "heal_ms": e.heal_ms, "retries": e.retries}),
                )
            } else {
                (
                    "idr_req",
                    json!({"reason": e.reason.as_str(), "epoch": e.epoch, "retry": e.retries}),
                )
            };
            self.write_event(e.t_ms * 1000, ev, d)?;
        }
        Ok(())
    }

    pub fn au(&mut self, t_us: u64, a: &Au) -> io::Result<()> {
        self.guard(|s| {
            writeln!(
                s.au,
                "{t_us} {} {} {} {} {} {}",
                a.sid,
                a.data.len(),
                a.flags,
                u8::from(a.complete),
                u8::from(a.salvaged),
                a.pts_us
            )
        })
    }

    /// Capture one body; stops (with one event) once the cap is reached.
    pub fn body(&mut self, b: &RxBody) -> io::Result<()> {
        if self.capture.is_none() {
            return Ok(());
        }
        self.guard(|s| {
            if let Some(c) = s.capture.as_mut() {
                c.body(b)?;
            }
            s.close_capture_at_cap(b.mono_us)
        })
    }

    /// Capture one tick; ticks count towards the cap like bodies do.
    pub fn tick(&mut self, now_us: u64) -> io::Result<()> {
        if self.capture.is_none() {
            return Ok(());
        }
        self.guard(|s| {
            if let Some(c) = s.capture.as_mut() {
                c.tick(now_us)?;
            }
            s.close_capture_at_cap(now_us)
        })
    }

    fn close_capture_at_cap(&mut self, t_us: u64) -> io::Result<()> {
        if self
            .capture
            .as_ref()
            .is_some_and(|c| c.written() >= self.capture_cap)
        {
            self.close_capture(t_us)?;
        }
        Ok(())
    }

    fn close_capture(&mut self, t_us: u64) -> io::Result<()> {
        if let Some(mut c) = self.capture.take() {
            c.flush()?;
            let bytes = c.written();
            self.write_event(t_us, "capture_cap", json!({ "bytes": bytes }))?;
        }
        Ok(())
    }

    pub fn flush(&mut self) -> io::Result<()> {
        self.guard(|s| {
            s.stats.flush()?;
            s.events.flush()?;
            s.au.flush()?;
            s.fec.flush()?;
            if let Some(c) = s.capture.as_mut() {
                c.flush()?;
            }
            Ok(())
        })
    }
}

impl Drop for Session {
    fn drop(&mut self) {
        // A stopped log is not flushed again: its error is already latched.
        if self.failed.is_none() {
            let _ = self.flush();
        }
    }
}

/// What `pal log` prints for one session directory.
#[derive(Debug, Default)]
pub struct Summary {
    pub span_s: f64,
    pub stats_lines: u64,
    pub aus: u64,
    pub au_bytes: u64,
    pub truncated: u64,
    pub truncated_base: u64,
    pub salvaged: u64,
    pub idr: u64,
    pub events: Vec<(String, u64)>,
    pub last_stats: Option<Value>,
    pub session_edges: Vec<(u64, Value)>,
    pub has_capture: bool,
    /// The `build` event; None in a session written before it existed.
    pub build: Option<Value>,
    /// Each `cards` event's chips, in order.
    pub cards: Vec<String>,
    /// A `panic_prev` event's text: how the previous run ended.
    pub panic_prev: Option<String>,
    /// The longest flight-log write call over the stats rows and the row
    /// time it was reported at (µs); None when no row has `log_max_us`.
    pub log_max_us: Option<(u64, u64)>,
    /// The last `params` event: its time (µs) and the drone's {id: value}.
    pub params: Option<(u64, Map<String, Value>)>,
    /// Channel core hop steps: (t_us, kind, epoch, target, ms since the
    /// order) in log order.
    pub hops: Vec<(u64, String, u64, u64, f64)>,
    /// fec.log's episodes per layer; None for a session written before it.
    pub fec: Option<[FecTotals; 2]>,
}

/// One layer's loss episodes in fec.log.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct FecTotals {
    pub episodes: u64,
    /// Episodes by missing source symbols: 1, 2, 3-5, 6-10, 11+.
    pub by_missing: [u64; 5],
    pub missing: u64,
    pub recovered: u64,
    pub retx: u64,
    pub abandoned: u64,
}

impl FecTotals {
    pub const BUCKETS: [&'static str; 5] = ["1", "2", "3-5", "6-10", "11+"];

    fn add(&mut self, missing: u64, recovered: u64, retx: u64, abandoned: u64) {
        self.episodes += 1;
        let b = match missing {
            0 | 1 => 0,
            2 => 1,
            3..=5 => 2,
            6..=10 => 3,
            _ => 4,
        };
        self.by_missing[b] += 1;
        self.missing += missing;
        self.recovered += recovered;
        self.retx += retx;
        self.abandoned += abandoned;
    }
}

/// Episode rows of a fec.log (`I` / `H` lines and unknown text skipped).
fn read_fec(path: &Path) -> io::Result<Option<[FecTotals; 2]>> {
    let f = match File::open(path) {
        Ok(f) => f,
        Err(e) if e.kind() == io::ErrorKind::NotFound => return Ok(None),
        Err(e) => return Err(e),
    };
    let mut t = [FecTotals::default(); 2];
    for l in BufReader::new(f).lines() {
        let l = l?;
        let f: Vec<&str> = l.split(' ').collect();
        if f.len() != 14 {
            continue;
        }
        let n = |i: usize| f[i].parse::<u64>().ok();
        let (Some(sid), Some(m), Some(rec), Some(rtx), Some(ab)) = (n(1), n(7), n(8), n(9), n(10))
        else {
            continue;
        };
        if let Some(x) = t.get_mut(sid as usize) {
            x.add(m, rec, rtx, ab);
        }
    }
    Ok(Some(t))
}

fn check_marker(first: Option<io::Result<String>>, want: &str, file: &str) -> io::Result<()> {
    let line = first.transpose()?.unwrap_or_default();
    let ok = serde_json::from_str::<Value>(&line)
        .ok()
        .and_then(|v| v.get("format").and_then(Value::as_str).map(|f| f == want))
        .unwrap_or(false);
    if ok {
        Ok(())
    } else {
        Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("{file}: no {want} marker"),
        ))
    }
}

pub fn summarize(dir: &Path) -> io::Result<Summary> {
    let mut s = Summary {
        has_capture: dir.join(CAPTURE_FILE).exists(),
        ..Summary::default()
    };
    let (mut t_min, mut t_max) = (u64::MAX, 0u64);

    let mut lines = BufReader::new(File::open(dir.join(AU_FILE))?).lines();
    if lines.next().transpose()?.as_deref() != Some(AULOG_MARKER) {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "au.log: no aulog marker",
        ));
    }
    for l in lines {
        let l = l?;
        if l.starts_with('#') {
            continue;
        }
        let f: Vec<u64> = l.split(' ').filter_map(|x| x.parse().ok()).collect();
        let [t, sid, bytes, flags, complete, salvaged, _pts] = f[..] else {
            continue;
        };
        t_min = t_min.min(t);
        t_max = t_max.max(t);
        s.aus += 1;
        s.au_bytes += bytes;
        s.idr += u64::from(flags & 0x01 != 0);
        if complete == 0 {
            s.truncated += 1;
            s.truncated_base += u64::from(sid == 0);
        }
        s.salvaged += salvaged;
    }

    let mut lines = BufReader::new(File::open(dir.join(STATS_FILE))?).lines();
    check_marker(lines.next(), STATS_FORMAT, STATS_FILE)?;
    for l in lines {
        if let Ok(v) = serde_json::from_str::<Value>(&l?) {
            if let Some(t) = v.get("t_us").and_then(Value::as_u64) {
                t_min = t_min.min(t);
                t_max = t_max.max(t);
                if let Some(us) = v.get("log_max_us").and_then(Value::as_u64) {
                    if s.log_max_us.is_none_or(|(m, _)| us > m) {
                        s.log_max_us = Some((us, t));
                    }
                }
            }
            s.stats_lines += 1;
            s.last_stats = Some(v);
        }
    }

    let mut counts: Vec<(String, u64)> = Vec::new();
    let mut lines = BufReader::new(File::open(dir.join(EVENTS_FILE))?).lines();
    check_marker(lines.next(), EVENTS_FORMAT, EVENTS_FILE)?;
    for l in lines {
        let Ok(v) = serde_json::from_str::<Value>(&l?) else {
            continue;
        };
        let ev = v
            .get("ev")
            .and_then(Value::as_str)
            .unwrap_or("?")
            .to_string();
        match ev.as_str() {
            "build" if s.build.is_none() => s.build = Some(v.clone()),
            "cards" => s
                .cards
                .push(v.get("chips").and_then(Value::as_str).unwrap_or("?").into()),
            "panic_prev" => s.panic_prev = v.get("text").and_then(Value::as_str).map(Into::into),
            "hop" => {
                let u = |k: &str| v.get(k).and_then(Value::as_u64).unwrap_or(0);
                let kind = v.get("kind").and_then(Value::as_str).unwrap_or("?");
                let ms = v.get("ms").and_then(Value::as_f64).unwrap_or(0.0);
                s.hops
                    .push((u("t_us"), kind.into(), u("epoch"), u("target"), ms));
            }
            "params" => {
                if let Some(m) = v.get("values").and_then(Value::as_object) {
                    let t = v.get("t_us").and_then(Value::as_u64).unwrap_or(0);
                    s.params = Some((t, m.clone()));
                }
            }
            _ => {}
        }
        if ev == "session" {
            let t = v.get("t_us").and_then(Value::as_u64).unwrap_or(0);
            s.session_edges
                .push((t, v.get("to").cloned().unwrap_or(Value::Null)));
        }
        if BOOKKEEPING.contains(&ev.as_str()) {
            continue;
        }
        match counts.iter_mut().find(|(k, _)| *k == ev) {
            Some((_, n)) => *n += 1,
            None => counts.push((ev, 1)),
        }
    }
    s.events = counts;
    s.fec = read_fec(&dir.join(FEC_FILE))?;
    if t_max >= t_min && t_min != u64::MAX {
        s.span_s = (t_max - t_min) as f64 / 1e6;
    }
    Ok(s)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Arc;

    const TEST_RUN: Run = Run {
        app: "test",
        core: "rust",
        mode: "gs",
        config: Some(b"abc"),
    };

    #[test]
    fn the_first_event_names_the_build() {
        let root = std::env::temp_dir().join(format!("pal-fl-build-{}", std::process::id()));
        let _ = fs::remove_dir_all(&root);
        let dir = Session::create(&root, UNIX_EPOCH, 0, &TEST_RUN)
            .expect("create")
            .dir()
            .to_path_buf();
        let ev = fs::read_to_string(dir.join("events.jsonl")).expect("events");
        let first: Value = serde_json::from_str(ev.lines().nth(1).expect("line 2")).expect("json");
        assert_eq!(first["ev"], "build");
        assert_eq!(first["version"], pal_build::VERSION);
        assert_eq!(first["core"], "rust");
        // FIPS 180-2 example: sha256("abc").
        assert_eq!(
            first["config_sha256"],
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
        let sum = summarize(&dir).expect("summary");
        assert_eq!(sum.build.as_ref().map(|b| &b["app"]), Some(&json!("test")));
        assert!(sum.events.is_empty(), "{:?}", sum.events);
        // A session from before the build event still reads.
        fs::write(
            dir.join("events.jsonl"),
            "{\"format\":\"pal-events\",\"v\":1}\n",
        )
        .expect("w");
        assert!(summarize(&dir).expect("old summary").build.is_none());
        let _ = fs::remove_dir_all(&root);
    }

    #[test]
    fn ticks_alone_reach_the_cap_and_the_event_has_the_file_size() {
        let root = std::env::temp_dir().join(format!("pal-fl-cap-{}", std::process::id()));
        let _ = fs::remove_dir_all(&root);
        let mut s = Session::create(&root, SystemTime::now(), 100, &TEST_RUN).unwrap();
        for t in 0..1000 {
            s.tick(t).unwrap();
        }
        s.flush().unwrap();
        let dir = fs::read_dir(&root).unwrap().next().unwrap().unwrap().path();
        // The magic is 9 bytes and a tick 9: 11 ticks are the first past 100.
        let len = fs::metadata(dir.join(CAPTURE_FILE)).unwrap().len();
        assert_eq!(len, 108);
        let events = fs::read_to_string(dir.join(EVENTS_FILE)).unwrap();
        let caps: Vec<&str> = events
            .lines()
            .filter(|l| l.contains("capture_cap"))
            .collect();
        assert_eq!(caps.len(), 1);
        assert!(caps[0].contains("\"bytes\":108"), "{}", caps[0]);
        fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn the_next_index_grows_past_four_digits() {
        let root = std::env::temp_dir().join(format!("pal-fl-index-{}", std::process::id()));
        fs::create_dir_all(root.join("fl-9999_20261102-1000")).unwrap();
        fs::create_dir_all(root.join("fl-10000_20261102-1001")).unwrap();
        assert_eq!(next_index(&root), 10001);
        fs::remove_dir_all(&root).unwrap();
        assert_eq!(session_index("fl-1747-keep"), Some(1747));
        assert_eq!(session_index("notes"), None);
    }

    #[test]
    fn a_panic_file_is_reported_once_by_the_next_session() {
        let root = std::env::temp_dir().join(format!("pal-fl-panic-{}", std::process::id()));
        let _ = fs::remove_dir_all(&root);
        fs::create_dir_all(&root).expect("root");
        fs::write(root.join(PANIC_FILE), "panicked at src/x.rs:7:5:\nboom\n").expect("w");
        let first = Session::create(&root, UNIX_EPOCH, 0, &TEST_RUN)
            .expect("create")
            .dir()
            .to_path_buf();
        // Tests in parallel open sessions too: only that some name is set.
        assert!(last_opened().is_some_and(|n| n.starts_with("fl-")));
        let sum = summarize(&first).expect("summary");
        assert!(sum
            .panic_prev
            .as_deref()
            .is_some_and(|t| t.contains("src/x.rs:7")));
        assert!(!root.join(PANIC_FILE).exists() && first.join(PANIC_PREV_FILE).exists());
        let second = Session::create(&root, UNIX_EPOCH, 0, &TEST_RUN)
            .expect("create")
            .dir()
            .to_path_buf();
        assert!(summarize(&second).expect("summary").panic_prev.is_none());
        let _ = fs::remove_dir_all(&root);
    }

    #[test]
    fn civil_dates() {
        assert_eq!(civil(0), (1970, 1, 1));
        assert_eq!(civil(20_393), (2025, 11, 1));
        assert_eq!(
            stamp(UNIX_EPOCH + std::time::Duration::from_secs(1_761_955_260)),
            "20251101-0001"
        );
    }

    #[test]
    fn session_roundtrip() {
        let root = std::env::temp_dir().join(format!("pal-fl-test-{}", std::process::id()));
        let _ = fs::remove_dir_all(&root);
        let dir = {
            let mut s = Session::create(&root, UNIX_EPOCH, 0, &TEST_RUN).expect("create");
            s.stats(1_000_000, r#"{"session":false,"rung":-1,"resets":0}"#)
                .expect("stats");
            s.stats(2_000_000, r#"{"session":true,"rung":0,"resets":1}"#)
                .expect("stats");
            let au = Au {
                data: vec![0; 100],
                flags: 1,
                complete: true,
                ..Au::default()
            };
            s.au(1_500_000, &au).expect("au");
            s.au(
                1_600_000,
                &Au {
                    complete: false,
                    ..au
                },
            )
            .expect("au");
            s.dir().to_path_buf()
        };
        assert!(dir
            .file_name()
            .and_then(|n| n.to_str())
            .is_some_and(|n| n.starts_with("fl-0001_")));
        let sum = summarize(&dir).expect("summary");
        assert_eq!(
            (sum.aus, sum.truncated, sum.idr, sum.stats_lines),
            (2, 1, 2, 2)
        );
        assert_eq!(sum.session_edges.len(), 1);
        assert!(sum.events.iter().any(|(k, n)| k == "rung" && *n == 1));
        assert!(!sum.events.iter().any(|(k, _)| k == "channel"));
        assert!((sum.span_s - 1.0).abs() < 1e-9);
        let second = Session::create(&root, UNIX_EPOCH, 0, &TEST_RUN).expect("create");
        assert!(second.dir().to_string_lossy().contains("fl-0002_"));
        drop(second);
        let _ = fs::remove_dir_all(&root);
    }

    /// Writes into a file that blocks (flash GC, a full device) stall the
    /// core thread; the stats row after such a call says how long.
    #[test]
    fn loss_episodes_and_refresh_rows_go_to_fec_log_in_mabur_text() {
        use pal_core::refresh::{Event, Reason};
        let root = std::env::temp_dir().join(format!("pal-fl-fec-{}", std::process::id()));
        let _ = fs::remove_dir_all(&root);
        let mut s = Session::create(&root, UNIX_EPOCH, 0, &TEST_RUN).expect("create");
        s.fec_rows(&[
            "1000 0 2 20 0.50 100 1 1 1 0 0 0 2 32".into(),
            "1200 0 2 20 0.50 200 12 7 5 1 1 0 3 32".into(),
            "1300 1 2 20 0.25 300 4 3 0 0 3 0 1 32".into(),
        ])
        .expect("rows");
        let ev = |kind, t_ms| Event {
            kind,
            reason: Reason::Lost,
            epoch: 4,
            t_ms,
            heal_ms: 350,
            retries: 1,
        };
        s.refresh_events(&[ev('I', 1250), ev('H', 1600)])
            .expect("refresh");
        let dir = s.dir().to_path_buf();
        drop(s);
        let text = fs::read_to_string(dir.join(FEC_FILE)).expect("fec.log");
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines[0], "feclog 4");
        assert_eq!(lines[4], "I 1250 lost 4");
        assert_eq!(lines[5], "H 1600 350 1");
        let fec = summarize(&dir).expect("summary").fec.expect("fec totals");
        assert_eq!(fec[0].episodes, 2);
        assert_eq!(fec[0].by_missing, [1, 0, 0, 1, 0]);
        assert_eq!(
            (
                fec[0].missing,
                fec[0].recovered,
                fec[0].retx,
                fec[0].abandoned
            ),
            (8, 6, 1, 1)
        );
        assert_eq!(fec[1].by_missing, [0, 0, 1, 0, 0]);
        assert_eq!(fec[1].abandoned, 3);
        let _ = fs::remove_dir_all(&root);
    }

    #[test]
    fn a_slow_write_shows_in_the_next_stats_row_and_the_summary() {
        struct Slow(LineWriter, Arc<AtomicBool>);
        impl Write for Slow {
            fn write(&mut self, b: &[u8]) -> io::Result<usize> {
                if self.1.load(Ordering::Relaxed) {
                    std::thread::sleep(std::time::Duration::from_millis(30));
                }
                self.0.write(b)
            }
            fn flush(&mut self) -> io::Result<()> {
                self.0.flush()
            }
        }
        let root = std::env::temp_dir().join(format!("pal-fl-slow-{}", std::process::id()));
        let _ = fs::remove_dir_all(&root);
        let slow = Arc::new(AtomicBool::new(false));
        let mut s = Session::create(&root, UNIX_EPOCH, 0, &TEST_RUN).expect("create");
        s.wrap_writers(|name, w| {
            if name == AU_FILE {
                Box::new(Slow(w, slow.clone()))
            } else {
                w
            }
        });
        s.stats(1_000_000, "{}").expect("stats");
        slow.store(true, Ordering::Relaxed);
        s.au(1_500_000, &Au::default()).expect("au");
        slow.store(false, Ordering::Relaxed);
        s.stats(2_000_000, "{}").expect("stats");
        s.stats(3_000_000, "{}").expect("stats");
        assert!(s.take_call_max_us() >= 30_000);
        assert!(s.take_call_max_us() < 30_000, "taken");
        let dir = s.dir().to_path_buf();
        drop(s);
        let rows: Vec<Value> = fs::read_to_string(dir.join(STATS_FILE))
            .expect("stats")
            .lines()
            .skip(1)
            .map(|l| serde_json::from_str(l).expect("json"))
            .collect();
        let max = |i: usize| rows[i]["log_max_us"].as_u64().expect("log_max_us");
        assert!(max(1) >= 30_000, "{:?}", rows[1]);
        assert!(max(2) < 30_000, "reset after the row: {:?}", rows[2]);
        let sum = summarize(&dir).expect("summary");
        assert_eq!(sum.log_max_us.map(|(_, t)| t), Some(2_000_000));
        let _ = fs::remove_dir_all(&root);
    }

    #[test]
    fn a_one_line_channel_blip_is_not_a_hop() {
        let root = std::env::temp_dir().join(format!("pal-fl-chan-{}", std::process::id()));
        let _ = fs::remove_dir_all(&root);
        let dir = {
            let mut s = Session::create(&root, UNIX_EPOCH, 0, &TEST_RUN).expect("create");
            // 36, a scout dwell on 149 for one line, then a real hop to 149.
            for (i, ch) in [36, 36, 149, 36, 36, 149, 149, 149].iter().enumerate() {
                s.stats(i as u64 * 1_000_000, &format!(r#"{{"channel":{ch}}}"#))
                    .expect("stats");
            }
            s.dir().to_path_buf()
        };
        let ev = fs::read_to_string(dir.join(EVENTS_FILE)).expect("events");
        let hops: Vec<&str> = ev.lines().filter(|l| l.contains(r#""channel""#)).collect();
        assert_eq!(hops.len(), 1, "{ev}");
        assert!(hops[0].contains(r#""from":36"#) && hops[0].contains(r#""to":149"#));
        assert!(hops[0].contains(r#""t_us":6000000"#), "{}", hops[0]);
        let _ = fs::remove_dir_all(&root);
    }
}
