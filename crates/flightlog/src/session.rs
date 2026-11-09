//! One flight-log directory per session (spec section 7):
//! `fl-NNNN_<yyyymmdd-HHMM>/` with `stats.jsonl`, `events.jsonl`, `au.log`
//! and an optional `bodies.bin` capture. Every file opens with a format
//! marker line carrying a version.
//!
//! Events are derived from consecutive stats lines (session, rung, MCS,
//! width, channel, key mismatch, drone state, video resets), so both cores
//! produce them without a hook into either; the host adds its own (USB
//! detach, capture cap) through `event`.

use std::fs::{self, File};
use std::io::{self, BufRead, BufReader, BufWriter, Write};
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use pal_core::{Au, RxBody};
use serde_json::{json, Map, Value};

use crate::capture;

pub const STATS_FORMAT: &str = "pal-stats";
pub const EVENTS_FORMAT: &str = "pal-events";
pub const AULOG_MARKER: &str = "aulog 1";
pub const VERSION: u32 = 1;

pub(crate) const CAPTURE_FILE: &str = "bodies.bin";
pub(crate) const AU_FILE: &str = "au.log";
pub(crate) const EVENTS_FILE: &str = "events.jsonl";
pub(crate) const STATS_FILE: &str = "stats.jsonl";
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

pub struct Session {
    dir: PathBuf,
    stats: BufWriter<File>,
    events: BufWriter<File>,
    au: BufWriter<File>,
    capture: Option<capture::Writer<BufWriter<File>>>,
    capture_cap: u64,
    last: Option<Map<String, Value>>,
    /// `channel` as last logged, and a new value seen once: the stat is
    /// card 0's tuning, so a scout dwell on card 0 shows for one line.
    channel: Option<Value>,
    channel_seen: Option<Value>,
}

fn create(dir: &Path, name: &str) -> io::Result<BufWriter<File>> {
    Ok(BufWriter::with_capacity(
        1 << 16,
        File::create(dir.join(name))?,
    ))
}

impl Session {
    /// `capture_cap`: bodies.bin byte cap, 0 = no capture.
    pub fn create(root: &Path, now: SystemTime, capture_cap: u64) -> io::Result<Self> {
        fs::create_dir_all(root)?;
        let dir = root.join(format!("fl-{:04}_{}", next_index(root), stamp(now)));
        fs::create_dir(&dir)?;
        let mut stats = create(&dir, STATS_FILE)?;
        writeln!(stats, "{}", json!({"format": STATS_FORMAT, "v": VERSION}))?;
        let mut events = create(&dir, EVENTS_FILE)?;
        writeln!(events, "{}", json!({"format": EVENTS_FORMAT, "v": VERSION}))?;
        let mut au = create(&dir, AU_FILE)?;
        writeln!(au, "{AULOG_MARKER}")?;
        writeln!(au, "# t_us sid bytes flags complete salvaged pts_us")?;
        let capture = if capture_cap > 0 {
            Some(capture::Writer::new(BufWriter::with_capacity(
                1 << 20,
                File::create(dir.join(CAPTURE_FILE))?,
            ))?)
        } else {
            None
        };
        Ok(Self {
            dir,
            stats,
            events,
            au,
            capture,
            capture_cap,
            last: None,
            channel: None,
            channel_seen: None,
        })
    }

    pub fn dir(&self) -> &Path {
        &self.dir
    }

    /// One `stats_json` line, stamped; changes of the watched keys since
    /// the previous line become events.
    pub fn stats(&mut self, t_us: u64, line: &str) -> io::Result<()> {
        let Ok(Value::Object(mut m)) = serde_json::from_str::<Value>(line) else {
            return self.event(t_us, "bad_stats", json!({ "line": line }));
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
            self.event(t_us, "channel", json!({"from": from, "to": ch}))?;
        } else {
            self.channel_seen = ch;
        }
        if let Some(prev) = self.last.take() {
            for k in WATCHED {
                let (a, b) = (prev.get(*k), m.get(*k));
                if a != b {
                    self.event(t_us, k, json!({"from": a, "to": b}))?;
                }
            }
        }
        m.insert("t_us".into(), json!(t_us));
        writeln!(self.stats, "{}", serde_json::to_string(&m)?)?;
        self.last = Some(m); // t_us is not watched
        Ok(())
    }

    pub fn event(&mut self, t_us: u64, ev: &str, detail: Value) -> io::Result<()> {
        let mut m = Map::new();
        m.insert("t_us".into(), json!(t_us));
        m.insert("ev".into(), json!(ev));
        if let Value::Object(d) = detail {
            m.extend(d);
        }
        writeln!(self.events, "{}", Value::Object(m))
    }

    /// IDR request path: `idr_req` per epoch bump, `idr_heal` per clearing IRAP.
    pub fn refresh_events(&mut self, evs: &[pal_core::refresh::Event]) -> io::Result<()> {
        for e in evs {
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
            self.event(e.t_ms * 1000, ev, d)?;
        }
        Ok(())
    }

    pub fn au(&mut self, t_us: u64, a: &Au) -> io::Result<()> {
        writeln!(
            self.au,
            "{t_us} {} {} {} {} {} {}",
            a.sid,
            a.data.len(),
            a.flags,
            u8::from(a.complete),
            u8::from(a.salvaged),
            a.pts_us
        )
    }

    /// Capture one body; stops (with one event) once the cap is reached.
    pub fn body(&mut self, b: &RxBody) -> io::Result<()> {
        let Some(c) = self.capture.as_mut() else {
            return Ok(());
        };
        c.body(b)?;
        self.close_capture_at_cap(b.mono_us)
    }

    /// Capture one tick; ticks count towards the cap like bodies do.
    pub fn tick(&mut self, now_us: u64) -> io::Result<()> {
        let Some(c) = self.capture.as_mut() else {
            return Ok(());
        };
        c.tick(now_us)?;
        self.close_capture_at_cap(now_us)
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
            self.event(t_us, "capture_cap", json!({ "bytes": bytes }))?;
        }
        Ok(())
    }

    pub fn flush(&mut self) -> io::Result<()> {
        self.stats.flush()?;
        self.events.flush()?;
        self.au.flush()?;
        if let Some(c) = self.capture.as_mut() {
            c.flush()?;
        }
        Ok(())
    }
}

impl Drop for Session {
    fn drop(&mut self) {
        let _ = self.flush();
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
        if ev == "session" {
            let t = v.get("t_us").and_then(Value::as_u64).unwrap_or(0);
            s.session_edges
                .push((t, v.get("to").cloned().unwrap_or(Value::Null)));
        }
        match counts.iter_mut().find(|(k, _)| *k == ev) {
            Some((_, n)) => *n += 1,
            None => counts.push((ev, 1)),
        }
    }
    s.events = counts;
    if t_max >= t_min && t_min != u64::MAX {
        s.span_s = (t_max - t_min) as f64 / 1e6;
    }
    Ok(s)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ticks_alone_reach_the_cap_and_the_event_has_the_file_size() {
        let root = std::env::temp_dir().join(format!("pal-fl-cap-{}", std::process::id()));
        let _ = fs::remove_dir_all(&root);
        let mut s = Session::create(&root, SystemTime::now(), 100).unwrap();
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
            let mut s = Session::create(&root, UNIX_EPOCH, 0).expect("create");
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
        let second = Session::create(&root, UNIX_EPOCH, 0).expect("create");
        assert!(second.dir().to_string_lossy().contains("fl-0002_"));
        drop(second);
        let _ = fs::remove_dir_all(&root);
    }

    #[test]
    fn a_one_line_channel_blip_is_not_a_hop() {
        let root = std::env::temp_dir().join(format!("pal-fl-chan-{}", std::process::id()));
        let _ = fs::remove_dir_all(&root);
        let dir = {
            let mut s = Session::create(&root, UNIX_EPOCH, 0).expect("create");
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
