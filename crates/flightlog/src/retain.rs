//! What a log root keeps. Captures are the bulk (up to 256 MB a session on
//! Android, 7.9 GB on the Quest after two days); a session's text files are
//! ~30 KB. So captures go first, oldest first, until they fit the budget;
//! whole sessions go only past a count cap; sessions that never saw a frame
//! (a link restarting with no drone: 1665 of them on 2026-11-02) go at once,
//! except the newest, which may be the one worth reading.
//!
//! A `keep` file in a session directory protects it from all of this.
//! Run before a new session opens: every session under the root is closed.

use std::fs;
use std::io::{self, BufRead};
use std::path::{Path, PathBuf};

use crate::session::{
    AU_FILE, AU_HEADER_LINES, CAPTURE_FILE, EVENTS_FILE, EVENTS_HEADER_LINES, STATS_FILE,
};

pub const KEEP_FILE: &str = "keep";

#[derive(Clone, Copy, Debug)]
pub struct Budget {
    /// Total bodies.bin bytes kept.
    pub capture_bytes: u64,
    /// Session directories kept.
    pub sessions: usize,
}

#[derive(Debug, Default, PartialEq, Eq)]
pub struct Pruned {
    pub empty_sessions: usize,
    pub captures: usize,
    pub old_sessions: usize,
    pub bytes: u64,
}

struct Dir {
    index: u32,
    path: PathBuf,
    keep: bool,
    capture: u64,
}

fn sessions(root: &Path) -> io::Result<Vec<Dir>> {
    let mut v: Vec<Dir> = fs::read_dir(root)?
        .filter_map(|e| e.ok())
        .filter_map(|e| {
            let name = e.file_name().into_string().ok()?;
            let index = crate::session::session_index(&name)?;
            let path = e.path();
            path.is_dir().then(|| Dir {
                index,
                keep: path.join(KEEP_FILE).exists(),
                capture: fs::metadata(path.join(CAPTURE_FILE)).map_or(0, |m| m.len()),
                path,
            })
        })
        .collect();
    v.sort_by_key(|d| d.index);
    Ok(v)
}

/// No AU in au.log and no event past the header: the link never got going.
/// A few seconds of stats and nothing else: no AU, no event. Length counts:
/// a 40 min session with every frame CRC-failed has neither AUs nor events
/// (fl-1747 on the Quest, 2026-11-03) and is worth reading. Reads at most a
/// few lines: a flight's logs run to megabytes.
fn is_empty(dir: &Path) -> bool {
    let lines = |f: &str, max: usize| {
        fs::File::open(dir.join(f)).map(|h| io::BufReader::new(h).lines().take(max + 1).count())
    };
    // Each file's header: au.log marker + column comment, one format line in
    // the others; stats come at 1 Hz.
    matches!(
        (lines(AU_FILE, AU_HEADER_LINES), lines(EVENTS_FILE, EVENTS_HEADER_LINES), lines(STATS_FILE, 6)),
        (Ok(a), Ok(e), Ok(s)) if a <= AU_HEADER_LINES && e <= EVENTS_HEADER_LINES && s <= 6
    )
}

fn dir_bytes(dir: &Path) -> u64 {
    fs::read_dir(dir).map_or(0, |rd| {
        rd.filter_map(|e| e.ok())
            .filter_map(|e| e.metadata().ok())
            .map(|m| m.len())
            .sum()
    })
}

pub fn prune(root: &Path, b: Budget) -> io::Result<Pruned> {
    let mut out = Pruned::default();
    let mut all = match sessions(root) {
        Ok(v) => v,
        Err(e) if e.kind() == io::ErrorKind::NotFound => return Ok(out),
        Err(e) => return Err(e),
    };
    let newest = all.last().map(|d| d.index);
    all.retain(|d| {
        if d.keep || Some(d.index) == newest || !is_empty(&d.path) {
            return true;
        }
        let n = dir_bytes(&d.path);
        if fs::remove_dir_all(&d.path).is_ok() {
            out.empty_sessions += 1;
            out.bytes += n;
            return false;
        }
        true
    });
    let mut total: u64 = all.iter().map(|d| d.capture).sum();
    for d in all.iter_mut().filter(|d| !d.keep && d.capture > 0) {
        if total <= b.capture_bytes {
            break;
        }
        if fs::remove_file(d.path.join(CAPTURE_FILE)).is_ok() {
            total -= d.capture;
            out.captures += 1;
            out.bytes += d.capture;
            d.capture = 0;
        }
    }
    let mut excess = all.len().saturating_sub(b.sessions);
    for d in all.iter().filter(|d| !d.keep) {
        if excess == 0 {
            break;
        }
        let n = dir_bytes(&d.path);
        if fs::remove_dir_all(&d.path).is_ok() {
            excess -= 1;
            out.old_sessions += 1;
            out.bytes += n;
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn session(root: &Path, i: u32, aus: usize, events: usize, capture: usize) -> PathBuf {
        session_n(root, i, aus, events, capture, 2)
    }

    fn session_n(
        root: &Path,
        i: u32,
        aus: usize,
        events: usize,
        capture: usize,
        stats: usize,
    ) -> PathBuf {
        let d = root.join(format!("fl-{i:04}_20261102-1000"));
        fs::create_dir_all(&d).unwrap();
        let mut au = String::from("aulog 1\n# t_us sid bytes flags complete salvaged pts_us\n");
        au += &"1 0 10 0 1 0 0\n".repeat(aus);
        fs::write(d.join(AU_FILE), au).unwrap();
        let mut ev = String::from("{\"format\":\"pal-events\",\"v\":1}\n");
        ev += &"{\"ev\":\"x\"}\n".repeat(events);
        fs::write(d.join(EVENTS_FILE), ev).unwrap();
        fs::write(d.join(STATS_FILE), "{}\n".repeat(1 + stats)).unwrap();
        if capture > 0 {
            fs::write(d.join(CAPTURE_FILE), vec![0u8; capture]).unwrap();
        }
        d
    }

    fn root(tag: &str) -> PathBuf {
        let r = std::env::temp_dir().join(format!("pal-retain-{tag}-{}", std::process::id()));
        let _ = fs::remove_dir_all(&r);
        fs::create_dir_all(&r).unwrap();
        r
    }

    const ROOMY: Budget = Budget {
        capture_bytes: u64::MAX,
        sessions: usize::MAX,
    };

    #[test]
    fn empty_sessions_go_but_the_newest_and_kept_stay() {
        let r = root("empty");
        let a = session(&r, 1, 0, 0, 100);
        let b = session(&r, 2, 5, 0, 0);
        let c = session(&r, 3, 0, 0, 0);
        fs::write(c.join(KEEP_FILE), "").unwrap();
        let d = session(&r, 4, 0, 1, 0);
        // Long, with frames but no AU and no event: every frame CRC-failed.
        let f = session_n(&r, 5, 0, 0, 100, 2494);
        let e = session(&r, 6, 0, 0, 0);
        let p = prune(&r, ROOMY).unwrap();
        assert_eq!(p.empty_sessions, 1);
        assert!(!a.exists());
        assert!(b.exists() && c.exists() && d.exists() && e.exists() && f.exists());
        let _ = fs::remove_dir_all(&r);
    }

    #[test]
    fn captures_go_oldest_first_until_they_fit() {
        let r = root("cap");
        let a = session(&r, 1, 1, 0, 400);
        let b = session(&r, 2, 1, 0, 400);
        fs::write(b.join(KEEP_FILE), "").unwrap();
        let c = session(&r, 3, 1, 0, 400);
        let d = session(&r, 4, 1, 0, 400);
        let p = prune(
            &r,
            Budget {
                capture_bytes: 900,
                sessions: usize::MAX,
            },
        )
        .unwrap();
        // 1600 -> drop a (1200) -> b is kept -> drop c (800).
        assert_eq!((p.captures, p.bytes), (2, 800));
        assert!(!a.join(CAPTURE_FILE).exists() && a.join(AU_FILE).exists());
        assert!(b.join(CAPTURE_FILE).exists());
        assert!(!c.join(CAPTURE_FILE).exists());
        assert!(d.join(CAPTURE_FILE).exists());
        let _ = fs::remove_dir_all(&r);
    }

    #[test]
    fn whole_sessions_go_past_the_count_cap() {
        let r = root("count");
        let dirs: Vec<_> = (1..=5).map(|i| session(&r, i, 1, 0, 0)).collect();
        fs::write(dirs[0].join(KEEP_FILE), "").unwrap();
        let p = prune(
            &r,
            Budget {
                capture_bytes: u64::MAX,
                sessions: 3,
            },
        )
        .unwrap();
        assert_eq!(p.old_sessions, 2);
        assert!(dirs[0].exists() && !dirs[1].exists() && !dirs[2].exists());
        assert!(dirs[3].exists() && dirs[4].exists());
        let _ = fs::remove_dir_all(&r);
    }

    #[test]
    fn the_newest_session_is_found_past_four_digits() {
        let r = root("five");
        let a = session(&r, 9999, 0, 0, 0);
        let b = session(&r, 10000, 0, 0, 0);
        let p = prune(&r, ROOMY).unwrap();
        assert_eq!(p.empty_sessions, 1);
        assert!(!a.exists() && b.exists());
    }

    #[test]
    fn a_missing_root_is_nothing_to_do() {
        let r = std::env::temp_dir().join(format!("pal-retain-none-{}", std::process::id()));
        let _ = fs::remove_dir_all(&r);
        assert_eq!(prune(&r, ROOMY).unwrap(), Pruned::default());
    }
}
