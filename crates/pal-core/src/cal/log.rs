//! `cal.log`, the raw per-cell record of a calibration run (mabur
//! `cal_log`). The format is mabur's `callog 6`, byte for byte, so mabur's
//! `maburcal report` reads a file the Rust GS wrote:
//!
//! ```text
//! callog 6                                       once per file
//! R <nonce> <margin_db> <ch> <pick> <ldpc>        once per run
//! S <seq> <power|capacity|floor> <bw> <frames|dwell_ms> scopes the rows below
//! C <rate> <idx> <expected> <recv0> <recv1> <corrupt> <rssi0> <rssi1>
//! W <rate> <wall> <floor> <best_card> <flags>
//! P <rate> <offered> <recv_best> <air_bits_best> <eff_x1000> <blocked_pct> <flags> <span_ms>
//! V <rate> <idx> <pct>
//! F <floor> <min_fps>                            per floor candidate tried
//! L <ref_fps> <found> <confirm_pct> <written>    per floor sweep; 0 = none
//! A <bw> <0|1> <reason to end of line>
//! N <bw> <why to end of line>                    the width's rest skipped
//! ```
//!
//! The line builders are pure; `CalLog` only appends them. Every I/O
//! failure turns the log into a no-op: a calibration never fails over its
//! log.

use std::fs::{File, OpenOptions};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

use pal_wire::cal::{FloorReport, Kind};

use super::analysis::{CalCell, CapCell, CapResult, RateWall, RSSI_NONE};

pub const MARKER: &str = "callog 6";

pub fn run_line(nonce: u32, margin_db: f64, ch: u8, pick: &str, ldpc: bool) -> String {
    format!("R {nonce} {margin_db:.2} {ch} {pick} {}", u8::from(ldpc))
}

/// A verify pass shares the `power` word: both are frame counts. A floor
/// sweep's param is its per-candidate dwell_ms.
pub fn sweep_line(seq: u8, kind: Kind, bw: u8, param: u32) -> String {
    let k = match kind {
        Kind::Capacity => "capacity",
        Kind::Floor => "floor",
        _ => "power",
    };
    format!("S {seq} {k} {bw} {param}")
}

/// -999 for a card without a reading, never the raw slot.
pub fn cell_line(rate: u8, idx: i32, c: &CalCell) -> String {
    let rssi = |i: usize| {
        if c.have_rssi[i] {
            c.rssi_dbm[i]
        } else {
            RSSI_NONE
        }
    };
    format!(
        "C {rate} {idx} {} {} {} {} {} {}",
        c.expected,
        c.received[0],
        c.received[1],
        c.corrupt,
        rssi(0),
        rssi(1)
    )
}

pub fn wall_line(rate: u8, w: &RateWall) -> String {
    format!(
        "W {rate} {} {} {} {}",
        w.wall, w.floor_idx, w.best_card, w.flags
    )
}

/// The best card's columns, never a union.
pub fn cap_line(rate: u8, c: &CapCell, r: &CapResult) -> String {
    let b = usize::try_from(r.best_card).unwrap_or(0).min(1);
    // lround: half away from zero, which f64::round is.
    let eff_x1000 = (r.eff * 1000.0).round() as i32;
    format!(
        "P {rate} {} {} {} {eff_x1000} {} {} {}",
        c.offered, c.frames[b], c.air_bits[b], c.blocked_pct, r.flags, r.span_ms
    )
}

pub fn verify_line(rate: u8, idx: i32, pct: i32) -> String {
    format!("V {rate} {idx} {pct}")
}

/// A control byte would split the record: each becomes a space.
fn flatten(s: &str) -> String {
    s.chars()
        .map(|c| if (c as u32) < 0x20 { ' ' } else { c })
        .collect()
}

/// The reason comes from the drone, so it is flattened.
pub fn applied_line(bw: u8, ok: bool, reason: &str) -> String {
    format!("A {bw} {} {}", u8::from(ok), flatten(reason))
}

/// A width's remaining sweeps skipped; `why` may carry the drone's words.
pub fn skipped_line(bw: u8, why: &str) -> String {
    format!("N {bw} {}", flatten(why))
}

/// One floor sweep: `F <floor> <min_fps>` per candidate tried, then
/// `L <ref_fps> <found> <confirm_pct> <write>` (found 0 = none passed,
/// write 0 = nothing written).
pub fn floor_lines(r: &FloorReport, cands: &[u8], write: u8) -> Vec<String> {
    let mut out: Vec<String> = cands
        .iter()
        .zip(r.min_fps.iter())
        .take(usize::from(r.tried))
        .map(|(c, m)| format!("F {c} {m}"))
        .collect();
    out.push(format!(
        "L {} {} {} {write}",
        r.ref_fps, r.floor, r.confirm_pct
    ));
    out
}

/// Whether the marker is due in `dir`, retiring a `cal.log` of another
/// format first (renamed to `cal.log.callog<N>` or `cal.log.unmarked`):
/// one file holds one format's rows.
pub fn prepare(dir: &Path) -> bool {
    let path = dir.join("cal.log");
    let Ok(text) = std::fs::read(&path) else {
        return true;
    };
    let first = text.split(|&b| b == b'\n').next().unwrap_or(&[]);
    let first = String::from_utf8_lossy(first);
    if first == MARKER {
        return false;
    }
    let tag = match first.strip_prefix("callog ") {
        Some(v) => format!(
            "callog{}",
            v.chars()
                .filter(char::is_ascii_alphanumeric)
                .collect::<String>()
        ),
        None => "unmarked".to_string(),
    };
    let mut retired = path.clone().into_os_string();
    retired.push(format!(".{tag}"));
    let _ = std::fs::rename(&path, PathBuf::from(retired));
    true
}

pub struct CalLog {
    w: Option<BufWriter<File>>,
}

impl CalLog {
    /// Appends to `<dir>/cal.log`, writing the marker when the file is new
    /// or held another format.
    pub fn open(dir: &Path) -> Self {
        let due = prepare(dir);
        let w = OpenOptions::new()
            .create(true)
            .append(true)
            .open(dir.join("cal.log"))
            .ok()
            .map(BufWriter::new);
        let mut l = Self { w };
        if due {
            l.line(MARKER);
        }
        l
    }

    /// A log that writes nothing (no flight-log directory).
    pub fn none() -> Self {
        Self { w: None }
    }

    pub fn ok(&self) -> bool {
        self.w.is_some()
    }

    pub fn line(&mut self, s: &str) {
        if let Some(w) = &mut self.w {
            if writeln!(w, "{s}").is_err() {
                self.w = None;
            }
        }
    }

    /// At the end of a run (done or failed): a reader looks at the file
    /// the moment the state says done.
    pub fn flush(&mut self) {
        if let Some(w) = &mut self.w {
            if w.flush().is_err() {
                self.w = None;
            }
        }
    }

    pub fn run(&mut self, nonce: u32, margin_db: f64, ch: u8, pick: &str, ldpc: bool) {
        self.line(&run_line(nonce, margin_db, ch, pick, ldpc));
    }
    pub fn sweep(&mut self, seq: u8, kind: Kind, bw: u8, param: u32) {
        self.line(&sweep_line(seq, kind, bw, param));
    }
    pub fn cell(&mut self, rate: u8, idx: i32, c: &CalCell) {
        self.line(&cell_line(rate, idx, c));
    }
    pub fn wall(&mut self, rate: u8, w: &RateWall) {
        self.line(&wall_line(rate, w));
    }
    pub fn cap(&mut self, rate: u8, c: &CapCell, r: &CapResult) {
        self.line(&cap_line(rate, c, r));
    }
    pub fn verify(&mut self, rate: u8, idx: i32, pct: i32) {
        self.line(&verify_line(rate, idx, pct));
    }
    pub fn applied(&mut self, bw: u8, ok: bool, reason: &str) {
        self.line(&applied_line(bw, ok, reason));
    }
    pub fn skipped(&mut self, bw: u8, why: &str) {
        self.line(&skipped_line(bw, why));
    }
    pub fn floor(&mut self, r: &FloorReport, cands: &[u8], write: u8) {
        for l in floor_lines(r, cands, write) {
            self.line(&l);
        }
    }
}

impl Drop for CalLog {
    fn drop(&mut self) {
        self.flush();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cal::analysis::{FLAG_SHORT_SPAN, FLAG_TX_BOUND};

    fn fresh(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("pal-callog-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d
    }
    fn lines(d: &Path) -> Vec<String> {
        std::fs::read_to_string(d.join("cal.log"))
            .unwrap_or_default()
            .lines()
            .map(str::to_string)
            .collect()
    }

    #[test]
    fn records_match_mabur() {
        assert_eq!(run_line(42, 1.0, 149, "auto", true), "R 42 1.00 149 auto 1");
        assert_eq!(
            run_line(7, 0.5, 64, "onecard", false),
            "R 7 0.50 64 onecard 0"
        );
        assert_eq!(sweep_line(0, Kind::Power, 20, 20), "S 0 power 20 20");
        assert_eq!(
            sweep_line(1, Kind::Capacity, 40, 2000),
            "S 1 capacity 40 2000"
        );
        assert_eq!(sweep_line(2, Kind::Verify, 20, 20), "S 2 power 20 20");
        let c = CalCell {
            idx: 56,
            expected: 100,
            received: [97, 12],
            corrupt: 3,
            rssi_dbm: [-67, 5],
            have_rssi: [true, false],
        };
        assert_eq!(cell_line(7, 56, &c), "C 7 56 100 97 12 3 -67 -999");
        let w = RateWall {
            wall: -1,
            floor_idx: -1,
            best_card: 0,
            flags: 0,
        };
        assert_eq!(wall_line(7, &w), "W 7 -1 -1 0 0");
        let w = RateWall {
            wall: -4,
            floor_idx: -40,
            best_card: 0,
            flags: 0,
        };
        assert_eq!(wall_line(7, &w), "W 7 -4 -40 0 0");
        assert_eq!(wall_line(3, &RateWall::default()), "W 3 -128 -128 -1 0");
        assert_eq!(verify_line(7, -8, 97), "V 7 -8 97");
        assert_eq!(verify_line(5, 50, 97), "V 5 50 97");
        assert_eq!(applied_line(20, true, "ok"), "A 20 1 ok");
        assert_eq!(
            applied_line(40, false, "out of range table"),
            "A 40 0 out of range table"
        );
        assert_eq!(
            applied_line(20, false, "line one\nline two\ttab"),
            "A 20 0 line one line two tab"
        );
        assert_eq!(sweep_line(2, Kind::Floor, 40, 4000), "S 2 floor 40 4000");
        assert_eq!(
            skipped_line(40, "floor: ch 165 has no\n40 MHz pair"),
            "N 40 floor: ch 165 has no 40 MHz pair"
        );
    }

    /// mabur tests/test_cal_log.cpp: the edge 0x24 behind a marginal 0x22.
    #[test]
    fn floor_rows_one_per_candidate_tried_then_the_result() {
        let mut r = FloorReport {
            ref_fps: 2484,
            floor: 0x24,
            tried: 3,
            confirm_pct: 100,
            ..FloorReport::default()
        };
        r.min_fps[..3].copy_from_slice(&[0, 2130, 2484]);
        let cands = pal_wire::cal::floor_candidates(0x20, 0x36);
        assert_eq!(
            floor_lines(&r, &cands, 0x35),
            ["F 32 0", "F 34 2130", "F 36 2484", "L 2484 36 100 53"]
        );
        assert_eq!(
            floor_lines(&FloorReport::default(), &cands, 0),
            ["L 0 0 0 0"]
        );
    }

    #[test]
    fn cap_record_takes_the_best_cards_columns() {
        let c = CapCell {
            offered: 4000,
            blocked_pct: 72,
            frames: [3950, 1200],
            air_bits: [58_000_000, 20_000_000],
            ..CapCell::default()
        };
        let r = CapResult {
            eff: 0.73,
            best_card: 0,
            span_ms: 2287,
            flags: FLAG_TX_BOUND,
        };
        assert_eq!(cap_line(5, &c, &r), "P 5 4000 3950 58000000 730 72 64 2287");
        let c = CapCell {
            offered: 500,
            blocked_pct: 10,
            frames: [100, 480],
            air_bits: [1000, 9000],
            ..CapCell::default()
        };
        let r = CapResult {
            eff: 0.5,
            best_card: 1,
            span_ms: 140,
            flags: FLAG_SHORT_SPAN,
        };
        assert_eq!(cap_line(2, &c, &r), "P 2 500 480 9000 500 10 512 140");
    }

    #[test]
    fn one_marker_per_file_across_runs_and_reopens() {
        let d = fresh("reopen");
        {
            let mut l = CalLog::open(&d);
            l.run(42, 1.0, 149, "auto", true);
            l.verify(0, 10, 90);
        }
        {
            let mut l = CalLog::open(&d);
            l.run(43, 1.5, 149, "stayed", true);
            l.verify(0, 12, 95);
        }
        assert_eq!(
            lines(&d),
            [
                "callog 6",
                "R 42 1.00 149 auto 1",
                "V 0 10 90",
                "R 43 1.50 149 stayed 1",
                "V 0 12 95"
            ]
        );
    }

    #[test]
    fn an_older_format_is_retired_not_appended_to() {
        let d = fresh("stale");
        std::fs::write(d.join("cal.log"), "callog 3\nR 1 1.00\n").unwrap();
        {
            let mut l = CalLog::open(&d);
            l.verify(1, 2, 3);
        }
        assert_eq!(lines(&d), ["callog 6", "V 1 2 3"]);
        let old = std::fs::read_to_string(d.join("cal.log.callog3")).unwrap();
        assert!(old.starts_with("callog 3\n"));
    }

    #[test]
    fn a_headerless_file_is_retired_as_unmarked() {
        let d = fresh("unmarked");
        std::fs::write(d.join("cal.log"), "R 1 1.00 149 auto 1\n").unwrap();
        assert!(prepare(&d));
        assert!(d.join("cal.log.unmarked").exists());
        assert!(!d.join("cal.log").exists());
    }

    #[test]
    fn a_missing_directory_is_a_silent_no_op() {
        let mut l = CalLog::open(Path::new("/nonexistent/pal-callog"));
        assert!(!l.ok());
        l.verify(0, 0, 0);
        l.flush();
    }
}
