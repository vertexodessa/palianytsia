//! The operator's channel scan: the panel's "how clear is each channel"
//! before choosing. Short dwells away from the link channel and straight
//! back (the in-flight scout's dwell), a few channels per step so the link
//! card is never gone for long, over several passes; per channel the mean
//! busy share of the air and the foreign frames heard. Pal only (WebGs has
//! no such view); it measures, it never moves the link.
use crate::json::J;

/// Dwell length: long enough for a busy reading. On one card it is not
/// cheap: a step (PER_STEP dwells back to back, each 10 ms plus about 2.3 ms
/// of retunes and register reads on an 8812AU) left 31 ms holes in the
/// drone's frames every STEP_MS, about 17 frames at the lowest rung and 70
/// at the top; the bench lost 19-27 % for the 6 s of a scan. So with one
/// card the panel scans only when asked.
pub const OBSERVE_MS: i32 = 10;
/// Passes over the set; a channel's reading is their mean.
pub const PASSES: u32 = 4;
/// Dwells per step, and the time between steps (the link's own time).
pub const PER_STEP: usize = 2;
pub const STEP_MS: f64 = 120.0;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ScanRow {
    pub ch: u8,
    /// Dwells that read the air (retunes both ways succeeded).
    pub n: u32,
    busy_n: u32,
    busy_sum: f64,
    foreign: u64,
    observe_ms: u64,
}

impl ScanRow {
    /// Mean busy share in percent; None before a valid reading.
    pub fn busy_pct(&self) -> Option<f64> {
        (self.busy_n > 0).then(|| self.busy_sum / self.busy_n as f64)
    }
    /// Foreign frames per second over the dwells.
    pub fn foreign_pps(&self) -> Option<f64> {
        (self.observe_ms > 0).then(|| self.foreign as f64 * 1000.0 / self.observe_ms as f64)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ScanState {
    #[default]
    Idle,
    Running,
    Done,
}

/// One dwell's reading as the scan keeps it.
#[derive(Clone, Copy, Debug, Default)]
pub struct Reading {
    pub ok: bool,
    pub busy: Option<f64>,
    pub foreign: u32,
    pub observe_ms: u64,
}

#[derive(Clone, Debug, Default)]
pub struct OpScan {
    pub state: ScanState,
    rows: Vec<ScanRow>,
    pass: u32,
    idx: usize,
    next_ms: f64,
}

impl OpScan {
    /// A fresh scan of `set` (a running one restarts).
    pub fn start(&mut self, set: &[u8], now_ms: f64) {
        *self = Self {
            state: if set.is_empty() {
                ScanState::Done
            } else {
                ScanState::Running
            },
            rows: set
                .iter()
                .map(|&ch| ScanRow {
                    ch,
                    ..Default::default()
                })
                .collect(),
            pass: 0,
            idx: 0,
            next_ms: now_ms,
        };
    }

    /// The channels to dwell on now (at most PER_STEP), or none when no
    /// step is due. The caller dwells and hands each reading to `record`.
    pub fn due(&self, now_ms: f64) -> Vec<u8> {
        if self.state != ScanState::Running || now_ms < self.next_ms {
            return Vec::new();
        }
        self.rows[self.idx..]
            .iter()
            .take(PER_STEP)
            .map(|r| r.ch)
            .collect()
    }

    /// The reading of the next channel in order.
    pub fn record(&mut self, r: Reading, now_ms: f64) {
        if self.state != ScanState::Running {
            return;
        }
        let row = &mut self.rows[self.idx];
        if r.ok {
            row.n += 1;
            if let Some(b) = r.busy {
                row.busy_n += 1;
                row.busy_sum += b;
            }
            row.foreign += u64::from(r.foreign);
            row.observe_ms += r.observe_ms;
        }
        self.idx += 1;
        if self.idx == self.rows.len() {
            self.idx = 0;
            self.pass += 1;
            if self.pass >= PASSES {
                self.state = ScanState::Done;
            }
        }
        self.next_ms = now_ms + STEP_MS;
    }

    /// The step could not run (a hop, the boot scout on the card): try
    /// again next step.
    pub fn defer(&mut self, now_ms: f64) {
        self.next_ms = now_ms + STEP_MS;
    }

    pub fn rows(&self) -> &[ScanRow] {
        &self.rows
    }

    /// Passes done, and of how many.
    pub fn progress(&self) -> (u32, u32) {
        (self.pass, PASSES)
    }

    /// The view for the UI line: state, progress, one row per channel.
    /// `op` marks the link's channel (its reading includes the link).
    pub fn json(&self, op: u8) -> Option<String> {
        if self.state == ScanState::Idle {
            return None;
        }
        let mut rows = J::arr();
        for r in &self.rows {
            let mut o = J::obj();
            o.set("ch", r.ch);
            o.set("n", r.n);
            o.set(
                "busy",
                r.busy_pct()
                    .map_or(J::Null, |b| J::from((b * 10.0).round() / 10.0)),
            );
            o.set(
                "foreign",
                r.foreign_pps().map_or(J::Null, |f| J::from(f.round())),
            );
            o.set("op", r.ch == op);
            rows.push(o);
        }
        let mut j = J::obj();
        let running = self.state == ScanState::Running;
        j.set("state", if running { "running" } else { "done" });
        j.set("pass", self.pass);
        j.set("passes", PASSES);
        j.set("rows", rows);
        Some(j.dump())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rd(busy: f64, foreign: u32) -> Reading {
        Reading {
            ok: true,
            busy: Some(busy),
            foreign,
            observe_ms: 10,
        }
    }

    #[test]
    fn steps_through_the_set_for_every_pass_then_stops() {
        let mut s = OpScan::default();
        assert!(s.json(40).is_none());
        s.start(&[40, 64, 112], 0.0);
        let mut t = 0.0;
        let mut dwells = 0;
        while s.state == ScanState::Running {
            let due = s.due(t);
            assert!(due.len() <= PER_STEP);
            for ch in due {
                s.record(rd(f64::from(ch), 1), t);
                dwells += 1;
            }
            t += 10.0;
        }
        assert_eq!(dwells, 3 * PASSES as usize);
        assert_eq!(s.progress(), (PASSES, PASSES));
        let r = s.rows()[1];
        assert_eq!((r.ch, r.n), (64, PASSES));
        assert_eq!(r.busy_pct(), Some(64.0));
        assert_eq!(r.foreign_pps(), Some(100.0));
        let j = s.json(64).unwrap();
        assert!(j.contains("\"state\":\"done\""), "{j}");
        assert!(
            j.contains("{\"busy\":64.0,\"ch\":64,\"foreign\":100.0,\"n\":4,\"op\":true}"),
            "{j}"
        );
        assert_eq!(j.matches("\"op\":true").count(), 1, "{j}");
    }

    #[test]
    fn waits_between_steps_and_skips_failed_dwells() {
        let mut s = OpScan::default();
        s.start(&[36, 40, 44], 0.0);
        assert_eq!(s.due(0.0), vec![36, 40]);
        s.record(rd(5.0, 0), 0.0);
        s.record(Reading::default(), 0.0);
        assert!(s.due(STEP_MS - 1.0).is_empty(), "between steps");
        assert_eq!(s.due(STEP_MS), vec![44]);
        assert_eq!(s.rows()[1].n, 0, "a failed dwell reads nothing");
        assert_eq!(s.rows()[1].busy_pct(), None);
        s.defer(STEP_MS);
        assert!(s.due(STEP_MS + 1.0).is_empty());
    }
}
