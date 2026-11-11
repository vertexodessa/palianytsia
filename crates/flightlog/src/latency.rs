//! Capture-to-complete delay of one session's base stream: from the drone's
//! capture stamp of a frame (its pts) to the moment the ground station had
//! the whole AU. Complete base AUs only: a truncated AU is emitted at its
//! deadline, which is a property of the wait and not of the link.
//!
//! Absolute when the session knows the drone clock (`pts_off_us` in the stats
//! rows, measured over the uplink round trip; the formula is
//! `pal_core::uplink::cap_to_complete_us`, the one the live loop uses).
//! Without it the delays are counted from the session's fastest AU, which
//! hides whatever every frame pays and drifts with the two clocks.

use crate::keyframes::pct;
use serde_json::Value;

#[derive(Debug, Default)]
pub struct Latency {
    /// The delays are absolute; false = over the session's fastest AU.
    pub absolute: bool,
    /// Every measured AU, sorted (µs).
    pub all_us: Vec<u64>,
    /// The same by the rung of the stats row before the AU, sorted by rung;
    /// rung -1 = before the first stats row or with no ladder.
    pub by_rung: Vec<(i64, Vec<u64>)>,
    /// Complete base AUs left out because no clock offset was known yet.
    pub unmeasured: u64,
    aus: Vec<(u64, u32)>,
    rows: Vec<(u64, i64, Option<i64>)>,
}

/// p50, p90, p95, p99 and max of sorted `v` (µs).
pub fn spread(v: &[u64]) -> [u64; 5] {
    [
        pct(v, 50),
        pct(v, 90),
        pct(v, 95),
        pct(v, 99),
        v.last().copied().unwrap_or(0),
    ]
}

impl Latency {
    pub(crate) fn au(&mut self, t_us: u64, pts_us: u64) {
        self.aus.push((t_us, pts_us as u32));
    }

    pub(crate) fn stats(&mut self, t_us: u64, v: &Value) {
        let rung = v.get("rung").and_then(Value::as_i64).unwrap_or(-1);
        let off = v.get("pts_off_us").and_then(Value::as_i64);
        self.rows.push((t_us, rung, off));
    }

    pub(crate) fn finish(&mut self) {
        self.absolute = self.rows.iter().any(|r| r.2.is_some());
        let mut out: Vec<(i64, i64)> = Vec::with_capacity(self.aus.len());
        for &(t, pts) in &self.aus {
            let i = self.rows.partition_point(|r| r.0 <= t);
            let row = i.checked_sub(1).map(|i| self.rows[i]);
            let rung = row.map_or(-1, |r| r.1);
            let d = if self.absolute {
                match row.and_then(|r| r.2) {
                    Some(off) => pal_core::uplink::cap_to_complete_us(pts, t, off).max(0),
                    None => {
                        self.unmeasured += 1;
                        continue;
                    }
                }
            } else {
                // The two clocks share no origin: keep the wrapped difference
                // and shift it to the fastest AU below.
                i64::from((t as u32).wrapping_sub(pts) as i32)
            };
            out.push((rung, d));
        }
        if !self.absolute {
            // Differences are taken against the first AU so that a u32 wrap
            // between two AUs does not read as a 71 minute step.
            if let Some(&(_, first)) = out.first() {
                for o in &mut out {
                    o.1 = i64::from((o.1 as i32).wrapping_sub(first as i32));
                }
                let min = out.iter().map(|o| o.1).min().unwrap_or(0);
                for o in &mut out {
                    o.1 -= min;
                }
            }
        }
        for (rung, d) in out {
            let d = d as u64;
            self.all_us.push(d);
            match self.by_rung.iter_mut().find(|(r, _)| *r == rung) {
                Some((_, v)) => v.push(d),
                None => self.by_rung.push((rung, vec![d])),
            }
        }
        self.all_us.sort_unstable();
        self.by_rung.sort_by_key(|(r, _)| *r);
        for (_, v) in &mut self.by_rung {
            v.sort_unstable();
        }
        self.aus = Vec::new();
        self.rows = Vec::new();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn spread_is_nearest_rank_below() {
        let v: Vec<u64> = (1..=100).collect();
        assert_eq!(spread(&v), [50, 90, 95, 99, 100]);
        assert_eq!(spread(&[]), [0; 5]);
        assert_eq!(spread(&[7]), [7; 5]);
    }

    /// With the clock offset the delay is the completion time in the drone's
    /// clock minus the capture stamp, split by the rung flown at the time.
    #[test]
    fn absolute_delay_by_rung() {
        let mut l = Latency::default();
        // Drone clock = GS clock + 1_000_000 µs.
        l.stats(0, &json!({"rung": 2, "pts_off_us": 1_000_000}));
        l.stats(1_000_000, &json!({"rung": 4, "pts_off_us": 1_000_000}));
        // Captured at GS time 100_000, complete 20 ms later, on rung 2.
        l.au(120_000, 1_100_000);
        // Captured at GS time 1_500_000, complete 7 ms later, on rung 4.
        l.au(1_507_000, 2_500_000);
        l.finish();
        assert!(l.absolute);
        assert_eq!(l.all_us, vec![7_000, 20_000]);
        assert_eq!(l.by_rung, vec![(2, vec![20_000]), (4, vec![7_000])]);
        assert_eq!(l.unmeasured, 0);
    }

    /// An AU that completes before the first row with an offset cannot be
    /// placed on the drone's clock: it is counted, not guessed.
    #[test]
    fn an_au_before_the_offset_is_left_out() {
        let mut l = Latency::default();
        l.stats(0, &json!({"rung": 0}));
        l.stats(1_000_000, &json!({"rung": 1, "pts_off_us": 500}));
        l.au(500_000, 400_000);
        l.au(1_200_000, 1_190_500);
        l.finish();
        assert_eq!(l.all_us, vec![10_000]);
        assert_eq!(l.unmeasured, 1);
    }

    /// The offset maps into the drone's u32 pts clock, so a stamp just
    /// before the wrap and a completion just after it still read 10 ms.
    #[test]
    fn absolute_delay_across_the_pts_wrap() {
        let mut l = Latency::default();
        let off = i64::from(u32::MAX) - 5_000; // GS 0 = drone pts 2^32 - 5001
        l.stats(0, &json!({"rung": 3, "pts_off_us": off}));
        // Captured at GS time 1_000 (pts 2^32 - 4001), complete at 11_000
        // (pts wrapped to 5_999).
        l.au(11_000, u64::from(u32::MAX - 4_000));
        l.finish();
        assert_eq!(l.all_us, vec![10_000]);
    }

    /// Without an offset the delays are counted from the fastest AU, and a
    /// wrap of the 32-bit difference between two AUs changes nothing.
    #[test]
    fn relative_delay_over_the_fastest_au() {
        let mut l = Latency::default();
        l.stats(0, &json!({"rung": 1}));
        // t - pts as u32: 4_294_967_290, then 4 (wrapped: 10 µs more), then
        // 4_294_967_280 (10 µs less than the first).
        l.au(4_294_967_290, 0);
        l.au(4_294_967_300, 0);
        l.au(4_294_967_280, 0);
        l.finish();
        assert!(!l.absolute);
        assert_eq!(l.all_us, vec![0, 10, 20]);
        assert_eq!(l.by_rung, vec![(1, vec![0, 10, 20])]);
    }
}
