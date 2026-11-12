//! Capture-to-complete delay of one session's base stream: from the drone's
//! capture stamp of a frame (its pts) to the moment the ground station had
//! the whole AU.
//!
//! The percentiles are over complete base AUs only, so they cannot be read
//! alone: a truncated AU is emitted at its deadline and a frame that never
//! came is not in the log at all, and those are the slowest frames of a bad
//! session. Three things stand beside them: the count of truncated and of
//! never emitted frames, the delays of every emitted AU with a truncated one
//! at its emit time (what the viewer waited, for a damaged frame), and the
//! share of all frames the drone captured that were complete within a bound
//! (`within`), which is the number to compare sessions by.
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
    /// Base AUs emitted incomplete, among those the delays cover.
    pub truncated: u64,
    /// Frames the drone captured and the ground station never emitted,
    /// counted from the holes in the pts sequence (`missing_before`).
    pub missing: u64,
    /// Every emitted base AU, a truncated one at its emit time, sorted (µs).
    /// Absolute sessions only.
    pub emitted_us: Vec<u64>,
    aus: Vec<(u64, u32, bool)>,
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

/// How many periods back a hole is judged from, each way.
const PERIOD_SPAN: usize = 32;

fn median(v: &[u64]) -> u64 {
    let mut s = v.to_vec();
    s.sort_unstable();
    s.get(s.len() / 2).copied().unwrap_or(0)
}

/// For each base AU, the frames the drone captured between it and the AU
/// before it that never came out. `aus` is (emit time, pts) in emit order.
///
/// The frame period is not in the log, so it is read from the stamps: the
/// median step of the 32 AUs on each side of a step that does not look like
/// one period. That follows a camera mode change within the session, and it
/// undercounts where more than half of the nearby frames are lost (the
/// median step is then two periods). A camera that stops (an encoder
/// restart) reads as lost frames: the stamps cannot tell the two apart.
/// A hole counts from 1.6 periods up, not from 1.5: with two temporal layers
/// the base stream steps two capture periods, and when the encoder skips one
/// capture slot it carries on from the next, so the base stream steps three
/// (50 ms among 33.3 ms steps: 2338 of 104525 steps of a 58 min session).
/// No base frame was lost there.
/// A step of the pts that disagrees with the ground clock by more than
/// 0.5 s + 1 % is a drone restart or a stamp fault; the ground clock's
/// step stands in for it.
fn missing_before(aus: &[(u64, u32)]) -> Vec<u64> {
    let mut step = vec![0u64; aus.len()];
    for i in 1..aus.len() {
        let by_pts = u64::from(aus[i].1.wrapping_sub(aus[i - 1].1));
        let by_clock = aus[i].0.saturating_sub(aus[i - 1].0);
        step[i] = if by_pts.abs_diff(by_clock) > 500_000 + by_clock / 100 {
            by_clock
        } else {
            by_pts
        };
    }
    let mut out = vec![0u64; aus.len()];
    if aus.len() < 2 {
        return out;
    }
    let mut period = median(&step[1..aus.len().min(2 * PERIOD_SPAN + 2)]);
    for i in 1..aus.len() {
        if period > 0 && step[i].abs_diff(period) <= period / 4 {
            continue;
        }
        let lo = i.saturating_sub(PERIOD_SPAN).max(1);
        let hi = (i + PERIOD_SPAN + 1).min(aus.len());
        let local = median(&step[lo..hi]);
        if local == 0 {
            continue;
        }
        let n = (step[i] + local * 2 / 5) / local;
        if n == 1 {
            period = local;
        }
        out[i] = n.saturating_sub(1);
    }
    out
}

impl Latency {
    /// One base AU as emitted, complete or truncated.
    pub(crate) fn au(&mut self, t_us: u64, pts_us: u64, complete: bool) {
        self.aus.push((t_us, pts_us as u32, complete));
    }

    /// Base frames the drone captured in the span the delays cover:
    /// complete, truncated and never emitted.
    pub fn captured(&self) -> u64 {
        self.all_us.len() as u64 + self.truncated + self.missing
    }

    /// The share of captured base frames that were complete within `ms` of
    /// their capture; a truncated or never emitted frame is a miss. None
    /// without the drone clock (a delay over the fastest AU is no bound).
    pub fn within(&self, ms: u64) -> Option<f64> {
        if !self.absolute || self.captured() == 0 {
            return None;
        }
        let hit = self.all_us.partition_point(|&d| d <= ms * 1000);
        Some(hit as f64 / self.captured() as f64)
    }

    pub(crate) fn stats(&mut self, t_us: u64, v: &Value) {
        let rung = v.get("rung").and_then(Value::as_i64).unwrap_or(-1);
        let off = v.get("pts_off_us").and_then(Value::as_i64);
        self.rows.push((t_us, rung, off));
    }

    pub(crate) fn finish(&mut self) {
        self.absolute = self.rows.iter().any(|r| r.2.is_some());
        let stamps: Vec<(u64, u32)> = self.aus.iter().map(|a| (a.0, a.1)).collect();
        let holes = missing_before(&stamps);
        let mut out: Vec<(i64, i64)> = Vec::with_capacity(self.aus.len());
        for (k, &(t, pts, complete)) in self.aus.iter().enumerate() {
            let i = self.rows.partition_point(|r| r.0 <= t);
            let row = i.checked_sub(1).map(|i| self.rows[i]);
            let rung = row.map_or(-1, |r| r.1);
            let d = if self.absolute {
                match row.and_then(|r| r.2) {
                    Some(off) => pal_core::uplink::cap_to_complete_us(pts, t, off).max(0),
                    None => {
                        // Before the drone clock: outside the span, with the
                        // hole in front of it.
                        self.unmeasured += u64::from(complete);
                        continue;
                    }
                }
            } else {
                0
            };
            self.missing += holes[k];
            if self.absolute {
                self.emitted_us.push(d as u64);
            }
            if !complete {
                self.truncated += 1;
                continue;
            }
            let d = if self.absolute {
                d
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
        self.emitted_us.sort_unstable();
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
        l.au(120_000, 1_100_000, true);
        // Captured at GS time 1_500_000, complete 7 ms later, on rung 4.
        l.au(1_507_000, 2_500_000, true);
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
        l.au(500_000, 400_000, true);
        l.au(1_200_000, 1_190_500, true);
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
        l.au(11_000, u64::from(u32::MAX - 4_000), true);
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
        l.au(4_294_967_290, 0, true);
        l.au(4_294_967_300, 0, true);
        l.au(4_294_967_280, 0, true);
        l.finish();
        assert!(!l.absolute);
        assert_eq!(l.all_us, vec![0, 10, 20]);
        assert_eq!(l.by_rung, vec![(1, vec![0, 10, 20])]);
    }

    /// A session with frames every 10 ms, drone clock = GS clock; `late` are
    /// (frame index, delay µs or None for a truncated frame at 150 ms) and
    /// every other frame completes after 20 ms.
    fn session(frames: u64, late: &[(u64, Option<u64>)], lost: &[u64]) -> Latency {
        let mut l = Latency::default();
        l.stats(0, &json!({"rung": 0, "pts_off_us": 0}));
        for f in 0..frames {
            if lost.contains(&f) {
                continue;
            }
            let pts = 1_000_000 + f * 10_000;
            match late.iter().find(|x| x.0 == f) {
                Some(&(_, Some(d))) => l.au(pts + d, pts, true),
                Some(&(_, None)) => l.au(pts + 150_000, pts, false),
                None => l.au(pts + 20_000, pts, true),
            }
        }
        l.finish();
        l
    }

    /// The survivorship the percentiles have: when the ten slowest frames
    /// are truncated instead of completing late, p99 reads better. The share
    /// within a bound must not: it reads worse.
    #[test]
    fn truncating_the_slowest_frames_reads_worse_not_better() {
        let slow: Vec<u64> = (0..10).map(|k| 5 + k * 10).collect();
        let complete_late: Vec<_> = slow.iter().map(|&f| (f, Some(80_000))).collect();
        let truncated: Vec<_> = slow.iter().map(|&f| (f, None)).collect();
        let a = session(100, &complete_late, &[]);
        let b = session(100, &truncated, &[]);
        assert_eq!((a.truncated, a.missing, a.captured()), (0, 0, 100));
        assert_eq!((b.truncated, b.missing, b.captured()), (10, 0, 100));
        assert!(spread(&b.all_us)[3] < spread(&a.all_us)[3]);
        assert_eq!(a.within(50), Some(0.9));
        assert_eq!(a.within(100), Some(1.0));
        assert_eq!(b.within(50), Some(0.9));
        assert_eq!(b.within(100), Some(0.9));
        // The viewer's wait for a truncated frame is in the second set.
        assert_eq!(b.emitted_us.len(), 100);
        assert_eq!(spread(&b.emitted_us)[4], 150_000);
        assert_eq!(spread(&b.all_us)[4], 20_000);
    }

    /// A frame that never came out is a hole in the pts sequence: it is in
    /// the denominator and a miss at every bound.
    #[test]
    fn never_emitted_frames_are_counted_from_the_pts_holes() {
        let l = session(200, &[], &[50, 51, 52, 120]);
        assert_eq!(l.all_us.len(), 196);
        assert_eq!((l.truncated, l.missing, l.captured()), (0, 4, 200));
        assert_eq!(l.within(30), Some(196.0 / 200.0));
    }

    /// A camera mode change inside a session (10 ms frames, then 20 ms) is
    /// not read as every second frame lost, and a hole after it is counted
    /// in the new period.
    #[test]
    fn the_frame_period_follows_a_mode_change() {
        let mut aus: Vec<(u64, u32)> = (0..100u32).map(|f| (0, 1_000_000 + f * 10_000)).collect();
        let base = 1_000_000 + 99 * 10_000;
        for f in 1..=100u32 {
            if f == 60 {
                continue;
            }
            aus.push((0, base + f * 20_000));
        }
        for a in &mut aus {
            a.0 = u64::from(a.1);
        }
        let holes = missing_before(&aus);
        assert_eq!(holes.iter().sum::<u64>(), 1);
        assert_eq!(holes[100 + 59], 1);
    }

    /// Base frames every 33.3 ms (two layers at 60 fps): a step of 50 ms is
    /// the encoder skipping one capture slot, no lost base frame; a step of
    /// 66.7 ms is one.
    #[test]
    fn a_step_of_one_and_a_half_periods_is_no_hole() {
        let mut aus = Vec::new();
        let mut pts = 1_000_000u32;
        for f in 0..120 {
            pts += match f {
                40 => 50_017,
                80 => 66_700,
                _ => 33_351,
            };
            aus.push((u64::from(pts), pts));
        }
        let holes = missing_before(&aus);
        assert_eq!(holes[40], 0);
        assert_eq!(holes[80], 1);
        assert_eq!(holes.iter().sum::<u64>(), 1);
    }

    /// A drone restart sets the pts back; the hole is sized by the ground
    /// clock (300 ms of 10 ms frames), not by the wrapped pts step.
    #[test]
    fn a_pts_reset_is_sized_by_the_ground_clock() {
        let mut aus: Vec<(u64, u32)> = (0..50u32)
            .map(|f| (5_000_000 + u64::from(f) * 10_000, 9_000_000 + f * 10_000))
            .collect();
        let t = aus[49].0 + 300_000;
        aus.extend((0..50u32).map(|f| (t + u64::from(f) * 10_000, 40_000 + f * 10_000)));
        let holes = missing_before(&aus);
        assert_eq!(holes.iter().sum::<u64>(), 29);
        assert_eq!(holes[50], 29);
    }

    /// Without the drone clock there is no bound to hold a delay against;
    /// the truncated and missing counts are still given.
    #[test]
    fn no_share_without_the_drone_clock() {
        let mut l = Latency::default();
        l.stats(0, &json!({"rung": 1}));
        for f in 0..40u64 {
            if f == 7 {
                continue;
            }
            l.au(f * 10_000 + 500, f * 10_000, f != 20);
        }
        l.finish();
        assert!(!l.absolute);
        assert_eq!((l.all_us.len(), l.truncated, l.missing), (38, 1, 1));
        assert_eq!(l.within(100), None);
        assert!(l.emitted_us.is_empty());
    }

    /// AUs before the first clock offset are outside the span: neither they
    /// nor the hole in front of them count.
    #[test]
    fn the_span_starts_with_the_drone_clock() {
        let mut l = Latency::default();
        l.stats(0, &json!({"rung": 0}));
        l.stats(1_000_000, &json!({"rung": 0, "pts_off_us": 0}));
        for f in 0..200u64 {
            if f == 10 || f == 150 {
                continue;
            }
            let pts = f * 10_000;
            l.au(pts + 20_000, pts, f != 30);
        }
        l.finish();
        // Frames 0..=97 are emitted before t = 1 s: 97 of them (10 is lost),
        // one truncated.
        assert_eq!(l.unmeasured, 96);
        assert_eq!((l.all_us.len(), l.truncated, l.missing), (101, 0, 1));
    }
}
