//! What the drone sweep (`search`) and the lost watch of
//! `run_runtime_owned` need from the card, the RX queue and the clock, so a
//! test can drive them on a virtual clock with a scripted drone.
use std::sync::mpsc::Receiver;
use std::time::Duration;

use pal_core::RxBody;

use crate::LOST_US;

/// The card, the body queue and the clock as the sweep uses them.
pub(crate) trait SweepIo {
    /// The full SetMonitorChannel.
    fn set_monitor(&mut self, ch: u8, width: u8) -> Result<(), String>;
    /// FastRetune (keeps width and HT40 offset); false = it failed.
    fn fast_retune(&mut self, ch: u8) -> bool;
    /// Monotonic microseconds.
    fn now_us(&self) -> u64;
    fn sleep(&mut self, d: Duration);
    /// Takes a queued body without waiting; false = the queue is empty.
    fn try_take(&mut self) -> bool;
    /// Takes a body, waiting up to `d` for one; false = none came.
    fn take_within(&mut self, d: Duration) -> bool;
}

/// The card the runtime holds and its RX queue.
pub(crate) struct LiveSweep<'a> {
    pub card: &'a devourer_sys::Card,
    pub rx: &'a Receiver<RxBody>,
}

impl SweepIo for LiveSweep<'_> {
    fn set_monitor(&mut self, ch: u8, width: u8) -> Result<(), String> {
        self.card.ctl().set_monitor(ch, width)
    }
    fn fast_retune(&mut self, ch: u8) -> bool {
        self.card.ctl().fast_retune(ch)
    }
    fn now_us(&self) -> u64 {
        crate::mono_default()
    }
    fn sleep(&mut self, d: Duration) {
        std::thread::sleep(d)
    }
    fn try_take(&mut self) -> bool {
        self.rx.try_recv().is_ok()
    }
    fn take_within(&mut self, d: Duration) -> bool {
        self.rx.recv_timeout(d).is_ok()
    }
}

/// When the drone was last heard: a move of the hit count is a drone frame.
pub(crate) struct Heard {
    hits: u32,
    at: u64,
}

impl Heard {
    pub fn new(hits: u32, at: u64) -> Self {
        Heard { hits, at }
    }

    /// True once `hits` has not moved for LOST_US up to `now`.
    pub fn lost(&mut self, hits: u32, now: u64) -> bool {
        if hits != self.hits {
            *self = Heard { hits, at: now };
            return false;
        }
        now.saturating_sub(self.at) >= LOST_US
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Shared, GRID_AFTER_ROUNDS};
    use std::sync::atomic::{AtomicU32, AtomicU8, Ordering};
    use std::sync::Arc;

    const T0: u64 = 1_000_000;
    const MS: u64 = 1000;
    /// One channel visit with no drone: the 5 ms settle, then 20 waits of
    /// 20 ms fill the 400 ms dwell exactly.
    const VISIT: u64 = 405 * MS;

    #[derive(Clone, Copy, Debug, PartialEq)]
    enum Op {
        Monitor(u8, u8),
        Fast(u8),
        Sleep(u64),
        Take(bool),
        Wait(bool),
    }

    /// A card on a virtual clock with a scripted drone. It stands in for
    /// the RX thread too: a drone frame on the tuned channel is a hit and a
    /// queued body only while `cur` is non-zero, as the runtime's stamp has it.
    struct Fake {
        t: u64,
        tuned: u8,
        /// Drone frames on air: (time, channel), sorted by time.
        air: Vec<(u64, u8)>,
        next: usize,
        queued: usize,
        cur: Arc<AtomicU8>,
        hits: Arc<AtomicU32>,
        shared: Arc<Shared>,
        quit_at: Option<u64>,
        fail_fast: Option<u8>,
        fail_monitor: Option<u8>,
        /// Every call with the `cur` it saw.
        ops: Vec<(Op, u8)>,
    }

    impl Fake {
        fn new(tuned: u8, air: Vec<(u64, u8)>) -> Self {
            Fake {
                t: T0,
                tuned,
                air,
                next: 0,
                queued: 0,
                cur: Arc::new(AtomicU8::new(tuned)),
                hits: Arc::default(),
                shared: Arc::default(),
                quit_at: None,
                fail_fast: None,
                fail_monitor: None,
                ops: Vec::new(),
            }
        }

        fn log(&mut self, op: Op) {
            self.ops.push((op, self.cur.load(Ordering::Relaxed)));
        }

        fn set_time(&mut self, t: u64) {
            self.t = t;
            if self.quit_at.is_some_and(|q| t >= q) {
                self.shared.quit.store(true, Ordering::Relaxed);
            }
        }

        /// Moves the clock to `to`; with `stop`, to the first body queued
        /// on the way instead (true).
        fn advance(&mut self, to: u64, stop: bool) -> bool {
            while let Some(&(at, ch)) = self.air.get(self.next) {
                if at > to {
                    break;
                }
                self.next += 1;
                self.set_time(at.max(self.t));
                if ch == self.tuned && self.cur.load(Ordering::Relaxed) != 0 {
                    self.hits.fetch_add(1, Ordering::Relaxed);
                    self.queued += 1;
                    if stop {
                        return true;
                    }
                }
            }
            self.set_time(to);
            false
        }

        fn run(
            &mut self,
            set: &[u8],
            first: u8,
            width: u8,
            tuned: &mut (u8, u8),
        ) -> Result<Option<u8>, String> {
            let (cur, hits, shared) = (self.cur.clone(), self.hits.clone(), self.shared.clone());
            crate::search(self, set, first, width, tuned, &cur, &hits, &shared)
        }

        /// The channels retuned to, in order.
        fn visits(&self) -> Vec<u8> {
            self.ops
                .iter()
                .filter_map(|(op, _)| match *op {
                    Op::Monitor(ch, _) | Op::Fast(ch) => Some(ch),
                    _ => None,
                })
                .collect()
        }

        fn events(&self) -> Vec<(&'static str, serde_json::Value)> {
            let q = self.shared.events.lock().unwrap();
            q.iter().map(|(n, v, _)| (*n, v.clone())).collect()
        }

        /// The time each event carries for the flight log.
        fn times(&self) -> Vec<Option<u64>> {
            let q = self.shared.events.lock().unwrap();
            q.iter().map(|(_, _, at)| *at).collect()
        }

        fn names(&self) -> Vec<&'static str> {
            self.events().into_iter().map(|(n, _)| n).collect()
        }
    }

    impl SweepIo for Fake {
        fn set_monitor(&mut self, ch: u8, width: u8) -> Result<(), String> {
            self.log(Op::Monitor(ch, width));
            if self.fail_monitor == Some(ch) {
                return Err(format!("monitor {ch} refused"));
            }
            self.tuned = ch;
            Ok(())
        }
        fn fast_retune(&mut self, ch: u8) -> bool {
            self.log(Op::Fast(ch));
            if self.fail_fast == Some(ch) {
                return false;
            }
            self.tuned = ch;
            true
        }
        fn now_us(&self) -> u64 {
            self.t
        }
        fn sleep(&mut self, d: Duration) {
            self.log(Op::Sleep(d.as_micros() as u64));
            self.advance(self.t + d.as_micros() as u64, false);
        }
        fn try_take(&mut self) -> bool {
            let got = self.queued > 0;
            self.queued -= usize::from(got);
            self.log(Op::Take(got));
            got
        }
        fn take_within(&mut self, d: Duration) -> bool {
            let got = self.queued > 0 || self.advance(self.t + d.as_micros() as u64, true);
            self.queued -= usize::from(got);
            self.log(Op::Wait(got));
            got
        }
    }

    /// The drone on `ch`, a frame every `every` us from T0 to `until`.
    fn drone(ch: u8, every: u64, until: u64) -> Vec<(u64, u8)> {
        (1..)
            .map(|k| (T0 + k * every, ch))
            .take_while(|&(t, _)| t <= until)
            .collect()
    }

    fn found(f: &Fake) -> serde_json::Value {
        let ev = f.events();
        let (name, v) = ev.last().expect("an event");
        assert_eq!(*name, "drone_found");
        v.clone()
    }

    #[test]
    fn found_on_the_first_channel() {
        let mut f = Fake::new(40, drone(40, 10 * MS, T0 + 60_000 * MS));
        let mut tuned = (40, 20);
        assert_eq!(f.run(&[36, 40, 44], 40, 20, &mut tuned), Ok(Some(40)));
        assert_eq!(f.visits(), [40]);
        // A sweep that ends in its first pass logs no "search".
        assert_eq!(f.names(), ["drone_found"]);
        // Frames at 10, 20, 30 ms: the third is the find.
        assert_eq!(
            found(&f),
            serde_json::json!({"ch": 40, "ms": 30, "rounds": 0})
        );
        assert!(!f.shared.searching.load(Ordering::Relaxed));
        assert_eq!(f.shared.channel.load(Ordering::Relaxed), 40);
        assert_eq!(f.cur.load(Ordering::Relaxed), 40);
        assert_eq!(tuned, (40, 20));
    }

    #[test]
    fn starts_at_first_then_the_set_in_order() {
        let mut f = Fake::new(44, vec![]);
        // Quit lands in the last dwell of pass one: seen before pass two.
        f.quit_at = Some(T0 + 3 * VISIT - 1);
        let mut tuned = (44, 20);
        assert_eq!(f.run(&[36, 40, 44], 44, 20, &mut tuned), Ok(None));
        assert_eq!(f.visits(), [44, 36, 40]);
        assert_eq!(f.names(), ["search"]);
        assert_eq!(
            f.events()[0].1,
            serde_json::json!({"set": [36, 40, 44], "first": 44})
        );

        // A first channel outside the set (a pinned core's) leads the order.
        let mut f = Fake::new(149, vec![]);
        f.quit_at = Some(T0 + 4 * VISIT - 1);
        let mut tuned = (149, 20);
        assert_eq!(f.run(&[36, 40, 44], 149, 20, &mut tuned), Ok(None));
        assert_eq!(f.visits(), [149, 36, 40, 44]);
    }

    #[test]
    fn settles_5_ms_blind_and_drains_before_the_dwell() {
        // Drone frames during the settle fall on cur = 0 and must not count;
        // bodies queued before the sweep are taken off and counted.
        let air = (1..=4).map(|k| (T0 + k * MS, 36)).collect();
        let mut f = Fake::new(36, air);
        f.queued = 5;
        f.quit_at = Some(T0 + VISIT - 1);
        let mut tuned = (36, 20);
        assert_eq!(f.run(&[36], 36, 20, &mut tuned), Ok(None));
        let head: Vec<(Op, u8)> = f.ops[..8].to_vec();
        assert_eq!(
            head,
            [
                (Op::Fast(36), 0),
                (Op::Sleep(5 * MS), 0),
                (Op::Take(true), 0),
                (Op::Take(true), 0),
                (Op::Take(true), 0),
                (Op::Take(true), 0),
                (Op::Take(true), 0),
                (Op::Take(false), 0),
            ]
        );
        assert_eq!(f.ops[8], (Op::Wait(false), 36));
        let waits = f
            .ops
            .iter()
            .filter(|(op, _)| matches!(op, Op::Wait(_)))
            .count();
        assert_eq!(waits, 20);
        assert_eq!(f.hits.load(Ordering::Relaxed), 0);
        assert_eq!(f.shared.lat.rx_recv.load(Ordering::Relaxed), 5);
        assert_eq!(f.t, T0 + VISIT);
    }

    #[test]
    fn three_hits_must_fall_in_one_dwell() {
        // One channel: dwell one listens from 5 ms until its last 20 ms wait
        // ends at 420 ms, dwell two from 425 ms. Two
        // frames late in the first and one in the second make three hits,
        // never three in one dwell.
        let air = vec![
            (T0 + 390 * MS, 36),
            (T0 + 400 * MS, 36),
            (T0 + 500 * MS, 36),
        ];
        let mut f = Fake::new(36, air);
        f.quit_at = Some(T0 + 900 * MS);
        let mut tuned = (36, 20);
        assert_eq!(f.run(&[36], 36, 20, &mut tuned), Ok(None));
        assert_eq!(f.hits.load(Ordering::Relaxed), 3);
        assert!(!f.names().contains(&"drone_found"));

        // The same three inside one dwell are a find.
        let air = vec![
            (T0 + 300 * MS, 36),
            (T0 + 350 * MS, 36),
            (T0 + 400 * MS, 36),
        ];
        let mut f = Fake::new(36, air);
        let mut tuned = (36, 20);
        assert_eq!(f.run(&[36], 36, 20, &mut tuned), Ok(Some(36)));
        assert_eq!(found(&f)["ms"], 400);
    }

    #[test]
    fn grid_channel_found_only_after_two_passes() {
        let mut f = Fake::new(36, drone(100, 10 * MS, T0 + 600_000 * MS));
        let mut tuned = (36, 20);
        assert_eq!(f.run(&[36, 40], 36, 20, &mut tuned), Ok(Some(100)));
        let v = f.visits();
        assert_eq!(v[..4], [36, 40, 36, 40]);
        let grid: Vec<u8> = pal_wire::ht40::grid_5g(20)
            .into_iter()
            .filter(|c| ![36, 40].contains(c))
            .collect();
        // Pass three: the set again, then the grid in its own order up to
        // the drone's channel.
        let at = grid.iter().position(|&c| c == 100).unwrap();
        assert_eq!(v[4..6], [36, 40]);
        assert_eq!(v[6..], grid[..=at]);
        assert_eq!(f.names(), ["search", "search_grid", "drone_found"]);
        assert_eq!(f.events()[1].1, serde_json::json!({"channels": grid.len()}));
        assert_eq!(found(&f)["rounds"], GRID_AFTER_ROUNDS);
        assert_eq!(found(&f)["ch"], 100);
        // The core loop does not tick while the sweep runs, so each event
        // carries its own time: the search from its start, the grid when the
        // second pass ended, the find when it happened.
        let ms = found(&f)["ms"].as_u64().unwrap();
        assert_eq!(
            f.times(),
            [Some(T0), Some(T0 + 4 * VISIT), Some(T0 + ms * 1000)]
        );
        assert!(ms * 1000 > 6 * VISIT, "{ms}");
    }

    #[test]
    fn grid_appended_exactly_once() {
        let mut f = Fake::new(36, vec![]);
        // Pass three and later cover set plus grid: 25 channels at 20 MHz.
        f.quit_at = Some(T0 + (2 * 2 + 3 * 25) * VISIT - 1);
        let mut tuned = (36, 20);
        assert_eq!(f.run(&[36, 40], 36, 20, &mut tuned), Ok(None));
        assert_eq!(f.names(), ["search", "search_grid"]);
        let v = f.visits();
        assert_eq!(v.len(), 4 + 3 * 25);
        let pass = &v[4..29];
        assert_eq!(&v[29..54], pass);
        assert_eq!(&v[54..], pass);
        let mut sorted = pass.to_vec();
        sorted.sort_unstable();
        assert_eq!(sorted, pal_wire::ht40::grid_5g(20));
    }

    #[test]
    fn grid_at_40_mhz_leaves_out_unpaired_channels() {
        let mut f = Fake::new(36, vec![]);
        f.quit_at = Some(T0 + (2 * 2 + 24) * VISIT - 1);
        let mut tuned = (36, 40);
        assert_eq!(f.run(&[36, 40], 36, 40, &mut tuned), Ok(None));
        assert_eq!(f.events()[1].1, serde_json::json!({"channels": 22}));
        assert!(!f.visits().contains(&165));
    }

    #[test]
    fn quit_before_the_sweep_returns_none_untouched() {
        let mut f = Fake::new(36, drone(36, 10 * MS, T0 + 1000 * MS));
        f.shared.quit.store(true, Ordering::Relaxed);
        let mut tuned = (36, 20);
        assert_eq!(f.run(&[36, 40], 36, 20, &mut tuned), Ok(None));
        assert!(f.ops.is_empty());
        assert!(f.names().is_empty());
        assert!(!f.shared.searching.load(Ordering::Relaxed));
    }

    #[test]
    fn quit_inside_a_dwell_is_seen_at_the_next_channel() {
        // The sweep checks quit once per channel: a quit 100 ms into a dwell
        // waits out the rest of it.
        let mut f = Fake::new(36, vec![]);
        f.quit_at = Some(T0 + 100 * MS);
        let mut tuned = (36, 20);
        assert_eq!(f.run(&[36, 40], 36, 20, &mut tuned), Ok(None));
        assert_eq!(f.visits(), [36]);
        assert_eq!(f.t, T0 + VISIT);
    }

    #[test]
    fn failing_fast_retune_is_an_error() {
        let mut f = Fake::new(36, vec![]);
        f.fail_fast = Some(40);
        let mut tuned = (36, 20);
        assert_eq!(
            f.run(&[36, 40], 36, 20, &mut tuned),
            Err("fast_retune(40) failed".to_string())
        );
        assert_eq!(f.visits(), [36, 40]);
        // The card is left where the last good move put it.
        assert_eq!(tuned, (36, 20));
        assert_eq!(f.cur.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn failing_set_monitor_is_an_error() {
        let mut f = Fake::new(36, vec![]);
        f.fail_monitor = Some(40);
        let mut tuned = (36, 40);
        assert_eq!(
            f.run(&[36, 40], 36, 40, &mut tuned),
            Err("monitor 40 refused".to_string())
        );
        assert_eq!(tuned, (36, 40));
    }

    #[test]
    fn retune_plan_picks_the_full_set_when_offset_or_width_change() {
        // 36 is HT40+, 40 HT40-, 165 has no pair, 44 is HT40+.
        let mut f = Fake::new(36, vec![]);
        f.quit_at = Some(T0 + 5 * VISIT - 1);
        let mut tuned = (36, 40);
        assert_eq!(f.run(&[36, 40, 165, 44], 36, 40, &mut tuned), Ok(None));
        let moves: Vec<Op> = f
            .ops
            .iter()
            .map(|(op, _)| *op)
            .filter(|op| matches!(op, Op::Monitor(..) | Op::Fast(_)))
            .collect();
        assert_eq!(
            moves,
            [
                Op::Fast(36),
                Op::Monitor(40, 40),
                Op::Monitor(165, 20),
                Op::Monitor(44, 40),
                Op::Fast(36),
            ]
        );
        assert_eq!(tuned, (36, 40));
    }

    #[test]
    fn lost_after_8_s_without_a_hit() {
        let mut h = Heard::new(7, T0);
        assert!(!h.lost(7, T0 + LOST_US - 1));
        assert!(h.lost(7, T0 + LOST_US));
        // A hit restarts the wait from when it is seen.
        assert!(!h.lost(8, T0 + LOST_US + 5));
        assert!(!h.lost(8, T0 + 2 * LOST_US + 4));
        assert!(h.lost(8, T0 + 2 * LOST_US + 5));
    }
}
