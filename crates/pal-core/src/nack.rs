//! Base-layer selective repeat: which sid-0 FEC source symbols the GS asks
//! the drone to re-send (mabur `common/src/nack_tracker.cpp`).
//!
//! Gap trigger: the decoder's erasure set reports a seq missing. Tail
//! trigger: the newest frame header promises more fragments than arrived.
//! First request after an adaptive settle (max natural lateness + 2 ms,
//! clamped), repeats every `repeat_ms` up to `max_tries`. An entry whose
//! answer could not land `min_lead_ms` before the gap deadline is never
//! asked: flight 0026 spent the drone's air bucket on doomed seqs. While the
//! ladder util is at or above `down_util` due entries die without a send, so
//! no catch-up burst follows.

use std::collections::{BTreeMap, VecDeque};

use pal_fec::SourceState;
use pal_wire::rc::{Nack, NackEntry, MAX_NACK_ENTRIES, NACK_FLAG_REPEAT};

use crate::config::NackCfg;

#[derive(Clone, Copy, Debug)]
pub struct NackTailView {
    pub count: u16,
    pub max_idx: u16,
    pub seq_at_max: u32,
    pub last_progress_ms: u64,
}

/// Lazily evaluated, as the C++ std::function inputs are.
pub struct NackInputs<'a> {
    pub missing: &'a dyn Fn() -> Vec<u32>,
    pub state: &'a dyn Fn(u32) -> SourceState,
    pub tail: &'a dyn Fn() -> Option<NackTailView>,
    pub util: &'a dyn Fn() -> f64,
    pub down_util: f64,
    pub gap_timeout_ms: u64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct NackStats {
    pub requests: u64,
    pub repeats: u64,
    pub syms_requested: u64,
    pub tail_requests: u64,
    pub filled: u64,
    pub late_fill: u64,
    pub wasted: u64,
    pub dropped_deadline: u64,
    pub suppressed: u64,
    pub lead_skipped: u64,
    /// Of `filled`, the fills seen only after the seq's deadline (`gap_timeout_ms` after t0): too
    /// late for the frame. Not in the stats line; `link.nack_residual = "timely"` books them as
    /// lost.
    pub filled_late: u64,
}

#[derive(Clone, Debug, Default)]
pub struct NackWindow {
    pub fill_ms: Vec<u32>,
    pub late_ms_max: u32,
    pub filled: u64,
}

pub const MAX_FILL_SAMPLES: usize = 4096;

/// A request answered by the symbol's own first copy (`late_fill`), kept for the flight log
/// only: mabur's tracker counts them and WebGs has no home for the detail. The drone sends in
/// order with no MAC retries (No-Ack broadcast), so a gap entry's copy arriving late means the
/// ground reordered it (two cards' USB batches), and a tail entry's means the drone's queue or
/// the air held it past the settle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LateFill {
    pub t_ms: u64,
    pub seq: u32,
    pub from_tail: bool,
    pub tries: i32,
    /// From first missing (gap: the decoder saw it missing; tail: the frame's last progress)
    /// to the copy.
    pub late_ms: u32,
    /// From the first request to the copy.
    pub after_req_ms: u32,
    pub settle_ms: i32,
}

/// Late fills kept between two takes. The loop takes them every tick; the cap only bounds a
/// loop that stopped taking (8-108 per 27 s arm under the bench jam, 2026-11-13).
pub const LATE_FILL_CAP: usize = 64;

#[derive(Clone, Copy, Default)]
struct Entry {
    first_missing_ms: u64,
    first_sent_ms: u64,
    last_sent_ms: u64,
    tries: i32,
    from_tail: bool,
    dead: bool,
    deadline_counted: bool,
}

pub struct NackTracker {
    cfg: NackCfg,
    entries: BTreeMap<u32, Entry>,
    late: VecDeque<(u64, u32)>,
    settle_ms: i32,
    counter: u32,
    stats: NackStats,
    win: NackWindow,
    late_fills: Vec<LateFill>,
    late_fills_dropped: u64,
}

impl NackTracker {
    pub fn new(cfg: NackCfg) -> Self {
        let settle_ms = cfg.settle_seed_ms;
        Self {
            cfg,
            entries: BTreeMap::new(),
            late: VecDeque::new(),
            settle_ms,
            counter: 0,
            stats: NackStats::default(),
            win: NackWindow::default(),
            late_fills: Vec::new(),
            late_fills_dropped: 0,
        }
    }

    /// frame_wire edge: entries only. The counter survives because the
    /// drone keeps its last accepted counter for the life of the vtx nonce.
    pub fn clear(&mut self) {
        self.entries.clear();
    }
    pub fn restart_counter(&mut self) {
        self.counter = 0;
    }
    pub fn stats(&self) -> &NackStats {
        &self.stats
    }
    pub fn settle_ms(&self) -> i32 {
        self.settle_ms
    }
    pub fn outstanding(&self) -> usize {
        self.entries
            .values()
            .filter(|e| !e.dead && e.tries < self.cfg.max_tries)
            .count()
    }
    pub fn take_window(&mut self) -> NackWindow {
        core::mem::take(&mut self.win)
    }
    /// The late fills since the last take, and how many the cap dropped.
    pub fn take_late_fills(&mut self) -> (Vec<LateFill>, u64) {
        let d = core::mem::take(&mut self.late_fills_dropped);
        (core::mem::take(&mut self.late_fills), d)
    }

    fn note_late(&mut self, now: u64, late_ms: u32) {
        self.late.push_back((now, late_ms));
        while let Some(&(t, _)) = self.late.front() {
            if now.wrapping_sub(t) > self.cfg.settle_window_ms as u64 {
                self.late.pop_front();
            } else {
                break;
            }
        }
        self.win.late_ms_max = self.win.late_ms_max.max(late_ms);
        if self.late.len() < self.cfg.settle_min_samples as usize {
            return;
        }
        let mx = self.late.iter().map(|&(_, l)| l).max().unwrap_or(0);
        self.settle_ms = (mx as i32 + 2).clamp(self.cfg.settle_min_ms, self.cfg.settle_max_ms);
    }

    fn resolve(&mut self, now: u64, inp: &NackInputs) {
        let seqs: Vec<u32> = self.entries.keys().copied().collect();
        for s in seqs {
            let mut e = self.entries[&s];
            let mut done = true;
            match (inp.state)(s) {
                SourceState::Unknown => {
                    done = false;
                    if now.wrapping_sub(e.first_missing_ms) > inp.gap_timeout_ms {
                        e.dead = true;
                        if e.tries > 0 && !e.deadline_counted {
                            self.stats.dropped_deadline += 1;
                            e.deadline_counted = true;
                        }
                    }
                }
                SourceState::Retx => {
                    if e.tries > 0 {
                        self.stats.filled += 1;
                        // Past the deadline: a poll already booked it dropped, or the fill
                        // landed between the deadline and this poll.
                        if e.deadline_counted
                            || now.wrapping_sub(e.first_missing_ms) > inp.gap_timeout_ms
                        {
                            self.stats.filled_late += 1;
                        }
                        self.win.filled += 1;
                        if self.win.fill_ms.len() < MAX_FILL_SAMPLES {
                            self.win
                                .fill_ms
                                .push(now.wrapping_sub(e.first_sent_ms) as u32);
                        }
                    }
                }
                SourceState::Direct => {
                    if e.tries > 0 {
                        self.stats.late_fill += 1;
                        if self.late_fills.len() < LATE_FILL_CAP {
                            self.late_fills.push(LateFill {
                                t_ms: now,
                                seq: s,
                                from_tail: e.from_tail,
                                tries: e.tries,
                                late_ms: now.wrapping_sub(e.first_missing_ms) as u32,
                                after_req_ms: now.wrapping_sub(e.first_sent_ms) as u32,
                                settle_ms: self.settle_ms,
                            });
                        } else {
                            self.late_fills_dropped += 1;
                        }
                    } else {
                        self.note_late(now, now.wrapping_sub(e.first_missing_ms) as u32);
                    }
                }
                SourceState::Recovered => {
                    if e.tries > 0 {
                        self.stats.wasted += 1;
                    }
                }
                SourceState::BelowFloor => {
                    if e.tries > 0 && !e.deadline_counted {
                        self.stats.dropped_deadline += 1;
                    }
                }
            }
            if done {
                self.entries.remove(&s);
            } else {
                self.entries.insert(s, e);
            }
        }
    }

    fn admit(&mut self, now: u64, inp: &NackInputs) {
        for s in (inp.missing)() {
            self.entries.entry(s).or_insert(Entry {
                first_missing_ms: now,
                ..Entry::default()
            });
        }
        let tail = (inp.tail)().filter(|tv| {
            now >= tv.last_progress_ms + self.settle_ms as u64
                && u32::from(tv.max_idx) + 1 < u32::from(tv.count)
        });
        if let Some(tv) = tail {
            let n_tail = u32::from(tv.count) - u32::from(tv.max_idx) - 1;
            for k in 1..=n_tail {
                let s = tv.seq_at_max.wrapping_add(k);
                if !self.entries.contains_key(&s) && (inp.state)(s) == SourceState::Unknown {
                    self.entries.insert(
                        s,
                        Entry {
                            first_missing_ms: tv.last_progress_ms,
                            from_tail: true,
                            ..Entry::default()
                        },
                    );
                }
            }
        }
    }

    pub fn poll(&mut self, now: u64, inp: &NackInputs) -> Option<Nack> {
        if !self.cfg.enable {
            return None;
        }
        self.resolve(now, inp);
        self.admit(now, inp);
        let mut due = Vec::new();
        for (&s, e) in self.entries.iter_mut() {
            if e.dead || e.tries >= self.cfg.max_tries {
                continue;
            }
            let (since, wait) = if e.tries == 0 {
                (e.first_missing_ms, self.settle_ms as u64)
            } else {
                (e.last_sent_ms, self.cfg.repeat_ms as u64)
            };
            if now < since + wait {
                continue;
            }
            let deadline = e.first_missing_ms + inp.gap_timeout_ms;
            if now + self.cfg.min_lead_ms as u64 > deadline {
                e.dead = true;
                self.stats.lead_skipped += 1;
                continue;
            }
            due.push(s);
        }
        if due.is_empty() {
            return None;
        }
        if (inp.util)() >= inp.down_util {
            self.stats.suppressed += 1;
            for s in &due {
                if let Some(e) = self.entries.get_mut(s) {
                    e.dead = true;
                }
            }
            return None;
        }
        self.counter = self.counter.wrapping_add(1);
        let mut n = Nack {
            counter: self.counter,
            sid: 0,
            ..Nack::default()
        };
        let mut i = 0;
        let mut any_repeat = false;
        while i < due.len() && usize::from(n.n) < MAX_NACK_ENTRIES {
            let mut en = NackEntry {
                first_seq: due[i],
                bitmap: 0,
            };
            while i < due.len() && due[i].wrapping_sub(en.first_seq) < 32 {
                en.bitmap |= 1u32 << due[i].wrapping_sub(en.first_seq);
                let e = self.entries.get_mut(&due[i]).expect("due entry");
                if e.tries == 0 {
                    e.first_sent_ms = now;
                } else {
                    any_repeat = true;
                }
                e.tries += 1;
                e.last_sent_ms = now;
                self.stats.syms_requested += 1;
                if e.from_tail && e.tries == 1 {
                    self.stats.tail_requests += 1;
                }
                i += 1;
            }
            n.e[usize::from(n.n)] = en;
            n.n += 1;
        }
        if any_repeat {
            n.flags |= NACK_FLAG_REPEAT;
            self.stats.repeats += 1;
        }
        self.stats.requests += 1;
        Some(n)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::RefCell;

    #[test]
    fn gap_then_fill_books_filled() {
        let cfg = NackCfg {
            enable: true,
            ..NackCfg::default()
        };
        let mut t = NackTracker::new(cfg);
        let st = RefCell::new(SourceState::Unknown);
        let missing = || vec![10u32, 11, 40];
        let state = |_s: u32| *st.borrow();
        let tail = || None;
        let util = || 0.0;
        let inp = NackInputs {
            missing: &missing,
            state: &state,
            tail: &tail,
            util: &util,
            down_util: 0.35,
            gap_timeout_ms: 50,
        };
        assert!(t.poll(100, &inp).is_none());
        let n = t.poll(112, &inp).expect("nack after settle");
        assert_eq!(n.counter, 1);
        // 40 - 10 = 30 < 32: one entry covers all three seqs.
        assert_eq!(n.n, 1);
        assert_eq!(n.e[0].first_seq, 10);
        assert_eq!(n.e[0].bitmap, 0b11 | (1 << 30));
        let n2 = t.poll(128, &inp).expect("repeat");
        assert_eq!(n2.flags & NACK_FLAG_REPEAT, NACK_FLAG_REPEAT);
        *st.borrow_mut() = SourceState::Retx;
        let none = || Vec::new();
        let inp2 = NackInputs {
            missing: &none,
            ..inp
        };
        assert!(t.poll(130, &inp2).is_none());
        assert_eq!(t.stats().filled, 3);
        assert_eq!(t.take_window().fill_ms, vec![18, 18, 18]);
    }

    #[test]
    fn fill_past_the_deadline_books_filled_late() {
        // gap_timeout 50 from t0 = 1000. 100 fills in time; 101 is seen dropped at 1060 and
        // fills at 1070; 102 fills between the deadline and the next poll, so no poll ever saw
        // it unknown past the deadline.
        let mut t = NackTracker::new(NackCfg {
            enable: true,
            ..NackCfg::default()
        });
        let st = RefCell::new(BTreeMap::from([
            (100u32, SourceState::Unknown),
            (101, SourceState::Unknown),
            (102, SourceState::Unknown),
        ]));
        let missing = || {
            st.borrow()
                .iter()
                .filter(|(_, v)| **v == SourceState::Unknown)
                .map(|(k, _)| *k)
                .collect()
        };
        let state = |s: u32| st.borrow().get(&s).copied().unwrap_or(SourceState::Direct);
        let tail = || None;
        let util = || 0.0;
        let inp = NackInputs {
            missing: &missing,
            state: &state,
            tail: &tail,
            util: &util,
            down_util: 0.35,
            gap_timeout_ms: 50,
        };
        assert!(t.poll(1000, &inp).is_none());
        assert!(t.poll(1012, &inp).is_some());
        st.borrow_mut().insert(100, SourceState::Retx);
        t.poll(1040, &inp);
        assert_eq!((t.stats().filled, t.stats().filled_late), (1, 0));
        t.poll(1049, &inp);
        st.borrow_mut().insert(102, SourceState::Retx);
        t.poll(1060, &inp);
        assert_eq!((t.stats().filled, t.stats().filled_late), (2, 1));
        assert_eq!(t.stats().dropped_deadline, 1);
        st.borrow_mut().insert(101, SourceState::Retx);
        t.poll(1070, &inp);
        assert_eq!((t.stats().filled, t.stats().filled_late), (3, 2));
        assert_eq!(
            t.stats().dropped_deadline,
            1,
            "booked once, not again on the fill"
        );
    }

    #[test]
    fn stop_rule_kills_due_entries() {
        let mut t = NackTracker::new(NackCfg {
            enable: true,
            ..NackCfg::default()
        });
        let missing = || vec![5u32];
        let state = |_s: u32| SourceState::Unknown;
        let tail = || None;
        let util = || 0.5;
        let inp = NackInputs {
            missing: &missing,
            state: &state,
            tail: &tail,
            util: &util,
            down_util: 0.35,
            gap_timeout_ms: 50,
        };
        t.poll(0, &inp);
        assert!(t.poll(20, &inp).is_none());
        assert_eq!(t.stats().suppressed, 1);
        assert_eq!(t.outstanding(), 0);
    }

    /// A requested gap seq and a requested tail seq whose own copies then arrive: two
    /// `late_fill`s, each kept with its kind and timing; the take empties the list.
    #[test]
    fn late_fills_keep_kind_and_timing() {
        let mut t = NackTracker::new(NackCfg {
            enable: true,
            ..NackCfg::default()
        });
        let st = RefCell::new(BTreeMap::from([
            (7u32, SourceState::Unknown),
            (21, SourceState::Unknown),
        ]));
        let gap_open = RefCell::new(true);
        let missing = || {
            if *gap_open.borrow() {
                vec![7u32]
            } else {
                Vec::new()
            }
        };
        let state = |s: u32| *st.borrow().get(&s).unwrap_or(&SourceState::Direct);
        // Frame promises 4 fragments, 2 arrived (max_idx 1 at seq 19): 20 and 21 are the tail;
        // 20 already arrived (Direct), so only 21 is asked.
        let tail = || {
            Some(NackTailView {
                count: 4,
                max_idx: 1,
                seq_at_max: 19,
                last_progress_ms: 995,
            })
        };
        let util = || 0.0;
        let inp = NackInputs {
            missing: &missing,
            state: &state,
            tail: &tail,
            util: &util,
            down_util: 0.35,
            gap_timeout_ms: 80,
        };
        assert!(t.poll(1000, &inp).is_none());
        let n = t
            .poll(1012, &inp)
            .expect("request after the 12 ms settle seed");
        assert_eq!(n.e[0].first_seq, 7);
        *gap_open.borrow_mut() = false;
        st.borrow_mut().insert(7, SourceState::Direct);
        st.borrow_mut().insert(21, SourceState::Direct);
        t.poll(1030, &inp);
        assert_eq!(t.stats().late_fill, 2, "7 and 21");
        let (rows, dropped) = t.take_late_fills();
        assert_eq!(dropped, 0);
        let gap = rows.iter().find(|r| r.seq == 7).expect("gap row");
        assert_eq!(
            (gap.from_tail, gap.tries, gap.late_ms, gap.after_req_ms),
            (false, 1, 30, 18)
        );
        let tl = rows.iter().find(|r| r.seq == 21).expect("tail row");
        assert_eq!((tl.from_tail, tl.late_ms, tl.settle_ms), (true, 35, 12));
        assert!(t.take_late_fills().0.is_empty());
    }
}
