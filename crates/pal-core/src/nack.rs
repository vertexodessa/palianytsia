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

/// How a requested seq ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// Its own first copy arrived (`late_fill`). The drone sends in order with no MAC retries
    /// (No-Ack broadcast), so a gap entry's copy arriving late means the ground reordered it
    /// (two cards' USB batches), and a tail entry's means the drone's queue or the air held it
    /// past the settle.
    Late,
    /// The frame's repairs solved it before the re-sent copy came (`wasted`).
    Wasted,
    /// The re-sent copy filled it (`filled`).
    Filled,
}

/// A requested seq as it resolved, kept for the flight log only: mabur's tracker counts the
/// outcomes and WebGs has no home for the detail. The frame fields say whether waiting for the
/// frame's own repairs would have spared the request: the drone sends a frame's repairs after
/// its last source fragment, so while that fragment has not arrived the repairs are still to
/// come.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Resolved {
    pub t_ms: u64,
    pub seq: u32,
    pub outcome: Outcome,
    pub from_tail: bool,
    pub tries: i32,
    /// From first missing (gap: the decoder saw it missing; tail: the frame's last progress)
    /// to the resolution.
    pub late_ms: u32,
    /// From the first request to the resolution.
    pub after_req_ms: u32,
    pub settle_ms: i32,
    /// At the first request the seq was in the newest frame and that frame's last source
    /// fragment had not arrived.
    pub open: bool,
    /// At the first request, for a seq in the newest frame: its source fragments not heard yet
    /// past the highest one heard, and the seqs of that frame the tracker held as missing, so
    /// together the holes the frame's repairs had to cover so far.
    pub src_left: u16,
    pub holes: u16,
    /// From the first request to the first poll that saw the frame's last source fragment or a
    /// newer frame; `None` when the request found the frame closed or it resolved first.
    pub closed_after_req_ms: Option<u32>,
}

/// Resolved rows kept between two takes. The loop takes them every tick; the cap only bounds a
/// loop that stopped taking.
pub const RESOLVED_CAP: usize = 256;

#[derive(Clone, Copy, Default)]
struct Entry {
    first_missing_ms: u64,
    first_sent_ms: u64,
    last_sent_ms: u64,
    tries: i32,
    from_tail: bool,
    dead: bool,
    deadline_counted: bool,
    /// Flight-log detail of the first request (`Resolved`); no decision reads it.
    at_req: FrameAtReq,
}

#[derive(Clone, Copy, Default)]
struct FrameAtReq {
    open: bool,
    frame_first: u32,
    src_left: u16,
    holes: u16,
    closed_ms: Option<u64>,
}

/// The first seq of the newest frame and its fragment count.
fn frame_of(tv: &NackTailView) -> (u32, u16) {
    (tv.seq_at_max.wrapping_sub(u32::from(tv.max_idx)), tv.count)
}

fn in_frame(s: u32, (first, count): (u32, u16)) -> bool {
    s.wrapping_sub(first) < u32::from(count)
}

pub struct NackTracker {
    cfg: NackCfg,
    entries: BTreeMap<u32, Entry>,
    late: VecDeque<(u64, u32)>,
    settle_ms: i32,
    counter: u32,
    stats: NackStats,
    win: NackWindow,
    resolved: Vec<Resolved>,
    resolved_dropped: u64,
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
            resolved: Vec::new(),
            resolved_dropped: 0,
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
    /// The requested seqs resolved since the last take, and how many the cap dropped.
    pub fn take_resolved(&mut self) -> (Vec<Resolved>, u64) {
        let d = core::mem::take(&mut self.resolved_dropped);
        (core::mem::take(&mut self.resolved), d)
    }

    fn note_resolved(&mut self, now: u64, s: u32, e: &Entry, outcome: Outcome) {
        if self.resolved.len() >= RESOLVED_CAP {
            self.resolved_dropped += 1;
            return;
        }
        let a = e.at_req;
        self.resolved.push(Resolved {
            t_ms: now,
            seq: s,
            outcome,
            from_tail: e.from_tail,
            tries: e.tries,
            late_ms: now.wrapping_sub(e.first_missing_ms) as u32,
            after_req_ms: now.wrapping_sub(e.first_sent_ms) as u32,
            settle_ms: self.settle_ms,
            open: a.open,
            src_left: a.src_left,
            holes: a.holes,
            closed_after_req_ms: a.closed_ms.map(|c| c.wrapping_sub(e.first_sent_ms) as u32),
        });
    }

    /// Flight-log detail only: marks the requested entries whose frame has closed since their
    /// first request (its last source fragment heard, or a newer frame begun).
    fn note_frame_close(&mut self, now: u64, tv: Option<NackTailView>) {
        let cur = tv.map(|tv| {
            (
                frame_of(&tv),
                u32::from(tv.max_idx) + 1 >= u32::from(tv.count),
            )
        });
        for e in self.entries.values_mut() {
            let a = &mut e.at_req;
            if e.tries == 0 || !a.open || a.closed_ms.is_some() {
                continue;
            }
            let closed = match cur {
                Some((f, done)) => f.0 != a.frame_first || done,
                None => true,
            };
            if closed {
                a.closed_ms = Some(now);
            }
        }
    }

    /// Flight-log detail only: the newest frame's state for a seq requested for the first time.
    fn frame_at_req(&self, s: u32, tv: Option<NackTailView>) -> FrameAtReq {
        let Some(tv) = tv else {
            return FrameAtReq::default();
        };
        let f = frame_of(&tv);
        if !in_frame(s, f) {
            return FrameAtReq::default();
        }
        let src_left = tv.count.saturating_sub(tv.max_idx + 1);
        let holes = self
            .entries
            .keys()
            .filter(|&&k| in_frame(k, f) && k.wrapping_sub(f.0) <= u32::from(tv.max_idx))
            .count() as u16;
        FrameAtReq {
            open: src_left > 0,
            frame_first: f.0,
            src_left,
            holes,
            closed_ms: None,
        }
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
                        self.note_resolved(now, s, &e, Outcome::Filled);
                    }
                }
                SourceState::Direct => {
                    if e.tries > 0 {
                        self.stats.late_fill += 1;
                        self.note_resolved(now, s, &e, Outcome::Late);
                    } else {
                        self.note_late(now, now.wrapping_sub(e.first_missing_ms) as u32);
                    }
                }
                SourceState::Recovered => {
                    if e.tries > 0 {
                        self.stats.wasted += 1;
                        self.note_resolved(now, s, &e, Outcome::Wasted);
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
        let tv = (inp.tail)();
        self.note_frame_close(now, tv);
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
                let at_req = if self.entries[&due[i]].tries == 0 {
                    self.frame_at_req(due[i], tv)
                } else {
                    FrameAtReq::default()
                };
                let e = self.entries.get_mut(&due[i]).expect("due entry");
                if e.tries == 0 {
                    e.first_sent_ms = now;
                    e.at_req = at_req;
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
        let (rows, dropped) = t.take_resolved();
        assert_eq!(dropped, 0);
        let gap = rows.iter().find(|r| r.seq == 7).expect("gap row");
        assert_eq!(
            (
                gap.outcome,
                gap.from_tail,
                gap.tries,
                gap.late_ms,
                gap.after_req_ms
            ),
            (Outcome::Late, false, 1, 30, 18)
        );
        let tl = rows.iter().find(|r| r.seq == 21).expect("tail row");
        assert_eq!((tl.from_tail, tl.late_ms, tl.settle_ms), (true, 35, 12));
        assert!(t.take_resolved().0.is_empty());
    }

    /// The frame detail of a request: a hole asked while its frame's last source had not
    /// arrived is `open`, with the holes and the sources still to come; the poll that sees the
    /// last source closes it, and the repair that then solves it books `Wasted` with the time
    /// from the request to the close. A hole asked after its frame closed is not `open`.
    #[test]
    fn resolved_rows_say_whether_the_frame_was_still_open() {
        let mut t = NackTracker::new(NackCfg {
            enable: true,
            ..NackCfg::default()
        });
        // Frame of 10 fragments at seqs 100..=109; 102 and 104 lost.
        let st = RefCell::new(BTreeMap::from([
            (102u32, SourceState::Unknown),
            (104, SourceState::Unknown),
        ]));
        let max_idx = RefCell::new(5u16);
        let missing = || {
            st.borrow()
                .iter()
                .filter(|(_, v)| **v == SourceState::Unknown)
                .map(|(k, _)| *k)
                .collect()
        };
        let state = |s: u32| st.borrow().get(&s).copied().unwrap_or(SourceState::Direct);
        let tail = || {
            let m = *max_idx.borrow();
            Some(NackTailView {
                count: 10,
                max_idx: m,
                seq_at_max: 100 + u32::from(m),
                last_progress_ms: 1000,
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
        t.poll(1000, &inp);
        assert!(t.poll(1012, &inp).is_some(), "both asked at the settle");
        *max_idx.borrow_mut() = 9;
        t.poll(1015, &inp);
        st.borrow_mut().insert(102, SourceState::Recovered);
        st.borrow_mut().insert(104, SourceState::Retx);
        t.poll(1020, &inp);
        let (rows, _) = t.take_resolved();
        let w = rows.iter().find(|r| r.seq == 102).expect("102");
        assert_eq!(
            (
                w.outcome,
                w.open,
                w.src_left,
                w.holes,
                w.closed_after_req_ms
            ),
            (Outcome::Wasted, true, 4, 2, Some(3))
        );
        let f = rows.iter().find(|r| r.seq == 104).expect("104");
        assert_eq!((f.outcome, f.after_req_ms), (Outcome::Filled, 8));

        // A hole in the same frame, now closed: asked, not open, no close time.
        st.borrow_mut().insert(107, SourceState::Unknown);
        t.poll(1030, &inp);
        t.poll(1042, &inp).expect("107 asked");
        st.borrow_mut().insert(107, SourceState::Retx);
        t.poll(1050, &inp);
        let (rows, _) = t.take_resolved();
        assert_eq!(
            (rows[0].seq, rows[0].open, rows[0].closed_after_req_ms),
            (107, false, None)
        );
    }
}
