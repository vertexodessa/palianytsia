//! mabur `channel_scout.{h,cpp}`: the GS's long-lived search + measure
//! scheduler on the spare card. has_work = search || (measure && !frozen).
//!
//! - search (link down): sweep the set, bursting DISC (`beaconing()`) for
//!   search_ms on every member dwell so the drone is found wherever it is.
//! - measure (auto mode, pick open): observe every dwell channel (both
//!   20 MHz halves of each member's pair at width 40) and rank it.
//!   Unlinked, the observe is `quiet()` (the TX card holds its DISC);
//!   linked, the TX card's leak is subtracted instead, and op's own halves
//!   are skipped (they carry the drone's video).
//! - no work: park on op at the link width, once.
//!
//! One card + measure: a silent prelude of one_card_ms first; afterwards
//! every step is an op window then one dwell. The clock and sleep are
//! injected so tests run instantly. mabur's one-round mode (calibration's
//! channel pick) is not ported.

use std::sync::atomic::{AtomicBool, AtomicU64, AtomicU8, Ordering::*};
use std::sync::Mutex;

use super::pair::{all_pairs_ranked, any_pair_ranked, pair_proposal, pair_ranking, scan_half_set};
use super::radio::{nhm_busy_pct, nhm_period_4us, NhmBusyRead, ScoutRadio};
use super::ranker::{ChannelRanker, RankEntry, RankSample};
use crate::config::ht40_pair_other;

#[derive(Clone, Debug, PartialEq)]
pub struct ScoutCfg {
    /// The set, config order (primaries at width 40).
    pub channels: Vec<u8>,
    /// The members a pick may name (`radio.use`); empty = all of
    /// `channels`. The search and the rounds still cover every member.
    pub usable: Vec<u8>,
    /// Auto: observe + rank; false (pinned): search only.
    pub measure: bool,
    pub dwell_ms: i32,
    pub settle_ms: i32,
    pub min_rounds: i32,
    pub search_ms: i32,
    /// One card: the beacon window on op between dwells.
    pub op_window_ms: i32,
    pub beacon_period_ms: i32,
    /// One card + measure: the silent prelude before the first DISC.
    pub one_card_ms: i32,
    pub pick_margin: u32,
    pub one_card: bool,
    /// 40: dwell every half at 20, pick a pair, park at 40.
    pub link_width_mhz: u8,
    pub busy_dbm: i32,
    pub blocked_pct: f64,
    /// RankSample::leak per TX-card frame sent during an observe.
    pub leak_per_frame: f64,
}

impl Default for ScoutCfg {
    fn default() -> Self {
        Self {
            channels: Vec::new(),
            usable: Vec::new(),
            measure: true,
            dwell_ms: 250,
            settle_ms: 30,
            min_rounds: 3,
            search_ms: 100,
            op_window_ms: 300,
            beacon_period_ms: 20,
            one_card_ms: 5000,
            pick_margin: 20,
            one_card: false,
            link_width_mhz: 20,
            busy_dbm: -83,
            blocked_pct: 1e9,
            leak_per_frame: 1.0,
        }
    }
}

pub const FLAG_RETUNE_FAILED: u32 = 1 << 1;
pub const FLAG_READ_FAILED: u32 = 1 << 2;
pub const FLAG_NHM_MISSING: u32 = 1 << 4;

/// One dwell (devourer `chanmig::SurveyDwell`, the fields the scout fills,
/// plus mabur's ScoutDwell extras).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ScoutDwell {
    pub seq: u64,
    pub ch: u8,
    /// What the card was tuned at: 20, or 40 with `offset` (the in-flight
    /// dwell keeps the card's tuning).
    pub width_mhz: u8,
    pub offset: u8,
    pub round: u64,
    pub t_start_ms: i64,
    pub t_end_ms: i64,
    pub retune_us: i64,
    pub settle_ms: i32,
    pub observe_ms: i64,
    pub valid_fa: bool,
    pub fa_ofdm: u32,
    pub cca_ofdm: u32,
    pub valid_igi: bool,
    pub igi: u8,
    pub valid_nhm: bool,
    pub flags: u32,
    pub dvr_frames: u32,
    pub frames: u32,
    pub floor_valid: bool,
    pub floor_dbm: i8,
    /// In-flight provenance (inflight_scout fills these).
    pub in_session: bool,
    pub to_us: i64,
    pub read_us: i64,
    pub back_us: i64,
    pub busy_valid: bool,
    pub busy_pct: f64,
}

/// devourer `chanmig::ScanScheduler` with the scout's flat cadence (every
/// revisit and the fail retry 0, no wide dwells): the most-overdue bin,
/// ties by plan order, so a plan-order round-robin; a round counts once
/// every bin has had a good dwell.
#[derive(Clone, Debug)]
struct FlatScheduler {
    bins: Vec<u8>,
    deadline: Vec<i64>,
    covered: Vec<bool>,
    covered_n: usize,
    rounds: u64,
}

impl FlatScheduler {
    fn new(channels: &[u8]) -> Self {
        let mut bins: Vec<u8> = Vec::new();
        for &c in channels {
            if !bins.contains(&c) {
                bins.push(c);
            }
        }
        let n = bins.len();
        Self {
            bins,
            deadline: vec![i64::MIN; n],
            covered: vec![false; n],
            covered_n: 0,
            rounds: 0,
        }
    }
    /// (bin index, channel, round)
    fn next(&self) -> Option<(usize, u8, u64)> {
        let mut best = 0;
        for i in 1..self.bins.len() {
            if self.deadline[i] < self.deadline[best] {
                best = i;
            }
        }
        (!self.bins.is_empty()).then(|| (best, self.bins[best], self.rounds))
    }
    fn complete(&mut self, i: usize, now_ms: i64, ok: bool) {
        self.deadline[i] = now_ms;
        if ok && !self.covered[i] {
            self.covered[i] = true;
            self.covered_n += 1;
            if self.covered_n == self.bins.len() {
                self.rounds += 1;
                self.covered_n = 0;
                self.covered.iter_mut().for_each(|c| *c = false);
            }
        }
    }
}

pub type NowFn = Box<dyn Fn() -> i64 + Send + Sync>;
pub type SleepFn = Box<dyn Fn(i32) + Send + Sync>;

struct Shared {
    ranker: ChannelRanker,
    dwells: Vec<ScoutDwell>,
    /// Dwells taken this round (bounds a dead half).
    round_dwells: usize,
    round_samples: Vec<RankSample>,
}

/// The one-round state (a calibration run's channel pick): measuring ->
/// measured (no work, not yet parked) -> idle once parked. One atomic
/// moved by compare-exchange, so a begin_round racing the run thread's
/// park is never lost.
const ROUND_IDLE: u8 = 0;
const ROUND_MEASURING: u8 = 1;
const ROUND_MEASURED: u8 = 2;

/// Run-thread state (mabur keeps it unlocked; the lock only makes `&self`
/// sound and is never contended).
struct RunState {
    sched: FlatScheduler,
    seq: u64,
    /// The next tune switches the card to 20 MHz.
    need_width: bool,
    /// First run_once() (the one-card prelude start).
    t0: Option<i64>,
}

pub struct ChannelScout<R: ScoutRadio + ?Sized> {
    cfg: ScoutCfg,
    /// `cfg.usable` as the operator last set it (`set_usable`). Taken
    /// after `sh`, never held while taking it.
    usable: Mutex<Vec<u8>>,
    radio: std::sync::Arc<R>,
    now: NowFn,
    sleep: SleepFn,
    sh: Mutex<Shared>,
    run: Mutex<RunState>,
    proposal: AtomicU8,
    op: AtomicU8,
    tx_frames: AtomicU64,
    search: AtomicBool,
    frozen: AtomicBool,
    stop: AtomicBool,
    working: AtomicBool,
    beaconing: AtomicBool,
    quiet: AtomicBool,
    prelude_done: AtomicBool,
    prelude_ack: AtomicBool,
    done: AtomicBool,
    rounds: AtomicU64,
    round: AtomicU8,
}

/// Pinned or width 20: the members; measuring at 40: both halves of every
/// member's pair. Also the ranker's row order.
fn dwell_channels(c: &ScoutCfg) -> Vec<u8> {
    if c.measure && c.link_width_mhz == 40 {
        scan_half_set(&c.channels)
    } else {
        c.channels.clone()
    }
}

impl<R: ScoutRadio + ?Sized> ChannelScout<R> {
    pub fn new(cfg: ScoutCfg, radio: std::sync::Arc<R>, now: NowFn, sleep: SleepFn) -> Self {
        let dc = dwell_channels(&cfg);
        // The first pick: the first usable member (radio.use narrows it).
        let first = cfg
            .usable
            .first()
            .or(cfg.channels.first())
            .copied()
            .unwrap_or(0);
        let prelude = cfg.one_card && cfg.measure;
        Self {
            usable: Mutex::new(cfg.usable.clone()),
            sh: Mutex::new(Shared {
                ranker: ChannelRanker::new(&dc, cfg.min_rounds, cfg.pick_margin, cfg.blocked_pct),
                dwells: Vec::new(),
                round_dwells: 0,
                round_samples: Vec::new(),
            }),
            run: Mutex::new(RunState {
                sched: FlatScheduler::new(&dc),
                seq: 0,
                need_width: true,
                t0: None,
            }),
            proposal: AtomicU8::new(first),
            op: AtomicU8::new(first),
            tx_frames: AtomicU64::new(0),
            search: AtomicBool::new(false),
            frozen: AtomicBool::new(false),
            stop: AtomicBool::new(false),
            working: AtomicBool::new(false),
            beaconing: AtomicBool::new(false),
            quiet: AtomicBool::new(false),
            prelude_done: AtomicBool::new(!prelude),
            prelude_ack: AtomicBool::new(!prelude),
            done: AtomicBool::new(false),
            rounds: AtomicU64::new(0),
            round: AtomicU8::new(ROUND_IDLE),
            cfg,
            radio,
            now,
            sleep,
        }
    }

    // ---- core -> scout ----

    pub fn set_op(&self, ch: u8) {
        self.op.store(ch, Release);
        self.publish(); // the margin is against op
    }
    pub fn set_search(&self, on: bool) {
        self.search.store(on, Release);
    }
    pub fn set_tx_frames(&self, cumulative: u64) {
        self.tx_frames.store(cumulative, Release);
    }
    /// The pick is closed: measuring stops for good.
    pub fn freeze(&self) {
        self.frozen.store(true, Release);
    }
    pub fn stop(&self) {
        self.stop.store(true, Release);
    }
    /// One card + measure: the core committed the prelude pick (op = the
    /// committed channel); op windows may run.
    pub fn ack_prelude(&self, op: u8) {
        self.set_op(op);
        self.prelude_ack.store(true, Release);
    }

    // ---- scout -> core ----

    pub fn working(&self) -> bool {
        self.working.load(Acquire)
    }
    pub fn beaconing(&self) -> bool {
        self.beaconing.load(Acquire)
    }
    pub fn quiet(&self) -> bool {
        self.quiet.load(Acquire)
    }
    pub fn pick_open(&self) -> bool {
        !self.frozen.load(Acquire)
    }
    pub fn prelude_done(&self) -> bool {
        self.prelude_done.load(Acquire)
    }
    /// Stale after freeze(): holds its last value.
    pub fn proposal(&self) -> u8 {
        self.proposal.load(Acquire)
    }
    pub fn rounds(&self) -> u64 {
        self.rounds.load(Acquire)
    }
    pub fn done(&self) -> bool {
        self.done.load(Acquire)
    }
    pub fn ranking(&self) -> Vec<RankEntry> {
        self.sh.lock().unwrap().ranker.all().to_vec()
    }
    pub fn take_dwells(&self) -> Vec<ScoutDwell> {
        std::mem::take(&mut self.sh.lock().unwrap().dwells)
    }

    fn has_work(&self) -> bool {
        self.search.load(Acquire) || self.measuring()
    }
    fn measuring(&self) -> bool {
        self.cfg.measure && (!self.frozen.load(Acquire) || self.round_measuring())
    }

    // ---- one measure round (a calibration run's channel pick) ----

    /// One measure round now, the pick already frozen and the scout parked
    /// (the core checks both). A fresh ranker: the round ranks only what it
    /// measures. round_open() reads true from here until the scout has
    /// parked again, so the core never touches the card in between.
    pub fn begin_round(&self) {
        {
            let mut sh = self.sh.lock().unwrap();
            sh.ranker = ChannelRanker::new(
                &dwell_channels(&self.cfg),
                self.cfg.min_rounds,
                self.cfg.pick_margin,
                self.cfg.blocked_pct,
            );
            sh.round_dwells = 0;
            sh.round_samples.clear();
        }
        self.round.store(ROUND_MEASURING, Release);
    }

    /// Ends a round early: no more measuring; the run thread parks the card
    /// on its next step. force_idle: the run thread is not running, so
    /// nothing will park: release the card at once.
    pub fn cancel_round(&self, force_idle: bool) {
        self.end_round_measuring();
        if force_idle {
            let _ = self
                .round
                .compare_exchange(ROUND_MEASURED, ROUND_IDLE, AcqRel, Acquire);
        }
    }

    pub fn round_measuring(&self) -> bool {
        self.round.load(Acquire) == ROUND_MEASURING
    }
    pub fn round_open(&self) -> bool {
        self.round.load(Acquire) != ROUND_IDLE
    }
    /// Every observed sample of the current (or last) round, in dwell order.
    pub fn round_samples(&self) -> Vec<RankSample> {
        self.sh.lock().unwrap().round_samples.clone()
    }

    fn end_round_measuring(&self) {
        let _ = self
            .round
            .compare_exchange(ROUND_MEASURING, ROUND_MEASURED, AcqRel, Acquire);
    }

    /// The round is over once every dwell channel it does not skip has a
    /// visit, or after two passes' worth of dwells (a half whose retune or
    /// read keeps failing must not hold the run's pick forever).
    fn note_round_dwell(&self) {
        if !self.round_measuring() {
            return;
        }
        let cap = 2 * dwell_channels(&self.cfg).len();
        let (all, taken) = {
            let mut sh = self.sh.lock().unwrap();
            sh.round_dwells += 1;
            let all = sh
                .ranker
                .all()
                .iter()
                .all(|e| self.skip_op_half(e.ch) || e.visits != 0);
            (all, sh.round_dwells)
        };
        if all || taken >= cap {
            self.end_round_measuring();
        }
    }
    /// The members a pick may name; the whole set unless narrowed.
    fn usable(&self) -> Vec<u8> {
        let u = self.usable.lock().unwrap();
        if u.is_empty() {
            self.cfg.channels.clone()
        } else {
            u.clone()
        }
    }

    /// The operator's new `radio.use` (empty = all); the proposal follows
    /// at once unless the pick is frozen.
    pub fn set_usable(&self, usable: Vec<u8>) {
        *self.usable.lock().unwrap() = usable;
        self.publish();
    }

    fn member(&self, ch: u8) -> bool {
        ch != 0 && self.cfg.channels.contains(&ch)
    }

    /// One card: once the prelude is done the pick may run on one visit
    /// per half until min_rounds full passes exist. When that flips back
    /// mature()/proposal() can step back once: the core latches.
    fn eff_min_rounds(&self) -> i32 {
        if self.cfg.one_card
            && self.prelude_done.load(Acquire)
            && self.rounds.load(Acquire) < self.cfg.min_rounds.max(0) as u64
        {
            1
        } else {
            self.cfg.min_rounds
        }
    }

    fn park(&self, rs: &mut RunState) {
        let op = self.op.load(Acquire);
        if self.cfg.link_width_mhz == 40 {
            // Not retried here: the core loop's width resync fixes the card
            // once it is ready again.
            if !self.radio.retune_width(op, 40) {
                eprintln!("pal chan: scout card width switch to 40 MHz on ch {op} failed; the core loop retries");
            }
        } else {
            self.radio.retune(op);
        }
        self.beaconing.store(false, Release);
        self.quiet.store(false, Release);
        self.working.store(false, Release);
        rs.need_width = true;
    }

    fn tune(&self, rs: &mut RunState, ch: u8) -> bool {
        let ok = if rs.need_width {
            self.radio.retune_width(ch, 20)
        } else {
            self.radio.retune(ch)
        };
        if ok {
            rs.need_width = false;
        }
        ok
    }

    /// Thread body: loops run_once() until stop(), parks the card on exit.
    pub fn run(&self) {
        while !self.stop.load(Acquire) {
            if !self.run_once() {
                (self.sleep)(self.cfg.beacon_period_ms);
            }
        }
        if self.working.load(Acquire) {
            self.park(&mut self.run.lock().unwrap());
        }
        self.done.store(true, Release);
    }

    /// While the search is off op's own pair carries the drone's video,
    /// which a 20 MHz observe at width 40 counts as busy and NHM counts
    /// either way: those halves are skipped and op keeps its pre-link
    /// visits.
    fn is_op_half(&self, ch: u8) -> bool {
        let op = self.op.load(Acquire);
        ch == op || (self.cfg.measure && self.cfg.link_width_mhz == 40 && ch == ht40_pair_other(op))
    }
    fn skip_op_half(&self, ch: u8) -> bool {
        !self.search.load(Acquire) && self.measuring() && self.is_op_half(ch)
    }

    /// One scheduler dwell. A skipped op half completes in the scheduler
    /// without touching the card and the next bin is taken; false when
    /// every bin is one of op's.
    fn step_dwell(&self, rs: &mut RunState, burst_ok: bool, observe: bool) -> bool {
        let bins = rs.sched.bins.len();
        for _ in 0..=bins {
            let Some((i, ch, round)) = rs.sched.next() else {
                return true;
            };
            if self.skip_op_half(ch) {
                // Stamped one dwell ahead, or a tie with the dwell that just
                // finished could win by plan order forever.
                rs.sched
                    .complete(i, (self.now)() + self.cfg.dwell_ms.max(1) as i64, true);
                self.rounds.store(rs.sched.rounds, Release);
                continue;
            }
            let burst = burst_ok && self.search.load(Acquire) && self.member(ch);
            let ok = self.dwell(rs, ch, round, burst, observe);
            rs.sched.complete(i, (self.now)(), ok);
            self.rounds.store(rs.sched.rounds, Release);
            if observe {
                self.note_round_dwell();
            }
            return true;
        }
        // Every bin is one of op's (a one-member set): nothing to visit.
        self.end_round_measuring();
        false
    }

    /// One scheduler step; false = nothing to do (idle, parked).
    pub fn run_once(&self) -> bool {
        let mut rs = self.run.lock().unwrap();
        let t0 = *rs.t0.get_or_insert_with(|| (self.now)());
        if !self.has_work() {
            if self.working.load(Acquire) {
                self.park(&mut rs);
            }
            // A round that just ended is over once parked.
            let _ = self
                .round
                .compare_exchange(ROUND_MEASURED, ROUND_IDLE, AcqRel, Acquire);
            return false;
        }
        self.working.store(true, Release);
        if self.cfg.one_card {
            if !self.prelude_done.load(Acquire) {
                if self.measuring() && (self.now)() - t0 < self.cfg.one_card_ms as i64 {
                    return self.step_dwell(&mut rs, false, true); // silent prelude
                }
                self.prelude_done.store(true, Release); // never re-arms
                self.publish();
                return true; // the core reads the ranking, commits, ack_prelude()s
            }
            // No op window before the core commits the prelude pick: DISCs
            // on the old op could be acked there and override the pick.
            if self.measuring() && !self.prelude_ack.load(Acquire) {
                return false;
            }
            let op = self.op.load(Acquire);
            if !self.tune(&mut rs, op) {
                return true;
            }
            self.beaconing.store(true, Release);
            (self.sleep)(self.cfg.op_window_ms);
            self.beaconing.store(false, Release);
            (self.sleep)(self.cfg.beacon_period_ms);
            if !self.has_work() {
                return true;
            }
        }
        let observe = self.measuring();
        self.step_dwell(&mut rs, true, observe)
    }

    fn push_dwell(&self, d: ScoutDwell) {
        self.sh.lock().unwrap().dwells.push(d);
    }

    /// Retune, settle, [burst + gap,] [discard read, observe dwell_ms,
    /// read]; records the dwell and, observed, the sample.
    fn dwell(&self, rs: &mut RunState, ch: u8, round: u64, burst: bool, observe: bool) -> bool {
        let mut d = ScoutDwell {
            seq: rs.seq,
            ch,
            width_mhz: 20,
            round,
            t_start_ms: (self.now)(),
            settle_ms: self.cfg.settle_ms,
            ..Default::default()
        };
        rs.seq += 1;
        if !self.tune(rs, ch) {
            d.flags |= FLAG_RETUNE_FAILED;
            d.t_end_ms = (self.now)();
            self.push_dwell(d);
            return false;
        }
        d.retune_us = ((self.now)() - d.t_start_ms) * 1000;
        (self.sleep)(self.cfg.settle_ms);
        if burst {
            // DISC on this member, then a gap so the last ack lands before
            // the quiet observe.
            self.beaconing.store(true, Release);
            (self.sleep)(self.cfg.search_ms);
            self.beaconing.store(false, Release);
            (self.sleep)(self.cfg.beacon_period_ms);
        }
        if !observe {
            d.t_end_ms = (self.now)();
            self.push_dwell(d);
            return true;
        }
        if self.search.load(Acquire) {
            self.quiet.store(true, Release);
        }
        // Discard barrier: zero the delta counters, let the USB pipe drain.
        let _ = self.radio.read_energy(false);
        let period = nhm_period_4us(self.cfg.dwell_ms);
        let armed = self.radio.arm_nhm_busy(period);
        let f0 = self.radio.frames();
        let tx0 = self.tx_frames.load(Acquire);
        let t0 = (self.now)();
        (self.sleep)(self.cfg.dwell_ms);
        let nb = if armed {
            self.radio.read_nhm_busy()
        } else {
            NhmBusyRead::default()
        };
        let busy = if nb.valid && nb.period == period {
            nhm_busy_pct(&nb, self.cfg.busy_dbm)
        } else {
            None
        };
        d.busy_valid = busy.is_some();
        d.busy_pct = busy.unwrap_or(0.0);
        let e = self.radio.read_energy(true);
        let f1 = self.radio.frames();
        let tx1 = self.tx_frames.load(Acquire);
        self.quiet.store(false, Release);
        d.observe_ms = (self.now)() - t0;
        d.t_end_ms = (self.now)();
        d.valid_fa = e.fa_valid;
        d.fa_ofdm = e.fa_ofdm;
        d.cca_ofdm = e.cca_ofdm;
        d.valid_igi = e.igi_valid;
        d.igi = e.igi;
        d.valid_nhm = e.nhm_valid;
        if !e.nhm_valid {
            d.flags |= FLAG_NHM_MISSING;
        }
        if !e.fa_valid {
            d.flags |= FLAG_READ_FAILED;
        }
        d.dvr_frames = f1.own.wrapping_sub(f0.own) as u32;
        d.frames = d
            .dvr_frames
            .wrapping_add(f1.foreign.wrapping_sub(f0.foreign) as u32);
        d.floor_valid = e.floor_valid;
        d.floor_dbm = e.floor_dbm;
        // A TX counter that went back is no leak.
        let dtx = tx1.saturating_sub(tx0);
        let rs_ = RankSample {
            ch,
            cca: e.cca_ofdm,
            fa: e.fa_ofdm,
            own: d.dvr_frames,
            foreign: d.frames.wrapping_sub(d.dvr_frames),
            leak: (self.cfg.leak_per_frame * dtx as f64).round() as u32,
            floor_valid: e.floor_valid,
            floor_dbm: e.floor_dbm,
            busy_valid: d.busy_valid,
            busy_pct: d.busy_pct,
        };
        {
            let mut sh = self.sh.lock().unwrap();
            if e.fa_valid {
                sh.ranker.add(&rs_);
                if self.round_measuring() {
                    sh.round_samples.push(rs_);
                }
            }
            sh.dwells.push(d);
        }
        self.publish();
        e.fa_valid
    }

    fn publish(&self) {
        if self.frozen.load(Acquire) {
            return;
        }
        // Lock before reading op: a concurrent set_op must not be
        // overwritten by a proposal computed against a stale op.
        let sh = self.sh.lock().unwrap();
        let op = self.op.load(Acquire);
        let mr = self.eff_min_rounds();
        let p = if self.cfg.link_width_mhz == 40 {
            pair_proposal(
                sh.ranker.all(),
                op,
                &self.usable(),
                mr,
                self.cfg.pick_margin,
                self.cfg.blocked_pct,
            )
        } else {
            sh.ranker.proposal_among(op, mr, &self.usable())
        };
        // Nothing usable ranked yet: the proposal stays op, which may be
        // outside radio.use (the drone's channel). A pick must never name
        // one: op if it is usable, else the first usable member.
        let usable = self.usable();
        let p = if usable.contains(&p) {
            p
        } else if usable.contains(&op) {
            op
        } else {
            usable[0]
        };
        self.proposal.store(p, Release);
    }

    fn op_ranked_locked(&self, sh: &Shared, mr: i32) -> bool {
        let op = self.op.load(Acquire);
        let all = sh.ranker.all();
        if self.cfg.link_width_mhz == 40 {
            return any_pair_ranked(all, &[op], mr);
        }
        all.iter()
            .find(|e| e.ch == op)
            .is_some_and(|e| e.visits as i64 >= mr as i64)
    }

    /// Op's pair (width 40) or op (width 20) has the effective min_rounds.
    pub fn op_ranked(&self) -> bool {
        let sh = self.sh.lock().unwrap();
        self.op_ranked_locked(&sh, self.eff_min_rounds())
    }

    /// Every member but op ranked (a linked scout never visits op's pair);
    /// while searching op too, or an unlinked commit could compare against
    /// an op not reached yet.
    pub fn mature(&self) -> bool {
        let sh = self.sh.lock().unwrap();
        let mr = self.eff_min_rounds();
        let op = self.op.load(Acquire);
        let all = sh.ranker.all();
        if all.is_empty() {
            return false;
        }
        if self.cfg.link_width_mhz == 40 {
            let others: Vec<u8> = self
                .cfg
                .channels
                .iter()
                .copied()
                .filter(|&c| c != op)
                .collect();
            if !others.is_empty() && !all_pairs_ranked(all, &others, mr) {
                return false;
            }
        } else if all
            .iter()
            .any(|e| e.ch != op && (e.visits as i64) < mr as i64)
        {
            return false;
        }
        !self.search.load(Acquire) || self.op_ranked_locked(&sh, mr)
    }

    pub fn pick_ranking(&self) -> Vec<u8> {
        let sh = self.sh.lock().unwrap();
        let mr = self.eff_min_rounds();
        if self.cfg.link_width_mhz == 40 {
            return pair_ranking(sh.ranker.all(), &self.usable(), mr, self.cfg.blocked_pct);
        }
        let usable = self.usable();
        sh.ranker
            .ranked_at(mr)
            .into_iter()
            .map(|e| e.ch)
            .filter(|c| usable.contains(c))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::super::radio::{ScoutEnergy, ScoutFrames};
    use super::*;
    use std::sync::{Arc, OnceLock, Weak};

    #[derive(Default)]
    struct FakeState {
        now: i64,
        calls: Vec<String>,
        ch: u8,
        width: u8,
        cca_per_ms_on: Vec<u32>,
        last_read: i64,
    }

    /// Fake radio + clock: sleep advances `now`, every call is recorded.
    struct FakeRadio(Mutex<FakeState>);

    impl FakeRadio {
        fn new() -> Arc<Self> {
            Arc::new(Self(Mutex::new(FakeState {
                width: 20,
                cca_per_ms_on: vec![0; 256],
                ..Default::default()
            })))
        }
        fn st(&self) -> std::sync::MutexGuard<'_, FakeState> {
            self.0.lock().unwrap()
        }
        fn calls(&self) -> Vec<String> {
            self.st().calls.clone()
        }
        fn now(&self) -> i64 {
            self.st().now
        }
        fn busy(&self, ch: u8, per_ms: u32) {
            self.st().cca_per_ms_on[ch as usize] = per_ms;
        }
    }

    impl ScoutRadio for FakeRadio {
        fn retune(&self, c: u8) -> bool {
            let mut s = self.st();
            s.ch = c;
            s.calls.push(format!("retune {c}"));
            true
        }
        fn retune_width(&self, c: u8, w: u8) -> bool {
            let mut s = self.st();
            s.ch = c;
            s.width = w;
            s.calls.push(format!("retune_width {c}/{w}"));
            true
        }
        fn read_energy(&self, with_nhm: bool) -> ScoutEnergy {
            let mut s = self.st();
            s.calls
                .push(if with_nhm { "read+nhm" } else { "read" }.into());
            let mut e = ScoutEnergy {
                fa_valid: true,
                cca_ofdm: ((s.now - s.last_read) as u32) * s.cca_per_ms_on[s.ch as usize],
                ..Default::default()
            };
            s.last_read = s.now;
            if with_nhm {
                e.floor_valid = true;
                e.floor_dbm = -95;
            }
            e
        }
        fn read_energy_scout(&self) -> ScoutEnergy {
            ScoutEnergy::default()
        }
        fn frames(&self) -> ScoutFrames {
            ScoutFrames::default()
        }
    }

    type Scout = ChannelScout<FakeRadio>;
    type Hook = Box<dyn Fn(&Scout, i32) + Send + Sync>;

    /// The scout plus the (flags, ms) of every sleep: B beaconing, Q quiet,
    /// W working. `hook` runs inside each sleep before the clock advances.
    struct Rig {
        r: Arc<FakeRadio>,
        s: Arc<Scout>,
        phases: Arc<Mutex<Vec<(String, i32)>>>,
    }

    impl Rig {
        fn with(c: ScoutCfg, hook: Option<Hook>) -> Self {
            let r = FakeRadio::new();
            let phases = Arc::new(Mutex::new(Vec::new()));
            let me: Arc<OnceLock<Weak<Scout>>> = Arc::new(OnceLock::new());
            let (rn, rs, ph, m) = (r.clone(), r.clone(), phases.clone(), me.clone());
            let s = Arc::new(ChannelScout::new(
                c,
                r.clone(),
                Box::new(move || rn.now()),
                Box::new(move |ms| {
                    if let Some(s) = m.get().and_then(Weak::upgrade) {
                        let f = format!(
                            "{}{}{}",
                            if s.beaconing() { "B" } else { "" },
                            if s.quiet() { "Q" } else { "" },
                            if s.working() { "W" } else { "" }
                        );
                        ph.lock().unwrap().push((f, ms));
                        if let Some(h) = &hook {
                            h(&s, ms);
                        }
                    }
                    rs.st().now += ms as i64;
                }),
            ));
            me.set(Arc::downgrade(&s)).ok().unwrap();
            Self { r, s, phases }
        }
        fn new(c: ScoutCfg) -> Self {
            Self::with(c, None)
        }
        fn phases(&self) -> Vec<(String, i32)> {
            self.phases.lock().unwrap().clone()
        }
        fn clear(&self) {
            self.phases.lock().unwrap().clear();
        }
        fn steps(&self, n: usize) {
            for _ in 0..n {
                self.s.run_once();
            }
        }
    }

    fn two(measure: bool) -> ScoutCfg {
        ScoutCfg {
            channels: vec![136, 144],
            measure,
            dwell_ms: 250,
            settle_ms: 30,
            min_rounds: 2,
            search_ms: 100,
            op_window_ms: 300,
            beacon_period_ms: 20,
            pick_margin: 20,
            link_width_mhz: 40,
            ..Default::default()
        }
    }
    fn ph(p: &(String, i32), f: &str, ms: i32) -> bool {
        p.0 == f && p.1 == ms
    }
    fn entry(s: &Scout, ch: u8) -> RankEntry {
        s.ranking().into_iter().find(|e| e.ch == ch).unwrap()
    }

    #[test]
    fn idle_until_search_or_measure_then_first_dwell_switches_to_20() {
        let g = Rig::new(two(false));
        g.s.set_op(136);
        g.s.set_search(false);
        assert!(!g.s.run_once() && !g.s.working());
        assert!(g.r.calls().is_empty());
        g.s.set_search(true);
        assert!(g.s.run_once() && g.s.working());
        assert_eq!(g.r.calls()[0], "retune_width 136/20");
        assert_eq!(g.r.st().width, 20);
        g.s.run_once();
        assert_eq!(g.r.calls().last().unwrap(), "retune 144");
    }

    #[test]
    fn pinned_dwell_is_retune_settle_burst_only_on_members() {
        let g = Rig::new(two(false));
        g.s.set_op(136);
        g.s.set_search(true);
        g.s.run_once();
        let p = g.phases();
        assert_eq!(p.len(), 3);
        assert!(ph(&p[0], "W", 30) && ph(&p[1], "BW", 100) && ph(&p[2], "W", 20));
        g.clear();
        g.s.run_once();
        assert_eq!(g.r.st().ch, 144);
        g.steps(2);
        for c in g.r.calls() {
            assert!(!c.starts_with("read") && !c.contains("132") && !c.contains("140"));
        }
        assert_eq!(g.s.rounds(), 2);
    }

    #[test]
    fn auto_unlinked_dwell_is_burst_then_quiet_observe_on_primaries_observe_only_on_secondaries() {
        let g = Rig::new(two(true));
        g.s.set_op(136);
        g.s.set_search(true);
        g.s.run_once();
        let p = g.phases();
        assert_eq!(p.len(), 2);
        assert!(ph(&p[0], "W", 30) && ph(&p[1], "QW", 250));
        assert_eq!(g.r.st().ch, 132);
        g.clear();
        g.s.run_once();
        let p = g.phases();
        assert_eq!(p.len(), 4);
        assert!(ph(&p[1], "BW", 100) && ph(&p[2], "W", 20) && ph(&p[3], "QW", 250));
        assert!(!g.s.quiet() && !g.s.beaconing());
        let d = g.s.take_dwells();
        assert_eq!(d.len(), 2);
        assert!(d[1].ch == 136 && d[1].observe_ms == 250);
        let k: Vec<u8> = g.s.ranking().iter().map(|e| e.ch).collect();
        assert_eq!(k, vec![132, 136, 140, 144]);
    }

    #[test]
    fn auto_linked_dwell_has_no_burst_is_never_quiet_and_subtracts_the_tx_leak() {
        let tx = Arc::new(AtomicU64::new(0));
        let t2 = tx.clone();
        let g = Rig::with(
            ScoutCfg {
                leak_per_frame: 2.0,
                ..two(true)
            },
            Some(Box::new(move |s, ms| {
                if ms == 250 {
                    s.set_tx_frames(t2.fetch_add(60, SeqCst) + 60);
                }
            })),
        );
        g.r.busy(144, 1);
        g.s.set_op(136);
        g.s.set_search(false);
        g.s.set_tx_frames(0);
        g.steps(4);
        let p = g.phases();
        assert!(!p.is_empty());
        for (f, _) in &p {
            assert!(!f.contains('B') && !f.contains('Q'));
        }
        assert_eq!(entry(&g.s, 144).worst_busy, 250 - 120);
    }

    #[test]
    fn a_tx_counter_that_goes_back_is_no_leak() {
        let g = Rig::with(
            two(true),
            Some(Box::new(|s, ms| {
                if ms == 250 {
                    s.set_tx_frames(0);
                }
            })),
        );
        g.r.busy(144, 1);
        g.s.set_op(136);
        g.s.set_search(false);
        for _ in 0..4 {
            g.s.set_tx_frames(1000);
            g.s.run_once();
        }
        assert_eq!(entry(&g.s, 144).worst_busy, 250);
    }

    #[test]
    fn two_card_proposal_matures_after_min_rounds_and_moves_only_past_the_margin() {
        let g = Rig::new(two(true));
        g.s.set_op(136);
        g.s.set_search(true);
        g.steps(4);
        assert!(!g.s.mature() && g.s.proposal() == 136 && g.s.pick_ranking().is_empty());
        g.steps(4);
        assert!(g.s.mature() && g.s.rounds() == 2 && g.s.proposal() == 136);
        let h = Rig::new(two(true));
        h.s.set_op(136);
        h.s.set_search(true);
        h.r.busy(132, 1);
        h.steps(8);
        assert_eq!(h.s.proposal(), 144);
        assert_eq!(h.s.pick_ranking(), vec![144, 136]);
        h.s.set_op(144);
        assert_eq!(h.s.proposal(), 144);
    }

    #[test]
    fn op_change_during_the_observe_is_what_the_dwell_publishes_against() {
        let observes = Arc::new(AtomicU64::new(0));
        let o2 = observes.clone();
        let g = Rig::with(
            two(true),
            Some(Box::new(move |s, ms| {
                if ms == 250 && o2.fetch_add(1, SeqCst) + 1 == 8 {
                    s.set_op(144);
                }
            })),
        );
        g.s.set_op(144);
        g.s.set_search(true);
        g.steps(7);
        g.s.set_op(136);
        assert_eq!(g.s.proposal(), 136);
        g.s.run_once();
        assert!(g.s.mature());
        assert_eq!(g.s.proposal(), 144);
    }

    #[test]
    fn two_card_width_20_ranks_the_members_and_proposes_past_the_margin() {
        let g = Rig::new(ScoutCfg {
            link_width_mhz: 20,
            ..two(true)
        });
        g.s.set_op(136);
        g.s.set_search(true);
        g.r.busy(136, 1);
        g.steps(4);
        assert!(g.s.mature() && g.s.rounds() == 2 && g.s.proposal() == 144);
        assert_eq!(g.s.pick_ranking(), vec![144, 136]);
        g.s.set_search(false);
        g.s.freeze();
        assert!(!g.s.run_once());
        assert_eq!(g.r.calls().last().unwrap(), "retune 136");
    }

    #[test]
    fn linked_dwells_never_tune_to_op_halves() {
        let g = Rig::new(two(true));
        g.s.set_op(136);
        g.s.set_search(false);
        for _ in 0..8 {
            assert!(g.s.run_once());
        }
        for c in g.r.calls() {
            assert!(!c.contains("132") && !c.contains("136"));
        }
        for e in g.s.ranking() {
            assert_eq!(e.visits, if e.ch == 132 || e.ch == 136 { 0 } else { 4 });
        }
        assert_eq!(g.s.rounds(), 4);
    }

    #[test]
    fn linked_width_20_skips_only_op() {
        let g = Rig::new(ScoutCfg {
            link_width_mhz: 20,
            ..two(true)
        });
        g.s.set_op(136);
        g.s.set_search(false);
        g.steps(3);
        assert!(g.r.calls().iter().all(|c| !c.contains("136")));
        for e in g.s.ranking() {
            assert_eq!(e.visits, if e.ch == 136 { 0 } else { 3 });
        }
    }

    #[test]
    fn linked_mature_ignores_op_and_op_ranked_reports_it() {
        let g = Rig::new(two(true));
        g.s.set_op(136);
        g.s.set_search(true);
        g.steps(2);
        g.s.set_search(false);
        g.steps(4);
        assert!(g.s.mature() && !g.s.op_ranked());
        let h = Rig::new(two(true));
        h.s.set_op(136);
        h.s.set_search(true);
        h.steps(6);
        h.s.set_search(false);
        assert!(!h.s.mature());
        h.steps(2);
        assert!(h.s.mature() && h.s.op_ranked());
    }

    #[test]
    fn unlinked_mature_waits_for_op() {
        let g = Rig::new(two(true));
        g.s.set_op(144);
        g.s.set_search(true);
        g.steps(7);
        assert!(!g.s.op_ranked() && !g.s.mature());
        g.s.run_once();
        assert!(g.s.op_ranked() && g.s.mature());
    }

    #[test]
    fn freeze_stops_measuring_and_parks_at_link_width_on_op() {
        let g = Rig::new(two(true));
        g.s.set_op(144);
        g.s.set_search(false);
        g.s.run_once();
        assert!(g.s.pick_open() && g.s.working());
        g.s.freeze();
        assert!(!g.s.pick_open());
        assert!(!g.s.run_once());
        assert!(!g.s.working() && !g.s.quiet() && !g.s.beaconing());
        assert_eq!(g.r.calls().last().unwrap(), "retune_width 144/40");
        let n = g.r.calls().len();
        assert!(!g.s.run_once());
        assert_eq!(g.r.calls().len(), n);
        g.s.set_search(true);
        g.clear();
        g.steps(2);
        assert!(g.phases().iter().all(|p| !p.0.contains('Q')));
        let calls = g.r.calls();
        assert!(calls[n..].iter().all(|c| !c.starts_with("read")));
        assert!(calls[n].starts_with("retune_width ") && g.r.st().width == 20);
        assert!(g.s.working());
    }

    fn one(measure: bool) -> ScoutCfg {
        ScoutCfg {
            one_card: true,
            one_card_ms: 1000,
            ..two(measure)
        }
    }

    #[test]
    fn one_card_prelude_is_silent_then_op_windows_alternate_with_dwells() {
        let g = Rig::new(one(true));
        g.s.set_op(136);
        g.s.set_search(true);
        assert!(!g.s.prelude_done());
        while g.r.now() < 1000 {
            g.s.run_once();
        }
        let p = g.phases();
        assert!(!p.is_empty() && p.iter().all(|x| !x.0.contains('B')));
        assert!(g.r.calls().iter().any(|c| c == "read+nhm"));
        assert!(!g.s.prelude_done());
        g.s.run_once();
        assert!(g.s.prelude_done() && g.s.mature());
        assert!(g.s.proposal() == 136 || g.s.proposal() == 144);
        g.s.ack_prelude(136);
        g.clear();
        let n = g.r.calls().len();
        g.s.run_once();
        assert_eq!(g.r.calls()[n], "retune 136");
        let p = g.phases();
        assert!(p.len() >= 2 && ph(&p[0], "BW", 300) && ph(&p[1], "W", 20));
        g.s.run_once();
        assert!(g.phases().iter().any(|x| ph(x, "BW", 100)));
        g.s.set_search(false);
        g.s.run_once();
        g.s.set_search(true);
        assert!(g.s.prelude_done());
    }

    #[test]
    fn one_card_no_disc_window_on_the_old_op_after_the_prelude() {
        let g = Rig::new(one(true));
        g.s.set_op(136);
        g.s.set_search(true);
        while !g.s.prelude_done() {
            g.s.run_once();
        }
        g.clear();
        let n = g.r.calls().len();
        for _ in 0..5 {
            assert!(!g.s.run_once());
        }
        assert_eq!(g.r.calls().len(), n);
        assert!(g.phases().iter().all(|p| !p.0.contains('B')));
        assert!(!g.s.beaconing() && g.s.working());
        g.s.set_op(136);
        assert!(!g.s.run_once());
        assert_eq!(g.r.calls().len(), n);
        g.s.ack_prelude(144);
        g.clear();
        assert!(g.s.run_once());
        let calls = g.r.calls();
        assert_eq!(calls[n], "retune 144");
        assert!(ph(&g.phases()[0], "BW", 300));
        assert!(calls[n..].iter().all(|c| c != "retune 136"));
    }

    #[test]
    fn one_card_frozen_before_ack_does_not_wait() {
        let g = Rig::new(one(true));
        g.s.set_op(136);
        g.s.set_search(true);
        while !g.s.prelude_done() {
            g.s.run_once();
        }
        g.s.freeze();
        g.clear();
        assert!(g.s.run_once());
        assert!(ph(&g.phases()[0], "BW", 300));
    }

    #[test]
    fn one_card_pinned_has_no_prelude() {
        let g = Rig::new(one(false));
        g.s.set_op(136);
        g.s.set_search(true);
        assert!(g.s.prelude_done());
        g.s.run_once();
        assert!(ph(&g.phases()[0], "BW", 300));
    }

    #[test]
    fn run_exits_on_stop_and_parks() {
        let g = Rig::new(two(true));
        g.s.set_op(136);
        g.s.set_search(true);
        g.s.run_once();
        assert!(g.s.working());
        g.s.stop();
        g.s.run();
        assert!(g.s.done() && !g.s.working());
        assert_eq!(g.r.calls().last().unwrap(), "retune_width 136/40");
    }

    #[test]
    fn flat_scheduler_is_plan_order_round_robin() {
        let mut s = FlatScheduler::new(&[1, 2, 2, 3]);
        let mut seen = Vec::new();
        for t in 0..6 {
            let (i, ch, _) = s.next().unwrap();
            seen.push(ch);
            s.complete(i, t, true);
        }
        assert_eq!(seen, vec![1, 2, 3, 1, 2, 3]);
        assert_eq!(s.rounds, 2);
        let (i, _, _) = s.next().unwrap();
        s.complete(i, 10, false);
        assert_eq!(s.rounds, 2);
    }
}
