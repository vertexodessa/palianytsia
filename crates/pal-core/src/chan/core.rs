//! mabur `channel_core.{h,cpp}`: the channel plan, the boot scout +
//! BootPick, the hop verdict/ranker/controller and the in-flight scout, plus
//! the per-card retune/width bookkeeping they share, as one unit.
//!
//! Seams: lifecycle (`on_card_died`/`on_card_reopened`/`shutdown`), the rc
//! sink (`on_rc_body`/`on_session_opened`), the drain hooks
//! (`note_video`/`note_au_end`), `tick()`, the send path
//! (`disc_targets`/`may_send`/`disc_for_card`/`note_tx_card`/`note_sent`)
//! and `snapshot()`. Per tick: drain, then `tick()`, then the send path.
//!
//! Threading: the core thread calls everything public; the scout thread, the
//! in-flight thread and the window sampler's thread are owned here.
//! `threaded = false` runs without threads (tests): `tick()` runs one scout
//! step itself and the in-flight body runs only through
//! `run_inflight_step()`.
//!
//! Not ported: the calibration pick (CalPicker, `cal_running`) and the relay
//! cards (sweeps, survey windows); Palianytsia has neither.

use std::sync::atomic::{
    AtomicBool, AtomicI32, AtomicU32, AtomicU64, AtomicU8, AtomicUsize, Ordering::Relaxed,
    Ordering::Release,
};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::JoinHandle;

use pal_wire::ht40::GRID_5G;
use pal_wire::rc::{self, DiscAck, LinkKey};

use super::boot::{BootPick, BootPickIn, BootPickOut};
use super::cal_pick::{cal_round_pick, CalReading};
use super::gate::{
    hop_active, hop_burst_due, hop_burst_period_ms, hop_burst_wanted, tx_selection_frozen,
    MoveEdgeHold,
};
use super::hop::{
    hop_store_blank_until, hop_verdict_link_loss, hop_verdict_loss_blank_until, HopAction,
    HopActionKind, HopController, HopEvent, HopState, HopTick, HOP_SETTLE_BLANK_MS,
};
use super::hop_ranker::{HopRanker, HopVisit};
use super::inflight::{InflightScout, InflightScoutCfg};
use super::op_scan::{self, OpScan, Reading};
use super::pair::any_pair_ranked;
use super::plan::{ChannelPlan, ChannelPlanCfg, MoveEvent};
use super::radio::{
    needs_width_fix, nhm_period_4us, width_resync_open, LinkCard, ScoutFrames, ScoutRadio,
    WidthCard, WIDTH_RETRY_MS,
};
use super::ranker::RankEntry;
use super::sampler::{WindowGates, WindowSampler, WindowSamplerCfg};
use super::scout::{ChannelScout, ScoutCfg, ScoutDwell, FLAG_RETUNE_FAILED};
use super::scout_pick::{
    pick_boot_scout, pick_burst_card, pick_hop_lead, pick_inflight_scout, scan_disc_targets,
};
use super::verdict::{
    rssi_raw_to_dbm, snr_raw_to_db, HopVerdict, Verdict, VerdictCardIn, VerdictLinkIn, VerdictOut,
};
use crate::aggregator::Aggregator;
use crate::config::{ht40_offset, HopCfg, RadioCfg};
use crate::link_health::S1LossWindow;
use crate::vrx::{disc_for_channel, VrxController};

/// hop.state on the sideport.
/// steer_into_use: a move out of a non-usable member that did not land
/// is ordered again after this long (the hop cap still applies).
const STEER_RETRY_MS: f64 = 10_000.0;

pub fn hop_state_name(s: HopState) -> &'static str {
    match s {
        HopState::Idle => "idle",
        HopState::Ordered => "ordered",
        HopState::Verifying => "verifying",
        HopState::Hold => "hold",
    }
}

#[derive(Clone, Debug)]
pub struct ChannelCoreCfg {
    /// Channels, pin, scan knobs (the width is `width_mhz`).
    pub radio: RadioCfg,
    /// [hop] + [hop.verdict]
    pub hop: HopCfg,
    /// disc_for_channel re-tags per channel.
    pub key: LinkKey,
    /// The receiver width every card is opened at (`Config::rx_width`).
    pub width_mhz: u8,
    /// The pin, else the remembered member, else channels[0].
    pub start_ch: u8,
    pub leak_per_frame: f64,
    /// false = no threads (tests; deterministic with an injected clock).
    pub threaded: bool,
    /// The remembered-channel store's name for the "could not write" line.
    pub store_name: String,
}

impl Default for ChannelCoreCfg {
    fn default() -> Self {
        Self {
            radio: RadioCfg::default(),
            hop: HopCfg::default(),
            key: LinkKey::default(),
            width_mhz: 20,
            start_ch: 0,
            leak_per_frame: 1.0,
            threaded: true,
            store_name: String::new(),
        }
    }
}

/// Where the records and the stderr-style lines go (the scan.log shapes).
pub trait ChannelSink: Send {
    fn dwell(&mut self, t_ms: f64, card: usize, d: &ScoutDwell);
    fn pick(
        &mut self,
        t_ms: f64,
        picked: Option<u8>,
        rounds: u64,
        all: &[RankEntry],
        min_rounds: i32,
    );
    fn moved(&mut self, e: &MoveEvent);
    fn verdict(&mut self, t_ms: f64, o: &VerdictOut, cards: &[VerdictCardIn], link: &VerdictLinkIn);
    fn hop(&mut self, e: &HopEvent);
    /// One stderr line, no trailing newline.
    fn log(&mut self, line: &str);
}

/// A sink shared with its reader (tests, the GS core's event log).
impl<T: ChannelSink> ChannelSink for Arc<Mutex<T>> {
    fn dwell(&mut self, t_ms: f64, card: usize, d: &ScoutDwell) {
        self.lock().unwrap().dwell(t_ms, card, d)
    }
    fn pick(&mut self, t: f64, p: Option<u8>, rounds: u64, all: &[RankEntry], mr: i32) {
        self.lock().unwrap().pick(t, p, rounds, all, mr)
    }
    fn moved(&mut self, e: &MoveEvent) {
        self.lock().unwrap().moved(e)
    }
    fn verdict(&mut self, t: f64, o: &VerdictOut, c: &[VerdictCardIn], l: &VerdictLinkIn) {
        self.lock().unwrap().verdict(t, o, c, l)
    }
    fn hop(&mut self, e: &HopEvent) {
        self.lock().unwrap().hop(e)
    }
    fn log(&mut self, line: &str) {
        self.lock().unwrap().log(line)
    }
}

pub struct ChannelTickIn<'a> {
    pub now_ms: f64,
    /// VrxState::Session
    pub in_session: bool,
    /// CalSession::running(): a run owns the air and freezes every channel
    /// change (the drone defers its own retunes the same way).
    pub cal_running: bool,
    /// The send path's TX card.
    pub tx_card: usize,
    /// Per-card crc_fail/rssi/snr and the decoder stats; None reads zeros.
    pub agg: Option<&'a Aggregator>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ChannelTickOut {
    /// Keep the selected TX card this tick.
    pub tx_frozen: bool,
    pub dwell_busy: bool,
}

/// One card's verdict-window energy (sideport `energy`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StatsEnergyIn {
    pub cca: u32,
    pub fa: u32,
    pub own: u64,
    pub foreign: u64,
    pub igi: Option<i32>,
    /// NHM busy %; None when the card's NHM window did not cover this one.
    pub busy_pct: Option<f64>,
    pub own_air_pct: Option<f64>,
}

/// One card's in-flight scouting: visits cumulative, score and cost the
/// last dwell's (score only from a successful dwell).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StatsDwellIn {
    pub visits: u32,
    pub score: u32,
    pub cost_us: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StatsHopIn {
    pub verdict: &'static str,
    pub evidence: u8,
    pub ref_rung: Option<i32>,
    pub epoch: u8,
    pub state: &'static str,
    /// The standing target every RCF carries; None before the first order.
    pub target: Option<u8>,
    pub hops: u32,
    pub holds: u32,
    pub last_ms: Option<u64>,
    pub sampler_dropped: u64,
}

impl Default for StatsHopIn {
    fn default() -> Self {
        Self {
            verdict: "unknown",
            evidence: 0,
            ref_rung: None,
            epoch: 0,
            state: "idle",
            target: None,
            hops: 0,
            holds: 0,
            last_ms: None,
            sampler_dropped: 0,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ChannelSnapshot {
    /// The TX card's live channel.
    pub channel: u8,
    /// off | scouting | moving | frozen
    pub scan_state: &'static str,
    pub scan_rounds: u64,
    pub scan_pick: Option<u8>,
    pub hop: StatsHopIn,
    pub energy: Vec<Option<StatsEnergyIn>>,
    pub dwell: Vec<Option<StatsDwellIn>>,
    pub scout_gated_sends: u64,
}

pub type MsClock = Arc<dyn Fn() -> u64 + Send + Sync>;
pub type UsClock = Arc<dyn Fn() -> u64 + Send + Sync>;
pub type Sleeper = Arc<dyn Fn(i32) + Send + Sync>;
/// The remembered channel; false = could not write.
pub type StoreFn = Box<dyn FnMut(u8) -> bool + Send>;

pub type Scout = ChannelScout<dyn LinkCard>;

/// A drained in-flight dwell: the card, the record, and its visit (None
/// when a retune failed).
type DwellRec = (usize, ScoutDwell, Option<HopVisit>);

/// What the in-flight thread shares with the core thread. Lock order:
/// `inflight` then a card's `sampler.hold_card()` then `drained`.
/// In-flight thread: how often its period wait checks for a posted burst.
const SCOUT_POLL_MS: i32 = 5;
/// How long the core waits for a posted burst before giving it up (a burst
/// takes ~75 ms at the defaults).
const BURST_LOST_MS: f64 = 2000.0;

/// What the in-flight thread hands the core thread: its dwells, and whether
/// a posted burst's results are among them.
#[derive(Default)]
struct Drained {
    recs: Vec<DwellRec>,
    burst_done: bool,
}

struct Live {
    cards: Vec<Arc<dyn LinkCard>>,
    gates: Arc<WindowGates>,
    sampler: Arc<WindowSampler>,
    /// mabur `inflight_mu_`: also taken by the core's burst and width resync.
    inflight: Mutex<Option<InflightScout>>,
    drained: Mutex<Drained>,
    /// A spare card's freshness burst, posted by the core thread and run by
    /// the in-flight thread (-1 = none).
    burst_req_card: AtomicI32,
    burst_req_op: AtomicU8,
    hopping: AtomicBool,
    tx_card_now: AtomicUsize,
    scout_working: AtomicBool,
    au_seq: AtomicU64,
    scout_run: AtomicBool,
    can_scout: Vec<bool>,
    reactive: bool,
    threaded: bool,
    sleep: Sleeper,
    now_ms: MsClock,
    dwell_period_ms: i32,
}

/// A card away from op and back (burst, operator scan): the in-flight lock,
/// then the card's device-use lock, the order every holder of both follows,
/// with the card's `tune_gen` bumped on the way out and again on the way
/// back. The card may return to the channel it left, so only the generation
/// can mark an NHM arm from before the excursion stale; the device-use lock
/// keeps the sampler from seeing the retune away (a card stores its channel
/// only after each retune returns).
struct Away<'a> {
    // Released before `inflight`, as the locals it replaces were.
    _hold: MutexGuard<'a, ()>,
    inflight: MutexGuard<'a, Option<InflightScout>>,
    tune_gen: &'a AtomicU32,
}

impl Drop for Away<'_> {
    fn drop(&mut self) {
        self.tune_gen.fetch_add(1, Release);
    }
}

impl Live {
    fn away_locked<'a>(
        &'a self,
        card: usize,
        inflight: MutexGuard<'a, Option<InflightScout>>,
    ) -> Away<'a> {
        let hold = self.sampler.hold_card(card);
        let tune_gen = &self.gates.tune_gen[card];
        tune_gen.fetch_add(1, Release);
        Away {
            _hold: hold,
            inflight,
            tune_gen,
        }
    }

    fn away(&self, card: usize) -> Away<'_> {
        self.away_locked(card, self.inflight.lock().unwrap())
    }

    fn dwelling(&self, card: usize) -> bool {
        self.gates.dwell_busy.load(Relaxed) && self.gates.dwell_card.load(Relaxed) == card as i32
    }

    /// One body of the in-flight loop: one dwell on the non-TX scout card.
    fn inflight_body(&self) {
        if !self.reactive
            || !self.gates.in_session.load(Relaxed)
            || self.gates.cal_running.load(Relaxed)
            || self.hopping.load(Relaxed)
            || self.scout_working.load(Relaxed)
            || self.cards.len() < 2
        {
            return;
        }
        let Some(card) = pick_inflight_scout(&self.can_scout, self.tx_card_now.load(Relaxed))
        else {
            return;
        };
        let fe = &self.cards[card];
        if !fe.ready() {
            return;
        }
        let dwell_ch = match self.inflight.lock().unwrap().as_mut() {
            Some(sc) => sc.next_candidate(fe.channel()),
            None => return,
        };
        let Some(dwell_ch) = dwell_ch else { return };
        if self.threaded {
            // Align to the next AU boundary (<= 17 ms wait).
            let s0 = self.au_seq.load(Relaxed);
            for _ in 0..20 {
                if self.au_seq.load(Relaxed) != s0 {
                    break;
                }
                (self.sleep)(1);
            }
        }
        let mut ilk = self.inflight.lock().unwrap();
        // A calibration measure round may have taken the scout card since
        // the checks above (step_cal starts the round under this lock).
        if self.scout_working.load(Relaxed) {
            return;
        }
        let Some(sc) = ilk.as_mut() else { return };
        let (d, v) = {
            // The sampler skips a dwelling card, but its dwell_busy check and
            // this dwell are otherwise unordered; the device-use lock makes
            // a pass finish before the dwell or start after it.
            let _hold = self.sampler.hold_card(card);
            sc.set_radio(fe.clone() as Arc<dyn ScoutRadio>);
            self.gates.dwell_card.store(card as i32, Relaxed);
            self.gates.dwell_busy.store(true, Relaxed);
            let r = sc.dwell(dwell_ch, fe.channel());
            self.gates.dwell_gen[card].fetch_add(1, Relaxed);
            self.gates.dwell_busy.store(false, Relaxed);
            self.gates.dwell_card.store(-1, Relaxed);
            r
        };
        self.drained.lock().unwrap().recs.push((card, d, v));
        drop(ilk);
    }

    /// One freshness burst on `card`, back to `op`; the caller holds
    /// `inflight` (the scout must exist). `dwell_busy`/`dwell_card` mark the
    /// card mid-dwell for the verdict, the TX selector and the mechanical
    /// retune loop, which run on while a posted burst is out.
    fn burst_on_card(
        &self,
        ilk: MutexGuard<'_, Option<InflightScout>>,
        card: usize,
        op: u8,
        recs: &mut Vec<ScoutDwell>,
    ) -> Vec<HopVisit> {
        let mut away = self.away_locked(card, ilk);
        let sc = away
            .inflight
            .as_mut()
            .expect("burst_on_card: scout present");
        sc.set_radio(self.cards[card].clone() as Arc<dyn ScoutRadio>);
        self.gates.dwell_card.store(card as i32, Relaxed);
        self.gates.dwell_busy.store(true, Relaxed);
        let visits = sc.burst(op, recs);
        self.gates.dwell_busy.store(false, Relaxed);
        self.gates.dwell_card.store(-1, Relaxed);
        visits
    }

    /// The in-flight thread's side of a posted burst.
    fn run_posted_burst(&self) {
        let card = self.burst_req_card.load(Relaxed);
        if card < 0 {
            return;
        }
        let mut recs = Vec::new();
        let mut visits = Vec::new();
        {
            let ilk = self.inflight.lock().unwrap();
            // A calibration measure round may have taken the scout card
            // since the post (step_cal starts the round under this lock):
            // stand down, but still report done so the core stops holding
            // the trigger.
            if ilk.is_some() && !self.scout_working.load(Relaxed) {
                visits = self.burst_on_card(
                    ilk,
                    card as usize,
                    self.burst_req_op.load(Relaxed),
                    &mut recs,
                );
            }
        }
        // Clear the request before the results become visible: once the
        // core sees burst_done it may post the next burst, which this must
        // not erase (nor one the core re-posted after giving this one up).
        let _ = self
            .burst_req_card
            .compare_exchange(card, -1, Relaxed, Relaxed);
        let mut dr = self.drained.lock().unwrap();
        // Visits pair with records in order, as the drain expects; a burst
        // record without its own visit (a failed retune) gets None.
        let mut vi = visits.into_iter();
        for d in recs {
            let v = if d.flags & FLAG_RETUNE_FAILED == 0 {
                vi.next()
            } else {
                None
            };
            dr.recs.push((card as usize, d, v));
        }
        dr.burst_done = true;
    }

    fn scout_loop(&self) {
        while self.scout_run.load(Relaxed) {
            // The periodic dwell's period, cut short by a posted burst.
            let t0 = (self.now_ms)();
            while self.scout_run.load(Relaxed)
                && self.burst_req_card.load(Relaxed) < 0
                && (self.now_ms)().saturating_sub(t0) < self.dwell_period_ms.max(0) as u64
            {
                (self.sleep)(SCOUT_POLL_MS);
            }
            if !self.scout_run.load(Relaxed) {
                break;
            }
            if self.burst_req_card.load(Relaxed) >= 0 {
                self.run_posted_burst();
            } else {
                self.inflight_body();
            }
        }
    }
}

pub struct ChannelCore {
    cfg: ChannelCoreCfg,
    live: Arc<Live>,
    sink: Box<dyn ChannelSink>,
    store: StoreFn,
    now_ms: MsClock,
    n_cards: usize,
    pinned: bool,
    /// Auto mode: the in-flight hop, its scout and burst run only here.
    reactive: bool,
    snr_ok: Vec<bool>,
    scout_card: Option<usize>,
    one_card: bool,
    /// The last value handed to `store`.
    saved_op: u8,
    gs_start_ms: u64,

    plan: ChannelPlan,
    scout: Option<Arc<Scout>>,
    boot_pick: BootPick,
    verdict: HopVerdict,
    ranker: HopRanker,
    hopc: HopController,
    /// The verdict's own pre-FEC loss windows, base and enhancement; the
    /// verdict reads the worse (`hop_verdict_link_loss`).
    s1_hop_loss: S1LossWindow,
    enh_hop_loss: S1LossWindow,

    cur_ch: Vec<u8>,
    /// The receiver width (cfg.width_mhz).
    width_mhz: u8,
    width_tried: Vec<bool>,
    /// When a failed resync may run again; 0 = not due.
    width_retry_ms: Vec<f64>,
    energy_last: Vec<Option<StatsEnergyIn>>,
    dwell_stats: Vec<Option<StatsDwellIn>>,
    window_prev: Vec<ScoutFrames>,
    window_prev_crc: Vec<u64>,
    /// The card's seq walk at the previous sample: the window's delivery
    /// ratio scales its own airtime.
    window_prev_seq_exp: Vec<u64>,
    window_prev_seq_rcv: Vec<u64>,
    window_prev_ok: Vec<bool>,
    window_prev_ms: Vec<u64>,

    scout_thread: Option<JoinHandle<()>>,
    scout_started: bool,
    scout_was_working: bool,
    scout_card_down: bool,
    scout_search_req: bool,

    link_edge_seen: bool,
    scout_card_died_seen: bool,
    pending_relocate: Option<u8>,
    frozen_pick: Option<u8>,
    /// The last steer_into_use order.
    steer_ms: Option<f64>,
    /// The operator's channel scan (op_scan.rs) and its dweller.
    op_scan: OpScan,
    scanner: Option<InflightScout>,
    last_bi: BootPickIn,

    hop_lead_latched: Option<usize>,
    recovered_prev_window: u64,
    hop_was_active: bool,
    /// A spare card's freshness burst posted to the in-flight thread and
    /// not yet ranked: the trigger is held back meanwhile.
    burst_inflight: bool,
    burst_posted_ms: f64,
    move_hold: MoveEdgeHold,
    last_verdict: Verdict,
    last_verdict_out: VerdictOut,
    last_hop_event_ms: Option<u64>,
    last_burst_ms: f64,
    rcf_sent_total: u64,
    rcf_sent_at_order: u64,
    ctrl_sent_total: u64,
    /// Bumped inside the `may_send` query.
    scout_gated_sends: AtomicU64,
    last_video_ch: u8,
    rc_body_rx_ch: u8,

    sampler_started: bool,
    inflight_started: bool,
    inflight_thread: Option<JoinHandle<()>>,
    shut: bool,
    now_ms_cur: f64,
    now_ms_u_cur: u64,
    cal_running_cur: bool,
    in_session_cur: bool,

    // ---- a calibration run's channel pick ----
    cal_meas: CalMeasure,
    cal_reloc: CalReloc,
    /// This tick's Relocate is the calibration's.
    cal_reloc_tick: bool,
    /// The latest verdict window's reading of op: the worst valid card on
    /// op, foreign frames scaled to radio.scan.dwell_ms.
    op_live: Option<CalReading>,
    op_live_ms: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
enum CalMeasureState {
    #[default]
    Off,
    Waiting,
    Running,
    Done,
}

#[derive(Clone, Copy, Debug, Default)]
struct CalMeasure {
    s: CalMeasureState,
    /// Round start (op's reading must be newer).
    t0: f64,
    /// Running: the round ends here with op as the answer.
    bound_ms: f64,
    best: u8,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
enum CalRelocState {
    #[default]
    Off,
    Waiting,
    InFlight,
    Settled,
}

#[derive(Clone, Copy, Debug, Default)]
struct CalReloc {
    s: CalRelocState,
    ch: u8,
    t0: f64,
    confirmed: bool,
    ok: bool,
}

impl ChannelCore {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        cfg: ChannelCoreCfg,
        cards: Vec<Arc<dyn LinkCard>>,
        sink: Box<dyn ChannelSink>,
        store: StoreFn,
        now_ms: MsClock,
        now_us: UsClock,
        sleep_ms: Sleeper,
    ) -> Self {
        let n = cards.len();
        let pinned = cfg.radio.pin.is_some();
        let reactive = !pinned;
        let can_scout: Vec<bool> = cards.iter().map(|c| c.can_scout()).collect();
        let snr_ok: Vec<bool> = cards.iter().map(|c| c.caps().snr_ok).collect();
        let one_card = n == 1;
        let scout_card = pick_boot_scout(&can_scout);
        let scan = &cfg.radio.scan;
        let (mut scout, mut inflight) = (None, None);
        if let Some(sci) = scout_card {
            let sc = ScoutCfg {
                channels: cfg.radio.channels.clone(),
                usable: cfg.radio.use_.clone(),
                measure: !pinned,
                dwell_ms: scan.dwell_ms,
                settle_ms: scan.settle_ms,
                min_rounds: scan.min_rounds,
                search_ms: scan.search_ms,
                op_window_ms: scan.op_window_ms,
                beacon_period_ms: 20,
                one_card_ms: scan.one_card_ms,
                pick_margin_pct: f64::from(scan.pick_margin_pct),
                one_card,
                link_width_mhz: cfg.width_mhz,
                busy_dbm: scan.busy.busy_dbm,
                blocked_pct: scan.busy.blocked_pct,
                leak_per_frame: cfg.leak_per_frame,
                // Two passes over the set find a drone on it; past that the
                // search also bursts on the rest of the 5 GHz grid.
                grid_after_rounds: 2,
            };
            let (nm, sl) = (now_ms.clone(), sleep_ms.clone());
            let s = Arc::new(Scout::new(
                sc,
                cards[sci].clone(),
                Box::new(move || nm() as i64),
                Box::new(move |ms| sl(ms)),
            ));
            s.set_op(cfg.start_ch);
            s.set_search(true);
            scout = Some(s);
            let (nu, sl) = (now_us.clone(), sleep_ms.clone());
            inflight = Some(InflightScout::new(
                InflightScoutCfg {
                    observe_ms: cfg.hop.dwell_observe_ms,
                    period_ms: cfg.hop.dwell_period_ms,
                    candidates: cfg.radio.usable().to_vec(),
                    width_mhz: cfg.width_mhz,
                    busy_dbm: scan.busy.busy_dbm,
                },
                cards[sci].clone() as Arc<dyn ScoutRadio>,
                Box::new(move || nu() as i64),
                Box::new(move |ms| sl(ms)),
            ));
        }
        let gates = Arc::new(WindowGates::new(n));
        gates.op.store(cfg.start_ch, Relaxed);
        let sampler = Arc::new(WindowSampler::new(
            cards.clone(),
            WindowSamplerCfg {
                window_ms: cfg.hop.window_ms,
                nhm_period_4us: nhm_period_4us((cfg.hop.window_ms - 10).max(1)),
                busy_dbm: scan.busy.busy_dbm,
            },
            gates.clone(),
        ));
        // The operator scan's own dweller (its radio is set per step).
        let scanner = cards.first().map(|c| {
            let (nu, sl) = (now_us.clone(), sleep_ms.clone());
            InflightScout::new(
                InflightScoutCfg {
                    observe_ms: op_scan::OBSERVE_MS,
                    period_ms: 0,
                    candidates: Vec::new(),
                    width_mhz: cfg.width_mhz,
                    busy_dbm: scan.busy.busy_dbm,
                },
                c.clone() as Arc<dyn ScoutRadio>,
                Box::new(move || nu() as i64),
                Box::new(move |ms| sl(ms)),
            )
        });
        let live = Arc::new(Live {
            cards,
            gates,
            sampler,
            inflight: Mutex::new(inflight),
            drained: Mutex::new(Drained::default()),
            burst_req_card: AtomicI32::new(-1),
            burst_req_op: AtomicU8::new(0),
            hopping: AtomicBool::new(false),
            tx_card_now: AtomicUsize::new(0),
            scout_working: AtomicBool::new(false),
            au_seq: AtomicU64::new(0),
            scout_run: AtomicBool::new(false),
            can_scout,
            reactive,
            threaded: cfg.threaded,
            sleep: sleep_ms,
            now_ms: now_ms.clone(),
            dwell_period_ms: cfg.hop.dwell_period_ms,
        });
        let gs_start_ms = now_ms();
        Self {
            plan: ChannelPlan::new(ChannelPlanCfg {
                start: cfg.start_ch,
                channels: cfg.radio.channels.clone(),
                n_cards: n,
                search_after_ms: scan.search_after_ms,
            }),
            boot_pick: BootPick::new(scout.is_some() && !pinned),
            verdict: HopVerdict::new(cfg.hop.clone(), scan.busy.clone()),
            ranker: HopRanker::new(
                cfg.hop.clone(),
                scan.busy.clone(),
                cfg.radio.usable().to_vec(),
                0,
                cfg.width_mhz,
            ),
            hopc: HopController::new(cfg.hop.clone()),
            s1_hop_loss: S1LossWindow::default(),
            enh_hop_loss: S1LossWindow::default(),
            width_mhz: cfg.width_mhz,
            scout_search_req: scout.is_some(),
            scout,
            live,
            sink,
            store,
            now_ms,
            n_cards: n,
            pinned,
            reactive,
            snr_ok,
            scout_card,
            one_card,
            saved_op: cfg.start_ch,
            gs_start_ms,
            cur_ch: vec![cfg.start_ch; n],
            width_tried: vec![false; n],
            width_retry_ms: vec![0.0; n],
            energy_last: vec![None; n],
            dwell_stats: vec![None; n],
            window_prev: vec![ScoutFrames::default(); n],
            window_prev_crc: vec![0; n],
            window_prev_seq_exp: vec![0; n],
            window_prev_seq_rcv: vec![0; n],
            window_prev_ok: vec![false; n],
            window_prev_ms: vec![0; n],
            scout_thread: None,
            scout_started: false,
            scout_was_working: false,
            scout_card_down: false,
            link_edge_seen: false,
            scout_card_died_seen: false,
            pending_relocate: None,
            frozen_pick: pinned.then_some(cfg.start_ch),
            steer_ms: None,
            op_scan: OpScan::default(),
            scanner,
            last_bi: BootPickIn::default(),
            hop_lead_latched: None,
            recovered_prev_window: 0,
            hop_was_active: false,
            burst_inflight: false,
            burst_posted_ms: 0.0,
            move_hold: MoveEdgeHold::default(),
            last_verdict: Verdict::Healthy,
            last_verdict_out: VerdictOut::default(),
            last_hop_event_ms: None,
            last_burst_ms: -1e18,
            rcf_sent_total: 0,
            rcf_sent_at_order: 0,
            ctrl_sent_total: 0,
            scout_gated_sends: AtomicU64::new(0),
            last_video_ch: cfg.start_ch,
            rc_body_rx_ch: 0,
            sampler_started: false,
            inflight_started: false,
            inflight_thread: None,
            shut: false,
            now_ms_cur: 0.0,
            in_session_cur: false,
            cal_meas: CalMeasure::default(),
            cal_reloc: CalReloc::default(),
            cal_reloc_tick: false,
            op_live: None,
            op_live_ms: -1.0,
            now_ms_u_cur: 0,
            cal_running_cur: false,
            cfg,
        }
    }

    fn scout_owns(&self) -> bool {
        self.scout.as_ref().is_some_and(|s| {
            s.working()
                || self.scout_search_req
                || (!self.pinned && s.pick_open())
                || s.round_open()
        })
    }

    fn is_scout_card(&self, i: usize) -> bool {
        self.scout_card == Some(i)
    }

    fn log(&mut self, line: String) {
        self.sink.log(&line);
    }

    fn ready_mask(&self) -> Vec<bool> {
        self.live.cards.iter().map(|c| c.ready()).collect()
    }

    // ---- state ----

    pub fn snapshot(&self, tx_card: usize) -> ChannelSnapshot {
        let lv = &self.last_verdict_out;
        let hc = self.hopc.hop_ch();
        ChannelSnapshot {
            channel: self.live.cards[tx_card].channel(),
            scan_state: if self.scout.is_none() || self.pinned {
                "off"
            } else if !self.boot_pick.open() {
                "frozen"
            } else if self.boot_pick.relocating() {
                "moving"
            } else {
                "scouting"
            },
            scan_rounds: self.scout.as_ref().map_or(0, |s| s.rounds()),
            scan_pick: self.frozen_pick,
            hop: StatsHopIn {
                verdict: lv.v.as_str(),
                evidence: lv.evidence,
                ref_rung: (lv.ref_rung >= 0).then_some(lv.ref_rung),
                epoch: self.hopc.epoch(),
                state: hop_state_name(self.hopc.state()),
                target: (hc != 0).then_some(hc),
                hops: self.hopc.hops(),
                holds: self.hopc.holds(),
                last_ms: self.last_hop_event_ms,
                sampler_dropped: self.sampler_dropped(),
            },
            energy: self.energy_last.clone(),
            dwell: self.dwell_stats.clone(),
            scout_gated_sends: self.scout_gated_sends(),
        }
    }
    pub fn op(&self) -> u8 {
        self.plan.op()
    }
    /// The boot scout card; it opens at 20 MHz.
    pub fn scout_card(&self) -> Option<usize> {
        self.scout_card
    }
    pub fn hopping(&self) -> bool {
        self.plan.hopping()
    }
    pub fn hop_target(&self) -> u8 {
        self.plan.hop_target()
    }
    /// Live reads (not the end-of-tick copy): the send path runs after
    /// tick(), and a dwell the scout thread starts in between still freezes
    /// TX.
    pub fn tx_frozen(&self) -> bool {
        tx_selection_frozen(self.dwell_busy(), self.plan.hopping())
    }
    pub fn dwell_busy(&self) -> bool {
        self.live.gates.dwell_busy.load(Relaxed)
    }
    pub fn scout_gated_sends(&self) -> u64 {
        self.scout_gated_sends.load(Relaxed)
    }
    pub fn pick_open(&self) -> bool {
        self.boot_pick.open()
    }
    pub fn pinned(&self) -> bool {
        self.pinned
    }
    /// The card reopen gate.
    pub fn scout_working(&self) -> bool {
        self.scout.as_ref().is_some_and(|s| s.working())
    }
    pub fn scout_owns_card(&self, card: usize) -> bool {
        self.scout_owns() && self.is_scout_card(card)
    }
    pub fn sampler_dropped(&self) -> u64 {
        self.live.sampler.dropped()
    }
    /// The card's device-use lock: the card lifecycle holds it across
    /// stop() + reopen so no window read or dwell is on the device.
    pub fn hold_card(&self, card: usize) -> MutexGuard<'_, ()> {
        self.live.sampler.hold_card(card)
    }
    #[cfg(test)]
    pub(crate) fn scout(&self) -> Option<&Arc<Scout>> {
        self.scout.as_ref()
    }

    // ---- receive hooks (core thread, in the drain loop) ----

    /// Before agg.on_rx_body().
    pub fn on_rc_body(&mut self, rx_ch: u8) {
        self.rc_body_rx_ch = rx_ch;
    }
    pub fn is_link_video(&self, rx_ch: u8) -> bool {
        self.plan.is_link_video(rx_ch)
    }
    pub fn note_video(&mut self, rx_ch: u8) {
        self.last_video_ch = rx_ch;
    }
    pub fn note_au_end(&self) {
        self.live.au_seq.fetch_add(1, Relaxed);
    }

    /// The DISC_ACK that opened the session: the link forms where it was
    /// heard. Only our nonce, not key-mismatch flagged (a stranger's drone
    /// cannot drag the cards), only on this body's rx channel; the
    /// relocation to want, if any, is an ordinary hop order (BootPick).
    pub fn on_session_opened(&mut self, ack: &DiscAck, now_ms: f64, vrx: &mut VrxController) {
        let x = self.rc_body_rx_ch;
        if ack.vrx_nonce == vrx.rz_nonce()
            && ack.flags & rc::ACK_KEY_MISMATCH == 0
            && x != 0
            && x != self.plan.op()
            && (self.plan.member(x) || GRID_5G.contains(&x))
            && !self.plan.hopping()
        {
            let op = self.plan.op();
            let off = if self.plan.member(x) {
                ""
            } else {
                " (outside the set)"
            };
            self.log(format!(
                "maburgs channel: drone found on {x} (op {op}): the link forms there{off}"
            ));
            self.plan.link_found(now_ms, x);
            // A DISC proposes the channel it is sent on: stay.
            vrx.set_proposal(self.plan.op());
        }
    }

    // ---- lifecycle ----

    /// The card reads !alive(): scout-card handling.
    pub fn on_card_died(&mut self, card: usize) {
        if self.scout_card_down || !self.is_scout_card(card) {
            return;
        }
        let Some(s) = self.scout.clone() else { return };
        self.scout_card_down = true;
        if s.working() {
            self.scout_card_died_seen = true;
            let (rounds, w) = (s.rounds(), self.width_mhz);
            self.log(format!(
                "maburgs channel: scout card {card} died at {rounds} rounds; search held \
                 until it reopens, card rejoins at {w} MHz"
            ));
        }
        self.scout_search_req = false;
        s.set_search(false);
    }

    /// Reopen succeeded: resync cur_ch / width_tried.
    pub fn on_card_reopened(&mut self, card: usize) {
        self.cur_ch[card] = self.live.cards[card].channel();
        self.width_tried[card] = false;
        if self.is_scout_card(card) {
            self.scout_card_down = false;
        }
    }

    /// Joins every thread; idempotent.
    pub fn shutdown(&mut self) {
        if self.shut {
            return;
        }
        self.shut = true;
        self.live.sampler.stop();
        self.live.scout_run.store(false, Relaxed);
        if let Some(h) = self.inflight_thread.take() {
            let _ = h.join();
        }
        if let Some(s) = &self.scout {
            s.stop();
        }
        if let Some(h) = self.scout_thread.take() {
            let _ = h.join();
        }
    }

    // ---- send path (core thread, after tick()) ----

    /// scan_disc_targets while the scout owns a card, else {tx}.
    pub fn disc_targets(&self, tx: usize) -> Vec<usize> {
        match &self.scout {
            Some(s) if self.scout_owns() => scan_disc_targets(
                self.n_cards,
                self.n_cards,
                self.scout_card,
                s.beaconing(),
                &self.ready_mask(),
            ),
            _ => vec![tx],
        }
    }

    /// The scout gate; false = drop (counted).
    pub fn may_send(&self, card: usize) -> bool {
        if let Some(s) = &self.scout {
            if self.scout_owns() && ((self.is_scout_card(card) && !s.beaconing()) || s.quiet()) {
                self.scout_gated_sends.fetch_add(1, Relaxed);
                return false;
            }
        }
        true
    }

    /// A DISC proposes the channel it is sent on.
    pub fn disc_for_card(&self, frame: &[u8], card: usize) -> Vec<u8> {
        let ch = self.live.cards[card].channel();
        if self.plan.member(ch) || GRID_5G.contains(&ch) {
            disc_for_channel(frame, ch, &self.cfg.key)
        } else {
            frame.to_vec()
        }
    }

    pub fn note_sent(&mut self, sent_ok: bool, is_rcf: bool) {
        if sent_ok {
            self.ctrl_sent_total += 1;
        }
        if is_rcf {
            self.rcf_sent_total += 1;
        }
    }

    /// The send path's TX choice, stored the instant it is made: the
    /// in-flight thread picks its dwell card against the live TX card.
    pub fn note_tx_card(&self, tx: usize) {
        self.live.tx_card_now.store(tx, Relaxed);
    }

    // ---- test seams (threaded = false) ----

    pub fn run_scout_step(&self) {
        if let Some(s) = &self.scout {
            s.run_once();
        }
    }
    pub fn run_inflight_step(&self) {
        self.live.inflight_body();
    }
    /// WindowSampler::step_now() at the tick's time.
    pub fn run_sampler_step(&self) {
        self.live.sampler.step_now(self.now_ms_u_cur);
    }

    // ---- the per-tick block ----

    pub fn tick(&mut self, inp: &ChannelTickIn, vrx: &mut VrxController) -> ChannelTickOut {
        self.now_ms_cur = inp.now_ms;
        self.in_session_cur = inp.in_session;
        self.now_ms_u_cur = inp.now_ms as u64;
        self.live.tx_card_now.store(inp.tx_card, Relaxed);
        self.live.gates.in_session.store(inp.in_session, Relaxed);
        self.live.gates.cal_running.store(inp.cal_running, Relaxed);
        // The scout thread starts once its card is up (threaded), or one
        // step runs here (tests).
        if let (Some(s), Some(sci)) = (&self.scout, self.scout_card) {
            if !self.scout_started && self.live.cards[sci].ready() {
                self.scout_started = true;
                if self.cfg.threaded {
                    let s = s.clone();
                    self.scout_thread = Some(std::thread::spawn(move || s.run()));
                }
            }
        }
        if !self.cfg.threaded && self.scout_started {
            self.run_scout_step();
        }
        self.cal_running_cur = inp.cal_running;
        // A run counts as linked: the sweeps outlast the link's failsafe.
        self.plan
            .tick(inp.now_ms, inp.in_session || inp.cal_running);
        self.step_scout_inputs();
        self.step_boot_pick(inp);
        self.step_store(vrx);
        self.step_gates();
        self.step_hop_edge_and_window(inp, vrx);
        self.step_cal(inp);
        self.step_controller(inp, vrx);
        self.step_op_scan(inp);
        self.step_move_edge(inp.now_ms, vrx);
        self.step_drains();
        self.step_width_resync();
        for ev in self.plan.take_events() {
            self.sink.moved(&ev);
            let card = ev.card.map_or(-1, |c| c as i64);
            self.log(format!(
                "maburgs channel: {} card {card} {} -> {}",
                ev.reason.as_str(),
                ev.from,
                ev.to
            ));
        }
        self.step_mechanical_retune();
        // Again at the end: a scout search started during this tick is gated
        // before the sampler's next pass, not a tick later.
        self.step_gates();
        if let Some(agg) = inp.agg {
            let s1 = agg.decoder().stats(0);
            self.s1_hop_loss
                .add(s1.arr_expected, s1.arr_arrived, inp.now_ms);
            let enh = agg.decoder().stats(1);
            self.enh_hop_loss
                .add(enh.arr_expected, enh.arr_arrived, inp.now_ms);
        }
        let dwell_busy = self.dwell_busy();
        ChannelTickOut {
            dwell_busy,
            tx_frozen: tx_selection_frozen(dwell_busy, self.plan.hopping()),
        }
    }

    fn step_gates(&self) {
        let g = &self.live.gates;
        g.op.store(self.plan.op(), Relaxed);
        let owns = self.scout_owns();
        for (i, b) in g.card_blocked.iter().enumerate() {
            b.store(
                owns && self.is_scout_card(i),
                std::sync::atomic::Ordering::Release,
            );
        }
    }

    fn freeze_pick(&mut self, t: f64, why: &str) {
        let Some(s) = self.scout.clone() else { return };
        s.freeze();
        let min_rounds = self.cfg.radio.scan.min_rounds;
        let mr = if self.one_card && s.prelude_done() && s.rounds() < min_rounds as u64 {
            1
        } else {
            min_rounds
        };
        let all = s.ranking();
        // The ranking's width is the scout's: its dwells ran at its card's width.
        let any = if s.link_width_mhz() == 40 {
            any_pair_ranked(&all, self.cfg.radio.usable(), mr)
        } else {
            all.iter().any(|e| e.visits >= mr as u32)
        };
        let pick = self.plan.want();
        if any {
            self.ranker.set_boot_pick(pick);
        }
        self.sink.pick(t, any.then_some(pick), s.rounds(), &all, mr);
        self.log(format!(
            "maburgs channel: pick frozen on {pick} ({why}) after {} rounds",
            s.rounds()
        ));
    }

    fn apply_hop_action(&mut self, act: &HopAction, now_ms: f64, vrx: &mut VrxController) {
        match act.kind {
            HopActionKind::Order => {
                vrx.set_hop(act.target, act.epoch);
                vrx.restore_rung(act.restore_rung, now_ms);
                vrx.blank_store(now_ms + self.cfg.hop.confirm_ms as f64 + HOP_SETTLE_BLANK_MS);
                // Two cards: retune the lead now. One card: the sole radio
                // stays on the old channel while the order rides
                // one_card_repeats RCFs; OneCardRetune moves it.
                if let Some(lead) = act.lead_card {
                    self.plan.hop_order(now_ms, act.target, Some(lead));
                }
            }
            HopActionKind::OneCardRetune => self.plan.hop_order(now_ms, act.target, None),
            HopActionKind::Confirm => {
                self.plan.hop_confirmed(now_ms);
                self.verdict.landed(now_ms);
            }
            HopActionKind::Withdraw => {
                vrx.set_hop(act.target, act.epoch);
                self.plan.hop_withdraw(now_ms);
            }
            // Thaw the frozen references.
            HopActionKind::VerifyPass => self.verdict.reset(),
            HopActionKind::Hold | HopActionKind::None => {}
        }
    }

    fn dispatch_hop_action(
        &mut self,
        act: &HopAction,
        relocate_tick: bool,
        now_ms: f64,
        vrx: &mut VrxController,
    ) {
        self.apply_hop_action(act, now_ms, vrx);
        self.boot_pick.note_hop_action(act.kind, relocate_tick);
        let cal_order = relocate_tick && self.cal_reloc_tick;
        if cal_order {
            if act.kind == HopActionKind::Order {
                self.cal_reloc.s = CalRelocState::InFlight;
            } else {
                self.settle_cal_reloc(false, "refused (hop cap)");
            }
        } else if self.cal_reloc.s == CalRelocState::InFlight {
            match act.kind {
                HopActionKind::Confirm => self.cal_reloc.confirmed = true,
                HopActionKind::VerifyPass => self.settle_cal_reloc(true, "landed"),
                HopActionKind::Withdraw => self.settle_cal_reloc(false, "withdrawn"),
                HopActionKind::Hold => self.settle_cal_reloc(false, "held"),
                _ => {}
            }
        }
        if let Some(b) = hop_verdict_loss_blank_until(act, now_ms) {
            self.s1_hop_loss.blank_until(b);
            self.enh_hop_loss.blank_until(b);
        }
        match act.kind {
            HopActionKind::Order => {
                self.live.hopping.store(true, Relaxed);
                self.rcf_sent_at_order = self.rcf_sent_total;
            }
            HopActionKind::Confirm | HopActionKind::Withdraw => {
                self.live.hopping.store(false, Relaxed)
            }
            _ => {}
        }
        for mut e in self.hopc.take_events() {
            if cal_order && e.kind == "relocate" {
                e.reason = "cal".into();
            }
            self.sink.hop(&e);
            let reason = if e.reason.is_empty() {
                String::new()
            } else {
                format!(" ({})", e.reason)
            };
            self.log(format!(
                "maburgs hop: {} epoch {} target {} score {} +{:.0} ms{reason}",
                e.kind, e.epoch, e.target, e.score, e.elapsed_ms
            ));
            self.last_hop_event_ms = Some(e.elapsed_ms.max(0.0) as u64);
        }
    }

    fn step_scout_inputs(&mut self) {
        let Some(s) = self.scout.clone() else {
            self.live.scout_working.store(false, Relaxed);
            return;
        };
        let sci = self.scout_card.expect("a scout has a card");
        s.set_op(self.plan.op());
        self.scout_search_req =
            self.plan.release_scout() && !self.scout_card_down && !self.live.dwelling(sci);
        s.set_search(self.scout_search_req);
        s.set_tx_frames(self.ctrl_sent_total);
        let w = self.scout_owns();
        if self.scout_was_working && !w {
            self.cur_ch[sci] = self.live.cards[sci].channel();
            self.width_tried[sci] = false;
        }
        self.scout_was_working = w;
        self.live.scout_working.store(w, Relaxed);
    }

    /// The operator's channel choice on a live link (`radio.use`): the
    /// members the link may sit on, one = that channel only, all = plain
    /// auto. The search keeps the drone's whole set; a link outside the new
    /// choice relocates into it by the boot pick's hop (steer_into_use).
    pub fn set_usable(&mut self, chosen: &[u8]) -> Result<(), String> {
        if self.pinned {
            return Err("a pinned config keeps its channel".into());
        }
        if chosen.is_empty() {
            return Err("choose at least one channel".into());
        }
        let set = &self.cfg.radio.channels;
        if let Some(c) = chosen.iter().find(|c| !set.contains(c)) {
            return Err(format!("{c} is not a member of radio.channels"));
        }
        let use_: Vec<u8> = if set.iter().all(|c| chosen.contains(c)) {
            Vec::new()
        } else {
            chosen.to_vec()
        };
        self.cfg.radio.use_ = use_.clone();
        let usable = self.cfg.radio.usable().to_vec();
        if let Some(s) = &self.scout {
            s.set_usable(use_);
        }
        if let Some(i) = self.live.inflight.lock().unwrap().as_mut() {
            i.set_candidates(usable.clone());
        }
        self.ranker.set_candidates(usable.clone());
        // A relocation wanted toward a member no longer chosen is dropped;
        // the next tick steers toward the new choice at once.
        let op = self.plan.op();
        if !self.plan.hopping() && !usable.contains(&self.plan.want()) {
            self.plan.set_want(self.now_ms_cur, op);
        }
        self.steer_ms = None;
        self.log(format!(
            "maburgs channel: operator channels {usable:?} (link on {op})"
        ));
        Ok(())
    }

    /// Start the operator's channel scan over `set` (empty = the drone's
    /// whole set); a running scan restarts.
    pub fn begin_op_scan(&mut self, set: &[u8]) {
        let set = if set.is_empty() {
            self.cfg.radio.channels.clone()
        } else {
            set.to_vec()
        };
        self.op_scan.start(&set, self.now_ms_cur);
        self.log(format!("maburgs channel: operator scan of {set:?}"));
    }

    /// The scan's view for the UI line; None before the first scan.
    pub fn op_scan_json(&self) -> Option<String> {
        self.op_scan.json(self.plan.op())
    }

    /// One scan step: a few short dwells on the spare card (two cards) or
    /// the link card (one), each straight back to where the card was. It
    /// waits while a hop or relocation is under way or the boot scout holds
    /// the card.
    fn step_op_scan(&mut self, inp: &ChannelTickIn) {
        let now = inp.now_ms;
        let due = self.op_scan.due(now);
        if due.is_empty() {
            return;
        }
        let card = if self.n_cards >= 2 {
            pick_inflight_scout(&self.live.can_scout, inp.tx_card)
        } else {
            self.live
                .can_scout
                .first()
                .copied()
                .unwrap_or(false)
                .then_some(0)
        };
        let blocked = self.plan.hopping()
            || !matches!(self.hopc.state(), HopState::Idle | HopState::Hold)
            || self.boot_pick.relocating()
            || self.pending_relocate.is_some()
            || card.is_none_or(|c| self.scout_owns_card(c) || !self.live.cards[c].ready());
        let (Some(card), Some(sc), false) = (card, self.scanner.as_mut(), blocked) else {
            self.op_scan.defer(now);
            return;
        };
        let live = self.live.clone();
        let fe = &live.cards[card];
        let back = fe.channel();
        let mut recs = Vec::with_capacity(due.len());
        {
            let _away = live.away(card);
            sc.set_radio(fe.clone() as Arc<dyn ScoutRadio>);
            for ch in due {
                let (d, v) = sc.dwell(ch, back);
                self.op_scan.record(
                    Reading {
                        ok: v.is_some(),
                        busy: d.busy_valid.then_some(d.busy_pct),
                        foreign: d.frames.wrapping_sub(d.dvr_frames),
                        observe_ms: d.observe_ms.max(0) as u64,
                    },
                    now,
                );
                recs.push(d);
            }
            self.cur_ch[card] = fe.channel();
        }
        for d in &recs {
            self.sink.dwell(now, card, d);
        }
    }

    /// A link found on a member outside `radio.use` (the drone parked
    /// there) moves to the best usable one once nothing else is moving it;
    /// a move that did not land is tried again after STEER_RETRY_MS.
    fn steer_into_use(&mut self, inp: &ChannelTickIn) {
        let usable = self.cfg.radio.usable();
        let op = self.plan.op();
        if self.pinned
            || !inp.in_session
            || usable.contains(&op)
            || self.plan.hopping()
            || self.plan.want() != op
            || self.boot_pick.open()
            || self.boot_pick.relocating()
            || self
                .steer_ms
                .is_some_and(|t| inp.now_ms - t < STEER_RETRY_MS)
        {
            return;
        }
        let ranked = self
            .scout
            .as_ref()
            .map(|s| s.pick_ranking())
            .unwrap_or_default();
        let to = ranked
            .into_iter()
            .find(|c| usable.contains(c))
            .unwrap_or(usable[0]);
        self.steer_ms = Some(inp.now_ms);
        self.plan.set_want(inp.now_ms, to);
        self.boot_pick.rearm_relocation();
        self.log(format!(
            "maburgs channel: link on {op} is outside radio.use: relocating to {to}"
        ));
    }

    fn step_boot_pick(&mut self, inp: &ChannelTickIn) {
        let now_ms = inp.now_ms;
        self.steer_into_use(inp);
        let mut bi = BootPickIn {
            now_ms,
            since_start_ms: self.now_ms_u_cur.wrapping_sub(self.gs_start_ms),
            max_ms: self.cfg.radio.scan.max_ms,
            in_session: inp.in_session,
            cal_running: inp.cal_running,
            one_card: self.one_card,
            scout_owns: self.scout_owns(),
            op: self.plan.op(),
            want: self.plan.want(),
            // Only radio.use (Rust only) narrows it: a link the grid search
            // found outside the set relocates as in mabur (relocate_due).
            op_usable: self.cfg.radio.use_.is_empty()
                || self.cfg.radio.use_.contains(&self.plan.op()),
            relocate_due: self.plan.relocate_due(),
            plan_hopping: self.plan.hopping(),
            hop_active: hop_active(inp.in_session, inp.cal_running),
            hop_idle_or_hold: matches!(self.hopc.state(), HopState::Idle | HopState::Hold),
            link_edge: self.link_edge_seen,
            scout_card_died: self.scout_card_died_seen,
            ..Default::default()
        };
        if let Some(s) = &self.scout {
            bi.scout_mature = s.mature();
            bi.scout_op_ranked = s.op_ranked();
            bi.scout_prelude_done = s.prelude_done();
            bi.proposal = s.proposal();
        }
        self.link_edge_seen = false;
        self.scout_card_died_seen = false;
        self.pending_relocate = None;
        self.last_bi = bi;
        let bo = self.boot_pick.tick(&bi);
        let op = self.plan.op();
        match bo {
            BootPickOut::Commit(ch) => {
                if ch != op {
                    self.log(format!("maburgs channel: commit {op} -> {ch} (no link)"));
                }
                self.plan.commit(now_ms, ch);
                self.freeze_pick(now_ms, "commit");
                self.frozen_pick = Some(self.plan.want());
            }
            BootPickOut::AckPrelude(ch) => {
                let linked = if inp.in_session || inp.cal_running {
                    ", linked: not committed"
                } else {
                    ""
                };
                self.log(format!(
                    "maburgs channel: one-card prelude ranking picks {} (op {op}){linked}",
                    bi.proposal
                ));
                if ch != 0 {
                    self.plan.commit(now_ms, ch);
                }
                if let Some(s) = &self.scout {
                    s.ack_prelude(self.plan.op());
                }
            }
            BootPickOut::WantPick(ch) => {
                self.plan.set_want(now_ms, ch);
                if let Some(s) = &self.scout {
                    s.freeze();
                }
                self.log(format!(
                    "maburgs channel: boot pick wants {ch}, link on {op}: relocating"
                ));
            }
            BootPickOut::Relocate(ch) => {
                self.pending_relocate = Some(ch);
                self.cal_reloc_tick =
                    self.cal_reloc.s == CalRelocState::Waiting && ch == self.cal_reloc.ch;
            }
            BootPickOut::Freeze { reason, accept_op } => {
                if accept_op {
                    self.plan.set_want(now_ms, op);
                }
                self.freeze_pick(now_ms, reason);
                self.frozen_pick = Some(self.plan.want());
            }
            BootPickOut::AcceptOp => {
                let want = self.plan.want();
                self.log(format!(
                    "maburgs channel: relocation to {want} did not land; staying on {op}"
                ));
                self.plan.set_want(now_ms, op);
            }
            BootPickOut::None => {}
        }
    }

    fn step_store(&mut self, vrx: &mut VrxController) {
        if self.plan.op() != self.saved_op {
            self.saved_op = self.plan.op();
            vrx.set_proposal(self.saved_op);
            if !(self.store)(self.saved_op) {
                let name = self.cfg.store_name.clone();
                self.log(format!("maburgs channel: could not write {name}"));
            }
        }
    }

    fn recovered_total(agg: Option<&Aggregator>) -> u64 {
        agg.map_or(0, |a| {
            a.decoder().stats(0).dec.syms_recovered + a.decoder().stats(1).dec.syms_recovered
        })
    }

    fn step_hop_edge_and_window(&mut self, inp: &ChannelTickIn, vrx: &mut VrxController) {
        let now_ms = inp.now_ms;
        let now_ms_u = self.now_ms_u_cur;
        let active = hop_active(inp.in_session, inp.cal_running);
        if active != self.hop_was_active {
            self.hop_was_active = active;
            if !active {
                let act = self.hopc.on_session_lost(now_ms, self.plan.op());
                self.dispatch_hop_action(&act, false, now_ms, vrx);
            }
            self.verdict.reset();
            self.last_verdict = Verdict::Healthy;
            self.last_verdict_out = VerdictOut::default();
            self.window_prev_ok.fill(false);
            self.recovered_prev_window = Self::recovered_total(inp.agg);
            self.verdict.new_session();
            self.live.sampler.reset(now_ms_u);
        }
        if !active {
            return;
        }
        if self.cfg.threaded {
            // One thread for the window's card reads, started once the hop
            // is live; it stays up across sessions and samples only while
            // active.
            if !self.sampler_started {
                self.sampler_started = true;
                let (nm, sl) = (self.now_ms.clone(), self.live.sleep.clone());
                self.live
                    .sampler
                    .start(Box::new(move || nm()), Box::new(move |ms| sl(ms)));
            }
        } else {
            self.live.sampler.step(now_ms_u);
        }
        let Some(rec) = self.live.sampler.take() else {
            return;
        };
        // The sample instant: every per-window span is measured to it.
        let t_ms = rec.t_ms;
        let mut vc = vec![VerdictCardIn::default(); self.n_cards];
        // The worst valid card's reading of op this window.
        let mut op_read: Option<CalReading> = None;
        for (si, cs) in rec.cards.iter().enumerate().take(self.n_cards) {
            if !cs.valid || cs.channel != self.plan.op() {
                self.window_prev_ok[si] = false;
                continue;
            }
            let ct = inp.agg.map(|a| &a.cards()[si]);
            let crc_fail = ct.map_or(0, |c| c.crc_fail);
            let seq_exp = ct.map_or(0, |c| c.seq_expected);
            let seq_rcv = ct.map_or(0, |c| c.seq_received);
            // The pooled EMA: the per-frame best of the two receive chains,
            // so a card still hearing the drone on one antenna is not weak.
            let rssi_raw = ct.map_or(0.0, |c| c.rf.rssi_ema);
            let snr_raw = ct.map_or(0.0, |c| c.rf.snr_ema);
            if self.window_prev_ok[si] {
                let prev = self.window_prev[si];
                let own = cs.frames.own.wrapping_sub(prev.own);
                let foreign = cs.frames.foreign.wrapping_sub(prev.foreign);
                let win_us = t_ms.wrapping_sub(self.window_prev_ms[si]) as f64 * 1000.0;
                // Own airtime is what the drone sent, not what this card
                // decoded: scaled by the window's delivery (the drone's seq
                // walk counts every frame it sent), floored at 1 (a stalled
                // walk reads under 1). A window that decoded nothing has no
                // ratio, so it gives no busy reading at all (mabur
                // channel_core.cpp).
                let d_exp = seq_exp.wrapping_sub(self.window_prev_seq_exp[si]);
                let d_rcv = seq_rcv.wrapping_sub(self.window_prev_seq_rcv[si]);
                let delivery = if d_rcv > 0 {
                    (d_exp as f64 / d_rcv as f64).max(1.0)
                } else {
                    1.0
                };
                let own_pct = if win_us > 0.0 {
                    (100.0 * delivery * cs.frames.own_air_us.wrapping_sub(prev.own_air_us) as f64
                        / win_us)
                        .min(100.0)
                } else {
                    0.0
                };
                let busy_pct = if d_rcv == 0 { None } else { cs.nhm_busy_pct };
                let snr_ok = self.snr_ok[si];
                vc[si] = VerdictCardIn {
                    valid: true,
                    fa: cs.energy.fa_ofdm,
                    cca: cs.energy.cca_ofdm,
                    foreign: foreign as u32,
                    crc_fail: crc_fail.wrapping_sub(self.window_prev_crc[si]) as u32,
                    rssi_dbm: rssi_raw_to_dbm(rssi_raw),
                    snr_db: if snr_ok {
                        snr_raw_to_db(snr_raw)
                    } else {
                        f64::NAN
                    },
                    snr_valid: snr_ok,
                    busy_valid: busy_pct.is_some(),
                    nhm_busy_pct: busy_pct.unwrap_or(0.0),
                    own_air_pct: own_pct,
                };
                self.energy_last[si] = Some(StatsEnergyIn {
                    cca: cs.energy.cca_ofdm,
                    fa: cs.energy.fa_ofdm,
                    own,
                    foreign,
                    igi: None,
                    busy_pct,
                    own_air_pct: Some(own_pct),
                });
                // Op's reading for a calibration run's pick (the scout never
                // dwells on op while linked): foreign frames scaled from this
                // window's span to one dwell_ms observe, and the foreign
                // share of the NHM busy airtime. The worst card on each term,
                // as the scout keeps a member's worst visit.
                if win_us > 0.0 {
                    let k = f64::from(self.cfg.radio.scan.dwell_ms) * 1000.0 / win_us;
                    let f = (foreign as f64 * k).round() as u32;
                    let r = op_read.get_or_insert(CalReading {
                        ch: self.plan.op(),
                        ..CalReading::default()
                    });
                    r.foreign = r.foreign.max(f);
                    if let Some(b) = busy_pct {
                        r.busy_valid = true;
                        r.busy_pct = r.busy_pct.max((b - own_pct).max(0.0));
                    }
                }
            }
            self.window_prev[si] = cs.frames;
            self.window_prev_crc[si] = crc_fail;
            self.window_prev_seq_exp[si] = seq_exp;
            self.window_prev_seq_rcv[si] = seq_rcv;
            self.window_prev_ms[si] = t_ms;
            self.window_prev_ok[si] = true;
        }
        if op_read.is_some() {
            self.op_live = op_read;
            self.op_live_ms = now_ms;
        }
        let recovered_now = Self::recovered_total(inp.agg);
        let vl = VerdictLinkIn {
            pre_fec_loss: hop_verdict_link_loss(
                self.s1_hop_loss.sample(now_ms),
                self.enh_hop_loss.sample(now_ms),
            ),
            recovered: recovered_now.wrapping_sub(self.recovered_prev_window) as u32,
        };
        self.recovered_prev_window = recovered_now;
        let vo = self.verdict.window(now_ms, &vc, &vl, vrx.ctl().rung());
        if vo.v != Verdict::Healthy || vo.v != self.last_verdict {
            self.sink.verdict(now_ms, &vo, &vc, &vl);
        }
        self.last_verdict = vo.v;
        self.last_verdict_out = vo;
        if let Some(b) = hop_store_blank_until(&vo, self.reactive, self.cfg.hop.confirm_ms) {
            vrx.blank_store(b);
        }
    }

    fn fill_hop_targets(&self, k: &mut HopTick, now_ms: f64) {
        let op = self.plan.op();
        k.best = self
            .ranker
            .best(now_ms, op, &self.hopc.backed_off(now_ms), true);
        k.escape = self
            .ranker
            .best(now_ms, op, &self.hopc.backed_off_failed(now_ms), true);
        k.best_score = 0;
        k.escape_score = 0;
        for e in self.ranker.ranking(now_ms) {
            if k.best == Some(e.ch) {
                k.best_score = e.score;
            }
            if k.escape == Some(e.ch) {
                k.escape_score = e.score;
            }
        }
    }

    /// The freshness burst on `card`, here and synchronously: every
    /// candidate once, back to op.
    fn burst(&mut self, card: usize, now_ms: f64) {
        let live = self.live.clone();
        let mut recs = Vec::new();
        {
            let ilk = live.inflight.lock().unwrap();
            if ilk.is_none() {
                return;
            }
            for v in live.burst_on_card(ilk, card, self.plan.op(), &mut recs) {
                self.ranker.add(v);
            }
        }
        self.cur_ch[card] = live.cards[card].channel();
        for d in &recs {
            self.sink.dwell(now_ms, card, d);
        }
    }

    /// The in-flight thread: started once the boot pick is closed and the
    /// boot scout owns no card. Two cards, auto mode only: pinned never
    /// ranks (nothing to move to). Also checked before the freshness burst:
    /// the tick the boot scout lets go of its card may burst, and that
    /// burst belongs on this thread.
    fn start_inflight_if_due(&mut self) {
        if self.inflight_started
            || !self.reactive
            || self.scout_owns()
            || self.boot_pick.open()
            || self.n_cards < 2
        {
            return;
        }
        self.inflight_started = true;
        if self.cfg.threaded {
            self.live.scout_run.store(true, Relaxed);
            let live = self.live.clone();
            self.inflight_thread = Some(std::thread::spawn(move || live.scout_loop()));
        }
    }

    fn step_controller(&mut self, inp: &ChannelTickIn, vrx: &mut VrxController) {
        let now_ms = inp.now_ms;
        if !hop_active(inp.in_session, inp.cal_running) {
            self.pending_relocate = None;
            self.cal_reloc_tick = false;
            return;
        }
        let mut ht = HopTick {
            now_ms,
            verdict: self.last_verdict_out,
            cur_op: self.plan.op(),
            ..Default::default()
        };
        if self.hopc.state() != HopState::Ordered {
            self.hop_lead_latched = pick_hop_lead(&self.ready_mask(), inp.tx_card);
        }
        ht.lead_card = self.hop_lead_latched;
        ht.n_cards = if ht.lead_card.is_some() {
            self.n_cards
        } else {
            1
        };
        self.fill_hop_targets(&mut ht, now_ms);
        ht.video_on_target = self.plan.hopping() && self.last_video_ch == self.plan.hop_target();
        ht.rcf_sent_since_order = self.rcf_sent_total.wrapping_sub(self.rcf_sent_at_order) as i32;
        let relocation_owns = self.boot_pick.relocating()
            || self.pending_relocate.is_some()
            || self.boot_pick.relocation_pending(&self.last_bi);
        // No relay sweeps here: a burst card is always a USB card.
        let burst_card = pick_burst_card(&self.live.can_scout, &[], inp.tx_card);
        let op = self.plan.op();
        let ranking_complete = self
            .ranker
            .ranking(now_ms)
            .iter()
            .all(|e| e.ch == op || e.ranked);
        let burst_period =
            hop_burst_period_ms(self.hopc.state(), false, ranking_complete, &self.cfg.hop);
        let spare_usb_burst = burst_card.is_some_and(|c| c != inp.tx_card);
        self.start_inflight_if_due();
        if self.reactive
            && !self.scout_owns()
            && !relocation_owns
            && !self.burst_inflight
            && hop_burst_wanted(spare_usb_burst, ranking_complete)
            && hop_burst_due(
                self.hopc.state(),
                self.last_verdict_out.trigger,
                now_ms,
                self.last_burst_ms,
                burst_period,
            )
        {
            self.last_burst_ms = now_ms;
            match burst_card {
                Some(card) if spare_usb_burst && self.cfg.threaded && self.inflight_started => {
                    // The spare card's burst runs on the in-flight thread,
                    // which owns that card's dwells anyway: the core loop
                    // keeps draining and publishing the TX card's video
                    // while it runs. step_drains ranks the results; the
                    // trigger is held back below until it has.
                    self.live.burst_req_op.store(op, Relaxed);
                    self.live.burst_req_card.store(card as i32, Relaxed);
                    self.burst_inflight = true;
                    self.burst_posted_ms = now_ms;
                }
                // A one-card GS (the burst card is the TX card, off op
                // either way) and a core without threads run it here.
                Some(card) => self.burst(card, now_ms),
                None => {}
            }
            self.fill_hop_targets(&mut ht, now_ms);
        }
        // A posted burst that never came back (the in-flight thread
        // stopped) must not hold the trigger forever.
        if self.burst_inflight && now_ms - self.burst_posted_ms >= BURST_LOST_MS {
            self.burst_inflight = false;
            self.live.burst_req_card.store(-1, Relaxed);
            let lost = now_ms - self.burst_posted_ms;
            self.log(format!("maburgs hop: posted burst lost after {lost:.0} ms"));
        }
        // A posted burst in flight: the ranking is about to change; holding
        // the trigger back keeps the controller out of hold_exhausted (and
        // its order off the bursting card) until it lands.
        if self.burst_inflight {
            ht.verdict.trigger = false;
            ht.best = None;
            ht.escape = None;
        }
        // Pinned: the verdict is still measured (OSD/sideport read it) but
        // the controller never sees a reactive trigger. Relocation re-arms it
        // on its own tick.
        if !self.reactive {
            ht.verdict.trigger = false;
        }
        if self.scout_owns() {
            ht.best = None;
            ht.escape = None;
        } else if let Some(to) = self.pending_relocate {
            ht.verdict.trigger = true;
            ht.relocate = true;
            // Onto the pin, or where a calibration run is about to
            // calibrate: the confirm lands it, no verify (the run's
            // cal_running would cut the verify window short anyway).
            ht.no_verify = !self.reactive || self.cal_reloc_tick;
            ht.best = Some(to);
            ht.best_score = 0;
            ht.escape = None;
            ht.escape_score = 0;
        } else if relocation_owns || !self.reactive {
            ht.best = None;
            ht.escape = None;
        }
        let act = self.hopc.tick(&ht);
        self.dispatch_hop_action(&act, ht.relocate, now_ms, vrx);
        if let (true, Some(to)) = (ht.relocate, self.pending_relocate) {
            let op = self.plan.op();
            let what = if act.kind == HopActionKind::Order {
                "placed"
            } else {
                "refused (hop cap)"
            };
            let cal = if self.cal_reloc_tick { " (cal)" } else { "" };
            self.log(format!(
                "maburgs channel: relocate {op} -> {to} {what}{cal}"
            ));
        }
        self.pending_relocate = None;
        self.cal_reloc_tick = false;
    }

    // ---- a calibration run's channel pick (CalPicker; core thread) ----
    //
    // begin_cal_measure arms one measure round of the boot scout: once the
    // boot pick has closed and nothing else holds the scout card, a hop or
    // the controller, the scout measures every 20 MHz half of the set once
    // on a fresh ranker, skipping op's own halves (they carry the drone's
    // video). Op is read from the verdict window instead. The pick
    // (cal_round_pick) is logged as a K record. The round needs the scout
    // thread up and its card alive, is dropped once a run already
    // calibrates, and ends at a bound (twice a full round's dwell time)
    // with op as the answer. order_cal_relocate is the boot pick's own
    // relocate order (reason cal), landed by the confirm without a verify
    // window. Pinned or one card: nothing is measured, op is the answer.

    /// Auto mode with a spare card that can measure (two or more cards).
    pub fn cal_can_measure(&self) -> bool {
        self.scout.is_some() && !self.pinned && !self.one_card && self.n_cards >= 2
    }

    /// Arms one measure round (restarts nothing while one is pending or
    /// running). Not measurable: a no-op, cal_measure_best() is op.
    pub fn begin_cal_measure(&mut self, now_ms: f64) {
        if !matches!(
            self.cal_reloc.s,
            CalRelocState::Waiting | CalRelocState::InFlight
        ) {
            self.cal_reloc = CalReloc::default();
        }
        if !self.cal_can_measure()
            || matches!(
                self.cal_meas.s,
                CalMeasureState::Waiting | CalMeasureState::Running
            )
        {
            return;
        }
        self.cal_meas = CalMeasure {
            s: CalMeasureState::Waiting,
            t0: now_ms,
            ..CalMeasure::default()
        };
    }

    /// The run is done with the pick: drop a pending round, end a running
    /// one (the scout parks), stop tracking a cal relocate (one not yet
    /// placed is withdrawn by want following op).
    pub fn cancel_cal_measure(&mut self) {
        if self.cal_meas.s == CalMeasureState::Running {
            if let Some(s) = &self.scout {
                s.cancel_round(!self.scout_started);
            }
            self.log("maburgs channel: calibration measure round cancelled".into());
        }
        self.cal_meas = CalMeasure::default();
        if self.cal_reloc.s == CalRelocState::Waiting && !self.plan.hopping() {
            let op = self.plan.op();
            self.plan.set_want(self.now_ms_cur, op);
        }
        self.cal_reloc = CalReloc::default();
        self.cal_reloc_tick = false;
    }

    /// The round's best member; None while it waits or runs, op when the
    /// GS cannot measure. A best other than op is held back until the
    /// scout has parked its card again: the round stays open until then,
    /// and the relocate the run orders next would be refused (scout_owns).
    /// maburgs answers at MEASURED and races its scout thread there.
    pub fn cal_measure_best(&self) -> Option<u8> {
        if !self.cal_can_measure() {
            return Some(self.plan.op());
        }
        let parked = !self.scout.as_ref().is_some_and(|s| s.round_open());
        let best = self.cal_meas.best;
        (self.cal_meas.s == CalMeasureState::Done && (parked || best == self.plan.op()))
            .then_some(best)
    }

    /// The boot pick's relocate toward ch, reason cal. false = refused: not
    /// a member, already op, link not live, a hop or relocation in flight
    /// or the pick still open, the scout on its card, the hop rate limit.
    pub fn order_cal_relocate(&mut self, ch: u8) -> bool {
        let now_ms = self.now_ms_cur;
        if matches!(
            self.cal_reloc.s,
            CalRelocState::Waiting | CalRelocState::InFlight
        ) || !self.plan.member(ch)
            || ch == self.plan.op()
            || !hop_active(self.in_session_cur, self.cal_running_cur)
            || self.plan.hopping()
            || matches!(self.hopc.state(), HopState::Ordered | HopState::Verifying)
            || self.boot_pick.open()
            || self.boot_pick.relocating()
            || self.scout_owns()
        {
            return false;
        }
        if self.hopc.hop_cap_reached(now_ms) {
            self.log(format!(
                "maburgs channel: calibration relocate to {ch} refused: hop cap"
            ));
            return false;
        }
        self.plan.set_want(now_ms, ch);
        self.boot_pick.rearm_relocation();
        self.cal_reloc = CalReloc {
            s: CalRelocState::Waiting,
            ch,
            t0: now_ms,
            ..CalReloc::default()
        };
        let op = self.plan.op();
        self.log(format!(
            "maburgs channel: calibration wants {ch}, link on {op}: relocating"
        ));
        true
    }

    /// true once the confirm landed it, false when withdrawn, held at the
    /// cap, lost with the session or never placed; None pending / none.
    pub fn cal_relocate_settled(&self) -> Option<bool> {
        (self.cal_reloc.s == CalRelocState::Settled).then_some(self.cal_reloc.ok)
    }

    fn settle_cal_reloc(&mut self, ok: bool, why: &str) {
        self.cal_reloc.s = CalRelocState::Settled;
        self.cal_reloc.ok = ok;
        let ch = self.cal_reloc.ch;
        self.log(format!(
            "maburgs channel: calibration relocate to {ch} {why}"
        ));
    }

    fn step_cal(&mut self, inp: &ChannelTickIn) {
        let now_ms = inp.now_ms;
        let active = hop_active(inp.in_session, inp.cal_running);
        if self.cal_reloc.s == CalRelocState::Waiting
            && (!active || now_ms - self.cal_reloc.t0 > self.cfg.hop.confirm_ms as f64)
        {
            // BootPick never handed the order out: nothing will move, so
            // want follows op.
            self.settle_cal_reloc(false, "never placed");
            if !self.plan.hopping() {
                let op = self.plan.op();
                self.plan.set_want(now_ms, op);
            }
        } else if self.cal_reloc.s == CalRelocState::InFlight
            && self.cal_reloc.confirmed
            && self.hopc.state() == HopState::Idle
        {
            // Confirmed, then the session's falling edge dropped the verify
            // state before its VerifyPass: the link is on the target.
            self.settle_cal_reloc(true, "landed");
        }
        // A round still pending once the run calibrates has missed its
        // moment: it must never take the scout card later.
        if self.cal_meas.s == CalMeasureState::Waiting && inp.cal_running {
            self.cal_meas = CalMeasure::default();
            self.log(
                "maburgs channel: calibration measure round dropped: a run is calibrating".into(),
            );
        }
        let Some(scout) = self.scout.clone() else {
            return;
        };
        let sci = self.scout_card.expect("a scout has a card");
        // Only with the scout thread up and its card alive: a round on a
        // scout that never steps would hold the card for good.
        if self.cal_meas.s == CalMeasureState::Waiting
            && active
            && self.scout_started
            && !self.scout_card_down
            && !self.scout_owns()
            && !self.boot_pick.open()
            && !self.boot_pick.relocating()
            && !self.plan.hopping()
            && !self.dwell_busy()
            && matches!(self.hopc.state(), HopState::Idle | HopState::Hold)
        {
            {
                // inflight then the card's device-use lock, the order every
                // holder of both follows: no in-flight dwell is on the scout
                // card while it changes hands, and the sampler sees the card
                // blocked before the scout's first retune lands under it.
                let live = self.live.clone();
                let _ilk = live.inflight.lock().unwrap();
                let _hold = live.sampler.hold_card(sci);
                scout.begin_round();
                self.live.scout_working.store(true, Relaxed);
                self.step_gates();
            }
            self.cal_meas.s = CalMeasureState::Running;
            self.cal_meas.t0 = now_ms;
            // Twice a full round's dwell time (every member, op's included,
            // at dwell_ms + settle_ms; one dwell per pair at 40), never under
            // a second.
            let n = self.cfg.radio.channels.len();
            let scan = &self.cfg.radio.scan;
            self.cal_meas.bound_ms =
                (2.0 * n as f64 * f64::from(scan.dwell_ms + scan.settle_ms)).max(1000.0);
            let op = self.plan.op();
            self.log(format!(
                "maburgs channel: calibration measure round on card {sci} (op {op})"
            ));
        } else if self.cal_meas.s == CalMeasureState::Running && !scout.round_measuring() {
            self.finish_cal_measure(now_ms, &scout);
        } else if self.cal_meas.s == CalMeasureState::Running
            && now_ms - self.cal_meas.t0 >= self.cal_meas.bound_ms
        {
            scout.cancel_round(!self.scout_started);
            self.cal_meas.s = CalMeasureState::Done;
            self.cal_meas.best = self.plan.op();
            let (b, op) = (self.cal_meas.bound_ms, self.plan.op());
            self.log(format!(
                "maburgs channel: calibration measure round overran {b:.0} ms; staying on {op}"
            ));
        }
    }

    fn finish_cal_measure(&mut self, now_ms: f64, scout: &Scout) {
        let op = self.plan.op();
        // Op's row is the verdict's live reading taken during the round;
        // without one op is unmeasured and a working link is not moved on a
        // one-sided comparison.
        let have_op = self.op_live.is_some() && self.op_live_ms >= self.cal_meas.t0;
        let members: Vec<CalReading> = scout
            .round_samples()
            .iter()
            .map(|s| CalReading {
                ch: s.ch,
                foreign: s.foreign,
                busy_valid: s.busy_valid,
                busy_pct: s.busy_pct,
            })
            .collect();
        let scan = &self.cfg.radio.scan;
        let pick = cal_round_pick(
            &members,
            if have_op { self.op_live } else { None },
            op,
            &self.cfg.radio.channels,
            f64::from(scan.pick_margin_pct),
            scan.busy.blocked_pct,
        );
        self.sink
            .pick(now_ms, Some(pick.best), scout.rounds(), &pick.rows, 1);
        let unread = if have_op { "" } else { " unread: staying" };
        self.log(format!(
            "maburgs channel: calibration measure picks {} (op {op}{unread})",
            pick.best
        ));
        self.cal_meas.s = CalMeasureState::Done;
        self.cal_meas.best = pick.best;
    }

    fn step_move_edge(&mut self, now_ms: f64, vrx: &mut VrxController) {
        vrx.set_proposal(self.plan.op());
        vrx.set_keepalive_hold(self.hopc.state() == HopState::Ordered);
        // The edge is held through a hop in flight and replayed once it is
        // over; agreed is read at replay time. on_ack() with agreed == op is
        // a no-op; a member agreed != op (after a withdrawn hop) moves op as
        // a normal ack move. Held, not consumed: an edge landing on a
        // hopping tick would otherwise be dropped.
        if self.move_hold.take(
            vrx.take_move_edge(),
            self.cal_running_cur || self.plan.hopping(),
        ) {
            let proposed = vrx.proposal();
            let agreed = vrx.agreed_channel();
            if !self.plan.member(agreed) && agreed != self.plan.op() {
                self.log(format!(
                    "maburgs channel: drone acked {agreed}, not in our set; ignored"
                ));
            }
            self.plan.on_ack(now_ms, agreed, proposed);
            vrx.set_proposal(self.plan.op());
            self.link_edge_seen = true;
        }
    }

    fn step_drains(&mut self) {
        if let (Some(s), Some(sci)) = (self.scout.clone(), self.scout_card) {
            for d in s.take_dwells() {
                self.sink.dwell(self.now_ms_cur, sci, &d);
            }
        }
        self.start_inflight_if_due();
        let drained = std::mem::take(&mut *self.live.drained.lock().unwrap());
        if drained.burst_done {
            // Its visits are ranked below, before the next controller step.
            self.burst_inflight = false;
        }
        let mut visits = Vec::new();
        for (card, d, v) in drained.recs {
            self.cur_ch[card] = self.live.cards[card].channel();
            self.sink.dwell(self.now_ms_cur, card, &d);
            let ok = d.flags & FLAG_RETUNE_FAILED == 0;
            let mut ds = self.dwell_stats[card].unwrap_or_default();
            ds.visits += 1;
            ds.cost_us = (d.to_us + d.read_us + d.back_us) as u32;
            if let (true, Some(v)) = (ok, &v) {
                ds.score = HopRanker::score(v);
            }
            self.dwell_stats[card] = Some(ds);
            visits.extend(v);
        }
        for v in visits {
            self.ranker.add(v);
        }
    }

    /// A boot pick or a calibration round in flight: the scout card's width
    /// is the scan's, not the link's.
    fn scan_holds_width(&self) -> bool {
        !width_resync_open(
            self.scout.is_some(),
            self.boot_pick.open(),
            self.scout.as_ref().is_some_and(|s| s.round_open()),
        )
    }

    fn step_width_resync(&mut self) {
        if self.scan_holds_width() {
            return;
        }
        // The scout thread drives only its own card, so while it searches
        // every other card is fixed here and the scout card is left to it.
        let scout_busy = self.scout_owns();
        let w = self.width_mhz;
        let live = self.live.clone();
        for (i, fe) in live.cards.iter().enumerate() {
            let scouting = scout_busy && self.is_scout_card(i);
            if self.width_retry_ms[i] != 0.0 && self.now_ms_cur >= self.width_retry_ms[i] {
                self.width_tried[i] = false;
                self.width_retry_ms[i] = 0.0;
            }
            let wc = WidthCard {
                ready: fe.ready(),
                width: fe.width(),
                tried: self.width_tried[i],
                busy: live.dwelling(i),
                scouting,
            };
            if needs_width_fix(&wc, w) {
                self.width_tried[i] = true;
                let _ilk = live.inflight.lock().unwrap();
                let ch = fe.channel();
                if fe.set_width(ch, w) {
                    self.cur_ch[i] = fe.channel();
                    live.gates.tune_gen[i].fetch_add(1, std::sync::atomic::Ordering::Release);
                } else {
                    drop(_ilk);
                    self.log(format!(
                        "maburgs radio: card {i} width resync to {w} MHz on ch {ch} failed"
                    ));
                    // 40 on a channel with no pair cannot work until the channel changes.
                    if w != 40 || ht40_offset(ch) != 0 {
                        self.width_retry_ms[i] = self.now_ms_cur + WIDTH_RETRY_MS;
                    }
                }
            }
            // The scout's dwells, ranks and picks are as wide as its card.
            // Told only while the scout has no work, so no dwell is in flight.
            if self.is_scout_card(i) && !scouting && fe.ready() {
                if let Some(s) = &self.scout {
                    s.set_link_width_mhz(fe.width());
                }
            }
        }
    }

    fn step_mechanical_retune(&mut self) {
        let owns = self.scout_owns();
        let live = self.live.clone();
        for (i, fe) in live.cards.iter().enumerate() {
            if (owns && self.is_scout_card(i)) || live.dwelling(i) || !fe.ready() {
                continue;
            }
            let want = self.plan.desired(i);
            if self.cur_ch[i] != want && fe.retune(want) {
                self.cur_ch[i] = want;
                live.gates.tune_gen[i].fetch_add(1, std::sync::atomic::Ordering::Release);
            }
        }
    }
}

impl Drop for ChannelCore {
    fn drop(&mut self) {
        self.shutdown();
    }
}

#[cfg(test)]
#[path = "core_tests.rs"]
mod tests;
