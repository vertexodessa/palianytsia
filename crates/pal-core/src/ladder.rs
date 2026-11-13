//! The rung ladder (mabur `gs/src/ladder_controller.cpp`, `rung_store.cpp`):
//! demote on residual loss, s1/s3 utilization, a predictive fade trigger or
//! video starvation; promote after a clean run, gated by the probe stream and
//! by per-rung probation penalties.
use crate::config::{LadderCfg, Rung};

#[derive(Clone, Copy, Debug, Default)]
pub struct LinkHealth {
    pub sample_valid: bool,
    pub pre_fec_loss: f64,
    pub residual_loss: f64,
    pub video_starved: bool,
    pub s3_valid: bool,
    pub s3_pre_fec_loss: f64,
    pub s3_residual_loss: f64,
    pub s3_expected_syms: u64,
    pub rf_snr_db: f64,
    pub rf_evm_db: f64,
    pub rf_rssi_dbm: f64,
    pub probe_valid: bool,
    pub probe_loss: f64,
    pub probe_expected_syms: u64,
    pub probe_bodies_total: u64,
    pub probe_rung: i32,
}

impl LinkHealth {
    /// mabur's default: every RF reading NaN, no probe rung.
    pub fn empty() -> Self {
        Self {
            rf_snr_db: f64::NAN,
            rf_evm_db: f64::NAN,
            rf_rssi_dbm: f64::NAN,
            probe_rung: -1,
            ..Self::default()
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum CtlReason {
    #[default]
    None,
    Residual,
    Util,
    Probation,
    Starved,
    Timeout,
    Promote,
    S3Residual,
    S3Util,
    Fade,
    PromoteProbed,
    HopRestore,
}

impl CtlReason {
    pub fn as_str(self) -> &'static str {
        use CtlReason::*;
        match self {
            None => "none",
            Residual => "residual",
            Util => "util",
            Probation => "probation",
            Starved => "starved",
            Timeout => "timeout",
            Promote => "promote",
            S3Residual => "s3_residual",
            S3Util => "s3_util",
            Fade => "fade",
            PromoteProbed => "promote_probed",
            HopRestore => "hop_restore",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ProbeGateState {
    #[default]
    Off,
    NoInfo,
    Clean,
    Lossy,
}

impl ProbeGateState {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Off => "off",
            Self::NoInfo => "noinfo",
            Self::Clean => "clean",
            Self::Lossy => "lossy",
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ProbeGate {
    pub state: ProbeGateState,
    pub rung: i32,
    pub u: f64,
    pub streak_bodies: u64,
}

/// `LadderController::up_status`.
#[derive(Clone, Copy, Debug)]
pub struct UpStatus {
    pub u: f64,
    pub up_util: f64,
    /// How long s1 utilization has stayed under up_util (0 = not now).
    pub clean_ms: f64,
    pub clean_need_ms: f64,
    pub probe: ProbeGate,
    pub probe_max_util: f64,
    pub probe_need: u64,
    /// Remaining block on the next rung after a failed promote.
    pub next_penalty_ms: i32,
    /// Remaining loss lockout on the next rung; -1 = latched, 0 = none.
    pub next_lock_ms: i32,
    /// Loss demotes the next rung has taken since the last clear.
    pub next_strikes: i32,
}

/// `LadderController::locked`: one rung the loss lockout holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Lockout {
    pub rung: i32,
    /// -1 = latched.
    pub ms_left: i32,
    pub strikes: i32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CtlEvent {
    pub t_ms: f64,
    pub from: i32,
    pub to: i32,
    pub reason: CtlReason,
    pub u: f64,
    pub snr_db: f64,
    pub evm_db: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CtlCounters {
    pub demotes_residual: u64,
    pub demotes_util: u64,
    pub promotes: u64,
    pub probation_fails: u64,
    pub starved_drops: u64,
    pub timeout_drops: u64,
    pub promotes_probed: u64,
    pub probe_holds: u64,
    pub demotes_s3_residual: u64,
    pub demotes_s3_util: u64,
    pub demotes_fade: u64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Ewma {
    pub v: f64,
    pub n: u64,
}

impl Ewma {
    fn add(&mut self, x: f64, alpha: f64) {
        self.v = if self.n == 0 {
            x
        } else {
            self.v + alpha * (x - self.v)
        };
        self.n += 1;
    }
}

#[derive(Clone, Copy, Debug)]
pub struct RungStat {
    pub u: Ewma,
    pub resid: Ewma,
    pub u3: Ewma,
    pub s3_resid: Ewma,
    pub probe_u: Ewma,
    pub evm_db: f64,
    pub evm_var_db2: f64,
    pub evm_n: u64,
    pub last_sample_ms: f64,
    pub last_probe_ms: f64,
    pub dwell_ms: f64,
    pub visits: u32,
    pub exits_bad: u32,
}

impl Default for RungStat {
    fn default() -> Self {
        Self {
            u: Ewma::default(),
            resid: Ewma::default(),
            u3: Ewma::default(),
            s3_resid: Ewma::default(),
            probe_u: Ewma::default(),
            evm_db: f64::NAN,
            evm_var_db2: 0.0,
            evm_n: 0,
            last_sample_ms: -1.0,
            last_probe_ms: -1.0,
            dwell_ms: 0.0,
            visits: 0,
            exits_bad: 0,
        }
    }
}

/// Per-rung history (observability; the ladder does not read it back).
pub struct RungStore {
    stats: Vec<RungStat>,
    alpha: f64,
    current: usize,
    parked_since_ms: f64,
}

fn clamp_util(u: f64) -> f64 {
    u.min(1e3)
}

impl RungStore {
    fn new(n: usize, half_life_samples: i32) -> Self {
        Self {
            stats: vec![RungStat::default(); n],
            alpha: 1.0 - 0.5f64.powf(1.0 / f64::from(half_life_samples)),
            current: 0,
            parked_since_ms: -1.0,
        }
    }
    fn observe_s1(&mut self, r: usize, u: f64, residual: bool, now: f64) {
        let a = self.alpha;
        let s = &mut self.stats[r];
        s.u.add(clamp_util(u), a);
        s.resid.add(if residual { 1.0 } else { 0.0 }, a);
        s.last_sample_ms = now;
        if self.parked_since_ms < 0.0 {
            self.parked_since_ms = now;
        }
    }
    fn observe_evm(&mut self, r: usize, evm: f64) {
        let a = self.alpha;
        let s = &mut self.stats[r];
        if s.evm_n == 0 {
            s.evm_db = evm;
            s.evm_var_db2 = 0.0;
        } else {
            let d = evm - s.evm_db;
            let incr = a * d;
            s.evm_db += incr;
            s.evm_var_db2 = (1.0 - a) * (s.evm_var_db2 + d * incr);
        }
        s.evm_n += 1;
    }
    fn observe_s3(&mut self, r: usize, u3: f64, resid: bool, now: f64) {
        let a = self.alpha;
        let s = &mut self.stats[r];
        s.u3.add(clamp_util(u3), a);
        s.s3_resid.add(if resid { 1.0 } else { 0.0 }, a);
        s.last_sample_ms = now;
    }
    fn observe_probe(&mut self, r: usize, u: f64, now: f64) {
        let a = self.alpha;
        let s = &mut self.stats[r];
        s.probe_u.add(clamp_util(u), a);
        s.last_probe_ms = now;
    }
    fn on_transition(&mut self, from: usize, to: usize, reason: CtlReason, now: f64) {
        if from == to {
            return;
        }
        if self.parked_since_ms >= 0.0 {
            self.stats[from].dwell_ms += now - self.parked_since_ms;
        }
        self.parked_since_ms = now;
        self.current = to;
        self.stats[to].visits += 1;
        if matches!(
            reason,
            CtlReason::Residual | CtlReason::S3Residual | CtlReason::Probation | CtlReason::Starved
        ) {
            self.stats[from].exits_bad += 1;
        }
    }
    pub fn stat(&self, r: usize) -> &RungStat {
        &self.stats[r]
    }
    pub fn len(&self) -> usize {
        self.stats.len()
    }
    pub fn is_empty(&self) -> bool {
        self.stats.is_empty()
    }
    pub fn dwell_ms(&self, r: usize, now: f64) -> f64 {
        let mut d = self.stats[r].dwell_ms;
        if r == self.current && self.parked_since_ms >= 0.0 {
            d += now - self.parked_since_ms;
        }
        d
    }
}

#[derive(Clone, Copy, Default)]
struct FadeEwma {
    fast: f64,
    slow: f64,
    last_ms: f64,
    has: bool,
}

impl FadeEwma {
    fn feed(&mut self, v: f64, now: f64) {
        if !self.has {
            self.fast = v;
            self.slow = v;
            self.last_ms = now;
            self.has = true;
            return;
        }
        let dt = (now - self.last_ms).max(1.0);
        self.last_ms = now;
        self.fast += (1.0 - (-dt / 300.0).exp()) * (v - self.fast);
        let tau = if v >= self.slow { 2000.0 } else { 20000.0 };
        self.slow += (1.0 - (-dt / tau).exp()) * (v - self.slow);
    }
    fn delta(&self) -> f64 {
        if self.has {
            self.slow - self.fast
        } else {
            f64::NAN
        }
    }
}

pub struct LadderController {
    cfg: LadderCfg,
    store: RungStore,
    idx: i32,
    u: f64,
    pre_fec_loss: f64,
    last_feedback_ms: f64,
    measured_rung: i32,
    starved_since_ms: f64,
    last_change_ms: f64,
    last_down_ms: f64,
    confirm_start_ms: f64,
    clean_start_ms: f64,
    fade_until_ms: f64,
    fade_rssi: FadeEwma,
    fade_snr: FadeEwma,
    fade_trig_start_ms: f64,
    fade_latched: bool,
    probation_active: bool,
    probation_until_ms: f64,
    probation_rung: i32,
    fail_count: Vec<i32>,
    penalty_until: Vec<f64>,
    strikes: Vec<i32>,
    lock_until: Vec<f64>,
    probe_state: ProbeGateState,
    probe_state_since_ms: f64,
    probe_streak_open: bool,
    probe_streak_start_bodies: u64,
    probe_bodies_total: u64,
    probe_last_sample_ms: f64,
    probe_u: f64,
    probe_hold_active: bool,
    u3: f64,
    s3_util_start_ms: f64,
    s3_last_live_ms: f64,
    s3_blank_until_ms: f64,
    blank_store_until_ms: f64,
    snr_now: f64,
    evm_now: f64,
    counters: CtlCounters,
    last_event: CtlEvent,
}

const NEVER: f64 = -1e18;
/// `lock_until` of a latched rung: later than any clock this process will see.
const LATCHED: f64 = 1e18;
/// A rung change retunes the encoder's bitrate, and on Star6E that emits a
/// keyframe several times a P frame's size, which can take hundreds of ms of air.
/// The frames queued behind it arrive late and read as residual loss. Demoting on
/// that loss changes the bitrate again, so each step pays for the next one. For
/// this long after a change, residual loss does not demote; util demotes still do.
const RUNG_CHANGE_RESIDUAL_HOLD_MS: f64 = 1200.0;

impl LadderController {
    pub fn new(cfg: LadderCfg) -> Self {
        assert!(!cfg.ladder.is_empty());
        let n = cfg.ladder.len();
        Self {
            store: RungStore::new(n, cfg.rung_stats_half_life_samples),
            cfg,
            idx: 0,
            u: 0.0,
            pre_fec_loss: 0.0,
            last_feedback_ms: NEVER,
            measured_rung: 0,
            starved_since_ms: -1.0,
            last_change_ms: NEVER,
            last_down_ms: NEVER,
            confirm_start_ms: -1.0,
            clean_start_ms: -1.0,
            fade_until_ms: NEVER,
            fade_rssi: FadeEwma::default(),
            fade_snr: FadeEwma::default(),
            fade_trig_start_ms: -1.0,
            fade_latched: false,
            probation_active: false,
            probation_until_ms: NEVER,
            probation_rung: -1,
            fail_count: vec![0; n],
            penalty_until: vec![NEVER; n],
            strikes: vec![0; n],
            lock_until: vec![NEVER; n],
            probe_state: ProbeGateState::Off,
            probe_state_since_ms: 0.0,
            probe_streak_open: false,
            probe_streak_start_bodies: 0,
            probe_bodies_total: 0,
            probe_last_sample_ms: NEVER,
            probe_u: 0.0,
            probe_hold_active: false,
            u3: 0.0,
            s3_util_start_ms: -1.0,
            s3_last_live_ms: NEVER,
            s3_blank_until_ms: NEVER,
            blank_store_until_ms: NEVER,
            snr_now: f64::NAN,
            evm_now: f64::NAN,
            counters: CtlCounters::default(),
            last_event: CtlEvent::default(),
        }
    }

    pub fn rung(&self) -> i32 {
        self.idx
    }
    pub fn measured_rung(&self) -> i32 {
        self.measured_rung
    }
    pub fn op(&self) -> &Rung {
        &self.cfg.ladder[self.idx as usize]
    }
    pub fn ladder(&self) -> &[Rung] {
        &self.cfg.ladder
    }
    pub fn util(&self) -> f64 {
        self.u
    }
    pub fn util3(&self) -> f64 {
        self.u3
    }
    pub fn pre_fec_loss(&self) -> f64 {
        self.pre_fec_loss
    }
    pub fn counters(&self) -> &CtlCounters {
        &self.counters
    }
    pub fn last_event(&self) -> &CtlEvent {
        &self.last_event
    }
    pub fn rungs(&self) -> &RungStore {
        &self.store
    }
    pub fn fade_active(&self, now: f64) -> bool {
        now < self.fade_until_ms
    }

    fn budget(ov: f64) -> f64 {
        ov / (1.0 + ov)
    }
    pub fn budget_base(&self) -> f64 {
        Self::budget(self.op().overhead_base)
    }
    pub fn budget_base_for(&self, r: i32) -> f64 {
        Self::budget(self.cfg.ladder[r as usize].overhead_base)
    }
    pub fn budget_enh_for(&self, r: i32) -> f64 {
        Self::budget(self.cfg.ladder[r as usize].overhead_enh)
    }
    /// What the next promote waits on, for a UI: the s1 clean run, the
    /// probe streak and a penalty on the next rung. `None` on the top rung.
    pub fn up_status(&self, now: f64) -> Option<UpStatus> {
        let next = self.idx as usize + 1;
        if next >= self.cfg.ladder.len() {
            return None;
        }
        let pen_ms = self
            .penalized(now)
            .into_iter()
            .find(|(r, _)| *r as usize == next)
            .map_or(0, |(_, ms)| ms);
        let lock = self
            .locked(now)
            .into_iter()
            .find(|l| l.rung as usize == next);
        Some(UpStatus {
            u: self.u,
            up_util: self.cfg.up_util,
            clean_ms: if self.clean_start_ms < 0.0 {
                0.0
            } else {
                now - self.clean_start_ms
            },
            clean_need_ms: f64::from(self.cfg.clean_ms),
            probe: self.probe_gate(),
            probe_max_util: self.probe_util_threshold(),
            probe_need: self.cfg.probe.clean_bodies as u64,
            next_penalty_ms: pen_ms,
            next_lock_ms: lock.map_or(0, |l| l.ms_left),
            next_strikes: self.strikes[next],
        })
    }
    fn probe_util_threshold(&self) -> f64 {
        if self.cfg.probe.max_util < 0.0 {
            self.cfg.down_util
        } else {
            self.cfg.probe.max_util
        }
    }
    fn s3_util_threshold(&self) -> f64 {
        if self.cfg.s3_down_util < 0.0 {
            self.cfg.down_util
        } else {
            self.cfg.s3_down_util
        }
    }
    fn in_fade_regime(&self, now: f64) -> bool {
        self.cfg.fade.cascade && now < self.fade_until_ms
    }
    fn eff_confirm_ms(&self, now: f64) -> f64 {
        f64::from(if self.in_fade_regime(now) {
            self.cfg.fade.confirm_ms
        } else {
            self.cfg.confirm_ms
        })
    }

    pub fn probe_rung(&self) -> i32 {
        if !self.cfg.probe.enable {
            return -1;
        }
        let top = self.cfg.ladder.len() as i32 - 1;
        if self.idx >= top {
            return -1;
        }
        (self.idx + self.cfg.probe.rung_offset).min(top)
    }

    pub fn probe_gate(&self) -> ProbeGate {
        ProbeGate {
            state: self.probe_state,
            rung: self.probe_rung(),
            u: self.probe_u,
            streak_bodies: if self.probe_state == ProbeGateState::Clean && self.probe_streak_open {
                self.probe_bodies_total - self.probe_streak_start_bodies
            } else {
                0
            },
        }
    }

    pub fn probation_ms_left(&self, now: f64) -> i32 {
        if self.probation_active && self.idx == self.probation_rung && now < self.probation_until_ms
        {
            (self.probation_until_ms - now) as i32
        } else {
            0
        }
    }

    pub fn penalized(&self, now: f64) -> Vec<(i32, i32)> {
        self.penalty_until
            .iter()
            .enumerate()
            .filter(|(_, &u)| now < u)
            .map(|(r, &u)| (r as i32, (u - now) as i32))
            .collect()
    }

    /// Rungs the loss lockout holds now.
    pub fn locked(&self, now: f64) -> Vec<Lockout> {
        self.lock_until
            .iter()
            .enumerate()
            .filter(|(_, &u)| now < u)
            .map(|(r, &u)| Lockout {
                rung: r as i32,
                ms_left: if u == LATCHED { -1 } else { (u - now) as i32 },
                strikes: self.strikes[r],
            })
            .collect()
    }

    pub fn strikes(&self, rung: usize) -> i32 {
        self.strikes[rung]
    }

    /// Forget every strike and lock: what they measured belongs to a link
    /// that is gone (a new drone session; `restore` does the same for a hop).
    pub fn clear_lockout(&mut self) {
        self.strikes.fill(0);
        self.lock_until.fill(NEVER);
    }

    /// One strike per loss demote, charged to the rung left. Strikes below
    /// `lockout_latch` lock it for `lockout_ms`, the latch-th until a clear;
    /// an expired lock keeps its strikes. Rung 0 is never left by a demote,
    /// so the floor is never locked.
    fn strike_rung(&mut self, rung: i32, now: f64) {
        if self.cfg.lockout_ms <= 0 {
            return;
        }
        let r = rung as usize;
        if self.strikes[r] < self.cfg.lockout_latch {
            self.strikes[r] += 1;
        }
        self.lock_until[r] = if self.strikes[r] >= self.cfg.lockout_latch {
            LATCHED
        } else {
            now + f64::from(self.cfg.lockout_ms)
        };
    }

    fn is_locked(&self, rung: usize, now: f64) -> bool {
        now < self.lock_until[rung]
    }

    fn set_probe_state(&mut self, s: ProbeGateState, now: f64) {
        if s == self.probe_state {
            return;
        }
        self.probe_state = s;
        self.probe_state_since_ms = now;
    }

    fn mark_transition(&mut self, now: f64) {
        self.s3_blank_until_ms = now + f64::from(self.cfg.s3_settle_ms);
        self.s3_util_start_ms = -1.0;
        self.probe_streak_open = false;
        self.probe_last_sample_ms = NEVER;
        self.probe_hold_active = false;
        if self.probe_state != ProbeGateState::Off {
            self.set_probe_state(ProbeGateState::NoInfo, now);
        }
    }

    fn update_probe_gate(&mut self, h: &LinkHealth, now: f64) {
        let pr = self.probe_rung();
        if pr < 0 {
            self.probe_streak_open = false;
            self.set_probe_state(ProbeGateState::Off, now);
            return;
        }
        let usable = h.probe_valid
            && h.probe_rung == pr
            && h.probe_expected_syms >= self.cfg.probe.min_syms as u64;
        if usable {
            let b = self.budget_enh_for(self.idx + 1);
            self.probe_u = if b > 0.0 {
                h.probe_loss / b
            } else if h.probe_loss > 0.0 {
                1e9
            } else {
                0.0
            };
            self.probe_last_sample_ms = now;
            if now >= self.blank_store_until_ms {
                self.store.observe_probe(pr as usize, self.probe_u, now);
            }
            if h.probe_bodies_total < self.probe_bodies_total {
                self.probe_streak_open = false;
            }
            self.probe_bodies_total = h.probe_bodies_total;
            if self.probe_u <= self.probe_util_threshold() {
                if !self.probe_streak_open {
                    self.probe_streak_open = true;
                    self.probe_streak_start_bodies = self.probe_bodies_total;
                }
                self.set_probe_state(ProbeGateState::Clean, now);
            } else {
                self.probe_streak_open = false;
                self.set_probe_state(ProbeGateState::Lossy, now);
            }
        } else if now - self.probe_last_sample_ms > f64::from(self.cfg.probe.silence_ms) {
            self.probe_streak_open = false;
            self.set_probe_state(ProbeGateState::NoInfo, now);
        }
    }

    fn reset_windows(&mut self) {
        self.confirm_start_ms = -1.0;
        self.clean_start_ms = -1.0;
        self.probe_hold_active = false;
    }

    fn set_event(&mut self, now: f64, from: i32, to: i32, reason: CtlReason, u: f64) {
        self.store
            .on_transition(from as usize, to as usize, reason, now);
        self.last_event = CtlEvent {
            t_ms: now,
            from,
            to,
            reason,
            u,
            snr_db: self.snr_now,
            evm_db: self.evm_now,
        };
    }

    fn check_probation_survival(&mut self, now: f64) {
        if self.probation_active
            && self.idx == self.probation_rung
            && now >= self.probation_until_ms
        {
            self.fail_count[self.idx as usize] = 0;
            self.probation_active = false;
        }
    }

    fn penalize_rung(&mut self, rung: i32, now: f64) {
        let r = rung as usize;
        if self.fail_count[r] < 32 {
            self.fail_count[r] += 1;
        }
        let k = self.fail_count[r];
        let shifted = (i64::from(self.cfg.penalty_base_ms) << (k - 1)) as f64;
        let dur = f64::from(self.cfg.penalty_max_ms).min(shifted);
        self.penalty_until[r] = now + dur;
    }

    fn is_penalized(&self, rung: usize, now: f64) -> bool {
        now < self.penalty_until[rung]
    }

    /// One step down with the shared bookkeeping of every demote path.
    fn demote(&mut self, now: f64, reason: CtlReason, u: f64, fade: bool) {
        let from = self.idx;
        if matches!(
            reason,
            CtlReason::Residual
                | CtlReason::S3Residual
                | CtlReason::Util
                | CtlReason::S3Util
                | CtlReason::Probation
        ) {
            self.strike_rung(from, now);
        }
        self.idx = from - 1;
        self.last_down_ms = now;
        self.last_change_ms = now;
        self.reset_windows();
        self.mark_transition(now);
        if fade {
            self.fade_until_ms = now + f64::from(self.cfg.fade.hold_ms);
        }
        self.set_event(now, from, self.idx, reason, u);
    }

    fn fail_probation_if_on_it(&mut self, now: f64) {
        if self.probation_active && self.idx == self.probation_rung {
            self.counters.probation_fails += 1;
            self.penalize_rung(self.idx, now);
            self.probation_active = false;
        }
    }

    pub fn update(&mut self, h: &LinkHealth, now: f64) -> bool {
        self.check_probation_survival(now);
        self.snr_now = h.rf_snr_db;
        self.evm_now = h.rf_evm_db;
        self.u3 = 0.0;
        if h.video_starved {
            if self.starved_since_ms < 0.0 {
                self.starved_since_ms = now;
            }
            if now - self.starved_since_ms < f64::from(self.cfg.starved_confirm_ms) || self.idx == 0
            {
                return false;
            }
            let from = self.idx;
            self.idx = 0;
            self.last_down_ms = now;
            self.last_change_ms = now;
            self.probation_active = false;
            self.reset_windows();
            self.mark_transition(now);
            self.counters.starved_drops += 1;
            self.set_event(now, from, 0, CtlReason::Starved, 0.0);
            return true;
        }
        self.starved_since_ms = -1.0;
        if !h.sample_valid {
            return false;
        }
        self.last_feedback_ms = now;
        self.measured_rung = self.idx;
        if !h.rf_rssi_dbm.is_nan() {
            self.fade_rssi.feed(h.rf_rssi_dbm, now);
        }
        if !h.rf_snr_db.is_nan() {
            self.fade_snr.feed(h.rf_snr_db, now);
        }
        self.pre_fec_loss = h.pre_fec_loss;
        self.u = h.pre_fec_loss / self.budget_base();
        self.update_probe_gate(h, now);
        if now >= self.s3_blank_until_ms && now >= self.blank_store_until_ms {
            let r = self.idx as usize;
            self.store.observe_s1(r, self.u, h.residual_loss > 0.0, now);
            if !h.rf_evm_db.is_nan() {
                self.store.observe_evm(r, h.rf_evm_db);
            }
        }
        let idr_settling = now - self.last_change_ms < RUNG_CHANGE_RESIDUAL_HOLD_MS;
        if h.residual_loss > 0.0 && self.idx > 0 && !idr_settling {
            self.fail_probation_if_on_it(now);
            self.counters.demotes_residual += 1;
            self.demote(now, CtlReason::Residual, self.u, true);
            return true;
        }
        if self.cfg.fade.predict {
            let measurable = !h.rf_rssi_dbm.is_nan()
                && !h.rf_snr_db.is_nan()
                && self.fade_rssi.has
                && self.fade_snr.has;
            let (dr, ds) = (self.fade_rssi.delta(), self.fade_snr.delta());
            let over = measurable && dr >= self.cfg.fade.rssi_db && ds >= self.cfg.fade.snr_db;
            if measurable && dr < self.cfg.fade.rssi_db && ds < self.cfg.fade.snr_db {
                self.fade_latched = false;
            }
            if !over {
                self.fade_trig_start_ms = -1.0;
            } else {
                if self.fade_trig_start_ms < 0.0 {
                    self.fade_trig_start_ms = now;
                }
                if !self.fade_latched
                    && self.idx > 0
                    && self.idx >= self.cfg.fade.min_rung
                    && now - self.fade_trig_start_ms >= f64::from(self.cfg.fade.trigger_ms)
                    && now - self.last_change_ms >= f64::from(self.cfg.min_between_changes_ms)
                {
                    self.probation_active = false; // cleared, no fail booked
                    self.counters.demotes_fade += 1;
                    self.demote(now, CtlReason::Fade, self.u, true);
                    self.fade_trig_start_ms = -1.0;
                    self.fade_latched = true;
                    return true;
                }
            }
        }
        let s3_live = h.s3_valid
            && h.s3_expected_syms >= self.cfg.s3_min_syms as u64
            && now >= self.s3_blank_until_ms;
        if now - self.s3_last_live_ms > f64::from(self.cfg.s3_settle_ms) {
            self.s3_util_start_ms = -1.0;
        }
        if !s3_live {
            self.s3_util_start_ms = -1.0;
        } else {
            self.s3_last_live_ms = now;
            let b3 = self.budget_enh_for(self.idx);
            self.u3 = if b3 > 0.0 {
                h.s3_pre_fec_loss / b3
            } else if h.s3_pre_fec_loss > 0.0 {
                1e9
            } else {
                0.0
            };
            if now >= self.blank_store_until_ms {
                self.store
                    .observe_s3(self.idx as usize, self.u3, h.s3_residual_loss > 0.0, now);
            }
        }
        if self.cfg.s3_demote
            && s3_live
            && h.s3_residual_loss > 0.0
            && self.idx > 0
            && !idr_settling
        {
            self.fail_probation_if_on_it(now);
            self.counters.demotes_s3_residual += 1;
            self.demote(now, CtlReason::S3Residual, self.u3, true);
            return true;
        }
        if self.u > self.cfg.down_util {
            if self.confirm_start_ms < 0.0 {
                self.confirm_start_ms = now;
            }
            if self.probation_active && self.idx == self.probation_rung {
                self.fail_probation_if_on_it(now);
                self.demote(now, CtlReason::Probation, self.u, false);
                return true;
            }
            if self.idx > 0
                && now - self.confirm_start_ms >= self.eff_confirm_ms(now)
                && now - self.last_change_ms >= f64::from(self.cfg.min_between_changes_ms)
            {
                self.counters.demotes_util += 1;
                self.demote(now, CtlReason::Util, self.u, true);
                return true;
            }
        } else {
            self.confirm_start_ms = -1.0;
        }
        if self.cfg.s3_demote && s3_live {
            if self.u3 > self.s3_util_threshold() {
                if self.s3_util_start_ms < 0.0 {
                    self.s3_util_start_ms = now;
                }
                if self.idx > 0
                    && now - self.s3_util_start_ms >= self.eff_confirm_ms(now)
                    && now - self.last_change_ms >= f64::from(self.cfg.min_between_changes_ms)
                {
                    self.fail_probation_if_on_it(now);
                    self.counters.demotes_s3_util += 1;
                    self.demote(now, CtlReason::S3Util, self.u3, true);
                    return true;
                }
            } else {
                self.s3_util_start_ms = -1.0;
            }
        }
        if self.u < self.cfg.up_util {
            if self.clean_start_ms < 0.0 {
                self.clean_start_ms = now;
            }
            let next = self.idx as usize + 1;
            if next < self.cfg.ladder.len()
                && now - self.clean_start_ms >= f64::from(self.cfg.clean_ms)
                && now - self.last_down_ms >= f64::from(self.cfg.hold_after_down_ms)
                && now - self.last_change_ms >= f64::from(self.cfg.min_between_changes_ms)
                && !self.is_penalized(next, now)
                && !self.is_locked(next, now)
            {
                let g = self.probe_gate();
                let mut reason = CtlReason::Promote;
                if g.state != ProbeGateState::Off {
                    if g.state == ProbeGateState::Clean
                        && g.streak_bodies >= self.cfg.probe.clean_bodies as u64
                    {
                        reason = CtlReason::PromoteProbed;
                    } else if g.state == ProbeGateState::Clean {
                        return false;
                    } else {
                        if !self.probe_hold_active {
                            self.probe_hold_active = true;
                            self.counters.probe_holds += 1;
                        }
                        return false;
                    }
                }
                let from = self.idx;
                self.idx = next as i32;
                self.last_change_ms = now;
                self.reset_windows();
                self.probation_active = true;
                self.probation_rung = self.idx;
                self.probation_until_ms = now + f64::from(self.cfg.probation_ms);
                self.mark_transition(now);
                self.counters.promotes += 1;
                if reason == CtlReason::PromoteProbed {
                    self.counters.promotes_probed += 1;
                }
                self.set_event(now, from, self.idx, reason, self.u);
                return true;
            }
        } else {
            self.clean_start_ms = -1.0;
            self.probe_hold_active = false;
        }
        false
    }

    /// Blind-side failsafe: no feedback for feedback_timeout_ms = rung 0.
    pub fn on_tick(&mut self, now: f64) -> bool {
        self.check_probation_survival(now);
        if now - self.last_feedback_ms > f64::from(self.cfg.feedback_timeout_ms) && self.idx > 0 {
            let from = self.idx;
            self.idx = 0;
            self.last_down_ms = now;
            self.last_change_ms = now;
            self.probation_active = false;
            self.reset_windows();
            self.mark_transition(now);
            self.counters.timeout_drops += 1;
            self.set_event(now, from, 0, CtlReason::Timeout, 0.0);
            return true;
        }
        false
    }

    pub fn restore(&mut self, rung: i32, now: f64) {
        // The strikes were measured on the channel the hop left.
        self.clear_lockout();
        let rung = rung.clamp(0, self.cfg.ladder.len() as i32 - 1);
        let from = self.idx;
        self.idx = rung;
        self.last_change_ms = now;
        self.probation_active = false;
        self.probation_until_ms = NEVER;
        self.probation_rung = -1;
        self.reset_windows();
        self.mark_transition(now);
        self.set_event(now, from, rung, CtlReason::HopRestore, self.u);
    }

    pub fn blank_store(&mut self, until_ms: f64) {
        self.blank_store_until_ms = self.blank_store_until_ms.max(until_ms);
    }
}

#[cfg(test)]
mod lockout_tests {
    use super::*;

    /// mabur's test ladder, probe off, no probation penalty, no hold after a
    /// demote, a 1 s clean run and a 30 s lockout latching at the 4th strike:
    /// a promote held back below is held by the lockout and nothing else.
    fn cfg() -> LadderCfg {
        let mut c = LadderCfg::default();
        c.probe.enable = false;
        c.penalty_base_ms = 0;
        c.penalty_max_ms = 0;
        c.hold_after_down_ms = 0;
        c.clean_ms = 1000;
        c.lockout_ms = 30000;
        c.lockout_latch = 4;
        c
    }
    fn ok(pre: f64) -> LinkHealth {
        LinkHealth {
            sample_valid: true,
            pre_fec_loss: pre,
            ..LinkHealth::empty()
        }
    }
    fn resid() -> LinkHealth {
        LinkHealth {
            residual_loss: 0.05,
            ..ok(0.0)
        }
    }
    /// Clean samples every 50 ms until `rung`; returns the promote time.
    fn climb(c: &mut LadderController, t: &mut f64, rung: i32) -> f64 {
        while c.rung() < rung {
            c.update(&ok(0.0), *t);
            if c.rung() == rung {
                return *t;
            }
            *t += 50.0;
            assert!(*t < 1e7);
        }
        *t
    }
    /// Neutral samples (between up_util and down_util) for `ms`.
    fn hold(c: &mut LadderController, t: &mut f64, ms: f64) {
        let end = *t + ms;
        while *t <= end {
            let b = c.budget_base();
            c.update(&ok(0.3 * b), *t);
            *t += 50.0;
        }
    }
    fn lock_ms(c: &LadderController, t: f64, rung: i32) -> i32 {
        c.locked(t)
            .iter()
            .find(|l| l.rung == rung)
            .map_or(0, |l| l.ms_left)
    }

    #[test]
    fn strikes_lock_for_lockout_ms_and_the_fourth_latches() {
        let cfg = cfg();
        let mut c = LadderController::new(cfg.clone());
        let mut t = 0.0;
        climb(&mut c, &mut t, 2);
        for strike in 1..=4 {
            hold(&mut c, &mut t, 4000.0);
            assert_eq!(c.rung(), 2);
            let down = t;
            assert!(c.update(&resid(), t));
            assert_eq!(c.last_event().reason, CtlReason::Residual);
            assert_eq!((c.rung(), c.strikes(2), c.strikes(1)), (1, strike, 0));
            if strike < 4 {
                assert_eq!(lock_ms(&c, t, 2), cfg.lockout_ms);
                let up = climb(&mut c, &mut t, 2);
                assert!(up >= down + f64::from(cfg.lockout_ms));
                assert!(up <= down + f64::from(cfg.lockout_ms + cfg.clean_ms) + 100.0);
                assert_eq!(c.strikes(2), strike, "an expired lock keeps its strikes");
            } else {
                assert_eq!(lock_ms(&c, t, 2), -1);
                let end = t + 300_000.0;
                while t < end {
                    t += 50.0;
                    c.update(&ok(0.0), t);
                }
                assert_eq!(c.rung(), 1, "latched: five clean minutes do not lift it");
            }
        }
    }

    #[test]
    fn a_locked_rung_is_a_ceiling_and_the_floor_is_never_locked() {
        let cfg = cfg();
        let mut c = LadderController::new(cfg.clone());
        let mut t = 0.0;
        climb(&mut c, &mut t, 2);
        hold(&mut c, &mut t, 4000.0);
        c.update(&resid(), t);
        let free = t + f64::from(cfg.lockout_ms);
        hold(&mut c, &mut t, 1300.0);
        c.update(&resid(), t);
        assert_eq!((c.rung(), c.strikes(1), c.strikes(0)), (0, 1, 0));
        assert_eq!(lock_ms(&c, t, 0), 0);
        let up1 = climb(&mut c, &mut t, 1);
        assert!(
            up1 >= free,
            "rung 1 was struck later, so it frees after rung 2"
        );
        // Rung 2 locked, rung 1 not: the climb off the floor stops at 1.
        let mut c = LadderController::new(cfg.clone());
        let mut t = 0.0;
        climb(&mut c, &mut t, 2);
        hold(&mut c, &mut t, 4000.0);
        c.update(&resid(), t);
        let free = t + f64::from(cfg.lockout_ms);
        assert!(
            c.on_tick(t + 2000.0),
            "timeout to the floor strikes nothing"
        );
        t += 2000.0;
        assert_eq!(c.strikes(1), 0);
        let up1 = climb(&mut c, &mut t, 1);
        assert!(up1 < free);
        let up2 = climb(&mut c, &mut t, 2);
        assert!(up2 >= free && up2 <= free + f64::from(cfg.clean_ms) + 100.0);
    }

    #[test]
    fn util_s3_and_probation_demotes_strike_timeout_and_starved_do_not() {
        let mut c = LadderController::new(cfg());
        let mut t = 0.0;
        climb(&mut c, &mut t, 3);
        hold(&mut c, &mut t, 4000.0);
        while c.rung() == 3 {
            let b = c.budget_base();
            c.update(&ok(0.9 * b), t);
            t += 50.0;
        }
        assert_eq!((c.last_event().reason, c.strikes(3)), (CtlReason::Util, 1));
        hold(&mut c, &mut t, 1300.0);
        let b = c.budget_base();
        c.update(
            &LinkHealth {
                s3_valid: true,
                s3_expected_syms: 500,
                s3_residual_loss: 0.05,
                ..ok(0.3 * b)
            },
            t,
        );
        assert_eq!(
            (c.last_event().reason, c.strikes(2)),
            (CtlReason::S3Residual, 1)
        );

        let mut p = LadderController::new(LadderCfg {
            lockout_ms: 1000,
            ..cfg()
        });
        let mut tp = 0.0;
        climb(&mut p, &mut tp, 1);
        assert!(p.probation_ms_left(tp) > 0);
        tp += 50.0;
        let b = p.budget_base();
        p.update(&ok(0.9 * b), tp);
        assert_eq!(
            (p.last_event().reason, p.strikes(1)),
            (CtlReason::Probation, 1)
        );
        assert_eq!(lock_ms(&p, tp, 1), 1000);

        let mut o = LadderController::new(cfg());
        let mut to = 0.0;
        climb(&mut o, &mut to, 2);
        assert!(o.on_tick(to + 2000.0));
        assert_eq!(o.last_event().reason, CtlReason::Timeout);
        let mut s = LadderController::new(cfg());
        let mut ts = 0.0;
        climb(&mut s, &mut ts, 2);
        while s.rung() > 0 {
            s.update(
                &LinkHealth {
                    video_starved: true,
                    ..ok(0.0)
                },
                ts,
            );
            ts += 50.0;
        }
        assert_eq!(s.last_event().reason, CtlReason::Starved);
        assert!(o.locked(to).is_empty() && s.locked(ts).is_empty());
    }

    #[test]
    fn off_by_default_and_clear_forgets_every_strike() {
        let mut c = LadderController::new(LadderCfg {
            lockout_ms: 0,
            ..cfg()
        });
        let mut t = 0.0;
        climb(&mut c, &mut t, 2);
        hold(&mut c, &mut t, 4000.0);
        c.update(&resid(), t);
        assert_eq!((c.rung(), c.strikes(2)), (1, 0));
        let down = t;
        let up = climb(&mut c, &mut t, 2);
        assert!(
            up < down + 2000.0,
            "straight back up, as before the lockout"
        );
        assert_eq!(LadderCfg::default().lockout_ms, 0);

        let mut c = LadderController::new(cfg());
        let mut t = 0.0;
        climb(&mut c, &mut t, 2);
        hold(&mut c, &mut t, 4000.0);
        c.update(&resid(), t);
        let up = c.up_status(t).unwrap();
        assert_eq!((up.next_lock_ms, up.next_strikes), (30000, 1));
        c.clear_lockout();
        assert!(c.locked(t).is_empty());
        assert_eq!(c.strikes(2), 0);
    }
}
