//! The calibration run on the GS (mabur `CalSession`). A run is an ordered
//! list of sweeps; per sweep the session commands the drone, waits for the
//! ack, keeps the GS radio silent while the drone airs stamped frames,
//! tallies them, analyses them when the drone says it is done (or the
//! window closes), sends the result for the drone to apply, verifies a
//! power apply at the parked indices and moves on.
//!
//! It talks to the drone only in `CalMsg`s: the owner encodes `due_mav`'s
//! output onto the tunnel's cal lane and feeds the drone's replies to
//! `on_mav`. It alone decides whether the GS may transmit while a run is in
//! progress (`radio_silent`).
//!
//! A run on an auto GS with a spare card picks its channel first
//! (`Picking`, through a `CalPicker`): one measure round, one relocate if
//! op is not the best member. The owner drives it with `step_pick` every
//! tick; nothing reaches the drone until the pick ends.

use pal_wire::cal::{
    park_rel_for, Abort, AckResult, Applied, ApplyCmd, CalMsg, FrameInfo, Kind, SweepCmd,
    SweepDone, SweepSpec, FLAG_LDPC, WALL_LEAVE,
};
use pal_wire::rc::TagCtx;

use super::analysis::{
    analyze_capacity, analyze_rate, CalCell, CalThresholds, CapCell, CapResult, RateWall,
    FLAG_LOST, FLAG_TX_BOUND, NO_WALL,
};
use super::log::CalLog;
use super::picker::{CalPicker, Mode};
use super::plan::{
    make_capacity_sweep, make_power_sweep, sweep_duration_ms, verify_cells, verify_duration_ms,
    VERIFY_FRAMES,
};

/// How often an unanswered SweepCmd or ApplyCmd goes again: the uplink
/// loses a large share of frames, and both are idempotent on the drone by
/// (nonce, seq).
const RESEND_MS: u64 = 200;
/// Copies of the closing Abort: the uplink is lossy and the drone ignores
/// a nonce it already closed.
pub const ABORT_COPIES: u32 = 3;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CalSessionCfg {
    pub th: CalThresholds,
    /// A SweepCmd repeated this long without an ack fails the run.
    pub ack_timeout_ms: u64,
    /// How long past a sweep's planned end the GS keeps listening (and
    /// silent) for late frames and the SweepDone; also pads verify.
    pub phase_slack_ms: u64,
    /// After a capacity SweepDone, how long the sweep's frames must have
    /// been quiet before it closes: the drone sends SweepDone ahead of what
    /// its TX queue is still airing.
    pub tail_quiet_ms: u64,
    pub apply_timeout_ms: u64,
    /// Bound on the channel pick (measure round + relocate confirm).
    pub pick_timeout_ms: u64,
    /// The GS's copy of the wall margin: predicts the verify parks (the
    /// drone parks by its own radio.wall_margin_db) and goes on the R line.
    pub margin_db: f64,
}

impl Default for CalSessionCfg {
    fn default() -> Self {
        Self {
            th: CalThresholds::default(),
            ack_timeout_ms: 3000,
            phase_slack_ms: 1000,
            tail_quiet_ms: 50,
            apply_timeout_ms: 5000,
            pick_timeout_ms: 8000,
            margin_db: 1.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    Idle,
    /// The channel pick before anything reaches the drone. Not running():
    /// the channel machinery must keep running while the pick measures.
    Picking,
    AwaitAck,
    Sweep,
    AwaitDone,
    Analyze,
    Apply,
    Verify,
    Done,
    Failed,
}

impl State {
    /// The names `maburcal status` prints.
    pub fn name(self) -> &'static str {
        match self {
            State::Idle => "idle",
            State::Picking => "picking",
            State::AwaitAck => "await_ack",
            State::Sweep => "sweep",
            State::AwaitDone => "await_done",
            State::Analyze => "analyze",
            State::Apply => "apply",
            State::Verify => "verify",
            State::Done => "done",
            State::Failed => "failed",
        }
    }
    /// A run owns the air.
    pub fn running(self) -> bool {
        matches!(
            self,
            State::AwaitAck
                | State::Sweep
                | State::AwaitDone
                | State::Analyze
                | State::Apply
                | State::Verify
        )
    }
}

/// What one width's sweeps produced so far.
#[derive(Clone, Debug, PartialEq)]
pub struct WidthResult {
    pub have_walls: bool,
    pub have_eff: bool,
    /// A capacity sweep ran without its SweepDone: nothing is written.
    pub eff_unknown: bool,
    pub applied: bool,
    pub apply_reason: String,
    pub walls: [RateWall; 8],
    pub cap: [CapResult; 8],
    /// What was sent; 0 = leave.
    pub eff_x1000: [u16; 8],
    /// Verify parks per rate; WALL_LEAVE = not parked.
    pub park: [i32; 8],
    /// Best-card verify delivery per rate; -1 = not verified.
    pub verify_pct: [i32; 8],
}

impl Default for WidthResult {
    fn default() -> Self {
        Self {
            have_walls: false,
            have_eff: false,
            eff_unknown: false,
            applied: false,
            apply_reason: String::new(),
            walls: [RateWall::default(); 8],
            cap: [CapResult::default(); 8],
            eff_x1000: [0; 8],
            park: [i32::from(WALL_LEAVE); 8],
            verify_pct: [-1; 8],
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct CalSnapshot {
    pub state: State,
    pub sweep_i: usize,
    pub sweep_n: usize,
    pub cur: SweepSpec,
    /// The latest tallied frame's cell, 1-based, of the sweep's total.
    pub cell_i: u32,
    pub cell_n: u32,
    pub channel: u8,
    pub pick_note: String,
    pub flags: u8,
    pub stamp_mismatch: u32,
    /// [0] = 20 MHz, [1] = 40 MHz.
    pub width: [WidthResult; 2],
    pub fail_reason: String,
}

impl CalSnapshot {
    /// "verify" during the verify pass, else the sweep's own kind.
    pub fn kind_name(&self) -> &'static str {
        if self.state == State::Verify {
            "verify"
        } else if self.cur.kind == Kind::Capacity {
            "capacity"
        } else {
            "power"
        }
    }
}

/// The operator's token for one sweep: power20, cap40, ...
pub fn sweep_token(s: &SweepSpec) -> String {
    let k = if s.kind == Kind::Capacity {
        "cap"
    } else {
        "power"
    };
    format!("{k}{}", s.bw)
}

fn median(mut v: Vec<i32>) -> i32 {
    v.sort_unstable();
    v[v.len() / 2]
}

fn ack_word(r: AckResult) -> &'static str {
    match r {
        AckResult::Accepted => "accepted",
        AckResult::AnchorUnreadable => "anchor_unreadable",
        AckResult::Busy => "busy",
        AckResult::Unsupported => "unsupported",
    }
}

/// Power before capacity within a width: a capacity sweep flies at the
/// power its own width's walls were just parked at.
fn sweep_key(s: &SweepSpec) -> (u8, u8) {
    (s.bw, u8::from(s.kind != Kind::Power))
}

pub struct CalSession {
    cfg: CalSessionCfg,
    log: CalLog,

    state: State,
    nonce: u32,
    linked: bool,
    cal_capable: bool,
    peer_ctx: TagCtx,
    tag_ctx: TagCtx,
    fail_reason: String,

    sweeps: Vec<SweepSpec>,
    sweep_i: usize,
    cmd: SweepCmd,

    /// The latest stamp any entry point has seen (see `advance`).
    now: u64,
    channel: u8,
    /// cal.log R's pick token: pinned, onecard, auto or stayed.
    pick: &'static str,
    /// The UI's one-line account of the pick.
    pick_note: String,
    pick_start_ms: u64,
    reloc_ordered: bool,
    reloc_to: u8,
    run_logged: bool,

    phase_start_ms: u64,
    last_sent_ms: u64,
    sent_once: bool,
    reached_drone: bool,
    pending_abort: Option<Abort>,
    aborts_left: u32,
    next_abort_ms: u64,

    ack_flags: u8,
    sweep_end_ms: u64,
    window_end_ms: u64,
    done: Option<SweepDone>,
    stamp_mismatch: u32,
    cell_i: u32,

    /// Power tally [rate][idx - idx_lo]; RSSI samples per card.
    cells: [Vec<CalCell>; 8],
    rssi: [Vec<[Vec<i32>; 2]>; 8],
    cap: [CapCell; 8],
    cap_rssi: [[Vec<i32>; 2]; 8],
    /// Session time a capacity frame was last handled; raised to the next
    /// due_mav tick while `cap_fresh`.
    last_cap_seen_ms: u64,
    cap_fresh: bool,
    verify_cells: Vec<(u8, i32)>,
    verify_recv: [[u16; 2]; 8],
    verify_seen_idx: [i32; 8],
    verify_seen: [bool; 8],

    apply: ApplyCmd,
    apply_wrote_walls: bool,
    width: [WidthResult; 2],
}

impl CalSession {
    pub fn new(cfg: CalSessionCfg, log: CalLog) -> Self {
        Self {
            cfg,
            log,
            state: State::Idle,
            nonce: 0,
            linked: false,
            cal_capable: false,
            peer_ctx: TagCtx::default(),
            tag_ctx: TagCtx::default(),
            fail_reason: String::new(),
            sweeps: Vec::new(),
            sweep_i: 0,
            cmd: SweepCmd::default(),
            now: 0,
            channel: 0,
            pick: "pinned",
            pick_note: String::new(),
            pick_start_ms: 0,
            reloc_ordered: false,
            reloc_to: 0,
            run_logged: false,
            phase_start_ms: 0,
            last_sent_ms: 0,
            sent_once: false,
            reached_drone: false,
            pending_abort: None,
            aborts_left: 0,
            next_abort_ms: 0,
            ack_flags: 0,
            sweep_end_ms: 0,
            window_end_ms: 0,
            done: None,
            stamp_mismatch: 0,
            cell_i: 0,
            cells: Default::default(),
            rssi: Default::default(),
            cap: [CapCell::default(); 8],
            cap_rssi: Default::default(),
            last_cap_seen_ms: 0,
            cap_fresh: false,
            verify_cells: Vec::new(),
            verify_recv: [[0; 2]; 8],
            verify_seen_idx: [0; 8],
            verify_seen: [false; 8],
            apply: ApplyCmd::default(),
            apply_wrote_walls: false,
            width: Default::default(),
        }
    }

    /// Every entry point folds its stamp in first: the receive path stamps
    /// on its own thread, so a stamp can be older than a tick already seen,
    /// and elapsed-time checks must never underflow.
    fn advance(&mut self, now_ms: u64) -> u64 {
        self.now = self.now.max(now_ms);
        self.now
    }

    /// Starts a run over `sweeps` (sorted per width ascending, power before
    /// capacity, deduplicated). Refuses an empty list, a Verify or
    /// non-20/40 entry, a run (or pick) already going, a down link or a
    /// peer without CAP_CALIBRATE. An Auto `picker` starts the channel
    /// pick; any other (or none) sweeps on its op at once.
    pub fn start(
        &mut self,
        nonce: u32,
        mut sweeps: Vec<SweepSpec>,
        now_ms: u64,
        picker: Option<&mut dyn CalPicker>,
    ) -> Result<(), String> {
        let now_ms = self.advance(now_ms);
        // Picking is a run too: a second start would fight the first one's
        // pick.
        if self.running() || self.state == State::Picking {
            return Err("a calibration session is already running".into());
        }
        if !self.linked {
            return Err("refused: link is down".into());
        }
        if !self.cal_capable {
            return Err("refused: peer does not advertise CAP_CALIBRATE".into());
        }
        if sweeps.is_empty() {
            return Err("refused: no sweeps".into());
        }
        for s in &sweeps {
            if s.kind == Kind::Verify {
                return Err("refused: a sweep is power or capacity".into());
            }
            if s.bw != 20 && s.bw != 40 {
                return Err("refused: a sweep's width is 20 or 40".into());
            }
        }
        sweeps.sort_by_key(sweep_key);
        sweeps.dedup_by_key(|s| sweep_key(s));

        self.nonce = nonce;
        self.sweeps = sweeps;
        self.sweep_i = 0;
        self.fail_reason.clear();
        self.run_logged = false;
        self.reached_drone = false;
        self.stamp_mismatch = 0;
        self.cell_i = 0;
        self.ack_flags = 0;
        self.done = None;
        self.width = Default::default();
        // pending_abort is kept: the previous run's Abort copies still owed
        // must still go out.
        self.reloc_ordered = false;
        self.channel = picker.as_ref().map_or(0, |p| p.op());
        let mode = picker.as_ref().map(|p| p.mode());
        let Some(p) = picker.filter(|_| mode == Some(Mode::Auto)) else {
            self.pick = if mode == Some(Mode::OneCard) {
                "onecard"
            } else {
                "pinned"
            };
            self.pick_note = self.pick.to_string();
            self.begin_sweep(0, now_ms);
            return Ok(());
        };
        self.pick = "auto";
        self.pick_note = "measuring".into();
        self.pick_start_ms = now_ms;
        self.state = State::Picking;
        p.begin_measure(now_ms);
        Ok(())
    }

    /// Advances the channel pick; a no-op outside Picking. The owner calls
    /// it every tick, before `due_mav`, with the picker the run started on.
    pub fn step_pick(&mut self, now_ms: u64, p: &mut dyn CalPicker) {
        let now_ms = self.advance(now_ms);
        if self.state != State::Picking {
            return;
        }
        let op = p.op();
        if now_ms - self.pick_start_ms >= self.cfg.pick_timeout_ms {
            p.cancel();
            self.pick_note = format!(
                "stayed on {op}, {}",
                if self.reloc_ordered {
                    "relocate unconfirmed"
                } else {
                    "measure timed out"
                }
            );
            self.finish_pick("stayed", p, now_ms);
            return;
        }
        if !self.reloc_ordered {
            let Some(best) = p.measured_best() else {
                return;
            };
            if best == op {
                self.pick_note = format!("auto: {op} is the best member");
                self.finish_pick("auto", p, now_ms);
                return;
            }
            if !p.order_relocate(best) {
                p.cancel();
                self.pick_note = format!("stayed on {op}, relocate refused");
                self.finish_pick("stayed", p, now_ms);
                return;
            }
            self.reloc_ordered = true;
            self.reloc_to = best;
            self.pick_note = format!("relocating {op} -> {best}");
            return;
        }
        match p.relocate_settled() {
            None => {}
            Some(true) => {
                self.pick_note = format!("auto: moved to {}", self.reloc_to);
                self.finish_pick("auto", p, now_ms);
                self.channel = self.reloc_to;
            }
            Some(false) => {
                p.cancel();
                self.pick_note = format!("stayed on {op}, relocate unconfirmed");
                self.finish_pick("stayed", p, now_ms);
            }
        }
    }

    fn finish_pick(&mut self, pick: &'static str, p: &dyn CalPicker, now_ms: u64) {
        self.pick = pick;
        self.channel = p.op();
        self.begin_sweep(0, now_ms);
    }

    pub fn picking(&self) -> bool {
        self.state == State::Picking
    }

    /// The drone's SweepAck, SweepDone and Applied; anything naming another
    /// nonce or sweep, or arriving in the wrong state, is ignored.
    pub fn on_mav(&mut self, m: &CalMsg, now_ms: u64) {
        let now_ms = self.advance(now_ms);
        self.step(now_ms);
        match m {
            CalMsg::Ack(a) => {
                if self.state != State::AwaitAck || a.nonce != self.nonce || a.seq != self.cmd.seq {
                    return;
                }
                if a.result != AckResult::Accepted {
                    self.fail(format!(
                        "sweep refused by the drone: {}",
                        ack_word(a.result)
                    ));
                    return;
                }
                self.on_accepted(a.flags, a.start_in_ms, now_ms);
            }
            CalMsg::Done(d) => {
                if !matches!(self.state, State::Sweep | State::AwaitDone)
                    || d.nonce != self.nonce
                    || d.seq != self.cmd.seq
                {
                    return;
                }
                self.done = Some(*d);
                if self.tail_quiet(now_ms) {
                    self.finish_sweep(now_ms);
                }
            }
            CalMsg::Applied(ap) => {
                // Matched on seq as well as width: a run applies twice at
                // one width and the drone answers repeats from its cache.
                if self.state != State::Apply
                    || ap.nonce != self.nonce
                    || ap.seq != self.cmd.seq
                    || ap.bw != self.cmd.bw
                {
                    return;
                }
                self.on_applied(ap, now_ms);
            }
            _ => {}
        }
    }

    /// The owner's once-per-pass tick, after the pass handled the frames it
    /// drained. The message to send now: a SweepCmd or ApplyCmd (repeated
    /// every 200 ms until answered) or a closing Abort. Sent as is: the
    /// session offers only when the air is the GS's.
    pub fn due_mav(&mut self, now_ms: u64) -> Option<CalMsg> {
        let now_ms = self.advance(now_ms);
        // Capacity frames handled since the last tick: not quiet up to this
        // tick. Their own receive stamps are not enough, a long pass puts
        // its tick far past them while the rest still waits in the queue.
        if self.cap_fresh {
            self.last_cap_seen_ms = self.last_cap_seen_ms.max(now_ms);
            self.cap_fresh = false;
        }
        self.step(now_ms);
        if let Some(a) = self.pending_abort {
            if now_ms >= self.next_abort_ms {
                self.aborts_left -= 1;
                if self.aborts_left == 0 {
                    self.pending_abort = None;
                }
                self.next_abort_ms = now_ms + RESEND_MS;
                return Some(CalMsg::Abort(a));
            }
        }
        if !matches!(self.state, State::AwaitAck | State::Apply) {
            return None;
        }
        if self.sent_once && now_ms - self.last_sent_ms < RESEND_MS {
            return None;
        }
        self.sent_once = true;
        self.last_sent_ms = now_ms;
        if self.state == State::AwaitAck {
            self.reached_drone = true;
            return Some(CalMsg::Sweep(self.cmd));
        }
        Some(CalMsg::Apply(self.apply))
    }

    /// A received calibration frame; `air_bytes` = dot11 header + body +
    /// FCS, `now_ms` its own receive stamp (a capacity point is timed by
    /// when its frames arrived). Counted only when its kind, width and
    /// flags match the running sweep; else `stamp_mismatch`.
    pub fn on_cal_frame(
        &mut self,
        card: usize,
        f: &FrameInfo,
        rssi_dbm: i32,
        crc_ok: bool,
        air_bytes: usize,
        now_ms: u64,
    ) {
        let rx_ms = now_ms;
        let now_ms = self.advance(now_ms);
        self.step(now_ms);
        if card > 1 || f.rate > 7 {
            return;
        }
        if self.state == State::AwaitAck {
            // The ack rides a lossy uplink; a clean frame of the commanded
            // sweep proves the drone accepted and carries the flags. Never
            // a corrupt one: a flipped LDPC/STBC bit would filter the sweep.
            if f.kind != self.cmd.kind || f.bw != self.cmd.bw {
                self.stamp_mismatch += 1;
                return;
            }
            if !crc_ok {
                return;
            }
            self.on_accepted(f.flags, 0, now_ms);
        }
        match self.state {
            State::Sweep | State::AwaitDone => {
                if f.kind != self.cmd.kind || f.bw != self.cmd.bw || f.flags != self.ack_flags {
                    self.stamp_mismatch += 1;
                    return;
                }
                if self.cmd.kind == Kind::Power {
                    self.tally_power(card, f, rssi_dbm, crc_ok);
                } else {
                    self.tally_capacity(card, f, rssi_dbm, crc_ok, air_bytes, rx_ms);
                }
            }
            State::Verify => {
                if f.kind != Kind::Verify || f.bw != self.cmd.bw || f.flags != self.ack_flags {
                    self.stamp_mismatch += 1;
                    return;
                }
                self.tally_verify(card, f, crc_ok);
            }
            _ => {}
        }
    }

    /// True whenever a GS transmit would corrupt the measurement: from the
    /// ack until the sweep's listen window closes, and through verify.
    pub fn radio_silent(&self, now_ms: u64) -> bool {
        let now_ms = now_ms.max(self.now);
        matches!(self.state, State::Sweep | State::AwaitDone | State::Verify)
            && now_ms < self.window_end_ms
    }

    pub fn running(&self) -> bool {
        self.state.running()
    }

    /// Abandons the run: Aborts go out through due_mav if the run reached
    /// the drone, the air reopens, Idle; the snapshot keeps `why`.
    /// `picker` is the one the run started on; a pick in progress is
    /// cancelled through it.
    pub fn abort(&mut self, why: &str, picker: Option<&mut dyn CalPicker>) {
        if self.running() || self.state == State::Picking {
            if let Some(p) = picker {
                p.cancel();
            }
            if self.reached_drone {
                self.queue_abort();
            }
            self.fail_reason = why.to_string();
            self.log.flush();
        }
        self.state = State::Idle;
    }

    /// Where cal.log goes; a run in progress keeps writing to the old one.
    pub fn set_log(&mut self, log: CalLog) {
        if !self.running() {
            self.log = log;
        }
    }

    /// Told every tick: link state, CAP_CALIBRATE, the pair's tag context.
    pub fn set_peer(&mut self, linked: bool, cal_capable: bool, ctx: TagCtx) {
        self.linked = linked;
        self.cal_capable = cal_capable;
        self.peer_ctx = ctx;
    }

    /// The context every cal message of this run is packed under, frozen
    /// when the run first commands the drone: the drone latches the pair
    /// its first SweepCmd arrived under.
    pub fn tag_ctx(&self) -> TagCtx {
        self.tag_ctx
    }
    pub fn state(&self) -> State {
        self.state
    }
    pub fn sweeps(&self) -> &[SweepSpec] {
        &self.sweeps
    }

    pub fn snapshot(&self) -> CalSnapshot {
        let mut s = CalSnapshot {
            state: self.state,
            sweep_i: self.sweep_i,
            sweep_n: self.sweeps.len(),
            cur: SweepSpec {
                kind: Kind::Power,
                bw: 20,
            },
            cell_i: self.cell_i,
            cell_n: 0,
            channel: self.channel,
            pick_note: self.pick_note.clone(),
            flags: self.ack_flags,
            stamp_mismatch: self.stamp_mismatch,
            width: self.width.clone(),
            fail_reason: self.fail_reason.clone(),
        };
        if let Some(cur) = self.sweeps.get(self.sweep_i) {
            s.cur = *cur;
            let rates = self.cmd.mcs_mask.count_ones();
            s.cell_n = if self.state == State::Verify {
                self.verify_cells.len() as u32
            } else if self.cmd.kind == Kind::Capacity {
                rates
            } else {
                rates * self.cells_per_row() as u32
            };
        }
        s
    }

    /// CRC-clean frames tallied on `card` into the power cell (rate, idx).
    pub fn cell_received(&self, rate: u8, idx: i32, card: usize) -> u16 {
        if rate > 7 || card > 1 {
            return 0;
        }
        let j = idx - i32::from(self.cmd.idx_lo);
        usize::try_from(j)
            .ok()
            .and_then(|j| self.cells[rate as usize].get(j))
            .map_or(0, |c| c.received[card])
    }

    // ---- private ----

    fn step(&mut self, now_ms: u64) {
        match self.state {
            State::AwaitAck if now_ms - self.phase_start_ms >= self.cfg.ack_timeout_ms => {
                self.fail("calibration ack timeout".into());
            }
            State::Sweep | State::AwaitDone => {
                // SweepDone normally ends this early; its loss costs at
                // most phase_slack_ms.
                if self.done.is_some() && self.tail_quiet(now_ms) {
                    self.finish_sweep(now_ms);
                    return;
                }
                if self.state == State::Sweep {
                    if now_ms < self.sweep_end_ms {
                        return;
                    }
                    self.state = State::AwaitDone;
                }
                if now_ms >= self.window_end_ms {
                    self.finish_sweep(now_ms);
                }
            }
            State::Apply if now_ms - self.phase_start_ms >= self.cfg.apply_timeout_ms => {
                self.fail("calibration apply timeout".into());
            }
            State::Verify if now_ms >= self.window_end_ms => self.finish_verify(now_ms),
            _ => {}
        }
    }

    fn begin_sweep(&mut self, i: usize, now_ms: u64) {
        self.sweep_i = i;
        let sp = self.sweeps[i];
        let seq = i as u8;
        self.cmd = if sp.kind == Kind::Power {
            make_power_sweep(self.nonce, seq, sp.bw)
        } else {
            make_capacity_sweep(self.nonce, seq, sp.bw)
        };
        // Frozen at the first command: the GS keeps beaconing between
        // sweeps and may adopt a fresh pair meanwhile.
        if i == 0 {
            self.tag_ctx = self.peer_ctx;
        }
        let n = if sp.kind == Kind::Power {
            self.cells_per_row()
        } else {
            0
        };
        let lo = i32::from(self.cmd.idx_lo);
        for r in 0..8 {
            self.cells[r] = (0..n)
                .map(|j| CalCell {
                    idx: lo + j as i32,
                    ..CalCell::default()
                })
                .collect();
            self.rssi[r] = vec![Default::default(); n];
        }
        self.cap = [CapCell::default(); 8];
        self.cap_rssi = Default::default();
        self.last_cap_seen_ms = 0;
        self.cap_fresh = false;
        self.done = None;
        self.cell_i = 0;
        self.phase_start_ms = now_ms;
        self.sent_once = false;
        self.state = State::AwaitAck;
    }

    fn on_accepted(&mut self, flags: u8, start_in_ms: u16, now_ms: u64) {
        self.ack_flags = flags;
        if !self.run_logged {
            // The GS has no LDPC setting: the drone's stamp flags say what
            // this run flies.
            self.log.run(
                self.nonce,
                self.cfg.margin_db,
                self.channel,
                self.pick,
                flags & FLAG_LDPC != 0,
            );
            self.run_logged = true;
        }
        let param = if self.cmd.kind == Kind::Capacity {
            self.cmd.dwell_ms
        } else {
            self.cmd.frames_per_cell
        };
        self.log
            .sweep(self.cmd.seq, self.cmd.kind, self.cmd.bw, u32::from(param));
        self.sweep_end_ms =
            now_ms + u64::from(start_in_ms) + u64::from(sweep_duration_ms(&self.cmd));
        self.window_end_ms = self.sweep_end_ms + self.cfg.phase_slack_ms;
        self.state = State::Sweep;
    }

    fn tally_power(&mut self, card: usize, f: &FrameInfo, rssi_dbm: i32, crc_ok: bool) {
        let idx = i32::from(f.idx);
        if idx < i32::from(self.cmd.idx_lo) || idx > i32::from(self.cmd.idx_hi) {
            return;
        }
        let j = (idx - i32::from(self.cmd.idx_lo)) as usize;
        let r = usize::from(f.rate);
        self.cell_i = (r * self.cells_per_row() + j + 1) as u32;
        let c = &mut self.cells[r][j];
        if !crc_ok {
            // CRC-bad frames reach the GS: loss, never delivery.
            c.corrupt = c.corrupt.saturating_add(1);
            return;
        }
        c.received[card] = c.received[card].saturating_add(1);
        self.rssi[r][j][card].push(rssi_dbm);
    }

    fn tally_capacity(
        &mut self,
        card: usize,
        f: &FrameInfo,
        rssi_dbm: i32,
        crc_ok: bool,
        air_bytes: usize,
        rx_ms: u64,
    ) {
        if self.cmd.mcs_mask & (1 << f.rate) == 0 {
            return;
        }
        self.cell_i = (self.cmd.mcs_mask & ((1u8 << f.rate) - 1)).count_ones() + 1;
        // Any frame, clean or not, says the air is still busy.
        self.last_cap_seen_ms = self.last_cap_seen_ms.max(self.now);
        self.cap_fresh = true;
        if !crc_ok {
            return;
        }
        let c = &mut self.cap[usize::from(f.rate)];
        let bits = air_bytes as u64 * 8;
        if c.frames[card] == 0 || rx_ms < c.first_ms[card] {
            c.first_ms[card] = rx_ms;
            c.first_bits[card] = bits;
        }
        if c.frames[card] == 0 || rx_ms > c.last_ms[card] {
            c.last_ms[card] = rx_ms;
        }
        c.frames[card] += 1;
        c.air_bits[card] += bits;
        self.cap_rssi[usize::from(f.rate)][card].push(rssi_dbm);
    }

    fn tail_quiet(&self, now_ms: u64) -> bool {
        self.cmd.kind != Kind::Capacity
            || (!self.cap_fresh && now_ms >= self.last_cap_seen_ms + self.cfg.tail_quiet_ms)
    }

    fn tally_verify(&mut self, card: usize, f: &FrameInfo, crc_ok: bool) {
        // By rate, whatever index arrives: the drone parks at its own
        // margin, and an exact-index match would read a margin
        // disagreement as a total verify failure.
        let Some(pos) = self.verify_cells.iter().rposition(|c| c.0 == f.rate) else {
            return;
        };
        self.cell_i = pos as u32 + 1;
        let r = usize::from(f.rate);
        if !self.verify_seen[r] {
            self.verify_seen[r] = true;
            self.verify_seen_idx[r] = i32::from(f.idx);
        }
        if crc_ok {
            self.verify_recv[r][card] = self.verify_recv[r][card].saturating_add(1);
        }
    }

    fn finish_sweep(&mut self, now_ms: u64) {
        self.state = State::Analyze;
        let apply = if self.cmd.kind == Kind::Power {
            self.analyze_power()
        } else {
            self.analyze_capacity_sweep()
        };
        if apply {
            self.phase_start_ms = now_ms;
            self.sent_once = false;
            self.state = State::Apply;
        } else {
            self.next_sweep(now_ms);
        }
    }

    fn fresh_apply(&self) -> ApplyCmd {
        ApplyCmd {
            nonce: self.nonce,
            seq: self.cmd.seq,
            bw: self.cmd.bw,
            ..ApplyCmd::default()
        }
    }

    fn analyze_power(&mut self) -> bool {
        let n = self.cells_per_row();
        let mut apply = self.fresh_apply();
        let mut rows: [Vec<CalCell>; 8] = Default::default();
        for (r, row) in rows.iter_mut().enumerate() {
            // Expected per cell: what the drone says it offered on this row
            // over the row's cells; the commanded count when the SweepDone
            // was lost. A drone dead mid-row reads as loss either way.
            let mut expected = u32::from(self.cmd.frames_per_cell);
            if let Some(d) = &self.done {
                let per = if n > 0 { d.offered[r] / n as u32 } else { 0 };
                if per > 0 {
                    expected = per.min(0xFFFF);
                }
            }
            for (j, cell) in self.cells[r].iter().enumerate() {
                let mut c = *cell;
                c.expected = expected as u16;
                for k in 0..2 {
                    let samples = &self.rssi[r][j][k];
                    if !samples.is_empty() {
                        c.rssi_dbm[k] = median(samples.clone());
                        c.have_rssi[k] = true;
                    }
                }
                row.push(c);
            }
        }
        for (r, row) in rows.iter().enumerate() {
            for c in row {
                self.log.cell(r as u8, c.idx, c);
            }
        }
        let mut any = false;
        let mut walls = [RateWall::default(); 8];
        for (r, row) in rows.iter().enumerate() {
            let rw = analyze_rate(row, &self.cfg.th);
            walls[r] = rw;
            self.log.wall(r as u8, &rw);
            // An undetermined wall is left alone: never a fabricated number
            // over a real measurement.
            if rw.wall != NO_WALL {
                apply.walls[r] = rw.wall as i8;
                any = true;
            }
        }
        apply.has_walls = any;
        // legacy_wall_rel is one number for every width; the 20 MHz table
        // owns it whenever this run measures one.
        apply.legacy = if self.cmd.bw == 20 || !self.has_power20() {
            apply.walls[0]
        } else {
            WALL_LEAVE
        };
        let w = self.width_result();
        w.walls = walls;
        w.have_walls = true;
        self.apply = apply;
        self.apply_wrote_walls = any;
        any
    }

    fn analyze_capacity_sweep(&mut self) -> bool {
        let mut apply = self.fresh_apply();
        self.apply_wrote_walls = false;
        let Some(done) = self.done else {
            // Without SweepDone neither offered nor blocked_pct is known, so
            // a tx_bound point cannot be told apart: nothing is written.
            let w = self.width_result();
            w.eff_unknown = true;
            w.have_eff = false;
            w.cap = [CapResult::default(); 8];
            w.eff_x1000 = [0; 8];
            self.apply = apply;
            return false;
        };
        let mut any = false;
        let mut caps = [CapResult::default(); 8];
        for (r, cap) in caps.iter_mut().enumerate() {
            if self.cmd.mcs_mask & (1 << r) == 0 {
                continue;
            }
            let mut c = self.cap[r];
            c.offered = done.offered[r];
            c.blocked_pct = done.blocked_pct[r];
            for k in 0..2 {
                if !self.cap_rssi[r][k].is_empty() {
                    c.rssi_dbm[k] = median(self.cap_rssi[r][k].clone());
                    c.have_rssi[k] = true;
                }
            }
            let res = analyze_capacity(
                &c,
                r as u8,
                self.cmd.bw,
                u32::from(self.cmd.dwell_ms),
                &self.cfg.th,
            );
            *cap = res;
            // A tx_bound point measured the drone's CPU/USB and a lost one
            // the loss, not the air: 0 leaves the config's value.
            let e = if res.flags & (FLAG_TX_BOUND | FLAG_LOST) != 0 {
                0
            } else {
                (res.eff * 1000.0).round() as u16
            };
            apply.eff_x1000[r] = e;
            any |= e != 0;
            self.log.cap(r as u8, &c, &res);
        }
        apply.has_eff = any;
        let w = self.width_result();
        w.cap = caps;
        w.eff_x1000 = apply.eff_x1000;
        w.have_eff = true;
        w.eff_unknown = false;
        self.apply = apply;
        any
    }

    fn on_applied(&mut self, a: &Applied, now_ms: u64) {
        self.log.applied(a.bw, a.ok, &a.reason);
        let w = self.width_result();
        w.applied = a.ok;
        w.apply_reason = a.reason.clone();
        if !a.ok {
            self.fail(format!("apply refused by the drone: {}", a.reason));
            return;
        }
        if self.cmd.kind == Kind::Power && self.apply_wrote_walls {
            self.begin_verify(now_ms);
        } else {
            self.next_sweep(now_ms);
        }
    }

    fn begin_verify(&mut self, now_ms: u64) {
        // The rule the drone parks its verify cells by, so the window
        // covers exactly the cells it flies.
        let park = park_rel_for(&self.apply.walls, self.cfg.margin_db);
        self.width_result().park = park;
        self.verify_cells = verify_cells(&park);
        self.verify_recv = [[0; 2]; 8];
        self.verify_seen = [false; 8];
        self.verify_seen_idx = [0; 8];
        self.cell_i = 0;
        self.window_end_ms = now_ms
            + u64::from(verify_duration_ms(self.verify_cells.len()))
            + self.cfg.phase_slack_ms;
        self.state = State::Verify;
    }

    fn finish_verify(&mut self, now_ms: u64) {
        // One V row per parked rate, heard or not: a rate dead at its own
        // parked power is what this pass exists to surface.
        let cells = self.verify_cells.clone();
        for (rate, idx) in cells {
            let r = usize::from(rate);
            let best = self.verify_recv[r][0].max(self.verify_recv[r][1]);
            let pct =
                ((100.0 * f64::from(best) / f64::from(VERIFY_FRAMES)).round() as i32).min(100);
            self.width_result().verify_pct[r] = pct;
            let at = if self.verify_seen[r] {
                self.verify_seen_idx[r]
            } else {
                idx
            };
            self.log.verify(rate, at, pct);
        }
        self.next_sweep(now_ms);
    }

    fn next_sweep(&mut self, now_ms: u64) {
        if self.sweep_i + 1 < self.sweeps.len() {
            self.begin_sweep(self.sweep_i + 1, now_ms);
            return;
        }
        self.state = State::Done;
        // Closes the drone's run now, restoring its power and video; the
        // close keeps what the applies wrote.
        self.queue_abort();
        self.log.flush();
    }

    fn fail(&mut self, why: String) {
        if self.reached_drone {
            self.queue_abort();
        }
        self.fail_reason = why;
        self.state = State::Failed;
        self.log.flush();
    }

    fn queue_abort(&mut self) {
        self.pending_abort = Some(Abort { nonce: self.nonce });
        self.aborts_left = ABORT_COPIES;
        self.next_abort_ms = 0;
    }

    fn cells_per_row(&self) -> usize {
        (i32::from(self.cmd.idx_hi) - i32::from(self.cmd.idx_lo) + 1).max(0) as usize
    }

    fn width_result(&mut self) -> &mut WidthResult {
        &mut self.width[usize::from(self.cmd.bw == 40)]
    }

    fn has_power20(&self) -> bool {
        self.sweeps
            .iter()
            .any(|s| s.kind == Kind::Power && s.bw == 20)
    }
}
