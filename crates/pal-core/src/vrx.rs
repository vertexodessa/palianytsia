//! GS link control: rendezvous (DISC / DiscAck), the RCF builder driving the
//! ladder (mabur `gs/src/rendezvous.cpp`, `vrx_controller.cpp`) and the RCF
//! slotter that parks control frames in the gaps between video bursts
//! (`gs/src/rcf_slot.cpp`).
use crate::config::{Config, LadderCfg, Rung};
use crate::ladder::{LadderController, LinkHealth};
use crate::link_health::OpPoint;
use pal_wire::profile::{self, PhyMode, Profile};
use pal_wire::rc::{self, Disc, DiscAck, LinkKey, Rcf, TagCtx};
use std::collections::VecDeque;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VrxState {
    Session,
    Beaconing,
    KeyMismatch,
}

impl VrxState {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Session => "SESSION",
            Self::Beaconing => "BEACONING",
            Self::KeyMismatch => "KEY_MISMATCH",
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum VrxAction {
    TxFeedback,
    Beacon,
    Idle,
}

const KEY_MISMATCH_MS: f64 = 1000.0;
const KEY_MISMATCH_BEACONS: i32 = 3;
const LINK_LOST_MS: f64 = 1000.0;
const BEACON_PERIOD_MS: f64 = 20.0;
// video_flowing(): link video bodies with no gap above VIDEO_GAP_MS for at
// least VIDEO_RUN_MS. The lowest encoder rate (low-power 30 fps) spaces
// frames 33 ms apart; the first frames of a booting encoder (~1.5 fps)
// never qualify.
const VIDEO_GAP_MS: f64 = 100.0;
const VIDEO_RUN_MS: f64 = 200.0;

/// SESSION: we hold the drone's vtx_nonce and may send RCFs. KEY_MISMATCH:
/// a run of flagged acks at least 1 s old with 3 of our DISCs unanswered.
pub struct VrxRendezvous {
    state: VrxState,
    last_video_ms: f64,
    // The current run of link video for video_flowing(): never touched by an ack.
    last_real_video_ms: f64,
    video_run_start_ms: f64,
    last_beacon_ms: f64,
    seq: u16,
    nonce: u32,
    proposal: u8,
    width: u8,
    vtx_nonce: Option<u32>,
    first_flagged_ms: Option<f64>,
    beacons_since_flagged: i32,
}

pub(crate) fn random_nonce() -> u32 {
    let mut b = [0u8; 4];
    loop {
        if let Ok(mut f) = std::fs::File::open("/dev/urandom") {
            use std::io::Read;
            if f.read_exact(&mut b).is_ok() {
                let n = u32::from_le_bytes(b);
                if n != 0 {
                    return n;
                }
                continue;
            }
        }
        // No urandom: time-derived, still never 0.
        let t = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(1, |d| d.subsec_nanos() ^ d.as_secs() as u32);
        return t.max(1);
    }
}

impl VrxRendezvous {
    pub fn new(op_channel: u8, nonce: u32, width: u8) -> Self {
        Self {
            state: VrxState::Beaconing,
            last_video_ms: 0.0,
            last_real_video_ms: -1e18,
            video_run_start_ms: 0.0,
            last_beacon_ms: -1e18,
            seq: 0,
            nonce: if nonce != 0 { nonce } else { random_nonce() },
            proposal: op_channel,
            width,
            vtx_nonce: None,
            first_flagged_ms: None,
            beacons_since_flagged: 0,
        }
    }
    fn feed_video(&mut self, now: f64) {
        self.last_video_ms = now;
        if now - self.last_real_video_ms > VIDEO_GAP_MS {
            self.video_run_start_ms = now;
        }
        self.last_real_video_ms = now;
        if self.state == VrxState::Beaconing && self.vtx_nonce.is_some() {
            self.state = VrxState::Session;
        }
    }
    fn tick(&mut self, now: f64) -> VrxAction {
        if self.state == VrxState::Session {
            if now - self.last_video_ms > LINK_LOST_MS {
                self.state = VrxState::Beaconing;
            } else {
                return VrxAction::TxFeedback;
            }
        }
        if now - self.last_beacon_ms >= BEACON_PERIOD_MS {
            self.last_beacon_ms = now;
            return VrxAction::Beacon;
        }
        VrxAction::Idle
    }
    fn beacon(&mut self) -> Disc {
        self.seq = self.seq.wrapping_add(1);
        if self.first_flagged_ms.is_some() {
            self.beacons_since_flagged += 1;
        }
        Disc {
            vrx_nonce: self.nonce,
            op_channel: self.proposal,
            // The receiver's width, as a proposal the drone retunes to.
            op_width: self.width,
            cap_bits: rc::DISC_CAP_WIDTH,
            seq: self.seq,
            ..Disc::default()
        }
    }
    /// Returns (accepted, adopted_new).
    fn feed_disc_ack(&mut self, ack: &DiscAck, now: f64) -> (bool, bool) {
        if ack.vrx_nonce != self.nonce {
            return (false, false);
        }
        if ack.flags & rc::ACK_KEY_MISMATCH != 0 {
            let first = *self.first_flagged_ms.get_or_insert_with(|| {
                self.beacons_since_flagged = 0;
                now
            });
            if now - first >= KEY_MISMATCH_MS && self.beacons_since_flagged >= KEY_MISMATCH_BEACONS
            {
                self.state = VrxState::KeyMismatch;
            }
            return (true, false);
        }
        self.first_flagged_ms = None;
        self.beacons_since_flagged = 0;
        let adopted = self.vtx_nonce != Some(ack.vtx_nonce);
        self.vtx_nonce = Some(ack.vtx_nonce);
        self.state = VrxState::Session;
        self.last_video_ms = now;
        (true, adopted)
    }
    pub fn state(&self) -> VrxState {
        self.state
    }
    /// Real link video is flowing (feed_video only). SESSION alone is not:
    /// a DISC_ACK opens it and refreshes the silence timer with no video, so
    /// a freshly booted drone whose encoder is not up yet holds SESSION for
    /// LINK_LOST_MS and then drops back to BEACONING, taking the RCFs with it.
    pub fn video_flowing(&self, now: f64) -> bool {
        now - self.last_real_video_ms <= VIDEO_GAP_MS
            && self.last_real_video_ms - self.video_run_start_ms >= VIDEO_RUN_MS
    }
    pub fn nonce(&self) -> u32 {
        self.nonce
    }
    pub fn proposal(&self) -> u8 {
        self.proposal
    }
    pub fn set_proposal(&mut self, ch: u8) {
        self.proposal = ch;
    }
    pub fn vtx_nonce(&self) -> Option<u32> {
        self.vtx_nonce
    }
}

#[derive(Clone, Debug)]
pub struct VrxCfg {
    pub key: LinkKey,
    pub op_channel: u8,
    pub feedback_ms: i32,
    pub beacon_keepalive_ms: i32,
    pub ladder: LadderCfg,
    pub pin_mcs: i32,
    pub pin_bw: i32,
    pub pin_overhead_base: f64,
    pub pin_overhead_enh: f64,
    pub probe_pin_mcs: i32,
    pub rz_nonce: u32,
    pub width: u8,
}

impl VrxCfg {
    /// mabur `vrx_cfg_from`.
    pub fn from_config(c: &Config, op_channel: u8, rz_nonce: u32) -> Self {
        Self {
            key: c.link.key,
            op_channel,
            feedback_ms: c.link.feedback_ms,
            beacon_keepalive_ms: c.link.beacon_keepalive_ms,
            ladder: c.link.ladder_cfg.clone(),
            pin_mcs: c.link.static_mcs,
            pin_bw: c.link.static_bw,
            pin_overhead_base: c.link.static_overhead_base,
            pin_overhead_enh: c.link.static_overhead_enh,
            probe_pin_mcs: c.link.ladder_cfg.probe.pin_mcs,
            rz_nonce,
            width: c.rx_width(),
        }
    }
}

pub struct VrxOut {
    pub frame: Vec<u8>,
    pub is_disc: bool,
}

/// A DISC re-tagged for channel `ch` (the scout beacons the proposal on every
/// candidate). Non-DISC bodies, ch 0 and an already-matching DISC pass through.
pub fn disc_for_channel(disc: &[u8], ch: u8, key: &LinkKey) -> Vec<u8> {
    if ch == 0 || rc::frame_type(disc) != Some(rc::T_DISC) {
        return disc.to_vec();
    }
    match rc::parse_disc(disc) {
        Some(mut d) if d.op_channel != ch => {
            d.op_channel = ch;
            rc::pack_disc(&d, key)
        }
        _ => disc.to_vec(),
    }
}

fn op_from_rung(r: &Rung) -> OpPoint {
    OpPoint {
        vht: false,
        mcs: r.mcs,
        bw: r.bw,
        overhead_base: r.overhead_base,
        overhead_enh: r.overhead_enh,
    }
}

pub const MOVE_AFTER_RCFS: i32 = 5;

pub struct VrxController {
    cfg: VrxCfg,
    ctrl: LadderController,
    rz: VrxRendezvous,
    last_fb_ms: f64,
    last_keepalive_ms: f64,
    seq32: u32,
    cur_op: OpPoint,
    peer_caps: u16,
    peer_acked: bool,
    keepalive_hold: bool,
    last_cmd_probe_profile: u8,
    agreed_channel: u8,
    move_armed: bool,
    move_edge: bool,
    rcfs_since_adopt: i32,
    hop_ch: u8,
    hop_epoch: u8,
    rec_wish: u8,
    idr_epoch: u8,
    snr_off: i16,
}

impl VrxController {
    pub fn new(cfg: VrxCfg) -> Self {
        let ctrl = LadderController::new(cfg.ladder.clone());
        let cur_op = op_from_rung(ctrl.op());
        Self {
            rz: VrxRendezvous::new(cfg.op_channel, cfg.rz_nonce, cfg.width),
            cfg,
            ctrl,
            last_fb_ms: -1e18,
            last_keepalive_ms: -1e18,
            seq32: 0,
            cur_op,
            peer_caps: 0,
            peer_acked: false,
            keepalive_hold: false,
            last_cmd_probe_profile: rc::NO_PROBE_PROFILE,
            agreed_channel: 0,
            move_armed: false,
            move_edge: false,
            rcfs_since_adopt: 0,
            hop_ch: 0,
            hop_epoch: 0,
            rec_wish: 0,
            idr_epoch: 0,
            snr_off: 0,
        }
    }

    fn sync_op(&mut self) {
        self.cur_op = op_from_rung(self.ctrl.op());
    }
    pub fn on_video(&mut self, now: f64) {
        self.rz.feed_video(now);
    }
    pub fn on_rc_frame(&mut self, b: &[u8], now: f64) {
        if rc::frame_type(b) != Some(rc::T_DISC_ACK) {
            return;
        }
        let Some(ack) = rc::parse_disc_ack(b) else {
            return;
        };
        let (ok, adopted) = self.rz.feed_disc_ack(&ack, now);
        if !ok || ack.flags & rc::ACK_KEY_MISMATCH != 0 {
            return;
        }
        self.agreed_channel = ack.agreed_channel;
        self.peer_caps = ack.chip_caps;
        self.peer_acked = true;
        if adopted {
            // A new drone session: the loss that struck the rungs was the old one's.
            self.ctrl.clear_lockout();
            self.seq32 = 0;
            self.rcfs_since_adopt = 0;
            self.move_armed = true;
            self.move_edge = false;
        }
    }
    pub fn note_drone_state(&mut self, telem_state: u8) {
        if telem_state == 2 && self.move_armed {
            self.move_edge = true;
            self.move_armed = false;
        }
    }

    pub fn step(&mut self, now: f64, health: &LinkHealth) -> Option<VrxOut> {
        if self.cfg.pin_mcs >= 0 {
            self.cur_op = OpPoint {
                vht: false,
                mcs: self.cfg.pin_mcs,
                bw: self.cfg.pin_bw,
                overhead_base: self.cfg.pin_overhead_base,
                overhead_enh: self.cfg.pin_overhead_enh,
            };
        } else if self.ctrl.on_tick(now) {
            self.sync_op();
        }
        match self.rz.tick(now) {
            VrxAction::Beacon => {
                let d = self.rz.beacon();
                return Some(VrxOut {
                    frame: rc::pack_disc(&d, &self.cfg.key),
                    is_disc: true,
                });
            }
            VrxAction::Idle => return None,
            VrxAction::TxFeedback => {}
        }
        // SESSION keep-alive DISC, in place of this tick's RCF.
        if !self.keepalive_hold
            && now - self.last_keepalive_ms >= f64::from(self.cfg.beacon_keepalive_ms)
        {
            self.last_keepalive_ms = now;
            let d = self.rz.beacon();
            return Some(VrxOut {
                frame: rc::pack_disc(&d, &self.cfg.key),
                is_disc: true,
            });
        }
        if now - self.last_fb_ms < f64::from(self.cfg.feedback_ms) {
            return None;
        }
        self.last_fb_ms = now;
        if self.cfg.pin_mcs < 0 && self.ctrl.update(health, now) {
            self.sync_op();
        }
        let r = self.build_rcf();
        self.last_cmd_probe_profile = r.probe_profile;
        if self.move_armed {
            self.rcfs_since_adopt += 1;
            if self.rcfs_since_adopt >= MOVE_AFTER_RCFS {
                self.move_edge = true;
                self.move_armed = false;
            }
        }
        let vtx = self.rz.vtx_nonce().expect("SESSION holds a vtx_nonce");
        let ctx = TagCtx {
            vrx_nonce: self.rz.nonce(),
            vtx_nonce: vtx,
            seq32: self.seq32,
        };
        Some(VrxOut {
            frame: rc::pack_rcf(&r, &self.cfg.key, &ctx),
            is_disc: false,
        })
    }

    fn build_rcf(&mut self) -> Rcf {
        self.seq32 = self.seq32.wrapping_add(1);
        let mode = if self.cur_op.vht {
            PhyMode::Vht
        } else {
            PhyMode::Ht
        };
        let enc = |mcs: i32, bw: i32| {
            profile::encode(Profile {
                mode,
                mcs: mcs as u8,
                bw_mhz: bw as u8,
            })
        };
        let mut r = Rcf {
            seq: self.seq32 as u16,
            profile: enc(self.cur_op.mcs, self.cur_op.bw),
            fec_overhead_base: self.cur_op.overhead_base,
            fec_overhead_enh: self.cur_op.overhead_enh,
            hop_ch: self.hop_ch,
            hop_epoch: self.hop_epoch,
            rec: self.rec_wish,
            idr_epoch: self.idr_epoch,
            snr_off: self.snr_off,
            probe_profile: rc::NO_PROBE_PROFILE,
        };
        if self.cfg.pin_mcs >= 0 {
            if self.cfg.probe_pin_mcs >= 0 {
                r.probe_profile = enc(self.cfg.probe_pin_mcs, self.cur_op.bw);
            }
        } else {
            let pr = self.ctrl.probe_rung();
            if pr >= 0 {
                let p = &self.cfg.ladder.ladder[pr as usize];
                r.probe_profile = enc(p.mcs, p.bw);
            }
        }
        r
    }

    /// `rung < 0`: the verdict froze no reference, as on a move off a healthy
    /// link. Stay on the current rung; rung 0 would cut the link's capacity
    /// to its floor while the encoder and the queue still run at this rate.
    pub fn restore_rung(&mut self, rung: i32, now: f64) {
        let rung = if rung < 0 { self.ctrl.rung() } else { rung };
        self.ctrl.restore(rung, now);
        self.sync_op();
    }
    pub fn rz_nonce(&self) -> u32 {
        self.rz.nonce()
    }
    pub fn proposal(&self) -> u8 {
        self.rz.proposal()
    }
    pub fn cur_op(&self) -> &OpPoint {
        &self.cur_op
    }
    pub fn ctl(&self) -> &LadderController {
        &self.ctrl
    }
    pub fn link_state(&self) -> VrxState {
        self.rz.state()
    }
    /// VrxRendezvous::video_flowing: link video is flowing, not just a session.
    pub fn video_flowing(&self, now: f64) -> bool {
        self.rz.video_flowing(now)
    }
    pub fn rcf_seq(&self) -> u16 {
        self.seq32 as u16
    }
    pub fn rcf_seq32(&self) -> u32 {
        self.seq32
    }
    pub fn probe_profile(&self) -> u8 {
        self.last_cmd_probe_profile
    }
    pub fn peer_caps(&self) -> u16 {
        self.peer_caps
    }
    pub fn peer_acked(&self) -> bool {
        self.peer_acked
    }
    pub fn agreed_channel(&self) -> u8 {
        self.agreed_channel
    }
    pub fn key_mismatch(&self) -> bool {
        self.rz.state() == VrxState::KeyMismatch
    }
    #[cfg(any(test, feature = "fake"))]
    pub fn test_set_move_edge(&mut self) {
        self.move_edge = true;
    }
    pub fn take_move_edge(&mut self) -> bool {
        std::mem::take(&mut self.move_edge)
    }
    pub fn set_rec_wish(&mut self, w: u8) {
        self.rec_wish = w;
    }
    pub fn set_idr_epoch(&mut self, e: u8) {
        self.idr_epoch = e;
    }
    pub fn set_keepalive_hold(&mut self, h: bool) {
        self.keepalive_hold = h;
    }
    /// In-flight hop: stamped into every RCF built from now on.
    pub fn set_hop(&mut self, ch: u8, epoch: u8) {
        self.hop_ch = ch;
        self.hop_epoch = epoch;
    }
    pub fn hop_ch(&self) -> u8 {
        self.hop_ch
    }
    pub fn hop_epoch(&self) -> u8 {
        self.hop_epoch
    }
    pub fn blank_store(&mut self, until_ms: f64) {
        self.ctrl.blank_store(until_ms);
    }
    pub fn set_proposal(&mut self, ch: u8) {
        self.rz.set_proposal(ch);
    }
    pub fn session_ctx(&self) -> TagCtx {
        TagCtx {
            vrx_nonce: self.rz.nonce(),
            vtx_nonce: self.rz.vtx_nonce().unwrap_or(0),
            seq32: 0,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct RcfSlotCfg {
    pub hold_max_ms: i32,
    pub video_recent_ms: i32,
    pub grace_ms: i32,
    pub lead_ms: i32,
    pub guard_ms: i32,
    pub probe_tail_ms: i32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum SlotReason {
    #[default]
    Passthru,
    Grace,
    Au,
    Timeout,
    Probe,
}

#[derive(Clone, Debug, Default)]
pub struct SlotFrame {
    pub frame: Vec<u8>,
    pub seq: u16,
    pub card: i32,
    pub stamp_rtt: bool,
    pub reason: SlotReason,
    pub offered_ms: u64,
}

const TAIL_MAX_MS: f64 = 15.0;

/// Holds control frames for the idle gap after a video burst instead of
/// colliding with the next aggregate.
pub struct RcfSlotter {
    cfg: RcfSlotCfg,
    pending: VecDeque<SlotFrame>,
    hold_start_ms: u64,
    last_au_ms: u64,
    last_first_ms: u64,
    have_au: bool,
    have_first: bool,
    period_ms: f64,
    release_pending: bool,
    release_reason: SlotReason,
    probe_wait: bool,
    probe_wait_from_ms: u64,
    probe_deadline_ms: u64,
    idle_from_ms: u64,
    tail_est_ms: f64,
    pub released_au: u64,
    pub released_timeout: u64,
    pub released_probe: u64,
    pub passthru: u64,
}

impl RcfSlotter {
    pub fn new(cfg: RcfSlotCfg) -> Self {
        Self {
            cfg,
            pending: VecDeque::new(),
            hold_start_ms: 0,
            last_au_ms: 0,
            last_first_ms: 0,
            have_au: false,
            have_first: false,
            period_ms: 16.667,
            release_pending: false,
            release_reason: SlotReason::Au,
            probe_wait: false,
            probe_wait_from_ms: 0,
            probe_deadline_ms: 0,
            idle_from_ms: 0,
            tail_est_ms: 0.0,
            released_au: 0,
            released_timeout: 0,
            released_probe: 0,
            passthru: 0,
        }
    }
    pub fn on_au_first(&mut self, now: u64) {
        if self.have_first && now > self.last_first_ms {
            let iv = (now - self.last_first_ms) as f64;
            if (5.0..=100.0).contains(&iv) {
                self.period_ms += (iv - self.period_ms) / 8.0;
            }
        }
        self.last_first_ms = now;
        self.have_first = true;
    }
    pub fn idle_ahead(&self, now: u64) -> bool {
        if !self.have_first {
            return true;
        }
        let next = self.last_first_ms as f64 + self.period_ms;
        (now as f64) + f64::from(self.cfg.lead_ms) < next - f64::from(self.cfg.guard_ms)
    }
    pub fn set_probe_tail_ms(&mut self, ms: i32) {
        self.cfg.probe_tail_ms = ms.max(0);
    }
    pub fn tail_ub_ms(&self) -> i32 {
        let floor = f64::from(self.cfg.probe_tail_ms);
        let est = self.tail_est_ms.max(floor);
        est.ceil() as i32 + 1
    }
    pub fn on_au_complete(&mut self, now: u64, probe_follows: bool) {
        self.last_au_ms = now;
        self.have_au = true;
        if probe_follows {
            self.probe_wait = true;
            self.probe_wait_from_ms = now;
            self.probe_deadline_ms = now + self.tail_ub_ms() as u64;
            self.idle_from_ms = self.probe_deadline_ms;
            return;
        }
        self.probe_wait = false;
        self.idle_from_ms = now;
        if !self.pending.is_empty() && self.idle_ahead(now) {
            self.release_pending = true;
            self.release_reason = SlotReason::Au;
        }
    }
    pub fn on_probe_tail(&mut self, now: u64) {
        if self.probe_wait && now >= self.probe_wait_from_ms {
            let obs = (now - self.probe_wait_from_ms) as f64;
            self.tail_est_ms *= 0.98;
            if obs > self.tail_est_ms {
                self.tail_est_ms = obs;
            }
            self.tail_est_ms = self.tail_est_ms.min(TAIL_MAX_MS);
        }
        self.probe_wait = false;
        self.idle_from_ms = now;
        if !self.pending.is_empty() && !self.release_pending && self.idle_ahead(now) {
            self.release_pending = true;
            self.release_reason = SlotReason::Probe;
        }
    }
    /// true = held (it will come out of take_due); false = send it now.
    pub fn offer(&mut self, f: &mut SlotFrame, now: u64, bypass: bool) -> bool {
        f.offered_ms = now;
        let enabled = self.cfg.hold_max_ms > 0;
        let video = self.have_au
            && now >= self.last_au_ms
            && now - self.last_au_ms <= self.cfg.video_recent_ms as u64;
        if !bypass && !self.pending.is_empty() {
            self.pending.push_back(std::mem::take(f));
            return true;
        }
        if !enabled || bypass || !video {
            self.passthru += 1;
            f.reason = SlotReason::Passthru;
            return false;
        }
        if now >= self.idle_from_ms
            && now - self.idle_from_ms <= self.cfg.grace_ms as u64
            && self.idle_ahead(now)
        {
            self.released_au += 1;
            f.reason = SlotReason::Grace;
            return false;
        }
        if self.pending.is_empty() {
            self.hold_start_ms = now;
        }
        self.pending.push_back(std::mem::take(f));
        true
    }
    pub fn take_due(&mut self, now: u64) -> Vec<SlotFrame> {
        if self.pending.is_empty() {
            self.release_pending = false;
            if self.probe_wait && now >= self.probe_deadline_ms {
                self.probe_wait = false;
            }
            return Vec::new();
        }
        if self.probe_wait && now >= self.probe_deadline_ms {
            self.probe_wait = false;
            if !self.release_pending && self.idle_ahead(now) {
                self.release_pending = true;
                self.release_reason = SlotReason::Au;
            }
        }
        let due = self.release_pending;
        let expired =
            now >= self.hold_start_ms && now - self.hold_start_ms >= self.cfg.hold_max_ms as u64;
        if !due && !expired {
            return Vec::new();
        }
        let reason = if due {
            self.release_reason
        } else {
            SlotReason::Timeout
        };
        let n = self.pending.len() as u64;
        match reason {
            SlotReason::Probe => self.released_probe += n,
            SlotReason::Au => self.released_au += n,
            _ => self.released_timeout += n,
        }
        self.release_pending = false;
        self.pending
            .drain(..)
            .map(|mut f| {
                f.reason = reason;
                f
            })
            .collect()
    }
    pub fn period_ms(&self) -> f64 {
        self.period_ms
    }
    pub fn pending(&self) -> usize {
        self.pending.len()
    }
}
