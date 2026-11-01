//! The Rust receive core, wired as mabur's WebGs wires it (`web/src/web_gs.cpp`):
//! Aggregator -> UepDecoder -> FrameStream -> Au, and in GS mode the control
//! side: rendezvous, ladder, RCF slotting, NACK, the MAVLink tunnel and the
//! parameter client. No ChannelCore: pal drives one card, like a page with
//! no roster.
use crate::aggregator::{Aggregator, Route};
use crate::config::Config;
use crate::frame_stream::{AuLat, FragArrival, FrameOut, FrameStream, FrameStreamCfg};
use crate::gap::{DroneRestartDetector, GapTimeoutPolicy};
use crate::ladder::LinkHealth;
use crate::link_health::{LinkHealthAssembler, LinkHealthInputs};
use crate::mav::{MavTunnel, ParamClient};
use crate::nack::{NackInputs, NackTailView, NackTracker};
use crate::uplink::{RttEstimator, UplinkDelivery};
use crate::vrx::{RcfSlotCfg, RcfSlotter, SlotFrame, VrxCfg, VrxController, VrxState};
use crate::{Au, CoreIo, GsCore, Mode, RxBody};
use pal_wire::frame_wire::FrameHdr;
use pal_wire::{rc, sbi, sw};

#[derive(Clone, Copy, Debug)]
pub struct RustOpts {
    pub mode: Mode,
    /// false = fixed `video.frame_gap_timeout_ms`.
    pub adaptive_gap: bool,
    /// Pinned rendezvous vrx_nonce (replay); 0 = random.
    pub rz_nonce: u32,
    /// The channel a DISC proposes; 0 = the first of `radio.channels`.
    pub channel: u8,
}

impl Default for RustOpts {
    fn default() -> Self {
        Self {
            mode: Mode::Spotter,
            adaptive_gap: true,
            rz_nonce: 0,
            channel: 0,
        }
    }
}

#[derive(Default, Clone, Copy, Debug)]
struct Counters {
    bodies: u64,
    aus_complete: u64,
    aus_truncated: u64,
    aus_truncated_base: u64,
    aus_salvaged: u64,
    resets: u64,
}

/// FrameStream's sink: builds the AU in flight and hands it to the host.
struct AuOut<'a> {
    cur: &'a mut Au,
    io: &'a mut dyn CoreIo,
    c: &'a mut Counters,
    now_us: u64,
    gs: Option<&'a mut Gs>,
}

impl FrameOut for AuOut<'_> {
    fn begin(&mut self, h: &FrameHdr, sid: u8, _nslices: u8) {
        *self.cur = Au {
            pts_us: h.pts_us,
            sid,
            flags: h.flags,
            ..Au::default()
        };
        if let Some(g) = self.gs.as_deref_mut() {
            let now_ms = self.now_us / 1000;
            g.slot.on_au_first(now_ms);
            // One probe expectation per video AU.
            g.lha.on_au_begin(sid, h.frame_id, now_ms as f64);
        }
    }
    fn data(&mut self, b: &[u8]) {
        self.cur.data.extend_from_slice(b);
    }
    fn end(&mut self, complete: bool, lat: &AuLat) {
        self.cur.complete = complete;
        self.cur.salvaged = !complete && lat.slice.salvaged;
        if let Some(g) = self.gs.as_deref_mut() {
            let probe_follows = self.cur.sid < 2 && g.lha.probe_commanded() != rc::NO_PROBE_PROFILE;
            g.slot.on_au_complete(self.now_us / 1000, probe_follows);
        }
        if self.cur.salvaged {
            self.c.aus_salvaged += 1;
        }
        if complete {
            self.c.aus_complete += 1;
        } else {
            self.c.aus_truncated += 1;
            if self.cur.sid == 0 {
                self.c.aus_truncated_base += 1;
            }
        }
        self.io.on_au(std::mem::take(self.cur));
    }
}

/// NACK block state (web_gs.cpp poll_nack_/drain_nack_window_).
struct NackState {
    t: NackTracker,
    lookback: u32,
    down_util: f64,
    vtx_seen: u32,
    sent: u64,
    tlm_seen: Option<u16>,
    drone_nack_rx: u64,
    drone_retx_syms: u64,
    drone_retx_refused: u64,
    win_t0_ms: u64,
    fill_pps: f64,
    late_max: u32,
    fill_p50: Option<u32>,
    fill_p90: Option<u32>,
    fill_max: Option<u32>,
}

/// Everything only GS mode owns.
struct Gs {
    vrx: VrxController,
    slot: RcfSlotter,
    lha: LinkHealthAssembler,
    rtt: RttEstimator,
    uplink: UplinkDelivery,
    nack: Option<NackState>,
    mav: MavTunnel,
    params: ParamClient,
    param_linked: bool,
    param_vtx_seen: u32,
    key: rc::LinkKey,
    sends: u64,
    rcf_sent: u64,
    last_health: LinkHealth,
}

impl Gs {
    fn send(&mut self, f: &SlotFrame, now_us: u64, io: &mut dyn CoreIo) -> bool {
        io.send(&f.frame);
        self.sends += 1;
        if f.stamp_rtt {
            self.rcf_sent += 1;
            self.rtt.on_rcf_sent(f.seq, now_us);
            self.uplink.on_rcf_sent(now_us / 1000);
        }
        true
    }

    /// The tunnel's send callback: slotted like an RCF, direct when the
    /// slotter holds nothing.
    fn send_mav_frames(&mut self, frames: Vec<Vec<u8>>, now_us: u64, io: &mut dyn CoreIo) {
        for frame in frames {
            let mut sf = SlotFrame {
                frame,
                ..SlotFrame::default()
            };
            if !self.slot.offer(&mut sf, now_us / 1000, false) {
                self.send(&sf, now_us, io);
            }
        }
    }

    fn drain_params(&mut self) {
        for p in self.params.take_out() {
            self.mav.push_outbound(&p);
        }
    }
}

pub struct RustCore {
    mode: Mode,
    opts: RustOpts,
    now_us: u64,
    agg: Aggregator,
    fs: FrameStream,
    gap: GapTimeoutPolicy,
    gap_update_ms: u64,
    frame_wire: bool,
    restart: DroneRestartDetector,
    telem: Option<rc::Telem>,
    air_mcs: i32,
    bw: u8,
    key_fp: String,
    cur: Au,
    c: Counters,
    gs: Option<Gs>,
}

macro_rules! au_out {
    ($s:expr, $io:expr) => {
        AuOut {
            cur: &mut $s.cur,
            io: $io,
            c: &mut $s.c,
            now_us: $s.now_us,
            gs: $s.gs.as_mut(),
        }
    };
}

impl RustCore {
    pub fn new(cfg: &Config, opts: RustOpts) -> Result<Self, String> {
        let layers = cfg.uep_layers();
        let gs = (opts.mode == Mode::Gs).then(|| {
            let ch = if opts.channel != 0 {
                opts.channel
            } else {
                cfg.radio.channels.first().copied().unwrap_or(0)
            };
            let vc = VrxCfg::from_config(cfg, ch, opts.rz_nonce);
            Gs {
                vrx: VrxController::new(vc),
                slot: RcfSlotter::new(RcfSlotCfg {
                    hold_max_ms: cfg.link.rcf_slot_hold_ms,
                    video_recent_ms: 100,
                    grace_ms: 2,
                    lead_ms: 3,
                    guard_ms: 1,
                    probe_tail_ms: 0,
                }),
                // Probe bodies ride the enh layer's geometry; blocks_per_body
                // is 4 on decode (config.cpp).
                lha: LinkHealthAssembler::new(1, 4, sw::HEADER_LEN + layers[1].symbol_size),
                rtt: RttEstimator::default(),
                uplink: UplinkDelivery::default(),
                nack: cfg.link.nack.enable.then(|| NackState {
                    t: NackTracker::new(cfg.link.nack.clone()),
                    lookback: cfg.link.nack.lookback as u32,
                    down_util: cfg.link.ladder_cfg.down_util,
                    vtx_seen: 0,
                    sent: 0,
                    tlm_seen: None,
                    drone_nack_rx: 0,
                    drone_retx_syms: 0,
                    drone_retx_refused: 0,
                    win_t0_ms: 0,
                    fill_pps: 0.0,
                    late_max: 0,
                    fill_p50: None,
                    fill_p90: None,
                    fill_max: None,
                }),
                mav: MavTunnel::new(cfg.link.key),
                params: ParamClient::default(),
                param_linked: false,
                param_vtx_seen: 0,
                key: cfg.link.key,
                sends: 0,
                rcf_sent: 0,
                last_health: LinkHealth::empty(),
            }
        });
        Ok(Self {
            mode: opts.mode,
            opts,
            now_us: 0,
            agg: Aggregator::new(&layers, cfg.fec.seq_horizon, 1, cfg.link.arrival_guard_syms),
            fs: FrameStream::new(FrameStreamCfg {
                gap_timeout_ms: cfg.video.frame_gap_timeout_ms,
                lookahead: cfg.video.frame_lookahead,
                stall_reset_ms: 500,
            }),
            gap: GapTimeoutPolicy::new(
                cfg.video.frame_gap_timeout_ms,
                cfg.video.frame_gap_timeout_max_ms,
            ),
            gap_update_ms: 0,
            // Spotter: no session to gate on, always decode. GS: only in
            // SESSION with a CAP_FRAME_WIRE peer (tick).
            frame_wire: opts.mode == Mode::Spotter,
            restart: DroneRestartDetector::default(),
            telem: None,
            air_mcs: -1,
            bw: cfg.radio.width,
            key_fp: rc::key_fingerprint(&cfg.link.key),
            cur: Au::default(),
            c: Counters::default(),
            gs,
        })
    }

    fn reset_video(&mut self, io: &mut dyn CoreIo) {
        self.agg.decoder_mut().reset_continuity();
        self.fs.reset(&mut au_out!(self, io));
        if let Some(n) = self.gs.as_mut().and_then(|g| g.nack.as_mut()) {
            n.t.clear();
        }
        self.cur = Au::default();
        self.c.resets += 1;
    }

    fn on_rc(&mut self, body: &[u8], us: u64, io: &mut dyn CoreIo) {
        let ft = rc::frame_type(body);
        if let (Some(rc::T_MAV), Some(g)) = (ft, self.gs.as_mut()) {
            for pkt in g.mav.on_rc_frame(body) {
                io.on_mav(&pkt);
                g.params.on_packet(&pkt, self.now_us / 1000);
            }
            g.drain_params();
            return;
        }
        if ft == Some(rc::T_TELEM) {
            if let Some(t) = rc::parse_telem(body) {
                let (seq, state, flags) = (t.tlm_seq, t.state, t.flags);
                // A maburd restart (new seqs, frame_ids from 0) is the one
                // discontinuity a spotter can see.
                if self.gs.is_none() && self.restart.on_telem(seq, us as f64 / 1000.0) {
                    self.telem = Some(t);
                    self.reset_video(io);
                    return;
                }
                if let Some(g) = self.gs.as_mut() {
                    // RTT pairs telem echoes with our own sends.
                    g.rtt.on_telem(
                        t.rcf_seq_echo,
                        flags & 0x08 != 0,
                        t.rcf_age_ms,
                        t.pts_at_build,
                        us,
                    );
                    g.uplink.on_telem(seq, t.rcf_rx, state, us / 1000);
                    // Drone NACK counters are per Telem period: sum each once.
                    if let Some(n) = g.nack.as_mut().filter(|n| n.tlm_seen != Some(seq)) {
                        n.tlm_seen = Some(seq);
                        n.drone_nack_rx += u64::from(t.nack_rx);
                        n.drone_retx_syms += u64::from(t.retx_syms);
                        n.drone_retx_refused += u64::from(t.retx_refused);
                    }
                }
                self.telem = Some(t);
            }
            return;
        }
        if let Some(g) = self.gs.as_mut() {
            g.vrx.on_rc_frame(body, us as f64 / 1000.0);
        }
    }

    /// Base-layer selective repeat: direct send, never slotted (a slotted
    /// fill lands past the gap timeout).
    fn poll_nack(&mut self, now_ms: u64, io: &mut dyn CoreIo) {
        let Self {
            agg,
            fs,
            gs,
            frame_wire,
            now_us,
            ..
        } = self;
        let Some(g) = gs.as_mut() else { return };
        let Some(n) = g.nack.as_mut() else { return };
        let sctx = g.vrx.session_ctx();
        // != 0: session_ctx() keeps reporting the held nonce in BEACONING.
        if sctx.vtx_nonce != 0 && sctx.vtx_nonce != n.vtx_seen {
            n.vtx_seen = sctx.vtx_nonce;
            n.t.restart_counter();
        }
        if sctx.vtx_nonce == 0 || !*frame_wire {
            n.t.clear();
            return;
        }
        let dec = agg.decoder();
        let lookback = n.lookback;
        let missing = || dec.missing_sources(0, lookback);
        let state = |s: u32| dec.source_state(0, s);
        let tail = || {
            fs.tail_view(0).map(|tv| NackTailView {
                count: tv.count,
                max_idx: tv.max_idx,
                seq_at_max: tv.seq_at_max,
                last_progress_ms: tv.last_progress_ms,
            })
        };
        let vrx = &g.vrx;
        let util = || vrx.ctl().util();
        let inp = NackInputs {
            missing: &missing,
            state: &state,
            tail: &tail,
            util: &util,
            down_util: n.down_util,
            gap_timeout_ms: fs.gap_ms(0),
        };
        if let Some(nk) = n.t.poll(now_ms, &inp) {
            let mut ctx = sctx;
            ctx.seq32 = nk.counter;
            let sf = SlotFrame {
                frame: rc::pack_nack(&nk, &g.key, &ctx),
                offered_ms: now_ms,
                ..SlotFrame::default()
            };
            n.sent += 1;
            g.send(&sf, *now_us, io);
        }
    }

    /// Once a second: the tracker's window into the stats readout.
    fn drain_nack_window(&mut self, now_ms: u64) {
        let Some(n) = self.gs.as_mut().and_then(|g| g.nack.as_mut()) else {
            return;
        };
        if n.win_t0_ms == 0 {
            n.win_t0_ms = now_ms;
            return;
        }
        if now_ms < n.win_t0_ms + 1000 {
            return;
        }
        let mut w = n.t.take_window();
        let secs = (now_ms - n.win_t0_ms) as f64 / 1000.0;
        n.win_t0_ms = now_ms;
        n.fill_pps = w.filled as f64 / secs;
        n.late_max = w.late_ms_max;
        w.fill_ms.sort_unstable();
        let pct = |p: f64| -> Option<u32> {
            if w.fill_ms.is_empty() {
                return None;
            }
            let rank = (p * w.fill_ms.len() as f64).ceil();
            let i = if rank < 1.0 { 0 } else { rank as usize - 1 };
            Some(w.fill_ms[i.min(w.fill_ms.len() - 1)])
        };
        n.fill_p50 = pct(0.5);
        n.fill_p90 = pct(0.9);
        n.fill_max = w.fill_ms.last().copied();
    }

    /// Replay only: a DISC_ACK from a drone that does not exist, so the
    /// session (and the frame-wire gate) opens on a capture with no drone.
    pub fn inject_disc_ack_for_replay(&mut self, now_us: u64) {
        let Some(g) = self.gs.as_mut() else { return };
        let ack = rc::DiscAck {
            vrx_nonce: g.vrx.rz_nonce(),
            vtx_nonce: 1,
            chip_caps: rc::CAP_FRAME_WIRE,
            agreed_channel: g.vrx.proposal(),
            seq: 1,
            ..rc::DiscAck::default()
        };
        let wire = rc::pack_disc_ack(&ack);
        g.vrx.on_rc_frame(&wire, now_us as f64 / 1000.0);
    }

    pub fn resets(&self) -> u64 {
        self.c.resets
    }
    pub fn frame_stream(&self) -> &FrameStream {
        &self.fs
    }
    pub fn aggregator(&self) -> &Aggregator {
        &self.agg
    }
    pub fn frame_gap_timeout_ms(&self) -> u64 {
        self.fs.gap_ms(0)
    }
}

impl GsCore for RustCore {
    fn on_rx(&mut self, m: &RxBody, io: &mut dyn CoreIo) {
        self.now_us = self.now_us.max(m.mono_us);
        self.c.bodies += 1;
        match self.agg.on_rx_body(m) {
            Route::Rc => self.on_rc(&m.body, m.mono_us, io),
            Route::Probe => {
                if let Some(g) = self.gs.as_mut() {
                    g.slot.on_probe_tail(self.now_us / 1000);
                    g.lha.on_probe_body(m.card_id, m);
                }
            }
            Route::Video(frags) if self.frame_wire => {
                let now_ms = self.now_us / 1000;
                let mut out = au_out!(self, io);
                for f in frags {
                    let arr = FragArrival {
                        body_mono_us: f.body_mono_us,
                        q_ms: f.q_ms,
                        enc_us: f.enc_us,
                        air_ms: f.air_ms,
                        sw_seq: Some(f.sw_seq),
                        retx: f.retx,
                        ..FragArrival::default()
                    };
                    self.fs
                        .push_fragment(f.stream_id, &f.frag, now_ms, &arr, &mut out);
                }
            }
            _ => {}
        }
        if !m.crc_ok {
            return;
        }
        let sid = sbi::peek_stream_id(&m.body);
        // MCS readout off the air: base-stream bodies only.
        if sid == Some(0) && m.mcs != 255 {
            self.air_mcs = i32::from(m.mcs);
        }
        let is_video = rc::frame_type(&m.body).is_none()
            && sid != Some(sbi::MSP_STREAM_ID)
            && sid != Some(sbi::PROBE_STREAM_ID);
        if let (true, Some(g)) = (is_video, self.gs.as_mut()) {
            // Only real video refreshes the rendezvous silence timer.
            g.vrx.on_video(m.mono_us as f64 / 1000.0);
        }
    }

    fn tick(&mut self, now_us: u64, io: &mut dyn CoreIo) {
        self.now_us = self.now_us.max(now_us);
        let now_ms = self.now_us / 1000;
        if self.opts.adaptive_gap && now_ms >= self.gap_update_ms + 1000 {
            self.gap_update_ms = now_ms;
            for s in 0..2 {
                let d = self.agg.decoder();
                self.gap
                    .update(s, d.newest_seq(s), d.repair_window(s), now_ms);
                self.fs.set_gap_timeout(s, self.gap.timeout_ms(s));
            }
        }
        if let Some(g) = self.gs.as_ref() {
            // Video tail only in SESSION with a CAP_FRAME_WIRE peer; any
            // change drops FRAG-seq continuity and half-assembled frames.
            let fw = g.vrx.link_state() == VrxState::Session
                && g.vrx.peer_caps() & rc::CAP_FRAME_WIRE != 0;
            if fw != self.frame_wire {
                self.frame_wire = fw;
                self.reset_video(io);
            }
        }
        if self.frame_wire {
            self.fs.poll(now_ms, &mut au_out!(self, io));
        }
        if self.gs.as_ref().is_some_and(|g| g.nack.is_some()) {
            self.poll_nack(now_ms, io);
            self.drain_nack_window(now_ms);
        }
        let now_us = self.now_us;
        let Some(g) = self.gs.as_mut() else { return };
        // Tag outbound MAVLink under the live session only.
        let linked = g.vrx.link_state() == VrxState::Session;
        g.mav.set_session(if linked {
            g.vrx.session_ctx()
        } else {
            rc::TagCtx::default()
        });
        // ParamClient re-lists on every session edge and on a vtx_nonce
        // change while linked; before flush() so the request leaves this tick.
        let nonce = g.vrx.session_ctx().vtx_nonce;
        if linked != g.param_linked || (linked && nonce != g.param_vtx_seen) {
            if g.param_linked {
                g.params.set_linked(false, now_ms);
            }
            if linked {
                g.params.set_linked(true, now_ms);
            }
            g.param_linked = linked;
            g.param_vtx_seen = nonce;
        }
        g.params.tick(now_ms);
        g.drain_params();
        let frames = g.mav.flush();
        g.send_mav_frames(frames, now_us, io);

        let inp = LinkHealthInputs {
            op: *g.vrx.cur_op(),
            probe_profile: g.vrx.probe_profile(),
            probe_rung: g.vrx.ctl().probe_rung(),
        };
        let lh = g.lha.tick(now_ms as f64, &mut self.agg, &inp);
        if let Some(t) = lh.probe_tail_ms {
            g.slot.set_probe_tail_ms(t);
        }
        if let Some(out) = g.vrx.step(now_ms as f64, &lh.health) {
            if !out.is_disc {
                g.lha.on_step_sent(&self.agg);
            }
            // rcf_seq() IS this frame's seq; DISCs are not matchable RTT sends.
            let mut sf = SlotFrame {
                frame: out.frame,
                seq: g.vrx.rcf_seq(),
                card: 0,
                stamp_rtt: !out.is_disc,
                ..SlotFrame::default()
            };
            if !g.slot.offer(&mut sf, now_ms, false) {
                g.send(&sf, now_us, io);
            }
        }
        for f in g.slot.take_due(now_ms) {
            g.send(&f, now_us, io);
        }
        g.last_health = lh.health;
    }

    fn mav_push_outbound(&mut self, bytes: &[u8]) {
        if let Some(g) = self.gs.as_mut() {
            g.mav.push_outbound(bytes);
        }
    }

    fn stats_json(&self) -> String {
        let card = self.agg.cards().first();
        let num = |v: Option<f64>| v.map_or("null".to_string(), |x| format!("{x:.1}"));
        let rssi = card.filter(|c| c.rf.has_ema).map(|c| c.rf.rssi_ema - 110.0);
        let snr = card.filter(|c| c.rf.has_ema).map(|c| c.rf.snr_ema);
        let fs = self.fs.stats();
        let g = self.gs.as_ref();
        let session = g.is_some_and(|g| g.vrx.link_state() == VrxState::Session);
        let rung = g.map_or(-1, |g| g.vrx.ctl().rung());
        let mcs = g.map_or(self.air_mcs, |g| g.vrx.cur_op().mcs);
        format!(
            concat!(
                "{{\"mode\":\"{}\",\"session\":{},\"rung\":{},\"mcs\":{},\"bw\":{},",
                "\"key_fp\":\"{}\",\"drone_state\":{},\"snr_db\":{},\"rssi_dbm\":{},",
                "\"bodies\":{},\"aus\":{},\"trunc\":{},\"trunc_base\":{},\"salvaged\":{},",
                "\"sends\":{},\"resets\":{},\"fs_dropped\":{},\"fs_stall_resets\":{},\"slice_fb\":{:?}}}"
            ),
            if self.mode == Mode::Gs { "gs" } else { "spotter" },
            session,
            rung,
            mcs,
            self.bw,
            self.key_fp,
            self.telem
                .as_ref()
                .map_or("null".to_string(), |t| t.state.to_string()),
            num(snr),
            num(rssi),
            self.c.bodies,
            self.c.aus_complete,
            self.c.aus_truncated,
            self.c.aus_truncated_base,
            self.c.aus_salvaged,
            g.map_or(0, |g| g.sends),
            self.c.resets,
            fs.dropped,
            fs.stall_resets,
            fs.slice_fallback,
        )
    }
}
