//! The Rust receive core, wired as mabur's WebGs wires it (`web/src/web_gs.cpp`):
//! Aggregator -> UepDecoder -> FrameStream -> Au, and in GS mode the control
//! side: rendezvous, ladder, RCF slotting, NACK, the MAVLink tunnel and the
//! parameter client. With cards (`with_cards`, live) the GS also runs the
//! channel core (search, boot pick, relocation, in-flight hops) and sends
//! through the cards; without them (replay, the parity gates) it is a page
//! with no roster and sends through `CoreIo`.
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::Arc;

use crate::aggregator::{Aggregator, Route};
use crate::cal::log::CalLog;
use crate::cal::picker::{CalPicker, FixedPicker, Mode as PickMode};
use crate::cal::plan::{default_sweeps, effective_width};
use crate::cal::session::{sweep_token, CalSession, CalSessionCfg};
use crate::cal::status_json as cal_status_json;
use crate::chan::core::{
    ChannelCore, ChannelCoreCfg, ChannelSink, ChannelTickIn, MsClock, Sleeper, UsClock,
};
use crate::chan::gate::cal_cmd_clear;
use crate::chan::hop::HopEvent;
use crate::chan::plan::MoveEvent;
use crate::chan::radio::LinkCard;
use crate::chan::ranker::RankEntry;
use crate::chan::scout::ScoutDwell;
use crate::chan::verdict::{VerdictCardIn, VerdictLinkIn, VerdictOut};
use crate::config::Config;
use crate::frame_stream::{AuLat, FragArrival, FrameOut, FrameStream, FrameStreamCfg};
use crate::gap::{DroneRestartDetector, GapTimeoutPolicy};
use crate::json::J;
use crate::ladder::LinkHealth;
use crate::link_health::{LinkHealthAssembler, LinkHealthInputs, OpPoint};
use crate::mav::{MavTunnel, ParamClient};
use crate::nack::{NackInputs, NackTailView, NackTracker};
use crate::tx_selector::{effective_tx_card, CardSnapshot, TxSelector, TxSelectorCfg};
use crate::uplink::{RttEstimator, UplinkDelivery, UplinkState};
use crate::vrx::{
    random_nonce, RcfSlotCfg, RcfSlotter, SlotFrame, VrxCfg, VrxController, VrxState,
};
use crate::{Au, CoreIo, GsCore, Mode, RxBody};
use pal_wire::cal::{self, SweepSpec};
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
    cal_frames: u64,
}

/// The drone's RadioTx sends calibration frames as QoS-Data (24-byte header
/// plus 2 bytes QoS); the RX path strips it, so a capacity sweep's air bits
/// add it and the FCS back.
const CAL_DOT11_HEADER_LEN: usize = 26;
const CAL_FCS_LEN: usize = 4;
/// maburgs' MAVLink identity for the calibration kit.
const CAL_SYSID: u8 = 255;
const CAL_COMPID: u8 = 190;

/// FrameStream's sink: builds the AU in flight and hands it to the host.
struct AuOut<'a> {
    cur: &'a mut Au,
    io: &'a mut dyn CoreIo,
    c: &'a mut Counters,
    now_us: u64,
    lha: &'a mut LinkHealthAssembler,
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
        let now_ms = self.now_us / 1000;
        if let Some(g) = self.gs.as_deref_mut() {
            g.slot.on_au_first(now_ms);
        }
        // One probe expectation per video AU, both modes.
        self.lha.on_au_begin(sid, h.frame_id, now_ms as f64);
    }
    fn data(&mut self, b: &[u8]) {
        self.cur.data.extend_from_slice(b);
    }
    fn end(&mut self, complete: bool, lat: &AuLat) {
        // Every AU end, clean or truncated (in-flight scout alignment).
        if let Some(c) = self.gs.as_deref().and_then(|g| g.chan.as_ref()) {
            c.note_au_end();
        }
        self.cur.complete = complete;
        self.cur.salvaged = !complete && lat.slice.salvaged;
        self.cur.skipped_before = lat.skipped_before;
        self.cur.vcl_nal = lat.vcl_nal;
        if let Some(g) = self.gs.as_deref_mut() {
            let probe_follows =
                self.cur.sid < 2 && self.lha.probe_commanded() != rc::NO_PROBE_PROFILE;
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
    fill_pps: Option<f64>,
    late_max: u32,
    fill_p50: Option<u32>,
    fill_p90: Option<u32>,
    fill_max: Option<u32>,
}

/// Everything only GS mode owns.
struct Gs {
    vrx: VrxController,
    slot: RcfSlotter,
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
    /// Live cards; empty = send through `CoreIo`.
    cards: Vec<Arc<dyn LinkCard>>,
    chan: Option<ChannelCore>,
    /// TX-power calibration (maburgs' CalSession); idle unless started.
    cal: CalSession,
    /// The GS's own radio.width: the widest sweep a run may ask for.
    radio_width: u8,
    ladder: Vec<crate::config::Rung>,
    /// The linked drone's DiscAck agreed_width (0 = none heard yet).
    peer_width: u8,
    /// MAVLink packet seq of the cal packets.
    cal_mav_seq: u8,
    /// The uplink TX card among `cards` (card 0 without a roster).
    sel: TxSelector,
}

impl Gs {
    /// WebGs::send_: false = the scout gate dropped it (counted by the core),
    /// or a calibration sweep owns the air (anything sent would corrupt
    /// the measurement; the slotter's backlog must not go out either).
    fn send(&mut self, f: &SlotFrame, now_us: u64, io: &mut dyn CoreIo) -> bool {
        if self.cal.radio_silent(now_us / 1000) {
            return false;
        }
        self.send_now(f, now_us, io)
    }

    /// The card this RCF step transmits on (maburgs main.cpp at vrx.step).
    /// A card the scout owns is off searching or measuring and never a
    /// candidate; while a dwell or a hop is in flight the last pick holds
    /// (`tx_selection_frozen`). Without a card roster: card 0.
    fn pick_tx(&mut self, agg: &Aggregator, now_ms: u64) -> usize {
        let Some(c) = self.chan.as_ref() else {
            return self.sel.selected();
        };
        if !c.tx_frozen() {
            let snaps: Vec<CardSnapshot> = self
                .cards
                .iter()
                .enumerate()
                .map(|(i, card)| {
                    let t = &agg.cards()[i];
                    CardSnapshot {
                        alive: !c.scout_owns_card(i) && card.alive(),
                        rssi_ema: t.rf.rssi_ema,
                        last_frame_us: t.last_frame_us,
                    }
                })
                .collect();
            self.sel.update(&snaps, now_ms * 1000);
        }
        c.note_tx_card(self.sel.selected());
        self.sel.selected()
    }

    /// Past the calibration gate: the cal lane's own frames, which the
    /// session offers only while the air is the GS's.
    fn send_now(&mut self, f: &SlotFrame, now_us: u64, io: &mut dyn CoreIo) -> bool {
        if self
            .chan
            .as_ref()
            .is_some_and(|c| !c.may_send(f.card as usize))
        {
            return false;
        }
        let ok = match self.cards.get(f.card as usize) {
            Some(c) => c.send_control(&f.frame),
            None => {
                io.send(&f.frame);
                true
            }
        };
        self.sends += 1;
        if f.stamp_rtt {
            self.rcf_sent += 1;
            self.rtt.on_rcf_sent(f.seq, now_us);
            self.uplink.on_rcf_sent(now_us / 1000);
        }
        if let Some(c) = self.chan.as_mut() {
            c.note_sent(ok, f.stamp_rtt);
        }
        true
    }

    /// The tunnel's send callback: slotted like an RCF, direct when the
    /// slotter holds nothing.
    fn send_mav_frames(&mut self, frames: Vec<Vec<u8>>, now_us: u64, io: &mut dyn CoreIo) {
        for frame in frames {
            let mut sf = SlotFrame {
                frame,
                card: self.sel.selected() as i32,
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
    key_fp: String,
    cur: Au,
    c: Counters,
    gs: Option<Gs>,
    lha: LinkHealthAssembler,
    last_health: LinkHealth,
    /// A spotter drives no ladder: its link setting is the configured width.
    spotter_op: OpPoint,
    telem_rx_us: u64,
    channel: u8,
    idr_req: u32,
    /// The channel core's remembered-channel store (0 = none yet).
    stored_ch: Arc<AtomicU8>,
}

/// What `with_cards` needs beyond the config: the cards and the channel
/// core's clock (real time when threaded; the tick clock otherwise).
pub struct ChanEnv {
    pub cards: Vec<Arc<dyn LinkCard>>,
    pub now_ms: MsClock,
    pub now_us: UsClock,
    pub sleep: Sleeper,
    /// WebGs Opts::core_threads: the scout and in-flight threads.
    pub threaded: bool,
}

/// WebGs's ChannelSink: the stderr lines go out, the scan.log records
/// have no home here.
struct LogSink;

impl ChannelSink for LogSink {
    fn dwell(&mut self, _: f64, _: usize, _: &ScoutDwell) {}
    fn pick(&mut self, _: f64, _: Option<u8>, _: u64, _: &[RankEntry], _: i32) {}
    fn moved(&mut self, _: &MoveEvent) {}
    fn verdict(&mut self, _: f64, _: &VerdictOut, _: &[VerdictCardIn], _: &VerdictLinkIn) {}
    fn hop(&mut self, _: &HopEvent) {}
    fn log(&mut self, line: &str) {
        eprintln!("{line}");
    }
}

macro_rules! au_out {
    ($s:expr, $io:expr) => {
        AuOut {
            cur: &mut $s.cur,
            io: $io,
            c: &mut $s.c,
            now_us: $s.now_us,
            lha: &mut $s.lha,
            gs: $s.gs.as_mut(),
        }
    };
}

impl RustCore {
    pub fn new(cfg: &Config, opts: RustOpts) -> Result<Self, String> {
        Self::build(cfg, opts, None)
    }

    /// A GS that drives `env.cards` (WebGs with a roster): `opts.channel`
    /// is the start channel (the card's), Mode::Gs.
    pub fn with_cards(cfg: &Config, opts: RustOpts, env: ChanEnv) -> Result<Self, String> {
        if opts.mode != Mode::Gs {
            return Err("cards need Mode::Gs".into());
        }
        if env.cards.is_empty() {
            return Err("with_cards needs a card".into());
        }
        Self::build(cfg, opts, Some(env))
    }

    fn build(cfg: &Config, opts: RustOpts, env: Option<ChanEnv>) -> Result<Self, String> {
        let layers = cfg.uep_layers();
        // maburgs sizes both by its card roster; with none (replay, the
        // parity gates) it is WebGs's one card.
        let n_cards = env.as_ref().map_or(1, |e| e.cards.len().max(1));
        let ch = if opts.channel != 0 {
            opts.channel
        } else {
            cfg.radio.channels.first().copied().unwrap_or(0)
        };
        let gs = (opts.mode == Mode::Gs).then(|| {
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
                    fill_pps: None,
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
                cards: Vec::new(),
                chan: None,
                // maburgs' defaults: margin 1 dB only predicts the verify
                // parks, and verify tallies by rate whatever the drone's
                // own margin parks at.
                cal: CalSession::new(CalSessionCfg::default(), CalLog::none()),
                radio_width: cfg.radio.width,
                ladder: cfg.link.ladder_cfg.ladder.clone(),
                peer_width: 0,
                cal_mav_seq: 0,
                sel: TxSelector::new(
                    TxSelectorCfg {
                        pinned: effective_tx_card(cfg.radio.tx_card, n_cards),
                        ..TxSelectorCfg::default()
                    },
                    n_cards,
                ),
            }
        });
        let stored_ch = Arc::new(AtomicU8::new(0));
        let mut gs = gs;
        if let (Some(g), Some(env)) = (gs.as_mut(), env) {
            let cc = ChannelCoreCfg {
                radio: cfg.radio.clone(),
                hop: cfg.hop.clone(),
                key: cfg.link.key,
                start_ch: ch,
                // As maburgs (bench row 7, docs/channel-select.md).
                leak_per_frame: 1.0,
                threaded: env.threaded,
                store_name: "CHANNEL line".into(),
            };
            let st = stored_ch.clone();
            g.chan = Some(ChannelCore::new(
                cc,
                env.cards.clone(),
                Box::new(LogSink),
                Box::new(move |c| {
                    st.store(c, Ordering::Relaxed);
                    true
                }),
                env.now_ms,
                env.now_us,
                env.sleep,
            ));
            g.cards = env.cards;
        }
        Ok(Self {
            mode: opts.mode,
            opts,
            now_us: 0,
            agg: Aggregator::new(
                &layers,
                cfg.fec.seq_horizon,
                n_cards,
                cfg.link.arrival_guard_syms,
            ),
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
            key_fp: rc::key_fingerprint(&cfg.link.key),
            cur: Au::default(),
            c: Counters::default(),
            gs,
            // Probe bodies ride the enh layer's geometry; blocks_per_body
            // is 4 on decode (config.cpp).
            lha: LinkHealthAssembler::new(n_cards, 4, sw::HEADER_LEN + layers[1].symbol_size),
            last_health: LinkHealth::empty(),
            spotter_op: OpPoint {
                bw: i32::from(cfg.radio.width),
                ..OpPoint::default()
            },
            telem_rx_us: 0,
            channel: ch,
            idr_req: 0,
            stored_ch,
        })
    }

    /// The channel the channel core last asked to remember.
    pub fn stored_channel(&self) -> Option<u8> {
        Some(self.stored_ch.load(Ordering::Relaxed)).filter(|&c| c != 0)
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
                // The drone replies whatever the session's state; it
                // ignores what does not fit.
                for m in cal::decode_all(&pkt) {
                    g.cal.on_mav(&m, us / 1000);
                }
            }
            g.drain_params();
            return;
        }
        if ft == Some(rc::T_TELEM) {
            if let Some(t) = rc::parse_telem(body) {
                let (seq, state, flags) = (t.tlm_seq, t.state, t.flags);
                self.telem_rx_us = us;
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
            let was_session = g.vrx.link_state() == VrxState::Session;
            g.vrx.on_rc_frame(body, us as f64 / 1000.0);
            if ft == Some(rc::T_DISC_ACK) {
                if let Some(ack) = rc::parse_disc_ack(body) {
                    g.peer_width = ack.agreed_width;
                }
            }
            // The ack that OPENED the session links where it was heard
            // (on_rc_body handed the core its rx channel).
            if let Some(c) = g.chan.as_mut() {
                if !was_session
                    && g.vrx.link_state() == VrxState::Session
                    && ft == Some(rc::T_DISC_ACK)
                {
                    if let Some(ack) = rc::parse_disc_ack(body) {
                        c.on_session_opened(&ack, us as f64 / 1000.0, &mut g.vrx);
                    }
                }
            }
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
                card: g.sel.selected() as i32,
                ..SlotFrame::default()
            };
            if g.send(&sf, *now_us, io) {
                if let Some(n) = g.nack.as_mut() {
                    n.sent += 1;
                }
            }
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
        n.fill_pps = Some(w.filled as f64 / secs);
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
        // A calibration sweep frame is its own body type: it must never
        // reach the decoder as garbage video. CRC-bad ones too (tallied as
        // corrupt); one whose corruption breaks the stamp fails the parse
        // and falls through as ordinary traffic.
        if let Some(g) = self.gs.as_mut() {
            if let Some(info) = cal::parse_payload(&m.body) {
                let rssi = i32::from(m.rssi[0].max(m.rssi[1])) - 110;
                g.cal.on_cal_frame(
                    usize::from(m.card_id),
                    &info,
                    rssi,
                    m.crc_ok,
                    CAL_DOT11_HEADER_LEN + info.len + CAL_FCS_LEN,
                    m.mono_us / 1000,
                );
                self.c.cal_frames += 1;
                return;
            }
        }
        // Before the aggregator routes it: the rc path reads it.
        if let Some(c) = self.gs.as_mut().and_then(|g| g.chan.as_mut()) {
            c.on_rc_body(m.rx_channel);
        }
        match self.agg.on_rx_body(m) {
            Route::Rc => self.on_rc(&m.body, m.mono_us, io),
            Route::Probe => {
                if let Some(g) = self.gs.as_mut() {
                    g.slot.on_probe_tail(self.now_us / 1000);
                }
                self.lha.on_probe_body(m.card_id, m);
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
            // Only real video heard where the link lives refreshes the
            // rendezvous silence timer (ChannelPlan::is_link_video).
            if g.chan
                .as_ref()
                .is_none_or(|c| c.is_link_video(m.rx_channel))
            {
                g.vrx.on_video(m.mono_us as f64 / 1000.0);
            }
            // Hop confirmation reads the stamp.
            if let Some(c) = g.chan.as_mut() {
                c.note_video(m.rx_channel);
            }
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
        if let Some(g) = self.gs.as_mut() {
            if let Some(c) = g.chan.as_mut() {
                // maburgs main.cpp's per-tick block, on the TX selector's
                // card as it stood after the last RCF step.
                let tx = g.sel.selected();
                let cin = ChannelTickIn {
                    now_ms: now_ms as f64,
                    in_session: g.vrx.link_state() == VrxState::Session,
                    cal_running: g.cal.running(),
                    tx_card: tx,
                    agg: Some(&self.agg),
                };
                c.tick(&cin, &mut g.vrx);
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
        if let Some(g) = self.gs.as_mut() {
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
            // Calibration first, so a run's message leaves this tick, ahead
            // of any live MAVLink backlog. The one hold: a SweepCmd while a
            // scout dwell begun just before the start is still off-channel
            // (repeated every 200 ms, so only late).
            let caps = g.vrx.peer_caps();
            g.cal.set_peer(
                linked,
                linked && caps & rc::CAP_CALIBRATE != 0,
                g.vrx.session_ctx(),
            );
            if let Some(c) = g.chan.as_mut() {
                g.cal.step_pick(now_ms, c);
            }
            if let Some(m) = g.cal.due_mav(now_ms) {
                let dwell_busy = g.chan.as_ref().is_some_and(|c| c.dwell_busy());
                let held = matches!(m, cal::CalMsg::Sweep(_))
                    && !cal_cmd_clear(g.cal.radio_silent(now_ms), dwell_busy);
                if !held {
                    let pkt = cal::encode(&m, CAL_SYSID, CAL_COMPID, g.cal_mav_seq);
                    g.cal_mav_seq = g.cal_mav_seq.wrapping_add(1);
                    g.mav.push_cal(pkt, g.cal.tag_ctx());
                }
            }
            for frame in g.mav.flush_cal() {
                let sf = SlotFrame {
                    frame,
                    offered_ms: now_ms,
                    card: g.sel.selected() as i32,
                    ..SlotFrame::default()
                };
                g.send_now(&sf, now_us, io);
            }
            let frames = g.mav.flush();
            g.send_mav_frames(frames, now_us, io);
        }

        let inp = match self.gs.as_ref() {
            Some(g) => LinkHealthInputs {
                op: *g.vrx.cur_op(),
                probe_profile: g.vrx.probe_profile(),
                probe_rung: g.vrx.ctl().probe_rung(),
            },
            None => LinkHealthInputs {
                op: self.spotter_op,
                probe_profile: rc::NO_PROBE_PROFILE,
                probe_rung: -1,
            },
        };
        let lh = self.lha.tick(now_ms as f64, &mut self.agg, &inp);
        if let Some(g) = self.gs.as_mut() {
            if let Some(t) = lh.probe_tail_ms {
                g.slot.set_probe_tail_ms(t);
            }
            if let Some(out) = g.vrx.step(now_ms as f64, &lh.health) {
                if !out.is_disc {
                    self.lha.on_step_sent(&self.agg);
                }
                let tx = g.pick_tx(&self.agg, now_ms);
                // A DISC follows the scout (the core's disc targets); an RCF
                // goes to the TX selector's card. rcf_seq() IS this frame's
                // seq; DISCs are not matchable RTT sends.
                let targets = match (&g.chan, out.is_disc) {
                    (Some(c), true) => c.disc_targets(tx),
                    _ => vec![tx],
                };
                for t in targets {
                    let frame = match (&g.chan, out.is_disc) {
                        // A DISC proposes the channel it is sent on.
                        (Some(c), true) => c.disc_for_card(&out.frame, t),
                        _ => out.frame.clone(),
                    };
                    let mut sf = SlotFrame {
                        frame,
                        seq: g.vrx.rcf_seq(),
                        card: t as i32,
                        stamp_rtt: !out.is_disc,
                        ..SlotFrame::default()
                    };
                    if !g.slot.offer(&mut sf, now_ms, false) {
                        g.send(&sf, now_us, io);
                    }
                }
            }
            for f in g.slot.take_due(now_ms) {
                g.send(&f, now_us, io);
            }
        }
        self.last_health = lh.health;
    }

    fn set_idr_requests(&mut self, n: u32) {
        self.idr_req = n;
        if let Some(g) = self.gs.as_mut() {
            g.vrx.set_idr_epoch(n as u8);
        }
    }

    fn ladder_json(&self) -> Option<String> {
        let g = self.gs.as_ref()?;
        let c = g.vrx.ctl();
        let now = (self.now_us / 1000) as f64;
        let mut j = J::obj();
        j.set("rung", c.rung());
        j.set("top", c.ladder().len() as i32 - 1);
        if let Some(u) = c.up_status(now) {
            j.set("u", u.u);
            j.set("up_util", u.up_util);
            j.set("clean_ms", u.clean_ms.round() as i64);
            j.set("clean_need_ms", u.clean_need_ms.round() as i64);
            j.set("probe", u.probe.state.as_str());
            j.set("probe_rung", u.probe.rung);
            j.set("probe_u", u.probe.u);
            j.set("probe_max_util", u.probe_max_util);
            j.set("streak", u.probe.streak_bodies);
            j.set("streak_need", u.probe_need);
            j.set("penalty_ms", u.next_penalty_ms);
        }
        let e = c.last_event();
        if e.t_ms > 0.0 {
            j.set("last_reason", e.reason.as_str());
            j.set("last_from", e.from);
            j.set("last_to", e.to);
            j.set("last_age_ms", (now - e.t_ms).max(0.0).round() as i64);
        }
        Some(j.dump())
    }

    fn params_json(&self, now_us: u64) -> String {
        params_json(self.gs.as_ref().map(|g| &g.params), now_us / 1000)
    }

    fn param_set(&mut self, row: usize, text: &str) -> Result<(), String> {
        let now_ms = self.now_us / 1000;
        let Some(g) = self.gs.as_mut() else {
            return Err("not linked".into());
        };
        let r = g.params.set(row, text, now_ms);
        // WebGs's client pushes onto the tunnel as it sends.
        g.drain_params();
        r
    }

    fn param_refresh(&mut self) {
        let now_ms = self.now_us / 1000;
        if let Some(g) = self.gs.as_mut() {
            g.params.refresh(now_ms);
            g.drain_params();
        }
    }

    fn mav_push_outbound(&mut self, bytes: &[u8]) {
        if let Some(g) = self.gs.as_mut() {
            g.mav.push_outbound(bytes);
        }
    }

    fn cal_start(&mut self, sweeps: &[SweepSpec]) -> Result<(), String> {
        let now_ms = self.now_us / 1000;
        let Some(g) = self.gs.as_mut() else {
            return Err("refused: a spotter does not transmit".into());
        };
        let linked = g.vrx.link_state() == VrxState::Session;
        let width = effective_width(g.radio_width, linked, g.peer_width);
        let sweeps = if sweeps.is_empty() {
            default_sweeps(width, &g.ladder)
        } else {
            if sweeps.iter().any(|s| s.bw > width) {
                return Err(format!("refused: the link flies {width} MHz at most"));
            }
            sweeps.to_vec()
        };
        // Refusals first, from the session's own last view of the peer.
        let caps = g.vrx.peer_caps();
        g.cal.set_peer(
            linked,
            linked && caps & rc::CAP_CALIBRATE != 0,
            g.vrx.session_ctx(),
        );
        // The channel core picks the channel first when it can measure (auto,
        // a spare card); without one the run sweeps on the configured channel.
        let mut fixed = FixedPicker {
            mode: if g.cards.len() >= 2 {
                PickMode::Pinned
            } else {
                PickMode::OneCard
            },
            op: self.channel,
        };
        let picker: &mut dyn CalPicker = match g.chan.as_mut() {
            Some(c) => c,
            None => &mut fixed,
        };
        g.cal.start(random_nonce(), sweeps, now_ms, Some(picker))?;
        let snap = g.cal.snapshot();
        eprintln!(
            "pal cal: started ch {} ({}) sweeps {:?}",
            snap.channel,
            snap.pick_note,
            g.cal.sweeps().iter().map(sweep_token).collect::<Vec<_>>()
        );
        Ok(())
    }

    fn set_channels(&mut self, chosen: &[u8]) -> Result<(), String> {
        match self.gs.as_mut().and_then(|g| g.chan.as_mut()) {
            Some(c) => c.set_usable(chosen),
            None => Err("no channel core (a spotter or a card-less replay)".into()),
        }
    }

    fn chan_scan(&mut self, set: &[u8]) -> Result<(), String> {
        match self.gs.as_mut().and_then(|g| g.chan.as_mut()) {
            Some(c) => {
                c.begin_op_scan(set);
                Ok(())
            }
            None => Err("no channel core (a spotter or a card-less replay)".into()),
        }
    }

    fn chan_scan_json(&self) -> Option<String> {
        self.gs.as_ref()?.chan.as_ref()?.op_scan_json()
    }

    fn cal_abort(&mut self) {
        if let Some(g) = self.gs.as_mut() {
            if g.cal.running() || g.cal.picking() {
                eprintln!("pal cal: aborted by the operator");
            }
            let picker = g.chan.as_mut().map(|c| c as &mut dyn CalPicker);
            g.cal.abort("aborted by the operator", picker);
        }
    }

    fn cal_json(&self) -> Option<String> {
        let g = self.gs.as_ref()?;
        let capable =
            g.vrx.link_state() == VrxState::Session && g.vrx.peer_caps() & rc::CAP_CALIBRATE != 0;
        Some(cal_status_json(&g.cal.snapshot(), g.cal.sweeps(), capable))
    }

    fn set_cal_log_dir(&mut self, dir: &std::path::Path) {
        if let Some(g) = self.gs.as_mut() {
            g.cal.set_log(CalLog::open(dir));
        }
    }

    /// WebGs::stats() + webgs::stats_json, key for key (no spotter follow,
    /// no MSP sink: those fields are null or 0).
    fn stats_json(&self) -> String {
        let g = self.gs.as_ref();
        let mut j = J::obj();
        j.set(
            "mode",
            if self.mode == Mode::Gs {
                "gs"
            } else {
                "spotter"
            },
        );
        j.set(
            "session",
            g.is_some_and(|g| g.vrx.link_state() == VrxState::Session),
        );
        j.set("peer_acked", g.is_some_and(|g| g.vrx.peer_acked()));
        j.set("rung", g.map_or(-1, |g| g.vrx.ctl().rung()));
        j.set("mcs", g.map_or(self.air_mcs, |g| g.vrx.cur_op().mcs));
        j.set("bw", g.map_or(self.spotter_op.bw, |g| g.vrx.cur_op().bw));
        j.set(
            "probe",
            g.map_or("", |g| g.vrx.ctl().probe_gate().state.as_str()),
        );
        j.set("key_mismatch", g.is_some_and(|g| g.vrx.key_mismatch()));
        j.set("key_fp", self.key_fp.as_str());
        let channel = g
            .and_then(|g| g.cards.first())
            .map_or(self.channel, |c| c.channel());
        j.set("channel", i64::from(channel));
        match g.and_then(|g| g.chan.as_ref()) {
            Some(c) => {
                let cs = c.snapshot(0);
                j.set("scan_state", cs.scan_state);
                j.set("scan_rounds", cs.scan_rounds);
                j.set("scan_pick", cs.scan_pick.map(i64::from));
                let hp = &cs.hop;
                let mut h = J::obj();
                h.set("verdict", hp.verdict);
                h.set("evidence", i64::from(hp.evidence));
                h.set("ref_rung", hp.ref_rung);
                h.set("epoch", i64::from(hp.epoch));
                h.set("state", hp.state);
                h.set("target", hp.target.map(i64::from));
                h.set("hops", hp.hops);
                h.set("holds", hp.holds);
                h.set("last_ms", hp.last_ms);
                // Relay sweeps only; Palianytsia has no relays.
                h.set("sweep_timeouts", 0u64);
                j.set("hop", h);
            }
            None => {
                for k in ["scan_state", "scan_rounds", "scan_pick", "hop"] {
                    j.set(k, J::Null);
                }
            }
        }
        for k in ["follow_state", "follows"] {
            j.set(k, J::Null);
        }
        let pre = self.lha.pre_all;
        j.set("pre_fec_loss", pre.valid.then_some(pre.loss));
        j.set("residual", self.lha.residual);
        // Per-card receive counters, only with two or more cards: a one-card
        // line stays byte-equal to WebGs's (the m1 parity gate).
        if self.agg.cards().len() > 1 {
            let cards = self.agg.cards().iter().map(|c| {
                let mut o = J::obj();
                o.set("frames", c.frames);
                o.set("video", c.video_bodies);
                o.set("crc_fail", c.crc_fail);
                // dBm, as LinkHealth reads the same EMA (raw RSSI - 110).
                o.set(
                    "rssi",
                    c.rf.has_ema.then(|| (c.rf.rssi_ema - 110.0).round() as i64),
                );
                o
            });
            j.set("cards", J::A(cards.collect()));
            if let Some(g) = g {
                j.set("tx_card", g.sel.selected() as u64);
                j.set("tx_switches", g.sel.switches());
            }
        }
        let rtt = g.map(|g| &g.rtt);
        j.set("rtt_ms", rtt.filter(|r| r.has_rtt()).map(|r| r.rtt_ms()));
        j.set(
            "rtt_min_ms",
            rtt.filter(|r| r.has_rtt()).map(|r| r.rtt_min_ms()),
        );
        j.set(
            "pts_off_us",
            rtt.filter(|r| r.has_offset()).map(|r| r.pts_off_us()),
        );
        let t = self.telem.as_ref();
        j.set("drone_rcf_rx", t.map(|t| t.rcf_rx));
        let (up, up_pct) = g.map_or((UplinkState::None, None), |g| {
            g.uplink.view(self.now_us / 1000)
        });
        j.set("uplink_pct", up_pct.map(i64::from));
        j.set("uplink", up.as_str());
        j.set("drone_state", t.map(|t| t.state));
        j.set(
            "drone_temp_c",
            t.filter(|t| t.soc_temp_c != -128)
                .map(|t| i64::from(t.soc_temp_c)),
        );
        // Gs only: a spotter has no RCF for the low_power readout to drive.
        const DRONE_LP_STALE_US: u64 = 3_000_000;
        j.set(
            "drone_lp",
            t.filter(|_| {
                g.is_some() && self.now_us.wrapping_sub(self.telem_rx_us) <= DRONE_LP_STALE_US
            })
            .map(|t| t.flags & 0x80 != 0),
        );
        j.set("rec_state", t.map(|t| t.rec_status & 0x03));
        j.set("rec_err", t.map(|t| t.rec_status >> 2));
        j.set("snr_db", self.last_health.rf_snr_db);
        j.set("rssi_dbm", self.last_health.rf_rssi_dbm);
        j.set("bodies", self.c.bodies);
        j.set("aus", self.c.aus_complete);
        j.set("trunc", self.c.aus_truncated);
        j.set("trunc_base", self.c.aus_truncated_base);
        j.set("salvaged", self.c.aus_salvaged);
        j.set("sends", g.map_or(0, |g| g.sends));
        j.set("rcf_sent", g.map_or(0, |g| g.rcf_sent));
        j.set("idr_req", self.idr_req);
        j.set("osd_snaps", 0u64);
        j.set("osd_screens", 0u64);
        j.set(
            "nack",
            g.and_then(|g| g.nack.as_ref()).map_or(J::Null, |n| {
                let c = n.t.stats();
                let mut o = J::obj();
                o.set("req", c.requests);
                o.set("rep", c.repeats);
                o.set("syms", c.syms_requested);
                o.set("tail", c.tail_requests);
                o.set("fill", c.filled);
                o.set("late", c.late_fill);
                o.set("waste", c.wasted);
                o.set("drop", c.dropped_deadline);
                o.set("sup", c.suppressed);
                o.set("lead", c.lead_skipped);
                o.set("sent", n.sent);
                o.set("settle_ms", n.t.settle_ms());
                o.set("fill_pps", n.fill_pps);
                o.set("fill_p50", n.fill_p50);
                o.set("fill_p90", n.fill_p90);
                o.set("fill_max", n.fill_max);
                o.set("late_max", n.late_max);
                o.set("drone_rx", n.drone_nack_rx);
                o.set("drone_syms", n.drone_retx_syms);
                o.set("drone_refused", n.drone_retx_refused);
                o
            }),
        );
        j.dump()
    }
}

/// `webgs::params_json`: the table with the client's state, or (Spotter,
/// no client) the static table with every value unknown.
fn params_json(pc: Option<&ParamClient>, now_ms: u64) -> String {
    use crate::mav::{Apply, ListState, PType, RowState, TABLE};
    fn ty(t: PType) -> &'static str {
        match t {
            PType::Bool => "bool",
            PType::Int32 => "int",
            PType::Real32 => "real",
            PType::Str => "str",
        }
    }
    fn apply(a: Apply) -> &'static str {
        match a {
            Apply::Knob => "knob",
            Apply::Reprogram => "reprogram",
            Apply::Rebuild => "rebuild",
        }
    }
    let mut j = J::obj();
    let mut rows = J::arr();
    let st = pc.map(ParamClient::state);
    for (i, row) in TABLE.iter().enumerate() {
        let mut r = J::obj();
        r.set("id", row.id);
        r.set("section", row.section);
        r.set("key", row.key);
        r.set("type", ty(row.ty));
        r.set("apply", apply(row.apply));
        r.set("help", row.help);
        match st {
            None => {
                r.set("value", J::Null);
                r.set("state", "idle");
                r.set("status", "");
                r.set("age_ms", J::Null);
            }
            Some(st) => {
                let rs = &st.rows[i];
                r.set("value", rs.value.as_ref().map(crate::mav::format_text));
                r.set("state", rs.st.as_str());
                r.set("status", rs.status.as_str());
                let terminal = matches!(
                    rs.st,
                    RowState::Accepted
                        | RowState::Failed
                        | RowState::Unsupported
                        | RowState::Timeout
                );
                r.set(
                    "age_ms",
                    terminal.then(|| now_ms.wrapping_sub(rs.result_ms) as i64),
                );
            }
        }
        rows.push(r);
    }
    match (pc, st) {
        (Some(pc), Some(st)) => {
            j.set("linked", st.linked);
            j.set("alive", (st.alive_ms != 0).then(|| pc.alive(now_ms)));
            j.set(
                "list",
                match st.list {
                    ListState::None => "none",
                    ListState::Pending => "pending",
                    ListState::Complete => "complete",
                    ListState::Partial => "partial",
                },
            );
            j.set("mismatch", st.table_mismatch);
            j.set("pending", st.pending_row.map_or(-1, |r| r as i64));
        }
        _ => {
            j.set("linked", false);
            j.set("mode", "spotter");
        }
    }
    j.set("rows", rows);
    j.dump()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chan::fake::{FakeCard, FakeClock};
    use crate::config::load_config;
    use std::path::Path;

    fn bundle() -> Config {
        let mut c = load_config(
            Path::new(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/../../third_party/mabur/gs/bundle/maburgs.default.toml"
            )),
            None,
        )
        .unwrap();
        c.radio.channels = vec![40, 64, 112, 144];
        c
    }

    #[derive(Default)]
    struct Io {
        sent: usize,
    }
    impl CoreIo for Io {
        fn on_au(&mut self, _: Au) {}
        fn send(&mut self, _: &[u8]) {
            self.sent += 1;
        }
    }

    fn opts(ch: u8) -> RustOpts {
        RustOpts {
            mode: Mode::Gs,
            channel: ch,
            ..RustOpts::default()
        }
    }

    fn env(card: &Arc<FakeCard>, clk: &FakeClock) -> ChanEnv {
        let (c1, c2, c3) = (clk.clone(), clk.clone(), clk.clone());
        ChanEnv {
            cards: vec![card.clone() as Arc<dyn LinkCard>],
            now_ms: Arc::new(move || c1.ms()),
            now_us: Arc::new(move || c2.ms() * 1000),
            sleep: Arc::new(move |ms| c3.advance(ms as u64)),
            threaded: false,
        }
    }

    /// In session (the search no longer owns the card) the RCFs leave
    /// through the card, none through CoreIo.
    #[test]
    fn with_cards_sends_through_the_card() {
        let mut cfg = bundle();
        cfg.radio.pin = Some(64);
        let clk = FakeClock::default();
        clk.set(1000);
        let card = FakeCard::new(clk.clone(), 64);
        let mut core = RustCore::with_cards(&cfg, opts(64), env(&card, &clk)).unwrap();
        let mut io = Io::default();
        core.tick(clk.ms() * 1000, &mut io);
        core.inject_disc_ack_for_replay(clk.ms() * 1000);
        for _ in 0..500 {
            clk.advance(2);
            core.tick(clk.ms() * 1000, &mut io);
        }
        assert_eq!(io.sent, 0);
        let sent = card.s().sent.clone();
        let g = core.gs.as_ref().unwrap();
        assert!(g.rcf_sent > 0, "no RCF went out");
        assert_eq!(sent.len() as u64, g.sends);
        let rcfs = sent
            .iter()
            .filter(|f| rc::frame_type(f) == Some(rc::T_RCF))
            .count();
        assert_eq!(rcfs as u64, g.rcf_sent);
        let j = core.stats_json();
        assert!(j.contains("\"hop\":{"), "{j}");
        assert!(j.contains("\"channel\":64"), "{j}");
    }

    /// A second card's bodies reach the aggregator (maburgs sizes it by the
    /// roster); with one card they are refused as a foreign card's.
    #[test]
    fn with_two_cards_both_feed_the_aggregator() {
        let mut cfg = bundle();
        cfg.radio.pin = Some(64);
        let clk = FakeClock::default();
        clk.set(1000);
        let a = FakeCard::new(clk.clone(), 64);
        let b = FakeCard::new(clk.clone(), 64);
        let mut two = env(&a, &clk);
        two.cards.push(b.clone() as Arc<dyn LinkCard>);
        let mut io = Io::default();
        let body = RxBody {
            card_id: 1,
            mono_us: clk.ms() * 1000,
            crc_ok: true,
            rx_channel: 64,
            body: vec![0u8; 64],
            ..RxBody::default()
        };
        let mut core = RustCore::with_cards(&cfg, opts(64), two).unwrap();
        core.on_rx(&body, &mut io);
        assert_eq!(core.agg.bad_card_msgs(), 0);
        assert!(core.stats_json().contains("\"cards\":[{"));
        assert!(core.stats_json().contains("\"tx_card\":0"));
        let mut one = RustCore::with_cards(&cfg, opts(64), env(&a, &clk)).unwrap();
        one.on_rx(&body, &mut io);
        assert_eq!(one.agg.bad_card_msgs(), 1);
        assert!(!one.stats_json().contains("\"cards\""));
    }

    /// The TX selector moves the RCFs to the card that hears the drone
    /// 10 dB better once the margin has held 2 s (maburgs TxSelector).
    #[test]
    fn rcfs_follow_the_stronger_card() {
        let mut cfg = bundle();
        cfg.radio.pin = Some(64);
        let clk = FakeClock::default();
        clk.set(1000);
        let a = FakeCard::new(clk.clone(), 64);
        let b = FakeCard::new(clk.clone(), 64);
        let mut two = env(&a, &clk);
        two.cards.push(b.clone() as Arc<dyn LinkCard>);
        let mut core = RustCore::with_cards(&cfg, opts(64), two).unwrap();
        let mut io = Io::default();
        core.tick(clk.ms() * 1000, &mut io);
        core.inject_disc_ack_for_replay(clk.ms() * 1000);
        let body = |card: u8, rssi: u8, us: u64| RxBody {
            card_id: card,
            mono_us: us,
            rssi: [rssi, rssi],
            // RF folds only from frames with a PHY status.
            phy_valid: true,
            crc_ok: true,
            rx_channel: 64,
            body: vec![0u8; 64],
            ..RxBody::default()
        };
        let rcfs = |c: &FakeCard| {
            c.s()
                .sent
                .iter()
                .filter(|f| rc::frame_type(f) == Some(rc::T_RCF))
                .count()
        };
        let mut b_at_1s = 0;
        for i in 0..3000 {
            clk.advance(2);
            let us = clk.ms() * 1000;
            core.on_rx(&body(0, 50, us), &mut io);
            core.on_rx(&body(1, 60, us), &mut io);
            core.tick(us, &mut io);
            if i == 500 {
                b_at_1s = rcfs(&b);
            }
        }
        let g = core.gs.as_ref().unwrap();
        assert!(rcfs(&a) > 0, "card 0 carried the start");
        assert_eq!(b_at_1s, 0, "switched before the 2 s hold");
        assert!(rcfs(&b) > 0, "never moved to the stronger card");
        assert_eq!(g.sel.selected(), 1);
        assert_eq!(g.sel.switches(), 1);
    }

    /// Auto: the search runs on the card and the stats line reports it.
    #[test]
    fn with_cards_reports_the_search() {
        let cfg = bundle();
        let clk = FakeClock::default();
        clk.set(1000);
        let card = FakeCard::new(clk.clone(), 40);
        let mut core = RustCore::with_cards(&cfg, opts(40), env(&card, &clk)).unwrap();
        let mut io = Io::default();
        // Unthreaded, the scout's dwells sleep inside tick (the clock jumps
        // with them): a few hundred ticks cover the boot pick and beyond.
        for _ in 0..300 {
            clk.advance(2);
            core.tick(clk.ms() * 1000, &mut io);
        }
        assert_eq!(io.sent, 0);
        assert!(card.calls().iter().any(|c| c.starts_with("retune")));
        let j = core.stats_json();
        assert!(j.contains("\"scan_pick\":40"), "{j}");
        assert!(j.contains("\"hop\":{"), "{j}");
    }

    /// The calibration lane end to end through the core: the SweepCmd
    /// leaves through the card on the cal lane, the drone's ack (on the
    /// tunnel) silences every other send, sweep frames are routed to the
    /// session and never reach the video path, and an abort reopens the
    /// air with the closing Aborts.
    #[test]
    fn calibration_runs_through_the_core_and_silences_the_link() {
        let mut cfg = bundle();
        cfg.radio.pin = Some(64);
        let clk = FakeClock::default();
        clk.set(1000);
        let card = FakeCard::new(clk.clone(), 64);
        let mut core = RustCore::with_cards(&cfg, opts(64), env(&card, &clk)).unwrap();
        let mut io = Io::default();
        let us = |c: &FakeClock| c.ms() * 1000;
        core.tick(us(&clk), &mut io);
        assert!(core.cal_start(&[]).unwrap_err().contains("link"));
        // A drone that advertises calibration.
        let ack = {
            let g = core.gs.as_ref().unwrap();
            rc::DiscAck {
                vrx_nonce: g.vrx.rz_nonce(),
                vtx_nonce: 9,
                chip_caps: rc::CAP_FRAME_WIRE | rc::CAP_CALIBRATE,
                agreed_channel: g.vrx.proposal(),
                agreed_width: 20,
                seq: 1,
                ..rc::DiscAck::default()
            }
        };
        let body = |b: Vec<u8>, c: &FakeClock| RxBody {
            mono_us: us(c),
            crc_ok: true,
            rx_channel: 64,
            body: b,
            ..RxBody::default()
        };
        core.on_rx(&body(rc::pack_disc_ack(&ack), &clk), &mut io);
        for _ in 0..50 {
            clk.advance(2);
            core.tick(us(&clk), &mut io);
        }
        core.cal_start(&[SweepSpec {
            kind: cal::Kind::Power,
            bw: 20,
        }])
        .unwrap();
        assert!(core.cal_start(&[]).unwrap_err().contains("already"));
        let n0 = card.s().sent.len();
        clk.advance(2);
        core.tick(us(&clk), &mut io);
        let cal_frames = |from: usize| -> Vec<cal::CalMsg> {
            card.s().sent[from..]
                .iter()
                .filter(|f| rc::frame_type(f) == Some(rc::T_MAV))
                .filter_map(|f| rc::parse_mav(f, true))
                .flat_map(|m| cal::decode_all(&m.payload))
                .collect()
        };
        let cmds = cal_frames(n0);
        let Some(cal::CalMsg::Sweep(cmd)) = cmds.first() else {
            panic!("no SweepCmd went out: {cmds:?}");
        };
        assert_eq!((cmd.kind, cmd.bw, cmd.seq), (cal::Kind::Power, 20, 0));

        // The drone accepts over the tunnel, tagged under the run's pair.
        let ctx = core.gs.as_ref().unwrap().cal.tag_ctx();
        let reply = cal::encode(
            &cal::CalMsg::Ack(cal::SweepAck {
                nonce: cmd.nonce,
                seq: 0,
                result: cal::AckResult::Accepted,
                start_in_ms: 0,
                flags: cal::FLAG_LDPC,
            }),
            1,
            1,
            0,
        );
        let m = rc::Mav {
            counter: 1,
            payload: reply,
        };
        core.on_rx(
            &body(rc::pack_mav(&m, false, &cfg.link.key, &ctx), &clk),
            &mut io,
        );
        assert_eq!(
            core.gs.as_ref().unwrap().cal.state(),
            crate::cal::session::State::Sweep
        );

        // Silent: nothing leaves while the sweep airs.
        let n1 = card.s().sent.len();
        for _ in 0..200 {
            clk.advance(5);
            core.tick(us(&clk), &mut io);
        }
        assert_eq!(card.s().sent.len(), n1, "a send broke the radio silence");

        // Sweep frames reach the session, not the video path.
        let p = cal::build_payload(
            cal::Kind::Power,
            0,
            0,
            20,
            cal::FLAG_LDPC,
            0,
            cal::PAYLOAD_LEN,
        );
        let bodies = core.c.bodies;
        core.on_rx(&body(p, &clk), &mut io);
        assert_eq!(core.c.cal_frames, 1);
        assert_eq!(core.c.bodies, bodies + 1);
        assert_eq!(core.gs.as_ref().unwrap().cal.cell_received(0, 0, 0), 1);
        let j = core.cal_json().unwrap();
        assert!(j.contains("\"state\":\"sweep\""), "{j}");

        // Abort: the air reopens and the closing Aborts go out.
        core.cal_abort();
        let n2 = card.s().sent.len();
        for _ in 0..200 {
            clk.advance(5);
            core.tick(us(&clk), &mut io);
        }
        let aborts = cal_frames(n2)
            .into_iter()
            .filter(|m| matches!(m, cal::CalMsg::Abort(a) if a.nonce == cmd.nonce))
            .count();
        assert_eq!(aborts, 3);
        // The fake drone said nothing for the sweep, so the session fell back
        // to beaconing: the reopened air carries DISC or RCF, whichever.
        assert!(card.s().sent[n2..]
            .iter()
            .any(|f| matches!(rc::frame_type(f), Some(rc::T_RCF | rc::T_DISC))));
    }

    #[test]
    fn without_cards_the_search_fields_stay_null() {
        let mut core = RustCore::new(&bundle(), opts(40)).unwrap();
        let mut io = Io::default();
        for t in 0..500u64 {
            core.tick(1_000_000 + t * 2000, &mut io);
        }
        assert!(io.sent > 0, "the DISC goes out through CoreIo");
        let j = core.stats_json();
        assert!(j.contains("\"scan_state\":null"), "{j}");
        assert!(j.contains("\"hop\":null"), "{j}");
    }

    #[test]
    fn with_cards_refuses_a_spotter_and_no_cards() {
        let clk = FakeClock::default();
        let mk = |cards: Vec<Arc<dyn LinkCard>>| {
            let (a, b) = (clk.clone(), clk.clone());
            ChanEnv {
                cards,
                now_ms: Arc::new(move || a.ms()),
                now_us: Arc::new(move || b.ms() * 1000),
                sleep: Arc::new(|_| {}),
                threaded: false,
            }
        };
        let spot = RustOpts {
            mode: Mode::Spotter,
            ..opts(40)
        };
        let card = FakeCard::new(clk.clone(), 40) as Arc<dyn LinkCard>;
        assert!(RustCore::with_cards(&bundle(), spot, mk(vec![card])).is_err());
        assert!(RustCore::with_cards(&bundle(), opts(40), mk(vec![])).is_err());
    }
}
