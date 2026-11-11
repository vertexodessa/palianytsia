//! Builds the ladder's LinkHealth from the decoder and the probe stream once
//! per tick (mabur `gs/src/link_health.cpp` with `s1_loss.h`,
//! `transition_edge.h`, `probe_track.cpp`, `ladder_residual.cpp`,
//! `rf_labels.h`).
use crate::aggregator::Aggregator;
use crate::ladder::LinkHealth;
use crate::uep::UepDecoder;
use crate::RxBody;
use pal_wire::probe::{self, ProbeRx};
use pal_wire::profile::{self, PhyMode, Profile};
use pal_wire::rc::NO_PROBE_PROFILE;
use std::collections::VecDeque;

pub use crate::chan::radio::{EVM_RAW_TO_DB, SNR_RAW_TO_DB};

/// The commanded operating point (power is constant, not part of it).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OpPoint {
    pub vht: bool,
    pub mcs: i32,
    pub bw: i32,
    pub overhead_base: f64,
    pub overhead_enh: f64,
}

impl Default for OpPoint {
    fn default() -> Self {
        Self {
            vht: false,
            mcs: 0,
            bw: 20,
            overhead_base: 1.0,
            overhead_enh: 1.0,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct LossSample {
    pub valid: bool,
    pub loss: f64,
}

/// Loss over a sliding time window from cumulative {expected, arrived}
/// counters; a counter going backwards (decoder reset) re-baselines.
pub struct S1LossWindow {
    window_ms: f64,
    prev_expected: u64,
    prev_arrived: u64,
    have_first: bool,
    blank_until_ms: f64,
    w: VecDeque<(f64, u64, u64)>,
}

impl Default for S1LossWindow {
    fn default() -> Self {
        Self::new(500.0)
    }
}

impl S1LossWindow {
    pub fn new(window_ms: f64) -> Self {
        Self {
            window_ms,
            prev_expected: 0,
            prev_arrived: 0,
            have_first: false,
            blank_until_ms: -1.0,
            w: VecDeque::new(),
        }
    }
    pub fn blank_until(&mut self, t: f64) {
        self.w.clear();
        self.blank_until_ms = t;
    }
    pub fn add(&mut self, expected: u64, arrived: u64, now: f64) {
        let mut reset = now < self.blank_until_ms;
        if self.have_first && (expected < self.prev_expected || arrived < self.prev_arrived) {
            self.w.clear();
            reset = true;
        }
        if !reset {
            let de = expected - self.prev_expected;
            let da = arrived - self.prev_arrived;
            if de > 0 || da > 0 {
                self.w.push_back((now, de, da));
            }
            let cut = now - self.window_ms;
            while self.w.front().is_some_and(|e| e.0 < cut) {
                self.w.pop_front();
            }
        }
        self.prev_expected = expected;
        self.prev_arrived = arrived;
        self.have_first = true;
    }
    fn in_window(&self, now: f64) -> impl Iterator<Item = &(f64, u64, u64)> {
        let cut = now - self.window_ms;
        self.w.iter().filter(move |e| e.0 >= cut && e.0 <= now)
    }
    pub fn sample(&self, now: f64) -> LossSample {
        let (mut e, mut a) = (0u64, 0u64);
        for x in self.in_window(now) {
            e += x.1;
            a += x.2;
        }
        if e == 0 {
            return LossSample {
                valid: false,
                loss: 0.0,
            };
        }
        let l = (e as f64 - a as f64) / e as f64;
        LossSample {
            valid: true,
            loss: l.clamp(0.0, 1.0),
        }
    }
    pub fn expected_in_window(&self, now: f64) -> u64 {
        self.in_window(now).map(|x| x.1).sum()
    }
}

/// Post-FEC {arrived, expected} of one layer for the stats (`residual_counts` in
/// `ladder_residual.cpp`): the decoder's own abandonment, plus symbols a NACK retransmit filled.
#[derive(Clone, Copy, Debug, Default)]
pub struct ResidualCounts {
    pub arrived: u64,
    pub expected: u64,
}

pub fn residual_counts(dec: &UepDecoder, sid: usize, cur: bool) -> ResidualCounts {
    let s = dec.stats(sid).dec;
    let ab = if cur {
        s.syms_abandoned - s.syms_abandoned_stale
    } else {
        s.syms_abandoned
    } + s.syms_retx;
    let exp = s.syms_delivered + s.syms_recovered + ab;
    ResidualCounts {
        arrived: exp - ab,
        expected: exp,
    }
}

/// The ladder's instant-demote input (`ladder_residual_counts` in `ladder_residual.cpp`): base
/// layer, current rung, the decoder's abandonment alone. A symbol a NACK retransmit filled counts
/// as arrived: the drone's resends race the frame's own FEC repairs, and the ones that land first
/// fill symbols the repairs would have solved anyway, so booking them as loss cost a rung on a
/// link that abandons nothing with the NACK off.
pub fn ladder_residual_counts(dec: &UepDecoder) -> ResidualCounts {
    let s = dec.stats(0).dec;
    let ab = s.syms_abandoned - s.syms_abandoned_stale;
    let exp = s.syms_delivered + s.syms_recovered + s.syms_retx + ab;
    ResidualCounts {
        arrived: exp - ab,
        expected: exp,
    }
}

pub fn residual_counts_pooled(dec: &UepDecoder, cur: bool) -> ResidualCounts {
    let (b, e) = (residual_counts(dec, 0, cur), residual_counts(dec, 1, cur));
    ResidualCounts {
        arrived: b.arrived + e.arrived,
        expected: b.expected + e.expected,
    }
}

const RESID_SETTLE_MS: f64 = 150.0;

/// Arms the decoder's transition boundary when the commanded op changes.
struct TransitionEdge {
    last_op_mcs: i32,
    last_op_bw: i32,
    last_op_ov: f64,
    last_enh_mcs: i32,
    last_enh_bw: i32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ProbeCounts {
    pub expected_blocks: u64,
    pub arrived_blocks: u64,
    pub bodies_rx: u64,
}

#[derive(Clone, Copy)]
struct Pending {
    seq: u32,
    profile: u8,
    first_ms: f64,
    bitmap: u32,
    card_bits: [u32; 8],
    au_matched: bool,
}

#[derive(Clone, Copy)]
struct PendingAu {
    enh_fid: u16,
    t_ms: f64,
    scored: bool,
}

const PROBE_RING_MAX: usize = 64;

/// Scores probe bodies against the enh AUs they shadow (`probe_track.cpp`).
pub struct ProbeTrack {
    bpb: u64,
    finalize_ms: f64,
    max_cards: usize,
    commanded: u8,
    ring: VecDeque<Pending>,
    au_pending: VecDeque<PendingAu>,
    union: ProbeCounts,
    cards: [ProbeCounts; 8],
    off_profile: u64,
}

impl ProbeTrack {
    fn new(bpb: usize, finalize_ms: f64, max_cards: usize) -> Self {
        Self {
            bpb: bpb as u64,
            finalize_ms,
            max_cards: max_cards.min(8),
            commanded: 0xFF,
            ring: VecDeque::new(),
            au_pending: VecDeque::new(),
            union: ProbeCounts::default(),
            cards: [ProbeCounts::default(); 8],
            off_profile: 0,
        }
    }
    pub fn union_counts(&self) -> &ProbeCounts {
        &self.union
    }
    pub fn off_profile(&self) -> u64 {
        self.off_profile
    }
    fn on_au(&mut self, sid: u8, fid: u16, now: f64) {
        if self.commanded == 0xFF || sid >= 2 {
            return;
        }
        self.au_pending.push_back(PendingAu {
            enh_fid: fid,
            t_ms: now,
            scored: true,
        });
    }
    fn on_body(&mut self, card: usize, rx: &ProbeRx, now: f64) {
        if card >= 8 {
            return;
        }
        let mut pos = self.ring.iter().position(|p| p.seq == rx.hdr.seq);
        let first_sight = pos.is_none();
        if first_sight {
            let mut p = Pending {
                seq: rx.hdr.seq,
                profile: rx.hdr.profile,
                first_ms: now,
                bitmap: 0,
                card_bits: [0; 8],
                au_matched: false,
            };
            if rx.hdr.profile == self.commanded {
                if let Some(i) = self
                    .au_pending
                    .iter()
                    .position(|a| a.enh_fid == rx.hdr.enh_fid)
                {
                    p.au_matched = true;
                    self.au_pending.remove(i);
                }
            }
            self.ring.push_back(p);
            if self.ring.len() > PROBE_RING_MAX {
                let f = self.ring.pop_front().expect("ring");
                self.finalize_body(&f);
            }
            pos = Some(self.ring.len() - 1);
        }
        let it = &mut self.ring[pos.expect("pos")];
        it.bitmap |= rx.survivors;
        it.card_bits[card] |= rx.survivors;
        if first_sight && rx.hdr.profile != self.commanded {
            self.off_profile += 1;
            if let Some(a) = self
                .au_pending
                .iter_mut()
                .find(|a| a.enh_fid == rx.hdr.enh_fid)
            {
                a.scored = false;
            }
        }
    }
    fn add_expected(&mut self) {
        self.union.expected_blocks += self.bpb;
        for c in &mut self.cards[..self.max_cards] {
            c.expected_blocks += self.bpb;
        }
    }
    fn finalize_body(&mut self, p: &Pending) {
        if p.profile != self.commanded {
            return;
        }
        if p.au_matched {
            self.add_expected();
        }
        self.union.arrived_blocks += u64::from(p.bitmap.count_ones());
        self.union.bodies_rx += 1;
        for (c, cnt) in self.cards[..self.max_cards].iter_mut().enumerate() {
            let b = p.card_bits[c];
            cnt.arrived_blocks += u64::from(b.count_ones());
            if b != 0 {
                cnt.bodies_rx += 1;
            }
        }
    }
    fn tick(&mut self, now: f64) {
        while self
            .au_pending
            .front()
            .is_some_and(|a| now - a.t_ms >= self.finalize_ms)
        {
            let a = self.au_pending.pop_front().expect("au");
            if a.scored {
                self.add_expected();
            }
        }
        while self
            .ring
            .front()
            .is_some_and(|p| now - p.first_ms >= self.finalize_ms)
        {
            let p = self.ring.pop_front().expect("ring");
            self.finalize_body(&p);
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct LinkHealthInputs {
    pub op: OpPoint,
    pub probe_profile: u8,
    pub probe_rung: i32,
}

pub struct LhTick {
    pub health: LinkHealth,
    /// Set only on a commanded-probe edge.
    pub probe_tail_ms: Option<i32>,
}

const PROBE_SWITCH_BLANK_MS: f64 = 150.0;

pub struct LinkHealthAssembler {
    n_cards: usize,
    probe_bpb: usize,
    probe_block_payload: usize,
    probe_track: ProbeTrack,
    probe_loss: S1LossWindow,
    probe_card_loss: Vec<S1LossWindow>,
    probe_cmd_last: u8,
    s1_loss: S1LossWindow,
    pre_loss_all: S1LossWindow,
    s3_loss: S1LossWindow,
    s1_loss_cur: S1LossWindow,
    s1_resid_cur: S1LossWindow,
    s3_loss_cur: S1LossWindow,
    s3_resid_cur: S1LossWindow,
    pool_resid: S1LossWindow,
    pool_resid_cur: S1LossWindow,
    layer_resid: [S1LossWindow; 2],
    prev_pool_frames: Vec<u64>,
    prev_pkts_out: u64,
    pkts_now: u64,
    edge: TransitionEdge,
    pub residual: Option<f64>,
    pub residual_cur: Option<f64>,
    pub pre_all: LossSample,
    pub layer_delivery_pct: [u8; 2],
    pub last_health: LinkHealth,
}

impl LinkHealthAssembler {
    pub fn new(n_cards: usize, probe_bpb: usize, probe_block_payload: usize) -> Self {
        Self {
            n_cards,
            probe_bpb,
            probe_block_payload,
            probe_track: ProbeTrack::new(probe_bpb, 100.0, n_cards),
            probe_loss: S1LossWindow::default(),
            probe_card_loss: (0..n_cards).map(|_| S1LossWindow::default()).collect(),
            probe_cmd_last: NO_PROBE_PROFILE,
            s1_loss: S1LossWindow::default(),
            pre_loss_all: S1LossWindow::default(),
            s3_loss: S1LossWindow::default(),
            s1_loss_cur: S1LossWindow::default(),
            s1_resid_cur: S1LossWindow::default(),
            s3_loss_cur: S1LossWindow::default(),
            s3_resid_cur: S1LossWindow::default(),
            pool_resid: S1LossWindow::default(),
            pool_resid_cur: S1LossWindow::default(),
            layer_resid: [S1LossWindow::default(), S1LossWindow::default()],
            prev_pool_frames: vec![0; n_cards],
            prev_pkts_out: 0,
            pkts_now: 0,
            edge: TransitionEdge {
                last_op_mcs: -1,
                last_op_bw: -1,
                last_op_ov: -1.0,
                last_enh_mcs: -1,
                last_enh_bw: -1,
            },
            residual: None,
            residual_cur: None,
            pre_all: LossSample {
                valid: false,
                loss: 0.0,
            },
            layer_delivery_pct: [0; 2],
            last_health: LinkHealth::empty(),
        }
    }

    pub fn probe_commanded(&self) -> u8 {
        self.probe_cmd_last
    }
    pub fn probe_track(&self) -> &ProbeTrack {
        &self.probe_track
    }

    pub fn on_au_begin(&mut self, sid: u8, frame_id: u16, now: f64) {
        self.probe_track.on_au(sid, frame_id, now);
    }

    pub fn on_probe_body(&mut self, card: u8, m: &RxBody) {
        if let Some(rx) = probe::parse_body(&m.body, self.probe_block_payload) {
            self.probe_track
                .on_body(usize::from(card), &rx, m.mono_us as f64 / 1000.0);
        }
    }

    fn edge_tick(&mut self, op: &OpPoint, dec: &mut UepDecoder, now: f64) {
        let e = &mut self.edge;
        if op.mcs != e.last_op_mcs || op.bw != e.last_op_bw || op.overhead_base != e.last_op_ov {
            let mode = if op.vht { PhyMode::Vht } else { PhyMode::Ht };
            dec.mark_transition(0, profile::layer_mcs(mode, op.mcs as u8), now as u64);
            self.s1_resid_cur.blank_until(now + RESID_SETTLE_MS);
            self.s1_loss_cur.blank_until(now);
            e.last_op_mcs = op.mcs;
            e.last_op_bw = op.bw;
            e.last_op_ov = op.overhead_base;
        }
        if op.mcs != e.last_enh_mcs || op.bw != e.last_enh_bw {
            dec.mark_transition(1, op.mcs as u8, now as u64);
            self.s3_resid_cur.blank_until(now + RESID_SETTLE_MS);
            self.s3_loss_cur.blank_until(now);
            e.last_enh_mcs = op.mcs;
            e.last_enh_bw = op.bw;
        }
    }

    pub fn tick(&mut self, now: f64, agg: &mut Aggregator, inp: &LinkHealthInputs) -> LhTick {
        let mut out_tail = None;
        self.edge_tick(&inp.op, agg.decoder_mut(), now);
        let dec = agg.decoder();
        let mut ld = [0u8; 2];
        for (s, slot) in ld.iter_mut().enumerate() {
            let lc = residual_counts(dec, s, false);
            let w = &mut self.layer_resid[s];
            w.add(lc.expected, lc.arrived, now);
            let ls = w.sample(now);
            let pct = if ls.valid {
                (100.0 * (1.0 - ls.loss)) as i32
            } else {
                100
            };
            *slot = pct.clamp(0, 100) as u8;
        }
        let rc_tot = residual_counts_pooled(dec, false);
        let rc_cur = residual_counts_pooled(dec, true);
        self.pool_resid.add(rc_tot.expected, rc_tot.arrived, now);
        self.pool_resid_cur
            .add(rc_cur.expected, rc_cur.arrived, now);
        let pr = self.pool_resid.sample(now);
        let prc = self.pool_resid_cur.sample(now);
        let s1 = dec.stats(0);
        let s2 = dec.stats(1);
        let pkts_now = s1.dec.packets_out + s2.dec.packets_out;
        let starved = pkts_now == self.prev_pkts_out && agg.last_video_us() != 0;
        self.s1_loss.add(s1.arr_expected, s1.arr_arrived, now);
        let s1_sample = self.s1_loss.sample(now);
        self.s1_loss_cur.add(
            s1.arr_expected - s1.arr_expected_stale,
            s1.arr_arrived - s1.arr_arrived_stale,
            now,
        );
        let s1_cur = self.s1_loss_cur.sample(now);
        self.pre_loss_all.add(
            (s1.arr_expected - s1.arr_expected_stale) + (s2.arr_expected - s2.arr_expected_stale),
            (s1.arr_arrived - s1.arr_arrived_stale) + (s2.arr_arrived - s2.arr_arrived_stale),
            now,
        );
        let pre_all = self.pre_loss_all.sample(now);
        let s1_rc = ladder_residual_counts(dec);
        self.s1_resid_cur.add(s1_rc.expected, s1_rc.arrived, now);
        let s1_rcur = self.s1_resid_cur.sample(now);
        let s3 = s2;
        self.s3_loss.add(s3.arr_expected, s3.arr_arrived, now);
        let s3_sample = self.s3_loss.sample(now);
        let s3_ab_cur = s3.dec.syms_abandoned - s3.dec.syms_abandoned_stale;
        let s3_exp_cur =
            s3.dec.syms_delivered + s3.dec.syms_recovered + s3.dec.syms_retx + s3_ab_cur;
        self.s3_loss_cur.add(
            s3.arr_expected - s3.arr_expected_stale,
            s3.arr_arrived - s3.arr_arrived_stale,
            now,
        );
        self.s3_resid_cur
            .add(s3_exp_cur, s3_exp_cur - s3_ab_cur, now);
        let s3_cur = self.s3_loss_cur.sample(now);
        let s3_rcur = self.s3_resid_cur.sample(now);

        self.probe_track.tick(now);
        if inp.probe_profile != self.probe_cmd_last {
            let pc = inp.probe_profile;
            self.probe_cmd_last = pc;
            self.probe_track.commanded = pc;
            self.probe_loss.blank_until(now + PROBE_SWITCH_BLANK_MS);
            for w in &mut self.probe_card_loss {
                w.blank_until(now + PROBE_SWITCH_BLANK_MS);
            }
            let mut tail = 0;
            if pc != NO_PROBE_PROFILE {
                let Profile { mode, mcs, bw_mhz } = profile::decode(pc);
                let rate = profile::phy_rate_mbps(mode, profile::layer_mcs(mode, mcs), bw_mhz);
                let us = if rate > 0.0 {
                    probe::body_len(self.probe_bpb, self.probe_block_payload) as f64 * 8.0 / rate
                } else {
                    0.0
                };
                tail = ((us / 1000.0).ceil() as i32).max(1);
            }
            out_tail = Some(tail);
        }
        let pu = *self.probe_track.union_counts();
        self.probe_loss
            .add(pu.expected_blocks, pu.arrived_blocks, now);
        for i in 0..self.n_cards {
            let c = self.probe_track.cards[i.min(7)];
            self.probe_card_loss[i].add(c.expected_blocks, c.arrived_blocks, now);
        }
        let probe_sample = self.probe_loss.sample(now);

        // Label card: the best fresh pooled SNR.
        let mut best: Option<usize> = None;
        for (i, c) in agg.cards().iter().enumerate().take(self.n_cards) {
            let ct = &c.rf_pool;
            if !ct.has_ema || ct.frames <= self.prev_pool_frames[i] {
                continue;
            }
            if best.is_none_or(|b| ct.snr_ema > agg.cards()[b].rf_pool.snr_ema) {
                best = Some(i);
            }
        }
        let (mut snr, mut evm, mut rssi) = (f64::NAN, f64::NAN, f64::NAN);
        if let Some(b) = best {
            let ct = &agg.cards()[b].rf_pool;
            snr = ct.snr_ema * SNR_RAW_TO_DB;
            if ct.evm_has {
                evm = ct.evm_ema * EVM_RAW_TO_DB;
            }
            rssi = ct.rssi_ema - 110.0;
        }
        let health = LinkHealth {
            sample_valid: s1_sample.valid,
            pre_fec_loss: if s1_cur.valid { s1_cur.loss } else { 0.0 },
            residual_loss: if s1_rcur.valid { s1_rcur.loss } else { 0.0 },
            video_starved: starved,
            s3_valid: s3_sample.valid,
            s3_pre_fec_loss: if s3_cur.valid { s3_cur.loss } else { 0.0 },
            s3_residual_loss: if s3_rcur.valid { s3_rcur.loss } else { 0.0 },
            s3_expected_syms: self.s3_loss.expected_in_window(now),
            rf_snr_db: snr,
            rf_evm_db: evm,
            rf_rssi_dbm: rssi,
            probe_valid: probe_sample.valid,
            probe_loss: if probe_sample.valid {
                probe_sample.loss
            } else {
                0.0
            },
            probe_expected_syms: self.probe_loss.expected_in_window(now),
            probe_bodies_total: pu.expected_blocks / self.probe_bpb as u64,
            probe_rung: inp.probe_rung,
        };
        self.residual = pr.valid.then_some(pr.loss);
        self.residual_cur = prc.valid.then_some(prc.loss);
        self.layer_delivery_pct = ld;
        self.pre_all = pre_all;
        self.pkts_now = pkts_now;
        self.last_health = health;
        LhTick {
            health,
            probe_tail_ms: out_tail,
        }
    }

    /// An RCF left: the starvation and label windows restart here.
    pub fn on_step_sent(&mut self, agg: &Aggregator) {
        self.prev_pkts_out = self.pkts_now;
        for (i, c) in agg.cards().iter().enumerate().take(self.n_cards) {
            self.prev_pool_frames[i] = c.rf_pool.frames;
        }
    }
}
