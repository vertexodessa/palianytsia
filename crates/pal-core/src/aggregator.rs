//! Per-card RX accounting and body routing (mabur `gs/src/aggregator.cpp`):
//! RC frames to the control path, MSP and probe streams to their sinks,
//! video into the UEP decoder. Instead of mabur's sink callbacks the route
//! is returned, so the core decides what to do with it.
use crate::config::UepLayerCfg;
use crate::uep::{DecodedFrag, UepDecoder};
use crate::RxBody;
use pal_wire::{rc, sbi};

const MAX_SEQ_GAP: u16 = 512;
const EMA_ALPHA: f64 = 0.1;

pub const RF_S0: usize = 0;
pub const RF_S1: usize = 1;
pub const RF_PROBE: usize = 2;
pub const RF_MSP: usize = 3;
pub const RF_CTRL: usize = 4;
pub const NUM_RF_CLASSES: usize = 5;

/// RF EMAs, one shape for a card, a class and the video pool.
#[derive(Clone, Copy, Debug, Default)]
pub struct RfTrack {
    pub frames: u64,
    pub bytes: u64,
    pub rssi_ema: f64,
    pub rssi_a_ema: f64,
    pub rssi_b_ema: f64,
    pub snr_ema: f64,
    pub snr_a_ema: f64,
    pub snr_b_ema: f64,
    pub has_ema: bool,
    pub evm_ema: f64,
    pub evm_a_ema: f64,
    pub evm_b_ema: f64,
    pub evm_has: bool,
    pub evm_a_has: bool,
    pub evm_b_has: bool,
}

impl RfTrack {
    fn fold_rf(&mut self, rssi: f64, snr: f64, ab: [u8; 2], sab: [i8; 2]) {
        let vals = [
            rssi,
            f64::from(ab[0]),
            f64::from(ab[1]),
            snr,
            f64::from(sab[0]),
            f64::from(sab[1]),
        ];
        let emas = [
            &mut self.rssi_ema,
            &mut self.rssi_a_ema,
            &mut self.rssi_b_ema,
            &mut self.snr_ema,
            &mut self.snr_a_ema,
            &mut self.snr_b_ema,
        ];
        let first = !self.has_ema;
        for (e, v) in emas.into_iter().zip(vals) {
            *e = if first {
                v
            } else {
                (1.0 - EMA_ALPHA) * *e + EMA_ALPHA * v
            };
        }
        self.has_ema = true;
    }
    fn fold_evm(&mut self, a: i8, b: i8) {
        let sampled = |v: i8| v != 0 && v != i8::MIN;
        let fold = |ema: &mut f64, has: &mut bool, v: i8| {
            if !*has {
                *ema = f64::from(v);
                *has = true;
            } else {
                *ema = (1.0 - EMA_ALPHA) * *ema + EMA_ALPHA * f64::from(v);
            }
        };
        if sampled(a) {
            fold(&mut self.evm_a_ema, &mut self.evm_a_has, a);
        }
        if sampled(b) {
            fold(&mut self.evm_b_ema, &mut self.evm_b_has, b);
        }
        let best = if sampled(a) && sampled(b) {
            a.min(b)
        } else if sampled(a) {
            a
        } else {
            b
        };
        if sampled(best) {
            fold(&mut self.evm_ema, &mut self.evm_has, best);
        }
    }
    fn fold(&mut self, m: &RxBody, rssi: f64, snr: f64) {
        if m.phy_valid {
            self.fold_rf(rssi, snr, m.rssi, m.snr);
        }
        self.fold_evm(m.evm[0], m.evm[1]);
    }
    fn count_and_fold(&mut self, m: &RxBody, rssi: f64, snr: f64) {
        self.frames += 1;
        self.bytes += m.body.len() as u64;
        self.fold(m, rssi, snr);
    }
}

#[derive(Clone, Debug, Default)]
pub struct CardTrack {
    pub frames: u64,
    pub crc_fail: u64,
    pub rc_frames: u64,
    pub self_frames: u64,
    pub video_bodies: u64,
    pub seq_expected: u64,
    pub seq_received: u64,
    pub rx_bytes: u64,
    pub rf: RfTrack,
    pub has_seq: bool,
    pub last_seq: u16,
    pub last_frame_us: u64,
    pub agg_pos: u32,
    pub cls: [RfTrack; NUM_RF_CLASSES],
    /// S0 + S1 pooled.
    pub rf_pool: RfTrack,
}

impl CardTrack {
    /// Every frame that is not our own, FCS-good or not.
    fn count_frame(&mut self, m: &RxBody) {
        self.frames += 1;
        self.rx_bytes += m.body.len() as u64;
        self.last_frame_us = m.mono_us;
        if !m.crc_ok {
            self.crc_fail += 1;
        }
    }

    /// An FCS-good frame: the aggregation position, the sequence walk, and
    /// the RF EMAs of the card, of its class and (video) of the pool.
    fn on_clean_frame(&mut self, m: &RxBody, class: Option<usize>) {
        self.agg_pos = if m.phy_valid { 0 } else { self.agg_pos + 1 };
        self.walk_seq(m.mac_seq);
        let rssi = f64::from(m.rssi[0].max(m.rssi[1]));
        let snr = f64::from(m.snr[0].max(m.snr[1]));
        self.rf.fold(m, rssi, snr);
        if let Some(ci) = class {
            self.cls[ci].count_and_fold(m, rssi, snr);
            if ci == RF_S0 || ci == RF_S1 {
                self.rf_pool.count_and_fold(m, rssi, snr);
            }
        }
    }

    /// The 802.11 sequence walk: a retry is one delivered frame, a jump
    /// beyond MAX_SEQ_GAP counts one, a step back (>= 2048) counts one
    /// received and none expected.
    fn walk_seq(&mut self, seq: u16) {
        self.seq_received += 1;
        if !self.has_seq {
            self.seq_expected += 1;
            self.has_seq = true;
            self.last_seq = seq;
            return;
        }
        let adv = seq.wrapping_sub(self.last_seq) & 0x0FFF;
        if adv == 0 {
            self.seq_expected += 1; // retry: one delivered frame
        } else if adv < 2048 {
            self.seq_expected += if adv > MAX_SEQ_GAP { 1 } else { u64::from(adv) };
            self.last_seq = seq;
        }
    }
}

/// Every type the GS sends, heard back on its own capture: RCF, DISC, NACK,
/// and its own (tagged) T_MAV; the drone's T_MAV is untagged. A self frame
/// carries the GS's seq counter: let into the walk, one landing ahead of the
/// drone's mark stalls the card's expected count while received climbs.
fn is_self_frame(m: &RxBody, rc_t: Option<u8>) -> bool {
    m.crc_ok
        && match rc_t {
            Some(rc::T_RCF | rc::T_DISC | rc::T_NACK) => true,
            Some(rc::T_MAV) => rc::parse_mav(&m.body, true).is_some(),
            _ => false,
        }
}

/// The RF class an FCS-good frame folds into: control, MSP, probe, base or
/// enhancement; None for any other stream id.
fn rf_class(rc_t: Option<u8>, stream_id: Option<u8>) -> Option<usize> {
    if rc_t.is_some() {
        return Some(RF_CTRL);
    }
    match stream_id {
        Some(sbi::MSP_STREAM_ID) => Some(RF_MSP),
        Some(sbi::PROBE_STREAM_ID) => Some(RF_PROBE),
        Some(s) if s < 2 => Some(usize::from(s)),
        _ => None,
    }
}

pub enum Route {
    /// A body from an unknown card.
    Dropped,
    /// An RC frame (CRC not checked for foreign frames; own RCF/DISC only
    /// when CRC-good, as mabur).
    Rc,
    Msp,
    Probe,
    Video(Vec<DecodedFrag>),
}

pub struct Aggregator {
    dec: UepDecoder,
    cards: Vec<CardTrack>,
    bad_card_msgs: u64,
    last_video_us: u64,
    last_video_seq: u16,
}

impl Aggregator {
    pub fn new(
        layers: &[UepLayerCfg; 2],
        seq_horizon: u32,
        n_cards: usize,
        arrival_guard: u32,
    ) -> Self {
        Self {
            dec: UepDecoder::new(layers, seq_horizon, arrival_guard),
            cards: vec![CardTrack::default(); n_cards],
            bad_card_msgs: 0,
            last_video_us: 0,
            last_video_seq: 0,
        }
    }
    pub fn decoder(&self) -> &UepDecoder {
        &self.dec
    }
    pub fn decoder_mut(&mut self) -> &mut UepDecoder {
        &mut self.dec
    }
    pub fn cards(&self) -> &[CardTrack] {
        &self.cards
    }
    pub fn bad_card_msgs(&self) -> u64 {
        self.bad_card_msgs
    }
    pub fn last_video_us(&self) -> u64 {
        self.last_video_us
    }
    pub fn last_video_seq(&self) -> u16 {
        self.last_video_seq
    }

    pub fn on_rx_body(&mut self, m: &RxBody) -> Route {
        let Some(c) = self.cards.get_mut(usize::from(m.card_id)) else {
            self.bad_card_msgs += 1;
            return Route::Dropped;
        };
        let rc_t = rc::frame_type(&m.body);
        if is_self_frame(m, rc_t) {
            // Our own control frames heard back.
            c.self_frames += 1;
            c.rc_frames += 1;
            return Route::Rc;
        }
        let stream_id = sbi::peek_stream_id(&m.body);
        c.count_frame(m);
        if m.crc_ok {
            c.on_clean_frame(m, rf_class(rc_t, stream_id));
            self.last_video_seq = m.mac_seq;
        }
        if rc_t.is_some() {
            c.rc_frames += 1;
            return Route::Rc;
        }
        // An FCS-bad body goes where its SBI header says; its CRC-good
        // sub-blocks are salvaged there.
        match stream_id {
            Some(sbi::MSP_STREAM_ID) => return Route::Msp,
            Some(sbi::PROBE_STREAM_ID) => return Route::Probe,
            _ => {}
        }
        c.video_bodies += 1;
        self.last_video_us = m.mono_us;
        Route::Video(
            self.dec
                .add_body(&m.body, m.mono_us / 1000, m.mcs, m.mono_us, m.crc_ok),
        )
    }
}

#[cfg(test)]
mod tests {
    //! mabur `test_aggregator.cpp`: the GS's own uplink stays out of the seq walk.
    use super::*;
    use pal_wire::rc::{Mav, Nack, TagCtx, DEFAULT_LINK_KEY};

    fn agg() -> Aggregator {
        Aggregator::new(&crate::config::Config::default().uep_layers(), 512, 1, 0)
    }
    fn msg(seq: u16, body: Vec<u8>) -> RxBody {
        RxBody {
            crc_ok: true,
            mac_seq: seq,
            body,
            ..Default::default()
        }
    }
    fn walk(a: &Aggregator) -> (u64, u64, u64) {
        let c = &a.cards()[0];
        (c.self_frames, c.seq_received, c.seq_expected)
    }

    /// mabur-ground `crc_failed_probe_body_is_delivered_for_salvage`: an
    /// FCS-bad body is routed by the stream its header names, whatever was
    /// heard clean before.
    #[test]
    fn crc_failed_body_is_routed_by_its_header() {
        let body = |sid: u8| {
            pal_wire::sbi::Packer::new(16, 1, sid)
                .add(&[0x5a; 16])
                .unwrap()
        };
        let at = |ms: u64, crc_ok: bool, sid: u8| RxBody {
            crc_ok,
            mono_us: ms * 1000,
            mac_seq: ms as u16,
            body: body(sid),
            ..Default::default()
        };
        let mut a = agg();
        a.on_rx_body(&at(10, true, 0)); // base heard clean
        assert!(matches!(
            a.on_rx_body(&at(11, false, sbi::PROBE_STREAM_ID)),
            Route::Probe
        ));
        assert!(matches!(
            a.on_rx_body(&at(12, false, sbi::MSP_STREAM_ID)),
            Route::Msp
        ));
        // enh never heard clean: routed all the same.
        assert!(matches!(a.on_rx_body(&at(13, false, 1)), Route::Video(_)));
        assert_eq!(a.cards()[0].crc_fail, 3);
    }

    #[test]
    fn gs_nack_kept_out_of_the_seq_walk() {
        let mut a = agg();
        let mut n = Nack {
            counter: 3,
            n: 1,
            ..Default::default()
        };
        n.e[0].first_seq = 40;
        let nack = rc::pack_nack(&n, &DEFAULT_LINK_KEY, &TagCtx::default());
        a.on_rx_body(&msg(10, vec![0; 20]));
        assert!(matches!(a.on_rx_body(&msg(1500, nack)), Route::Rc));
        a.on_rx_body(&msg(11, vec![0; 20]));
        a.on_rx_body(&msg(12, vec![0; 20]));
        assert_eq!(walk(&a), (1, 3, 3));
    }

    #[test]
    fn gs_tagged_mav_is_self_drone_mav_walks() {
        let mut a = agg();
        let m = Mav {
            counter: 5,
            payload: vec![0xFD, 1, 2, 3],
        };
        let k = (&DEFAULT_LINK_KEY, &TagCtx::default());
        a.on_rx_body(&msg(10, vec![0; 20]));
        a.on_rx_body(&msg(1500, rc::pack_mav(&m, true, k.0, k.1)));
        a.on_rx_body(&msg(11, rc::pack_mav(&m, false, k.0, k.1)));
        a.on_rx_body(&msg(12, vec![0; 20]));
        assert_eq!(walk(&a), (1, 3, 3));
    }
}
