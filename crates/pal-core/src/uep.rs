//! Receiver side of the two UEP layers (mabur `common/src/uep_decoder.cpp`):
//! route a body by its SBI stream id to that layer's unpack + SwDecoder and
//! hand out the FRAG fragments that come out. Duplicate bodies from several
//! cards need no dedup: seq identity makes them idempotent.
use crate::config::UepLayerCfg;
use pal_fec::decoder::DecoderStats;
use pal_fec::{Boundary, LossEpisode, SourceState, SwConfig, SwDecoder};
use pal_wire::{frame_wire::FRAG_HDR_LEN, sbi, sw};

pub const MCS_UNKNOWN: u8 = 255;
/// A transition boundary older than this is force-closed: a straggler this
/// old is not transition debris.
const BOUNDARY_EXPIRY_MS: u64 = 1000;

#[derive(Clone, Debug, Default)]
pub struct DecodedFrag {
    pub stream_id: u8,
    pub frag: Vec<u8>,
    /// RX stamp of the body whose arrival emitted this fragment (a repair
    /// carries the completing body's stamp).
    pub body_mono_us: u64,
    /// SBI q_ms / enc_us / air_ms; 0 from an FCS-corrupt body (the header
    /// sits outside the per-block CRCs).
    pub q_ms: u16,
    pub enc_us: u16,
    pub air_ms: u16,
    pub sw_seq: u32,
    pub retx: bool,
}

#[derive(Clone, Debug, Default)]
pub struct LayerStats {
    pub bodies: u64,
    pub subblocks_failed: u64,
    pub bodies_corrupt: u64,
    pub subblocks_salvaged: u64,
    pub dec: DecoderStats,
    pub rows_in_flight: usize,
}

struct Layer {
    env_size: usize,
    sw: SwDecoder,
    bodies: u64,
    subblocks_failed: u64,
    bodies_corrupt: u64,
    subblocks_salvaged: u64,
    cur_mcs: u8,
    bnd_armed: bool,
    bnd_open: bool,
    bnd_arm_ms: u64,
    bnd_close_ms: f64,
}

pub struct UepDecoder {
    layers: [Layer; 2],
    bodies_misrouted: u64,
}

impl UepDecoder {
    pub fn new(layers: &[UepLayerCfg; 2], seq_horizon: u32, arrival_guard: u32) -> Self {
        let mk = |c: &UepLayerCfg| Layer {
            env_size: sw::HEADER_LEN + c.symbol_size,
            sw: SwDecoder::new(
                SwConfig {
                    symbol_size: c.symbol_size,
                    window: c.window,
                    overhead: 0.5,
                },
                seq_horizon,
                arrival_guard,
            ),
            bodies: 0,
            subblocks_failed: 0,
            bodies_corrupt: 0,
            subblocks_salvaged: 0,
            cur_mcs: MCS_UNKNOWN,
            bnd_armed: false,
            bnd_open: false,
            bnd_arm_ms: 0,
            bnd_close_ms: -1.0,
        };
        Self {
            layers: [mk(&layers[0]), mk(&layers[1])],
            bodies_misrouted: 0,
        }
    }

    /// The GS commanded stream `sid` onto a new op at `new_mcs`.
    pub fn mark_transition(&mut self, sid: usize, new_mcs: u8, now_ms: u64) {
        let Some(l) = self.layers.get_mut(sid) else {
            return;
        };
        let prev = l.cur_mcs;
        l.cur_mcs = new_mcs;
        l.bnd_armed = true;
        l.bnd_arm_ms = now_ms;
        l.bnd_open = prev != MCS_UNKNOWN && new_mcs != MCS_UNKNOWN && new_mcs != prev;
        l.sw.mark_transition();
        if !l.bnd_open {
            l.sw.close_boundary();
        }
    }

    pub fn add_body(
        &mut self,
        body: &[u8],
        now_ms: u64,
        rx_mcs: u8,
        body_mono_us: u64,
        crc_ok: bool,
    ) -> Vec<DecodedFrag> {
        let sid = match sbi::peek_stream_id(body) {
            Some(s) if s <= 1 => s,
            _ => {
                self.bodies_misrouted += 1;
                return Vec::new();
            }
        };
        let l = &mut self.layers[usize::from(sid)];
        l.bodies += 1;
        let mut hint = Boundary::None;
        if l.bnd_armed {
            if now_ms > l.bnd_arm_ms && now_ms - l.bnd_arm_ms > BOUNDARY_EXPIRY_MS {
                l.bnd_armed = false;
                l.bnd_open = false;
                l.sw.close_boundary();
            } else if rx_mcs != MCS_UNKNOWN && l.cur_mcs != MCS_UNKNOWN {
                hint = if rx_mcs == l.cur_mcs {
                    Boundary::Post
                } else {
                    Boundary::Pre
                };
            }
        }
        // Retransmits are boundary-neutral.
        if body[3] & sbi::RETX_MARK != 0 {
            hint = Boundary::None;
        }
        if hint == Boundary::Post && l.bnd_open {
            l.bnd_open = false;
            if l.bnd_arm_ms <= now_ms {
                l.bnd_close_ms = (now_ms - l.bnd_arm_ms) as f64;
            }
        }
        let r = sbi::unpack(body, l.env_size);
        l.subblocks_failed += r.n_failed as u64;
        if !crc_ok {
            l.bodies_corrupt += 1;
            l.subblocks_salvaged += r.survivors.len() as u64;
        }
        let mut out = Vec::new();
        for env in &r.survivors {
            let pkts = l.sw.add_symbol(env, now_ms, hint, crc_ok, r.retx);
            let seqs = l.sw.last_out_seqs();
            for (k, pkt) in pkts.into_iter().enumerate() {
                if pkt.len() < FRAG_HDR_LEN {
                    continue;
                }
                let seq = seqs.get(k).copied().unwrap_or(0);
                out.push(DecodedFrag {
                    stream_id: sid,
                    frag: pkt,
                    body_mono_us,
                    q_ms: if crc_ok { r.q_ms } else { 0 },
                    enc_us: if crc_ok { r.enc_us } else { 0 },
                    air_ms: if crc_ok { r.air_ms } else { 0 },
                    sw_seq: seq,
                    retx: r.retx,
                });
            }
        }
        out
    }

    pub fn newest_seq(&self, sid: usize) -> u64 {
        self.layers.get(sid).map_or(0, |l| l.sw.newest_seq())
    }
    pub fn repair_window(&self, sid: usize) -> usize {
        self.layers.get(sid).map_or(0, |l| l.sw.repair_window())
    }
    pub fn missing_sources(&self, sid: usize, lookback: u32) -> Vec<u32> {
        self.layers
            .get(sid)
            .map_or_else(Vec::new, |l| l.sw.missing_sources(lookback))
    }
    pub fn source_state(&self, sid: usize, wire_seq: u32) -> SourceState {
        self.layers
            .get(sid)
            .map_or(SourceState::Unknown, |l| l.sw.source_state(wire_seq))
    }
    pub fn take_episodes(&mut self, sid: usize) -> Vec<LossEpisode> {
        self.layers
            .get_mut(sid)
            .map_or_else(Vec::new, |l| l.sw.take_episodes())
    }
    pub fn last_boundary_close_ms(&self, sid: usize) -> f64 {
        self.layers.get(sid).map_or(-1.0, |l| l.bnd_close_ms)
    }
    /// Session change: drop boundary state (the decoder re-anchors on its own).
    pub fn reset_continuity(&mut self) {
        for l in &mut self.layers {
            l.bnd_armed = false;
            l.bnd_open = false;
            l.cur_mcs = MCS_UNKNOWN;
            l.sw.close_boundary();
        }
    }
    pub fn stats(&self, sid: usize) -> LayerStats {
        let l = &self.layers[sid];
        LayerStats {
            bodies: l.bodies,
            subblocks_failed: l.subblocks_failed,
            bodies_corrupt: l.bodies_corrupt,
            subblocks_salvaged: l.subblocks_salvaged,
            dec: *l.sw.stats(),
            rows_in_flight: l.sw.rows_in_flight(),
        }
    }
    pub fn bodies_misrouted(&self) -> u64 {
        self.bodies_misrouted
    }
}
