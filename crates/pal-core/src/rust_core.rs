//! The Rust receive core, wired as mabur's WebGs wires it (`web/src/web_gs.cpp`):
//! Aggregator -> UepDecoder -> FrameStream -> Au. Spotter mode only so far;
//! GS mode (rendezvous, RCF, ladder) is M2.
use crate::aggregator::{Aggregator, Route};
use crate::config::Config;
use crate::frame_stream::{AuLat, FragArrival, FrameOut, FrameStream, FrameStreamCfg};
use crate::gap::{DroneRestartDetector, GapTimeoutPolicy};
use crate::{Au, CoreIo, GsCore, Mode, RxBody};
use pal_wire::frame_wire::FrameHdr;
use pal_wire::{rc, sbi};

#[derive(Clone, Copy, Debug)]
pub struct RustOpts {
    pub mode: Mode,
    /// false = fixed `video.frame_gap_timeout_ms`.
    pub adaptive_gap: bool,
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
}

impl FrameOut for AuOut<'_> {
    fn begin(&mut self, h: &FrameHdr, sid: u8, _nslices: u8) {
        *self.cur = Au {
            pts_us: h.pts_us,
            sid,
            flags: h.flags,
            ..Au::default()
        };
    }
    fn data(&mut self, b: &[u8]) {
        self.cur.data.extend_from_slice(b);
    }
    fn end(&mut self, complete: bool, lat: &AuLat) {
        self.cur.complete = complete;
        self.cur.salvaged = !complete && lat.slice.salvaged;
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
}

impl RustCore {
    pub fn new(cfg: &Config, opts: RustOpts) -> Result<Self, String> {
        if opts.mode == Mode::Gs {
            return Err("rust core: GS mode is not ported yet (M2); use --core ref".into());
        }
        Ok(Self {
            mode: opts.mode,
            opts,
            now_us: 0,
            agg: Aggregator::new(
                &cfg.uep_layers(),
                cfg.fec.seq_horizon,
                1,
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
            // Spotter: no session to gate on, always decode.
            frame_wire: true,
            restart: DroneRestartDetector::default(),
            telem: None,
            air_mcs: -1,
            bw: cfg.radio.width,
            key_fp: rc::key_fingerprint(&cfg.link.key),
            cur: Au::default(),
            c: Counters::default(),
        })
    }

    fn reset_video(&mut self, io: &mut dyn CoreIo) {
        self.agg.decoder_mut().reset_continuity();
        let mut out = AuOut {
            cur: &mut self.cur,
            io,
            c: &mut self.c,
        };
        self.fs.reset(&mut out);
        self.cur = Au::default();
        self.c.resets += 1;
    }

    fn on_rc(&mut self, body: &[u8], us: u64, io: &mut dyn CoreIo) {
        if rc::frame_type(body) != Some(rc::T_TELEM) {
            return; // Spotter: RCF follow and MAV are not wired yet.
        }
        if let Some(t) = rc::parse_telem(body) {
            let seq = t.tlm_seq;
            self.telem = Some(t);
            // A maburd restart (new seqs, frame_ids from 0) is the one
            // discontinuity a spotter can see.
            if self.restart.on_telem(seq, us as f64 / 1000.0) {
                self.reset_video(io);
            }
        }
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
}

impl GsCore for RustCore {
    fn on_rx(&mut self, m: &RxBody, io: &mut dyn CoreIo) {
        self.now_us = self.now_us.max(m.mono_us);
        self.c.bodies += 1;
        match self.agg.on_rx_body(m) {
            Route::Rc => self.on_rc(&m.body, m.mono_us, io),
            Route::Video(frags) if self.frame_wire => {
                let now_ms = self.now_us / 1000;
                let mut out = AuOut {
                    cur: &mut self.cur,
                    io,
                    c: &mut self.c,
                };
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
        if m.crc_ok && sbi::peek_stream_id(&m.body) == Some(0) && m.mcs != 255 {
            self.air_mcs = i32::from(m.mcs);
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
        if self.frame_wire {
            let mut out = AuOut {
                cur: &mut self.cur,
                io,
                c: &mut self.c,
            };
            self.fs.poll(now_ms, &mut out);
        }
    }

    fn stats_json(&self) -> String {
        let card = self.agg.cards().first();
        let num = |v: Option<f64>| v.map_or("null".to_string(), |x| format!("{x:.1}"));
        let rssi = card.filter(|c| c.rf.has_ema).map(|c| c.rf.rssi_ema - 110.0);
        let snr = card.filter(|c| c.rf.has_ema).map(|c| c.rf.snr_ema);
        let fs = self.fs.stats();
        format!(
            concat!(
                "{{\"mode\":\"{}\",\"session\":false,\"rung\":-1,\"mcs\":{},\"bw\":{},",
                "\"key_fp\":\"{}\",\"drone_state\":{},\"snr_db\":{},\"rssi_dbm\":{},",
                "\"bodies\":{},\"aus\":{},\"trunc\":{},\"trunc_base\":{},\"salvaged\":{},",
                "\"sends\":0,\"resets\":{},\"fs_dropped\":{},\"fs_stall_resets\":{},\"slice_fb\":{:?}}}"
            ),
            if self.mode == Mode::Gs { "gs" } else { "spotter" },
            self.air_mcs,
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
            self.c.resets,
            fs.dropped,
            fs.stall_resets,
            fs.slice_fallback,
        )
    }
}
