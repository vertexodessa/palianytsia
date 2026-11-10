//! The contract every ground-station core implements: the Rust core (the
//! product) and `pal-ref` (mabur's C++ WebGs, the test oracle). Both are fed
//! the same bodies on the same clock and must produce the same AUs and the
//! same sent bytes.
//!
//! Single-threaded by contract, like WebGs: `on_rx`, `tick` and `stats` from
//! one thread, one clock (monotonic microseconds) for RX stamps and ticks.

pub mod aggregator;
pub mod cal;
pub mod chan;
pub mod config;
pub mod frame_stream;
pub mod gap;
pub mod json;
pub mod ladder;
pub mod link_health;
pub mod mav;
pub mod nack;
pub mod refresh;
pub mod rust_core;
pub mod slice_asm;
pub mod tx_selector;
pub mod uep;
pub mod uplink;
pub mod vrx;

/// One received 802.11 body with its RX metadata. Mirrors
/// `mabur::node::RxBody` field for field.
#[derive(Clone, Debug, Default)]
pub struct RxBody {
    pub card_id: u8,
    /// Core clock, microseconds.
    pub mono_us: u64,
    /// Per chain, dBm = value - 110. Valid only when `phy_valid`.
    pub rssi: [u8; 2],
    pub snr: [i8; 2],
    /// Raw half-dB, 0 = not sampled.
    pub evm: [i8; 2],
    pub phy_valid: bool,
    /// 802.11 FCS result; corrupt frames still carry bodies for salvage.
    pub crc_ok: bool,
    pub mac_seq: u16,
    pub tsfl: u32,
    /// HT MCS from the RX descriptor, 255 = unknown.
    pub mcs: u8,
    /// Channel the body was received on, 0 = unknown.
    pub rx_channel: u8,
    /// dot11 header stripped.
    pub body: Vec<u8>,
}

/// One access unit out of the core.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Au {
    /// Annex-B, FrameHdr stripped.
    pub data: Vec<u8>,
    /// Drone capture stamp (u32, wraps).
    pub pts_us: u32,
    /// 0 base, 1 enh.
    pub sid: u8,
    /// Frame-wire flags: 0x01 IDR, 0x02 DISCONT.
    pub flags: u8,
    pub complete: bool,
    /// Rebuilt from complete slices plus skip fills; never a sync point.
    pub salvaged: bool,
    /// Whole AUs FrameStream skipped right before this one (webgs::Au,
    /// AuLatMeta::skipped_before).
    pub skipped_before: u32,
    /// NAL type of the first slice; 0 when the AU carries none.
    pub vcl_nal: u8,
}

/// What a core is started with; the same for the Rust core and mabur's.
#[derive(Clone, Copy, Debug)]
pub struct CoreOpts {
    pub mode: Mode,
    /// false = fixed `video.frame_gap_timeout_ms` (parity with maburgs
    /// --dry-run).
    pub adaptive_gap: bool,
    /// Pinned rendezvous vrx_nonce and RCF tags (replay); 0 = random.
    pub rz_nonce: u32,
    /// The channel a DISC proposes; 0 = the first of `radio.channels`.
    pub channel: u8,
}

impl Default for CoreOpts {
    fn default() -> Self {
        Self {
            mode: Mode::Spotter,
            adaptive_gap: true,
            rz_nonce: 0,
            channel: 0,
        }
    }
}

/// The rendezvous nonce every app pins ("PAL1"): replays stay comparable.
pub const RZ_NONCE: u32 = 0x5041_4c31;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Mode {
    /// Drives the link: rendezvous, ladder, RCFs.
    Gs,
    /// Receive only; no transmit path by construction.
    #[default]
    Spotter,
}

/// What a core hands back to its host. Called from inside `on_rx` and
/// `tick`, never from another thread.
pub trait CoreIo {
    fn on_au(&mut self, au: Au);
    /// One RC body to wrap and transmit. Gs mode only.
    fn send(&mut self, rc_body: &[u8]);
    /// One whole MAVLink 2 packet from the drone (T_MAV tunnel). Gs mode only.
    fn on_mav(&mut self, _pkt: &[u8]) {}
}

pub trait GsCore {
    fn on_rx(&mut self, body: &RxBody, io: &mut dyn CoreIo);
    fn tick(&mut self, now_us: u64, io: &mut dyn CoreIo);
    /// One line of JSON, the `webgs::stats_json` shape.
    fn stats_json(&self) -> String;
    /// `(mcs, bw)` of the `stats_json` line; mcs is -1 while none is known.
    fn link_op(&self) -> (i32, i32);
    /// Bytes from a local MAVLink peer (a GCS), cut at any boundary; tunneled
    /// to the drone under the live session. Gs mode only.
    fn mav_push_outbound(&mut self, _bytes: &[u8]) {}
    /// IDR requests so far (WebGs::set_idr_requests): the RCF's idr_epoch
    /// byte becomes `n as u8`; the drone serves one IDR per change.
    fn set_idr_requests(&mut self, _n: u32) {}
    /// The drone parameter table (F18), `webgs::params_json` at `now_us` on
    /// the core clock: one JSON object, rows in table order.
    fn params_json(&self, _now_us: u64) -> String {
        String::new()
    }
    /// Set one row from its text form (WebGs::param_set); `Err(why)` on a
    /// refusal ("not linked", "apply in progress", "bad value for int", ...).
    fn param_set(&mut self, _row: usize, _text: &str) -> Result<(), String> {
        Err("not linked".into())
    }
    /// Re-list the drone's parameters (WebGs::param_refresh).
    fn param_refresh(&mut self) {}
    /// Why the ladder is not stepping up, as one JSON object (rung, s1
    /// clean run, probe streak, penalty, last change). UI only: WebGs has no
    /// such view, so the oracle returns None and the m1 parity lines never
    /// carry it.
    fn ladder_json(&self) -> Option<String> {
        None
    }
    /// Start a TX-power calibration run (maburcal start): `sweeps` empty =
    /// the defaults for the effective width. Video stops for the run and
    /// the drone's walls are rewritten; `Err(why)` on a refusal.
    fn cal_start(&mut self, _sweeps: &[pal_wire::cal::SweepSpec]) -> Result<(), String> {
        Err("calibration is not supported by this core".into())
    }
    /// Abandon the run; the drone restores its power table at once.
    fn cal_abort(&mut self) {}
    /// The operator's channel choice on a live link (`radio.use`, see
    /// `ChannelCore::set_usable`); `Err(why)` when this core cannot.
    fn set_channels(&mut self, _chosen: &[u8]) -> Result<(), String> {
        Err("channel choice needs the Rust channel core".into())
    }
    /// Start the operator's channel scan over `set` (empty = the drone's
    /// set); its view comes back through `chan_scan_json`.
    fn chan_scan(&mut self, _set: &[u8]) -> Result<(), String> {
        Err("channel scan needs the Rust channel core".into())
    }
    /// The scan's state and rows as one JSON object; UI only, like
    /// `ladder_json`.
    fn chan_scan_json(&self) -> Option<String> {
        None
    }
    /// The channel core's records since the last call (hop steps, plan
    /// moves, verdict changes, scout dwells, picks), one JSON object each
    /// with the event name in `ev`; for the flight log only. mabur's GS
    /// writes them to scan.log, which the oracle has no home for: empty.
    fn take_chan_records(&mut self) -> Vec<String> {
        Vec::new()
    }
    /// Loss episodes closed since the last call, one fec.log row each in
    /// mabur's `feclog 4` text: `t_ms sid mcs bw ov first_seq span missing
    /// recovered retx abandoned stale repairs window`. The oracle's shim
    /// does not expose its episodes: empty.
    fn take_fec_rows(&mut self) -> Vec<String> {
        Vec::new()
    }
    /// The run's state and result table as one JSON object (`cal::status_json`).
    fn cal_json(&self) -> Option<String> {
        None
    }
    /// Where cal.log goes (the flight-log directory); unset = no log.
    fn set_cal_log_dir(&mut self, _dir: &std::path::Path) {}
}
