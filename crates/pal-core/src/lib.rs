//! The contract every ground-station core implements: the Rust core (the
//! product) and `pal-ref` (mabur's C++ WebGs, the test oracle). Both are fed
//! the same bodies on the same clock and must produce the same AUs and the
//! same sent bytes.
//!
//! Single-threaded by contract, like WebGs: `on_rx`, `tick` and `stats` from
//! one thread, one clock (monotonic microseconds) for RX stamps and ticks.

pub mod aggregator;
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
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// Drives the link: rendezvous, ladder, RCFs.
    Gs,
    /// Receive only; no transmit path by construction.
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
    /// Bytes from a local MAVLink peer (a GCS), cut at any boundary; tunneled
    /// to the drone under the live session. Gs mode only.
    fn mav_push_outbound(&mut self, _bytes: &[u8]) {}
    /// IDR requests so far (WebGs::set_idr_requests): the RCF's idr_epoch
    /// byte becomes `n as u8`; the drone serves one IDR per change.
    fn set_idr_requests(&mut self, _n: u32) {}
}
