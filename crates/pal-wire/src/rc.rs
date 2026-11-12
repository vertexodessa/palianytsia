//! RC control plane, RC_VERSION 18 (mabur `rc_proto.cpp`): RCF, DISC,
//! DISC_ACK, TELEM, NACK, MAV. Little-endian; every frame ends with a CRC16
//! over the bytes before it. Tagged frames (DISC/RCF/NACK, GS->drone MAV)
//! carry an 8-byte SipHash tag before the CRC, keyed by the link key and
//! hashed over the frame plus a never-sent TagCtx.
use crate::crc16::crc16_ccitt;
use crate::siphash::siphash24;
use alloc::vec::Vec;

pub const RC_MAGIC: u16 = 0x5243;
pub const RC_VERSION: u8 = 18;
pub const NO_PROBE_PROFILE: u8 = 0xFF;
pub const REC_ON: u8 = 0x01;
pub const REC_KNOWN: u8 = 0x02;
pub const SNR_OFF_RANGE_MFPS: i16 = 500;

pub const T_RCF: u8 = 1;
pub const T_DISC: u8 = 2;
pub const T_DISC_ACK: u8 = 3;
pub const T_TELEM: u8 = 4;
pub const T_NACK: u8 = 7;
pub const T_MAV: u8 = 8;
pub const F_DISCOVERY: u8 = 0x04;
pub const TAG_LEN: usize = 8;
pub const ACK_KEY_MISMATCH: u8 = 0x01;
pub const TELEM_AUTH_REJECT: u8 = 0x02;

pub const CAP_FRAME_WIRE: u16 = 0x0001;
pub const CAP_TELEMETRY: u16 = 0x0002;
pub const CAP_CALIBRATE: u16 = 0x0004;
pub const CAP_MAVLINK: u16 = 0x0008;
/// The drone's calibration runs the floor sweep (cal kind 4) and tunes
/// itself to 40 MHz for a run's 40 MHz sweeps, back at the run's end
/// (mabur rc_proto.h). A bit, not an RC_VERSION bump: an older drone still
/// calibrates, it is just never sent kind 4 (its decoder would take it for
/// a power sweep) and calibrates 40 MHz only while both ends fly it.
pub const CAP_CAL_WIDE: u16 = 0x0010;
/// Disc.cap_bits (GS -> drone): op_width is a proposal the drone retunes to.
pub const DISC_CAP_WIDTH: u16 = 0x0001;

pub type LinkKey = [u8; 16];
pub const DEFAULT_LINK_KEY: LinkKey = *b"mabur-default-00";

const RCF_HEAD_LEN: usize = 17;
const DISC_LEN: usize = 17;
const DISC_ACK_LEN: usize = 20;
const TELEM_LEN: usize = 54;
const NACK_FIXED_LEN: usize = 11;
const NACK_ENTRY_LEN: usize = 8;
pub const MAV_FIXED_LEN: usize = 10;
pub const MAV_MAX_PAYLOAD: usize = 600;
pub const NACK_FLAG_REPEAT: u8 = 0x01;
pub const MAX_NACK_ENTRIES: usize = 4;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TagCtx {
    pub vrx_nonce: u32,
    pub vtx_nonce: u32,
    pub seq32: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rcf {
    pub seq: u16,
    pub profile: u8,
    pub fec_overhead_base: f64,
    pub fec_overhead_enh: f64,
    pub probe_profile: u8,
    pub hop_ch: u8,
    pub hop_epoch: u8,
    pub rec: u8,
    pub idr_epoch: u8,
    pub snr_off: i16,
}

impl Default for Rcf {
    fn default() -> Self {
        Self {
            seq: 0,
            profile: 0,
            fec_overhead_base: 0.5,
            fec_overhead_enh: 0.5,
            probe_profile: NO_PROBE_PROFILE,
            hop_ch: 0,
            hop_epoch: 0,
            rec: 0,
            idr_epoch: 0,
            snr_off: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Disc {
    pub vrx_nonce: u32,
    pub op_channel: u8,
    pub op_width: u8,
    pub table_ver: u8,
    pub init_profile: u8,
    pub cap_bits: u16,
    pub seq: u16,
}

impl Default for Disc {
    fn default() -> Self {
        Self {
            vrx_nonce: 0,
            op_channel: 0,
            op_width: 20,
            table_ver: 1,
            init_profile: 0,
            cap_bits: 0,
            seq: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DiscAck {
    pub vrx_nonce: u32,
    pub vtx_nonce: u32,
    pub chip_caps: u16,
    pub agreed_channel: u8,
    pub agreed_width: u8,
    pub flags: u8,
    pub seq: u16,
}

impl Default for DiscAck {
    fn default() -> Self {
        Self {
            vrx_nonce: 0,
            vtx_nonce: 0,
            chip_caps: 0,
            agreed_channel: 0,
            agreed_width: 20,
            flags: 0,
            seq: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Telem {
    pub tlm_seq: u16,
    pub state: u8,
    /// bit0 failsafe_shed, bit1 auth_reject, bit3 rcf_seq_echo valid,
    /// bit4 congestion_shed, bit7 low_power.
    pub flags: u8,
    pub rcf_age_ms: u16,
    pub rcf_seq_echo: u16,
    pub pts_at_build: u64,
    pub rcf_rx: u32,
    pub cmd_kbps: u16,
    pub txq_drops: u32,
    pub txq_wait_max_ms: u16,
    pub usb_fail: u16,
    pub up_rssi: [u8; 2],
    pub up_snr: [i8; 2],
    pub soc_temp_c: i8,
    pub cpu_busy_x100: u16,
    pub rx_own: u16,
    pub rx_foreign: u16,
    pub rx_crcfail: u16,
    pub rec_status: u8,
    pub nack_rx: u16,
    pub retx_syms: u16,
    pub retx_refused: u16,
}

impl Default for Telem {
    fn default() -> Self {
        Self {
            tlm_seq: 0,
            state: 0,
            flags: 0,
            rcf_age_ms: 0,
            rcf_seq_echo: 0,
            pts_at_build: 0,
            rcf_rx: 0,
            cmd_kbps: 0,
            txq_drops: 0,
            txq_wait_max_ms: 0,
            usb_fail: 0,
            up_rssi: [0; 2],
            up_snr: [0; 2],
            soc_temp_c: -128,
            cpu_busy_x100: 65535,
            rx_own: 0,
            rx_foreign: 0,
            rx_crcfail: 0,
            rec_status: 0,
            nack_rx: 0,
            retx_syms: 0,
            retx_refused: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NackEntry {
    pub first_seq: u32,
    /// bit i = wire seq first_seq + i requested; bit 0 always set on the wire.
    pub bitmap: u32,
}

impl Default for NackEntry {
    fn default() -> Self {
        Self {
            first_seq: 0,
            bitmap: 1,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Nack {
    pub counter: u32,
    pub sid: u8,
    pub flags: u8,
    pub n: u8,
    pub e: [NackEntry; MAX_NACK_ENTRIES],
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Mav {
    pub counter: u32,
    pub payload: Vec<u8>,
}

fn g16(b: &[u8], o: usize) -> u16 {
    u16::from_le_bytes([b[o], b[o + 1]])
}
fn g32(b: &[u8], o: usize) -> u32 {
    u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}
fn g64(b: &[u8], o: usize) -> u64 {
    u64::from(g32(b, o)) | (u64::from(g32(b, o + 4)) << 32)
}

fn head(out: &mut Vec<u8>, ty: u8) {
    out.extend_from_slice(&RC_MAGIC.to_le_bytes());
    out.push(RC_VERSION);
    out.push(ty);
}

fn put_crc(b: &mut Vec<u8>) {
    let c = crc16_ccitt(b);
    b.extend_from_slice(&c.to_le_bytes());
}

fn tag(bytes: &[u8], key: &LinkKey, ctx: &TagCtx) -> u64 {
    let mut m = Vec::with_capacity(bytes.len() + 12);
    m.extend_from_slice(bytes);
    m.extend_from_slice(&ctx.vrx_nonce.to_le_bytes());
    m.extend_from_slice(&ctx.vtx_nonce.to_le_bytes());
    m.extend_from_slice(&ctx.seq32.to_le_bytes());
    siphash24(key, &m)
}

fn put_tag(b: &mut Vec<u8>, key: &LinkKey, ctx: &TagCtx) {
    let t = tag(b, key, ctx);
    b.extend_from_slice(&t.to_le_bytes());
}

fn check(b: &[u8], ty: u8, crc_at: usize) -> bool {
    b.len() >= crc_at + 2
        && g16(b, 0) == RC_MAGIC
        && b[2] == RC_VERSION
        && b[3] == ty
        && g16(b, crc_at) == crc16_ccitt(&b[..crc_at])
}

/// Fractional overhead to the wire's x100 byte, rounded, clamped 5..200.
pub fn overhead_to_x100(ov: f64) -> u8 {
    libm_round(ov.clamp(0.05, 2.0) * 100.0) as u8
}

// no_std: f64::round lives in std; this is round-half-away-from-zero for
// the non-negative range used here.
fn libm_round(x: f64) -> f64 {
    let t = x as i64 as f64;
    if x - t >= 0.5 {
        t + 1.0
    } else {
        t
    }
}

pub fn pack_rcf(r: &Rcf, key: &LinkKey, ctx: &TagCtx) -> Vec<u8> {
    let mut b = Vec::with_capacity(RCF_HEAD_LEN + TAG_LEN + 2);
    head(&mut b, T_RCF);
    b.push(0);
    b.extend_from_slice(&r.seq.to_le_bytes());
    b.push(r.profile);
    b.push(overhead_to_x100(r.fec_overhead_base));
    b.push(overhead_to_x100(r.fec_overhead_enh));
    b.push(r.probe_profile);
    b.push(r.hop_ch);
    b.push(r.hop_epoch);
    b.push(r.rec);
    b.push(r.idr_epoch);
    b.extend_from_slice(&(r.snr_off as u16).to_le_bytes());
    put_tag(&mut b, key, ctx);
    put_crc(&mut b);
    b
}

pub fn parse_rcf(b: &[u8]) -> Option<Rcf> {
    if !check(b, T_RCF, RCF_HEAD_LEN + TAG_LEN) {
        return None;
    }
    Some(Rcf {
        seq: g16(b, 5),
        profile: b[7],
        fec_overhead_base: f64::from(b[8]) / 100.0,
        fec_overhead_enh: f64::from(b[9]) / 100.0,
        probe_profile: b[10],
        hop_ch: b[11],
        hop_epoch: b[12],
        rec: b[13],
        idr_epoch: b[14],
        snr_off: g16(b, 15) as i16,
    })
}

/// Tag ctx all-zero: pre-rendezvous.
pub fn pack_disc(d: &Disc, key: &LinkKey) -> Vec<u8> {
    let mut b = Vec::with_capacity(DISC_LEN + TAG_LEN + 2);
    head(&mut b, T_DISC);
    b.push(F_DISCOVERY);
    b.extend_from_slice(&d.vrx_nonce.to_le_bytes());
    b.push(d.op_channel);
    b.push(d.op_width);
    b.push(d.table_ver);
    b.push(d.init_profile);
    b.extend_from_slice(&d.cap_bits.to_le_bytes());
    b.extend_from_slice(&d.seq.to_le_bytes());
    put_tag(&mut b, key, &TagCtx::default());
    put_crc(&mut b);
    b
}

pub fn parse_disc(b: &[u8]) -> Option<Disc> {
    if !check(b, T_DISC, DISC_LEN + TAG_LEN) {
        return None;
    }
    Some(Disc {
        vrx_nonce: g32(b, 5),
        op_channel: b[9],
        op_width: b[10],
        table_ver: b[11],
        init_profile: b[12],
        cap_bits: g16(b, 13),
        seq: g16(b, 15),
    })
}

pub fn pack_disc_ack(a: &DiscAck) -> Vec<u8> {
    let mut b = Vec::with_capacity(DISC_ACK_LEN + 2);
    head(&mut b, T_DISC_ACK);
    b.push(F_DISCOVERY);
    b.extend_from_slice(&a.vrx_nonce.to_le_bytes());
    b.extend_from_slice(&a.vtx_nonce.to_le_bytes());
    b.extend_from_slice(&a.chip_caps.to_le_bytes());
    b.push(a.agreed_channel);
    b.push(a.agreed_width);
    b.push(a.flags);
    b.extend_from_slice(&a.seq.to_le_bytes());
    put_crc(&mut b);
    b
}

pub fn parse_disc_ack(b: &[u8]) -> Option<DiscAck> {
    if !check(b, T_DISC_ACK, DISC_ACK_LEN) {
        return None;
    }
    Some(DiscAck {
        vrx_nonce: g32(b, 5),
        vtx_nonce: g32(b, 9),
        chip_caps: g16(b, 13),
        agreed_channel: b[15],
        agreed_width: b[16],
        flags: b[17],
        seq: g16(b, 18),
    })
}

pub fn pack_telem(t: &Telem) -> Vec<u8> {
    let mut b = Vec::with_capacity(TELEM_LEN + 2);
    head(&mut b, T_TELEM);
    b.push(t.flags);
    b.extend_from_slice(&t.tlm_seq.to_le_bytes());
    b.push(t.state);
    b.extend_from_slice(&t.rcf_age_ms.to_le_bytes());
    b.extend_from_slice(&t.rcf_seq_echo.to_le_bytes());
    b.extend_from_slice(&t.pts_at_build.to_le_bytes());
    b.extend_from_slice(&t.rcf_rx.to_le_bytes());
    b.extend_from_slice(&t.cmd_kbps.to_le_bytes());
    b.extend_from_slice(&t.txq_drops.to_le_bytes());
    b.extend_from_slice(&t.txq_wait_max_ms.to_le_bytes());
    b.extend_from_slice(&t.usb_fail.to_le_bytes());
    b.extend_from_slice(&t.up_rssi);
    b.push(t.up_snr[0] as u8);
    b.push(t.up_snr[1] as u8);
    b.push(t.soc_temp_c as u8);
    b.extend_from_slice(&t.cpu_busy_x100.to_le_bytes());
    b.extend_from_slice(&t.rx_own.to_le_bytes());
    b.extend_from_slice(&t.rx_foreign.to_le_bytes());
    b.extend_from_slice(&t.rx_crcfail.to_le_bytes());
    b.push(t.rec_status);
    b.extend_from_slice(&t.nack_rx.to_le_bytes());
    b.extend_from_slice(&t.retx_syms.to_le_bytes());
    b.extend_from_slice(&t.retx_refused.to_le_bytes());
    put_crc(&mut b);
    b
}

pub fn parse_telem(b: &[u8]) -> Option<Telem> {
    if !check(b, T_TELEM, TELEM_LEN) {
        return None;
    }
    let o = TELEM_LEN - 6;
    Some(Telem {
        flags: b[4],
        tlm_seq: g16(b, 5),
        state: b[7],
        rcf_age_ms: g16(b, 8),
        rcf_seq_echo: g16(b, 10),
        pts_at_build: g64(b, 12),
        rcf_rx: g32(b, 20),
        cmd_kbps: g16(b, 24),
        txq_drops: g32(b, 26),
        txq_wait_max_ms: g16(b, 30),
        usb_fail: g16(b, 32),
        up_rssi: [b[34], b[35]],
        up_snr: [b[36] as i8, b[37] as i8],
        soc_temp_c: b[38] as i8,
        cpu_busy_x100: g16(b, 39),
        rx_own: g16(b, 41),
        rx_foreign: g16(b, 43),
        rx_crcfail: g16(b, 45),
        rec_status: b[47],
        nack_rx: g16(b, o),
        retx_syms: g16(b, o + 2),
        retx_refused: g16(b, o + 4),
    })
}

pub fn pack_nack(n: &Nack, key: &LinkKey, ctx: &TagCtx) -> Vec<u8> {
    let cnt = usize::from(n.n).min(MAX_NACK_ENTRIES);
    let mut b = Vec::with_capacity(NACK_FIXED_LEN + cnt * NACK_ENTRY_LEN + TAG_LEN + 2);
    head(&mut b, T_NACK);
    b.push(n.flags);
    b.extend_from_slice(&n.counter.to_le_bytes());
    b.push(n.sid);
    b.push(cnt as u8);
    for e in &n.e[..cnt] {
        b.extend_from_slice(&e.first_seq.to_le_bytes());
        b.extend_from_slice(&(e.bitmap | 1).to_le_bytes());
    }
    put_tag(&mut b, key, ctx);
    put_crc(&mut b);
    b
}

pub fn parse_nack(b: &[u8]) -> Option<Nack> {
    if b.len() < NACK_FIXED_LEN || g16(b, 0) != RC_MAGIC || b[2] != RC_VERSION || b[3] != T_NACK {
        return None;
    }
    let cnt = usize::from(b[NACK_FIXED_LEN - 1]);
    if cnt == 0 || cnt > MAX_NACK_ENTRIES {
        return None;
    }
    let plen = NACK_FIXED_LEN + cnt * NACK_ENTRY_LEN + TAG_LEN;
    if b.len() < plen + 2 || g16(b, plen) != crc16_ccitt(&b[..plen]) {
        return None;
    }
    let mut n = Nack {
        flags: b[4],
        counter: g32(b, 5),
        sid: b[9],
        n: cnt as u8,
        ..Default::default()
    };
    for i in 0..cnt {
        let o = NACK_FIXED_LEN + i * NACK_ENTRY_LEN;
        n.e[i] = NackEntry {
            first_seq: g32(b, o),
            bitmap: g32(b, o + 4),
        };
    }
    Some(n)
}

/// tagged = GS->drone layout. Payload must be <= MAV_MAX_PAYLOAD.
pub fn pack_mav(m: &Mav, tagged: bool, key: &LinkKey, ctx: &TagCtx) -> Vec<u8> {
    assert!(m.payload.len() <= MAV_MAX_PAYLOAD);
    let mut b = Vec::with_capacity(MAV_FIXED_LEN + m.payload.len() + TAG_LEN + 2);
    head(&mut b, T_MAV);
    b.extend_from_slice(&m.counter.to_le_bytes());
    b.extend_from_slice(&(m.payload.len() as u16).to_le_bytes());
    b.extend_from_slice(&m.payload);
    if tagged {
        put_tag(&mut b, key, ctx);
    }
    put_crc(&mut b);
    b
}

pub fn parse_mav(b: &[u8], tagged: bool) -> Option<Mav> {
    if b.len() < MAV_FIXED_LEN || g16(b, 0) != RC_MAGIC || b[2] != RC_VERSION || b[3] != T_MAV {
        return None;
    }
    let pl = usize::from(g16(b, 8));
    if pl > MAV_MAX_PAYLOAD {
        return None;
    }
    let plen = MAV_FIXED_LEN + pl + if tagged { TAG_LEN } else { 0 };
    if b.len() < plen + 2 || g16(b, plen) != crc16_ccitt(&b[..plen]) {
        return None;
    }
    Some(Mav {
        counter: g32(b, 4),
        payload: b[MAV_FIXED_LEN..MAV_FIXED_LEN + pl].to_vec(),
    })
}

/// Recomputes a tagged frame's tag at its structural offset; bytes past
/// tag + CRC (the 802.11 FCS) are ignored.
pub fn verify_control(b: &[u8], key: &LinkKey, ctx: &TagCtx) -> bool {
    let tag_at = match frame_type(b) {
        Some(T_DISC) => DISC_LEN,
        Some(T_RCF) => RCF_HEAD_LEN,
        Some(T_NACK) => {
            if b.len() < NACK_FIXED_LEN {
                return false;
            }
            let n = usize::from(b[NACK_FIXED_LEN - 1]);
            if n == 0 || n > MAX_NACK_ENTRIES {
                return false;
            }
            NACK_FIXED_LEN + n * NACK_ENTRY_LEN
        }
        Some(T_MAV) => {
            if b.len() < MAV_FIXED_LEN {
                return false;
            }
            let l = usize::from(g16(b, 8));
            if l > MAV_MAX_PAYLOAD {
                return false;
            }
            MAV_FIXED_LEN + l
        }
        _ => return false,
    };
    if b.len() < tag_at + TAG_LEN + 2 {
        return false;
    }
    (tag(&b[..tag_at], key, ctx) ^ g64(b, tag_at)) == 0
}

/// RC frame type without a CRC check; None for short or non-RC bodies.
pub fn frame_type(b: &[u8]) -> Option<u8> {
    if b.len() < 4 || g16(b, 0) != RC_MAGIC || b[2] != RC_VERSION {
        return None;
    }
    Some(b[3])
}

/// `default` for the shipped key, else 4 hex chars of SipHash("mabur.key").
pub fn key_fingerprint(k: &LinkKey) -> alloc::string::String {
    if *k == DEFAULT_LINK_KEY {
        return "default".into();
    }
    let h = siphash24(k, b"mabur.key");
    alloc::format!("{:02x}{:02x}", h & 0xff, (h >> 8) & 0xff)
}
