//! TX-power calibration wire formats (mabur `cal_wire.h`, `cal_proto.h`,
//! `cal_park.h`, `cal_floor.h`, `cal_mav.cpp`): the stamp every sweep frame
//! carries, the six calibration messages of the mabur MAVLink dialect
//! (60000-60005), the verify-pass park rule and the floor sweep's rules
//! both ends share.

use alloc::string::String;
use alloc::vec::Vec;

use crate::mavlink::{self, Framing, Parser};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Kind {
    Power = 1,
    Verify = 2,
    Capacity = 3,
    /// The wide-TX receive-gain floor (40 MHz only; `floor` rules below).
    Floor = 4,
}

impl Kind {
    pub fn from_u8(b: u8) -> Option<Kind> {
        match b {
            1 => Some(Kind::Power),
            2 => Some(Kind::Verify),
            3 => Some(Kind::Capacity),
            4 => Some(Kind::Floor),
            _ => None,
        }
    }
}

pub const FLAG_LDPC: u8 = 1;
pub const FLAG_STBC: u8 = 2;
/// Power and verify frames.
pub const PAYLOAD_LEN: usize = 64;
/// magic(4) kind rate idx bw flags seq_hi seq(2)
pub const HEADER_LEN: usize = 12;
pub const MAGIC: [u8; 4] = *b"MCAL";
/// The chip's 7-bit signed diff range (rc_proto.h kRelMin/kRelMax).
pub const REL_MIN: i32 = -64;
pub const REL_MAX: i32 = 63;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FrameInfo {
    pub kind: Kind,
    /// HT MCS 0..7.
    pub rate: u8,
    /// Relative to the chip's TXAGC anchor, [-64, 63].
    pub idx: i8,
    pub bw: u8,
    pub flags: u8,
    /// 24-bit frame counter.
    pub seq: u32,
    /// Payload bytes the fill matched, any trailing FCS excluded.
    pub len: usize,
}

/// The stamp padded with `0x5A ^ idx` to `len` (>= HEADER_LEN).
pub fn build_payload(
    kind: Kind,
    rate: u8,
    idx: i8,
    bw: u8,
    flags: u8,
    seq: u32,
    len: usize,
) -> Vec<u8> {
    let fill = 0x5A ^ idx as u8;
    let mut p = alloc::vec![fill; len.max(HEADER_LEN)];
    p[..4].copy_from_slice(&MAGIC);
    p[4] = kind as u8;
    p[5] = rate;
    p[6] = idx as u8;
    p[7] = bw;
    p[8] = flags;
    p[9] = (seq >> 16) as u8;
    p[10] = seq as u8;
    p[11] = (seq >> 8) as u8;
    p
}

/// None on anything that is not an intact calibration payload. CRC-bad
/// frames reach this parser; the fill check is what keeps a corrupt frame
/// out of the cell its (possibly garbage) idx names. The body may carry a
/// trailing FCS or not: both readings are tried, no-FCS first.
pub fn parse_payload(p: &[u8]) -> Option<FrameInfo> {
    if p.len() < HEADER_LEN || p[..4] != MAGIC {
        return None;
    }
    let kind = Kind::from_u8(p[4])?;
    let (rate, idx, bw, flags) = (p[5], p[6] as i8, p[7], p[8]);
    if rate > 7 || (bw != 20 && bw != 40) || flags & !(FLAG_LDPC | FLAG_STBC) != 0 {
        return None;
    }
    // idx is an i8, so the [-64, 63] check is the only range left.
    if !(REL_MIN..=REL_MAX).contains(&(idx as i32)) {
        return None;
    }
    let fill = 0x5A ^ p[6];
    let fill_ok = |total: usize| p[HEADER_LEN..total].iter().all(|&b| b == fill);
    let len = if fill_ok(p.len()) {
        p.len()
    } else if p.len() >= HEADER_LEN + 4 && fill_ok(p.len() - 4) {
        p.len() - 4
    } else {
        return None;
    };
    Some(FrameInfo {
        kind,
        rate,
        idx,
        bw,
        flags,
        seq: (u32::from(p[9]) << 16) | u32::from(p[10]) | (u32::from(p[11]) << 8),
        len,
    })
}

// ---- protocol (cal_proto.h) ----

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SweepSpec {
    pub kind: Kind,
    pub bw: u8,
}

impl SweepSpec {
    /// The operator's token for a sweep (maburgs `CalControl::parse_token`):
    /// power20, power40, cap20, cap40, floor (the 40 MHz floor sweep).
    pub fn parse_token(tok: &str) -> Option<SweepSpec> {
        let (kind, bw) = match tok {
            "power20" => (Kind::Power, 20),
            "power40" => (Kind::Power, 40),
            "cap20" => (Kind::Capacity, 20),
            "cap40" => (Kind::Capacity, 40),
            "floor" => (Kind::Floor, 40),
            _ => return None,
        };
        Some(SweepSpec { kind, bw })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SweepCmd {
    pub nonce: u32,
    pub seq: u8,
    pub kind: Kind,
    pub bw: u8,
    pub frames_per_cell: u16,
    pub settle_ms: u16,
    pub gap_us: u16,
    pub dwell_ms: u16,
    pub idx_lo: i8,
    pub idx_hi: i8,
    pub mcs_mask: u8,
}

impl Default for SweepCmd {
    fn default() -> Self {
        Self {
            nonce: 0,
            seq: 0,
            kind: Kind::Power,
            bw: 20,
            frames_per_cell: 20,
            settle_ms: 5,
            gap_us: 1000,
            dwell_ms: 2000,
            idx_lo: -41,
            idx_hi: 63,
            mcs_mask: 0xFF,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum AckResult {
    Accepted = 0,
    AnchorUnreadable = 1,
    Busy = 2,
    /// A kind this drone does not run (a floor sweep on a radio without a
    /// runtime receive-gain floor, or a kind newer than its build).
    Unsupported = 3,
    /// A 40 MHz sweep on a channel with no HT40 pair.
    NoPair = 4,
}

impl AckResult {
    /// The C++ side casts any byte; an unknown one is kept as Unsupported.
    pub fn from_u8(b: u8) -> AckResult {
        match b {
            0 => AckResult::Accepted,
            1 => AckResult::AnchorUnreadable,
            2 => AckResult::Busy,
            4 => AckResult::NoPair,
            _ => AckResult::Unsupported,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SweepAck {
    pub nonce: u32,
    pub seq: u8,
    pub result: AckResult,
    pub start_in_ms: u16,
    pub flags: u8,
}

impl Default for SweepAck {
    fn default() -> Self {
        Self {
            nonce: 0,
            seq: 0,
            result: AckResult::Accepted,
            start_in_ms: 200,
            flags: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SweepDone {
    pub nonce: u32,
    pub seq: u8,
    pub offered: [u32; 8],
    pub blocked_pct: [u8; 8],
}

/// walls[i]: leave the config entry; eff_x1000 0 = leave.
pub const WALL_LEAVE: i8 = -128;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ApplyCmd {
    pub nonce: u32,
    pub seq: u8,
    pub bw: u8,
    pub has_walls: bool,
    pub has_eff: bool,
    pub walls: [i8; 8],
    pub legacy: i8,
    pub eff_x1000: [u16; 8],
    /// radio.wide_tx_igi_floor from a floor sweep (bw 40). Never in the
    /// same apply as walls, so on the wire it rides `legacy` (has bit 2).
    pub has_floor: bool,
    pub floor: u8,
}

impl Default for ApplyCmd {
    fn default() -> Self {
        Self {
            nonce: 0,
            seq: 0,
            bw: 20,
            has_walls: false,
            has_eff: false,
            walls: [WALL_LEAVE; 8],
            legacy: WALL_LEAVE,
            eff_x1000: [0; 8],
            has_floor: false,
            floor: 0,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Applied {
    pub nonce: u32,
    pub seq: u8,
    pub bw: u8,
    pub ok: bool,
    /// At most 49 bytes on the wire.
    pub reason: String,
}

impl Default for Applied {
    fn default() -> Self {
        Self {
            nonce: 0,
            seq: 0,
            bw: 20,
            ok: false,
            reason: String::new(),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Abort {
    pub nonce: u32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CalMsg {
    Sweep(SweepCmd),
    Ack(SweepAck),
    Done(SweepDone),
    Apply(ApplyCmd),
    Applied(Applied),
    Abort(Abort),
}

pub const MSG_ID_LO: u32 = 60000;
pub const MSG_ID_HI: u32 = 60005;
pub const MSG_SWEEP: u32 = 60000;
pub const MSG_ACK: u32 = 60001;
pub const MSG_SWEEP_DONE: u32 = 60002;
pub const MSG_APPLY: u32 = 60003;
pub const MSG_APPLIED: u32 = 60004;
pub const MSG_ABORT: u32 = 60005;

pub fn is_cal_msgid(id: u32) -> bool {
    (MSG_ID_LO..=MSG_ID_HI).contains(&id)
}

// ---- dialect (cal_mav.cpp); payloads in MAVLink wire order: fields by
// element size, largest first, declaration order within a size ----

fn put32(v: &mut Vec<u8>, x: u32) {
    v.extend_from_slice(&x.to_le_bytes());
}
fn put16(v: &mut Vec<u8>, x: u16) {
    v.extend_from_slice(&x.to_le_bytes());
}
fn rd32(p: &[u8], o: usize) -> u32 {
    u32::from_le_bytes([p[o], p[o + 1], p[o + 2], p[o + 3]])
}
fn rd16(p: &[u8], o: usize) -> u16 {
    u16::from_le_bytes([p[o], p[o + 1]])
}

/// The full-width payload and msgid of `m`.
pub fn payload(m: &CalMsg) -> (u32, Vec<u8>) {
    let mut v = Vec::with_capacity(57);
    let id = match m {
        CalMsg::Sweep(c) => {
            put32(&mut v, c.nonce);
            for x in [c.frames_per_cell, c.settle_ms, c.gap_us, c.dwell_ms] {
                put16(&mut v, x);
            }
            v.extend_from_slice(&[
                c.seq,
                c.kind as u8,
                c.bw,
                c.idx_lo as u8,
                c.idx_hi as u8,
                c.mcs_mask,
            ]);
            MSG_SWEEP
        }
        CalMsg::Ack(a) => {
            put32(&mut v, a.nonce);
            put16(&mut v, a.start_in_ms);
            v.extend_from_slice(&[a.seq, a.result as u8, a.flags]);
            MSG_ACK
        }
        CalMsg::Done(d) => {
            put32(&mut v, d.nonce);
            for x in d.offered {
                put32(&mut v, x);
            }
            v.push(d.seq);
            v.extend_from_slice(&d.blocked_pct);
            MSG_SWEEP_DONE
        }
        CalMsg::Apply(a) => {
            put32(&mut v, a.nonce);
            for x in a.eff_x1000 {
                put16(&mut v, x);
            }
            let has =
                u8::from(a.has_walls) | (u8::from(a.has_eff) << 1) | (u8::from(a.has_floor) << 2);
            v.extend_from_slice(&[a.seq, a.bw, has]);
            v.extend(a.walls.iter().map(|&w| w as u8));
            v.push(if a.has_floor { a.floor } else { a.legacy as u8 });
            MSG_APPLY
        }
        CalMsg::Applied(a) => {
            put32(&mut v, a.nonce);
            v.extend_from_slice(&[a.seq, a.bw, u8::from(a.ok)]);
            // char[50]: 49 bytes at most, NUL padded.
            let r = a.reason.as_bytes();
            let n = r.len().min(49);
            v.extend_from_slice(&r[..n]);
            v.resize(v.len() + 50 - n, 0);
            MSG_APPLIED
        }
        CalMsg::Abort(a) => {
            put32(&mut v, a.nonce);
            MSG_ABORT
        }
    };
    (id, v)
}

/// One MAVLink 2 packet for `m`.
pub fn encode(m: &CalMsg, sysid: u8, compid: u8, seq: u8) -> Vec<u8> {
    let (id, p) = payload(m);
    mavlink::pack_v2(id, sysid, compid, seq, &p)
}

/// `p` is zero-extended to the message's full length (the parser's view).
pub fn decode(msgid: u32, p: &[u8]) -> Option<CalMsg> {
    if !is_cal_msgid(msgid) || p.len() < usize::from(mavlink::entry(msgid)?.2) {
        return None;
    }
    Some(match msgid {
        MSG_SWEEP => CalMsg::Sweep(SweepCmd {
            nonce: rd32(p, 0),
            frames_per_cell: rd16(p, 4),
            settle_ms: rd16(p, 6),
            gap_us: rd16(p, 8),
            dwell_ms: rd16(p, 10),
            seq: p[12],
            // C++ casts any byte into the enum; an unknown kind cannot be
            // a sweep the GS sent, so it does not decode.
            kind: Kind::from_u8(p[13])?,
            bw: p[14],
            idx_lo: p[15] as i8,
            idx_hi: p[16] as i8,
            mcs_mask: p[17],
        }),
        MSG_ACK => CalMsg::Ack(SweepAck {
            nonce: rd32(p, 0),
            start_in_ms: rd16(p, 4),
            seq: p[6],
            result: AckResult::from_u8(p[7]),
            flags: p[8],
        }),
        MSG_SWEEP_DONE => {
            let mut d = SweepDone {
                nonce: rd32(p, 0),
                seq: p[36],
                ..SweepDone::default()
            };
            for i in 0..8 {
                d.offered[i] = rd32(p, 4 + 4 * i);
                d.blocked_pct[i] = p[37 + i];
            }
            CalMsg::Done(d)
        }
        MSG_APPLY => {
            let has_floor = p[22] & 4 != 0;
            let mut a = ApplyCmd {
                nonce: rd32(p, 0),
                seq: p[20],
                bw: p[21],
                has_walls: p[22] & 1 != 0,
                has_eff: p[22] & 2 != 0,
                has_floor,
                ..ApplyCmd::default()
            };
            // 0..0x7f: the floor fits the int8 legacy field.
            if has_floor {
                a.floor = p[31];
            } else {
                a.legacy = p[31] as i8;
            }
            for i in 0..8 {
                a.eff_x1000[i] = rd16(p, 4 + 2 * i);
                a.walls[i] = p[23 + i] as i8;
            }
            CalMsg::Apply(a)
        }
        MSG_APPLIED => {
            let r = &p[7..57];
            let n = r.iter().position(|&b| b == 0).unwrap_or(50);
            CalMsg::Applied(Applied {
                nonce: rd32(p, 0),
                seq: p[4],
                bw: p[5],
                ok: p[6] != 0,
                reason: String::from_utf8_lossy(&r[..n]).into_owned(),
            })
        }
        MSG_ABORT => CalMsg::Abort(Abort { nonce: rd32(p, 0) }),
        _ => return None,
    })
}

/// Every whole packet in `bytes` with a fresh parser.
fn packets(bytes: &[u8], mut f: impl FnMut(u32, &[u8]) -> bool) -> (usize, bool) {
    let mut ps = Parser::default();
    let mut n = 0;
    for &c in bytes {
        if ps.feed(c) == Framing::Ok {
            n += 1;
            let m = ps.message();
            let full = mavlink::entry(m.msgid).map_or(m.len, |e| e.2) as usize;
            if !f(m.msgid, &m.payload[..full.max(m.len as usize)]) {
                return (n, false);
            }
        }
    }
    (n, ps.is_idle())
}

/// True iff at least one packet parsed, every packet is a calibration
/// message and nothing partial is left over.
pub fn all_packets_are_cal(bytes: &[u8]) -> bool {
    if bytes.is_empty() {
        return false;
    }
    let (n, ok) = packets(bytes, |id, _| is_cal_msgid(id));
    n > 0 && ok
}

/// Every calibration message in `bytes`, decoded, in order; anything else
/// is skipped.
pub fn decode_all(bytes: &[u8]) -> Vec<CalMsg> {
    let mut out = Vec::new();
    packets(bytes, |id, p| {
        if let Some(m) = decode(id, p) {
            out.push(m);
        }
        true
    });
    out
}

// ---- cal_park.h ----

/// The wall margin in the chip's 0.25 dB steps (`std::lround`).
pub fn margin_steps(margin_db: f64) -> i32 {
    let x = margin_db * 4.0;
    // lround: half away from zero.
    (if x >= 0.0 { x + 0.5 } else { x - 0.5 }) as i32
}

/// Where a verify pass parks each rate an apply wrote:
/// clamp(wall - margin, -64, 63); WALL_LEAVE stays WALL_LEAVE.
pub fn park_rel_for(walls: &[i8; 8], margin_db: f64) -> [i32; 8] {
    let m = margin_steps(margin_db);
    let mut park = [WALL_LEAVE as i32; 8];
    for (p, &w) in park.iter_mut().zip(walls) {
        if w != WALL_LEAVE {
            *p = (w as i32 - m).clamp(REL_MIN, REL_MAX);
        }
    }
    park
}

// ---- the floor sweep (cal_floor.h) ----

/// HT MCS 2 at 40 MHz, production-sized frames: the rate the stall was
/// measured at.
pub const FLOOR_MCS: u8 = 2;
/// Candidates, ascending, two IGI steps apart: 0x20 is the bottom of the
/// receiver's normal DIG window on a DFS channel, 0x36 the first even step
/// past devourer's default 0x35.
pub const FLOOR_LO: u8 = 0x20;
pub const FLOOR_HI: u8 = 0x36;
pub const FLOOR_STEP: u8 = 2;
pub const FLOOR_MAX_CANDIDATES: usize = 12;
/// The top of the 7-bit IGI field.
pub const FLOOR_MAX: u8 = 0x7f;
pub const FLOOR_SETTLE_MS: u16 = 300;
pub const FLOOR_DWELL_MS: u16 = 4000;
pub const FLOOR_REF_DWELL_MS: u32 = 2000;
pub const FLOOR_CONFIRM_MS: u32 = 8000;
pub const FLOOR_MAX_CONFIRMS: u32 = 2;
/// The unit's edge plus this many IGI steps (about 1 dB each of the
/// drone's uplink sensitivity while 40 MHz flies) is what the unit needs.
pub const FLOOR_MARGIN_STEPS: u8 = 4;
/// devourer's default floor, what the bundle's drone config ships.
pub const FLOOR_DEFAULT: u8 = 0x35;

/// lo, lo + FLOOR_STEP, ... up to hi, at most FLOOR_MAX_CANDIDATES; empty
/// for a range outside [1, FLOOR_MAX] or upside down.
pub fn floor_candidates(lo: i32, hi: i32) -> Vec<u8> {
    if lo < 1 || hi > i32::from(FLOOR_MAX) || lo > hi {
        return Vec::new();
    }
    (lo..=hi)
        .step_by(usize::from(FLOOR_STEP))
        .take(FLOOR_MAX_CANDIDATES)
        .map(|v| v as u8)
        .collect()
}

/// The value written for a unit whose lowest confirmed floor is `edge`:
/// edge + FLOOR_MARGIN_STEPS, never under FLOOR_DEFAULT, at most FLOOR_MAX.
/// Calibration raises the floor for a unit that needs more than the
/// default and otherwise leaves the default; it never lowers it. On one
/// RTL8822E at 40 MHz (edge 0x24) floors 0x28 and 0x35 gave the same
/// video (capture to complete AU p50 24 ms, p90 27-28 ms) but the drone
/// took 62 % and 73 % of the GS's RCFs at 0x28 against 93 % and 98 % at
/// 0x35: the lower floor hears the close-range uplink worse. What a lower
/// floor buys at long range is unmeasured, so the policy stays here, in one
/// place (mabur `floor_to_write`).
pub fn floor_to_write(edge: u8) -> u8 {
    let need = (u32::from(edge) + u32::from(FLOOR_MARGIN_STEPS)).min(u32::from(FLOOR_MAX));
    need.max(u32::from(FLOOR_DEFAULT)) as u8
}

/// The floor sweep as the GS commands it: the candidates in idx_lo/idx_hi,
/// the per-candidate dwell in dwell_ms, the settle in settle_ms, the one
/// MCS in mcs_mask. Always 40 MHz.
pub fn make_floor_sweep(nonce: u32, seq: u8) -> SweepCmd {
    SweepCmd {
        nonce,
        seq,
        kind: Kind::Floor,
        bw: 40,
        frames_per_cell: 0,
        gap_us: 0,
        settle_ms: FLOOR_SETTLE_MS,
        dwell_ms: FLOOR_DWELL_MS,
        idx_lo: FLOOR_LO as i8,
        idx_hi: FLOOR_HI as i8,
        mcs_mask: 1 << FLOOR_MCS,
    }
}

/// The longest the drone can take for `c`: the reference point, every
/// candidate and FLOOR_MAX_CONFIRMS confirmations, each with its settle.
pub fn floor_sweep_max_ms(c: &SweepCmd) -> u32 {
    let n = floor_candidates(i32::from(c.idx_lo), i32::from(c.idx_hi)).len() as u32;
    let points = 1 + n + FLOOR_MAX_CONFIRMS;
    points * u32::from(c.settle_ms)
        + FLOOR_REF_DWELL_MS
        + n * u32::from(c.dwell_ms)
        + FLOOR_MAX_CONFIRMS * FLOOR_CONFIRM_MS
}

/// What a floor sweep found, as the drone's SweepDone carries it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FloorReport {
    /// Reference rate, frames/s.
    pub ref_fps: u32,
    /// Lowest confirmed value; 0 = none passed.
    pub floor: u8,
    /// Candidates dwelt on, from the first.
    pub tried: u8,
    /// The last confirmation's worst second, % of the reference.
    pub confirm_pct: u8,
    /// Per candidate tried, its worst second in frames/s.
    pub min_fps: [u16; FLOOR_MAX_CANDIDATES],
}

/// SweepDone for Kind::Floor: offered[0] the reference rate, offered[1]
/// the confirmed floor, offered[2..7] the candidates' worst seconds two
/// per word (the even candidate in the low half), blocked_pct[0] the
/// candidates tried, blocked_pct[1] the confirmation's percent.
pub fn pack_floor_done(nonce: u32, seq: u8, r: &FloorReport) -> SweepDone {
    let mut d = SweepDone {
        nonce,
        seq,
        ..SweepDone::default()
    };
    d.offered[0] = r.ref_fps;
    d.offered[1] = u32::from(r.floor);
    for i in 0..FLOOR_MAX_CANDIDATES / 2 {
        d.offered[2 + i] = u32::from(r.min_fps[2 * i]) | (u32::from(r.min_fps[2 * i + 1]) << 16);
    }
    d.blocked_pct[0] = r.tried;
    d.blocked_pct[1] = r.confirm_pct;
    d
}

pub fn unpack_floor_done(d: &SweepDone) -> FloorReport {
    let mut r = FloorReport {
        ref_fps: d.offered[0],
        floor: d.offered[1].min(u32::from(FLOOR_MAX)) as u8,
        tried: (d.blocked_pct[0] as usize).min(FLOOR_MAX_CANDIDATES) as u8,
        confirm_pct: d.blocked_pct[1],
        ..FloorReport::default()
    };
    for i in 0..FLOOR_MAX_CANDIDATES / 2 {
        r.min_fps[2 * i] = (d.offered[2 + i] & 0xFFFF) as u16;
        r.min_fps[2 * i + 1] = (d.offered[2 + i] >> 16) as u16;
    }
    r
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    #[test]
    fn round_trips_every_field() {
        let p = build_payload(Kind::Power, 5, -41, 40, 3, 0x12345, PAYLOAD_LEN);
        assert_eq!(p.len(), PAYLOAD_LEN);
        let f = parse_payload(&p).unwrap();
        assert_eq!(f.kind, Kind::Power);
        assert_eq!(
            (f.rate, f.idx, f.bw, f.flags, f.seq),
            (5, -41, 40, 3, 0x12345)
        );
    }

    #[test]
    fn capacity_frame_is_body_sized() {
        let p = build_payload(Kind::Capacity, 3, 0, 20, 2, 7, 1405);
        assert_eq!(p.len(), 1405);
        let f = parse_payload(&p).unwrap();
        assert_eq!((f.seq, f.kind, f.len), (7, Kind::Capacity, 1405));
    }

    #[test]
    fn fcs_tail_is_tolerated_on_both_sizes() {
        let mut small = build_payload(Kind::Verify, 0, 40, 20, 0, 1, PAYLOAD_LEN);
        small.extend_from_slice(&[0xAA; 4]);
        let f = parse_payload(&small).unwrap();
        assert_eq!((f.idx, f.len), (40, PAYLOAD_LEN));
        let mut big = build_payload(Kind::Capacity, 3, 0, 20, 2, 7, 1405);
        big.extend_from_slice(&[0xAA; 4]);
        let f = parse_payload(&big).unwrap();
        assert_eq!((f.seq, f.len), (7, 1405));
    }

    #[test]
    fn rejects_bad_kind_bw_flags() {
        let base = build_payload(Kind::Power, 0, 5, 20, 0, 1, PAYLOAD_LEN);
        for (i, b) in [(4, 5u8), (7, 30), (8, 4)] {
            let mut p = base.clone();
            p[i] = b;
            assert!(parse_payload(&p).is_none(), "byte {i} = {b}");
        }
        let mut p = base.clone();
        p[6] = 64; // idx out of range (fill no longer matters)
        assert!(parse_payload(&p).is_none());
    }

    #[test]
    fn one_flipped_fill_byte_is_refused() {
        let mut p = build_payload(Kind::Power, 3, 60, 20, 0, 1, PAYLOAD_LEN);
        p[40] ^= 0xFF;
        assert!(parse_payload(&p).is_none());
    }

    fn all_msgs() -> Vec<CalMsg> {
        vec![
            CalMsg::Sweep(SweepCmd {
                nonce: 0xDEADBEEF,
                seq: 3,
                kind: Kind::Capacity,
                bw: 40,
                frames_per_cell: 21,
                settle_ms: 300,
                gap_us: 999,
                dwell_ms: 2000,
                idx_lo: -41,
                idx_hi: 63,
                mcs_mask: 0x7F,
            }),
            CalMsg::Ack(SweepAck {
                nonce: 7,
                seq: 1,
                result: AckResult::AnchorUnreadable,
                start_in_ms: 200,
                flags: 3,
            }),
            CalMsg::Done(SweepDone {
                nonce: 9,
                seq: 2,
                offered: [1, 2, 3, 4, 5, 6, 7, 0x01020304],
                blocked_pct: [0, 10, 20, 30, 40, 50, 60, 100],
            }),
            CalMsg::Apply(ApplyCmd {
                nonce: 11,
                seq: 4,
                bw: 20,
                has_walls: true,
                has_eff: true,
                walls: [63, 63, 60, 50, 45, 23, WALL_LEAVE, -64],
                legacy: 63,
                eff_x1000: [930, 880, 840, 800, 760, 0, 780, 1000],
                ..ApplyCmd::default()
            }),
            CalMsg::Applied(Applied {
                nonce: 12,
                seq: 4,
                bw: 40,
                ok: true,
                reason: "ok; mcs6 kept from the 20 MHz table".into(),
            }),
            CalMsg::Abort(Abort { nonce: 13 }),
        ]
    }

    #[test]
    fn every_message_round_trips_through_the_dialect() {
        for m in all_msgs() {
            let w = encode(&m, 255, 190, 0);
            let got = decode_all(&w);
            assert_eq!(got, vec![m.clone()], "{m:?}");
            assert!(all_packets_are_cal(&w));
        }
    }

    /// Bytes from mabur's own encoder (cal_mav.cpp, sysid 255 compid 190,
    /// the channel's packet seq 0..5), for the messages of all_msgs().
    #[test]
    fn encode_matches_mabur_byte_for_byte() {
        const GOLD: [&str; 6] = [
            "fd12000000ffbe60ea00efbeadde15002c01e703d007030328d73f7f8b8e",
            "fd09000001ffbe61ea0007000000c8000101039f7e",
            "fd2d000002ffbe62ea0009000000010000000200000003000000040000000500000006000000070000000403020102000a141e28323c64058e",
            "fd20000003ffbe63ea000b000000a203700348032003f80200000c03e8030414033f3f3c322d1780c03ff516",
            "fd2a000004ffbe64ea000c0000000428016f6b3b206d637336206b6570742066726f6d20746865203230204d487a207461626c65db0d",
            "fd01000005ffbe65ea000d7514",
        ];
        for (i, (m, g)) in all_msgs().iter().zip(GOLD).enumerate() {
            let w = encode(m, 255, 190, i as u8);
            let hex: String = w.iter().map(|b| alloc::format!("{b:02x}")).collect();
            assert_eq!(hex, g, "{m:?}");
        }
    }

    #[test]
    fn payload_lengths_match_the_dialect() {
        for m in all_msgs() {
            let (id, p) = payload(&m);
            assert_eq!(p.len() as u8, mavlink::entry(id).unwrap().2, "{id}");
        }
    }

    #[test]
    fn reason_is_cut_to_49_bytes() {
        let long: String = core::iter::repeat_n('x', 80).collect();
        let m = CalMsg::Applied(Applied {
            reason: long,
            ..Applied::default()
        });
        match &decode_all(&encode(&m, 1, 1, 0))[0] {
            CalMsg::Applied(a) => assert_eq!(a.reason.len(), 49),
            o => panic!("{o:?}"),
        }
    }

    #[test]
    fn mixed_or_partial_streams_are_not_all_cal() {
        let cal = encode(&CalMsg::Abort(Abort { nonce: 1 }), 1, 1, 0);
        let hb = mavlink::pack_v2(mavlink::MSG_HEARTBEAT, 1, 1, 0, &[0u8; 9]);
        let mut mixed = cal.clone();
        mixed.extend_from_slice(&hb);
        assert!(!all_packets_are_cal(&mixed));
        assert_eq!(decode_all(&mixed).len(), 1);
        let mut partial = cal.clone();
        partial.extend_from_slice(&cal[..5]);
        assert!(!all_packets_are_cal(&partial));
        assert!(!all_packets_are_cal(&[]));
    }

    #[test]
    fn floor_frame_parses_with_the_reference_index() {
        let p = build_payload(Kind::Floor, FLOOR_MCS, -1, 40, 3, 9, 1405);
        let f = parse_payload(&p).unwrap();
        assert_eq!(
            (f.kind, f.rate, f.idx, f.bw),
            (Kind::Floor, FLOOR_MCS, -1, 40)
        );
    }

    /// mabur cal_mav.cpp: has bit 2, the floor in the legacy byte.
    #[test]
    fn a_floor_apply_rides_the_legacy_byte() {
        let a = ApplyCmd {
            nonce: 5,
            seq: 2,
            bw: 40,
            has_floor: true,
            floor: 0x7f,
            ..ApplyCmd::default()
        };
        let (_, p) = payload(&CalMsg::Apply(a));
        assert_eq!((p[22], p[31]), (4, 0x7f));
        assert_eq!(
            decode_all(&encode(&CalMsg::Apply(a), 1, 1, 0)),
            vec![CalMsg::Apply(a)]
        );
        // Without the bit the byte is legacy again.
        let w = ApplyCmd {
            legacy: -5,
            ..ApplyCmd::default()
        };
        assert_eq!(
            decode_all(&encode(&CalMsg::Apply(w), 1, 1, 0)),
            vec![CalMsg::Apply(w)]
        );
    }

    #[test]
    fn no_pair_decodes_and_an_unknown_result_is_unsupported() {
        assert_eq!(AckResult::from_u8(4), AckResult::NoPair);
        assert_eq!(AckResult::from_u8(9), AckResult::Unsupported);
    }

    #[test]
    fn floor_candidates_and_the_longest_walk() {
        let c = floor_candidates(i32::from(FLOOR_LO), i32::from(FLOOR_HI));
        assert_eq!(c.len(), 12);
        assert_eq!((c[0], c[11]), (0x20, 0x36));
        assert!(floor_candidates(0, 10).is_empty());
        assert!(floor_candidates(0x30, 0x20).is_empty());
        // 15 settles + 2 + 12 x 4 + 2 x 8 s.
        assert_eq!(floor_sweep_max_ms(&make_floor_sweep(1, 0)), 70_500);
        let s = make_floor_sweep(7, 3);
        assert_eq!(
            (s.kind, s.bw, s.mcs_mask),
            (Kind::Floor, 40, 1 << FLOOR_MCS)
        );
    }

    #[test]
    fn the_written_floor_is_the_edge_plus_margin_never_under_the_default() {
        assert_eq!(FLOOR_DEFAULT, 0x35);
        assert_eq!(floor_to_write(0x24), 0x35); // needs 0x28: the default stays
        assert_eq!(floor_to_write(0x31), 0x35);
        assert_eq!(floor_to_write(0x32), 0x36);
        assert_eq!(floor_to_write(0x34), 0x38); // needs more: raised
        assert_eq!(floor_to_write(0x7c), 0x7f);
        assert_eq!(floor_to_write(0x7e), 0x7f); // clamped to the IGI field
    }

    #[test]
    fn a_floor_report_round_trips_through_sweep_done() {
        let mut r = FloorReport {
            ref_fps: 2484,
            floor: 0x24,
            tried: 3,
            confirm_pct: 100,
            ..FloorReport::default()
        };
        r.min_fps[..3].copy_from_slice(&[0, 2130, 2484]);
        let d = pack_floor_done(9, 4, &r);
        assert_eq!(
            (d.nonce, d.seq, d.offered[1], d.offered[2]),
            (9, 4, 0x24, 2130 << 16)
        );
        assert_eq!(unpack_floor_done(&d), r);
    }

    #[test]
    fn park_rule_matches_cal_park() {
        let walls = [63, 40, -60, WALL_LEAVE, 0, 10, 20, 30];
        let p = park_rel_for(&walls, 1.0);
        assert_eq!(p, [59, 36, -64, WALL_LEAVE as i32, -4, 6, 16, 26]);
        assert_eq!(margin_steps(0.625), 3); // 2.5 rounds away from zero
        assert_eq!(margin_steps(-0.625), -3);
    }
}
