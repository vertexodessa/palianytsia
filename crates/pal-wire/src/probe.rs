//! Probe stream bodies (mabur `common/src/probe_wire.cpp`): an SBI body on
//! stream 5 whose every block starts with a 9 B header
//! `<u16 magic 0xF5A5, u32 seq, u8 profile, u16 enh_fid>` and is filled
//! with splitmix64 bytes.
use crate::crc16::crc16_ccitt;
use crate::sbi;

pub const MAGIC: u16 = 0xF5A5;
pub const HDR_LEN: usize = 9;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ProbeHdr {
    pub seq: u32,
    pub profile: u8,
    pub enh_fid: u16,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ProbeRx {
    pub hdr: ProbeHdr,
    pub n_blocks: usize,
    /// Bit i: block i CRC-clean and header-consistent.
    pub survivors: u32,
    pub n_ok: u32,
}

pub fn parse_hdr(p: &[u8]) -> Option<ProbeHdr> {
    if p.len() < HDR_LEN || u16::from_le_bytes([p[0], p[1]]) != MAGIC {
        return None;
    }
    Some(ProbeHdr {
        seq: u32::from_le_bytes([p[2], p[3], p[4], p[5]]),
        profile: p[6],
        enh_fid: u16::from_le_bytes([p[7], p[8]]),
    })
}

pub fn body_len(bpb: usize, block_payload: usize) -> usize {
    sbi::HDR_LEN + bpb * (2 + block_payload)
}

pub fn parse_body(body: &[u8], block_payload: usize) -> Option<ProbeRx> {
    if sbi::peek_stream_id(body) != Some(sbi::PROBE_STREAM_ID) {
        return None;
    }
    if usize::from(u16::from_le_bytes([body[4], body[5]])) != block_payload {
        return None;
    }
    let n = usize::from(body[6]);
    if !(1..=32).contains(&n) {
        return None;
    }
    let stride = 2 + block_payload;
    if body.len() < sbi::HDR_LEN + n * stride {
        return None;
    }
    let mut rx = ProbeRx {
        n_blocks: n,
        ..ProbeRx::default()
    };
    let mut have = false;
    for i in 0..n {
        let blk = &body[sbi::HDR_LEN + i * stride..sbi::HDR_LEN + (i + 1) * stride];
        if u16::from_le_bytes([blk[0], blk[1]]) != crc16_ccitt(&blk[2..]) {
            continue;
        }
        let Some(h) = parse_hdr(&blk[2..]) else {
            continue;
        };
        if have && h != rx.hdr {
            return None;
        }
        rx.hdr = h;
        have = true;
        rx.survivors |= 1 << i;
        rx.n_ok += 1;
    }
    have.then_some(rx)
}
