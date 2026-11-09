//! Sub-Block Integrity framing (mabur `common/src/sbi.cpp`): a 13 B header,
//! then per FEC envelope `<u16 crc16_ccitt LE, envelope>`. The receiver takes
//! block_payload from its own config; the header is only sanity-checked, so a
//! corrupt header cannot desync the scan of a kept-corrupt body.
use crate::crc16::crc16_ccitt;
use alloc::vec::Vec;

pub const MAGIC: u16 = 0xF5B0;
pub const HDR_LEN: usize = 13;
/// Ver 0 (7 B) and 1 (11 B, no air_ms) are rejected outright.
pub const VER: u8 = 2;
pub const Q_MS_OFF: usize = 7;
pub const ENC_US_OFF: usize = 9;
pub const AIR_MS_OFF: usize = 11;
pub const MSP_STREAM_ID: u8 = 4;
pub const PROBE_STREAM_ID: u8 = 5;
pub const RETX_MARK: u8 = 0x80;
pub const STREAM_ID_MASK: u8 = 0x7F;

fn rd16(p: &[u8]) -> u16 {
    u16::from_le_bytes([p[0], p[1]])
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Unpacked<'a> {
    /// CRC-valid sub-block payloads, in body order.
    pub survivors: Vec<&'a [u8]>,
    pub n_failed: usize,
    pub header_ok: bool,
    pub retx: bool,
    pub q_ms: u16,
    pub enc_us: u16,
    pub air_ms: u16,
}

pub fn unpack(body: &[u8], block_payload: usize) -> Unpacked<'_> {
    let mut r = Unpacked::default();
    if block_payload == 0 || body.len() < HDR_LEN {
        return r;
    }
    r.retx = body[3] & RETX_MARK != 0;
    r.header_ok =
        rd16(body) == MAGIC && body[2] == VER && usize::from(rd16(&body[4..])) == block_payload;
    if r.header_ok {
        r.q_ms = rd16(&body[Q_MS_OFF..]);
        r.enc_us = rd16(&body[ENC_US_OFF..]);
        r.air_ms = rd16(&body[AIR_MS_OFF..]);
    }
    let stride = 2 + block_payload;
    let region = &body[HDR_LEN..];
    for blk in region.chunks_exact(stride) {
        let payload = &blk[2..];
        if crc16_ccitt(payload) == rd16(blk) {
            r.survivors.push(payload);
        } else {
            r.n_failed += 1;
        }
    }
    r
}

/// Stream id for routing, None on a short / bad-magic / bad-version header.
pub fn peek_stream_id(body: &[u8]) -> Option<u8> {
    if body.len() < HDR_LEN || rd16(body) != MAGIC || body[2] != VER {
        return None;
    }
    Some(body[3] & STREAM_ID_MASK)
}

/// Packs envelopes into bodies of `blocks_per_body` (the drone side; here for
/// tests and synthetic captures).
pub struct Packer {
    block_payload: usize,
    blocks_per_body: usize,
    stream_id: u8,
    body: Vec<u8>,
    pending: usize,
}

impl Packer {
    pub fn new(block_payload: usize, blocks_per_body: usize, stream_id: u8) -> Self {
        Self {
            block_payload,
            blocks_per_body,
            stream_id,
            body: Vec::new(),
            pending: 0,
        }
    }
    fn take(&mut self) -> Vec<u8> {
        self.body[6] = self.pending as u8;
        self.pending = 0;
        core::mem::take(&mut self.body)
    }
    pub fn add(&mut self, env: &[u8]) -> Option<Vec<u8>> {
        if env.len() != self.block_payload {
            return None;
        }
        if self.pending == 0 {
            self.body.clear();
            self.body.extend_from_slice(&MAGIC.to_le_bytes());
            self.body.push(VER);
            self.body.push(self.stream_id);
            self.body
                .extend_from_slice(&(self.block_payload as u16).to_le_bytes());
            self.body.extend_from_slice(&[0; 7]);
        }
        self.body.extend_from_slice(&crc16_ccitt(env).to_le_bytes());
        self.body.extend_from_slice(env);
        self.pending += 1;
        (self.pending >= self.blocks_per_body).then(|| self.take())
    }
    pub fn flush(&mut self) -> Option<Vec<u8>> {
        (self.pending > 0).then(|| self.take())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_and_salvage() {
        let mut p = Packer::new(20, 3, 1);
        assert!(p.add(&[1; 20]).is_none());
        assert!(p.add(&[2; 20]).is_none());
        let mut b = p.add(&[3; 20]).unwrap();
        assert_eq!(b.len(), HDR_LEN + 3 * 22);
        assert_eq!(peek_stream_id(&b), Some(1));
        b[HDR_LEN + 22 + 5] ^= 0xFF; // corrupt block 1
        let u = unpack(&b, 20);
        assert!(u.header_ok);
        assert_eq!((u.survivors.len(), u.n_failed), (2, 1));
        assert_eq!(
            u.survivors,
            alloc::vec![alloc::vec![1u8; 20], alloc::vec![3u8; 20]]
        );
        assert!(!unpack(&b, 21).header_ok);
    }
}
