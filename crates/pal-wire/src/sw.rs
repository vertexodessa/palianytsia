//! Sliding-window FEC envelope (mabur `common/src/sw_wire.cpp`): one fixed
//! 14-byte little-endian header for source and repair symbols, so SBI can cut
//! bodies at a config-fixed block payload.
//!
//! `MAGIC=0xF541 u16, flags u8 (bit0 repair), symbol_size u16,
//! seq u32 (source seq | repair window start), window_len u8, repair_key u32`.

pub const MAGIC: u16 = 0xF541;
pub const HEADER_LEN: usize = 14;
pub const FLAG_REPAIR: u8 = 0x01;
/// window_len is a u8.
pub const MAX_WINDOW: usize = 255;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Header {
    pub repair: bool,
    pub symbol_size: u16,
    pub seq: u32,
    pub window_len: u8,
    pub repair_key: u32,
}

impl Header {
    pub fn pack(&self, out: &mut [u8; HEADER_LEN]) {
        out[0..2].copy_from_slice(&MAGIC.to_le_bytes());
        out[2] = if self.repair { FLAG_REPAIR } else { 0 };
        out[3..5].copy_from_slice(&self.symbol_size.to_le_bytes());
        out[5..9].copy_from_slice(&self.seq.to_le_bytes());
        out[9] = self.window_len;
        out[10..14].copy_from_slice(&self.repair_key.to_le_bytes());
    }

    /// None on short input, bad magic, unknown flags, or fields that
    /// contradict the type (repair with window 0; source with window or key).
    pub fn parse(p: &[u8]) -> Option<Header> {
        if p.len() < HEADER_LEN || u16::from_le_bytes([p[0], p[1]]) != MAGIC {
            return None;
        }
        let flags = p[2];
        if flags & !FLAG_REPAIR != 0 {
            return None;
        }
        let h = Header {
            repair: flags & FLAG_REPAIR != 0,
            symbol_size: u16::from_le_bytes([p[3], p[4]]),
            seq: u32::from_le_bytes([p[5], p[6], p[7], p[8]]),
            window_len: p[9],
            repair_key: u32::from_le_bytes([p[10], p[11], p[12], p[13]]),
        };
        let ok = if h.repair {
            h.window_len != 0
        } else {
            h.window_len == 0 && h.repair_key == 0
        };
        ok.then_some(h)
    }
}

fn splitmix64(s: &mut u64) -> u64 {
    *s = s.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = *s;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// Frozen coefficient contract (mabur `tests/vectors/sw.json`): splitmix64
/// seeded with the key, one draw per window position, low byte mapped to
/// [1, 255]. Coefficient i scales source `window_start + i`.
pub fn repair_coeffs(repair_key: u32, out: &mut [u8]) {
    let mut s = u64::from(repair_key);
    for c in out.iter_mut() {
        *c = ((splitmix64(&mut s) & 0xFF) % 255 + 1) as u8;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn header_roundtrip_and_rejects() {
        let h = Header {
            repair: true,
            symbol_size: 332,
            seq: 0xDEAD_BEEF,
            window_len: 32,
            repair_key: 0x1234_5678,
        };
        let mut b = [0u8; HEADER_LEN];
        h.pack(&mut b);
        assert_eq!(Header::parse(&b), Some(h));
        let mut bad = b;
        bad[2] = 0x02;
        assert_eq!(Header::parse(&bad), None);
        let mut src = b;
        src[2] = 0; // source with a window: contradiction
        assert_eq!(Header::parse(&src), None);
        assert_eq!(Header::parse(&b[..13]), None);
    }

    #[test]
    fn coeffs_never_zero() {
        let mut c = [0u8; MAX_WINDOW];
        for key in [0u32, 1, 7, u32::MAX] {
            repair_coeffs(key, &mut c);
            assert!(c.iter().all(|&x| x != 0));
        }
    }
}
