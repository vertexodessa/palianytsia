//! Per-frame header maburd prepends to each Annex-B frame before
//! fragmentation (mabur `common/src/frame_wire.cpp`). All fields LE.
//! frame_id is global across UEP layers: the GS orders AUs by it.

pub const HDR_LEN: usize = 8;
pub const FLAG_IDR: u8 = 0x01;
/// pts / frame_id re-base point (producer restart).
pub const FLAG_DISCONT: u8 = 0x02;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FrameHdr {
    pub frame_id: u16,
    pub flags: u8,
    /// 64-px CTU rows per slice of this AU; 0 = one slice.
    pub slice_rows: u8,
    /// Capture time, microseconds, u32-truncated.
    pub pts_us: u32,
}

impl FrameHdr {
    pub fn pack(&self) -> [u8; HDR_LEN] {
        let id = self.frame_id.to_le_bytes();
        let p = self.pts_us.to_le_bytes();
        [
            id[0],
            id[1],
            self.flags,
            self.slice_rows,
            p[0],
            p[1],
            p[2],
            p[3],
        ]
    }
    pub fn parse(b: &[u8]) -> Option<Self> {
        if b.len() < HDR_LEN {
            return None;
        }
        Some(Self {
            frame_id: u16::from_le_bytes([b[0], b[1]]),
            flags: b[2],
            slice_rows: b[3],
            pts_us: u32::from_le_bytes([b[4], b[5], b[6], b[7]]),
        })
    }
}

/// FRAG fragment header: `<u16 fseq, u16 idx, u16 count>` LE.
pub const FRAG_HDR_LEN: usize = 6;

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn roundtrip() {
        let h = FrameHdr {
            frame_id: 0xBEEF,
            flags: FLAG_IDR,
            slice_rows: 5,
            pts_us: 0x0102_0304,
        };
        assert_eq!(h.pack(), [0xEF, 0xBE, 1, 5, 4, 3, 2, 1]);
        assert_eq!(FrameHdr::parse(&h.pack()), Some(h));
        assert_eq!(FrameHdr::parse(&[0; 7]), None);
    }
}
