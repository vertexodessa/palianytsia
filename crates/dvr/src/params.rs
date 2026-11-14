//! VPS/SPS/PPS bookkeeping and the hvcC they make (mabur `hevc_params.cpp`).

use hevc::nal::split_nals;

/// True when the AU carries VPS, SPS and PPS: every IDR and also the GDR
/// encoder's refresh start, which is a P slice and so not a sync point.
pub fn au_has_param_sets(au: &[u8]) -> bool {
    let (mut v, mut s, mut p) = (false, false, false);
    for n in split_nals(au) {
        v |= n.ty == 32;
        s |= n.ty == 33;
        p |= n.ty == 34;
    }
    v && s && p
}

/// An IRAP picture (NAL types 16..=23): the only place a file may begin.
pub fn au_is_irap(au: &[u8]) -> bool {
    split_nals(au).iter().any(|n| (16..=23).contains(&n.ty))
}

/// Annex B to MP4 sample layout: each NAL behind a 4-byte big-endian length.
pub fn annexb_to_length_prefixed(au: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(au.len() + 16);
    for n in split_nals(au) {
        out.extend_from_slice(&(n.len as u32).to_be_bytes());
        out.extend_from_slice(&au[n.off..n.off + n.len]);
    }
    out
}

/// The last VPS, SPS and PPS seen, NAL header included, escaped as on the wire.
#[derive(Default, Clone)]
pub struct HevcParams {
    vps: Vec<u8>,
    sps: Vec<u8>,
    pps: Vec<u8>,
}

/// RBSP of an escaped NAL payload (emulation-prevention bytes dropped),
/// from `from` on, at most `max` bytes.
fn rbsp(nal: &[u8], from: usize, max: usize) -> Vec<u8> {
    let mut out = Vec::with_capacity(nal.len().min(max));
    let mut zeros = 0;
    for &b in nal.iter().skip(from) {
        if out.len() >= max {
            break;
        }
        if zeros >= 2 && b == 0x03 {
            zeros = 0;
            continue;
        }
        zeros = if b == 0 { zeros + 1 } else { 0 };
        out.push(b);
    }
    out
}

struct Bits<'a> {
    b: &'a [u8],
    pos: usize,
    bad: bool,
}

impl Bits<'_> {
    fn u(&mut self, n: u32) -> u32 {
        let mut v = 0u32;
        for _ in 0..n {
            let byte = self.pos >> 3;
            let Some(&x) = self.b.get(byte) else {
                self.bad = true;
                return 0;
            };
            v = (v << 1) | ((x >> (7 - (self.pos & 7))) & 1) as u32;
            self.pos += 1;
        }
        v
    }
    fn ue(&mut self) -> u32 {
        let mut lz = 0;
        while self.u(1) == 0 {
            lz += 1;
            if self.bad || lz > 31 {
                self.bad = true;
                return 0;
            }
        }
        ((1u64 << lz) - 1) as u32 + self.u(lz)
    }
}

impl HevcParams {
    /// Takes the AU's parameter sets; true when all three are known.
    pub fn feed(&mut self, au: &[u8]) -> bool {
        for n in split_nals(au) {
            let nal = &au[n.off..n.off + n.len];
            match n.ty {
                32 => self.vps = nal.to_vec(),
                33 => self.sps = nal.to_vec(),
                34 => self.pps = nal.to_vec(),
                _ => {}
            }
        }
        self.complete()
    }

    pub fn complete(&self) -> bool {
        !self.vps.is_empty() && !self.sps.is_empty() && !self.pps.is_empty()
    }

    /// HEVCDecoderConfigurationRecord for the hvcC box; None until complete
    /// or for an SPS too short to hold its profile_tier_level.
    pub fn hvcc(&self) -> Option<Vec<u8>> {
        if !self.complete() {
            return None;
        }
        // RBSP bytes 0..12 after the 2-byte NAL header: the vps id / sub
        // layers / nesting byte, then the 12-byte general profile_tier_level.
        // Copying the escaped bytes would shift every field after the first
        // emulation-prevention byte, and a Main-profile SPS has two there.
        let r = rbsp(&self.sps, 2, 13);
        if r.len() < 13 {
            return None;
        }
        let mut out = vec![0x01];
        out.extend_from_slice(&r[1..13]);
        out.extend_from_slice(&[
            0xF0, 0x00, // min_spatial_segmentation_idc 0
            0xFC, // parallelismType 0
            0xFD, // chromaFormat 1 (4:2:0)
            0xF8, // bitDepthLumaMinus8 0
            0xF8, // bitDepthChromaMinus8 0
            0x00, 0x00, // avgFrameRate 0
            // constantFrameRate 0, numTemporalLayers 2, temporalIdNested 0,
            // lengthSizeMinusOne 3: mabur's value, kept byte for byte.
            0x13, 0x03, // numOfArrays: VPS, SPS, PPS
        ]);
        for (ty, nal) in [(32u8, &self.vps), (33, &self.sps), (34, &self.pps)] {
            out.push(0x80 | ty);
            out.extend_from_slice(&1u16.to_be_bytes());
            out.extend_from_slice(&(nal.len() as u16).to_be_bytes());
            out.extend_from_slice(nal);
        }
        Some(out)
    }

    /// The SPS's displayed size (conformance window applied).
    pub fn sps_dimensions(&self) -> Option<(u32, u32)> {
        if self.sps.len() < 3 {
            return None;
        }
        let rb = rbsp(&self.sps, 2, usize::MAX);
        let mut r = Bits {
            b: &rb,
            pos: 0,
            bad: false,
        };
        r.u(4);
        let max_sub = r.u(3).min(7) as usize;
        r.u(1);
        r.u(8);
        r.u(32);
        r.u(24);
        r.u(24);
        r.u(8);
        let mut prof = [false; 8];
        let mut lvl = [false; 8];
        for i in 0..max_sub {
            prof[i] = r.u(1) == 1;
            lvl[i] = r.u(1) == 1;
        }
        if max_sub > 0 {
            for _ in max_sub..8 {
                r.u(2);
            }
        }
        for i in 0..max_sub {
            if prof[i] {
                r.u(32);
                r.u(32);
                r.u(24);
            }
            if lvl[i] {
                r.u(8);
            }
        }
        r.ue();
        let chroma = r.ue();
        if chroma == 3 {
            r.u(1);
        }
        let mut pw = r.ue() as i64;
        let mut ph = r.ue() as i64;
        if r.u(1) == 1 {
            let (l, rt, t, bt) = (r.ue() as i64, r.ue() as i64, r.ue() as i64, r.ue() as i64);
            let sw = if chroma == 1 || chroma == 2 { 2 } else { 1 };
            let sh = if chroma == 1 { 2 } else { 1 };
            pw -= sw * (l + rt);
            ph -= sh * (t + bt);
        }
        if r.bad || pw <= 0 || ph <= 0 || pw > 16384 || ph > 16384 {
            return None;
        }
        Some((pw as u32, ph as u32))
    }
}
