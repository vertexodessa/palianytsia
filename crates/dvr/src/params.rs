//! VPS/SPS/PPS bookkeeping and the hvcC they make (mabur `hevc_params.cpp`).

use hevc::bits::unescape;
use hevc::nal::split_nals;
use hevc::ps::sps_display_size;

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
        let r = unescape(&self.sps[2.min(self.sps.len())..]);
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
        sps_display_size(&self.sps)
    }
}
