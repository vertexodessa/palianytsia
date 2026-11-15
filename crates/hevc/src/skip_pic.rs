//! The whole-frame skip picture that stands in for a damaged or lost
//! P picture (pal-pzj.11, pal-pzj.39): every CTU skip, merge_idx 0, the
//! header of the newest reference P picture with only the POC changed.
//! Measured in apps/pal/src/bin/hole_policy; the GS inserts one for a base
//! AU lost whole (runtime::skip_fill).
use crate::bits::{ceil_log2, unescape, BitReader};
use crate::nal::split_nals;
use crate::ps::{ParamTracker, Pps, Sps};
use crate::skip_slice::make_skip_slice;
use crate::slice::parse_slice_header;

/// A short-term RPS: the delta POCs it keeps (negative first), used or not.
pub type RpsSet = Vec<i32>;

/// st_ref_pic_set(idx) with the deltas derived (H.265 7.4.8). `sets` are
/// the SPS's sets before `idx`; `in_slice` for the one a slice header
/// codes (idx == num_short_term_ref_pic_sets). Some((negative, positive))
/// unless the syntax is bad.
pub fn read_st_rps(
    r: &mut BitReader,
    idx: usize,
    in_slice: bool,
    sets: &[(Vec<i32>, Vec<i32>)],
) -> Option<(Vec<i32>, Vec<i32>)> {
    let inter = idx != 0 && r.flag();
    if inter {
        let delta_idx = if in_slice { r.ue() as usize + 1 } else { 1 };
        let rf = idx.checked_sub(delta_idx)?;
        let (rneg, rpos) = sets.get(rf)?;
        let sign = r.u(1);
        let delta_rps = (1 - 2 * sign as i32) * (r.ue() as i32 + 1);
        let n = rneg.len() + rpos.len();
        let mut use_delta = vec![true; n + 1];
        for u in use_delta.iter_mut() {
            if !r.flag() {
                *u = r.flag();
            }
        }
        let (mut neg, mut pos) = (Vec::new(), Vec::new());
        for j in (0..rpos.len()).rev() {
            let d = rpos[j] + delta_rps;
            if d < 0 && use_delta[rneg.len() + j] {
                neg.push(d);
            }
        }
        if delta_rps < 0 && use_delta[n] {
            neg.push(delta_rps);
        }
        for (j, &rn) in rneg.iter().enumerate() {
            let d = rn + delta_rps;
            if d < 0 && use_delta[j] {
                neg.push(d);
            }
        }
        for j in (0..rneg.len()).rev() {
            let d = rneg[j] + delta_rps;
            if d > 0 && use_delta[j] {
                pos.push(d);
            }
        }
        if delta_rps > 0 && use_delta[n] {
            pos.push(delta_rps);
        }
        for (j, &rp) in rpos.iter().enumerate() {
            let d = rp + delta_rps;
            if d > 0 && use_delta[rneg.len() + j] {
                pos.push(d);
            }
        }
        (!r.bad()).then_some((neg, pos))
    } else {
        let nneg = r.ue();
        let npos = r.ue();
        if nneg > 16 || npos > 16 {
            return None;
        }
        let (mut neg, mut pos) = (Vec::new(), Vec::new());
        let mut poc = 0i32;
        for _ in 0..nneg {
            poc -= r.ue() as i32 + 1;
            r.u(1);
            neg.push(poc);
        }
        poc = 0;
        for _ in 0..npos {
            poc += r.ue() as i32 + 1;
            r.u(1);
            pos.push(poc);
        }
        (!r.bad()).then_some((neg, pos))
    }
}

/// The SPS's short-term RPS sets with their deltas (hevc::ps keeps only
/// the counts). `nal` from its 2-byte header, escaped.
pub fn sps_rps_sets(nal: &[u8]) -> Option<Vec<(Vec<i32>, Vec<i32>)>> {
    let rb = unescape(nal.get(2..)?);
    let mut r = BitReader::new(&rb);
    r.u(4);
    let max_sub = r.u(3) as usize;
    r.u(1);
    // profile_tier_level(1, max_sub_layers_minus1): 88 bits + level
    r.u(32);
    r.u(32);
    r.u(32);
    let mut flags = vec![(0, 0); max_sub];
    for f in flags.iter_mut() {
        *f = (r.u(1), r.u(1));
    }
    if max_sub > 0 {
        for _ in max_sub..8 {
            r.u(2);
        }
    }
    for (pp, lp) in flags {
        if pp != 0 {
            r.u(32);
            r.u(32);
            r.u(24);
        }
        if lp != 0 {
            r.u(8);
        }
    }
    r.ue(); // sps_id
    let chroma = r.ue();
    if chroma == 3 {
        r.u(1);
    }
    r.ue();
    r.ue();
    if r.flag() {
        for _ in 0..4 {
            r.ue();
        }
    }
    r.ue();
    r.ue();
    r.ue(); // log2_max_poc_lsb_minus4
    let sub_info = r.flag();
    let start = if sub_info { 0 } else { max_sub };
    for _ in start..=max_sub {
        r.ue();
        r.ue();
        r.ue();
    }
    for _ in 0..6 {
        r.ue();
    }
    if r.flag() && r.flag() {
        return None; // scaling list data: not walked
    }
    r.u(1); // amp
    r.u(1); // sao
    if r.flag() {
        r.u(4);
        r.u(4);
        r.ue();
        r.ue();
        r.u(1);
    }
    let n = r.ue() as usize;
    if n > 64 {
        return None;
    }
    let mut sets = Vec::with_capacity(n);
    for i in 0..n {
        let s = read_st_rps(&mut r, i, false, &sets)?;
        sets.push(s);
    }
    (!r.bad()).then_some(sets)
}

/// The part of a first slice's header the skip rules look at.
#[derive(Clone, Debug)]
pub struct Hdr {
    pub nal_type: u8,
    pub slice_type: u32,
    pub poc_lsb: u32,
    /// Bit position of slice_pic_order_cnt_lsb in the RBSP.
    pub poc_pos: usize,
    /// Kept delta POCs of the short-term RPS; None for an IDR.
    pub rps: Option<RpsSet>,
    /// How the RPS was coded: Some(idx) for an SPS set, None explicit.
    pub rps_idx: Option<u32>,
}

/// First-slice header fields; `nal` from its 2-byte header, escaped.
pub fn parse_hdr(nal: &[u8], sps: &Sps, pps: &Pps, sets: &[(Vec<i32>, Vec<i32>)]) -> Option<Hdr> {
    if nal.len() < 4 {
        return None;
    }
    let nal_type = crate::nal::nal_type(nal[0]);
    let rb = unescape(&nal[2..]);
    let mut r = BitReader::new(&rb);
    let first = r.flag();
    if (16..=23).contains(&nal_type) {
        r.u(1);
    }
    if r.ue() != pps.pps_id {
        return None;
    }
    if !first {
        if pps.dependent_slices && r.flag() {
            return None;
        }
        r.u(sps.addr_bits());
    }
    r.u(pps.extra_bits);
    let slice_type = r.ue();
    if pps.output_flag {
        r.u(1);
    }
    let mut h = Hdr {
        nal_type,
        slice_type,
        poc_lsb: 0,
        poc_pos: r.pos(),
        rps: None,
        rps_idx: None,
    };
    if nal_type != 19 && nal_type != 20 {
        h.poc_lsb = r.u(sps.log2_max_poc_lsb);
        let (neg, pos) = if !r.flag() {
            read_st_rps(&mut r, sets.len(), true, sets)?
        } else {
            let mut idx = 0;
            if sps.num_st_rps > 1 {
                idx = r.u(ceil_log2(sps.num_st_rps));
            }
            h.rps_idx = Some(idx);
            sets.get(idx as usize)?.clone()
        };
        if sps.long_term {
            return None;
        }
        h.rps = Some(neg.into_iter().chain(pos).collect());
    }
    (!r.bad()).then_some(h)
}

/// The first VCL NAL of an AU (escaped, from its NAL header).
pub fn first_vcl(au: &[u8]) -> Option<&[u8]> {
    split_nals(au)
        .into_iter()
        .find(|v| v.ty < 32)
        .map(|v| &au[v.off..v.off + v.len])
}

/// Parameter sets and RPS sets of the complete AUs seen so far.
#[derive(Default)]
pub struct Params {
    pub pt: ParamTracker,
    pub sets: Vec<(Vec<i32>, Vec<i32>)>,
}

impl Params {
    /// Every whole NAL of `au`: all of a complete AU, the NALs before the
    /// last start code of an incomplete one (a truncated IDR still brings
    /// its VPS/SPS/PPS).
    pub fn feed(&mut self, au: &[u8], complete: bool) {
        let au = if complete {
            au
        } else {
            match split_nals(au).last() {
                Some(v) => &au[..v.off.saturating_sub(3)],
                None => return,
            }
        };
        self.pt.feed(au);
        for v in split_nals(au) {
            if v.ty == 33 {
                if let Some(s) = sps_rps_sets(&au[v.off..v.off + v.len]) {
                    self.sets = s;
                }
            }
        }
    }
    pub fn hdr(&self, au: &[u8]) -> Option<Hdr> {
        if !self.pt.usable() {
            return None;
        }
        parse_hdr(first_vcl(au)?, self.pt.sps(), self.pt.pps(), &self.sets)
    }
}

/// A P picture the skip decoder fed, as a donor candidate.
#[derive(Clone)]
struct Pic {
    hdr: Hdr,
    nal: Vec<u8>,
    pts: u32,
}

/// Builds the whole-frame skip picture for a damaged AU, or says why not.
/// Donor rules (waybeam-link): the donor is the newest reference P
/// picture fed (TRAIL_R, never TRAIL_N); it and the reference P picture
/// before it carry the same short-term RPS; the skip copies the donor's
/// slice header (so its L0 count, collocated_ref_idx and TMVP flag) with
/// only the POC changed, and every picture the copied RPS names must be in
/// the DPB (the donor itself or one the donor's RPS kept).
#[derive(Default)]
pub struct SkipMaker {
    pub ps: Params,
    /// The last two reference P pictures fed, oldest first.
    refs: Vec<Pic>,
    /// The last two P pictures fed of any kind (the rule read literally).
    any: Vec<Hdr>,
    /// Damaged AUs for which the literal rule (last two P pictures of any
    /// kind share the RPS) failed.
    pub literal_fail: u32,
}

impl SkipMaker {
    /// An AU that went to this decoder: `whole` when complete or a skip
    /// picture (a skip TRAIL_R picture is a reference like any other).
    pub fn fed(&mut self, data: &[u8], complete: bool, whole: bool, pts: u32) {
        self.ps.feed(data, complete);
        let Some(h) = self.ps.hdr(data) else { return };
        if (16..=23).contains(&h.nal_type) {
            self.refs.clear();
            self.any.clear();
            return;
        }
        if h.slice_type == 2 || !whole {
            return;
        }
        self.any.push(h.clone());
        if self.any.len() > 2 {
            self.any.remove(0);
        }
        if h.nal_type % 2 == 1 {
            self.refs.push(Pic {
                hdr: h,
                nal: first_vcl(data).unwrap_or_default().to_vec(),
                pts,
            });
            if self.refs.len() > 2 {
                self.refs.remove(0);
            }
        }
    }

    /// The skip picture standing for the AU `data` (what of it arrived;
    /// empty for one lost whole) captured at `pts`, `frame_us` apart from
    /// its neighbours on the camera clock.
    pub fn make(&mut self, data: &[u8], pts: u32, frame_us: u32) -> Result<Vec<u8>, &'static str> {
        if self.any.len() == 2 && self.any[0].rps != self.any[1].rps {
            self.literal_fail += 1;
        }
        if !self.ps.pt.usable() {
            return Err("no_params");
        }
        let (sps, pps) = (self.ps.pt.sps().clone(), self.ps.pt.pps().clone());
        if self.refs.len() < 2 {
            return Err("no_two_ref_p");
        }
        let (prev, donor) = (&self.refs[0], &self.refs[1]);
        if prev.hdr.rps != donor.hdr.rps {
            return Err("rps_differs");
        }
        if donor.hdr.nal_type != 1 {
            return Err("donor_not_trail_r");
        }
        let deltas = donor.hdr.rps.clone().ok_or("donor_no_rps")?;
        let max = 1u32 << sps.log2_max_poc_lsb;
        // The damaged picture's own POC when its first slice header arrived,
        // else the donor's plus the frame steps its pts says passed.
        let poc = match self.ps.hdr(data) {
            Some(h) => h.poc_lsb,
            None => {
                let steps = (pts.wrapping_sub(donor.pts) + frame_us / 2) / frame_us.max(1);
                (donor.hdr.poc_lsb + steps) % max
            }
        };
        let dpoc = ((poc + max - donor.hdr.poc_lsb) % max) as i32;
        if dpoc == 0 || dpoc > (max / 2) as i32 {
            return Err("poc_not_after_donor");
        }
        for d in &deltas {
            let want = dpoc + d;
            if want != 0 && !deltas.contains(&want) {
                return Err("ref_not_in_dpb");
            }
        }
        let mut tmpl =
            parse_slice_header(&donor.nal, &sps, &pps).map_err(|_| "donor_unparsable")?;
        let w = sps.log2_max_poc_lsb as usize;
        for b in 0..w {
            let p = donor.hdr.poc_pos + b;
            let m = 0x80u8 >> (p % 8);
            if (poc >> (w - 1 - b)) & 1 != 0 {
                tmpl.rbsp[p / 8] |= m;
            } else {
                tmpl.rbsp[p / 8] &= !m;
            }
        }
        let total = sps.pic_w_ctbs() * sps.pic_h_ctbs();
        make_skip_slice(&sps, &pps, &tmpl, 0, total).ok_or("make_skip_slice")
    }
}
