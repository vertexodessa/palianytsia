//! slice_segment_header parse and rewrite (H.265 7.3.6.1; mabur
//! `hevc_slice.cpp`). Parsed far enough to rewrite first/address/SAO; the
//! bits between those positions are copied verbatim.
use crate::bits::{ceil_log2, unescape, BitReader, BitWriter};
use crate::ps::{parse_st_rps, Pps, Sps};

#[derive(Clone, Debug, Default)]
pub struct SliceHeader {
    pub nal_hdr: [u8; 2],
    pub nal_type: u8,
    pub rbsp: Vec<u8>,
    pub first: bool,
    pub address: u32,
    /// 0 B, 1 P, 2 I
    pub slice_type: u32,
    pub slice_qp_y: i32,
    pub cabac_init_flag: bool,
    pub max_num_merge_cand: u32,
    pub sao_luma: bool,
    pub sao_chroma: bool,
    pub deblocking_disabled: bool,
    pub lf_across_present: bool,
    pub lf_across: bool,
    pub pos_after_pps_id: usize,
    pub pos_after_address: usize,
    pub pos_sao: usize,
    pub pos_lf_across: usize,
    pub pos_end: usize,
    pub data_byte: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SliceParse {
    Malformed,
    Unsupported,
    Dependent,
}

/// nal: from the 2-byte NAL header, escaped.
pub fn parse_slice_header(nal: &[u8], sps: &Sps, pps: &Pps) -> Result<SliceHeader, SliceParse> {
    use SliceParse::*;
    if nal.len() < 4 {
        return Err(Malformed);
    }
    let mut h = SliceHeader {
        nal_hdr: [nal[0], nal[1]],
        nal_type: (nal[0] >> 1) & 0x3F,
        slice_qp_y: 26,
        max_num_merge_cand: 5,
        ..Default::default()
    };
    if h.nal_type > 21 {
        return Err(Malformed);
    }
    h.rbsp = unescape(&nal[2..]);
    let rbsp = std::mem::take(&mut h.rbsp);
    let mut r = BitReader::new(&rbsp);
    h.first = r.flag();
    if (16..=23).contains(&h.nal_type) {
        r.u(1);
    }
    if r.ue() != pps.pps_id {
        return Err(Unsupported);
    }
    h.pos_after_pps_id = r.pos();
    let mut dependent = false;
    if !h.first {
        if pps.dependent_slices {
            dependent = r.flag();
        }
        h.address = r.u(sps.addr_bits());
    }
    h.pos_after_address = r.pos();
    if dependent {
        return Err(Dependent);
    }
    r.u(pps.extra_bits);
    h.slice_type = r.ue();
    if h.slice_type > 2 {
        return Err(Malformed);
    }
    if pps.output_flag {
        r.u(1);
    }
    let mut slice_tmvp = false;
    let mut total_curr = 0u32;
    if h.nal_type != 19 && h.nal_type != 20 {
        r.u(sps.log2_max_poc_lsb);
        if !r.flag() {
            let (_, used) = parse_st_rps(
                &mut r,
                sps.num_st_rps,
                sps.num_st_rps,
                &sps.st_num_delta_pocs,
            )
            .ok_or(Malformed)?;
            total_curr += used;
        } else {
            let mut idx = 0;
            if sps.num_st_rps > 1 {
                idx = r.u(ceil_log2(sps.num_st_rps));
            }
            if idx >= sps.num_st_rps {
                return Err(Malformed);
            }
            total_curr += sps.st_num_used[idx as usize];
        }
        if sps.long_term {
            let n_sps = if sps.num_lt_sps > 0 { r.ue() } else { 0 };
            let n_pics = r.ue();
            if n_sps > sps.num_lt_sps || n_sps + n_pics > 32 {
                return Err(Malformed);
            }
            for i in 0..n_sps + n_pics {
                if i < n_sps {
                    let mut li = 0;
                    if sps.num_lt_sps > 1 {
                        li = r.u(ceil_log2(sps.num_lt_sps));
                    }
                    if li >= sps.num_lt_sps {
                        return Err(Malformed);
                    }
                    total_curr += u32::from(sps.lt_used_sps[li as usize]);
                } else {
                    r.u(sps.log2_max_poc_lsb);
                    total_curr += r.u(1);
                }
                if r.flag() {
                    r.ue();
                }
            }
        }
        if sps.tmvp {
            slice_tmvp = r.flag();
        }
    }
    if sps.sao {
        h.pos_sao = r.pos();
        h.sao_luma = r.flag();
        if sps.chroma_format_idc != 0 {
            h.sao_chroma = r.flag();
        }
    }
    let (mut nl0, mut nl1) = (pps.num_ref_idx_l0_default, pps.num_ref_idx_l1_default);
    if h.slice_type != 2 {
        if r.flag() {
            nl0 = r.ue() + 1;
            if h.slice_type == 0 {
                nl1 = r.ue() + 1;
            }
        }
        if nl0 > 15 || nl1 > 15 {
            return Err(Malformed);
        }
        if pps.lists_modification && total_curr > 1 {
            let bits = ceil_log2(total_curr);
            if r.flag() {
                for _ in 0..nl0 {
                    r.u(bits);
                }
            }
            if h.slice_type == 0 && r.flag() {
                for _ in 0..nl1 {
                    r.u(bits);
                }
            }
        }
        if h.slice_type == 0 {
            r.u(1);
        }
        if pps.cabac_init_present {
            h.cabac_init_flag = r.flag();
        }
        if slice_tmvp {
            let mut col_l0 = true;
            if h.slice_type == 0 {
                col_l0 = r.flag();
            }
            if (col_l0 && nl0 > 1) || (!col_l0 && nl1 > 1) {
                r.ue();
            }
        }
        let five_minus = r.ue();
        if five_minus > 4 {
            return Err(Malformed);
        }
        h.max_num_merge_cand = 5 - five_minus;
    }
    h.slice_qp_y = pps.init_qp + r.se();
    if pps.chroma_qp_offsets_present {
        r.se();
        r.se();
    }
    let mut over = false;
    if pps.deblocking_override_enabled {
        over = r.flag();
    }
    h.deblocking_disabled = pps.deblocking_disabled;
    if over {
        h.deblocking_disabled = r.flag();
        if !h.deblocking_disabled {
            r.se();
            r.se();
        }
    }
    if pps.loop_filter_across_slices && (h.sao_luma || h.sao_chroma || !h.deblocking_disabled) {
        h.lf_across_present = true;
        h.pos_lf_across = r.pos();
        h.lf_across = r.flag();
    }
    h.pos_end = r.pos();
    if r.u(1) != 1 {
        return Err(Malformed);
    }
    while r.pos() & 7 != 0 {
        if r.u(1) != 0 {
            return Err(Malformed);
        }
    }
    if r.bad() {
        return Err(Malformed);
    }
    h.data_byte = r.pos() / 8;
    h.rbsp = rbsp;
    Ok(h)
}

#[derive(Clone, Copy, Debug)]
pub struct HeaderEdit {
    pub first: bool,
    pub address: u32,
    pub clear_sao: bool,
}

/// Writes t's header with e applied, then byte_alignment().
pub fn write_slice_header(
    w: &mut BitWriter,
    t: &SliceHeader,
    e: &HeaderEdit,
    sps: &Sps,
    pps: &Pps,
) {
    let b = &t.rbsp;
    w.put(e.first);
    w.copy(b, 1, t.pos_after_pps_id);
    if !e.first {
        if pps.dependent_slices {
            w.put(false);
        }
        w.u(e.address, sps.addr_bits());
    }
    let mut from = t.pos_after_address;
    if sps.sao {
        let sao_bits = if sps.chroma_format_idc != 0 { 2 } else { 1 };
        w.copy(b, from, t.pos_sao);
        if e.clear_sao {
            for _ in 0..sao_bits {
                w.put(false);
            }
        } else {
            w.copy(b, t.pos_sao, t.pos_sao + sao_bits);
        }
        from = t.pos_sao + sao_bits;
    }
    let sao_on = !e.clear_sao && (t.sao_luma || t.sao_chroma);
    let lf_present = pps.loop_filter_across_slices && (sao_on || !t.deblocking_disabled);
    if t.lf_across_present {
        w.copy(b, from, t.pos_lf_across);
        if lf_present {
            w.put(t.lf_across);
        }
        w.copy(b, t.pos_lf_across + 1, t.pos_end);
    } else {
        w.copy(b, from, t.pos_end);
    }
    w.put(true);
    w.align_zero();
}
