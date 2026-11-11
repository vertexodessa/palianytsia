//! Fill slice: every CU a skip CU with merge_idx 0 over CTUs
//! [first_ctb, end_ctb) (mabur `hevc_skip_slice.cpp`).
use crate::bits::{escape, BitWriter};
use crate::cabac::{init_ctx, CabacCtx, CabacEncoder};
use crate::ps::{Pps, Sps};
use crate::slice::{write_slice_header, HeaderEdit, SliceHeader};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FillBin {
    Split,
    Skip,
    MergeIdx,
    EndOfSlice,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FillStep {
    pub kind: FillBin,
    pub ctx_inc: u8,
    pub value: u8,
}

struct Grid {
    w: u32,
    h: u32,
    log2_min: u32,
    log2_ctb: u32,
    wctb: u32,
    first_ctb: u32,
    cols: usize,
    depth: Vec<u8>,
    skip: Vec<u8>,
    coded: Vec<u8>,
}

impl Grid {
    fn new(sps: &Sps, first_ctb: u32) -> Self {
        let cols = (sps.width >> sps.log2_min_cb) as usize;
        let rows = (sps.height >> sps.log2_min_cb) as usize;
        Self {
            w: sps.width,
            h: sps.height,
            log2_min: sps.log2_min_cb,
            log2_ctb: sps.log2_ctb,
            wctb: sps.pic_w_ctbs(),
            first_ctb,
            cols,
            depth: vec![0; cols * rows],
            skip: vec![0; cols * rows],
            coded: vec![0; cols * rows],
        }
    }
    fn idx(&self, x: i32, y: i32) -> usize {
        ((y as usize) >> self.log2_min) * self.cols + ((x as usize) >> self.log2_min)
    }
    fn available(&self, x: i32, y: i32) -> bool {
        if x < 0 || y < 0 || x >= self.w as i32 || y >= self.h as i32 {
            return false;
        }
        let ctb = ((y as u32) >> self.log2_ctb) * self.wctb + ((x as u32) >> self.log2_ctb);
        ctb >= self.first_ctb && self.coded[self.idx(x, y)] != 0
    }
    fn mark(&mut self, x0: i32, y0: i32, size: i32, depth: i32) {
        let step = 1i32 << self.log2_min;
        let mut y = y0;
        while y < y0 + size && y < self.h as i32 {
            let mut x = x0;
            while x < x0 + size && x < self.w as i32 {
                let i = self.idx(x, y);
                self.depth[i] = depth as u8;
                self.skip[i] = 1;
                self.coded[i] = 1;
                x += step;
            }
            y += step;
        }
    }
}

fn walk(
    g: &mut Grid,
    max_merge: u32,
    x0: i32,
    y0: i32,
    log2: i32,
    depth: i32,
    out: &mut Vec<FillStep>,
) {
    let size = 1i32 << log2;
    // Inside the picture the fill codes split_cu_flag = 0; across an edge the
    // split is implicit.
    let split = if x0 + size <= g.w as i32 && y0 + size <= g.h as i32 && log2 > g.log2_min as i32 {
        let a = g.available(x0 - 1, y0) && i32::from(g.depth[g.idx(x0 - 1, y0)]) > depth;
        let b = g.available(x0, y0 - 1) && i32::from(g.depth[g.idx(x0, y0 - 1)]) > depth;
        out.push(FillStep {
            kind: FillBin::Split,
            ctx_inc: u8::from(a) + u8::from(b),
            value: 0,
        });
        false
    } else {
        log2 > g.log2_min as i32
    };
    if split {
        let half = size >> 1;
        walk(g, max_merge, x0, y0, log2 - 1, depth + 1, out);
        if x0 + half < g.w as i32 {
            walk(g, max_merge, x0 + half, y0, log2 - 1, depth + 1, out);
        }
        if y0 + half < g.h as i32 {
            walk(g, max_merge, x0, y0 + half, log2 - 1, depth + 1, out);
        }
        if x0 + half < g.w as i32 && y0 + half < g.h as i32 {
            walk(g, max_merge, x0 + half, y0 + half, log2 - 1, depth + 1, out);
        }
        return;
    }
    let a = g.available(x0 - 1, y0) && g.skip[g.idx(x0 - 1, y0)] != 0;
    let b = g.available(x0, y0 - 1) && g.skip[g.idx(x0, y0 - 1)] != 0;
    out.push(FillStep {
        kind: FillBin::Skip,
        ctx_inc: u8::from(a) + u8::from(b),
        value: 1,
    });
    if max_merge > 1 {
        out.push(FillStep {
            kind: FillBin::MergeIdx,
            ctx_inc: 0,
            value: 0,
        });
    }
    g.mark(x0, y0, size, depth);
}

pub fn skip_fill_steps(
    sps: &Sps,
    max_num_merge_cand: u32,
    first_ctb: u32,
    end_ctb: u32,
) -> Vec<FillStep> {
    let mut out = Vec::new();
    let mut g = Grid::new(sps, first_ctb);
    let wctb = sps.pic_w_ctbs();
    for ctb in first_ctb..end_ctb {
        let x0 = ((ctb % wctb) << sps.log2_ctb) as i32;
        let y0 = ((ctb / wctb) << sps.log2_ctb) as i32;
        walk(
            &mut g,
            max_num_merge_cand,
            x0,
            y0,
            sps.log2_ctb as i32,
            0,
            &mut out,
        );
        out.push(FillStep {
            kind: FillBin::EndOfSlice,
            ctx_inc: 0,
            value: u8::from(ctb + 1 == end_ctb),
        });
    }
    out
}

/// Annex-B fill NAL (00 00 00 01 + tmpl's NAL header + escaped RBSP); None
/// for an I-slice template or an empty / out-of-picture range.
/// Level 6.2 of H.265 (Table A.8): MaxLumaPs, and the longest side it
/// permits, sqrt(8 * MaxLumaPs).
const MAX_LUMA_PS: u64 = 35_651_584;
const MAX_PIC_SIDE: u32 = 16_888;

pub fn make_skip_slice(
    sps: &Sps,
    pps: &Pps,
    tmpl: &SliceHeader,
    first_ctb: u32,
    end_ctb: u32,
) -> Option<Vec<u8>> {
    if tmpl.slice_type == 2 {
        return None;
    }
    // The grid below is sized by the SPS, and the SPS is whatever the stream
    // delivered (video bodies carry no key tag): a forged 2^20 x 2^20 picture
    // asks for three vectors of 17 GB and the failed allocation ends the
    // process. Nothing above the largest picture an HEVC level allows gets a
    // fill.
    if sps.width > MAX_PIC_SIDE
        || sps.height > MAX_PIC_SIDE
        || u64::from(sps.width) * u64::from(sps.height) > MAX_LUMA_PS
    {
        return None;
    }
    let total = sps.pic_w_ctbs().wrapping_mul(sps.pic_h_ctbs());
    if first_ctb >= end_ctb || end_ctb > total {
        return None;
    }
    let mut w = BitWriter::default();
    write_slice_header(
        &mut w,
        tmpl,
        &HeaderEdit {
            first: first_ctb == 0,
            address: first_ctb,
            clear_sao: true,
        },
        sps,
        pps,
    );
    let init_type = if tmpl.slice_type == 1 {
        if tmpl.cabac_init_flag {
            2
        } else {
            1
        }
    } else if tmpl.cabac_init_flag {
        1
    } else {
        2
    };
    const SPLIT: [i32; 3] = [107, 139, 126];
    const SKIP: [i32; 3] = [197, 185, 201];
    let merge_init = if init_type == 1 { 122 } else { 137 };
    let mut split = [CabacCtx::default(); 3];
    let mut skip = [CabacCtx::default(); 3];
    for i in 0..3 {
        split[i] = init_ctx(SPLIT[i], tmpl.slice_qp_y);
        skip[i] = init_ctx(SKIP[i], tmpl.slice_qp_y);
    }
    let mut merge = init_ctx(merge_init, tmpl.slice_qp_y);
    let steps = skip_fill_steps(sps, tmpl.max_num_merge_cand, first_ctb, end_ctb);
    {
        let mut enc = CabacEncoder::new(&mut w);
        for s in steps {
            match s.kind {
                FillBin::Split => enc.encode(&mut split[s.ctx_inc as usize], s.value),
                FillBin::Skip => enc.encode(&mut skip[s.ctx_inc as usize], s.value),
                FillBin::MergeIdx => enc.encode(&mut merge, s.value),
                FillBin::EndOfSlice => enc.terminate(s.value),
            }
        }
    }
    w.align_zero();
    let mut out = vec![0, 0, 0, 1, tmpl.nal_hdr[0], tmpl.nal_hdr[1]];
    out.extend_from_slice(&escape(w.bytes()));
    Some(out)
}
