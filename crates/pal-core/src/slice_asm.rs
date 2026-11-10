//! One access unit of a row-sliced H.265 picture rebuilt from FrameStream's
//! fragment chunks (mabur `gs/src/slice_assembler.cpp`).
//!
//! Fragment i covers AU bytes [i*F - hdr_len, ...), F = chunk 0's size.
//! drain() streams whole NAL units of the contiguous prefix; finish() emits
//! the rest: the remaining bytes of a complete AU, or every remaining slice
//! (original or skip fill) of a salvageable damaged AU, or the raw prefix
//! remainder when salvage is refused. A slice counts only if its start code
//! and every byte to the next start code lie in one run of consecutive
//! fragments.
use hevc::ps::{Pps, Sps};
use hevc::skip_slice::make_skip_slice;
use hevc::slice::{parse_slice_header, SliceHeader, SliceParse};
use std::collections::BTreeMap;

pub type ChunkMap = BTreeMap<u16, Vec<u8>>;

const CTB_LOG2: u32 = 6;
const MAX_SLICES: u32 = 64;
const RESERVE_CAP: usize = 1 << 20;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum SliceFallback {
    #[default]
    None = 0,
    NoParams,
    Unsupported,
    ISlice,
    NoTemplate,
    Geometry,
}
pub const SLICE_FB_COUNT: usize = 6;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SliceSalvage {
    pub salvaged: bool,
    pub slices: u8,
    pub kept: u8,
    pub filled: u8,
    pub kept_after_hole: u8,
    pub fallback: SliceFallback,
}

#[derive(Clone, Copy)]
struct Code {
    pos: usize,
    len: usize,
}

fn find_codes(b: &[u8], from: usize, out: &mut Vec<Code>) {
    let mut j = from;
    while let Some((pos, len)) = hevc::nal::find_start_code(b, j) {
        out.push(Code { pos, len });
        j = pos + len;
    }
}

pub struct SliceAssembler {
    sps: Sps,
    pps: Pps,
    slice_rows: u8,
    count: u16,
    hdr_len: usize,
    n: u8,
    frag: usize,
    prefix_chunks: u16,
    buf: Vec<u8>,
    sc: Vec<Code>,
    scanned: usize,
    emitted: usize,
    result: SliceSalvage,
}

struct Slice {
    bytes: Vec<u8>,
    h: SliceHeader,
    off: usize,
}

impl SliceAssembler {
    pub fn new(sps: &Sps, pps: &Pps, slice_rows: u8, count: u16, hdr_len: usize) -> Self {
        let mut n = 0u8;
        if sps.log2_ctb == CTB_LOG2 && slice_rows != 0 {
            let rows = sps.pic_h_ctbs();
            let k = rows.div_ceil(u32::from(slice_rows));
            if k <= MAX_SLICES {
                n = k as u8;
            }
        }
        Self {
            sps: sps.clone(),
            pps: pps.clone(),
            slice_rows,
            count,
            hdr_len,
            n,
            frag: 0,
            prefix_chunks: 0,
            buf: Vec::new(),
            sc: Vec::new(),
            scanned: 0,
            emitted: 0,
            result: SliceSalvage::default(),
        }
    }

    pub fn slices(&self) -> u8 {
        self.n
    }
    pub fn result(&self) -> &SliceSalvage {
        &self.result
    }

    fn extend_prefix(&mut self, chunks: &ChunkMap) {
        while let Some(c) = chunks.get(&self.prefix_chunks) {
            if self.prefix_chunks == 0 {
                self.frag = c.len();
                self.buf
                    .reserve((usize::from(self.count) * self.frag).min(RESERVE_CAP));
                if c.len() > self.hdr_len {
                    self.buf.extend_from_slice(&c[self.hdr_len..]);
                }
            } else {
                self.buf.extend_from_slice(c);
            }
            self.prefix_chunks += 1;
        }
    }

    fn scan_prefix(&mut self) {
        let mut found = Vec::new();
        find_codes(&self.buf, self.scanned, &mut found);
        // The 3-byte rescan window can sit inside a 4-byte code already found.
        for c in found {
            if self.sc.last().is_none_or(|b| c.pos >= b.pos + b.len) {
                self.sc.push(c);
            }
        }
        self.scanned = self.buf.len().saturating_sub(3);
    }

    pub fn drain(&mut self, chunks: &ChunkMap, out: &mut dyn FnMut(&[u8])) {
        self.extend_prefix(chunks);
        self.scan_prefix();
        if let Some(last) = self.sc.last() {
            if last.pos > self.emitted {
                out(&self.buf[self.emitted..last.pos]);
                self.emitted = last.pos;
            }
        }
    }

    /// `raw_out` takes the passthrough remainder (may end mid-NAL).
    pub fn finish(
        &mut self,
        chunks: &ChunkMap,
        out: &mut dyn FnMut(&[u8]),
        raw_out: &mut dyn FnMut(&[u8]),
    ) {
        self.extend_prefix(chunks);
        if self.prefix_chunks == self.count {
            out(&self.buf[self.emitted..]);
            self.emitted = self.buf.len();
            self.result.slices = self.n;
            self.result.kept = self.n;
            return;
        }
        let mut pieces = Vec::new();
        let mut res = SliceSalvage::default();
        let fb = self.plan(chunks, &mut pieces, &mut res);
        if fb != SliceFallback::None {
            self.result.fallback = fb;
            raw_out(&self.buf[self.emitted..]);
            self.emitted = self.buf.len();
            return;
        }
        for p in &pieces {
            out(p);
        }
        self.result = res;
    }

    fn plan(
        &mut self,
        chunks: &ChunkMap,
        pieces: &mut Vec<Vec<u8>>,
        res: &mut SliceSalvage,
    ) -> SliceFallback {
        use SliceFallback as F;
        if self.sps.log2_ctb != CTB_LOG2 {
            return F::Unsupported;
        }
        if self.n == 0 || self.frag <= self.hdr_len {
            return F::Geometry;
        }
        let frag = self.frag;
        let hdr = self.hdr_len;
        let count = self.count;
        if self.buf.len() != usize::from(self.prefix_chunks) * frag - hdr {
            return F::Geometry;
        }
        // 1. Runs of consecutive fragments as AU byte ranges; run 0 = prefix.
        let mut runs: Vec<(usize, Vec<u8>)> = vec![(0, self.buf.clone())];
        let mut i = self.prefix_chunks + 1;
        while i < count {
            if !chunks.contains_key(&i) {
                i += 1;
                continue;
            }
            let off = usize::from(i) * frag - hdr;
            let mut b = Vec::new();
            while i < count {
                let Some(c) = chunks.get(&i) else { break };
                if i + 1 < count && c.len() != frag {
                    return F::Geometry;
                }
                b.extend_from_slice(c);
                i += 1;
            }
            runs.push((off, b));
        }
        let au_end = chunks
            .get(&(count - 1))
            .map(|c| usize::from(count - 1) * frag - hdr + c.len());
        // 2. Complete NAL units of every run.
        let n = usize::from(self.n);
        let mut slices: Vec<Option<Slice>> = (0..n).map(|_| None).collect();
        let mut prefix_nonvcl: Vec<(usize, Vec<u8>)> = Vec::new();
        let span = u32::from(self.slice_rows).wrapping_mul(self.sps.pic_w_ctbs());
        let mut seen_vcl = false;
        for (r, (roff, rb)) in runs.iter().enumerate() {
            let mut codes = Vec::new();
            find_codes(rb, 0, &mut codes);
            let reaches_end = au_end == Some(roff + rb.len());
            for k in 0..codes.len() {
                let s = codes[k].pos;
                let body = s + codes[k].len;
                let e = if k + 1 < codes.len() {
                    codes[k + 1].pos
                } else if reaches_end {
                    rb.len()
                } else {
                    break;
                };
                // A 3-byte code opening a run after a hole may be the tail of
                // a 4-byte one whose first zero was lost.
                if r > 0 && s == 0 && codes[k].len == 3 {
                    continue;
                }
                if body + 2 > e {
                    continue;
                }
                let nal = &rb[body..e];
                let ty = hevc::nal::nal_type(nal[0]);
                let bytes = rb[s..e].to_vec();
                if ty >= 32 {
                    if r == 0 && !seen_vcl {
                        prefix_nonvcl.push((s, bytes));
                    }
                    continue;
                }
                seen_vcl = true;
                let h = match parse_slice_header(nal, &self.sps, &self.pps) {
                    Ok(h) => h,
                    Err(SliceParse::Unsupported | SliceParse::Dependent) => return F::Unsupported,
                    Err(_) => return F::Geometry,
                };
                let addr = if h.first { 0 } else { h.address };
                // A picture wider than u32 CTBs wraps the span to 0 (mabur
                // divides by it and dies): no slice geometry fits.
                if span == 0 {
                    return F::Geometry;
                }
                if h.first != (addr == 0)
                    || addr % span != 0
                    || (addr / span) as usize >= n
                    || slices[(addr / span) as usize].is_some()
                {
                    return F::Geometry;
                }
                slices[(addr / span) as usize] = Some(Slice {
                    bytes,
                    h,
                    off: roff + s,
                });
            }
        }
        // 3. Template and rules.
        let Some(tmpl) = slices.iter().flatten().next().map(|s| s.h.clone()) else {
            return F::NoTemplate;
        };
        if tmpl.slice_type == 2 {
            return F::ISlice;
        }
        // 4. Slices drain() already streamed: the first `done` must be 0..done-1.
        let mut done = 0usize;
        for c in &self.sc {
            if c.pos >= self.emitted {
                break;
            }
            if c.pos + c.len < self.buf.len() && hevc::nal::nal_type(self.buf[c.pos + c.len]) < 32 {
                done += 1;
            }
        }
        for s in slices.iter().take(done) {
            match s {
                Some(s) if s.off < self.emitted => {}
                _ => return F::Geometry,
            }
        }
        let first_missing = slices.iter().position(Option::is_none).unwrap_or(n);
        if done == 0 {
            for (off, bytes) in prefix_nonvcl {
                if off >= self.emitted {
                    pieces.push(bytes);
                }
            }
        }
        let total = self.sps.pic_w_ctbs().wrapping_mul(self.sps.pic_h_ctbs());
        let mut kept = 0u8;
        let mut kept_after_hole = 0u8;
        for (k, s) in slices.iter_mut().enumerate() {
            if s.is_some() {
                kept += 1;
                if k > first_missing {
                    kept_after_hole += 1;
                }
            }
            if k < done {
                continue;
            }
            match s.take() {
                Some(s) => pieces.push(s.bytes),
                None => {
                    let k = k as u32;
                    let Some(fill) = make_skip_slice(
                        &self.sps,
                        &self.pps,
                        &tmpl,
                        k.wrapping_mul(span),
                        (k + 1).wrapping_mul(span).min(total),
                    ) else {
                        return F::Geometry;
                    };
                    pieces.push(fill);
                }
            }
        }
        res.salvaged = true;
        res.slices = self.n;
        res.kept = kept;
        res.kept_after_hole = kept_after_hole;
        res.filled = self.n - kept;
        F::None
    }
}
