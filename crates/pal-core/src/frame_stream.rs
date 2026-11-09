//! Reassembles frames from FRAG fragments of both UEP layers, orders them by
//! unwrapped frame_id and streams each frame's contiguous prefix out as
//! fragments arrive (mabur `gs/src/frame_stream.cpp`). Row-sliced AUs with a
//! known SPS/PPS go through SliceAssembler so a damaged tail can be salvaged.
use crate::slice_asm::{ChunkMap, SliceAssembler, SliceFallback, SliceSalvage, SLICE_FB_COUNT};
use hevc::nal::split_nals;
use hevc::ps::ParamTracker;
use pal_wire::frame_wire::{FrameHdr, FLAG_DISCONT, FRAG_HDR_LEN, HDR_LEN};
use std::collections::BTreeMap;

#[derive(Clone, Copy, Debug, Default)]
pub struct FragArrival {
    pub body_mono_us: u64,
    pub q_ms: u16,
    pub enc_us: u16,
    pub air_ms: u16,
    pub sw_seq: Option<u32>,
    pub retx: bool,
    pub drain_mono_us: u64,
    pub push_mono_us: u64,
}

/// Per-AU latency latch handed to `end`.
#[derive(Clone, Copy, Debug, Default)]
pub struct AuLat {
    pub t_first_us: u64,
    pub t_last_arr_us: u64,
    pub t_push_us: u64,
    pub t_drain_us: u64,
    pub drone_q_ms: u16,
    pub enc_us: u16,
    pub drone_air_ms: u16,
    pub hdr_retx: bool,
    pub skipped_before: u32,
    pub vcl_nal: u8,
    pub slice: SliceSalvage,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct TailView {
    pub count: u16,
    pub max_idx: u16,
    pub seq_at_max: u32,
    pub last_progress_ms: u64,
    pub first_ms: u64,
}

#[derive(Clone, Copy, Debug)]
pub struct FrameStreamCfg {
    pub gap_timeout_ms: u64,
    pub lookahead: u64,
    /// Frames arriving but none emitted this long = self-reset; 0 disables.
    pub stall_reset_ms: u64,
}

impl Default for FrameStreamCfg {
    fn default() -> Self {
        Self {
            gap_timeout_ms: 50,
            lookahead: 8,
            stall_reset_ms: 500,
        }
    }
}

/// Where frames go. `tail` is the one piece of a split AU that may end
/// mid-NAL; by default it is plain data.
pub trait FrameOut {
    fn begin(&mut self, hdr: &FrameHdr, sid: u8, nslices: u8);
    fn data(&mut self, b: &[u8]);
    fn tail(&mut self, b: &[u8]) {
        self.data(b)
    }
    fn end(&mut self, complete: bool, lat: &AuLat);
}

#[derive(Default)]
struct Slot {
    sid: u8,
    fseq: u16,
    chunks: ChunkMap,
    count: u16,
    have_hdr: bool,
    hdr: FrameHdr,
    id64: u64,
    first_ms: u64,
    last_progress_ms: u64,
    last_arrival_ms: u64,
    emitted_upto: u16,
    max_idx: u16,
    seq_at_max: u32,
    have_seq_at_max: bool,
    began: bool,
    discont: bool,
    lat: AuLat,
    sa: Option<SliceAssembler>,
}

fn key(sid: u8, fseq: u16) -> u32 {
    (u32::from(sid) << 16) | u32::from(fseq)
}

#[derive(Clone, Copy, Debug, Default)]
pub struct FrameStreamStats {
    pub clean: u64,
    pub truncated: u64,
    pub dropped: u64,
    pub bad_frags: u64,
    pub stall_resets: u64,
    pub slice_salvaged: u64,
    pub slices_kept: u64,
    pub slices_filled: u64,
    pub slices_after_hole: u64,
    pub slice_fallback: [u64; SLICE_FB_COUNT],
}

/// A frame's repairs can all leave after its last symbol, so a hole near its
/// head waits for the rest of the frame to serialize, and the window-based
/// timeout only knows frames already seen. us_per_sym 0 turns this off.
#[derive(Clone, Copy, Debug, Default)]
struct FrameSpan {
    us_per_sym: [u32; 2],
    margin_ms: u32,
    cap_ms: u64,
}

impl FrameSpan {
    /// The wait for a hole in a frame with `left` symbols still to emit: a
    /// slot still missing its header counts the whole frame. Never shorter
    /// than the window-based timeout.
    fn wait_ms(&self, gap_ms: &[u64; 2], sid: u8, left: u16) -> u64 {
        let sid = if sid <= 1 { usize::from(sid) } else { 0 };
        let us = self.us_per_sym[sid];
        if us == 0 {
            return gap_ms[sid];
        }
        let span = u64::from(left) * u64::from(us) / 1000 + u64::from(self.margin_ms);
        gap_ms[sid].max(span.min(self.cap_ms))
    }
}

pub struct FrameStream {
    cfg: FrameStreamCfg,
    gap_ms: [u64; 2],
    span: FrameSpan,
    slots: BTreeMap<u32, Slot>,
    have_id_base: bool,
    last_id64: u64,
    next_emit_id64: u64,
    have_next_emit: bool,
    in_discont_run: bool,
    stall_armed: bool,
    stall_arm_ms: u64,
    discont_seen_since_emit: bool,
    last_stall_log_ms: u64,
    params: ParamTracker,
    st: FrameStreamStats,
}

impl FrameStream {
    pub fn new(cfg: FrameStreamCfg) -> Self {
        Self {
            cfg,
            gap_ms: [cfg.gap_timeout_ms; 2],
            span: FrameSpan::default(),
            slots: BTreeMap::new(),
            have_id_base: false,
            last_id64: 0,
            next_emit_id64: 0,
            have_next_emit: false,
            in_discont_run: false,
            stall_armed: false,
            stall_arm_ms: 0,
            discont_seen_since_emit: false,
            last_stall_log_ms: 0,
            params: ParamTracker::default(),
            st: FrameStreamStats::default(),
        }
    }

    pub fn stats(&self) -> &FrameStreamStats {
        &self.st
    }
    pub fn set_gap_timeout(&mut self, sid: usize, ms: u64) {
        if sid <= 1 {
            self.gap_ms[sid] = ms;
        }
    }
    pub fn set_gap_frame_span(&mut self, sid: usize, us_per_sym: u32, margin_ms: u32, cap_ms: u64) {
        if sid <= 1 {
            self.span.us_per_sym[sid] = us_per_sym;
            self.span.margin_ms = margin_ms;
            self.span.cap_ms = cap_ms;
        }
    }
    pub fn gap_ms(&self, sid: u8) -> u64 {
        self.gap_ms[if sid <= 1 { usize::from(sid) } else { 0 }]
    }
    fn gap_ms_max(&self) -> u64 {
        self.gap_ms[0].max(self.gap_ms[1])
    }

    pub fn push_fragment(
        &mut self,
        sid: u8,
        pkt: &[u8],
        now_ms: u64,
        arr: &FragArrival,
        out: &mut dyn FrameOut,
    ) {
        if pkt.len() < FRAG_HDR_LEN {
            self.st.bad_frags += 1;
            return;
        }
        if self.stall_armed
            && self.cfg.stall_reset_ms != 0
            && now_ms - self.stall_arm_ms >= self.cfg.stall_reset_ms
        {
            if self.last_stall_log_ms == 0 || now_ms - self.last_stall_log_ms >= 5000 {
                self.last_stall_log_ms = now_ms;
                eprintln!(
                    "pal: frame stall: no emit for {} ms with frames arriving (next_emit={} last_id64={} discont_seen={}) -> reset",
                    now_ms - self.stall_arm_ms,
                    self.next_emit_id64,
                    self.last_id64,
                    u8::from(self.discont_seen_since_emit)
                );
            }
            self.st.stall_resets += 1;
            self.reset(out);
        }
        let fseq = u16::from_le_bytes([pkt[0], pkt[1]]);
        let idx = u16::from_le_bytes([pkt[2], pkt[3]]);
        let count = u16::from_le_bytes([pkt[4], pkt[5]]);
        if count == 0 || idx >= count {
            self.st.bad_frags += 1;
            return;
        }
        let k = key(sid, fseq);
        let s = self.slots.entry(k).or_default();
        if s.chunks.is_empty() {
            s.sid = sid;
            s.fseq = fseq;
            s.first_ms = now_ms;
            s.last_progress_ms = now_ms;
            s.count = count;
        }
        s.chunks.insert(idx, pkt[FRAG_HDR_LEN..].to_vec());
        if let Some(seq) = arr.sw_seq {
            if !s.have_seq_at_max || idx > s.max_idx {
                s.max_idx = idx;
                s.seq_at_max = seq;
                s.have_seq_at_max = true;
            }
        }
        s.last_arrival_ms = now_ms;
        if arr.body_mono_us != 0 && (s.lat.t_first_us == 0 || arr.body_mono_us < s.lat.t_first_us) {
            s.lat.t_first_us = arr.body_mono_us;
        }
        if arr.body_mono_us > s.lat.t_last_arr_us {
            s.lat.t_last_arr_us = arr.body_mono_us;
        }
        if arr.push_mono_us != 0 && arr.push_mono_us >= s.lat.t_push_us {
            s.lat.t_push_us = arr.push_mono_us;
            s.lat.t_drain_us = arr.drain_mono_us;
        }
        if idx == 0 && !s.have_hdr {
            let Some(h) = FrameHdr::parse(&s.chunks[&0]) else {
                self.st.bad_frags += 1;
                self.slots.remove(&k);
                return;
            };
            s.have_hdr = true;
            s.lat.drone_q_ms = arr.q_ms;
            s.lat.enc_us = arr.enc_us;
            s.lat.drone_air_ms = arr.air_ms;
            s.lat.hdr_retx = arr.retx;
            s.hdr = h;
            let (id64, rebased) = self.unwrap_id(h.frame_id, h.flags);
            let s = self.slots.get_mut(&k).expect("slot");
            s.id64 = id64;
            s.discont = rebased;
            if rebased {
                self.params.reset(); // a producer restart may change resolution
            }
            if h.flags & FLAG_DISCONT != 0 {
                self.discont_seen_since_emit = true;
            }
            if self.have_next_emit && !self.stall_armed {
                self.stall_armed = true;
                self.stall_arm_ms = now_ms;
            }
        }
        self.try_emit(now_ms, out);
    }

    fn unwrap_id(&mut self, id: u16, flags: u8) -> (u64, bool) {
        let discont = flags & FLAG_DISCONT != 0;
        if !self.have_id_base || (discont && !self.in_discont_run) {
            // Re-base far above anything emitted; only the first flagged frame
            // of a discont run re-bases, the rest delta-track.
            self.have_id_base = true;
            let base = if self.have_next_emit {
                self.next_emit_id64 + 0x20000
            } else {
                0x10000
            } & !0xFFFF;
            self.last_id64 = base + u64::from(id);
            self.in_discont_run = discont;
            return (self.last_id64, true);
        }
        self.in_discont_run = discont;
        let d = id.wrapping_sub(self.last_id64 as u16) as i16;
        self.last_id64 = (self.last_id64 as i64 + i64::from(d)) as u64;
        (self.last_id64, false)
    }

    fn try_emit(&mut self, now_ms: u64, out: &mut dyn FrameOut) {
        loop {
            // Late arrivals behind the emit cursor are never emitted.
            if self.have_next_emit {
                let ne = self.next_emit_id64;
                let before = self.slots.len();
                self.slots
                    .retain(|_, s| !(s.have_hdr && s.id64 < ne && !s.began));
                self.st.dropped += (before - self.slots.len()) as u64;
            }
            // Head of line: the known-header slot with the lowest id64.
            let mut head: Option<(u32, u64)> = None;
            let mut max_known = 0u64;
            for (k, s) in &self.slots {
                if !s.have_hdr {
                    continue;
                }
                max_known = max_known.max(s.id64);
                if head.is_none_or(|(_, id)| s.id64 < id) {
                    head = Some((*k, s.id64));
                }
            }
            let Some((hk, _)) = head else { return };
            let mut h = self.slots.remove(&hk).expect("head");
            if self.have_next_emit && h.id64 > self.next_emit_id64 {
                if h.discont {
                    // Synthetic jump of a re-base, not lost frames.
                    self.next_emit_id64 = h.id64;
                } else {
                    let stale = now_ms >= h.first_ms + self.gap_ms_max();
                    let ahead = max_known >= self.next_emit_id64 + self.cfg.lookahead;
                    if !stale && !ahead {
                        self.slots.insert(hk, h);
                        return;
                    }
                    self.st.dropped += h.id64 - self.next_emit_id64;
                    h.lat.skipped_before += (h.id64 - self.next_emit_id64) as u32;
                    self.next_emit_id64 = h.id64;
                }
            }
            if !self.have_next_emit {
                self.have_next_emit = true;
                self.next_emit_id64 = h.id64;
            }
            if !h.began {
                h.began = true;
                if h.hdr.slice_rows > 0 && self.params.usable() {
                    h.sa = Some(SliceAssembler::new(
                        self.params.sps(),
                        self.params.pps(),
                        h.hdr.slice_rows,
                        h.count,
                        HDR_LEN,
                    ));
                }
                let ns = h.sa.as_ref().map_or(1, SliceAssembler::slices);
                out.begin(&h.hdr, h.sid, ns);
            }
            while let Some(c) = h.chunks.get(&h.emitted_upto) {
                if h.sa.is_none() {
                    let skip = if h.emitted_upto == 0 { HDR_LEN } else { 0 };
                    if c.len() > skip {
                        out.data(&c[skip..]);
                    }
                }
                h.emitted_upto += 1;
                h.last_progress_ms = now_ms;
            }
            if let Some(sa) = h.sa.as_mut() {
                sa.drain(&h.chunks, &mut |b| out.data(b));
            }
            if h.emitted_upto == h.count {
                self.finish(h, true, out);
                continue;
            }
            let left = h.count.saturating_sub(h.emitted_upto);
            let stale = now_ms >= h.last_progress_ms + self.span.wait_ms(&self.gap_ms, h.sid, left);
            let ahead = max_known >= h.id64 + self.cfg.lookahead;
            if stale || ahead {
                self.finish(h, false, out);
                continue;
            }
            self.slots.insert(hk, h);
            return;
        }
    }

    fn finish(&mut self, mut s: Slot, complete: bool, out: &mut dyn FrameOut) {
        if let Some(sa) = s.sa.as_mut() {
            // Two sinks into one `out`: route through a small enum buffer.
            let mut pieces: Vec<(bool, Vec<u8>)> = Vec::new();
            {
                let cell = std::cell::RefCell::new(&mut pieces);
                sa.finish(
                    &s.chunks,
                    &mut |b| cell.borrow_mut().push((false, b.to_vec())),
                    &mut |b| cell.borrow_mut().push((true, b.to_vec())),
                );
            }
            for (tail, b) in &pieces {
                if *tail {
                    out.tail(b)
                } else {
                    out.data(b)
                }
            }
            s.lat.slice = *sa.result();
        } else if !complete && s.hdr.slice_rows > 0 {
            s.lat.slice.fallback = if self.params.unsupported() {
                SliceFallback::Unsupported
            } else {
                SliceFallback::NoParams
            };
        }
        if complete {
            self.feed_params(&mut s);
        }
        if s.lat.slice.salvaged {
            self.st.slice_salvaged += 1;
            self.st.slices_kept += u64::from(s.lat.slice.kept);
            self.st.slices_filled += u64::from(s.lat.slice.filled);
            self.st.slices_after_hole += u64::from(s.lat.slice.kept_after_hole);
        } else if !complete && s.lat.slice.fallback != SliceFallback::None {
            self.st.slice_fallback[s.lat.slice.fallback as usize] += 1;
        }
        out.end(complete, &s.lat);
        if complete {
            self.st.clean += 1;
        } else {
            self.st.truncated += 1;
        }
        self.next_emit_id64 = s.id64 + 1;
        self.stall_armed = false;
        self.discont_seen_since_emit = false;
    }

    /// A complete AU opening with a parameter set feeds the salvage tracker;
    /// only the leading parameter sets are copied, never the slice data.
    fn feed_params(&mut self, s: &mut Slot) {
        let Some(b) = s.chunks.get(&0) else { return };
        let h = HDR_LEN;
        let p = if b.len() >= h + 5 && b[h..h + 4] == [0, 0, 0, 1] {
            h + 4
        } else if b.len() >= h + 4 && b[h..h + 3] == [0, 0, 1] {
            h + 3
        } else {
            return;
        };
        let ty = hevc::nal::nal_type(b[p]);
        if !(32..=34).contains(&ty) {
            return;
        }
        let mut head = b[h..].to_vec();
        let vcl_started = |head: &[u8]| split_nals(head).iter().any(|v| v.ty < 32);
        let mut i = 1u16;
        while i < s.count && !vcl_started(&head) {
            let Some(c) = s.chunks.get(&i) else { return };
            head.extend_from_slice(c);
            i += 1;
        }
        if let Some(v) = split_nals(&head).iter().find(|v| v.ty < 32) {
            s.lat.vcl_nal = v.ty;
        }
        self.params.feed(&head);
    }

    /// Drives timeouts; call every tick.
    pub fn poll(&mut self, now_ms: u64, out: &mut dyn FrameOut) {
        let gap = self.gap_ms;
        let span = self.span;
        let before = self.slots.len();
        self.slots.retain(|_, s| {
            let left = s.count.saturating_sub(s.emitted_upto);
            s.have_hdr || now_ms < s.first_ms + span.wait_ms(&gap, s.sid, left)
        });
        self.st.dropped += (before - self.slots.len()) as u64;
        self.try_emit(now_ms, out);
    }

    pub fn tail_view(&self, sid: u8) -> Option<TailView> {
        self.slots
            .values()
            .filter(|s| s.sid == sid && s.have_hdr && s.have_seq_at_max && s.emitted_upto < s.count)
            .fold(None::<&Slot>, |b, s| match b {
                Some(b) if b.id64 >= s.id64 => Some(b),
                _ => Some(s),
            })
            .map(|b| TailView {
                count: b.count,
                max_idx: b.max_idx,
                seq_at_max: b.seq_at_max,
                last_progress_ms: b.last_arrival_ms,
                first_ms: b.first_ms,
            })
    }

    /// Session change: close begun frames as truncated without touching the
    /// counters, drop everything, forget the parameter sets.
    pub fn reset(&mut self, out: &mut dyn FrameOut) {
        for s in self.slots.values() {
            if s.began {
                out.end(false, &s.lat);
            }
        }
        self.slots.clear();
        self.have_id_base = false;
        self.last_id64 = 0;
        self.next_emit_id64 = 0;
        self.have_next_emit = false;
        self.in_discont_run = false;
        self.stall_armed = false;
        self.discont_seen_since_emit = false;
        self.params.reset();
    }
}

#[cfg(test)]
mod frame_span_tests {
    use super::FrameSpan;

    fn span(us_per_sym: u32, cap_ms: u64) -> FrameSpan {
        FrameSpan {
            us_per_sym: [us_per_sym, 0],
            margin_ms: 15,
            cap_ms,
        }
    }

    #[test]
    fn a_big_frame_waits_for_its_own_send_time() {
        let gap = [50, 50];
        assert_eq!(span(1000, 1000).wait_ms(&gap, 0, 90), 105); // the time left to send it
        assert_eq!(span(1000, 80).wait_ms(&gap, 0, 90), 80); // capped
        assert_eq!(span(100, 1000).wait_ms(&gap, 0, 90), 50); // never below the timeout
        assert_eq!(span(0, 1000).wait_ms(&gap, 0, 90), 50); // off
        assert_eq!(span(1000, 1000).wait_ms(&gap, 1, 90), 50); // sids are independent
    }
}
