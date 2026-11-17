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
    /// Some fragment of this AU was new when it arrived out of a NACK
    /// retransmit body (directly, or recovered by FEC on that body's
    /// symbol): a complete base AU with it is one the NACK may have saved.
    /// An upper bound: later repairs could have recovered the same symbols.
    pub retx_fill: bool,
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
    /// Frames emitted when the header arrived.
    emits_at_hdr: u64,
    lat: AuLat,
    sa: Option<SliceAssembler>,
}

fn key(sid: u8, fseq: u16) -> u32 {
    (u32::from(sid) << 16) | u32::from(fseq)
}

/// The slice assembler's remainder into `out`: its slices as data, a
/// passthrough remainder as tail, in the order it produces them.
fn drain_slices(sa: &mut SliceAssembler, chunks: &ChunkMap, out: &mut dyn FrameOut) {
    let out = std::cell::RefCell::new(out);
    sa.finish(chunks, &mut |b| out.borrow_mut().data(b), &mut |b| {
        out.borrow_mut().tail(b)
    });
}

/// Length of the Annex B start code at `at` (4 or 3 bytes), if one is there
/// with at least one byte after it.
fn start_code_len(b: &[u8], at: usize) -> Option<usize> {
    if b.len() >= at + 5 && b[at..at + 4] == [0, 0, 0, 1] {
        Some(4)
    } else if b.len() >= at + 4 && b[at..at + 3] == [0, 0, 1] {
        Some(3)
    } else {
        None
    }
}

/// A frame that opens with a parameter set (VPS/SPS/PPS): its bytes from
/// the first chunk on, through the first chunk where a VCL NAL starts. None
/// when it opens otherwise or a chunk before that is missing.
fn leading_param_sets(s: &Slot) -> Option<Vec<u8>> {
    let b = s.chunks.get(&0)?;
    let sc = start_code_len(b, HDR_LEN)?;
    if !(32..=34).contains(&hevc::nal::nal_type(b[HDR_LEN + sc])) {
        return None;
    }
    let mut head = b[HDR_LEN..].to_vec();
    let vcl_started = |head: &[u8]| split_nals(head).iter().any(|v| v.ty < 32);
    let mut i = 1u16;
    while i < s.count && !vcl_started(&head) {
        head.extend_from_slice(s.chunks.get(&i)?);
        i += 1;
    }
    Some(head)
}

/// What try_emit does with a head above the emit cursor.
enum Gap {
    Go,
    Hold,
    DropHead,
}

/// The fragment header: frame seq, chunk index, chunk count.
#[derive(Clone, Copy)]
struct FragHead {
    fseq: u16,
    idx: u16,
    count: u16,
}

impl FragHead {
    /// `pkt` holds at least FRAG_HDR_LEN bytes.
    fn parse(pkt: &[u8]) -> Self {
        Self {
            fseq: u16::from_le_bytes([pkt[0], pkt[1]]),
            idx: u16::from_le_bytes([pkt[2], pkt[3]]),
            count: u16::from_le_bytes([pkt[4], pkt[5]]),
        }
    }
}

impl AuLat {
    /// The earliest and latest body arrivals, and the newest push stamp
    /// with its drain stamp.
    fn note_arrival(&mut self, arr: &FragArrival) {
        if arr.body_mono_us != 0 && (self.t_first_us == 0 || arr.body_mono_us < self.t_first_us) {
            self.t_first_us = arr.body_mono_us;
        }
        if arr.body_mono_us > self.t_last_arr_us {
            self.t_last_arr_us = arr.body_mono_us;
        }
        if arr.push_mono_us != 0 && arr.push_mono_us >= self.t_push_us {
            self.t_push_us = arr.push_mono_us;
            self.t_drain_us = arr.drain_mono_us;
        }
    }
}

impl Slot {
    /// One chunk in: the slot opens on its first, a retransmit that fills a
    /// hole marks the frame, the SW seq of the highest index is kept.
    fn add_chunk(&mut self, sid: u8, fh: FragHead, data: &[u8], arr: &FragArrival, now_ms: u64) {
        if self.chunks.is_empty() {
            self.sid = sid;
            self.fseq = fh.fseq;
            self.first_ms = now_ms;
            self.last_progress_ms = now_ms;
            self.count = fh.count;
        }
        if arr.retx && !self.chunks.contains_key(&fh.idx) {
            self.lat.retx_fill = true;
        }
        self.chunks.insert(fh.idx, data.to_vec());
        if let Some(seq) = arr.sw_seq {
            if !self.have_seq_at_max || fh.idx > self.max_idx {
                self.max_idx = fh.idx;
                self.seq_at_max = seq;
                self.have_seq_at_max = true;
            }
        }
        self.last_arrival_ms = now_ms;
        self.lat.note_arrival(arr);
    }

    /// Emits the chunks contiguous from the cursor (through the slice
    /// assembler when there is one; the frame header skipped otherwise).
    fn emit_ready(&mut self, now_ms: u64, out: &mut dyn FrameOut) {
        while let Some(c) = self.chunks.get(&self.emitted_upto) {
            if self.sa.is_none() {
                let skip = if self.emitted_upto == 0 { HDR_LEN } else { 0 };
                if c.len() > skip {
                    out.data(&c[skip..]);
                }
            }
            self.emitted_upto += 1;
            self.last_progress_ms = now_ms;
        }
        if let Some(sa) = self.sa.as_mut() {
            sa.drain(&self.chunks, &mut |b| out.data(b));
        }
    }
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
        self.check_stall(now_ms, out);
        let fh = FragHead::parse(pkt);
        if fh.count == 0 || fh.idx >= fh.count {
            self.st.bad_frags += 1;
            return;
        }
        let k = key(sid, fh.fseq);
        let s = self.slots.entry(k).or_default();
        s.add_chunk(sid, fh, &pkt[FRAG_HDR_LEN..], arr, now_ms);
        if fh.idx == 0 && !s.have_hdr && !self.take_header(k, arr, now_ms) {
            return;
        }
        self.try_emit(now_ms, out);
    }

    /// Frames arriving with nothing emitted for stall_reset_ms: reset.
    fn check_stall(&mut self, now_ms: u64, out: &mut dyn FrameOut) {
        if !(self.stall_armed
            && self.cfg.stall_reset_ms != 0
            && now_ms - self.stall_arm_ms >= self.cfg.stall_reset_ms)
        {
            return;
        }
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

    /// The first chunk of a frame carries its header: parse it, stamp the
    /// slot, unwrap the id. False when it does not parse (the slot goes).
    fn take_header(&mut self, k: u32, arr: &FragArrival, now_ms: u64) -> bool {
        let s = self.slots.get_mut(&k).expect("slot");
        let Some(h) = FrameHdr::parse(&s.chunks[&0]) else {
            self.st.bad_frags += 1;
            self.slots.remove(&k);
            return false;
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
        s.emits_at_hdr = self.st.clean + self.st.truncated;
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
        true
    }

    fn unwrap_id(&mut self, id: u16, flags: u8) -> (u64, bool) {
        let discont = flags & FLAG_DISCONT != 0;
        let d = id.wrapping_sub(self.last_id64 as u16) as i16;
        // Ids that step back past zero (the first base is 0x10000, so 0x10000
        // net backwards; only forged or corrupt headers do that) re-base like a
        // producer restart instead of wrapping the id, the gap count and the
        // emit cursor around 2^64.
        let below_zero = d < 0 && u64::from(d.unsigned_abs()) > self.last_id64;
        if !self.have_id_base || (discont && !self.in_discont_run) || below_zero {
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
        self.last_id64 = (self.last_id64 as i64 + i64::from(d)) as u64;
        (self.last_id64, false)
    }

    fn try_emit(&mut self, now_ms: u64, out: &mut dyn FrameOut) {
        loop {
            self.drop_late();
            let Some((hk, max_known)) = self.find_head() else {
                return;
            };
            let head_lone = self.lone(&self.slots[&hk]);
            let mut h = self.slots.remove(&hk).expect("head");
            match self.cross_gap(&mut h, head_lone, max_known, now_ms) {
                Gap::Hold => {
                    self.slots.insert(hk, h);
                    return;
                }
                Gap::DropHead => {
                    self.st.dropped += 1;
                    continue;
                }
                Gap::Go => {}
            }
            if !self.have_next_emit {
                self.have_next_emit = true;
                self.next_emit_id64 = h.id64;
            }
            if !h.began {
                self.begin_frame(&mut h, out);
            }
            h.emit_ready(now_ms, out);
            if h.emitted_upto == h.count {
                self.finish(h, true, out);
                continue;
            }
            if self.expired(&h, max_known, now_ms) {
                self.finish(h, false, out);
                continue;
            }
            self.slots.insert(hk, h);
            return;
        }
    }

    /// Late arrivals behind the emit cursor are never emitted.
    fn drop_late(&mut self) {
        if !self.have_next_emit {
            return;
        }
        let ne = self.next_emit_id64;
        let before = self.slots.len();
        self.slots
            .retain(|_, s| !(s.have_hdr && s.id64 < ne && !s.began));
        self.st.dropped += (before - self.slots.len()) as u64;
    }

    /// Head of line: the known-header slot with the lowest id64. Of two
    /// slots with one id (a repeated frame id on another fseq or layer:
    /// forged, or a corrupt header the block CRC let through) the one
    /// already begun stays the head, so a second frame never begins
    /// inside an open one; the other is late once the head finishes.
    /// Returns the head's key and the highest id64 among the frames that
    /// are not lone (see `lone`): those do not count as the stream having
    /// run ahead.
    fn find_head(&self) -> Option<(u32, u64)> {
        let mut head: Option<(u32, u64)> = None;
        let mut max_known = 0u64;
        for (k, s) in &self.slots {
            if !s.have_hdr {
                continue;
            }
            if s.id64 > max_known && !self.lone(s) {
                max_known = s.id64;
            }
            if head.is_none_or(|(_, id)| s.id64 < id || (s.id64 == id && s.began)) {
                head = Some((*k, s.id64));
            }
        }
        head.map(|(k, _)| (k, max_known))
    }

    /// A head above the emit cursor: a re-base jumps to it; otherwise the
    /// frames between are skipped once the head is stale or (not lone) the
    /// stream ran lookahead frames ahead, and held until then.
    fn cross_gap(&mut self, h: &mut Slot, head_lone: bool, max_known: u64, now_ms: u64) -> Gap {
        if !self.have_next_emit || h.id64 <= self.next_emit_id64 {
            return Gap::Go;
        }
        if h.discont {
            // Synthetic jump of a re-base, not lost frames.
            self.next_emit_id64 = h.id64;
            return Gap::Go;
        }
        let stale = now_ms >= h.first_ms + self.gap_ms_max();
        if head_lone {
            if !stale {
                return Gap::Hold;
            }
            // Frames were emitted after this one arrived, so the
            // stream is alive below it: the far id is forged or
            // corrupt, not the next frame after an outage.
            // Skipping to it would make every real frame late
            // until the stall watchdog fired.
            if self.st.clean + self.st.truncated != h.emits_at_hdr {
                return Gap::DropHead;
            }
        } else {
            let ahead = max_known >= self.next_emit_id64 + self.cfg.lookahead;
            if !stale && !ahead {
                return Gap::Hold;
            }
        }
        self.st.dropped += h.id64 - self.next_emit_id64;
        h.lat.skipped_before += (h.id64 - self.next_emit_id64) as u32;
        self.next_emit_id64 = h.id64;
        Gap::Go
    }

    /// The frame's begin: a slice assembler when the frame is sliced and the
    /// parameter sets are known, then out.begin with its slice count.
    fn begin_frame(&mut self, h: &mut Slot, out: &mut dyn FrameOut) {
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

    /// An open frame ends truncated when it made no progress for its wait,
    /// or the stream ran lookahead frames past it.
    fn expired(&self, h: &Slot, max_known: u64, now_ms: u64) -> bool {
        let left = h.count.saturating_sub(h.emitted_upto);
        let stale = now_ms >= h.last_progress_ms + self.span.wait_ms(&self.gap_ms, h.sid, left);
        let ahead = max_known >= h.id64 + self.cfg.lookahead;
        stale || ahead
    }

    /// A frame at least lookahead ids past the emit cursor, without the
    /// discont flag, with no other known frame within lookahead ids of it.
    /// Video carries no key tag, so one forged or corrupt header can name any
    /// id; a real stream that resumes after an outage confirms the jump with
    /// its next frame.
    fn lone(&self, s: &Slot) -> bool {
        let la = self.cfg.lookahead;
        if !self.have_next_emit || s.discont || la == 0 || s.id64 < self.next_emit_id64 + la {
            return false;
        }
        !self
            .slots
            .values()
            .any(|t| t.have_hdr && t.id64 != s.id64 && t.id64.abs_diff(s.id64) <= la)
    }

    fn finish(&mut self, mut s: Slot, complete: bool, out: &mut dyn FrameOut) {
        if let Some(sa) = s.sa.as_mut() {
            drain_slices(sa, &s.chunks, out);
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
        self.count_slices(&s.lat, complete);
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

    /// The slice outcome of a finished frame into the stats.
    fn count_slices(&mut self, lat: &AuLat, complete: bool) {
        if lat.slice.salvaged {
            self.st.slice_salvaged += 1;
            self.st.slices_kept += u64::from(lat.slice.kept);
            self.st.slices_filled += u64::from(lat.slice.filled);
            self.st.slices_after_hole += u64::from(lat.slice.kept_after_hole);
        } else if !complete && lat.slice.fallback != SliceFallback::None {
            self.st.slice_fallback[lat.slice.fallback as usize] += 1;
        }
    }

    /// A complete AU opening with a parameter set feeds the salvage tracker;
    /// only the leading parameter sets are copied, never the slice data.
    fn feed_params(&mut self, s: &mut Slot) {
        let Some(head) = leading_param_sets(s) else {
            return;
        };
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
