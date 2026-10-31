//! Streaming sliding-window decoder, a line-for-line port of mabur
//! `common/src/sw_decoder.cpp` (pin 51cea32f). Map iteration order, cascade
//! order (LIFO queue) and every counter follow the C++ so packet output and
//! stats match it byte for byte; the differential gate checks that.
//!
//! Sources deliver immediately; repairs reduce against known symbols and join
//! an incremental Gaussian elimination over the missing seqs. Wire u32 seqs
//! are unwrapped to u64 virtual seqs anchored at 2^32, so `newest - horizon`
//! never underflows once anchored.

use crate::arrival::{ArrivalTracker, DEFAULT_GUARD};
use crate::gf256 as gf;
use pal_wire::sw;
use std::collections::{BTreeMap, BTreeSet, VecDeque};

/// A source seq this far from the newest is an encoder restart, not wrap.
const RESET_SPAN: u64 = 1 << 20;
const ANCHOR: u64 = 1 << 32;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SwConfig {
    pub symbol_size: usize,
    pub window: usize,
    pub overhead: f64,
}

impl Default for SwConfig {
    fn default() -> Self {
        Self {
            symbol_size: 64,
            window: 128,
            overhead: 0.25,
        }
    }
}

/// Transition-boundary hint of the air frame that carried an envelope.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Boundary {
    #[default]
    None,
    Pre,
    Post,
}

/// One loss episode (fec.log gauge); see mabur `LossEpisode`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LossEpisode {
    pub first_seq: u64,
    pub span: u32,
    pub missing: u32,
    pub recovered: u32,
    pub retx: u32,
    pub abandoned: u32,
    pub stale: u32,
    pub repairs: u32,
    pub window: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SourceState {
    Unknown,
    Direct,
    Recovered,
    Retx,
    BelowFloor,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Origin {
    Source,
    Repair,
    Retx,
}

struct Row {
    coeffs: BTreeMap<u64, u8>,
    payload: Vec<u8>,
    first_seen_ms: u64,
}

#[derive(Clone, Copy)]
struct RepairSpan {
    ws: u64,
    we: u64,
    key: u32,
}

pub const MAX_QUEUED_EPISODES: usize = 1024;

#[derive(Default, Clone, Copy, Debug)]
pub struct DecoderStats {
    pub syms_delivered: u64,
    pub syms_recovered: u64,
    pub syms_recovered_arrived: u64,
    pub syms_abandoned: u64,
    pub syms_abandoned_stale: u64,
    pub syms_retx: u64,
    pub syms_retx_arrived: u64,
    pub symbols_in: u64,
    pub symbols_dropped_bad_cfg: u64,
    pub symbols_dropped_stale: u64,
    pub repairs_skipped: u64,
    pub packets_out: u64,
    pub resets: u64,
}

pub struct SwDecoder {
    cfg: SwConfig,
    horizon: u64,
    have_seq: bool,
    newest_v: u64,
    base: u64,
    known: BTreeMap<u64, Vec<u8>>,
    rows: BTreeMap<u64, Row>,
    recovered_await_src: BTreeSet<u64>,
    retx_await_src: BTreeSet<u64>,
    last_out_seqs: Vec<u32>,
    st: DecoderStats,
    repair_window_hwm: usize,
    wm_open: bool,
    wm_valid: bool,
    wm: u64,
    repairs_seen: VecDeque<RepairSpan>,
    ep: LossEpisode,
    ep_last: u64,
    ep_open: bool,
    episodes: Vec<LossEpisode>,
    arr_guard: u32,
    arr: ArrivalTracker,
}

impl SwDecoder {
    /// seq_horizon 0 = 4 * window; arrival_guard 0 = 32.
    pub fn new(cfg: SwConfig, seq_horizon: u32, arrival_guard: u32) -> Self {
        let horizon = if seq_horizon != 0 {
            u64::from(seq_horizon)
        } else {
            4 * cfg.window as u64
        };
        let arr_guard = if arrival_guard != 0 {
            arrival_guard
        } else {
            DEFAULT_GUARD
        };
        Self {
            cfg,
            horizon,
            have_seq: false,
            newest_v: 0,
            base: 0,
            known: BTreeMap::new(),
            rows: BTreeMap::new(),
            recovered_await_src: BTreeSet::new(),
            retx_await_src: BTreeSet::new(),
            last_out_seqs: Vec::new(),
            st: DecoderStats::default(),
            repair_window_hwm: 0,
            wm_open: false,
            wm_valid: false,
            wm: 0,
            repairs_seen: VecDeque::new(),
            ep: LossEpisode::default(),
            ep_last: 0,
            ep_open: false,
            episodes: Vec::new(),
            arr_guard,
            arr: ArrivalTracker::new(arr_guard),
        }
    }

    pub fn config(&self) -> &SwConfig {
        &self.cfg
    }
    pub fn stats(&self) -> &DecoderStats {
        &self.st
    }
    pub fn arrival(&self) -> &ArrivalTracker {
        &self.arr
    }
    pub fn arrival_guard(&self) -> u32 {
        self.arr_guard
    }
    /// Wire seq of the symbol each packet of the last `add_symbol` came from.
    pub fn last_out_seqs(&self) -> &[u32] {
        &self.last_out_seqs
    }
    pub fn newest_seq(&self) -> u64 {
        if self.have_seq {
            self.newest_v
        } else {
            0
        }
    }
    /// Largest repair window seen: the TX window as flown.
    pub fn repair_window(&self) -> usize {
        self.repair_window_hwm
    }
    pub fn rows_in_flight(&self) -> usize {
        self.rows.len()
    }
    pub fn boundary_open(&self) -> bool {
        self.wm_open
    }
    pub fn close_boundary(&mut self) {
        self.wm_open = false;
    }

    fn unwrap(&self, s: u32) -> u64 {
        let d = s.wrapping_sub(self.newest_v as u32) as i32 as i64;
        self.newest_v.wrapping_add(d as u64)
    }

    fn live_floor(&self) -> u64 {
        self.newest_v.wrapping_sub(self.horizon)
    }

    fn arr_stale_end(&self) -> u64 {
        if self.wm_open {
            u64::MAX
        } else if self.wm_valid {
            self.wm + 1
        } else {
            0
        }
    }

    fn reset_state(&mut self, v: u64) {
        self.known.clear();
        self.rows.clear();
        self.recovered_await_src.clear();
        self.retx_await_src.clear();
        self.newest_v = v;
        self.base = v;
        self.arr.reset(v);
        self.st.resets += 1;
        self.wm_open = false;
        self.wm_valid = false;
        self.repairs_seen.clear();
        self.ep_open = false;
    }

    pub fn mark_transition(&mut self) {
        if !self.have_seq {
            self.wm_open = false;
            self.wm_valid = false;
            return;
        }
        self.wm = self.newest_v;
        self.wm_valid = true;
        self.wm_open = true;
    }

    /// Wire seqs in (newest - lookback, newest) above the floor and unknown.
    pub fn missing_sources(&self, lookback: u32) -> Vec<u32> {
        let mut out = Vec::new();
        if !self.have_seq {
            return out;
        }
        let floor = self.live_floor();
        let mut from = if self.newest_v > u64::from(lookback) {
            self.newest_v - u64::from(lookback)
        } else {
            0
        };
        from = from.max(floor).max(self.base);
        for v in from..self.newest_v {
            if !self.known.contains_key(&v) {
                out.push(v as u32);
            }
        }
        out
    }

    pub fn source_state(&self, wire_seq: u32) -> SourceState {
        if !self.have_seq {
            return SourceState::Unknown;
        }
        let v = self.unwrap(wire_seq);
        if v < self.live_floor() || v < self.base {
            return SourceState::BelowFloor;
        }
        if !self.known.contains_key(&v) {
            return SourceState::Unknown;
        }
        if self.retx_await_src.contains(&v) {
            return SourceState::Retx;
        }
        if self.recovered_await_src.contains(&v) {
            SourceState::Recovered
        } else {
            SourceState::Direct
        }
    }

    fn advance(&mut self, newest_candidate: u64) {
        if newest_candidate <= self.newest_v {
            return;
        }
        self.newest_v = newest_candidate;
        let nb = self.newest_v.wrapping_sub(self.horizon);
        if nb <= self.base {
            return;
        }
        let stale_end = if self.wm_open {
            nb
        } else if self.wm_valid {
            nb.min(self.wm + 1)
        } else {
            self.base
        };
        for s in self.base..nb {
            let known = self.known.contains_key(&s);
            let rec = known && self.recovered_await_src.contains(&s);
            let rtx = known && self.retx_await_src.contains(&s);
            if known && !rec && !rtx {
                continue;
            }
            let via = if rtx {
                Origin::Retx
            } else if rec {
                Origin::Repair
            } else {
                Origin::Source
            };
            self.note_missing(s, via, s < stale_end);
        }
        self.settle_episodes(nb);
        let mut known_in_range = 0u64;
        let mut known_stale = 0u64;
        let keep = self.known.split_off(&nb);
        for &k in self.known.keys() {
            if k >= self.base {
                known_in_range += 1;
                if k < stale_end {
                    known_stale += 1;
                }
            }
        }
        self.known = keep;
        self.st.syms_abandoned += (nb - self.base) - known_in_range;
        if stale_end > self.base {
            self.st.syms_abandoned_stale += (stale_end - self.base) - known_stale;
        }
        self.recovered_await_src = self.recovered_await_src.split_off(&nb);
        self.retx_await_src = self.retx_await_src.split_off(&nb);
        self.rows = self.rows.split_off(&nb);
        self.base = nb;
    }

    fn unpack_symbol(&mut self, sym: &[u8], out: &mut Vec<Vec<u8>>) {
        let ss = self.cfg.symbol_size;
        let mut pos = 0;
        while pos + 2 <= ss {
            let ln = u16::from_le_bytes([sym[pos], sym[pos + 1]]) as usize;
            if ln == 0 {
                break;
            }
            let end = pos + 2 + ln;
            if end > ss {
                break;
            }
            out.push(sym[pos + 2..end].to_vec());
            self.st.packets_out += 1;
            pos = end;
        }
    }

    fn insert_row(&mut self, mut r: Row, solved: &mut Vec<(u64, Vec<u8>)>) {
        let ss = self.cfg.symbol_size;
        loop {
            let Some((&pivot, &lead)) = r.coeffs.iter().next() else {
                return; // linearly dependent
            };
            let Some(e) = self.rows.get(&pivot) else {
                if lead != 1 {
                    let ilead = gf::inv(lead);
                    for c in r.coeffs.values_mut() {
                        *c = gf::mul(*c, ilead);
                    }
                    let mut scaled = vec![0u8; ss];
                    gf::lincomb(&mut scaled, &r.payload, ilead);
                    r.payload = scaled;
                }
                if r.coeffs.len() == 1 {
                    solved.push((pivot, r.payload));
                    return;
                }
                self.rows.insert(pivot, r);
                return;
            };
            let f = lead;
            for (&s, &c) in &e.coeffs {
                let cur = r.coeffs.get(&s).copied();
                let nv = cur.unwrap_or(0) ^ gf::mul(f, c);
                if nv != 0 {
                    r.coeffs.insert(s, nv);
                } else if cur.is_some() {
                    r.coeffs.remove(&s);
                }
            }
            gf::lincomb(&mut r.payload, &e.payload, f);
        }
    }

    fn ingest(&mut self, v: u64, sym: Vec<u8>, origin: Origin, out: &mut Vec<Vec<u8>>) {
        let mut queue: Vec<(u64, Vec<u8>)> = vec![(v, sym)];
        let mut first = true;
        while let Some((s, payload)) = queue.pop() {
            let o = if first {
                origin
            } else if origin == Origin::Retx {
                Origin::Retx
            } else {
                Origin::Repair
            };
            first = false;
            if s < self.live_floor() || self.known.contains_key(&s) {
                continue;
            }
            self.unpack_symbol(&payload, out);
            while self.last_out_seqs.len() < out.len() {
                self.last_out_seqs.push(s as u32);
            }
            match o {
                Origin::Source => self.st.syms_delivered += 1,
                Origin::Retx => {
                    self.st.syms_retx += 1;
                    self.retx_await_src.insert(s);
                }
                Origin::Repair => {
                    self.st.syms_recovered += 1;
                    self.recovered_await_src.insert(s);
                }
            }
            // Rows referencing s, in pivot order (as the C++ map walk).
            let hit: Vec<u64> = self
                .rows
                .iter()
                .filter(|(_, r)| r.coeffs.contains_key(&s))
                .map(|(&k, _)| k)
                .collect();
            let affected: Vec<Row> = hit.iter().filter_map(|k| self.rows.remove(k)).collect();
            self.known.insert(s, payload);
            for mut r in affected {
                let c = r.coeffs.remove(&s).unwrap_or(0);
                gf::lincomb(&mut r.payload, &self.known[&s], c);
                let mut solved = Vec::new();
                self.insert_row(r, &mut solved);
                queue.extend(solved);
            }
        }
    }

    /// Feeds one envelope; returns the app packets of every symbol that became
    /// known (source first, cascades after). `clean`: the carrying body's FCS
    /// verdict. `retx`: the body is a NACK retransmit.
    pub fn add_symbol(
        &mut self,
        env: &[u8],
        now_ms: u64,
        mut b: Boundary,
        clean: bool,
        retx: bool,
    ) -> Vec<Vec<u8>> {
        self.last_out_seqs.clear();
        let mut out = Vec::new();
        let Some(h) = sw::Header::parse(env) else {
            return out;
        };
        let ss = self.cfg.symbol_size;
        if usize::from(h.symbol_size) != ss || env.len() != sw::HEADER_LEN + ss {
            self.st.symbols_dropped_bad_cfg += 1;
            return out;
        }
        self.st.symbols_in += 1;
        let payload = &env[sw::HEADER_LEN..];

        if !h.repair {
            if !self.have_seq {
                self.have_seq = true;
                self.newest_v = ANCHOR + u64::from(h.seq);
                self.base = self.newest_v;
            }
            let mut v = self.unwrap(h.seq);
            if v > self.newest_v.wrapping_add(RESET_SPAN)
                || v.wrapping_add(RESET_SPAN) < self.newest_v
            {
                self.reset_state(ANCHOR + u64::from(h.seq));
                v = self.newest_v;
            }
            if retx {
                b = Boundary::None;
            }
            if b == Boundary::Pre && self.wm_valid && v > self.wm {
                self.wm = v;
            }
            if b == Boundary::Post && self.wm_open {
                self.wm_open = false;
                if self.wm_valid && v > 0 && v - 1 > self.wm {
                    self.wm = v - 1;
                }
            }
            if !retx {
                let se = self.arr_stale_end();
                self.arr.on_source(v, se, clean);
                let se = self.arr_stale_end();
                self.arr.advance(v, se);
            }
            if v < self.live_floor() || self.known.contains_key(&v) {
                if retx {
                    self.st.symbols_dropped_stale += 1;
                    return out;
                }
                if self.recovered_await_src.remove(&v) {
                    self.st.syms_recovered_arrived += 1;
                } else if self.retx_await_src.remove(&v) {
                    self.st.syms_retx_arrived += 1;
                } else {
                    self.st.symbols_dropped_stale += 1;
                }
                return out;
            }
            self.advance(v);
            let origin = if retx { Origin::Retx } else { Origin::Source };
            self.ingest(v, payload.to_vec(), origin, &mut out);
            return out;
        }

        if !self.have_seq {
            self.st.symbols_dropped_stale += 1;
            return out;
        }
        let ws = self.unwrap(h.seq);
        let wl = usize::from(h.window_len);
        self.repair_window_hwm = self.repair_window_hwm.max(wl);
        let wend = ws.wrapping_add(wl as u64);
        if wend > self.newest_v.wrapping_add(RESET_SPAN)
            || wend.wrapping_add(RESET_SPAN) < self.newest_v
        {
            self.st.symbols_dropped_stale += 1;
            return out;
        }
        if b == Boundary::Pre && self.wm_valid && wend - 1 > self.wm {
            self.wm = wend - 1;
        }
        self.advance(wend - 1);
        let se = self.arr_stale_end();
        self.arr.advance(wend - 1, se);
        if ws < self.live_floor() {
            self.st.symbols_dropped_stale += 1;
            return out;
        }
        let mut solvable = self.note_repair(ws, wend, h.repair_key);
        if solvable {
            solvable = (0..wl as u64).any(|i| !self.known.contains_key(&(ws + i)));
        }
        if !solvable {
            self.st.repairs_skipped += 1;
            return out;
        }
        let mut r = Row {
            coeffs: BTreeMap::new(),
            payload: payload.to_vec(),
            first_seen_ms: now_ms,
        };
        let mut coeffs = [0u8; sw::MAX_WINDOW];
        sw::repair_coeffs(h.repair_key, &mut coeffs[..wl]);
        for (i, &c) in coeffs[..wl].iter().enumerate() {
            let s = ws + i as u64;
            match self.known.get(&s) {
                Some(k) => gf::lincomb(&mut r.payload, k, c),
                None => {
                    r.coeffs.insert(s, c);
                }
            }
        }
        let mut solved = Vec::new();
        self.insert_row(r, &mut solved);
        for (sv, sym) in solved {
            self.ingest(sv, sym, Origin::Repair, &mut out);
        }
        out
    }

    fn episode_window(&self) -> u64 {
        if self.repair_window_hwm > 0 {
            self.repair_window_hwm as u64
        } else {
            self.cfg.window as u64
        }
    }

    fn note_repair(&mut self, ws: u64, we: u64, key: u32) -> bool {
        if self
            .repairs_seen
            .iter()
            .rev()
            .any(|r| r.ws == ws && r.key == key)
        {
            return false;
        }
        self.repairs_seen.push_back(RepairSpan { ws, we, key });
        true
    }

    fn note_missing(&mut self, v: u64, via: Origin, stale: bool) {
        if self.ep_open && v > self.ep_last + self.episode_window() {
            self.settle_episodes(v);
        }
        if !self.ep_open {
            self.ep = LossEpisode {
                first_seq: v,
                ..Default::default()
            };
            self.ep_open = true;
        }
        self.ep_last = v;
        self.ep.missing += 1;
        match via {
            Origin::Repair => self.ep.recovered += 1,
            Origin::Retx => self.ep.retx += 1,
            Origin::Source => self.ep.abandoned += 1,
        }
        if stale {
            self.ep.stale += 1;
        }
    }

    fn settle_episodes(&mut self, evict_end: u64) {
        let w = self.episode_window();
        if self.ep_open && evict_end > self.ep_last + w + 1 {
            self.ep.span = (self.ep_last - self.ep.first_seq + 1) as u32;
            self.ep.window = w as u32;
            let (first, last) = (self.ep.first_seq, self.ep_last);
            self.ep.repairs = self
                .repairs_seen
                .iter()
                .filter(|r| r.ws <= last && r.we > first)
                .count() as u32;
            if self.episodes.len() >= MAX_QUEUED_EPISODES {
                self.episodes.remove(0);
            }
            self.episodes.push(self.ep);
            self.ep_open = false;
        }
        let keep_from = if self.ep_open {
            self.ep.first_seq
        } else {
            evict_end
        };
        while self.repairs_seen.front().is_some_and(|r| r.we <= keep_from) {
            self.repairs_seen.pop_front();
        }
    }

    pub fn take_episodes(&mut self) -> Vec<LossEpisode> {
        std::mem::take(&mut self.episodes)
    }

    /// Drops repair rows first seen more than deadline_ms ago.
    pub fn expire_rows_older_than(&mut self, deadline_ms: u64, now_ms: u64) -> usize {
        let before = self.rows.len();
        self.rows
            .retain(|_, r| !(now_ms > r.first_seen_ms && now_ms - r.first_seen_ms > deadline_ms));
        before - self.rows.len()
    }
}
