//! The NACK block of WebGs (web_gs.cpp poll_nack_/drain_nack_window_),
//! as three parts that never read each other: the tracker with its session
//! bookkeeping, the drone's own counters off T_TELEM, and the once-a-second
//! window the stats line shows.
use crate::config::Config;
use crate::json::J;
use crate::nack::{NackTracker, NackWindow};
use pal_wire::rc;

pub(super) struct NackLane {
    pub t: NackTracker,
    pub lookback: u32,
    pub down_util: f64,
    /// The vtx_nonce the tracker's counter belongs to.
    pub vtx_seen: u32,
    pub sent: u64,
    pub drone: DroneNack,
    pub win: WindowView,
}

impl NackLane {
    /// None with `link.nack.enable = false`.
    pub fn from_config(cfg: &Config) -> Option<Self> {
        cfg.link.nack.enable.then(|| Self {
            t: NackTracker::new(cfg.link.nack.clone()),
            lookback: cfg.link.nack.lookback as u32,
            down_util: cfg.link.ladder_cfg.down_util,
            vtx_seen: 0,
            sent: 0,
            drone: DroneNack::default(),
            win: WindowView::default(),
        })
    }

    /// The stats line's `nack` object.
    pub fn json(&self) -> J {
        let c = self.t.stats();
        let w = &self.win;
        let mut o = J::obj();
        o.set("req", c.requests);
        o.set("rep", c.repeats);
        o.set("syms", c.syms_requested);
        o.set("tail", c.tail_requests);
        o.set("fill", c.filled);
        o.set("late", c.late_fill);
        o.set("waste", c.wasted);
        o.set("drop", c.dropped_deadline);
        o.set("sup", c.suppressed);
        o.set("lead", c.lead_skipped);
        o.set("sent", self.sent);
        o.set("settle_ms", self.t.settle_ms());
        o.set("fill_pps", w.fill_pps);
        o.set("fill_p50", w.fill_p50);
        o.set("fill_p90", w.fill_p90);
        o.set("fill_max", w.fill_max);
        o.set("late_max", w.late_max);
        o.set("drone_rx", self.drone.nack_rx);
        o.set("drone_syms", self.drone.retx_syms);
        o.set("drone_refused", self.drone.retx_refused);
        o
    }
}

/// The drone's NACK counters: each T_TELEM carries its period's counts.
#[derive(Default)]
pub(super) struct DroneNack {
    tlm_seen: Option<u16>,
    pub nack_rx: u64,
    pub retx_syms: u64,
    pub retx_refused: u64,
}

impl DroneNack {
    /// Sums a Telem period once, however often it is heard.
    pub fn on_telem(&mut self, t: &rc::Telem) {
        if self.tlm_seen == Some(t.tlm_seq) {
            return;
        }
        self.tlm_seen = Some(t.tlm_seq);
        self.nack_rx += u64::from(t.nack_rx);
        self.retx_syms += u64::from(t.retx_syms);
        self.retx_refused += u64::from(t.retx_refused);
    }
}

/// The tracker's last full second, as the stats line reads it.
#[derive(Default)]
pub(super) struct WindowView {
    t0_ms: u64,
    pub fill_pps: Option<f64>,
    pub late_max: u32,
    pub fill_p50: Option<u32>,
    pub fill_p90: Option<u32>,
    pub fill_max: Option<u32>,
}

impl WindowView {
    /// Once a second: takes the tracker's window. The first call only
    /// starts the clock.
    pub fn drain(&mut self, now_ms: u64, t: &mut NackTracker) {
        if self.t0_ms == 0 {
            self.t0_ms = now_ms;
            return;
        }
        if now_ms < self.t0_ms + 1000 {
            return;
        }
        let secs = (now_ms - self.t0_ms) as f64 / 1000.0;
        *self = Self::summarize(t.take_window(), secs);
        self.t0_ms = now_ms;
    }

    fn summarize(mut w: NackWindow, secs: f64) -> Self {
        w.fill_ms.sort_unstable();
        let pct = |p: f64| -> Option<u32> {
            if w.fill_ms.is_empty() {
                return None;
            }
            let rank = (p * w.fill_ms.len() as f64).ceil();
            let i = if rank < 1.0 { 0 } else { rank as usize - 1 };
            Some(w.fill_ms[i.min(w.fill_ms.len() - 1)])
        };
        Self {
            t0_ms: 0,
            fill_pps: Some(w.filled as f64 / secs),
            late_max: w.late_ms_max,
            fill_p50: pct(0.5),
            fill_p90: pct(0.9),
            fill_max: w.fill_ms.last().copied(),
        }
    }
}
