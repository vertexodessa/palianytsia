//! Records for the flight log that WebGs has no home for, so neither the stats line nor the
//! m1 parity lines carry them: the drone's congestion shed as episodes, the ground's arrival
//! order, and the NACK requests their own copy answered. Three parts that never read each
//! other; the core feeds them and the loop takes the queue every tick.
//!
//! Cost bound: nothing here allocates per body. `RxOrder` is a bit test and a store per
//! CRC-good video body. The log volume is bounded whatever the air does: at most
//! `LATE_ROWS_PER_S` `nack_late` rows a second (the rest only counted), one `rx_reorder` row a
//! second, and a `drone_shed` row per Telem edge; the queue holds at most `CAP` rows, the oldest
//! dropped first.
use std::collections::VecDeque;

use crate::json::J;
use crate::nack::LateFill;

/// Rows kept between two takes. The loop takes every tick (about 1 ms), so this only bounds a
/// loop that stopped taking.
const CAP: usize = 256;
/// `nack_late` rows a second; the rest go into the next `nack_late_capped` count. The bench jam
/// gave 8-108 late fills per 27 s arm (2026-11-13), so 50 a second keeps every one there.
const LATE_ROWS_PER_S: u32 = 50;

#[derive(Default)]
pub(super) struct DiagLane {
    q: VecDeque<String>,
    dropped: u64,
    pub shed: ShedWatch,
    pub order: RxOrder,
    order_t0_ms: u64,
    late_t0_ms: u64,
    late_rows: u32,
    late_capped: u64,
}

impl DiagLane {
    pub fn push(&mut self, ev: &str, mut o: J) {
        if self.q.len() >= CAP {
            self.q.pop_front();
            self.dropped += 1;
        }
        o.set("ev", ev);
        self.q.push_back(o.dump());
    }

    pub fn take(&mut self) -> Vec<String> {
        if self.dropped > 0 {
            let mut o = J::obj();
            o.set("rows", core::mem::take(&mut self.dropped));
            self.push("diag_dropped", o);
        }
        self.q.drain(..).collect()
    }

    /// One `nack_late` row per late fill, with the drone's queue as its last Telem told it.
    pub fn late_fills(&mut self, rows: &[LateFill], capped: u64, dq_wait_ms: Option<u16>) {
        self.late_capped += capped;
        for r in rows {
            if r.t_ms >= self.late_t0_ms + 1000 {
                self.late_t0_ms = r.t_ms;
                self.late_rows = 0;
                self.flush_late_capped();
            }
            if self.late_rows >= LATE_ROWS_PER_S {
                self.late_capped += 1;
                continue;
            }
            self.late_rows += 1;
            let mut o = J::obj();
            o.set("t_ms", r.t_ms);
            o.set("seq", u64::from(r.seq));
            o.set("tail", r.from_tail);
            o.set("tries", i64::from(r.tries));
            o.set("late_ms", u64::from(r.late_ms));
            o.set("after_req_ms", u64::from(r.after_req_ms));
            o.set("settle_ms", i64::from(r.settle_ms));
            o.set("dq_wait_ms", dq_wait_ms.map(u64::from));
            self.push("nack_late", o);
        }
    }

    fn flush_late_capped(&mut self) {
        if self.late_capped > 0 {
            let mut o = J::obj();
            o.set("capped", core::mem::take(&mut self.late_capped));
            self.push("nack_late_capped", o);
        }
    }

    /// Once a second: the arrival-order window as one `rx_reorder` row, only when something
    /// arrived out of order.
    pub fn order_window(&mut self, now_ms: u64) {
        if self.order_t0_ms == 0 {
            self.order_t0_ms = now_ms;
            return;
        }
        if now_ms < self.order_t0_ms + 1000 {
            return;
        }
        self.order_t0_ms = now_ms;
        if now_ms >= self.late_t0_ms + 1000 {
            self.flush_late_capped();
        }
        let w = self.order.take_window();
        if w.late == 0 {
            return;
        }
        let mut o = J::obj();
        o.set("late", w.late);
        o.set("cross_card", w.cross_card);
        o.set("lag_ms_max", w.lag_ms_max);
        o.set("lag_ms_mean", w.lag_ms_sum / w.late);
        o.set("back_max", u64::from(w.back_max));
        self.push("rx_reorder", o);
    }
}

/// The drone's congestion shed (Telem flags bit4: the enh layer is dropped while its TxQueue is
/// at or past half its cap) as episodes. The bit is per Telem; an episode opens on the first
/// Telem with it set and closes on the first without it.
#[derive(Default)]
pub(super) struct ShedWatch {
    since_ms: Option<u64>,
    /// Worst `txq_wait_max_ms` the drone reported during the episode.
    wait_max_ms: u16,
}

pub(super) enum ShedEdge {
    Began { txq_wait_ms: u16 },
    Ended { dur_ms: u64, txq_wait_max_ms: u16 },
}

impl ShedWatch {
    pub fn on_telem(&mut self, shed: bool, txq_wait_ms: u16, now_ms: u64) -> Option<ShedEdge> {
        match (self.since_ms, shed) {
            (None, true) => {
                self.since_ms = Some(now_ms);
                self.wait_max_ms = txq_wait_ms;
                Some(ShedEdge::Began { txq_wait_ms })
            }
            (Some(t0), false) => {
                self.since_ms = None;
                Some(ShedEdge::Ended {
                    dur_ms: now_ms.saturating_sub(t0),
                    txq_wait_max_ms: self.wait_max_ms.max(txq_wait_ms),
                })
            }
            (Some(_), true) => {
                self.wait_max_ms = self.wait_max_ms.max(txq_wait_ms);
                None
            }
            (None, false) => None,
        }
    }

    /// How long the current episode has lasted; None while the drone is not shedding.
    pub fn active_ms(&self, now_ms: u64) -> Option<u64> {
        self.since_ms.map(|t0| now_ms.saturating_sub(t0))
    }

    /// A Telem gap past staleness ends what the core can say about the drone.
    pub fn forget(&mut self) {
        self.since_ms = None;
    }
}

/// Arrival order of the drone's frames at the union of the cards, by the 802.11 sequence number.
/// The drone sends from one hardware queue with hardware sequence numbers and no MAC retries, so
/// on the air the order is strict; a frame that reaches the core behind the newest one and was
/// not seen before was reordered on the ground (another card's USB batch, or one card's RX
/// aggregation).
pub(super) struct RxOrder {
    newest: Option<u16>,
    newest_card: u8,
    /// One bit per sequence number: delivered once already (the other card's copy is a duplicate).
    seen: [u64; 64],
    /// When the newest moved past each sequence number, ms on the core clock; 0 = not tracked.
    passed_ms: Box<[u32; 4096]>,
    win: OrderWindow,
}

/// Steps forward larger than this are an outage or a restart: tracking starts over.
const ORDER_SPAN: u16 = 512;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(super) struct OrderWindow {
    pub late: u64,
    pub cross_card: u64,
    pub lag_ms_max: u64,
    pub lag_ms_sum: u64,
    pub back_max: u16,
}

impl Default for RxOrder {
    fn default() -> Self {
        Self {
            newest: None,
            newest_card: 0,
            seen: [0; 64],
            passed_ms: Box::new([0; 4096]),
            win: OrderWindow::default(),
        }
    }
}

impl RxOrder {
    fn bit(&self, s: u16) -> bool {
        self.seen[usize::from(s >> 6)] & (1 << (s & 63)) != 0
    }
    fn set(&mut self, s: u16, v: bool) {
        let (w, b) = (usize::from(s >> 6), 1u64 << (s & 63));
        if v {
            self.seen[w] |= b;
        } else {
            self.seen[w] &= !b;
        }
    }

    /// One CRC-good drone frame (`mac_seq` is 12 bits).
    pub fn on_frame(&mut self, mac_seq: u16, card: u8, now_ms: u64) {
        let s = mac_seq & 0x0FFF;
        let stamp = (now_ms as u32).max(1);
        let Some(newest) = self.newest else {
            self.restart(s, card);
            return;
        };
        let ahead = s.wrapping_sub(newest) & 0x0FFF;
        if ahead == 0 {
            return;
        }
        if ahead < 2048 {
            if ahead > ORDER_SPAN {
                self.restart(s, card);
                return;
            }
            for k in 1..ahead {
                let p = newest.wrapping_add(k) & 0x0FFF;
                self.set(p, false);
                self.passed_ms[usize::from(p)] = stamp;
            }
            self.set(s, true);
            self.newest = Some(s);
            self.newest_card = card;
            return;
        }
        let back = 4096 - ahead;
        let passed = self.passed_ms[usize::from(s)];
        if back > ORDER_SPAN || self.bit(s) || passed == 0 {
            return;
        }
        self.set(s, true);
        let w = &mut self.win;
        w.late += 1;
        if card != self.newest_card {
            w.cross_card += 1;
        }
        let lag = u64::from(stamp.wrapping_sub(passed));
        w.lag_ms_max = w.lag_ms_max.max(lag);
        w.lag_ms_sum += lag;
        w.back_max = w.back_max.max(back);
    }

    fn restart(&mut self, s: u16, card: u8) {
        self.seen = [0; 64];
        self.passed_ms.fill(0);
        self.set(s, true);
        self.newest = Some(s);
        self.newest_card = card;
    }

    pub fn take_window(&mut self) -> OrderWindow {
        core::mem::take(&mut self.win)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two cards, card 1 a USB batch behind: its copies of 10..12 are duplicates, its first
    /// copy of 13 (card 0 lost it) arrives after 14 and counts once, cross-card, 3 ms late.
    #[test]
    fn a_frame_behind_the_newest_and_never_seen_counts_once() {
        let mut o = RxOrder::default();
        for (s, c, t) in [(10, 0, 100), (11, 0, 101), (12, 0, 102), (14, 0, 104)] {
            o.on_frame(s, c, t);
        }
        for (s, t) in [(10, 105), (11, 105), (12, 105), (13, 107), (14, 107)] {
            o.on_frame(s, 1, t);
        }
        o.on_frame(13, 0, 108);
        let w = o.take_window();
        assert_eq!(
            w,
            OrderWindow {
                late: 1,
                cross_card: 1,
                lag_ms_max: 3,
                lag_ms_sum: 3,
                back_max: 1
            }
        );
        assert_eq!(o.take_window(), OrderWindow::default());
    }

    /// The 12-bit wrap is a step forward; a step past the span (an outage) starts over and
    /// what arrives behind it is not counted.
    #[test]
    fn wrap_is_forward_and_an_outage_starts_over() {
        let mut o = RxOrder::default();
        o.on_frame(4094, 0, 10);
        o.on_frame(1, 0, 11);
        o.on_frame(4095, 0, 12);
        assert_eq!(o.take_window().late, 1, "4095 was passed at the wrap");
        o.on_frame(1500, 0, 20);
        o.on_frame(1400, 0, 21);
        assert_eq!(o.take_window().late, 0);
    }

    #[test]
    fn shed_episode_opens_and_closes_with_its_worst_wait() {
        let mut w = ShedWatch::default();
        assert!(w.on_telem(false, 3, 0).is_none());
        assert!(matches!(
            w.on_telem(true, 40, 1000),
            Some(ShedEdge::Began { txq_wait_ms: 40 })
        ));
        assert!(w.on_telem(true, 90, 1100).is_none());
        assert_eq!(w.active_ms(1500), Some(500));
        assert!(matches!(
            w.on_telem(false, 5, 3000),
            Some(ShedEdge::Ended {
                dur_ms: 2000,
                txq_wait_max_ms: 90
            })
        ));
        assert_eq!(w.active_ms(3000), None);
    }

    /// 120 late fills inside one second: 50 rows, and the other 70 as one count once the
    /// second is over.
    #[test]
    fn late_rows_have_a_budget_per_second() {
        let mut d = DiagLane::default();
        let r = |t_ms| LateFill {
            t_ms,
            seq: 1,
            from_tail: true,
            tries: 1,
            late_ms: 20,
            after_req_ms: 8,
            settle_ms: 12,
        };
        let rows: Vec<LateFill> = (0..120).map(|i| r(5000 + i)).collect();
        d.late_fills(&rows, 0, Some(30));
        d.order_window(5000);
        d.order_window(6200);
        let out = d.take();
        assert_eq!(
            out.iter().filter(|l| l.contains("\"nack_late\"")).count(),
            50
        );
        assert_eq!(
            out.last().unwrap(),
            r#"{"capped":70,"ev":"nack_late_capped"}"#
        );
        assert!(out[0].contains(r#""dq_wait_ms":30"#), "{}", out[0]);
    }

    #[test]
    fn the_queue_is_bounded_and_says_what_it_dropped() {
        let mut d = DiagLane::default();
        for _ in 0..CAP + 5 {
            d.push("x", J::obj());
        }
        let rows = d.take();
        assert_eq!(rows.len(), CAP);
        assert!(rows.last().unwrap().contains("\"diag_dropped\""));
        assert!(rows.last().unwrap().contains("\"rows\":5"));
    }
}
