//! mabur `hop_ranker.{h,cpp}`: ranks in-flight hop candidates by the energy
//! the scout's brief visits found on them. NHM-blocked channels form their
//! own tier below every unblocked ranked one. Pure; the caller passes the
//! clock. Ties go to the boot pick, then config order.

use std::collections::VecDeque;

use crate::config::{BusyCfg, HopCfg};

/// Which card gathered a visit: a USB dwell (5 ms, CCA counts) or a relay
/// sweep (20 ms survey, no CCA). Not one scale, so a ranking uses only the
/// newest visit's kind.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum VisitSrc {
    #[default]
    Usb,
    Relay,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HopVisit {
    pub ch: u8,
    pub t_ms: f64,
    pub fa: u32,
    pub cca: u32,
    pub own: u32,
    pub foreign: u32,
    pub busy_valid: bool,
    pub busy_pct: f64,
    pub src: VisitSrc,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HopRankEntry {
    pub ch: u8,
    pub score: u32,
    pub visits: i32,
    pub ranked: bool,
    /// The mean busy_pct of the fresh visits that had a reading reaches
    /// `blocked_pct`.
    pub blocked: bool,
    pub busy_pct: f64,
}

#[derive(Clone, Debug)]
pub struct HopRanker {
    cfg: HopCfg,
    busy: BusyCfg,
    candidates: Vec<u8>,
    boot_pick: u8,
    visits: Vec<VecDeque<HopVisit>>,
}

impl HopRanker {
    pub fn new(cfg: HopCfg, busy: BusyCfg, channels: Vec<u8>, boot_pick: u8) -> Self {
        let n = channels.len();
        Self {
            cfg,
            busy,
            candidates: channels,
            boot_pick,
            visits: vec![VecDeque::new(); n],
        }
    }

    pub fn add(&mut self, v: HopVisit) {
        if let Some(i) = self.candidates.iter().position(|&c| c == v.ch) {
            let dq = &mut self.visits[i];
            dq.push_back(v);
            while dq.len() as i64 > self.cfg.rank_visits as i64 {
                dq.pop_front();
            }
        }
    }

    /// The boot scan's pick arrives with the first DiscAck; until then (0)
    /// ties fall through to config order.
    pub fn set_boot_pick(&mut self, ch: u8) {
        self.boot_pick = ch;
    }

    /// fa + max(cca - own, 0) + 4 * foreign, wrapping like mabur's uint32.
    pub fn score(v: &HopVisit) -> u32 {
        let busy = (v.cca as i64 - v.own as i64).max(0) as u32;
        v.fa.wrapping_add(busy)
            .wrapping_add(v.foreign.wrapping_mul(4))
    }

    /// Best first, unranked last.
    pub fn ranking(&self, now_ms: f64) -> Vec<HopRankEntry> {
        let max_age = self.cfg.rank_max_age_ms as f64;
        let mut newest_t = -1.0;
        let mut src = VisitSrc::Usb;
        for v in self.visits.iter().flatten() {
            if now_ms - v.t_ms <= max_age && v.t_ms >= newest_t {
                newest_t = v.t_ms;
                src = v.src;
            }
        }
        let mut entries: Vec<HopRankEntry> = self
            .candidates
            .iter()
            .zip(&self.visits)
            .map(|(&ch, dq)| {
                let (mut fresh, mut busy_n, mut sum, mut busy_sum) = (0, 0, 0u32, 0.0);
                for v in dq
                    .iter()
                    .filter(|v| now_ms - v.t_ms <= max_age && v.src == src)
                {
                    fresh += 1;
                    sum = sum.wrapping_add(Self::score(v));
                    if v.busy_valid {
                        busy_n += 1;
                        busy_sum += v.busy_pct;
                    }
                }
                let busy_pct = if busy_n > 0 {
                    busy_sum / busy_n as f64
                } else {
                    0.0
                };
                HopRankEntry {
                    ch,
                    score: sum,
                    visits: fresh,
                    ranked: fresh >= 2,
                    blocked: busy_n > 0 && busy_pct >= self.busy.blocked_pct,
                    busy_pct,
                }
            })
            .collect();
        let boot = self.boot_pick;
        entries.sort_by(|a, b| {
            use std::cmp::Ordering::*;
            if a.ranked != b.ranked {
                return if a.ranked { Less } else { Greater };
            }
            if !a.ranked {
                return Equal;
            }
            if a.blocked != b.blocked {
                return if a.blocked { Greater } else { Less };
            }
            if a.blocked && a.busy_pct != b.busy_pct {
                return a.busy_pct.partial_cmp(&b.busy_pct).unwrap_or(Equal);
            }
            if a.score != b.score {
                return a.score.cmp(&b.score);
            }
            (b.ch == boot).cmp(&(a.ch == boot))
        });
        entries
    }

    /// The first ranked candidate that is not `exclude` nor in `skip` (and
    /// not blocked with `require_unblocked`: the hop never orders a channel
    /// the dwells read as blocked).
    pub fn best(
        &self,
        now_ms: f64,
        exclude: u8,
        skip: &[u8],
        require_unblocked: bool,
    ) -> Option<u8> {
        self.ranking(now_ms)
            .into_iter()
            .find(|e| {
                e.ranked
                    && e.ch != exclude
                    && !(require_unblocked && e.blocked)
                    && !skip.contains(&e.ch)
            })
            .map(|e| e.ch)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> HopCfg {
        HopCfg {
            rank_visits: 5,
            rank_max_age_ms: 10000,
            ..Default::default()
        }
    }
    fn v(ch: u8, t: f64, fa: u32, cca: u32, own: u32, foreign: u32) -> HopVisit {
        HopVisit {
            ch,
            t_ms: t,
            fa,
            cca,
            own,
            foreign,
            ..Default::default()
        }
    }
    fn f(ch: u8, t: f64, fa: u32) -> HopVisit {
        v(ch, t, fa, 0, 0, 0)
    }
    fn bv(ch: u8, t: f64, fa: u32, busy: f64) -> HopVisit {
        HopVisit {
            busy_valid: true,
            busy_pct: busy,
            ..f(ch, t, fa)
        }
    }
    fn r3(boot: u8) -> HopRanker {
        let mut r = HopRanker::new(cfg(), BusyCfg::default(), vec![120, 149, 165], boot);
        for i in 0..3 {
            for ch in [120, 149, 165] {
                r.add(f(ch, i as f64 * 100.0, 2));
            }
        }
        r
    }

    #[test]
    fn score_formula() {
        assert_eq!(HopRanker::score(&v(1, 0.0, 3, 10, 12, 0)), 3);
        assert_eq!(HopRanker::score(&v(1, 0.0, 3, 10, 2, 2)), 3 + 8 + 8);
    }

    #[test]
    fn unranked_below_two_fresh_visits() {
        let mut r = HopRanker::new(cfg(), BusyCfg::default(), vec![120, 149, 165], 136);
        r.add(f(120, 0.0, 0));
        assert!(!r.ranking(100.0)[0].ranked);
        assert_eq!(r.best(100.0, 136, &[], false), None);
    }

    #[test]
    fn sum_of_last_five_visits_lowest_wins() {
        let mut r = HopRanker::new(cfg(), BusyCfg::default(), vec![120, 149, 165], 136);
        for i in 0..8 {
            let t = i as f64 * 300.0;
            r.add(f(120, t, if i < 3 { 30 } else { 0 }));
            r.add(f(149, t, 4));
            r.add(f(165, t, 13));
        }
        let k = r.ranking(2500.0);
        assert!(k[0].ch == 120 && k[0].score == 0);
        assert!(k[1].ch == 149 && k[1].score == 20);
        assert!(k[2].ch == 165 && k[2].score == 65);
        assert_eq!(r.best(2500.0, 136, &[], false), Some(120));
    }

    #[test]
    fn stale_visits_are_dropped() {
        let mut r = HopRanker::new(cfg(), BusyCfg::default(), vec![120], 136);
        r.add(f(120, 0.0, 0));
        r.add(f(120, 100.0, 0));
        assert!(r.best(5000.0, 136, &[], false).is_some());
        assert!(r.best(20000.0, 136, &[], false).is_none());
    }

    #[test]
    fn tie_break_falls_through_to_config_order() {
        assert_eq!(r3(200).best(1000.0, 255, &[], false), Some(120));
    }

    #[test]
    fn exclude_and_skip_lists_and_tiebreak() {
        let r = r3(149);
        assert_eq!(r.best(1000.0, 136, &[], false), Some(149));
        assert_eq!(r.best(1000.0, 149, &[], false), Some(120));
        assert_eq!(r.best(1000.0, 149, &[120], false), Some(165));
        assert_eq!(r.best(1000.0, 149, &[120, 165], false), None);
    }

    #[test]
    fn boot_pick_published_after_construction_wins_the_tiebreak() {
        let mut r = r3(0);
        assert_eq!(r.best(1000.0, 136, &[], false), Some(120));
        r.set_boot_pick(165);
        assert_eq!(r.best(1000.0, 136, &[], false), Some(165));
    }

    #[test]
    fn no_boot_pick_falls_through_to_config_order() {
        assert_eq!(r3(0).best(1000.0, 255, &[], false), Some(120));
    }

    fn pair(b: BusyCfg, a: (f64, u32, f64), c: (f64, u32, f64)) -> HopRanker {
        let mut r = HopRanker::new(HopCfg::default(), b, vec![144, 64], 0);
        for i in 0..3 {
            let t = i as f64;
            r.add(bv(144, t + a.0, a.1, a.2));
            r.add(bv(64, t + c.0, c.1, c.2));
        }
        r
    }

    #[test]
    fn blocked_channel_loses_to_a_busier_by_events_unblocked_one() {
        let r = pair(BusyCfg::default(), (0.0, 0, 90.0), (0.0, 40, 0.0));
        assert_eq!(r.best(10.0, 136, &[], false), Some(64));
    }

    #[test]
    fn blocked_threshold_comes_from_busy_cfg() {
        let low = BusyCfg {
            blocked_pct: 30.0,
            ..Default::default()
        };
        assert_eq!(
            pair(low, (0.0, 0, 40.0), (0.0, 40, 0.0)).best(10.0, 136, &[], false),
            Some(64)
        );
        let d = pair(BusyCfg::default(), (0.0, 0, 40.0), (0.0, 40, 0.0));
        assert_eq!(d.best(10.0, 136, &[], false), Some(144));
    }

    #[test]
    fn busy_is_the_mean_over_fresh_visits() {
        let mut r = HopRanker::new(HopCfg::default(), BusyCfg::default(), vec![144], 0);
        for (t, b) in [(1.0, 100.0), (2.0, 0.0), (3.0, 0.0), (4.0, 0.0)] {
            r.add(bv(144, t, 0, b));
        }
        let e = r.ranking(10.0)[0];
        assert!(!e.blocked);
        assert_eq!(e.busy_pct, 25.0);
    }

    #[test]
    fn all_blocked_least_busy_wins() {
        let r = pair(BusyCfg::default(), (0.0, 0, 95.0), (0.0, 0, 60.0));
        assert_eq!(r.best(10.0, 136, &[], false), Some(64));
    }

    #[test]
    fn visits_without_busy_rank_as_today() {
        let mut r = HopRanker::new(HopCfg::default(), BusyCfg::default(), vec![144, 64], 0);
        for i in 0..3 {
            r.add(f(144, i as f64, 5));
            r.add(f(64, i as f64, 1));
        }
        assert_eq!(r.best(10.0, 136, &[], false), Some(64));
    }

    #[test]
    fn best_require_unblocked_skips_blocked() {
        let mut r = HopRanker::new(HopCfg::default(), BusyCfg::default(), vec![144, 112], 0);
        for i in 0..3 {
            r.add(bv(144, i as f64, 0, 95.0));
            r.add(bv(112, i as f64, 40, 0.0));
        }
        assert_eq!(r.best(10.0, 136, &[], true), Some(112));
        assert_eq!(r.best(10.0, 136, &[112], true), None);
        assert_eq!(r.best(10.0, 136, &[112], false), Some(144));
        let e = r.ranking(10.0).into_iter().find(|e| e.ch == 144).unwrap();
        assert!(e.blocked);
    }

    #[test]
    fn ranking_uses_only_the_newest_visit_source() {
        let mut r = HopRanker::new(HopCfg::default(), BusyCfg::default(), vec![40, 64, 112], 0);
        let usb = |ch, t, cca| v(ch, t, 0, cca, 0, 0);
        let rel = |ch, t, fa| HopVisit {
            src: VisitSrc::Relay,
            ..f(ch, t, fa)
        };
        r.add(usb(64, 100.0, 500));
        r.add(usb(64, 110.0, 500));
        r.add(usb(112, 100.0, 0));
        r.add(usb(112, 110.0, 0));
        r.add(rel(64, 200.0, 0));
        r.add(rel(64, 201.0, 0));
        r.add(rel(112, 200.0, 90));
        r.add(rel(112, 201.0, 90));
        assert_eq!(r.best(300.0, 40, &[], false), Some(64));
        let e = r.ranking(300.0).into_iter().find(|e| e.ch == 112).unwrap();
        assert!(e.visits == 2 && e.score == 180);
    }
}
