//! mabur `channel_ranker.{h,cpp}`: boot-time channel ranking from scout
//! visits. The worst visit decides; ties go to a valid lower noise floor,
//! then config order. A channel whose worst NHM busy % reaches
//! `blocked_pct` ranks after every unblocked one. `proposal(current)` keeps
//! the current channel unless the best is `margin` busy units cleaner.

/// One scout visit's worth of the scouting card's own counters.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RankSample {
    pub ch: u8,
    pub cca: u32,
    pub fa: u32,
    pub own: u32,
    pub foreign: u32,
    /// Own-TX leak estimate (frames x k), subtracted with `own`.
    pub leak: u32,
    pub floor_valid: bool,
    pub floor_dbm: i8,
    pub busy_valid: bool,
    pub busy_pct: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RankEntry {
    pub ch: u8,
    pub worst_busy: u32,
    pub visits: u32,
    pub floor_valid: bool,
    pub floor_dbm: i8,
    pub busy_valid: bool,
    pub worst_busy_pct: f64,
}

#[derive(Clone, Debug)]
pub struct ChannelRanker {
    min_rounds: i32,
    margin: u32,
    blocked_pct: f64,
    entries: Vec<RankEntry>, // config order, duplicates dropped
}

impl ChannelRanker {
    pub fn new(channels: &[u8], min_rounds: i32, margin: u32, blocked_pct: f64) -> Self {
        let mut entries: Vec<RankEntry> = Vec::new();
        for &c in channels {
            if !entries.iter().any(|e| e.ch == c) {
                entries.push(RankEntry {
                    ch: c,
                    ..Default::default()
                });
            }
        }
        Self {
            min_rounds,
            margin,
            blocked_pct,
            entries,
        }
    }

    /// mabur's defaults: no margin, nothing blocked.
    pub fn plain(channels: &[u8], min_rounds: i32) -> Self {
        Self::new(channels, min_rounds, 0, 1e9)
    }

    pub fn busy(s: &RankSample) -> u32 {
        let foreign_cca = s.cca as i64 - s.own as i64 - s.leak as i64;
        foreign_cca.max(0) as u32 + s.fa + s.foreign
    }

    pub fn add(&mut self, s: &RankSample) {
        let Some(e) = self.entries.iter_mut().find(|e| e.ch == s.ch) else {
            return;
        };
        let b = Self::busy(s);
        if e.visits == 0 || b > e.worst_busy {
            e.worst_busy = b;
        }
        e.visits += 1;
        if s.floor_valid && (!e.floor_valid || s.floor_dbm > e.floor_dbm) {
            e.floor_valid = true;
            e.floor_dbm = s.floor_dbm;
        }
        if s.busy_valid && (!e.busy_valid || s.busy_pct > e.worst_busy_pct) {
            e.busy_valid = true;
            e.worst_busy_pct = s.busy_pct;
        }
    }

    pub fn is_blocked(&self, e: &RankEntry) -> bool {
        e.busy_valid && e.worst_busy_pct >= self.blocked_pct
    }

    pub fn ranked(&self) -> Vec<RankEntry> {
        self.ranked_at(self.min_rounds)
    }

    /// Ranked with a caller-chosen visit bar.
    pub fn ranked_at(&self, min_rounds: i32) -> Vec<RankEntry> {
        let mut out: Vec<RankEntry> = self
            .entries
            .iter()
            .filter(|e| e.visits as i64 >= min_rounds as i64)
            .copied()
            .collect();
        // Stable: config order is the final tie-break.
        out.sort_by(|a, b| {
            use std::cmp::Ordering::*;
            let (ab, bb) = (self.is_blocked(a), self.is_blocked(b));
            if ab != bb {
                return if ab { Greater } else { Less };
            }
            if ab && a.worst_busy_pct != b.worst_busy_pct {
                return a
                    .worst_busy_pct
                    .partial_cmp(&b.worst_busy_pct)
                    .unwrap_or(Equal);
            }
            if a.worst_busy != b.worst_busy {
                return a.worst_busy.cmp(&b.worst_busy);
            }
            if a.floor_valid != b.floor_valid {
                return if a.floor_valid { Less } else { Greater };
            }
            if a.floor_valid && a.floor_dbm != b.floor_dbm {
                return a.floor_dbm.cmp(&b.floor_dbm);
            }
            Equal
        });
        out
    }

    pub fn all(&self) -> &[RankEntry] {
        &self.entries
    }

    pub fn proposal(&self, current: u8) -> u8 {
        self.proposal_at(current, self.min_rounds)
    }

    pub fn proposal_at(&self, current: u8, min_rounds: i32) -> u8 {
        self.proposal_among(current, min_rounds, &[])
    }

    /// `proposal_at` over the members in `usable` (empty = all); `current`
    /// may be outside it, and then any usable ranked member beats it.
    pub fn proposal_among(&self, current: u8, min_rounds: i32, usable: &[u8]) -> u8 {
        let mut k = self.ranked_at(min_rounds);
        if !usable.is_empty() {
            k.retain(|e| usable.contains(&e.ch));
        }
        let Some(best) = k.first() else {
            return current;
        };
        if best.ch == current || self.margin == 0 {
            return best.ch;
        }
        if let Some(e) = k.iter().find(|e| e.ch == current) {
            if self.is_blocked(e) && !self.is_blocked(best) {
                return best.ch; // the margin applies within a tier
            }
            return if best.worst_busy as u64 + self.margin as u64 <= e.worst_busy as u64 {
                best.ch
            } else {
                current
            };
        }
        best.ch // current not ranked yet
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(ch: u8, cca: u32, fa: u32, own: u32, foreign: u32) -> RankSample {
        RankSample {
            ch,
            cca,
            fa,
            own,
            foreign,
            ..Default::default()
        }
    }
    fn sf(ch: u8, cca: u32, f: i8) -> RankSample {
        RankSample {
            floor_valid: true,
            floor_dbm: f,
            ..s(ch, cca, 0, 0, 0)
        }
    }
    fn rsb(ch: u8, fa: u32, busy: f64) -> RankSample {
        RankSample {
            ch,
            fa,
            busy_valid: true,
            busy_pct: busy,
            ..Default::default()
        }
    }
    fn chs(v: &[RankEntry]) -> Vec<u8> {
        v.iter().map(|e| e.ch).collect()
    }

    #[test]
    fn proposal_among_names_only_usable_members() {
        let mut r = ChannelRanker::plain(&[36, 40, 44], 1);
        r.add(&s(36, 0, 0, 0, 0));
        r.add(&s(40, 50, 0, 0, 0));
        r.add(&s(44, 90, 0, 0, 0));
        assert_eq!(r.proposal_at(44, 1), 36);
        assert_eq!(r.proposal_among(44, 1, &[]), 36);
        assert_eq!(r.proposal_among(44, 1, &[40, 44]), 40);
        // Current outside the usable set: the best usable, however busy.
        assert_eq!(r.proposal_among(36, 1, &[44]), 44);
        // Nothing usable ranked yet: stay.
        assert_eq!(r.proposal_among(36, 2, &[44]), 36);
    }

    #[test]
    fn busy_subtracts_own_frames_and_never_underflows() {
        assert_eq!(ChannelRanker::busy(&s(1, 100, 5, 40, 2)), 67);
        assert_eq!(ChannelRanker::busy(&s(1, 10, 0, 40, 0)), 0);
    }

    #[test]
    fn unranked_until_min_rounds() {
        let mut r = ChannelRanker::plain(&[136, 149, 153], 3);
        r.add(&s(149, 0, 0, 0, 0));
        r.add(&s(149, 0, 0, 0, 0));
        for _ in 0..3 {
            r.add(&s(136, 500, 0, 0, 0));
        }
        assert_eq!(r.ranked().len(), 1);
        assert_eq!(r.proposal(136), 136);
        r.add(&s(149, 0, 0, 0, 0));
        assert_eq!(r.ranked().len(), 2);
        assert_eq!(r.proposal(136), 149);
    }

    #[test]
    fn worst_visit_beats_mean() {
        let mut r = ChannelRanker::plain(&[136, 149], 2);
        r.add(&s(136, 5, 0, 0, 0));
        r.add(&s(136, 5, 0, 0, 0));
        r.add(&s(149, 0, 0, 0, 0));
        r.add(&s(149, 900, 0, 0, 0));
        let k = r.ranked();
        assert_eq!(chs(&k), vec![136, 149]);
        assert_eq!((k[0].worst_busy, k[1].worst_busy), (5, 900));
    }

    #[test]
    fn floor_tiebreak_only_when_both_valid_and_home_first_otherwise() {
        let mut r = ChannelRanker::plain(&[136, 149, 153], 1);
        r.add(&sf(136, 3, -90));
        r.add(&sf(149, 3, -96));
        r.add(&s(153, 3, 0, 0, 0));
        assert_eq!(chs(&r.ranked()), vec![149, 136, 153]);
    }

    #[test]
    fn config_order_final_tiebreak_and_home_dedup() {
        let mut r = ChannelRanker::plain(&[136, 153, 136, 149], 1);
        r.add(&s(153, 1, 0, 0, 0));
        r.add(&s(149, 1, 0, 0, 0));
        r.add(&s(136, 7, 0, 0, 0));
        assert_eq!(chs(r.all()), vec![136, 153, 149]);
        assert_eq!(chs(&r.ranked()), vec![153, 149, 136]);
    }

    #[test]
    fn unknown_channel_ignored() {
        let mut r = ChannelRanker::plain(&[136, 149], 1);
        r.add(&s(44, 0, 0, 0, 0));
        assert!(r.ranked().is_empty());
        assert_eq!(r.proposal(136), 136);
    }

    #[test]
    fn floor_tiebreak_is_transitive_with_a_floorless_entry_between() {
        let mut r = ChannelRanker::plain(&[1, 2, 3, 4], 1);
        r.add(&s(1, 0, 0, 0, 0));
        r.add(&sf(2, 0, -70));
        r.add(&s(3, 0, 0, 0, 0));
        r.add(&sf(4, 0, -95));
        assert_eq!(chs(&r.ranked()), vec![4, 2, 1, 3]);
    }

    #[test]
    fn home_margin_keeps_home_unless_a_candidate_is_clearly_cleaner() {
        let mk = |rounds, margin, v: &[(u8, u32)]| {
            let set: Vec<u8> = v.iter().map(|x| x.0).collect();
            let mut r = ChannelRanker::new(&set, rounds, margin, 1e9);
            for &(c, b) in v {
                r.add(&s(c, b, 0, 0, 0));
            }
            r
        };
        assert_eq!(
            mk(1, 20, &[(136, 6), (149, 2), (165, 0)]).proposal(136),
            136
        );
        assert_eq!(
            mk(1, 20, &[(136, 30), (149, 8), (165, 12)]).proposal(136),
            149
        );
        assert_eq!(mk(1, 20, &[(136, 25), (149, 5)]).proposal(136), 149);
        assert_eq!(mk(1, 20, &[(136, 24), (149, 5)]).proposal(136), 136);
        let mut r5 = ChannelRanker::new(&[136, 149], 2, 20, 1e9);
        r5.add(&s(149, 0, 0, 0, 0));
        r5.add(&s(149, 0, 0, 0, 0));
        r5.add(&s(136, 300, 0, 0, 0));
        assert_eq!(r5.proposal(136), 149);
        assert_eq!(mk(1, 0, &[(136, 6), (149, 2)]).proposal(136), 149);
    }

    #[test]
    fn worst_busy_visit_blocks_the_channel() {
        let mut r = ChannelRanker::new(&[136, 144], 1, 0, 30.0);
        r.add(&rsb(144, 0, 5.0));
        r.add(&rsb(144, 0, 80.0));
        let e = *r.all().iter().find(|e| e.ch == 144).unwrap();
        assert!(r.is_blocked(&e));
    }

    #[test]
    fn blocked_home_loses_despite_margin() {
        let mut r = ChannelRanker::new(&[136, 144], 1, 20, 30.0);
        r.add(&rsb(136, 0, 100.0));
        r.add(&rsb(144, 15, 0.0));
        assert_eq!(r.proposal(136), 144);
    }

    #[test]
    fn unblocked_candidate_still_needs_the_margin() {
        let mut r = ChannelRanker::new(&[136, 144], 1, 20, 30.0);
        r.add(&rsb(136, 10, 0.0));
        r.add(&rsb(144, 0, 0.0));
        assert_eq!(r.proposal(136), 136);
    }

    #[test]
    fn leak_is_subtracted_before_the_clamp() {
        let mut x = s(1, 100, 5, 40, 2);
        x.leak = 50;
        assert_eq!(ChannelRanker::busy(&x), 17);
        x.leak = 500;
        assert_eq!(ChannelRanker::busy(&x), 7);
    }

    #[test]
    fn margin_is_against_the_current_channel_not_a_fixed_home() {
        let mut r = ChannelRanker::new(&[136, 149], 1, 20, 1e9);
        r.add(&s(136, 30, 0, 0, 0));
        r.add(&s(149, 15, 0, 0, 0));
        assert_eq!(r.proposal(136), 136);
        assert_eq!(r.proposal(149), 149);
        r.add(&s(136, 60, 0, 0, 0));
        assert_eq!(r.proposal(136), 149);
    }

    #[test]
    fn ranked_with_an_explicit_visit_bar_overrides_min_rounds() {
        let mut r = ChannelRanker::plain(&[136, 149], 3);
        r.add(&s(136, 100, 0, 0, 0));
        r.add(&s(149, 0, 0, 0, 0));
        assert!(r.ranked().is_empty());
        assert_eq!(chs(&r.ranked_at(1)), vec![149, 136]);
        assert!(r.ranked_at(2).is_empty());
    }

    #[test]
    fn proposal_with_an_explicit_visit_bar_overrides_min_rounds() {
        let mut r = ChannelRanker::new(&[136, 149], 3, 20, 1e9);
        r.add(&s(136, 100, 0, 0, 0));
        r.add(&s(149, 0, 0, 0, 0));
        assert_eq!(r.proposal(136), 136);
        assert_eq!(r.proposal_at(136, 1), 149);
        assert_eq!(r.proposal_at(136, 2), 136);
    }
}
