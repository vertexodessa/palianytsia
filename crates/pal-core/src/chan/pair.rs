//! mabur `pair_pick.{h,cpp}`: the 40 MHz boot scan ranks each 20 MHz half
//! like a channel; a pair scores as its worse half, is ranked only once both
//! halves have `min_rounds` visits, and is blocked when either half is.

use super::ranker::RankEntry;
use crate::config::ht40_pair_other;

/// Both halves of every pair in `set`, deduplicated, config order (low half
/// first). A channel with no pair is skipped.
pub fn scan_half_set(set: &[u8]) -> Vec<u8> {
    let mut out = Vec::new();
    for &c in set {
        let other = ht40_pair_other(c);
        if other == 0 {
            continue;
        }
        for ch in [c.min(other), c.max(other)] {
            if ch != 0 && !out.contains(&ch) {
                out.push(ch);
            }
        }
    }
    out
}

#[derive(Clone, Copy, Debug)]
struct PairScore {
    primary: u8,
    score: u32,
    blocked: bool,
    busy: f64,
}

/// Tier first (unblocked wins), blocked pairs by busy, then score.
fn better(a: &PairScore, b: &PairScore) -> bool {
    if a.blocked != b.blocked {
        return !a.blocked;
    }
    if a.blocked && a.busy != b.busy {
        return a.busy < b.busy;
    }
    a.score < b.score
}

fn score_pair(
    all: &[RankEntry],
    primary: u8,
    min_rounds: i32,
    blocked_pct: f64,
) -> Option<PairScore> {
    let other = ht40_pair_other(primary);
    if other == 0 {
        return None;
    }
    let (mut worst, mut blocked, mut busy, mut seen) = (0u32, false, 0.0f64, 0);
    for e in all.iter().filter(|e| e.ch == primary || e.ch == other) {
        if (e.visits as i64) < min_rounds as i64 {
            return None;
        }
        worst = worst.max(e.worst_busy);
        blocked = blocked || (e.busy_valid && e.worst_busy_pct >= blocked_pct);
        busy = busy.max(if e.busy_valid { e.worst_busy_pct } else { 0.0 });
        seen += 1;
    }
    (seen == 2).then_some(PairScore {
        primary,
        score: worst,
        blocked,
        busy,
    })
}

/// The winning pair's primary. Current's pair keeps `margin` within a tier;
/// an unblocked candidate beats a blocked current outright; ties stay on
/// current; config order breaks candidate ties. Current when nothing is
/// ranked, the best ranked candidate when current's pair is not.
pub fn pair_proposal(
    all: &[RankEntry],
    current: u8,
    set: &[u8],
    min_rounds: i32,
    margin: u32,
    blocked_pct: f64,
) -> u8 {
    let h = score_pair(all, current, min_rounds, blocked_pct);
    let mut best: Option<PairScore> = None;
    for &c in set.iter().filter(|&&c| c != current) {
        if let Some(s) = score_pair(all, c, min_rounds, blocked_pct) {
            if best.is_none_or(|b| better(&s, &b)) {
                best = Some(s);
            }
        }
    }
    let Some(best) = best else { return current };
    let Some(h) = h else { return best.primary };
    if h.blocked && !best.blocked {
        return best.primary;
    }
    if !better(&best, &h) {
        return current;
    }
    if best.score as u64 + margin as u64 <= h.score as u64 {
        best.primary
    } else {
        current
    }
}

/// Any pair in `set` ranked (both halves at `min_rounds`).
pub fn any_pair_ranked(all: &[RankEntry], set: &[u8], min_rounds: i32) -> bool {
    set.iter()
        .any(|&c| score_pair(all, c, min_rounds, 1e9).is_some())
}

/// Every pair in `set` ranked; false on an empty set.
pub fn all_pairs_ranked(all: &[RankEntry], set: &[u8], min_rounds: i32) -> bool {
    !set.is_empty()
        && set
            .iter()
            .all(|&c| score_pair(all, c, min_rounds, 1e9).is_some())
}

/// Ranked pairs, best first, in `pair_proposal`'s order; unranked skipped.
pub fn pair_ranking(all: &[RankEntry], set: &[u8], min_rounds: i32, blocked_pct: f64) -> Vec<u8> {
    let mut ps: Vec<PairScore> = set
        .iter()
        .filter_map(|&c| score_pair(all, c, min_rounds, blocked_pct))
        .collect();
    // Stable: config order on ties, like std::stable_sort with better().
    ps.sort_by(|a, b| {
        if better(a, b) {
            std::cmp::Ordering::Less
        } else if better(b, a) {
            std::cmp::Ordering::Greater
        } else {
            std::cmp::Ordering::Equal
        }
    });
    ps.into_iter().map(|p| p.primary).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const NB: f64 = 1e9;

    fn e(ch: u8, worst_busy: u32, visits: u32) -> RankEntry {
        RankEntry {
            ch,
            worst_busy,
            visits,
            floor_valid: false,
            floor_dbm: 0,
            busy_valid: false,
            worst_busy_pct: 0.0,
        }
    }
    fn re(ch: u8, busy: u32, bpct: f64) -> RankEntry {
        RankEntry {
            busy_valid: true,
            worst_busy_pct: bpct,
            ..e(ch, busy, 3)
        }
    }

    #[test]
    fn pairs_are_the_ht40_table() {
        assert_eq!(ht40_pair_other(136), 132);
        assert_eq!(ht40_pair_other(132), 136);
        assert_eq!(ht40_pair_other(144), 140);
        assert_eq!(ht40_pair_other(40), 36);
        assert_eq!(ht40_pair_other(128), 124);
        assert_eq!(ht40_pair_other(165), 0);
    }

    #[test]
    fn half_set_is_both_halves_of_home_and_candidates_in_config_order() {
        assert_eq!(
            scan_half_set(&[136, 144, 40, 128]),
            vec![132, 136, 140, 144, 36, 40, 124, 128]
        );
        assert_eq!(scan_half_set(&[136, 165]).len(), 2);
        assert_eq!(scan_half_set(&[136, 132]).len(), 2);
    }

    #[test]
    fn pair_score_is_the_worse_half_and_home_margin_applies_to_the_pair() {
        let all = [e(136, 0, 3), e(132, 500, 3), e(144, 100, 3), e(140, 120, 3)];
        assert_eq!(pair_proposal(&all, 136, &[136, 144], 3, 20, NB), 144);
        assert_eq!(pair_proposal(&all, 136, &[136, 144], 3, 400, NB), 136);
    }

    #[test]
    fn pair_unranked_until_both_halves_reach_min_rounds() {
        let all = [e(136, 0, 3), e(132, 0, 1), e(144, 50, 3), e(140, 50, 3)];
        assert_eq!(pair_proposal(&all, 136, &[136, 144], 3, 20, NB), 144);
        let none = [e(136, 0, 3), e(132, 0, 1), e(144, 50, 3), e(140, 50, 1)];
        assert_eq!(pair_proposal(&none, 136, &[136, 144], 3, 20, NB), 136);
        let half = [e(136, 0, 3), e(132, 0, 3), e(144, 0, 3)];
        assert_eq!(pair_proposal(&half, 136, &[136, 144], 3, 0, NB), 136);
    }

    #[test]
    fn pair_ties_stay_on_current_then_a_strictly_cleaner_pair_wins_at_margin_zero() {
        let mut all = vec![
            e(136, 10, 3),
            e(132, 10, 3),
            e(144, 10, 3),
            e(140, 10, 3),
            e(40, 10, 3),
            e(36, 10, 3),
        ];
        assert_eq!(pair_proposal(&all, 136, &[136, 144, 40], 3, 0, NB), 136);
        all[4].worst_busy = 5;
        all[5].worst_busy = 5;
        assert_eq!(pair_proposal(&all, 136, &[136, 144, 40], 3, 0, NB), 40);
    }

    #[test]
    fn any_pair_ranked_needs_both_halves_of_some_pair() {
        let one_half = [e(136, 0, 3), e(132, 0, 1), e(144, 0, 2), e(140, 0, 2)];
        assert!(!any_pair_ranked(&one_half, &[136, 144], 3));
        assert!(!any_pair_ranked(&[e(136, 0, 3)], &[136, 144], 3));
        assert!(!any_pair_ranked(&[], &[136, 144], 3));
        let cand = [e(136, 0, 3), e(132, 0, 1), e(144, 0, 3), e(140, 0, 3)];
        assert!(any_pair_ranked(&cand, &[136, 144], 3));
        let home = [e(136, 0, 3), e(132, 0, 4), e(144, 0, 1)];
        assert!(any_pair_ranked(&home, &[136, 144], 3));
    }

    #[test]
    fn one_blocked_half_blocks_the_pair() {
        let mut all = vec![
            re(132, 0, 0.0),
            re(136, 0, 0.0),
            re(140, 0, 90.0),
            re(144, 0, 0.0),
            re(60, 5, 0.0),
            re(64, 5, 0.0),
        ];
        assert_eq!(
            pair_proposal(&all, 136, &[136, 144, 64], 3, 1000, 30.0),
            136
        );
        all[0] = re(132, 0, 100.0);
        assert_eq!(pair_proposal(&all, 136, &[136, 144, 64], 3, 1000, 30.0), 64);
    }

    #[test]
    fn pair_ranking_is_best_first_and_skips_unranked() {
        let all = [
            e(136, 10, 3),
            e(132, 10, 3),
            e(144, 50, 3),
            e(140, 5, 3),
            e(40, 1, 3),
            e(36, 1, 3),
            e(112, 0, 1),
            e(108, 0, 1),
        ];
        assert_eq!(
            pair_ranking(&all, &[40, 136, 144, 112], 3, NB),
            vec![40, 136, 144]
        );
        assert!(!all_pairs_ranked(&all, &[40, 136, 144, 112], 3));
        assert!(all_pairs_ranked(&all, &[40, 136, 144], 3));
    }

    #[test]
    fn current_unranked_means_best_ranked_wins() {
        let all = [e(144, 50, 3), e(140, 50, 3)];
        assert_eq!(pair_proposal(&all, 136, &[136, 144], 3, 20, NB), 144);
        assert_eq!(pair_proposal(&[], 136, &[136, 144], 3, 20, NB), 136);
    }
}
