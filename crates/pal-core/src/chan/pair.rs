//! mabur `pair_pick.{h,cpp}`: the 40 MHz boot pick over one RankEntry row
//! per pair, keyed by its primary. The scout card is tuned 40, so one dwell
//! on the primary reads the whole pair: NHM busy % over both halves, frame
//! terms from the primary. Order: tier (unblocked first), a busy reading
//! before none, lower busy %, lower frame score, config order.

use super::ranker::RankEntry;

/// None = unranked: never visited, or short of `min_rounds`.
fn row_for(all: &[RankEntry], primary: u8, min_rounds: i32) -> Option<RankEntry> {
    let e = all.iter().find(|e| e.ch == primary)?;
    ((e.visits as i64) >= min_rounds as i64).then_some(*e)
}

fn blocked(e: &RankEntry, blocked_pct: f64) -> bool {
    e.busy_valid && e.worst_busy_pct >= blocked_pct
}

/// Strict: equal rows are not better, so callers keep config order on ties.
fn better(a: &RankEntry, b: &RankEntry, blocked_pct: f64) -> bool {
    let (ab, bb) = (blocked(a, blocked_pct), blocked(b, blocked_pct));
    if ab != bb {
        return !ab;
    }
    if a.busy_valid != b.busy_valid {
        return a.busy_valid;
    }
    if a.busy_valid && a.worst_busy_pct != b.worst_busy_pct {
        return a.worst_busy_pct < b.worst_busy_pct;
    }
    a.worst_busy < b.worst_busy
}

/// The winning primary. Current's pair keeps the pick unless the best
/// candidate is in a better tier, reads busy where current does not, or
/// both read and it beats current's busy by `margin_pct` points. An exact
/// busy tie, or neither side reading, stays. Current when nothing is
/// ranked, the best ranked candidate when current is not.
pub fn pair_proposal(
    all: &[RankEntry],
    current: u8,
    set: &[u8],
    min_rounds: i32,
    margin_pct: f64,
    blocked_pct: f64,
) -> u8 {
    let h = row_for(all, current, min_rounds);
    let mut best: Option<RankEntry> = None;
    for &c in set.iter().filter(|&&c| c != current) {
        if let Some(r) = row_for(all, c, min_rounds) {
            if best.is_none_or(|b| better(&r, &b, blocked_pct)) {
                best = Some(r);
            }
        }
    }
    let Some(best) = best else { return current };
    let Some(h) = h else { return best.ch };
    if !better(&best, &h, blocked_pct) {
        return current;
    }
    if blocked(&best, blocked_pct) != blocked(&h, blocked_pct) || best.busy_valid != h.busy_valid {
        return best.ch;
    }
    if best.busy_valid && best.worst_busy_pct != h.worst_busy_pct {
        return if h.worst_busy_pct - best.worst_busy_pct >= margin_pct {
            best.ch
        } else {
            current
        };
    }
    current
}

/// Any pair in `set` ranked (its primary's row at `min_rounds`).
pub fn any_pair_ranked(all: &[RankEntry], set: &[u8], min_rounds: i32) -> bool {
    set.iter().any(|&c| row_for(all, c, min_rounds).is_some())
}

/// Every pair in `set` ranked; false on an empty set.
pub fn all_pairs_ranked(all: &[RankEntry], set: &[u8], min_rounds: i32) -> bool {
    !set.is_empty() && set.iter().all(|&c| row_for(all, c, min_rounds).is_some())
}

/// Ranked pairs, best first, in `pair_proposal`'s order; unranked skipped.
pub fn pair_ranking(all: &[RankEntry], set: &[u8], min_rounds: i32, blocked_pct: f64) -> Vec<u8> {
    let mut rows: Vec<RankEntry> = set
        .iter()
        .filter_map(|&c| row_for(all, c, min_rounds))
        .collect();
    // Stable: config order on ties, like std::stable_sort with better().
    rows.sort_by(|a, b| {
        if better(a, b, blocked_pct) {
            std::cmp::Ordering::Less
        } else if better(b, a, blocked_pct) {
            std::cmp::Ordering::Greater
        } else {
            std::cmp::Ordering::Equal
        }
    });
    rows.into_iter().map(|r| r.ch).collect()
}

#[cfg(test)]
mod tests {
    //! mabur `test_pair_pick.cpp`.
    use super::*;

    fn e(ch: u8, worst: u32, visits: u32, bv: bool, busy: f64) -> RankEntry {
        RankEntry {
            ch,
            worst_busy: worst,
            visits,
            busy_valid: bv,
            worst_busy_pct: busy,
            ..Default::default()
        }
    }

    #[test]
    fn pair_ranked_on_the_primary_row_alone() {
        let all = [e(136, 0, 3, true, 5.0), e(144, 0, 2, true, 5.0)];
        assert!(any_pair_ranked(&all, &[136], 3) && !any_pair_ranked(&all, &[144], 3));
        assert!(!all_pairs_ranked(&all, &[136, 144], 3) && all_pairs_ranked(&all, &[136, 144], 2));
    }

    #[test]
    fn lower_busy_pct_beats_lower_frame_score() {
        let all = [e(136, 10, 3, true, 40.0), e(144, 500, 3, true, 12.0)];
        assert_eq!(pair_proposal(&all, 136, &[136, 144], 3, 10.0, 1e9), 144);
        assert_eq!(pair_ranking(&all, &[136, 144], 3, 1e9)[0], 144);
    }

    #[test]
    fn margin_pct_holds_current_pair() {
        let all = [e(136, 10, 3, true, 20.0), e(144, 0, 3, true, 12.0)];
        assert_eq!(pair_proposal(&all, 136, &[136, 144], 3, 10.0, 1e9), 136);
        assert_eq!(pair_proposal(&all, 136, &[136, 144], 3, 8.0, 1e9), 144);
    }

    #[test]
    fn busy_tie_stays_on_current() {
        let all = [e(136, 100, 3, true, 15.0), e(144, 70, 3, true, 15.0)];
        assert_eq!(pair_proposal(&all, 136, &[136, 144], 3, 10.0, 1e9), 136);
        assert_eq!(pair_proposal(&all, 136, &[136, 144], 3, 0.0, 1e9), 136);
    }

    #[test]
    fn blocked_pair_ranks_last_and_blocked_current_loses_to_any_unblocked() {
        let all = [e(136, 0, 3, true, 60.0), e(144, 900, 3, true, 30.0)];
        assert_eq!(pair_proposal(&all, 136, &[136, 144], 3, 100.0, 50.0), 144);
        assert_eq!(pair_ranking(&all, &[136, 144], 3, 50.0)[0], 144);
    }

    #[test]
    fn pair_without_busy_reading_ranks_after_pairs_with_one() {
        let all = [e(136, 0, 3, false, 0.0), e(144, 300, 3, true, 45.0)];
        assert_eq!(pair_ranking(&all, &[136, 144], 3, 1e9)[0], 144);
        assert_eq!(pair_proposal(&all, 136, &[136, 144], 3, 10.0, 1e9), 144);
    }

    #[test]
    fn unranked_current_yields_to_best_ranked_and_nothing_ranked_stays() {
        let a = [e(136, 0, 1, true, 0.0), e(144, 50, 3, true, 20.0)];
        assert_eq!(pair_proposal(&a, 136, &[136, 144], 3, 10.0, 1e9), 144);
        let b = [e(136, 0, 1, true, 0.0), e(144, 50, 1, true, 20.0)];
        assert_eq!(pair_proposal(&b, 136, &[136, 144], 3, 10.0, 1e9), 136);
    }

    #[test]
    fn config_order_breaks_full_ties() {
        let all = [e(144, 10, 3, true, 10.0), e(136, 10, 3, true, 10.0)];
        assert_eq!(pair_ranking(&all, &[136, 144], 3, 1e9)[0], 136);
    }
}
