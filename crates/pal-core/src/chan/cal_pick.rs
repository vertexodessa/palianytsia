//! The pick of a calibration run's measure round (maburgs
//! `cal_round_pick.h`): which member of the set the run calibrates on.
//!
//! Op and the candidates are read by different instruments: each candidate
//! by the scout card's own observe at the link width (at 40 MHz one dwell
//! on the pair's primary reads the whole pair), op by the link cards'
//! verdict window (the scout never dwells on op while linked: op carries
//! the drone's video). Only terms that mean the same on both sides are
//! compared: NHM busy % (foreign airtime; op's has our own video taken
//! out) and decoded foreign frames per dwell_ms observe. The CCA term is
//! left out: on op, A-MPDU video clamps `cca - own` to zero while a
//! candidate keeps its CCA, so op would always look clean.
//!
//! The best candidate mirrors pair_pick's order: unblocked first, a busy
//! reading before none, lower busy %, lower foreign + busy % score, config
//! order. A candidate with no busy reading is not ranked while any has
//! one. Leaving op is the ranker's margin rule: a better tier, or a busy
//! reading where op has none, moves at once; both read, the best must beat
//! op's busy % by `margin_pct` points. A tie, or neither reading, stays.

use super::ranker::RankEntry;

/// One reading of one channel: a member from the round's scout dwell, or
/// op from the verdict window.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CalReading {
    pub ch: u8,
    /// Decoded foreign frames, per dwell_ms observe.
    pub foreign: u32,
    pub busy_valid: bool,
    /// NHM foreign busy airtime, %.
    pub busy_pct: f64,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct CalPickOut {
    pub best: u8,
    /// The K record's rows: every member read this round plus op's,
    /// worst_busy = the score above, one visit each.
    pub rows: Vec<RankEntry>,
}

#[derive(Clone, Copy, Debug, Default)]
struct Side {
    ok: bool,
    foreign: u32,
    busy_valid: bool,
    busy_pct: f64,
}

fn score(s: &Side, use_busy: bool) -> f64 {
    f64::from(s.foreign) + if use_busy { s.busy_pct } else { 0.0 }
}

fn blocked(s: &Side, blocked_pct: f64) -> bool {
    s.busy_valid && s.busy_pct >= blocked_pct
}

/// The member's own reading; not ok unless it was read this round.
fn member_side(rd: &[CalReading], member: u8) -> Side {
    rd.iter()
        .find(|r| r.ch == member)
        .map(|r| Side {
            ok: true,
            foreign: r.foreign,
            busy_valid: r.busy_valid,
            busy_pct: if r.busy_valid { r.busy_pct } else { 0.0 },
        })
        .unwrap_or_default()
}

/// Strict: equal candidates are not better (config order on ties).
fn better(a: &Side, b: &Side, blocked_pct: f64) -> bool {
    let (ab, bb) = (blocked(a, blocked_pct), blocked(b, blocked_pct));
    if ab != bb {
        return !ab;
    }
    if a.busy_valid != b.busy_valid {
        return a.busy_valid;
    }
    if a.busy_valid && a.busy_pct != b.busy_pct {
        return a.busy_pct < b.busy_pct;
    }
    score(a, a.busy_valid) < score(b, b.busy_valid)
}

fn row(r: &CalReading) -> RankEntry {
    let side = Side {
        ok: true,
        foreign: r.foreign,
        busy_valid: r.busy_valid,
        busy_pct: r.busy_pct,
    };
    RankEntry {
        ch: r.ch,
        worst_busy: score(&side, r.busy_valid).round() as u32,
        visits: 1,
        busy_valid: r.busy_valid,
        worst_busy_pct: r.busy_pct,
        ..RankEntry::default()
    }
}

pub fn cal_round_pick(
    members: &[CalReading],
    op_read: Option<CalReading>,
    op: u8,
    set: &[u8],
    margin_pct: f64,
    blocked_pct: f64,
) -> CalPickOut {
    let mut out = CalPickOut {
        best: op,
        rows: Vec::new(),
    };
    let mut any_busy = false;
    for &c in set.iter().filter(|&&c| c != op) {
        let s = member_side(members, c);
        if !s.ok {
            continue;
        }
        out.rows.push(row(&CalReading {
            ch: c,
            foreign: s.foreign,
            busy_valid: s.busy_valid,
            busy_pct: s.busy_pct,
        }));
        any_busy |= s.busy_valid;
    }
    let Some(o) = op_read else {
        return out;
    };
    out.rows.push(row(&o));
    let o = Side {
        ok: true,
        foreign: o.foreign,
        busy_valid: o.busy_valid,
        busy_pct: if o.busy_valid { o.busy_pct } else { 0.0 },
    };
    let mut best: Option<(u8, Side)> = None;
    for &c in set.iter().filter(|&&c| c != op) {
        let s = member_side(members, c);
        if !s.ok || (any_busy && !s.busy_valid) {
            continue;
        }
        if best.is_none_or(|(_, bs)| better(&s, &bs, blocked_pct)) {
            best = Some((c, s));
        }
    }
    let Some((bc, bs)) = best else {
        return out;
    };
    let (bb, ob) = (blocked(&bs, blocked_pct), blocked(&o, blocked_pct));
    if bb != ob {
        if !bb {
            out.best = bc; // a blocked candidate never replaces an unblocked op
        }
        return out;
    }
    if bs.busy_valid != o.busy_valid {
        if bs.busy_valid {
            out.best = bc;
        }
        return out;
    }
    if bs.busy_valid && o.busy_pct - bs.busy_pct >= margin_pct {
        out.best = bc;
    }
    out
}

#[cfg(test)]
mod tests {
    //! mabur `test_channel_core.cpp`, "the calibration round's pick rule".
    use super::*;

    const SET: [u8; 4] = [40, 64, 112, 144];

    fn r(ch: u8, foreign: u32, busy: Option<f64>) -> CalReading {
        CalReading {
            ch,
            foreign,
            busy_valid: busy.is_some(),
            busy_pct: busy.unwrap_or(0.0),
        }
    }
    fn all(busy: Option<f64>) -> Vec<CalReading> {
        [64, 112, 144].iter().map(|&c| r(c, 0, busy)).collect()
    }
    fn pick(h: &[CalReading], op: Option<CalReading>) -> u8 {
        cal_round_pick(h, op, 40, &SET, 20.0, 50.0).best
    }

    #[test]
    fn busy_margin_decides_among_read_candidates() {
        let h = [
            r(64, 0, Some(40.0)),
            r(112, 0, Some(15.0)),
            r(144, 0, Some(40.0)),
        ];
        assert_eq!(pick(&h, Some(r(40, 0, Some(30.0)))), 40);
        assert_eq!(pick(&h, Some(r(40, 0, Some(35.0)))), 112);
    }

    #[test]
    fn busy_tie_stays_on_op() {
        let h = [
            r(64, 0, Some(30.0)),
            r(112, 0, Some(30.0)),
            r(144, 0, Some(50.0)),
        ];
        assert_eq!(pick(&h, Some(r(40, 300, Some(30.0)))), 40);
    }

    #[test]
    fn tiers_move_at_once_one_way_only() {
        assert_eq!(pick(&all(Some(5.0)), Some(r(40, 0, Some(90.0)))), 64);
        assert_eq!(pick(&all(Some(45.0)), Some(r(40, 0, Some(55.0)))), 64);
        assert_eq!(pick(&all(Some(60.0)), Some(r(40, 300, Some(40.0)))), 40);
    }

    #[test]
    fn rows_and_unread_members() {
        let out = cal_round_pick(
            &all(Some(5.0)),
            Some(r(40, 0, Some(5.0))),
            40,
            &SET,
            20.0,
            50.0,
        );
        assert_eq!(out.rows.len(), 4);
        let h = [r(64, 0, Some(10.0)), r(112, 0, Some(40.0))];
        assert_eq!(pick(&h, Some(r(40, 0, Some(50.0)))), 64);
        assert_eq!(pick(&h, None), 40);
        let out = cal_round_pick(&h, Some(r(40, 0, Some(50.0))), 40, &SET, 20.0, 50.0);
        assert_eq!(out.rows.len(), 3);
        assert!(out.rows.iter().all(|e| e.visits == 1));
    }

    #[test]
    fn a_candidate_without_a_busy_reading_never_outranks_one_with() {
        let h = [
            r(64, 10, Some(20.0)),
            r(112, 0, None),
            r(144, 5, Some(10.0)),
        ];
        assert_eq!(pick(&h, Some(r(40, 100, Some(30.0)))), 144);
        let f = [r(64, 50, None), r(112, 0, None), r(144, 5, None)];
        assert_eq!(pick(&f, Some(r(40, 100, None))), 40);
    }

    #[test]
    fn best_ranks_on_busy_before_score() {
        let h = [r(64, 2, Some(20.0)), r(112, 30, Some(5.0))];
        assert_eq!(pick(&h, Some(r(40, 0, Some(30.0)))), 112);
        assert_eq!(pick(&h, Some(r(40, 0, Some(20.0)))), 40);
    }

    #[test]
    fn width_20_compares_members() {
        let h = [r(40, 0, Some(40.0)), r(44, 0, Some(5.0))];
        let out = cal_round_pick(
            &h,
            Some(r(36, 0, Some(50.0))),
            36,
            &[36, 40, 44],
            20.0,
            50.0,
        );
        assert_eq!(out.best, 44);
    }
}
