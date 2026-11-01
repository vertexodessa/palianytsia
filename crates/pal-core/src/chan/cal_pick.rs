//! The pick of a calibration run's measure round (maburgs
//! `cal_round_pick.h`): which member of the set the run calibrates on.
//!
//! Op and the candidates are read by different instruments: each candidate
//! half by the scout card's 20 MHz observe, op by the link cards' verdict
//! window at the link width (the scout never dwells on op while linked: op
//! carries the drone's video). Only terms that mean the same on both sides
//! are compared: NHM busy % (foreign airtime; op's has our own video taken
//! out) and decoded foreign frames per dwell_ms observe. The CCA term is
//! left out: on op, A-MPDU video clamps `cca - own` to zero while a
//! candidate keeps its CCA, so op would always look clean.
//!
//! score = foreign + busy % (the busy term only when both sides compared
//! have one). A pair scores as its worse half; a candidate with no busy
//! reading is not ranked while any candidate has one. Leaving op takes
//! beating it by `margin` in every tier: a blocked candidate never replaces
//! an unblocked op, and an unblocked one still has to be `margin` better
//! than a blocked op. Op unread, or no candidate read on every half: stay.
use crate::config::ht40_pair_other;

use super::ranker::RankEntry;

/// One reading of one channel: a 20 MHz half from the round's scout dwell,
/// or op from the verdict window.
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
    /// The K record's rows: every half read this round plus op's (both
    /// halves at width 40), worst_busy = the score above, one visit each.
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

fn halves_of(member: u8, width_mhz: u8) -> Vec<u8> {
    if width_mhz != 40 {
        return vec![member];
    }
    match ht40_pair_other(member) {
        0 => vec![member],
        other => vec![member, other],
    }
}

/// The worse half's terms; not ok unless every half has a reading.
fn member_side(rd: &[CalReading], member: u8, width_mhz: u8) -> Side {
    let mut s = Side {
        ok: true,
        busy_valid: true,
        ..Side::default()
    };
    for h in halves_of(member, width_mhz) {
        let (mut seen, mut busy_seen) = (false, false);
        for r in rd.iter().filter(|r| r.ch == h) {
            seen = true;
            s.foreign = s.foreign.max(r.foreign);
            if r.busy_valid {
                busy_seen = true;
                s.busy_pct = s.busy_pct.max(r.busy_pct);
            }
        }
        if !seen {
            s.ok = false;
        }
        if !busy_seen {
            s.busy_valid = false;
        }
    }
    if !s.busy_valid {
        s.busy_pct = 0.0;
    }
    s
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
    halves: &[CalReading],
    op_read: Option<CalReading>,
    op: u8,
    set: &[u8],
    width_mhz: u8,
    margin: u32,
    blocked_pct: f64,
) -> CalPickOut {
    let mut out = CalPickOut {
        best: op,
        rows: halves.iter().map(row).collect(),
    };
    let Some(o) = op_read else {
        return out;
    };
    for h in halves_of(op, width_mhz) {
        out.rows.push(row(&CalReading { ch: h, ..o }));
    }
    let o = Side {
        ok: true,
        foreign: o.foreign,
        busy_valid: o.busy_valid,
        busy_pct: if o.busy_valid { o.busy_pct } else { 0.0 },
    };
    // A candidate with no busy reading would score on foreign alone, a term
    // short of what the others pay: not eligible while any has one.
    let any_busy = set
        .iter()
        .filter(|&&c| c != op)
        .map(|&c| member_side(halves, c, width_mhz))
        .any(|s| s.ok && s.busy_valid);
    // Best candidate: unblocked first, then score, then config order.
    let mut best: Option<(u8, Side)> = None;
    for &c in set.iter().filter(|&&c| c != op) {
        let s = member_side(halves, c, width_mhz);
        if !s.ok || (any_busy && !s.busy_valid) {
            continue;
        }
        let Some((_, bs)) = best else {
            best = Some((c, s));
            continue;
        };
        let (sb, bb) = (blocked(&s, blocked_pct), blocked(&bs, blocked_pct));
        if sb != bb {
            if !sb {
                best = Some((c, s));
            }
            continue;
        }
        if score(&s, s.busy_valid) < score(&bs, bs.busy_valid) {
            best = Some((c, s));
        }
    }
    let Some((c, bs)) = best else {
        return out;
    };
    if blocked(&bs, blocked_pct) && !blocked(&o, blocked_pct) {
        return out;
    }
    let use_busy = bs.busy_valid && o.busy_valid;
    if score(&bs, use_busy) + f64::from(margin) <= score(&o, use_busy) {
        out.best = c;
    }
    out
}

#[cfg(test)]
mod tests {
    //! mabur tests/test_channel_core.cpp, the cal_pick_* cases.
    use super::*;

    const SET: [u8; 4] = [40, 64, 112, 144];

    fn halves_with(fr: &[(u8, u32)], busy: Option<f64>) -> Vec<CalReading> {
        fr.iter()
            .map(|&(ch, foreign)| CalReading {
                ch,
                foreign,
                busy_valid: busy.is_some(),
                busy_pct: busy.unwrap_or(0.0),
            })
            .collect()
    }

    fn rd(ch: u8, foreign: u32) -> CalReading {
        CalReading {
            ch,
            foreign,
            ..CalReading::default()
        }
    }

    fn rdb(ch: u8, foreign: u32, busy: f64) -> CalReading {
        CalReading {
            ch,
            foreign,
            busy_valid: true,
            busy_pct: busy,
        }
    }

    fn pick(h: &[CalReading], op: Option<CalReading>) -> u8 {
        cal_round_pick(h, op, 40, &SET, 40, 20, 50.0).best
    }

    #[test]
    fn op_slightly_worse_than_a_candidate_stays() {
        // 112's pair reads 15, op 30: better, but not by the margin (20).
        let h = halves_with(
            &[
                (60, 100),
                (64, 100),
                (108, 15),
                (112, 15),
                (140, 100),
                (144, 100),
            ],
            None,
        );
        assert_eq!(pick(&h, Some(rd(40, 30))), 40);
        // 20 better than op: it moves.
        assert_eq!(pick(&h, Some(rd(40, 35))), 112);
    }

    #[test]
    fn clearly_blocked_op_moves() {
        let h = halves_with(
            &[(60, 0), (64, 0), (108, 0), (112, 0), (140, 0), (144, 0)],
            Some(5.0),
        );
        assert_eq!(pick(&h, Some(rdb(40, 0, 90.0))), 64);
    }

    #[test]
    fn margin_holds_in_the_blocked_tier() {
        // Op just blocked (55 %), every candidate just under (45 %).
        let all = [(60, 0), (64, 0), (108, 0), (112, 0), (140, 0), (144, 0)];
        let h = halves_with(&all, Some(45.0));
        assert_eq!(pick(&h, Some(rdb(40, 0, 55.0))), 40);
        // A blocked candidate never replaces an unblocked op.
        let b = halves_with(&all, Some(60.0));
        assert_eq!(pick(&b, Some(rdb(40, 300, 40.0))), 40);
    }

    #[test]
    fn scores_a_pair_by_its_worse_half_and_needs_every_half() {
        // 108 (112's lower half) is busy: 112's pair scores 100, 144's 0.
        let h = halves_with(
            &[(60, 50), (64, 50), (108, 100), (112, 0), (140, 0), (144, 0)],
            None,
        );
        assert_eq!(pick(&h, Some(rd(40, 200))), 144);
        // 140 never read: 144 is not ranked; 64's pair is the best left.
        let m = halves_with(&[(60, 50), (64, 50), (108, 100), (112, 0), (144, 0)], None);
        assert_eq!(pick(&m, Some(rd(40, 200))), 64);
        // Op unread: stay.
        assert_eq!(pick(&h, None), 40);
        // K rows: every half read plus op's two halves, one visit each.
        let out = cal_round_pick(&h, Some(rd(40, 200)), 40, &SET, 40, 20, 50.0);
        assert_eq!(out.rows.len(), 8);
        assert!(out.rows.iter().all(|r| r.visits == 1));
    }

    #[test]
    fn a_candidate_without_a_busy_reading_never_outranks_one_with() {
        let h = [
            rdb(60, 10, 20.0),
            rdb(64, 10, 20.0),
            rd(108, 0),
            rdb(112, 0, 0.0),
            rdb(140, 5, 10.0),
            rdb(144, 5, 10.0),
        ];
        assert_eq!(pick(&h, Some(rdb(40, 100, 30.0))), 144);
        // With no busy reading anywhere, foreign alone still ranks them.
        let f = halves_with(
            &[(60, 50), (64, 50), (108, 0), (112, 0), (140, 5), (144, 5)],
            None,
        );
        assert_eq!(pick(&f, Some(rd(40, 100))), 112);
    }

    #[test]
    fn width_20_compares_members() {
        let h = halves_with(&[(40, 90), (44, 0)], None);
        assert_eq!(
            cal_round_pick(&h, Some(rd(36, 50)), 36, &[36, 40, 44], 20, 20, 50.0).best,
            44
        );
    }
}
