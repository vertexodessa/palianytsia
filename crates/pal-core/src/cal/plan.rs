//! The GS's sweep commands and how long the drone airs each (mabur
//! `cal_plan.h`). Pure.

use pal_wire::cal::{floor_sweep_max_ms, Kind, SweepCmd, SweepSpec, WALL_LEAVE};

use crate::config::Rung;

/// One full-resolution pass over the relative index range. -41, not -40,
/// so the last cell lands exactly on the +63 rail a no-dip row parks at:
/// the rail must be a cell this sweep measured.
pub const SWEEP_LO: i32 = -41;
pub const SWEEP_HI: i32 = 63;
/// rc_proto.h kRailRel.
pub const RAIL_REL: i32 = 63;
const _: () = assert!(SWEEP_HI == RAIL_REL);

pub const POWER_FRAMES: u16 = 20;
pub const POWER_SETTLE_MS: u16 = 5;
pub const GAP_US: u16 = 1000;
pub const VERIFY_FRAMES: u16 = 20;
pub const CAP_DWELL_MS: u16 = 2000;
pub const CAP_SETTLE_MS: u16 = 300;

pub fn make_power_sweep(nonce: u32, seq: u8, bw: u8) -> SweepCmd {
    SweepCmd {
        nonce,
        seq,
        kind: Kind::Power,
        bw,
        frames_per_cell: POWER_FRAMES,
        settle_ms: POWER_SETTLE_MS,
        gap_us: GAP_US,
        idx_lo: SWEEP_LO as i8,
        idx_hi: SWEEP_HI as i8,
        ..SweepCmd::default()
    }
}

/// The drone reads settle_ms as its per-point settle for capacity.
pub fn make_capacity_sweep(nonce: u32, seq: u8, bw: u8) -> SweepCmd {
    SweepCmd {
        nonce,
        seq,
        kind: Kind::Capacity,
        bw,
        settle_ms: CAP_SETTLE_MS,
        dwell_ms: CAP_DWELL_MS,
        ..SweepCmd::default()
    }
}

/// The (rate, parked index) pairs a verify pass flies, in rate order.
pub fn verify_cells(park_rel: &[i32; 8]) -> Vec<(u8, i32)> {
    (0u8..8)
        .filter(|&r| park_rel[r as usize] != WALL_LEAVE as i32)
        .map(|r| (r, park_rel[r as usize]))
        .collect()
}

/// How long the drone airs a SweepCmd, counted as its engine walks it. A
/// floor sweep: the longest walk the rules allow; the drone's SweepDone
/// normally ends the window well before it.
pub fn sweep_duration_ms(c: &SweepCmd) -> u32 {
    if c.kind == Kind::Capacity {
        return c.mcs_mask.count_ones() * (u32::from(c.settle_ms) + u32::from(c.dwell_ms));
    }
    if c.kind == Kind::Floor {
        return floor_sweep_max_ms(c);
    }
    let cells = 8 * (i32::from(c.idx_hi) - i32::from(c.idx_lo) + 1).max(0) as u32;
    let per_cell =
        u32::from(c.settle_ms) + u32::from(c.frames_per_cell) * u32::from(c.gap_us) / 1000;
    cells * per_cell
}

pub fn verify_duration_ms(cells: usize) -> u32 {
    let per_cell = u32::from(POWER_SETTLE_MS) + u32::from(VERIFY_FRAMES) * u32::from(GAP_US) / 1000;
    cells as u32 * per_cell
}

/// What a bare start runs (maburgs `default_sweeps`). A peer that tunes
/// itself to 40 for a 40 MHz sweep (`peer_wide`, CAP_CAL_WIDE, with GS
/// cards that follow) gets both widths whatever the link flies: power20,
/// cap20, the floor, power40, cap40. Otherwise power then capacity at 20
/// MHz, and at 40 when the GS flies 40 and the ladder has a 40 MHz rung.
pub fn default_sweeps(radio_width: u8, ladder: &[Rung], peer_wide: bool) -> Vec<SweepSpec> {
    let mut out = vec![
        SweepSpec {
            kind: Kind::Power,
            bw: 20,
        },
        SweepSpec {
            kind: Kind::Capacity,
            bw: 20,
        },
    ];
    if peer_wide {
        out.extend(
            [Kind::Floor, Kind::Power, Kind::Capacity].map(|kind| SweepSpec { kind, bw: 40 }),
        );
        return out;
    }
    if radio_width == 40 && ladder.iter().any(|r| r.bw == 40) {
        out.push(SweepSpec {
            kind: Kind::Power,
            bw: 40,
        });
        out.push(SweepSpec {
            kind: Kind::Capacity,
            bw: 40,
        });
    }
    out
}

/// The widest sweep a run may ask for: the narrower of the GS's width and
/// the linked drone's agreed one (0 = not heard yet).
pub fn effective_width(gs_width: u8, linked: bool, peer_width: u8) -> u8 {
    if !linked || peer_width == 0 {
        gs_width
    } else {
        gs_width.min(peer_width)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pal_wire::cal::park_rel_for;

    #[test]
    fn power_sweep_is_full_resolution_twenty_frames() {
        let c = make_power_sweep(42, 3, 20);
        assert_eq!((c.nonce, c.seq, c.kind, c.bw), (42, 3, Kind::Power, 20));
        assert_eq!(
            (c.frames_per_cell, c.settle_ms, c.gap_us),
            (POWER_FRAMES, POWER_SETTLE_MS, GAP_US)
        );
        assert_eq!((c.idx_lo as i32, c.idx_hi as i32), (SWEEP_LO, SWEEP_HI));
        // 840 cells * 25 ms.
        assert_eq!(sweep_duration_ms(&c), 21000);
    }

    #[test]
    fn capacity_sweep_is_eight_two_second_points() {
        let c = make_capacity_sweep(5, 1, 40);
        assert_eq!((c.nonce, c.seq, c.kind, c.bw), (5, 1, Kind::Capacity, 40));
        assert_eq!(
            (c.settle_ms, c.dwell_ms, c.mcs_mask),
            (CAP_SETTLE_MS, CAP_DWELL_MS, 0xFF)
        );
        assert_eq!(sweep_duration_ms(&c), 18400);
    }

    #[test]
    fn verify_cells_skip_undetermined() {
        let park = [59, WALL_LEAVE as i32, 59, 38, 16, -3, -6, -8];
        let cells = verify_cells(&park);
        assert_eq!(cells.len(), 7);
        assert!(cells.iter().all(|c| c.0 != 1));
        assert_eq!(cells[0], (0, 59));
        assert_eq!(*cells.last().unwrap(), (7, -8));
        assert_eq!(verify_duration_ms(cells.len()), 175);
    }

    #[test]
    fn default_sweeps_add_40_only_when_both_fly_it() {
        let rung = |bw| Rung {
            mcs: 0,
            overhead_base: 0.0,
            overhead_enh: 0.0,
            bw,
        };
        let (r20, r40) = (rung(20), rung(40));
        assert_eq!(default_sweeps(20, &[r20, r40], false).len(), 2);
        assert_eq!(default_sweeps(40, &[r20], false).len(), 2);
        let s = default_sweeps(40, &[r20, r40], false);
        assert_eq!(s.len(), 4);
        assert_eq!(
            s[2],
            SweepSpec {
                kind: Kind::Power,
                bw: 40
            }
        );
        assert_eq!(effective_width(40, true, 20), 20);
        assert_eq!(effective_width(40, true, 0), 40);
        assert_eq!(effective_width(40, false, 20), 40);
    }

    /// mabur test_cal_control: a wide peer gets both widths and the floor
    /// whatever the GS's own width and ladder.
    #[test]
    fn a_wide_peer_gets_both_widths_and_the_floor() {
        let r20 = Rung {
            mcs: 0,
            overhead_base: 0.0,
            overhead_enh: 0.0,
            bw: 20,
        };
        let s = default_sweeps(20, &[r20], true);
        let toks: Vec<(Kind, u8)> = s.iter().map(|s| (s.kind, s.bw)).collect();
        assert_eq!(
            toks,
            [
                (Kind::Power, 20),
                (Kind::Capacity, 20),
                (Kind::Floor, 40),
                (Kind::Power, 40),
                (Kind::Capacity, 40)
            ]
        );
        assert_eq!(default_sweeps(40, &[r20], true), s);
    }

    #[test]
    fn a_floor_sweep_lasts_its_longest_walk() {
        assert_eq!(
            sweep_duration_ms(&pal_wire::cal::make_floor_sweep(1, 2)),
            72_500
        );
    }

    #[test]
    fn the_grid_ends_on_the_rail() {
        assert_eq!(make_power_sweep(1, 0, 20).idx_hi as i32, RAIL_REL);
    }

    #[test]
    fn verify_cells_includes_a_clamped_park() {
        let mut walls = [WALL_LEAVE; 8];
        walls[2] = -64;
        let park = park_rel_for(&walls, 1.0);
        assert_eq!(park[2], -64);
        assert_eq!(verify_cells(&park), vec![(2, -64)]);
    }
}
