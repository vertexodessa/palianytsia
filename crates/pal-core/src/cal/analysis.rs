//! Per-rate wall finding and capacity efficiency (mabur `cal_analysis`).
//! Pure: one row of cells in, one wall out.

use pal_wire::profile::{phy_rate_mbps, PhyMode};

use super::plan::RAIL_REL;

/// This card measured no RSSI in this cell.
pub const RSSI_NONE: i32 = -999;
/// Undetermined: the caller leaves the config entry untouched.
pub const NO_WALL: i32 = -128;

/// One (rate, idx) measurement; `expected` comes from the GS's own plan.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CalCell {
    pub idx: i32,
    pub expected: u16,
    /// Per card, CRC-clean and parseable.
    pub received: [u16; 2],
    /// CRC-bad, diagnostics only.
    pub corrupt: u16,
    pub rssi_dbm: [i32; 2],
    pub have_rssi: [bool; 2],
}

/// Never fell below threshold; wall = the rail.
pub const FLAG_NO_DIP: u32 = 1 << 0;
/// Never reached threshold; no wall derivable.
pub const FLAG_UNDETERMINED: u32 = 1 << 1;
pub const FLAG_NARROW: u32 = 1 << 2;
/// RX front-end compression suspected.
pub const FLAG_SATURATED: u32 = 1 << 3;
pub const FLAG_CARD_DISAGREE: u32 = 1 << 4;
// 1 << 5 (drift) retired, never reused.
/// Sender rarely blocked: measured CPU/USB, not air.
pub const FLAG_TX_BOUND: u32 = 1 << 6;
pub const FLAG_LOSSY: u32 = 1 << 7;
/// Under half arrived: not written.
pub const FLAG_LOST: u32 = 1 << 8;
/// Too few frames or too short a span to time: eff over the dwell.
pub const FLAG_SHORT_SPAN: u32 = 1 << 9;
/// The run passed a single failing cell that had clean cells above it.
pub const FLAG_BRIDGED: u32 = 1 << 10;
/// The wall sits under a faster rate's wall (`mark_under_faster`).
pub const FLAG_UNDER_FASTER: u32 = 1 << 11;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CalThresholds {
    pub deliver_pct: f64,
    pub sat_rssi_dbm: f64,
    pub narrow_span: i32,
    /// A single failing cell does not end the run when this many cells right
    /// above it deliver at least `bridge_pct`: more power delivered again, so
    /// the cell lost its frames to the channel, not to the PA. 0 = every
    /// failing cell ends it.
    pub bridge_clean: usize,
    /// Near-clean (19 of 20), not merely over the wall threshold: right at
    /// the edge a dip followed by 18 20 19 (a 20 MHz mcs4 sweep) is the slope
    /// starting, and passing it put the wall 4 indices (1 dB) high.
    pub bridge_pct: f64,
    pub card_disagree: i32,
    pub tx_bound_blocked_pct: i32,
    pub lossy_ratio: f64,
    pub lost_ratio: f64,
    pub span_min_frames: u32,
    pub span_min_ms: u32,
}

impl Default for CalThresholds {
    fn default() -> Self {
        Self {
            deliver_pct: 90.0,
            sat_rssi_dbm: -45.0,
            narrow_span: 4,
            bridge_clean: 3,
            bridge_pct: 95.0,
            card_disagree: 2,
            tx_bound_blocked_pct: 50,
            lossy_ratio: 0.9,
            lost_ratio: 0.5,
            span_min_frames: 20,
            span_min_ms: 500,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RateWall {
    pub wall: i32,
    pub floor_idx: i32,
    pub best_card: i32,
    pub flags: u32,
}

impl Default for RateWall {
    fn default() -> Self {
        Self {
            wall: NO_WALL,
            floor_idx: NO_WALL,
            best_card: -1,
            flags: 0,
        }
    }
}

fn delivery_pct(c: &CalCell, card: usize) -> f64 {
    if c.expected == 0 {
        return 0.0;
    }
    100.0 * f64::from(c.received[card]) / f64::from(c.expected)
}

/// (floor_idx, wall, bridged) of the first contiguous run at or above `pct`;
/// `bridged` counts the single failing cells it passed over because the
/// `bridge_clean` cells right above each reached `bridge_pct`.
fn first_run(
    cells: &[CalCell],
    card: usize,
    pct: f64,
    bridge_clean: usize,
    bridge_pct: f64,
) -> (i32, i32, u32) {
    let ok = |c: &CalCell| delivery_pct(c, card) >= pct;
    let clean = |c: &CalCell| delivery_pct(c, card) >= bridge_pct;
    let Some(mut i) = cells.iter().position(ok) else {
        return (NO_WALL, NO_WALL, 0);
    };
    let floor = cells[i].idx;
    let (mut wall, mut bridged) = (floor, 0);
    while i < cells.len() {
        if ok(&cells[i]) {
            wall = cells[i].idx;
        } else if bridge_clean > 0
            && i + bridge_clean < cells.len()
            && cells[i + 1..=i + bridge_clean].iter().all(clean)
        {
            bridged += 1;
        } else {
            break;
        }
        i += 1;
    }
    (floor, wall, bridged)
}

/// `cells` sorted ascending by idx. The wall is the END OF THE FIRST
/// CONTIGUOUS run at or above `deliver_pct`: past the first dip delivery
/// is a comb, and an island there is not headroom. One exception: a single
/// failing cell with `bridge_clean` cells right above it that each reach
/// `bridge_pct` does not end the run (`FLAG_BRIDGED`); with 20 frames a cell
/// three frames lost to the channel fail it, and the comb never shows three
/// near-clean cells in a row straight after its first dip. Scored on the best single card, never the
/// union (diversity does not protect against PA compression).
pub fn analyze_rate(cells: &[CalCell], th: &CalThresholds) -> RateWall {
    let mut out = RateWall::default();
    if cells.is_empty() {
        out.flags |= FLAG_UNDETERMINED;
        return out;
    }
    let mut best = 0usize;
    let mut best_total: i64 = -1;
    for card in 0..2 {
        let total: i64 = cells.iter().map(|c| i64::from(c.received[card])).sum();
        if total > best_total {
            best_total = total;
            best = card;
        }
    }
    out.best_card = best as i32;
    // Before any early return: an undetermined row most needs this flag.
    let peak = cells
        .iter()
        .filter(|c| c.have_rssi[best])
        .map(|c| c.rssi_dbm[best])
        .fold(RSSI_NONE, i32::max);
    if peak != RSSI_NONE && f64::from(peak) > th.sat_rssi_dbm {
        out.flags |= FLAG_SATURATED;
    }
    let (floor, wall, bridged) =
        first_run(cells, best, th.deliver_pct, th.bridge_clean, th.bridge_pct);
    out.floor_idx = floor;
    if bridged > 0 {
        out.flags |= FLAG_BRIDGED;
    }
    if floor == NO_WALL {
        out.flags |= FLAG_UNDETERMINED;
        return out;
    }
    if wall >= cells.last().unwrap().idx {
        // No wall inside the sweep: park at the rail, itself a swept cell.
        out.flags |= FLAG_NO_DIP;
        out.wall = RAIL_REL;
    } else {
        out.wall = wall;
    }
    if out.wall - floor <= th.narrow_span {
        out.flags |= FLAG_NARROW;
    }
    let other = 1 - best;
    if cells.iter().any(|c| c.received[other] > 0) {
        let (ofloor, owall, _) =
            first_run(cells, other, th.deliver_pct, th.bridge_clean, th.bridge_pct);
        // Compared against the raw run end, as the C++ does.
        if ofloor != NO_WALL && (owall - wall).abs() > th.card_disagree {
            out.flags |= FLAG_CARD_DISAGREE;
        }
    }
    out
}

/// A slower rate tolerates more transmitter distortion than a faster one, so
/// its compression wall cannot be the lower of the two: a wall under a
/// faster rate's wall was cut short by loss that had nothing to do with
/// power. Such rows get `FLAG_UNDER_FASTER`; the walls stay as measured.
pub fn mark_under_faster(walls: &mut [RateWall]) {
    // The highest wall among the rates above the current one.
    let mut faster = NO_WALL;
    for w in walls.iter_mut().rev() {
        if w.wall == NO_WALL {
            continue;
        }
        if faster != NO_WALL && w.wall < faster {
            w.flags |= FLAG_UNDER_FASTER;
        }
        faster = faster.max(w.wall);
    }
}

/// One capacity dwell at a single (mcs, bw), per card.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CapCell {
    pub offered: u32,
    pub blocked_pct: u8,
    pub frames: [u32; 2],
    pub air_bits: [u64; 2],
    pub first_bits: [u64; 2],
    pub first_ms: [u64; 2],
    pub last_ms: [u64; 2],
    pub rssi_dbm: [i32; 2],
    pub have_rssi: [bool; 2],
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CapResult {
    pub eff: f64,
    pub best_card: i32,
    pub span_ms: u32,
    pub flags: u32,
}

/// Delivered air efficiency, timed by the GS's receive clock on the best
/// card: (air_bits - first_bits) / (span_s * nominal), rounded to 0.01 and
/// clamped to [0.05, 1.0]; the commanded dwell when the span is too short
/// to time (FLAG_SHORT_SPAN).
pub fn analyze_capacity(
    c: &CapCell,
    mcs: u8,
    bw: u8,
    dwell_ms: u32,
    th: &CalThresholds,
) -> CapResult {
    let mut out = CapResult::default();
    let b = usize::from(c.air_bits[1] > c.air_bits[0]);
    out.best_card = b as i32;
    if c.have_rssi[b] && f64::from(c.rssi_dbm[b]) > th.sat_rssi_dbm {
        out.flags |= FLAG_SATURATED;
    }
    if i32::from(c.blocked_pct) < th.tx_bound_blocked_pct {
        out.flags |= FLAG_TX_BOUND;
    }
    let frames = f64::from(c.frames[b]);
    if frames < th.lossy_ratio * f64::from(c.offered) {
        out.flags |= FLAG_LOSSY;
    }
    if frames < th.lost_ratio * f64::from(c.offered) {
        out.flags |= FLAG_LOST | FLAG_LOSSY;
    }
    let nominal_bps = phy_rate_mbps(PhyMode::Ht, mcs, bw) * 1e6;
    let span_ms = c.last_ms[b].saturating_sub(c.first_ms[b]);
    out.span_ms = span_ms.min(u64::from(u32::MAX)) as u32;
    let mut eff = 0.0;
    if nominal_bps > 0.0 {
        if c.frames[b] >= th.span_min_frames && span_ms >= u64::from(th.span_min_ms) {
            let bits = c.air_bits[b].wrapping_sub(c.first_bits[b]) as f64;
            eff = bits / (span_ms as f64 / 1000.0 * nominal_bps);
        } else {
            out.flags |= FLAG_SHORT_SPAN;
            if dwell_ms > 0 {
                eff = c.air_bits[b] as f64 / (f64::from(dwell_ms) / 1000.0 * nominal_bps);
            }
        }
    }
    out.eff = ((eff * 100.0).round() / 100.0).clamp(0.05, 1.0);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cell(idx: i32, pct: i32, rssi: i32) -> CalCell {
        CalCell {
            idx,
            expected: 100,
            received: [(100 * pct / 100) as u16, 0],
            rssi_dbm: [rssi, 0],
            have_rssi: [rssi != RSSI_NONE, false],
            ..CalCell::default()
        }
    }
    fn c(idx: i32, pct: i32) -> CalCell {
        cell(idx, pct, -70)
    }
    fn th() -> CalThresholds {
        CalThresholds::default()
    }

    #[test]
    fn mcs7_first_dip_not_last_good() {
        let mut v: Vec<CalCell> = (0..=33).map(|i| c(i, 10)).collect();
        v.extend((34..=49).map(|i| c(i, 98)));
        v.extend((50..=55).map(|i| c(i, 30)));
        v.push(c(56, 2));
        v.push(c(57, 95)); // the comb's island: not headroom
        v.extend((58..=63).map(|i| c(i, 0)));
        let w = analyze_rate(&v, &th());
        assert_eq!((w.wall, w.floor_idx), (49, 34));
        assert_eq!(w.flags & (FLAG_NO_DIP | FLAG_UNDETERMINED), 0);
    }

    #[test]
    fn mcs0_no_dip_parks_at_the_diff_field_rail() {
        let v: Vec<CalCell> = (-41..=63)
            .step_by(4)
            .map(|i| {
                let rssi = if i <= 28 {
                    -80
                } else {
                    -80 + (i - 28) * 3 / 10
                };
                cell(i, 100, rssi)
            })
            .collect();
        let w = analyze_rate(&v, &th());
        assert_ne!(w.flags & FLAG_NO_DIP, 0);
        assert_eq!(w.flags & FLAG_UNDETERMINED, 0);
        assert_eq!(w.wall, RAIL_REL);
    }

    #[test]
    fn row_never_reaching_threshold_is_undetermined() {
        let v: Vec<CalCell> = (0..=127).step_by(4).map(|i| c(i, 40)).collect();
        let w = analyze_rate(&v, &th());
        assert_ne!(w.flags & FLAG_UNDETERMINED, 0);
        assert_eq!(w.wall, NO_WALL);
    }

    #[test]
    fn negative_relative_indices_are_ordinary_cells() {
        let mut v: Vec<CalCell> = (-40..=-8).step_by(4).map(|i| c(i, 98)).collect();
        v.extend((-4..=60).step_by(4).map(|i| c(i, 10)));
        let w = analyze_rate(&v, &th());
        assert_eq!((w.wall, w.floor_idx), (-8, -40));
        assert_eq!(w.flags & FLAG_UNDETERMINED, 0);
    }

    #[test]
    fn uses_best_single_card_not_union() {
        let v: Vec<CalCell> = (40..=60)
            .map(|i| CalCell {
                idx: i,
                expected: 100,
                received: [if i <= 50 { 98 } else { 20 }, if i <= 50 { 0 } else { 70 }],
                rssi_dbm: [-70, -85],
                have_rssi: [true, true],
                ..CalCell::default()
            })
            .collect();
        let w = analyze_rate(&v, &th());
        assert_eq!((w.wall, w.best_card), (50, 0));
        assert_eq!(w.flags & FLAG_NO_DIP, 0);
    }

    #[test]
    fn flags_saturation_from_peak_rssi() {
        let v: Vec<CalCell> = (0..=60).map(|i| cell(i, 98, -40)).collect();
        assert_ne!(analyze_rate(&v, &th()).flags & FLAG_SATURATED, 0);
    }

    #[test]
    fn flags_narrow_window() {
        let mut v: Vec<CalCell> = (0..=43).map(|i| c(i, 10)).collect();
        v.extend((44..=46).map(|i| c(i, 98)));
        v.extend((47..=60).map(|i| c(i, 5)));
        let w = analyze_rate(&v, &th());
        assert_eq!(w.wall, 46);
        assert_ne!(w.flags & FLAG_NARROW, 0);
    }

    #[test]
    fn flags_card_disagreement() {
        let v: Vec<CalCell> = (40..=60)
            .map(|i| CalCell {
                idx: i,
                expected: 100,
                received: [if i <= 55 { 98 } else { 10 }, if i <= 45 { 98 } else { 10 }],
                rssi_dbm: [-70, -70],
                have_rssi: [true, true],
                ..CalCell::default()
            })
            .collect();
        assert_ne!(analyze_rate(&v, &th()).flags & FLAG_CARD_DISAGREE, 0);
    }

    #[test]
    fn empty_input_is_undetermined() {
        let w = analyze_rate(&[], &th());
        assert_eq!(w.wall, NO_WALL);
        assert_ne!(w.flags & FLAG_UNDETERMINED, 0);
    }

    const BITS: u64 = 1435 * 8;

    /// A row from frames received out of 20 per cell, starting at idx -41.
    fn row20(got: &[i32]) -> Vec<CalCell> {
        got.iter()
            .enumerate()
            .map(|(i, g)| c(-41 + i as i32, g * 5))
            .collect()
    }

    /// 40 MHz mcs5 of a field run, a room between the pair: every cell 19 or
    /// 20 of 20 up to idx 31 except idx -12, which got 15; delivery ends at 32.
    #[rustfmt::skip]
    const FIELD_MCS5: [i32; 105] = [
        20, 20, 20, 20, 20, 20, 19, 20, 19, 20, 20, 20, 20, 20, 20, 19, 20, 20, 20, 18, 20, // -41..-21
        20, 20, 19, 20, 20, 20, 19, 20, 15, 20, 20, 19, 20, 20, 20, 19, 20, 20, 20, 20,     // -20..-1
        20, 20, 20, 20, 20, 20, 20, 19, 20, 20, 20, 20, 20, 20, 20, 20, 20, 19, 20, 20, 19, // 0..20
        20, 20, 20, 20, 20, 20, 20, 19, 19, 20, 19, 10, 7, 8, 3, 1, 0, 0, 0, 0,             // 21..40
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                         // 41..60
        0, 0, 0,                                                                            // 61..63
    ];

    #[test]
    fn a_single_failing_cell_with_clean_cells_above_does_not_end_the_run() {
        let cells = row20(&FIELD_MCS5);
        let w = analyze_rate(&cells, &th());
        assert_eq!((w.wall, w.floor_idx), (31, -41));
        assert_ne!(w.flags & FLAG_BRIDGED, 0);
        assert_eq!(w.flags & FLAG_NO_DIP, 0);
        // With the exception off the old wall comes back.
        let strict = CalThresholds {
            bridge_clean: 0,
            ..th()
        };
        let old = analyze_rate(&cells, &strict);
        assert_eq!(old.wall, -13);
        assert_eq!(old.flags & FLAG_BRIDGED, 0);
    }

    #[test]
    fn a_dip_at_the_edge_still_ends_the_run() {
        // 20 MHz mcs6 of the same run: 19 16 20 13 10 from idx 25 on.
        let mut got = vec![20; 66];
        got.extend([19, 16, 20, 13, 10, 12, 10]);
        got.resize(105, 0);
        let w = analyze_rate(&row20(&got), &th());
        assert_eq!(w.wall, 25);
        assert_eq!(w.flags & FLAG_BRIDGED, 0);
    }

    #[test]
    fn a_dip_followed_by_cells_just_over_the_threshold_ends_the_run() {
        // 20 MHz mcs4 of a bench sweep: 16 at idx 18, then 18 20 19, then 14
        // 13 5. The three cells above the dip clear 90 % but not all 95 %.
        let mut got = vec![20; 59];
        got.extend([16, 18, 20, 19, 14, 13, 5, 2, 0]);
        got.resize(105, 0);
        let w = analyze_rate(&row20(&got), &th());
        assert_eq!(w.wall, 17);
        assert_eq!(w.flags & FLAG_BRIDGED, 0);
        // With the clean bar at the wall threshold it was passed.
        let loose = CalThresholds {
            bridge_pct: 90.0,
            ..th()
        };
        let w = analyze_rate(&row20(&got), &loose);
        assert_eq!(w.wall, 21);
        assert_ne!(w.flags & FLAG_BRIDGED, 0);
    }

    #[test]
    fn two_failing_cells_in_a_row_end_the_run() {
        let mut got = [20; 105];
        got[30] = 15;
        got[31] = 15;
        let w = analyze_rate(&row20(&got), &th());
        assert_eq!(w.wall, -12);
        assert_eq!(w.flags & (FLAG_BRIDGED | FLAG_NO_DIP), 0);
    }

    #[test]
    fn a_failing_cell_near_the_top_of_the_sweep_ends_the_run() {
        let mut got = [20; 105];
        got[102] = 15; // idx 61, two cells above it
        let w = analyze_rate(&row20(&got), &th());
        assert_eq!(w.wall, 60);
        assert_eq!(w.flags & (FLAG_BRIDGED | FLAG_NO_DIP), 0);
        got[102] = 20;
        got[101] = 15; // idx 60, three clean cells above it
        let w = analyze_rate(&row20(&got), &th());
        assert_eq!(w.wall, RAIL_REL);
        assert_eq!(
            w.flags & (FLAG_BRIDGED | FLAG_NO_DIP),
            FLAG_BRIDGED | FLAG_NO_DIP
        );
    }

    #[test]
    fn a_wall_under_a_faster_rates_wall_is_marked() {
        let wall = |wall| RateWall {
            wall,
            ..RateWall::default()
        };
        let mut w = [63, 63, 63, 63, 44, -13, 30, 25].map(wall);
        w[7].flags = FLAG_SATURATED;
        mark_under_faster(&mut w);
        for (r, x) in w.iter().enumerate() {
            assert_eq!(x.flags & FLAG_UNDER_FASTER != 0, r == 5, "rate {r}");
        }
        assert_ne!(w[7].flags & FLAG_SATURATED, 0);
        // An undetermined rate is neither marked nor a reference; equal walls pass.
        let mut u = [20, NO_WALL, 20, NO_WALL].map(wall);
        mark_under_faster(&mut u);
        assert!(u.iter().all(|x| x.flags & FLAG_UNDER_FASTER == 0));
        u[1].wall = 10;
        mark_under_faster(&mut u);
        assert_ne!(u[1].flags & FLAG_UNDER_FASTER, 0);
        assert_eq!(u[0].flags & FLAG_UNDER_FASTER, 0);
    }

    #[test]
    fn flag_bits_match_the_cal_log() {
        assert_eq!((FLAG_BRIDGED, FLAG_UNDER_FASTER), (1024, 2048));
    }

    #[test]
    fn efficiency_is_delivered_over_the_receive_span() {
        let c = CapCell {
            offered: 5705,
            blocked_pct: 80,
            frames: [5705, 0],
            air_bits: [5705 * BITS, 0],
            first_bits: [BITS, 0],
            first_ms: [1000, 0],
            last_ms: [3300, 0],
            rssi_dbm: [-70, 0],
            have_rssi: [true, false],
        };
        let r = analyze_capacity(&c, 4, 20, 2000, &th());
        assert_eq!((r.eff, r.span_ms, r.best_card), (0.73, 2300, 0));
        assert_eq!(r.flags & FLAG_SHORT_SPAN, 0);
    }

    #[test]
    fn the_span_is_the_best_cards() {
        let c = CapCell {
            offered: 5705,
            blocked_pct: 80,
            frames: [3000, 5705],
            air_bits: [3000 * BITS, 5705 * BITS],
            first_bits: [BITS, BITS],
            first_ms: [500, 1000],
            last_ms: [4000, 3300],
            ..CapCell::default()
        };
        let r = analyze_capacity(&c, 4, 20, 2000, &th());
        assert_eq!((r.best_card, r.span_ms, r.eff), (1, 2300, 0.73));
    }

    #[test]
    fn a_short_span_falls_back_to_the_commanded_dwell_and_says_so() {
        let mut c = CapCell {
            offered: 5000,
            blocked_pct: 80,
            frames: [5000, 0],
            air_bits: [5000 * BITS, 0],
            first_bits: [BITS, 0],
            first_ms: [1000, 0],
            last_ms: [1499, 0],
            ..CapCell::default()
        };
        let r = analyze_capacity(&c, 4, 20, 2000, &th());
        assert_ne!(r.flags & FLAG_SHORT_SPAN, 0);
        assert_eq!((r.eff, r.span_ms), (0.74, 499));
        c.last_ms = [1500, 0];
        let r = analyze_capacity(&c, 4, 20, 2000, &th());
        assert_eq!(r.flags & FLAG_SHORT_SPAN, 0);
        assert_eq!(r.eff, 1.0);
        c.frames = [19, 0];
        c.air_bits = [19 * BITS, 0];
        let r = analyze_capacity(&c, 0, 20, 2000, &th());
        assert_ne!(r.flags & FLAG_SHORT_SPAN, 0);
        assert_eq!(r.eff, 0.05);
        c.frames = [20, 0];
        c.air_bits = [20 * BITS, 0];
        let r = analyze_capacity(&c, 0, 20, 2000, &th());
        assert_eq!(r.flags & FLAG_SHORT_SPAN, 0);
        assert_eq!((r.span_ms, r.eff), (500, 0.07));
    }

    #[test]
    fn best_card_is_the_one_with_more_bits() {
        let c = CapCell {
            offered: 1000,
            blocked_pct: 80,
            frames: [400, 900],
            air_bits: [10_000_000, 56_800_000],
            rssi_dbm: [-70, -70],
            have_rssi: [true, true],
            ..CapCell::default()
        };
        assert_eq!(analyze_capacity(&c, 4, 20, 2000, &th()).best_card, 1);
    }

    fn one(frames: u32, blocked: u8) -> CapResult {
        let c = CapCell {
            offered: 1000,
            blocked_pct: blocked,
            frames: [frames, 0],
            air_bits: [u64::from(frames) * BITS, 0],
            rssi_dbm: [-70, 0],
            have_rssi: [true, false],
            ..CapCell::default()
        };
        analyze_capacity(&c, 4, 20, 2000, &th())
    }

    #[test]
    fn tx_bound_lossy_lost_and_zero() {
        assert_ne!(one(980, 30).flags & FLAG_TX_BOUND, 0);
        assert_ne!(one(800, 80).flags & FLAG_LOSSY, 0);
        let z = one(0, 80);
        assert_eq!(z.eff, 0.05);
        assert_eq!(z.flags & (FLAG_LOSSY | FLAG_LOST), FLAG_LOSSY | FLAG_LOST);
        assert_eq!(
            one(499, 80).flags & (FLAG_LOSSY | FLAG_LOST),
            FLAG_LOSSY | FLAG_LOST
        );
        assert_eq!(one(500, 80).flags & (FLAG_LOSSY | FLAG_LOST), FLAG_LOSSY);
        assert_eq!(one(950, 80).flags & (FLAG_LOSSY | FLAG_LOST), 0);
    }

    #[test]
    fn drift_flag_bit_is_not_reused() {
        assert_eq!(
            (FLAG_TX_BOUND, FLAG_LOSSY, FLAG_LOST, FLAG_SHORT_SPAN),
            (64, 128, 256, 512)
        );
    }
}
