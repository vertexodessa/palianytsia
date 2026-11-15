//! Parameter sets the drone only takes in an order. mabur's loader caps an
//! enabled recorder's rate at the camera's (`record.fps <= venc.fps`) and
//! leaves the order to the GS (its test `ordering_rules_are_enforced_by_the_loader`:
//! REC_FPS first, then the slower VENC_MODE). The drone config Palianytsia
//! ships records at 90, so from the menu every camera mode under 90 fps
//! (2944x1656@60, 4K@33) was refused with "record.fps: must be in
//! [1, venc.fps]" (the wing, 2026-11-14). A VENC_MODE set under the current
//! REC_FPS therefore sends REC_FPS = the mode's rate first and holds the
//! VENC_MODE set until the drone accepted it.
//!
//! The loader also checks `venc.slices` against the new frame height: a
//! count that does not cut that height into whole 64-px CTU rows refuses
//! the whole mode (26 slices are legal at 1656 px, not at 816). A
//! VENC_MODE set whose height does not take the current VENC_SLICES sends
//! the largest legal count at or under it first, the same way.

use pal_osd::params::Table;

/// A held set gives up after this long without REC_FPS settling.
pub const HOLD_US: u64 = 30_000_000;

#[derive(Debug, PartialEq)]
pub struct Held {
    pub row: usize,
    pub text: String,
    pub rec_row: usize,
    pub rec_text: String,
    pub since_us: u64,
}

/// For a set of `row` to `text` against the table `t`: the REC_FPS row and
/// text to send before it, or None when the set can go out as it is (not
/// VENC_MODE, a rate at or above REC_FPS, the recorder off, or REC_FPS not
/// read yet: the drone then answers as before).
pub fn rec_fps_first(t: &Table, row: usize, text: &str) -> Option<(usize, String)> {
    if t.rows.get(row)?.id != "VENC_MODE" {
        return None;
    }
    let (fps, ..) = pal_core::mav::parse_mode_text(text)?;
    let find = |id: &str| t.rows.iter().position(|r| r.id == id);
    let rec_row = find("REC_FPS")?;
    let rec_fps: i32 = t.rows[rec_row].value.as_deref()?.parse().ok()?;
    let rec_on = find("REC_ENABLE")
        .and_then(|i| t.rows[i].value.as_deref())
        .is_none_or(|v| v != "0");
    (rec_on && rec_fps > fps).then(|| (rec_row, fps.to_string()))
}

/// For a VENC_MODE set: the VENC_SLICES row and the count to send before
/// it when the current count is not legal at the new height, else None.
pub fn slices_first(t: &Table, row: usize, text: &str) -> Option<(usize, String)> {
    if t.rows.get(row)?.id != "VENC_MODE" {
        return None;
    }
    let (_, _, h, _) = pal_core::mav::parse_mode_text(text)?;
    let s_row = t.rows.iter().position(|r| r.id == "VENC_SLICES")?;
    let cur: u32 = t.rows[s_row].value.as_deref()?.parse().ok()?;
    let ok = pal_osd::params::slice_counts(u32::try_from(h).ok()?);
    if ok.contains(&cur) {
        return None;
    }
    let pick = ok.into_iter().filter(|&s| s <= cur).max().unwrap_or(1);
    Some((s_row, pick.to_string()))
}

/// The set a VENC_MODE set needs first, if any: REC_FPS, then VENC_SLICES.
pub fn first_needed(t: &Table, row: usize, text: &str) -> Option<(usize, String)> {
    rec_fps_first(t, row, text).or_else(|| slices_first(t, row, text))
}

#[derive(Debug, PartialEq)]
pub enum Step {
    /// REC_FPS still on its way.
    Wait,
    /// REC_FPS accepted at the held rate: send the held set now.
    Send,
    /// REC_FPS refused, timed out, overwritten or the link went: drop the
    /// held set, with the reason.
    Drop(String),
}

/// What to do with `h` given the table at `now`.
pub fn step(h: &Held, t: &Table, now: u64) -> Step {
    if !t.linked {
        return Step::Drop("link lost".into());
    }
    let Some(r) = t.rows.get(h.rec_row) else {
        return Step::Drop("no prerequisite row".into());
    };
    let id = &r.id;
    if t.pending.is_none() {
        match r.state.as_str() {
            "accepted" if r.value.as_deref() == Some(h.rec_text.as_str()) => return Step::Send,
            "accepted" => return Step::Drop(format!("{id} reads {:?}", r.value)),
            "failed" | "unsupported" | "timeout" => {
                return Step::Drop(format!("{id} {}: {}", r.state, r.status));
            }
            _ => {}
        }
    }
    if now.saturating_sub(h.since_us) >= HOLD_US {
        return Step::Drop(format!("{id} did not settle"));
    }
    Step::Wait
}

#[cfg(test)]
mod tests {
    use super::*;
    use pal_osd::params::Row;

    fn row(id: &str, value: Option<&str>, state: &str) -> Row {
        Row {
            id: id.into(),
            value: value.map(str::to_string),
            state: state.into(),
            ..Row::default()
        }
    }

    fn table(rec_on: &str, rec_fps: &str, rec_state: &str, pending: Option<usize>) -> Table {
        Table {
            linked: true,
            pending,
            rows: vec![
                row("VENC_MODE", Some("90@1920x1080/binned"), "idle"),
                row("REC_ENABLE", Some(rec_on), "idle"),
                row("REC_FPS", Some(rec_fps), rec_state),
            ],
            ..Table::default()
        }
    }

    fn slices_table(slices: &str) -> Table {
        Table {
            linked: true,
            rows: vec![
                row("VENC_MODE", Some("60@2944x1656/native"), "idle"),
                row("VENC_SLICES", Some(slices), "idle"),
                row("REC_ENABLE", Some("0"), "idle"),
                row("REC_FPS", Some("60"), "idle"),
            ],
            ..Table::default()
        }
    }

    #[test]
    fn a_mode_whose_height_refuses_the_slice_count_lowers_it_first() {
        // The Quest, 2026-11-16: 26 slices at 1656 px, then 120@1728x816
        // (13 CTU rows) was refused whole.
        let t = slices_table("26");
        assert_eq!(
            first_needed(&t, 0, "120@1728x816/binned"),
            Some((1, "13".into()))
        );
        // 9 is legal at 1656 and 1080, not at 816 (1-5, 7, 13): down to 7.
        assert_eq!(
            slices_first(&slices_table("9"), 0, "120@1728x816/binned"),
            Some((1, "7".into()))
        );
        // A count the new height takes needs nothing.
        assert_eq!(
            first_needed(&slices_table("4"), 0, "120@1728x816/binned"),
            None
        );
        assert_eq!(slices_first(&slices_table("26"), 1, "4"), None);
    }

    #[test]
    fn a_mode_under_the_recording_rate_lowers_rec_fps_first() {
        let t = table("1", "90", "idle", None);
        assert_eq!(
            rec_fps_first(&t, 0, "60@2944x1656/native"),
            Some((2, "60".into()))
        );
        assert_eq!(
            rec_fps_first(&t, 0, "33@3840x2160/native"),
            Some((2, "33".into()))
        );
    }

    #[test]
    fn a_mode_at_or_over_the_recording_rate_goes_out_as_it_is() {
        let t = table("1", "90", "idle", None);
        assert_eq!(rec_fps_first(&t, 0, "90@2272x1278/native"), None);
        assert_eq!(rec_fps_first(&t, 0, "120@1728x816/binned"), None);
    }

    #[test]
    fn the_recorder_off_another_row_or_an_unread_rec_fps_need_nothing() {
        assert_eq!(
            rec_fps_first(&table("0", "90", "idle", None), 0, "60@2944x1656/native"),
            None
        );
        assert_eq!(
            rec_fps_first(&table("1", "90", "idle", None), 2, "30"),
            None
        );
        let mut t = table("1", "90", "idle", None);
        t.rows[2].value = None;
        assert_eq!(rec_fps_first(&t, 0, "60@2944x1656/native"), None);
        assert_eq!(
            rec_fps_first(&table("1", "90", "idle", None), 0, "junk"),
            None
        );
    }

    fn held() -> Held {
        Held {
            row: 0,
            text: "60@2944x1656/native".into(),
            rec_row: 2,
            rec_text: "60".into(),
            since_us: 1_000_000,
        }
    }

    #[test]
    fn the_held_mode_goes_once_rec_fps_is_accepted_at_its_rate() {
        let h = held();
        assert_eq!(
            step(&h, &table("1", "90", "sending", Some(2)), 2_000_000),
            Step::Wait
        );
        assert_eq!(
            step(&h, &table("1", "90", "in_progress", Some(2)), 2_000_000),
            Step::Wait
        );
        assert_eq!(
            step(&h, &table("1", "60", "accepted", None), 3_000_000),
            Step::Send
        );
    }

    #[test]
    fn a_refused_or_stuck_rec_fps_drops_the_held_mode() {
        let h = held();
        assert!(matches!(
            step(&h, &table("1", "90", "failed", None), 2_000_000),
            Step::Drop(_)
        ));
        assert!(matches!(
            step(&h, &table("1", "90", "timeout", None), 2_000_000),
            Step::Drop(_)
        ));
        assert!(matches!(
            step(&h, &table("1", "30", "accepted", None), 2_000_000),
            Step::Drop(_)
        ));
        assert!(matches!(
            step(
                &h,
                &table("1", "90", "in_progress", Some(2)),
                1_000_000 + HOLD_US
            ),
            Step::Drop(_)
        ));
        let mut t = table("1", "60", "accepted", None);
        t.linked = false;
        assert!(matches!(step(&h, &t, 2_000_000), Step::Drop(_)));
    }
}
