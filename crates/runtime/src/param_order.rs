//! Parameter sets the drone only takes in an order. mabur's loader caps an
//! enabled recorder's rate at the camera's (`record.fps <= venc.fps`) and
//! leaves the order to the GS (its test `ordering_rules_are_enforced_by_the_loader`:
//! REC_FPS first, then the slower VENC_MODE). The drone config Palianytsia
//! ships records at 90, so from the menu every camera mode under 90 fps
//! (2944x1656@60, 4K@33) was refused with "record.fps: must be in
//! [1, venc.fps]" (the wing, 2026-11-14). A VENC_MODE set under the current
//! REC_FPS therefore sends REC_FPS = the mode's rate first and holds the
//! VENC_MODE set until the drone accepted it.

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
        return Step::Drop("no REC_FPS row".into());
    };
    if t.pending.is_none() {
        match r.state.as_str() {
            "accepted" if r.value.as_deref() == Some(h.rec_text.as_str()) => return Step::Send,
            "accepted" => return Step::Drop(format!("REC_FPS reads {:?}", r.value)),
            "failed" | "unsupported" | "timeout" => {
                return Step::Drop(format!("REC_FPS {}: {}", r.state, r.status));
            }
            _ => {}
        }
    }
    if now.saturating_sub(h.since_us) >= HOLD_US {
        return Step::Drop("REC_FPS did not settle".into());
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
