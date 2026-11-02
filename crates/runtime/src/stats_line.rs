//! The stats line the UI gets: the core's webgs line (the one the flight
//! log keeps and the parity gates compare) with the Rust-only views added.
//! Pure functions, so what the UI reads is pinned by tests and not by the
//! loop that sends it.

/// The core's capture-clock offset from its stats line, None when the
/// core has none yet (null) or the line has no such key.
pub fn pts_off_us(line: &str) -> Option<i64> {
    line.split("\"pts_off_us\":")
        .nth(1)
        .and_then(|r| r.split([',', '}']).next())
        .and_then(|v| v.parse().ok())
}

/// `line` with `"ladder"` and `"cal"` added (each when present).
pub fn ui_line(line: String, ladder: Option<&str>, cal: Option<&str>) -> String {
    let mut ui = match ladder {
        Some(l) if line.ends_with('}') => format!("{},\"ladder\":{l}}}", &line[..line.len() - 1]),
        _ => line,
    };
    if let Some(c) = cal {
        if ui.ends_with('}') {
            ui = format!("{},\"cal\":{c}}}", &ui[..ui.len() - 1]);
        }
    }
    ui
}

/// The calibration JSON with the last refusal added.
pub fn with_refusal(cal: String, refusal: Option<&str>) -> String {
    match refusal {
        Some(r) if cal.ends_with('}') => format!(
            "{},\"refusal\":{}}}",
            &cal[..cal.len() - 1],
            serde_json::Value::from(r)
        ),
        _ => cal,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{json, Value};

    fn v(s: &str) -> Value {
        serde_json::from_str(s).expect("valid JSON")
    }

    const LINE: &str = r#"{"t":1.5,"pts_off_us":-123456,"link":{"rssi":-50},"key_fp":"default"}"#;

    #[test]
    fn pts_off_reads_the_number_or_nothing() {
        assert_eq!(pts_off_us(LINE), Some(-123456));
        assert_eq!(pts_off_us(r#"{"a":1,"pts_off_us":null}"#), None);
        assert_eq!(pts_off_us(r#"{"a":1}"#), None);
        assert_eq!(pts_off_us(r#"{"pts_off_us":42}"#), Some(42));
    }

    #[test]
    fn ui_line_adds_the_views_it_has() {
        let ladder = r#"{"rung":2,"rungs":[1,2]}"#;
        let cal = r#"{"state":"idle"}"#;
        let mut want = v(LINE);
        assert_eq!(v(&ui_line(LINE.into(), None, None)), want);
        want["ladder"] = v(ladder);
        assert_eq!(v(&ui_line(LINE.into(), Some(ladder), None)), want);
        want["cal"] = v(cal);
        assert_eq!(v(&ui_line(LINE.into(), Some(ladder), Some(cal))), want);
        let mut only_cal = v(LINE);
        only_cal["cal"] = v(cal);
        assert_eq!(v(&ui_line(LINE.into(), None, Some(cal))), only_cal);
    }

    #[test]
    fn refusal_is_a_json_string() {
        let cal = r#"{"state":"failed","widths":[]}"#;
        assert_eq!(v(&with_refusal(cal.into(), None)), v(cal));
        let mut want = v(cal);
        want["refusal"] = json!("refused: link is \"down\"");
        assert_eq!(
            v(&with_refusal(cal.into(), Some("refused: link is \"down\""))),
            want
        );
    }
}
