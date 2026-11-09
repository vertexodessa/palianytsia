//! The stats line the UI gets: the core's webgs line (the one the flight
//! log keeps and the parity gates compare) with the Rust-only views added.
//! Pure functions, so what the UI reads is pinned by tests and not by the
//! loop that sends it.

use serde_json::Value;

/// The core's stats line, parsed once: None when it is not a JSON object
/// (the UI then gets the text as it is).
pub fn parse(line: &str) -> Option<Value> {
    serde_json::from_str::<Value>(line)
        .ok()
        .filter(Value::is_object)
}

/// The core's capture-clock offset, None while it has none (null).
pub fn pts_off_us(stats: &Value) -> Option<i64> {
    stats.get("pts_off_us").and_then(Value::as_i64)
}

/// The UI line: `stats` with each present view (`"ladder"`, `"chscan"`)
/// and `"cal"` added.
pub fn ui_line<const N: usize>(
    mut stats: Value,
    views: [(&str, Option<Value>); N],
    cal: Option<Value>,
) -> String {
    for (k, view) in views {
        if let Some(v) = view {
            stats[k] = v;
        }
    }
    if let Some(c) = cal {
        stats["cal"] = c;
    }
    stats.to_string()
}

/// The calibration view with the last refusal added.
pub fn with_refusal(mut cal: Value, refusal: Option<&str>) -> Value {
    if let (Some(r), Some(o)) = (refusal, cal.as_object_mut()) {
        o.insert("refusal".into(), Value::from(r));
    }
    cal
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{json, Value};

    fn v(s: &str) -> Value {
        serde_json::from_str(s).expect("valid JSON")
    }

    const LINE: &str = r#"{"t":1.5,"pts_off_us":-123456,"link":{"rssi":-50},"key_fp":"default"}"#;

    fn off(line: &str) -> Option<i64> {
        pts_off_us(&parse(line).unwrap())
    }

    #[test]
    fn pts_off_reads_the_number_or_nothing() {
        assert_eq!(off(LINE), Some(-123456));
        assert_eq!(off(r#"{"a":1,"pts_off_us":null}"#), None);
        assert_eq!(off(r#"{"a":1}"#), None);
        assert_eq!(off(r#"{"pts_off_us":42}"#), Some(42));
        assert!(parse("not json").is_none() && parse("[1]").is_none());
    }

    #[test]
    fn ui_line_adds_the_views_it_has() {
        let ladder = r#"{"rung":2,"rungs":[1,2]}"#;
        let cal = r#"{"state":"idle"}"#;
        let mut want = v(LINE);
        let l = |t: Option<&str>| [("ladder", t.map(v)), ("chscan", None)];
        assert_eq!(v(&ui_line(v(LINE), l(None), None)), want);
        want["ladder"] = v(ladder);
        assert_eq!(v(&ui_line(v(LINE), l(Some(ladder)), None)), want);
        want["cal"] = v(cal);
        assert_eq!(v(&ui_line(v(LINE), l(Some(ladder)), Some(v(cal)))), want);
        let mut only_cal = v(LINE);
        only_cal["cal"] = v(cal);
        assert_eq!(v(&ui_line(v(LINE), l(None), Some(v(cal)))), only_cal);
        let scan = r#"{"state":"done","rows":[]}"#;
        let mut with_scan = v(LINE);
        with_scan["chscan"] = v(scan);
        assert_eq!(
            v(&ui_line(v(LINE), [("chscan", Some(v(scan)))], None)),
            with_scan
        );
    }

    #[test]
    fn refusal_is_a_json_string() {
        let cal = r#"{"state":"failed","widths":[]}"#;
        assert_eq!(with_refusal(v(cal), None), v(cal));
        let mut want = v(cal);
        want["refusal"] = json!("refused: link is \"down\"");
        assert_eq!(
            with_refusal(v(cal), Some("refused: link is \"down\"")),
            want
        );
    }
}
