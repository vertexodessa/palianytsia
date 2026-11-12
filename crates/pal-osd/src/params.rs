//! Drone settings (F18): the `GsCore::params_json` table as an egui window.
//! The window only reads the snapshot it is handed and returns commands;
//! the app forwards them to the core thread.
use serde_json::Value;

use crate::json::{bool_or, opt_i64, str_or};

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Row {
    pub id: String,
    pub section: String,
    pub key: String,
    pub ty: String,
    pub apply: String,
    pub help: String,
    pub value: Option<String>,
    pub state: String,
    pub status: String,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Table {
    pub linked: bool,
    /// None until the drone's first HEARTBEAT on this link.
    pub alive: Option<bool>,
    pub list: String,
    pub mismatch: bool,
    pub pending: Option<usize>,
    pub spotter: bool,
    pub rows: Vec<Row>,
}

impl Table {
    pub fn parse(line: &str) -> Option<Self> {
        let v: Value = serde_json::from_str(line).ok()?;
        let rows = v
            .get("rows")?
            .as_array()?
            .iter()
            .map(|r| Row {
                id: str_or(r, "id", ""),
                section: str_or(r, "section", ""),
                key: str_or(r, "key", ""),
                ty: str_or(r, "type", ""),
                apply: str_or(r, "apply", ""),
                help: str_or(r, "help", ""),
                value: r.get("value").and_then(Value::as_str).map(str::to_string),
                state: str_or(r, "state", ""),
                status: str_or(r, "status", ""),
            })
            .collect();
        Some(Self {
            linked: bool_or(&v, "linked", false),
            alive: v.get("alive").and_then(Value::as_bool),
            list: str_or(&v, "list", ""),
            mismatch: bool_or(&v, "mismatch", false),
            pending: opt_i64(&v, "pending").and_then(|p| usize::try_from(p).ok()),
            spotter: str_or(&v, "mode", "") == "spotter",
            rows,
        })
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Cmd {
    Set(usize, String),
    Refresh,
    /// TX-power calibration with the default sweeps (pal_core::cal).
    CalStart,
    /// A run over these sweeps only (the bench: `pal-desktop --cal-start`).
    CalSweeps(Vec<pal_wire::cal::SweepSpec>),
    CalAbort,
    /// The channels the link may sit on, applied on the live link
    /// (`GsCore::set_channels`).
    Channels(Vec<u8>),
    /// Scan these channels (empty = the drone's set) for the channel menu.
    ChannelScan(Vec<u8>),
}

/// Per-row edit buffers (empty: the current value shows as a hint) and the
/// last refusal, kept by the app across frames.
#[derive(Default)]
pub struct Edits {
    pub text: Vec<String>,
    /// The core's answer to the last Set the app forwarded.
    pub last: Option<(usize, Result<(), String>)>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_gs_and_spotter_tables() {
        let t = Table::parse(r#"{"alive":true,"linked":true,"list":"complete","mismatch":false,"pending":-1,"rows":[{"age_ms":null,"apply":"rebuild","help":"h","id":"VENC_MODE","key":"fps","section":"venc","state":"accepted","status":"","type":"str","value":"90@1920x1080/binned"}]}"#)
            .expect("gs");
        assert!(t.linked && t.alive == Some(true) && t.pending.is_none() && !t.spotter);
        assert_eq!(t.rows[0].value.as_deref(), Some("90@1920x1080/binned"));
        let s = Table::parse(r#"{"linked":false,"mode":"spotter","rows":[{"age_ms":null,"apply":"knob","help":"","id":"X","key":"k","section":"s","state":"idle","status":"","type":"int","value":null}]}"#)
            .expect("spotter");
        assert!(s.spotter && s.alive.is_none() && s.rows[0].value.is_none());
        let p = Table::parse(r#"{"linked":true,"pending":3,"rows":[]}"#).expect("pending");
        assert_eq!(p.pending, Some(3));
    }
}
