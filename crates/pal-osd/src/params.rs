//! Drone settings (F18): the `GsCore::params_json` table as an egui window.
//! The window only reads the snapshot it is handed and returns commands;
//! the app forwards them to the core thread.
use egui::{Color32, RichText};
use serde_json::Value;

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

fn s(v: &Value, k: &str) -> String {
    v.get(k).and_then(Value::as_str).unwrap_or("").to_string()
}

impl Table {
    pub fn parse(line: &str) -> Option<Self> {
        let v: Value = serde_json::from_str(line).ok()?;
        let rows = v
            .get("rows")?
            .as_array()?
            .iter()
            .map(|r| Row {
                id: s(r, "id"),
                section: s(r, "section"),
                key: s(r, "key"),
                ty: s(r, "type"),
                apply: s(r, "apply"),
                help: s(r, "help"),
                value: r.get("value").and_then(Value::as_str).map(str::to_string),
                state: s(r, "state"),
                status: s(r, "status"),
            })
            .collect();
        Some(Self {
            linked: v.get("linked").and_then(Value::as_bool).unwrap_or(false),
            alive: v.get("alive").and_then(Value::as_bool),
            list: s(&v, "list"),
            mismatch: v.get("mismatch").and_then(Value::as_bool).unwrap_or(false),
            pending: v
                .get("pending")
                .and_then(Value::as_i64)
                .and_then(|p| usize::try_from(p).ok()),
            spotter: s(&v, "mode") == "spotter",
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

const OK: Color32 = Color32::from_rgb(90, 200, 120);
const WARN: Color32 = Color32::from_rgb(230, 180, 60);
const BAD: Color32 = Color32::from_rgb(230, 80, 70);

fn state_color(st: &str) -> Color32 {
    match st {
        "accepted" => OK,
        "sending" | "in_progress" => WARN,
        "failed" | "unsupported" | "timeout" => BAD,
        _ => Color32::GRAY,
    }
}

/// The table body; returns at most one command per frame.
pub fn widget(ui: &mut egui::Ui, t: &Table, e: &mut Edits) -> Option<Cmd> {
    let mut cmd = None;
    if e.text.len() != t.rows.len() {
        e.text = vec![String::new(); t.rows.len()];
    }
    ui.horizontal(|ui| {
        let (txt, c) = if t.spotter {
            (
                "spotter: no session to read settings over".to_string(),
                WARN,
            )
        } else if !t.linked {
            ("not linked".to_string(), WARN)
        } else {
            let alive = match t.alive {
                Some(true) => "heartbeat ok",
                Some(false) => "no heartbeat",
                None => "waiting for heartbeat",
            };
            (
                format!("linked, {alive}, list {}", t.list),
                if t.alive == Some(false) { BAD } else { OK },
            )
        };
        ui.label(RichText::new(txt).color(c));
        if t.mismatch {
            ui.label(RichText::new("table mismatch: drone build differs").color(BAD));
        }
        if ui
            .add_enabled(t.linked, egui::Button::new("refresh"))
            .clicked()
        {
            cmd = Some(Cmd::Refresh);
        }
    });
    if let Some((row, Err(why))) = &e.last {
        let id = t.rows.get(*row).map_or("?", |r| r.id.as_str());
        ui.label(RichText::new(format!("{id}: refused, {why}")).color(BAD));
    }
    ui.separator();
    egui::Grid::new("params")
        .striped(true)
        .num_columns(5)
        .show(ui, |ui| {
            for h in ["parameter", "value", "new", "", "state"] {
                ui.label(RichText::new(h).small().color(Color32::GRAY));
            }
            ui.end_row();
            for (i, r) in t.rows.iter().enumerate() {
                ui.label(format!("{}.{}", r.section, r.key))
                    .on_hover_text(format!(
                        "{} ({}, applies by {})\n{}",
                        r.id, r.ty, r.apply, r.help
                    ));
                ui.label(RichText::new(r.value.as_deref().unwrap_or("-")).monospace());
                ui.add(
                    egui::TextEdit::singleline(&mut e.text[i])
                        .hint_text(r.value.as_deref().unwrap_or(""))
                        .desired_width(110.0),
                );
                let can = t.linked
                    && t.pending.is_none()
                    && r.value.is_some()
                    && r.apply != "read-only"
                    && !e.text[i].trim().is_empty();
                if ui.add_enabled(can, egui::Button::new("set")).clicked() {
                    cmd = Some(Cmd::Set(i, e.text[i].trim().to_string()));
                    e.text[i].clear();
                }
                let mut st = r.state.clone();
                if !r.status.is_empty() {
                    st += &format!(": {}", r.status);
                }
                ui.label(RichText::new(st).color(state_color(&r.state)));
                ui.end_row();
            }
        });
    cmd
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
