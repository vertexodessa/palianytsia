//! OSD widgets (spec section 6) drawn from the core's `stats_json` line, so
//! every app and both cores share one OSD. Reads only the snapshot it is
//! handed: no locks toward the RX or core path.
pub mod telem;

use std::collections::VecDeque;

use egui::{Align2, Color32, FontId, Pos2, Rect, Stroke, Ui, Vec2};
use serde_json::Value;

/// The fields the OSD shows, parsed from one stats line.
#[derive(Clone, Debug, Default)]
pub struct Snapshot {
    pub mode: String,
    pub session: bool,
    pub key_mismatch: bool,
    pub rung: i64,
    pub mcs: i64,
    pub bw: i64,
    pub drone_state: Option<i64>,
    pub rssi_dbm: Option<f64>,
    pub snr_db: Option<f64>,
    pub bodies: u64,
    pub aus: u64,
    pub trunc: u64,
    pub salvaged: u64,
    pub sends: u64,
    pub resets: u64,
}

fn u(v: &Value, k: &str) -> u64 {
    v.get(k).and_then(Value::as_u64).unwrap_or(0)
}

impl Snapshot {
    pub fn parse(line: &str) -> Option<Self> {
        let v: Value = serde_json::from_str(line).ok()?;
        Some(Self {
            mode: v
                .get("mode")
                .and_then(Value::as_str)
                .unwrap_or("?")
                .to_string(),
            session: v.get("session").and_then(Value::as_bool).unwrap_or(false),
            key_mismatch: v
                .get("key_mismatch")
                .and_then(Value::as_bool)
                .unwrap_or(false),
            rung: v.get("rung").and_then(Value::as_i64).unwrap_or(-1),
            mcs: v.get("mcs").and_then(Value::as_i64).unwrap_or(-1),
            bw: v.get("bw").and_then(Value::as_i64).unwrap_or(0),
            drone_state: v.get("drone_state").and_then(Value::as_i64),
            rssi_dbm: v.get("rssi_dbm").and_then(Value::as_f64),
            snr_db: v.get("snr_db").and_then(Value::as_f64),
            bodies: u(&v, "bodies"),
            aus: u(&v, "aus"),
            trunc: u(&v, "trunc"),
            salvaged: u(&v, "salvaged"),
            sends: u(&v, "sends"),
            resets: u(&v, "resets"),
        })
    }
}

/// Drone `Telem.state` names (mabur rc_proto).
pub fn drone_state_name(s: i64) -> &'static str {
    match s {
        0 => "boot",
        1 => "rendezvous",
        2 => "linked",
        3 => "failsafe",
        _ => "?",
    }
}

const OK: Color32 = Color32::from_rgb(90, 200, 120);
const WARN: Color32 = Color32::from_rgb(230, 180, 60);
const BAD: Color32 = Color32::from_rgb(230, 80, 70);

/// Per-second history for the 60 s monitor plots.
#[derive(Default)]
pub struct History {
    rows: VecDeque<(f64, Snapshot)>,
}

impl History {
    pub const SPAN_S: f64 = 60.0;
    pub fn push(&mut self, t_s: f64, s: Snapshot) {
        self.rows.push_back((t_s, s));
        while self
            .rows
            .front()
            .is_some_and(|(t, _)| t_s - t > Self::SPAN_S)
        {
            self.rows.pop_front();
        }
    }
    pub fn last(&self) -> Option<&Snapshot> {
        self.rows.back().map(|(_, s)| s)
    }
    /// Per-interval deltas of a counter, as (t, value per second).
    fn rate(&self, f: impl Fn(&Snapshot) -> u64) -> Vec<(f64, f64)> {
        self.rows
            .iter()
            .zip(self.rows.iter().skip(1))
            .filter_map(|((t0, a), (t1, b))| {
                let dt = t1 - t0;
                (dt > 0.0).then(|| (*t1, f(b).saturating_sub(f(a)) as f64 / dt))
            })
            .collect()
    }
    fn series(&self, f: impl Fn(&Snapshot) -> Option<f64>) -> Vec<(f64, f64)> {
        self.rows
            .iter()
            .filter_map(|(t, s)| f(s).map(|v| (*t, v)))
            .collect()
    }
}

fn chip(ui: &mut Ui, label: &str, text: &str, color: Color32) {
    egui::Frame::NONE
        .fill(Color32::from_black_alpha(160))
        .corner_radius(4.0)
        .inner_margin(Vec2::new(6.0, 3.0))
        .show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new(label).small().color(Color32::GRAY));
                ui.label(egui::RichText::new(text).monospace().color(color));
            });
        });
}

/// The top OSD row: link, signal, loss, uplink side.
pub fn chips(ui: &mut Ui, h: &History, fps: f64, decode: (u64, u64)) {
    let Some(s) = h.last() else {
        chip(ui, "core", "waiting for stats", WARN);
        return;
    };
    ui.horizontal_wrapped(|ui| {
        let (link, c) = if s.key_mismatch {
            ("KEY MISMATCH".to_string(), BAD)
        } else if s.mode == "spotter" {
            (format!("spotter MCS{} {}M", s.mcs, s.bw), OK)
        } else if s.session {
            (format!("rung {} MCS{} {}M", s.rung, s.mcs, s.bw), OK)
        } else {
            ("beaconing".to_string(), WARN)
        };
        chip(ui, "link", &link, c);
        let sig = match (s.rssi_dbm, s.snr_db) {
            (Some(r), Some(n)) => format!("{r:.0} dBm  SNR {n:.0}"),
            _ => "no signal".into(),
        };
        chip(
            ui,
            "signal",
            &sig,
            if s.snr_db.unwrap_or(0.0) > 15.0 {
                OK
            } else {
                WARN
            },
        );
        let total = s.aus + s.trunc;
        let pct = if total > 0 {
            100.0 * s.trunc as f64 / total as f64
        } else {
            0.0
        };
        chip(
            ui,
            "loss",
            &format!("trunc {pct:.1} %  salv {}", s.salvaged),
            if pct < 2.0 {
                OK
            } else if pct < 10.0 {
                WARN
            } else {
                BAD
            },
        );
        chip(
            ui,
            "video",
            &format!("{fps:.0} fps  dec err {}", decode.1),
            OK,
        );
        if let Some(d) = s.drone_state {
            let c = if d == 2 { OK } else { WARN };
            chip(ui, "drone", drone_state_name(d), c);
        }
        if s.mode == "gs" {
            chip(ui, "tx", &format!("{} sends", s.sends), OK);
        }
    });
}

fn plot(ui: &mut Ui, title: &str, pts: &[(f64, f64)], now: f64, color: Color32) {
    let (rect, _) =
        ui.allocate_exact_size(Vec2::new(ui.available_width(), 70.0), egui::Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, 3.0, Color32::from_black_alpha(150));
    let font = FontId::monospace(11.0);
    let last = pts.last().map_or("-".into(), |(_, v)| format!("{v:.1}"));
    p.text(
        rect.left_top() + Vec2::new(4.0, 2.0),
        Align2::LEFT_TOP,
        format!("{title} {last}"),
        font,
        Color32::GRAY,
    );
    if pts.len() < 2 {
        return;
    }
    let (lo, hi) = pts.iter().fold((f64::MAX, f64::MIN), |(a, b), (_, v)| {
        (a.min(*v), b.max(*v))
    });
    let (lo, hi) = if hi - lo < 1e-6 {
        (lo - 1.0, hi + 1.0)
    } else {
        (lo, hi)
    };
    let inner = Rect::from_min_max(
        rect.min + Vec2::new(2.0, 16.0),
        rect.max - Vec2::new(2.0, 2.0),
    );
    let map = |(t, v): (f64, f64)| {
        let x = inner.right() - ((now - t) / History::SPAN_S) as f32 * inner.width();
        let y = inner.bottom() - ((v - lo) / (hi - lo)) as f32 * inner.height();
        Pos2::new(x.max(inner.left()), y)
    };
    let line: Vec<Pos2> = pts.iter().copied().map(map).collect();
    p.add(egui::Shape::line(line, Stroke::new(1.5_f32, color)));
}

/// 60 s monitor plots: signal, loss, rung.
pub fn monitor(ui: &mut Ui, h: &History, now: f64) {
    plot(ui, "SNR dB", &h.series(|s| s.snr_db), now, OK);
    plot(
        ui,
        "RSSI dBm",
        &h.series(|s| s.rssi_dbm),
        now,
        Color32::LIGHT_BLUE,
    );
    plot(ui, "truncated AU/s", &h.rate(|s| s.trunc), now, BAD);
    plot(
        ui,
        "rung",
        &h.series(|s| (s.rung >= 0).then_some(s.rung as f64)),
        now,
        WARN,
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_rust_core_line() {
        let s = Snapshot::parse(r#"{"mode":"gs","session":true,"rung":2,"mcs":3,"bw":40,"drone_state":2,"snr_db":30.5,"rssi_dbm":-40.0,"aus":10,"trunc":1}"#)
            .expect("parse");
        assert!(s.session);
        assert_eq!((s.rung, s.mcs, s.aus, s.trunc), (2, 3, 10, 1));
        let mut h = History::default();
        h.push(0.0, s.clone());
        h.push(1.0, Snapshot { trunc: 4, ..s });
        assert_eq!(h.rate(|s| s.trunc), vec![(1.0, 3.0)]);
    }
}
