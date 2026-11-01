//! The Drone tab's TX-power calibration block: a Calibrate button behind a
//! confirm step, the run's progress, the result table and Abort. The core
//! is mabur's CalSession (`pal_core::cal`); this only reads its JSON
//! (`cal` in the UI stats line) and sends `Cmd::CalStart` / `Cmd::CalAbort`.
use egui::{Color32, RichText, Ui};
use serde_json::Value;

use crate::params::Cmd;

const OK: Color32 = Color32::from_rgb(110, 220, 140);
const WARN: Color32 = Color32::from_rgb(240, 190, 60);
const BAD: Color32 = Color32::from_rgb(240, 80, 70);

#[derive(Clone, Debug, Default, PartialEq)]
pub struct CalRate {
    pub mcs: u8,
    pub wall: Option<i64>,
    pub park: Option<i64>,
    pub verify_pct: Option<i64>,
    pub eff: Option<f64>,
    pub wall_flags: u64,
    pub cap_flags: u64,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct CalWidth {
    pub bw: u8,
    pub applied: bool,
    pub apply_reason: String,
    pub eff_unknown: bool,
    pub rates: Vec<CalRate>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct CalView {
    pub state: String,
    pub running: bool,
    pub sweep_k: u64,
    pub sweep_n: u64,
    pub kind: String,
    pub bw: u64,
    pub cell_i: u64,
    pub cell_n: u64,
    pub channel: u64,
    /// The channel pick's one-line account ("measuring", "relocating 40 ->
    /// 112", "auto: moved to 112", "pinned", ...).
    pub pick: String,
    pub fail_reason: String,
    pub sweeps: Vec<String>,
    pub widths: Vec<CalWidth>,
    /// The last start's refusal ("refused: link is down", ...).
    pub refusal: Option<String>,
    /// The linked drone advertises CAP_CALIBRATE.
    pub capable: bool,
}

impl CalView {
    pub fn parse(v: &Value) -> Option<Self> {
        let s = |v: &Value, k: &str| v.get(k).and_then(Value::as_str).unwrap_or("").to_string();
        let u = |v: &Value, k: &str| v.get(k).and_then(Value::as_u64).unwrap_or(0);
        let widths = v
            .get("widths")
            .and_then(Value::as_array)
            .map(|ws| {
                ws.iter()
                    .map(|w| CalWidth {
                        bw: u(w, "bw") as u8,
                        applied: w.get("applied").and_then(Value::as_bool).unwrap_or(false),
                        apply_reason: s(w, "apply_reason"),
                        eff_unknown: w
                            .get("eff_unknown")
                            .and_then(Value::as_bool)
                            .unwrap_or(false),
                        rates: w
                            .get("rates")
                            .and_then(Value::as_array)
                            .map(|rs| {
                                rs.iter()
                                    .map(|r| CalRate {
                                        mcs: u(r, "mcs") as u8,
                                        wall: r.get("wall").and_then(Value::as_i64),
                                        park: r.get("park").and_then(Value::as_i64),
                                        verify_pct: r.get("verify_pct").and_then(Value::as_i64),
                                        eff: r.get("eff").and_then(Value::as_f64),
                                        wall_flags: u(r, "wall_flags"),
                                        cap_flags: u(r, "cap_flags"),
                                    })
                                    .collect()
                            })
                            .unwrap_or_default(),
                    })
                    .collect()
            })
            .unwrap_or_default();
        Some(Self {
            state: v.get("state")?.as_str()?.to_string(),
            running: v.get("running").and_then(Value::as_bool).unwrap_or(false),
            sweep_k: u(v, "sweep_k"),
            sweep_n: u(v, "sweep_n"),
            kind: s(v, "kind"),
            bw: u(v, "bw"),
            cell_i: u(v, "cell_i"),
            cell_n: u(v, "cell_n"),
            channel: u(v, "channel"),
            pick: s(v, "pick"),
            fail_reason: s(v, "fail_reason"),
            sweeps: v
                .get("sweeps")
                .and_then(Value::as_array)
                .map(|a| {
                    a.iter()
                        .filter_map(Value::as_str)
                        .map(str::to_string)
                        .collect()
                })
                .unwrap_or_default(),
            widths,
            refusal: v.get("refusal").and_then(Value::as_str).map(str::to_string),
            capable: v.get("capable").and_then(Value::as_bool).unwrap_or(false),
        })
    }

    /// One line for the HUD while a run owns the air (video is paused).
    pub fn hud_line(&self) -> Option<String> {
        self.running.then(|| {
            format!(
                "CALIBRATING {}/{} {} {} MHz  {}",
                self.sweep_k,
                self.sweep_n,
                self.kind,
                self.bw,
                pct(self.cell_i, self.cell_n)
            )
        })
    }
}

fn pct(i: u64, n: u64) -> String {
    (100 * i.min(n))
        .checked_div(n)
        .map_or_else(String::new, |p| format!("{p}%"))
}

/// Wall flags worth a word in the table (cal_analysis bits).
fn flag_words(wall: u64, cap: u64) -> String {
    let mut w = vec![];
    for (bit, name) in [
        (1, "no dip"),
        (2, "undetermined"),
        (4, "narrow"),
        (8, "saturated"),
        (16, "cards disagree"),
    ] {
        if wall & bit != 0 {
            w.push(name);
        }
    }
    for (bit, name) in [(64, "tx bound"), (256, "lost"), (512, "short")] {
        if cap & bit != 0 {
            w.push(name);
        }
    }
    w.join(", ")
}

/// The block; `confirm` is the panel's two-step state across frames.
pub fn ui(ui: &mut Ui, v: Option<&CalView>, confirm: &mut bool, touch: bool) -> Option<Cmd> {
    let mut cmd = None;
    let h = if touch { 40.0 } else { 0.0 };
    ui.label(RichText::new("TX power calibration").strong());
    let Some(v) = v else {
        ui.label(RichText::new("not available on this core").color(Color32::GRAY));
        return None;
    };
    if v.state == "picking" {
        // Before the run reaches the drone: video still flows.
        *confirm = false;
        ui.label(
            RichText::new(format!(
                "choosing the channel: {} (ch {})",
                v.pick, v.channel
            ))
            .color(WARN),
        );
        if ui
            .add(egui::Button::new(RichText::new("Abort").color(BAD)).min_size(egui::vec2(0.0, h)))
            .clicked()
        {
            cmd = Some(Cmd::CalAbort);
        }
    } else if v.running {
        *confirm = false;
        ui.label(
            RichText::new(format!(
                "running: sweep {}/{} {} {} MHz on ch {}, {} ({})",
                v.sweep_k,
                v.sweep_n,
                v.kind,
                v.bw,
                v.channel,
                pct(v.cell_i, v.cell_n),
                v.state
            ))
            .color(WARN),
        );
        ui.label(
            RichText::new("Video is paused until the run ends (about 40-80 s).")
                .small()
                .color(Color32::GRAY),
        );
        if ui
            .add(egui::Button::new(RichText::new("Abort").color(BAD)).min_size(egui::vec2(0.0, h)))
            .clicked()
        {
            cmd = Some(Cmd::CalAbort);
        }
    } else if *confirm {
        ui.label(
            RichText::new(
                "The drone sweeps its TX power on every MCS and writes its own walls \
                 (power_mode = offset). Video stops for about 40-80 s. Put the drone \
                 a few metres away or more first: close up the receiver saturates \
                 and the walls come out wrong.",
            )
            .color(WARN),
        );
        ui.horizontal(|ui| {
            if ui
                .add(egui::Button::new("Start calibration").min_size(egui::vec2(0.0, h)))
                .clicked()
            {
                cmd = Some(Cmd::CalStart);
                *confirm = false;
            }
            if ui
                .add(egui::Button::new("Cancel").min_size(egui::vec2(0.0, h)))
                .clicked()
            {
                *confirm = false;
            }
        });
    } else {
        let b = egui::Button::new("Calibrate").min_size(egui::vec2(0.0, h));
        if ui.add_enabled(v.capable, b).clicked() {
            *confirm = true;
        }
        if !v.capable {
            ui.label(
                RichText::new("needs a linked drone that supports calibration")
                    .small()
                    .color(Color32::GRAY),
            );
        }
    }
    if let Some(r) = &v.refusal {
        ui.label(RichText::new(r).color(BAD));
    }
    match v.state.as_str() {
        "done" => {
            ui.label(RichText::new("last run: done").color(OK));
        }
        "failed" => {
            ui.label(RichText::new(format!("last run failed: {}", v.fail_reason)).color(BAD));
        }
        "idle" if !v.fail_reason.is_empty() => {
            ui.label(RichText::new(format!("last run: {}", v.fail_reason)).color(WARN));
        }
        _ => {}
    }
    for w in &v.widths {
        results(ui, w);
    }
    cmd
}

fn results(ui: &mut Ui, w: &CalWidth) {
    ui.separator();
    let applied = if w.applied {
        RichText::new("written to the drone").color(OK)
    } else if w.apply_reason.is_empty() {
        RichText::new("not written").color(Color32::GRAY)
    } else {
        RichText::new(format!("not written: {}", w.apply_reason)).color(BAD)
    };
    ui.horizontal(|ui| {
        ui.label(RichText::new(format!("{} MHz", w.bw)).strong());
        ui.label(applied);
    });
    if w.eff_unknown {
        ui.label(
            RichText::new("capacity: the drone's summary was lost, efficiency not written")
                .color(WARN),
        );
    }
    egui::Grid::new(("cal-grid", w.bw))
        .striped(true)
        .show(ui, |ui| {
            for h in ["rate", "wall", "park", "verify", "eff", ""] {
                ui.label(RichText::new(h).small().color(Color32::GRAY));
            }
            ui.end_row();
            let dash = || "-".to_string();
            for r in &w.rates {
                ui.label(RichText::new(format!("mcs{}", r.mcs)).monospace());
                ui.label(RichText::new(r.wall.map_or_else(dash, |x| x.to_string())).monospace());
                ui.label(RichText::new(r.park.map_or_else(dash, |x| x.to_string())).monospace());
                let vc = match r.verify_pct {
                    Some(p) if p >= 90 => OK,
                    Some(p) if p >= 50 => WARN,
                    Some(_) => BAD,
                    None => Color32::GRAY,
                };
                ui.label(
                    RichText::new(r.verify_pct.map_or_else(dash, |p| format!("{p}%")))
                        .monospace()
                        .color(vc),
                );
                ui.label(RichText::new(r.eff.map_or_else(dash, |e| format!("{e:.2}"))).monospace());
                ui.label(
                    RichText::new(flag_words(r.wall_flags, r.cap_flags))
                        .small()
                        .color(WARN),
                );
                ui.end_row();
            }
        });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_the_core_json() {
        let j = r#"{"state":"verify","running":true,"sweep_k":1,"sweep_n":2,"kind":"verify",
            "bw":20,"cell_i":3,"cell_n":7,"channel":144,"pick":"onecard","stamp_mismatch":0,
            "fail_reason":"","sweeps":["power20","cap20"],"capable":true,
            "widths":[{"bw":20,"applied":true,"apply_reason":"ok","eff_unknown":false,
            "rates":[{"mcs":0,"wall":50,"wall_flags":1,"park":46,"verify_pct":100,"eff":null,
            "cap_flags":0},{"mcs":7,"wall":null,"wall_flags":2,"park":null,"verify_pct":null,
            "eff":0.73,"cap_flags":64}]}]}"#;
        let v = CalView::parse(&serde_json::from_str(j).unwrap()).unwrap();
        assert!(v.running && v.capable);
        assert_eq!((v.sweep_k, v.sweep_n, v.channel), (1, 2, 144));
        assert_eq!(v.pick, "onecard");
        assert_eq!(v.sweeps, ["power20", "cap20"]);
        let w = &v.widths[0];
        assert!(w.applied);
        assert_eq!(
            (w.rates[0].wall, w.rates[0].park, w.rates[0].verify_pct),
            (Some(50), Some(46), Some(100))
        );
        assert_eq!((w.rates[1].wall, w.rates[1].eff), (None, Some(0.73)));
        assert_eq!(
            flag_words(w.rates[1].wall_flags, w.rates[1].cap_flags),
            "undetermined, tx bound"
        );
        assert_eq!(v.hud_line().unwrap(), "CALIBRATING 1/2 verify 20 MHz  42%");
    }

    fn draw(v: Option<&CalView>, confirm: &mut bool) -> Option<Cmd> {
        let ctx = egui::Context::default();
        let mut cmd = None;
        let _ = ctx.run(egui::RawInput::default(), |ctx| {
            egui::CentralPanel::default().show(ctx, |ui| {
                cmd = ui_fn(ui, v, confirm);
            });
        });
        cmd
    }
    fn ui_fn(ui: &mut Ui, v: Option<&CalView>, confirm: &mut bool) -> Option<Cmd> {
        super::ui(ui, v, confirm, true)
    }

    #[test]
    fn every_state_draws() {
        let mut confirm = false;
        assert!(draw(None, &mut confirm).is_none());
        let mut v = CalView {
            state: "idle".into(),
            capable: true,
            ..CalView::default()
        };
        draw(Some(&v), &mut confirm);
        confirm = true;
        draw(Some(&v), &mut confirm);
        assert!(confirm, "the confirm step stays until answered");
        v.state = "picking".into();
        v.pick = "relocating 40 -> 112".into();
        draw(Some(&v), &mut confirm);
        assert!(!confirm, "a pick in progress clears the confirm");
        assert!(v.hud_line().is_none(), "video still flows while it picks");
        confirm = true;
        v.running = true;
        v.state = "sweep".into();
        draw(Some(&v), &mut confirm);
        assert!(!confirm, "a running run clears the confirm");
        v.running = false;
        v.state = "failed".into();
        v.fail_reason = "calibration ack timeout".into();
        v.refusal = Some("refused: link is down".into());
        let rate = |mcs| CalRate {
            mcs,
            wall: Some(40 - 8 * i64::from(mcs)),
            park: Some(36 - 8 * i64::from(mcs)),
            verify_pct: Some(100),
            eff: Some(0.7),
            wall_flags: 4,
            cap_flags: 0,
        };
        for bw in [20, 40] {
            v.widths.push(CalWidth {
                bw,
                applied: bw == 20,
                apply_reason: "ok".into(),
                eff_unknown: bw == 40,
                rates: (0..8).map(rate).collect(),
            });
        }
        draw(Some(&v), &mut confirm);
    }
}
