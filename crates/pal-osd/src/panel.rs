//! Settings while flying: a panel that slides over the right side of the
//! video, which keeps playing. Every row shows its live value and state
//! and applies on change (drone rows through F18, `params::Cmd::Set`;
//! ground rows to `HudPrefs`); there is no save button. Desktop and phone
//! draw the same panel; `touch` swaps text fields for steppers and presets,
//! since the phone has no keyboard path into egui yet.
use egui::{Color32, RichText, Ui};

use crate::params::{Cmd, Edits, Row, Table};
use crate::prefs::HudPrefs;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Tab {
    #[default]
    Video,
    Record,
    Link,
    Osd,
    Drone,
    Logs,
}

impl Tab {
    const ALL: [Tab; 6] = [
        Tab::Video,
        Tab::Record,
        Tab::Link,
        Tab::Osd,
        Tab::Drone,
        Tab::Logs,
    ];
    fn name(self) -> &'static str {
        match self {
            Tab::Video => "Video",
            Tab::Record => "Record",
            Tab::Link => "Link",
            Tab::Osd => "OSD",
            Tab::Drone => "Drone",
            Tab::Logs => "Logs",
        }
    }
}

#[derive(Default)]
pub struct Panel {
    pub open: bool,
    pub tab: Tab,
    /// Phone: steppers and presets instead of text fields.
    pub touch: bool,
    /// Calibrate or Reset calibration pressed, waiting for the confirm.
    pub cal_confirm: crate::cal::Confirm,
}

/// What the panel needs from the app, and the facts it shows read-only.
pub struct PanelIn<'a> {
    pub table: Option<&'a Table>,
    pub edits: &'a mut Edits,
    pub prefs: &'a mut HudPrefs,
    /// e.g. "ch 40 / 40 MHz, key default".
    pub link_info: String,
    /// Where flight logs go, or why there are none.
    pub log_info: String,
    /// The calibration block from the latest stats line.
    pub cal: Option<&'a crate::cal::CalView>,
    /// The config's channel set; empty = no channel choice (desktop).
    pub channels: &'a [u8],
}

#[derive(Default)]
pub struct PanelOut {
    pub cmd: Option<Cmd>,
    pub prefs_changed: bool,
    /// `prefs.link_bw` or `prefs.link_channel` changed: restart the link
    /// so the core picks it up.
    pub relink: bool,
}

const OK: Color32 = Color32::from_rgb(110, 220, 140);
const WARN: Color32 = Color32::from_rgb(240, 190, 60);
const BAD: Color32 = Color32::from_rgb(240, 80, 70);

/// Rows per tab, by F18 id.
const VIDEO: [&str; 5] = [
    "VENC_MODE",
    "ENC_BITRATE_MAX",
    "ENC_AIRTIME",
    "VENC_SLICES",
    "LOWPWR_ENABLE",
];
const RECORD: [&str; 4] = ["REC_ENABLE", "REC_BITRATE", "REC_FPS", "REC_SIZE"];
const LINK: [&str; 2] = ["RADIO_WALL_MARGN", "RADIO_LDPC"];

/// Stepper increments for numeric rows on touch.
fn step(id: &str) -> f64 {
    match id {
        "ENC_BITRATE_MAX" | "REC_BITRATE" => 1000.0,
        "ENC_AIRTIME" => 0.05,
        "RADIO_WALL_MARGN" => 0.5,
        _ => 1.0,
    }
}

/// Choices for string rows on touch. maburd picks the sensor mode from the
/// driver's own list. Binned: the widest 2x2-binned mode that reaches the
/// fps (1920x1080 up to 90; above it 1472x816 on OpenIPC's stock IMX415
/// driver, 1728x972 / 1728x816 on waybeam's). 1280x720 fits 1472x816 after
/// a 1.5 % crop, but not waybeam's 2.12:1 120 fps mode, where the drone
/// refuses it. Native: the widest non-binned window, 1:1 pixels and a
/// zoomed view; up to 60 fps every driver has one (2560x1440 stock, 2944x1656
/// waybeam), 61-100 only waybeam's six-mode build (2304x1296). On any other
/// driver 90/100 native fails the pipeline start and maburd restores the
/// previous mode (mabur docs/mavlink.md), so offering them is safe.
/// 720p also needs venc.slices to divide its 12 CTU rows (1, 2, 3, 4, 6, 12).
fn presets(id: &str) -> &'static [&'static str] {
    match id {
        "VENC_MODE" => &[
            "60@1920x1080/binned",
            "90@1920x1080/binned",
            "60@1280x720/binned",
            "90@1280x720/binned",
            "100@1280x720/binned",
            "120@1280x720/binned",
            "60@1920x1080/native",
            "90@1920x1080/native",
            "100@1920x1080/native",
            "60@1280x720/native",
        ],
        "REC_SIZE" => &["1920x1080", "1280x720"],
        _ => &[],
    }
}

fn state_color(st: &str) -> Color32 {
    match st {
        "accepted" => OK,
        "sending" | "in_progress" => WARN,
        "failed" | "unsupported" | "timeout" => BAD,
        _ => Color32::GRAY,
    }
}

/// One drone row: name, live value, a control, and its state.
fn row_ui(ui: &mut Ui, i: usize, r: &Row, t: &Table, e: &mut Edits, touch: bool) -> Option<Cmd> {
    let mut cmd = None;
    let can = t.linked && t.pending.is_none() && r.value.is_some();
    let help = format!("{} ({}, applies by {})\n{}", r.id, r.ty, r.apply, r.help);
    ui.label(RichText::new(format!("{}.{}", r.section, r.key)).strong())
        .on_hover_text(&help);
    ui.horizontal(|ui| {
        let v = r.value.clone().unwrap_or_else(|| "-".into());
        match r.ty.as_str() {
            "bool" => {
                let mut on = v == "1";
                if ui
                    .add_enabled(can, egui::Checkbox::new(&mut on, ""))
                    .changed()
                {
                    cmd = Some(Cmd::Set(i, if on { "1" } else { "0" }.into()));
                }
            }
            "int" | "real" if touch => {
                let x: f64 = v.parse().unwrap_or(0.0);
                let s = step(&r.id);
                let fmt = |y: f64| {
                    if r.ty == "int" {
                        format!("{}", y.round() as i64)
                    } else {
                        format!("{:.2}", y)
                    }
                };
                if ui.add_enabled(can, egui::Button::new(" - ")).clicked() {
                    cmd = Some(Cmd::Set(i, fmt(x - s)));
                }
                ui.label(RichText::new(&v).monospace());
                if ui.add_enabled(can, egui::Button::new(" + ")).clicked() {
                    cmd = Some(Cmd::Set(i, fmt(x + s)));
                }
            }
            "str" if touch && !presets(&r.id).is_empty() => {
                egui::ComboBox::from_id_salt(&r.id)
                    .selected_text(&v)
                    .show_ui(ui, |ui| {
                        for p in presets(&r.id) {
                            if ui.add_enabled(can, egui::Button::new(*p)).clicked() {
                                cmd = Some(Cmd::Set(i, (*p).into()));
                            }
                        }
                    });
            }
            _ => {
                ui.label(RichText::new(&v).monospace());
                if !touch {
                    ui.add(
                        egui::TextEdit::singleline(&mut e.text[i])
                            .hint_text("new")
                            .desired_width(110.0),
                    );
                    let ok = can && !e.text[i].trim().is_empty();
                    if ui.add_enabled(ok, egui::Button::new("set")).clicked() {
                        cmd = Some(Cmd::Set(i, e.text[i].trim().to_string()));
                        e.text[i].clear();
                    }
                }
            }
        }
    });
    // Below the controls, not beside them: a refusal reason is a sentence
    // and a horizontal row clipped it to "does not fit sens".
    if !r.state.is_empty() && r.state != "idle" {
        // The drone says IN_PROGRESS once and its final answer can be lost
        // on a lossy link; the client then waits FINAL_MS (15 s) before
        // "timeout", which otherwise reads as a hang.
        let mut st = match r.state.as_str() {
            "in_progress" => "in progress: waiting for the drone's answer (up to 15 s)".to_string(),
            "timeout" => "timeout: the drone's answer did not arrive; refresh shows the live value"
                .to_string(),
            s => s.to_string(),
        };
        if !r.status.is_empty() {
            st += &format!(": {}", r.status);
        }
        ui.add(egui::Label::new(RichText::new(st).small().color(state_color(&r.state))).wrap());
    }
    ui.add_space(6.0);
    cmd
}

fn rows_ui(
    ui: &mut Ui,
    ids: &[&str],
    t: Option<&Table>,
    e: &mut Edits,
    touch: bool,
) -> Option<Cmd> {
    let Some(t) = t else {
        ui.label(RichText::new("no drone settings yet (GS mode, linked)").color(WARN));
        return None;
    };
    if e.text.len() != t.rows.len() {
        e.text = vec![String::new(); t.rows.len()];
    }
    if !t.linked {
        ui.label(RichText::new("not linked: values are the last known").color(WARN));
    } else if let Some(n) = t.pending {
        ui.label(RichText::new(format!("applying ({n} pending)")).color(WARN));
    }
    if let Some((row, Err(why))) = &e.last {
        let id = t.rows.get(*row).map_or("?", |r| r.id.as_str());
        ui.label(RichText::new(format!("{id}: refused, {why}")).color(BAD));
    }
    let mut cmd = None;
    for (i, r) in t.rows.iter().enumerate() {
        if ids.is_empty() || ids.contains(&r.id.as_str()) {
            if let Some(c) = row_ui(ui, i, r, t, e, touch) {
                cmd = Some(c);
            }
        }
    }
    cmd
}

/// The small button that opens the panel; draw every frame.
pub fn gear(ctx: &egui::Context, p: &mut Panel) {
    if p.open {
        return;
    }
    egui::Area::new(egui::Id::new("pal-gear"))
        .anchor(egui::Align2::RIGHT_CENTER, egui::vec2(-8.0, 0.0))
        .show(ctx, |ui| {
            let b = egui::Button::new(RichText::new("⚙").size(22.0))
                .fill(Color32::from_black_alpha(140));
            if ui.add(b).clicked() {
                p.open = !p.open;
            }
        });
}

/// Inner width of the open panel; the HUD keeps its right column left of
/// `width + 2 * MARGIN` so the link readout stays visible while tuning.
pub fn width(screen: egui::Rect) -> f32 {
    (screen.width() * 0.42).clamp(300.0, 420.0)
}

pub const MARGIN: f32 = 10.0;

/// The screen area the HUD may use: all of `r`, or left of the open panel.
pub fn hud_rect(ctx: &egui::Context, p: &Panel, r: egui::Rect) -> egui::Rect {
    if !p.open {
        return r;
    }
    let left = ctx.screen_rect().right() - width(ctx.screen_rect()) - 2.0 * MARGIN;
    egui::Rect::from_min_max(r.min, egui::pos2(r.right().min(left), r.bottom()))
}

/// The panel itself (no-op while closed).
pub fn show(ctx: &egui::Context, p: &mut Panel, i: PanelIn) -> PanelOut {
    let mut out = PanelOut::default();
    if !p.open {
        return out;
    }
    let screen = ctx.screen_rect();
    let w = width(screen);
    egui::Area::new(egui::Id::new("pal-panel"))
        .anchor(egui::Align2::RIGHT_TOP, egui::vec2(0.0, 0.0))
        .show(ctx, |ui| {
            egui::Frame::NONE
                .fill(Color32::from_black_alpha(215))
                .inner_margin(MARGIN)
                .show(ui, |ui| {
                    ui.set_width(w);
                    ui.set_height(screen.height() - 2.0 * MARGIN);
                    if p.touch {
                        // Finger-sized targets: ~44 pt rows (Android's
                        // 48 dp guideline, minus the row spacing).
                        let st = ui.style_mut();
                        st.spacing.interact_size = egui::vec2(44.0, 40.0);
                        st.spacing.button_padding = egui::vec2(14.0, 8.0);
                        st.spacing.item_spacing = egui::vec2(10.0, 8.0);
                        st.spacing.icon_width = 26.0;
                        for f in st.text_styles.values_mut() {
                            f.size *= 1.2;
                        }
                    }
                    ui.horizontal_wrapped(|ui| {
                        for t in Tab::ALL {
                            if ui.selectable_label(p.tab == t, t.name()).clicked() {
                                p.tab = t;
                            }
                        }
                        if ui.button("close").clicked() {
                            p.open = false;
                        }
                    });
                    ui.separator();
                    egui::ScrollArea::vertical().show(ui, |ui| {
                        let touch = p.touch;
                        match p.tab {
                            Tab::Video => out.cmd = rows_ui(ui, &VIDEO, i.table, i.edits, touch),
                            Tab::Record => {
                                ui.label("On the drone (microSD):");
                                out.cmd = rows_ui(ui, &RECORD, i.table, i.edits, touch);
                                ui.separator();
                                ui.label(
                                    RichText::new("Ground DVR: not implemented yet")
                                        .color(Color32::GRAY),
                                );
                            }
                            Tab::Link => {
                                ui.label(RichText::new(&i.link_info).monospace());
                                ui.separator();
                                ui.horizontal(|ui| {
                                    ui.label("Video width");
                                    for bw in [20u8, 40] {
                                        let on = i.prefs.link_bw == bw;
                                        let b = egui::Button::new(format!("{bw} MHz"))
                                            .selected(on)
                                            .min_size(egui::vec2(
                                                0.0,
                                                if touch { 40.0 } else { 0.0 },
                                            ));
                                        if ui.add(b).clicked() && !on {
                                            i.prefs.link_bw = bw;
                                            out.prefs_changed = true;
                                            out.relink = true;
                                        }
                                    }
                                });
                                ui.label(
                                    RichText::new(
                                        "20 MHz reaches farther at the robust rungs (4-6 dB at \
                                         MCS0); 40 MHz carries more per rung. Switching restarts \
                                         the link for a few seconds; the drone needs no change.",
                                    )
                                    .small()
                                    .color(Color32::GRAY),
                                );
                                if !i.channels.is_empty() {
                                    ui.separator();
                                    let h = if touch { 40.0 } else { 0.0 };
                                    ui.horizontal_wrapped(|ui| {
                                        ui.label("Channel");
                                        let opts =
                                            std::iter::once(0u8).chain(i.channels.iter().copied());
                                        for ch in opts {
                                            let on = i.prefs.link_channel == ch;
                                            let t = if ch == 0 {
                                                "auto".to_string()
                                            } else {
                                                ch.to_string()
                                            };
                                            let b = egui::Button::new(t)
                                                .selected(on)
                                                .min_size(egui::vec2(0.0, h));
                                            if ui.add(b).clicked() && !on {
                                                i.prefs.link_channel = ch;
                                                out.prefs_changed = true;
                                                out.relink = true;
                                            }
                                        }
                                    });
                                    ui.label(
                                        RichText::new(
                                            "auto moves the link to the quietest channel and \
                                             hops away from interference. A fixed channel never \
                                             moves: use it to calibrate, or where you know the \
                                             air. The GS finds the drone on any channel of the \
                                             set and moves it there. Switching restarts the link.",
                                        )
                                        .small()
                                        .color(Color32::GRAY),
                                    );
                                }
                                ui.separator();
                                out.cmd = rows_ui(ui, &LINK, i.table, i.edits, touch);
                            }
                            Tab::Osd => out.prefs_changed = osd_ui(ui, i.prefs),
                            Tab::Drone => {
                                if let Some(c) =
                                    crate::cal::ui(ui, i.cal, i.table, &mut p.cal_confirm, touch)
                                {
                                    out.cmd = Some(c);
                                }
                                ui.separator();
                                if let Some(t) = i.table {
                                    ui.horizontal(|ui| {
                                        let alive = match t.alive {
                                            Some(true) => "heartbeat ok",
                                            Some(false) => "no heartbeat",
                                            None => "waiting for heartbeat",
                                        };
                                        ui.label(format!("list {}, {alive}", t.list));
                                        if ui
                                            .add_enabled(t.linked, egui::Button::new("refresh"))
                                            .clicked()
                                        {
                                            out.cmd = Some(Cmd::Refresh);
                                        }
                                    });
                                    if t.mismatch {
                                        ui.label(
                                            RichText::new("table mismatch: drone build differs")
                                                .color(BAD),
                                        );
                                    }
                                }
                                if let Some(c) = rows_ui(ui, &[], i.table, i.edits, touch) {
                                    out.cmd = Some(c);
                                }
                            }
                            Tab::Logs => {
                                ui.label(&i.log_info);
                            }
                        }
                    });
                });
        });
    out
}

fn osd_ui(ui: &mut Ui, p: &mut HudPrefs) -> bool {
    let before = p.clone();
    ui.checkbox(&mut p.dot, "Centre dot");
    ui.checkbox(&mut p.battery, "Battery");
    ui.checkbox(&mut p.timer, "Arm state and flight timer");
    ui.checkbox(&mut p.nav, "Altitude, speed, home, satellites");
    ui.checkbox(&mut p.link, "Link readout (RSSI, MCS)");
    ui.checkbox(&mut p.messages, "Flight controller messages");
    ui.separator();
    ui.checkbox(&mut p.stats, "Link statistics (detailed)");
    ui.add(egui::Slider::new(&mut p.scale, 0.6..=2.0).text("text size"));
    *p != before
}
