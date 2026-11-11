//! Settings while flying: a panel that slides over the right side of the
//! video, which keeps playing. Every row shows its live value and state
//! and applies on change (drone rows through F18, `params::Cmd::Set`;
//! ground rows to `HudPrefs`); there is no save button. The parts stand
//! apart so a front end can put them on its own surface: [`tabs`] chooses,
//! [`body`] draws the chosen tab, [`docked`] is the container desktop and
//! phone use, and a [`Look`] says how big the rows are and whether they
//! use steppers (no keyboard) or text fields.
use egui::{Color32, RichText, Ui};

use crate::params::{Cmd, Edits, Row, Table};
use crate::prefs::HudPrefs;

/// A button `h` points tall at least (0 = sized by its text, on desktop).
pub(crate) fn btn(text: impl Into<egui::WidgetText>, h: f32) -> egui::Button<'static> {
    egui::Button::new(text).min_size(egui::vec2(0.0, h))
}

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

/// Widget spacing of a [`Look`], in points.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Spacing {
    pub interact: egui::Vec2,
    pub button_padding: egui::Vec2,
    pub item: egui::Vec2,
    pub icon: f32,
}

/// How big the panel's rows are and which controls they use: one value the
/// front end picks.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Look {
    /// Least height of a button (0: sized by its text).
    pub row_h: f32,
    /// Factor on every text style.
    pub text: f32,
    /// Steppers and presets instead of text fields: no keyboard.
    pub steppers: bool,
    /// None: egui's own.
    pub spacing: Option<Spacing>,
    /// Row gap of the channel grid.
    pub grid_gap: f32,
}

impl Look {
    /// Mouse and keyboard.
    pub const DESKTOP: Look = Look {
        row_h: 0.0,
        text: 1.0,
        steppers: false,
        spacing: None,
        grid_gap: 2.0,
    };
    /// Finger-sized targets: ~44 pt rows (Android's 48 dp guideline, minus
    /// the row spacing).
    pub const TOUCH: Look = Look {
        row_h: 40.0,
        text: 1.2,
        steppers: true,
        spacing: Some(Spacing {
            interact: egui::vec2(44.0, 40.0),
            button_padding: egui::vec2(14.0, 8.0),
            item: egui::vec2(10.0, 8.0),
            icon: 26.0,
        }),
        grid_gap: 6.0,
    };

    /// Onto `ui` and everything drawn in it after this.
    pub fn apply(&self, ui: &mut Ui) {
        let st = ui.style_mut();
        if let Some(s) = self.spacing {
            st.spacing.interact_size = s.interact;
            st.spacing.button_padding = s.button_padding;
            st.spacing.item_spacing = s.item;
            st.spacing.icon_width = s.icon;
        }
        if self.text != 1.0 {
            for f in st.text_styles.values_mut() {
                f.size *= self.text;
            }
        }
    }
}

impl Default for Look {
    fn default() -> Self {
        Self::DESKTOP
    }
}

#[derive(Default)]
pub struct Panel {
    pub open: bool,
    pub tab: Tab,
    pub look: Look,
    /// Calibrate or Reset calibration pressed, waiting for the confirm.
    pub cal_confirm: crate::cal::Confirm,
    /// The Link tab shows the channel menu instead of its rows.
    pub chan_menu: bool,
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
    /// The latest channel scan, for the channel menu.
    pub scan: Option<&'a crate::channels::ScanView>,
}

#[derive(Default)]
pub struct PanelOut {
    pub cmd: Option<Cmd>,
    pub prefs_changed: bool,
    /// `prefs.link_bw` changed: restart the link so the core picks it up.
    /// (A channel choice goes to the live link as `Cmd::Channels`.)
    pub relink: bool,
    /// The close button was pressed. [`docked`] has closed the panel
    /// already; a front end with its own container acts on it.
    pub close: bool,
}

const OK: Color32 = Color32::from_rgb(110, 220, 140);
const WARN: Color32 = Color32::from_rgb(240, 190, 60);
const BAD: Color32 = Color32::from_rgb(240, 80, 70);

/// Rows per tab, by F18 id.
const VIDEO: [&str; 8] = [
    "VENC_MODE",
    "ENC_BITRATE_MAX",
    "ENC_AIRTIME",
    "VENC_SLICES",
    "VENC_MIN_IQP",
    "LOWPWR_ENABLE",
    "VENC_MIRROR",
    "VENC_FLIP",
];
const RECORD: [&str; 4] = ["REC_ENABLE", "REC_BITRATE", "REC_FPS", "REC_SIZE"];
const LINK: [&str; 2] = ["RADIO_WALL_MARGN", "RADIO_LDPC"];

/// Stepper increments for numeric rows on touch.
fn step(id: &str) -> f64 {
    match id {
        "ENC_BITRATE_MAX" | "REC_BITRATE" => 1000.0,
        "ENC_AIRTIME" => 0.05,
        "RADIO_WALL_MARGN" => 0.5,
        "VENC_MIN_IQP" => 2.0,
        _ => 1.0,
    }
}

/// Choices for string rows on touch. VENC_MODE's come from the drone's
/// VENC_MODES row, the list its camera driver offers ([`mode_menu`]); a
/// drone without that row (maburd before mabur's VENC_MODES), or one whose
/// camera has not started, gets [`FALLBACK_MODES`].
fn presets(id: &str, t: &Table) -> Vec<String> {
    match id {
        "VENC_MODE" => {
            let listed = t
                .rows
                .iter()
                .find(|r| r.id == "VENC_MODES")
                .and_then(|r| r.value.as_deref())
                .map(mode_menu)
                .unwrap_or_default();
            if listed.is_empty() {
                FALLBACK_MODES.iter().map(|s| s.to_string()).collect()
            } else {
                listed
            }
        }
        "REC_SIZE" => vec!["1920x1080".into(), "1280x720".into()],
        _ => vec![],
    }
}

/// The menu for a drone that does not report its driver's list (maburd
/// before VENC_MODES): binned 1920x1080 up to 90 and 720p; 100 fps 720p and
/// the 1728x816 120 fps crop on waybeam's driver; native windows of waybeam's
/// six-mode driver (2944x1656@60, 2272x1278@100) and the stock one
/// (2560x1440@60). A mode the loaded driver lacks costs only a refusal:
/// maburd keeps the previous mode.
const FALLBACK_MODES: [&str; 13] = [
    "60@1920x1080/binned",
    "90@1920x1080/binned",
    "60@1280x720/binned",
    "90@1280x720/binned",
    "100@1280x720/binned",
    "120@1728x816/binned",
    "60@1920x1080/native",
    "90@1920x1080/native",
    "100@1920x1080/native",
    "60@1280x720/native",
    "100@2272x1278/native",
    "60@2560x1440/native",
    "60@2944x1656/native",
];

/// One driver mode from VENC_MODES: its top rate, size and readout.
#[derive(Clone, Copy, PartialEq, Debug)]
struct DriverMode {
    fps: u32,
    w: u32,
    h: u32,
    binned: bool,
}

fn parse_mode(s: &str) -> Option<DriverMode> {
    let (fps, rest) = s.split_once('@')?;
    let (size, readout) = rest.split_once('/')?;
    let (w, h) = size.split_once('x')?;
    Some(DriverMode {
        fps: fps.parse().ok()?,
        w: w.parse().ok()?,
        h: h.parse().ok()?,
        binned: match readout {
            "binned" => true,
            "native" => false,
            _ => return None,
        },
    })
}

/// mabur's camera_mode_for on the reported list, for one readout: of the
/// modes reaching `fps`, the largest, then the lowest top rate.
fn pick(modes: &[DriverMode], binned: bool, fps: u32) -> Option<DriverMode> {
    modes
        .iter()
        .filter(|m| m.binned == binned && m.fps >= fps)
        .copied()
        .reduce(|b, m| {
            let (a, ba) = (m.w * m.h, b.w * b.h);
            if a > ba || (a == ba && m.fps < b.fps) {
                m
            } else {
                b
            }
        })
}

/// mabur's camera_mode_size_ok: the mode's own aspect, exactly, and no
/// larger than the mode (the VPE scales down, never crops or up).
fn size_fits(m: DriverMode, w: u32, h: u32) -> bool {
    w > 0 && u64::from(w) * u64::from(m.h) == u64::from(h) * u64::from(m.w) && w <= m.w && h <= m.h
}

/// The VENC_MODE menu from the drone's driver list: each mode at its own
/// size and top rate, plus the 60 and 90 fps rates that still pick that
/// mode, each at the mode's size and at the 16:9 sizes below it that fit
/// its window. Every entry is one the drone takes, since it runs the same
/// pick and fit; binned first, then by rate and size.
fn mode_menu(list: &str) -> Vec<String> {
    const RATES: [u32; 2] = [60, 90];
    const SIZES: [(u32, u32); 3] = [(2560, 1440), (1920, 1080), (1280, 720)];
    let modes: Vec<DriverMode> = list.split(',').filter_map(parse_mode).collect();
    let mut out: Vec<(bool, u32, u32, String)> = Vec::new();
    for &m in &modes {
        let rates = RATES
            .iter()
            .copied()
            .filter(|&r| r < m.fps && pick(&modes, m.binned, r) == Some(m))
            .chain(std::iter::once(m.fps));
        for fps in rates {
            let sizes = std::iter::once((m.w, m.h)).chain(
                SIZES
                    .iter()
                    .copied()
                    .filter(|&(w, h)| (w, h) != (m.w, m.h) && size_fits(m, w, h)),
            );
            for (w, h) in sizes {
                let readout = if m.binned { "binned" } else { "native" };
                let text = format!("{fps}@{w}x{h}/{readout}");
                if !out.iter().any(|o| o.3 == text) {
                    out.push((!m.binned, fps, u32::MAX - w * h, text));
                }
            }
        }
    }
    out.sort();
    out.into_iter().map(|o| o.3).collect()
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
/// Whether a row takes a set now. A read-only row never does.
fn settable(r: &Row, t: &Table) -> bool {
    t.linked && t.pending.is_none() && r.value.is_some() && r.apply != "read-only"
}

fn row_ui(ui: &mut Ui, i: usize, r: &Row, t: &Table, e: &mut Edits, look: Look) -> Option<Cmd> {
    let mut cmd = None;
    let can = settable(r, t);
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
            "int" | "real" if look.steppers => {
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
            "str" if look.steppers && !presets(&r.id, t).is_empty() => {
                egui::ComboBox::from_id_salt(&r.id)
                    .selected_text(&v)
                    .show_ui(ui, |ui| {
                        for p in presets(&r.id, t) {
                            if ui.add_enabled(can, egui::Button::new(&p)).clicked() {
                                cmd = Some(Cmd::Set(i, p));
                            }
                        }
                    });
            }
            _ => {
                ui.label(RichText::new(&v).monospace());
                if !look.steppers && r.apply != "read-only" {
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

fn rows_ui(ui: &mut Ui, ids: &[&str], t: Option<&Table>, e: &mut Edits, look: Look) -> Option<Cmd> {
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
            if let Some(c) = row_ui(ui, i, r, t, e, look) {
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
    // Foreground: the stats chips are Middle Areas the operator may drag
    // anywhere, and the gear and the panel must stay above them.
    egui::Area::new(egui::Id::new("pal-gear"))
        .order(egui::Order::Foreground)
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

/// The tab chooser, laid out by the caller (a wrapped row, a column).
/// True when its close button was pressed.
pub fn tabs(ui: &mut Ui, p: &mut Panel) -> bool {
    for t in Tab::ALL {
        if ui.selectable_label(p.tab == t, t.name()).clicked() {
            p.tab = t;
        }
    }
    ui.button("close").clicked()
}

/// The container desktop and phone use: the right side of the screen, the
/// tabs in a row above the scrolling body (no-op while closed).
pub fn docked(ctx: &egui::Context, p: &mut Panel, i: PanelIn) -> PanelOut {
    let mut out = PanelOut::default();
    if !p.open {
        return out;
    }
    let screen = ctx.screen_rect();
    let w = width(screen);
    egui::Area::new(egui::Id::new("pal-panel"))
        .order(egui::Order::Foreground)
        .anchor(egui::Align2::RIGHT_TOP, egui::vec2(0.0, 0.0))
        .show(ctx, |ui| {
            egui::Frame::NONE
                .fill(Color32::from_black_alpha(215))
                .inner_margin(MARGIN)
                .show(ui, |ui| {
                    ui.set_width(w);
                    ui.set_height(screen.height() - 2.0 * MARGIN);
                    p.look.apply(ui);
                    let close = ui.horizontal_wrapped(|ui| tabs(ui, p)).inner;
                    ui.separator();
                    egui::ScrollArea::vertical().show(ui, |ui| {
                        out = body(ui, p, i);
                    });
                    out.close = close;
                });
        });
    if out.close {
        p.open = false;
    }
    out
}

/// The chosen tab's rows, into the caller's `ui` (its scroll area).
pub fn body(ui: &mut Ui, p: &mut Panel, i: PanelIn) -> PanelOut {
    let mut out = PanelOut::default();
    let look = p.look;
    let h = look.row_h;
    match p.tab {
        Tab::Video => out.cmd = rows_ui(ui, &VIDEO, i.table, i.edits, look),
        Tab::Record => {
            ui.label("On the drone (microSD):");
            out.cmd = rows_ui(ui, &RECORD, i.table, i.edits, look);
            ui.separator();
            ui.label(RichText::new("Ground DVR: not implemented yet").color(Color32::GRAY));
        }
        Tab::Link if p.chan_menu && !i.channels.is_empty() => {
            let m = channel_menu(ui, i.prefs, i.channels, i.scan, look);
            p.chan_menu = !m.back;
            if let Some(c) = m.choose {
                out.prefs_changed = true;
                out.cmd = Some(Cmd::Channels(c));
            } else if let Some(set) = m.rescan {
                out.cmd = Some(Cmd::ChannelScan(set));
            }
        }
        Tab::Link => {
            ui.label(RichText::new(&i.link_info).monospace());
            ui.separator();
            ui.horizontal(|ui| {
                ui.label("Video width");
                for bw in [20u8, 40] {
                    let on = i.prefs.link_bw == bw;
                    let b = btn(format!("{bw} MHz"), h).selected(on);
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
            ui.separator();
            if ui
                .checkbox(&mut i.prefs.s3_demote, "Step down on enhance-layer loss")
                .changed()
            {
                out.prefs_changed = true;
                out.relink = true;
            }
            ui.label(
                RichText::new(
                    "On: any lost droppable frame steps the ladder down (early, \
                                         cautious). Off: only base-layer loss does, so the bitrate \
                                         climbs higher. Restarts the link.",
                )
                .small()
                .color(Color32::GRAY),
            );
            if !i.channels.is_empty() {
                ui.separator();
                if let Some(set) = channels_button(ui, i.prefs, i.channels, h) {
                    p.chan_menu = true;
                    out.cmd = Some(Cmd::ChannelScan(set));
                }
            }
            ui.separator();
            if let Some(c) = rows_ui(ui, &LINK, i.table, i.edits, look) {
                out.cmd = Some(c);
            }
        }
        Tab::Osd => out.prefs_changed = osd_ui(ui, i.prefs),
        Tab::Drone => {
            if let Some(c) = crate::cal::ui(ui, i.cal, i.table, &mut p.cal_confirm, h) {
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
                    ui.label(RichText::new("table mismatch: drone build differs").color(BAD));
                }
            }
            if let Some(c) = rows_ui(ui, &[], i.table, i.edits, look) {
                out.cmd = Some(c);
            }
        }
        Tab::Logs => {
            ui.label(&i.log_info);
            ui.label(
                RichText::new(format!("Palianytsia {}", pal_build::ID))
                    .small()
                    .color(Color32::GRAY),
            );
        }
    }
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
    ui.separator();
    ui.horizontal(|ui| {
        use crate::prefs::VideoFilter as F;
        ui.label("Video filter (Quest)");
        ui.radio_value(&mut p.video_filter, F::Off, "off");
        ui.radio_value(&mut p.video_filter, F::Sharpen, "sharpen");
        ui.radio_value(&mut p.video_filter, F::Supersample, "supersample");
    });
    ui.horizontal(|ui| {
        use crate::prefs::QuestView as V;
        ui.label("View (Quest)");
        ui.radio_value(&mut p.quest_view, V::Goggle, "goggle");
        ui.radio_value(&mut p.quest_view, V::Window, "window in the room");
    });
    *p != before
}

/// The Link tab's "Channels" button with the current choice on it.
/// Clicked: the channels to scan as the menu opens (the ones the width
/// offers).
fn channels_button(ui: &mut Ui, prefs: &HudPrefs, set: &[u8], h: f32) -> Option<Vec<u8>> {
    use crate::channels::{chosen, offered};
    let offer = offered(set, prefs.link_bw);
    let cur = chosen(&prefs.link_channels, &offer);
    let what = if cur.len() == offer.len() {
        "all, auto".to_string()
    } else if cur.len() == 1 {
        format!("{} only", cur[0])
    } else {
        let l: Vec<String> = cur.iter().map(u8::to_string).collect();
        format!("{}, auto", l.join(" "))
    };
    let b = btn(format!("Channels: {what}  >"), h);
    ui.add(b).clicked().then_some(offer)
}

#[derive(Default)]
struct MenuOut {
    back: bool,
    /// The channels to fly, changed by a click (saved in prefs already).
    choose: Option<Vec<u8>>,
    /// Scan these again.
    rescan: Option<Vec<u8>>,
}

/// Bar colour for a busy share: what the boot pick would call clean,
/// contested, blocked (radio.scan.blocked_pct is 50 in the bundle).
fn busy_color(b: f64) -> Color32 {
    if b < 20.0 {
        OK
    } else if b < 50.0 {
        WARN
    } else {
        BAD
    }
}

/// The channel menu: one row per channel the width offers, its toggle, the
/// scan's reading of it, and "all" / "rescan" / back.
fn channel_menu(
    ui: &mut Ui,
    prefs: &mut HudPrefs,
    set: &[u8],
    scan: Option<&crate::channels::ScanView>,
    look: Look,
) -> MenuOut {
    use crate::channels::{chosen, offered, toggle};
    let mut out = MenuOut::default();
    let offer = offered(set, prefs.link_bw);
    let cur = chosen(&prefs.link_channels, &offer);
    let h = look.row_h;
    let mut next = None;
    ui.horizontal(|ui| {
        if ui.add(btn("<  Link", h)).clicked() {
            out.back = true;
        }
        let all = cur.len() == offer.len();
        let b = btn("all", h).selected(all);
        if ui.add(b).clicked() && !all {
            next = Some(Vec::new());
        }
        let running = scan.is_some_and(|s| s.running);
        let b = btn("rescan", h);
        if ui.add_enabled(!running, b).clicked() {
            out.rescan = Some(offer.clone());
        }
    });
    let status = match scan {
        Some(s) if s.running => format!("scanning, pass {} of {}", s.pass + 1, s.passes),
        Some(_) => "scan done".to_string(),
        None => "no scan (no link core yet)".to_string(),
    };
    ui.label(RichText::new(status).small().color(Color32::GRAY));
    egui::Grid::new("pal-chan-menu")
        .num_columns(3)
        .spacing(egui::vec2(10.0, look.grid_gap))
        .show(ui, |ui| {
            for &ch in &offer {
                let b = egui::Button::new(format!("{ch:>3}"))
                    .selected(cur.contains(&ch))
                    .min_size(egui::vec2(56.0, h));
                if ui.add(b).clicked() {
                    if let Some(n) = toggle(&prefs.link_channels, &offer, prefs.link_bw, ch) {
                        next = Some(n);
                    }
                }
                let row = scan.and_then(|s| s.row(ch));
                match row.and_then(|r| r.busy) {
                    Some(b) => {
                        ui.add(
                            egui::ProgressBar::new((b / 100.0).clamp(0.0, 1.0) as f32)
                                .desired_width(120.0)
                                .fill(busy_color(b))
                                .text(format!("{b:.0}% busy")),
                        );
                    }
                    None => {
                        ui.label(RichText::new("-").color(Color32::GRAY));
                    }
                }
                let mut note = row
                    .and_then(|r| r.foreign)
                    .map_or(String::new(), |f| format!("{f:.0} foreign/s"));
                if (52..=144).contains(&ch) {
                    // Radar channels: the rules want a listen-before-talk
                    // check this link does not do.
                    note.push_str("  DFS");
                }
                if row.is_some_and(|r| r.op) {
                    note.push_str("  link here");
                }
                ui.label(RichText::new(note).small());
                ui.end_row();
            }
        });
    let mode = if cur.len() == 1 {
        format!("Fixed on {}: the link never leaves it.", cur[0])
    } else {
        "Auto: the link sits on the quietest chosen channel and hops among them away from \
         interference."
            .to_string()
    };
    ui.label(
        RichText::new(format!(
            "{mode} A change moves the link by a hop, without restarting it. The link \
             channel's reading includes the link itself. With one card the scan leaves the \
             link channel for 10 ms at a time: expect a few dropped frames while it runs."
        ))
        .small()
        .color(Color32::GRAY),
    );
    if let Some(n) = next {
        prefs.link_channels = n;
        out.choose = Some(chosen(&prefs.link_channels, &offer));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn menu_from_waybeams_six_mode_driver() {
        let m = mode_menu(
            "60@2944x1656/native,90@1920x1080/binned,100@2272x1278/native,100@1728x972/binned,120@1728x816/binned",
        );
        assert_eq!(
            m,
            [
                "60@1920x1080/binned",
                "60@1280x720/binned",
                "90@1920x1080/binned",
                "90@1280x720/binned",
                "100@1728x972/binned",
                "100@1280x720/binned",
                "120@1728x816/binned",
                "60@2944x1656/native",
                "60@2560x1440/native",
                "60@1920x1080/native",
                "60@1280x720/native",
                "90@2272x1278/native",
                "90@1920x1080/native",
                "90@1280x720/native",
                "100@2272x1278/native",
                "100@1920x1080/native",
                "100@1280x720/native",
            ]
        );
    }

    #[test]
    fn a_driver_without_native_100_offers_no_native_above_60() {
        let m = mode_menu("60@2944x1656/native,90@1920x1080/binned,120@1728x816/binned");
        // 1280x720 is 16:9, 1728x816 is not: no 720p at 120.
        assert!(!m.contains(&"120@1280x720/binned".to_string()));
        assert!(m.contains(&"120@1728x816/binned".to_string()));
        assert!(!m
            .iter()
            .any(|s| s.starts_with("100@") || s.starts_with("90@2944")));
        assert!(m.contains(&"60@2560x1440/native".to_string()));
    }

    #[test]
    fn a_read_only_row_takes_no_set() {
        let t = Table {
            linked: true,
            ..Default::default()
        };
        let r = Row {
            value: Some("1".into()),
            apply: "live".into(),
            ..Default::default()
        };
        assert!(settable(&r, &t));
        let ro = Row {
            apply: "read-only".into(),
            ..r
        };
        assert!(!settable(&ro, &t));
    }

    #[test]
    fn empty_or_garbled_list_falls_back() {
        let row = |v: &str| {
            format!(
                r#"{{"age_ms":null,"apply":"read-only","help":"h","id":"VENC_MODES","key":"driver_modes","section":"venc","state":"idle","status":"","type":"str","value":"{v}"}}"#
            )
        };
        for v in ["", "junk"] {
            let t = Table::parse(&format!(r#"{{"linked":true,"rows":[{}]}}"#, row(v))).unwrap();
            assert_eq!(presets("VENC_MODE", &t).len(), FALLBACK_MODES.len());
        }
        let t = Table::parse(&format!(
            r#"{{"linked":true,"rows":[{}]}}"#,
            row("90@1920x1080/binned")
        ))
        .unwrap();
        assert_eq!(
            presets("VENC_MODE", &t),
            [
                "60@1920x1080/binned",
                "60@1280x720/binned",
                "90@1920x1080/binned",
                "90@1280x720/binned"
            ]
        );
    }
}
