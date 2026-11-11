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
    /// What a pilot does in the air: recording, the view, the statistics.
    #[default]
    Flight,
    Picture,
    Link,
    Recording,
    /// The HUD and how the video is shown.
    Screen,
    /// Versions, logs, rarely touched rows and the whole parameter table.
    System,
}

impl Tab {
    pub const ALL: [Tab; 6] = [
        Tab::Flight,
        Tab::Picture,
        Tab::Link,
        Tab::Recording,
        Tab::Screen,
        Tab::System,
    ];
    pub fn name(self) -> &'static str {
        match self {
            Tab::Flight => "Flight",
            Tab::Picture => "Picture",
            Tab::Link => "Link",
            Tab::Recording => "Recording",
            Tab::Screen => "Screen",
            Tab::System => "System",
        }
    }
    /// By name, any case.
    pub fn parse(s: &str) -> Option<Tab> {
        Tab::ALL
            .into_iter()
            .find(|t| t.name().eq_ignore_ascii_case(s))
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
    /// A drone row's name in a column this wide with its control beside
    /// it (a wide surface); None: the control under the name.
    pub label_w: Option<f32>,
}

impl Look {
    /// Mouse and keyboard.
    pub const DESKTOP: Look = Look {
        row_h: 0.0,
        text: 1.0,
        steppers: false,
        spacing: None,
        grid_gap: 2.0,
        label_w: None,
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
        label_w: None,
    };

    /// A headset's menu surface ([`surface`], [`SURFACE_POINTS`]) read at
    /// about a metre and aimed at with a hand-held ray: egui's 14 pt body
    /// text becomes 25 pt and its 10 pt small text 18 pt.
    pub const HEADSET: Look = Look {
        row_h: 56.0,
        text: 1.8,
        steppers: true,
        spacing: Some(Spacing {
            interact: egui::vec2(56.0, 56.0),
            button_padding: egui::vec2(18.0, 12.0),
            item: egui::vec2(12.0, 10.0),
            icon: 34.0,
        }),
        grid_gap: 8.0,
        label_w: Some(330.0),
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
    /// The front end is a headset: the view and video filter rows apply.
    pub xr: bool,
    /// System shows the whole parameter table unfolded (desktop `--params`).
    pub table_open: bool,
    /// A set held back because the drone is armed and the row restarts the
    /// video: row index and value, waiting for Apply or Cancel.
    pub confirm: Option<(usize, String)>,
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
    /// A standing warning for the Flight tab and whether it is an error
    /// (`widgets::key_warning`).
    pub warning: Option<(String, bool)>,
    /// The flight controller's arm state; None before any telemetry.
    pub armed: Option<bool>,
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

/// Drone rows per tab, by F18 id: the main part, then what folds under
/// "Advanced". The whole table is under System.
const PICTURE: [&str; 4] = ["VENC_MODE", "ENC_BITRATE_MAX", "VENC_MIRROR", "VENC_FLIP"];
const PICTURE_ADV: [&str; 3] = ["ENC_AIRTIME", "VENC_SLICES", "VENC_MIN_IQP"];
const LINK: [&str; 2] = ["RADIO_PWR_MODE", "RADIO_WALL_MARGN"];
const LINK_ADV: [&str; 1] = ["RADIO_LDPC"];
const RECORDING: [&str; 4] = ["REC_ENABLE", "REC_SIZE", "REC_FPS", "REC_BITRATE"];
const SYSTEM: [&str; 2] = ["LOWPWR_ENABLE", "MSP_ENABLE"];

/// A row's name for a pilot; the drone's `section.key` shows small under
/// it. None for a row this build does not know: its key is the name.
fn plain_name(id: &str) -> Option<&'static str> {
    Some(match id {
        "VENC_MODE" => "Camera mode",
        "ENC_BITRATE_MAX" => "Bitrate ceiling, kbps",
        "ENC_AIRTIME" => "Share of the air the video may use",
        "VENC_SLICES" => "Slices per frame",
        "VENC_MIN_IQP" => "Keyframe quality floor",
        "VENC_MIRROR" => "Mirror left-right",
        "VENC_FLIP" => "Flip upside down",
        // Not "record now": the row switches the recorder on the drone on
        // or off as a whole. A recording is started by the GS's record
        // wish in the RCF (mabur docs/vtx-recorder.md), which this GS does
        // not send yet.
        "REC_ENABLE" => "Onboard recorder available",
        "REC_SIZE" => "Recording size",
        "REC_FPS" => "Recording frame rate",
        "REC_BITRATE" => "Recording bitrate, kbps",
        "RADIO_PWR_MODE" => "TX power table",
        "RADIO_WALL_MARGN" => "TX power margin, dB",
        "RADIO_LDPC" => "LDPC coding",
        "LOWPWR_ENABLE" => "Low power while disarmed",
        "MSP_ENABLE" => "Flight controller link (MSP)",
        "VENC_MODES" => "Camera driver modes",
        _ => return None,
    })
}

/// The drone rebuilds its video pipeline to apply the row: the picture
/// stops while it does.
fn restarts_video(r: &Row) -> bool {
    r.apply == "rebuild"
}

/// What becomes of the panel's command this frame. A set of a row that
/// restarts the video is held in `held` while the drone is armed, and the
/// panel asks; everything else goes out as it is.
fn gate(
    cmd: Option<Cmd>,
    t: Option<&Table>,
    armed: Option<bool>,
    held: &mut Option<(usize, String)>,
) -> Option<Cmd> {
    match cmd {
        Some(Cmd::Set(i, v))
            if armed == Some(true) && t.and_then(|t| t.rows.get(i)).is_some_and(restarts_video) =>
        {
            *held = Some((i, v));
            None
        }
        c => c,
    }
}

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

/// A row's name, and under it the drone's key and what a set costs.
fn row_label(ui: &mut Ui, r: &Row) {
    let help = format!("{} ({}, applies by {})\n{}", r.id, r.ty, r.apply, r.help);
    let key = format!("{}.{}", r.section, r.key);
    match plain_name(&r.id) {
        Some(name) => {
            ui.label(RichText::new(name).strong()).on_hover_text(&help);
            ui.scope(|ui| {
                // The key line is text, not a control: it does not need a
                // control's height (a row is as tall as `interact_size`).
                ui.spacing_mut().interact_size.y = 0.0;
                ui.horizontal_wrapped(|ui| {
                    ui.label(RichText::new(key).small().color(Color32::GRAY));
                    if restarts_video(r) {
                        ui.label(RichText::new("restarts the video").small().color(WARN));
                    }
                });
            });
        }
        None => {
            ui.label(RichText::new(key).strong()).on_hover_text(&help);
        }
    }
}

/// A row's live value and the control that changes it.
fn row_control(
    ui: &mut Ui,
    i: usize,
    r: &Row,
    t: &Table,
    e: &mut Edits,
    look: Look,
) -> Option<Cmd> {
    let mut cmd = None;
    let can = settable(r, t);
    {
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
    }
    cmd
}

fn row_ui(ui: &mut Ui, i: usize, r: &Row, t: &Table, e: &mut Edits, look: Look) -> Option<Cmd> {
    let cmd = match look.label_w {
        // Name on the left, control on the right: one row high. A text
        // value (a camera mode, a size) is too long to sit beside its name.
        Some(w) if r.ty != "str" => {
            ui.horizontal(|ui| {
                ui.allocate_ui_with_layout(
                    egui::vec2(w, 0.0),
                    egui::Layout::top_down(egui::Align::LEFT),
                    |ui| {
                        ui.set_width(w);
                        ui.spacing_mut().item_spacing.y = 2.0;
                        row_label(ui, r);
                    },
                );
                row_control(ui, i, r, t, e, look)
            })
            .inner
        }
        _ => {
            row_label(ui, r);
            ui.horizontal(|ui| row_control(ui, i, r, t, e, look)).inner
        }
    };
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

/// The state of the drone's table, once above a tab's drone rows: no
/// table, not linked, a set on its way, the last refusal.
fn table_status(ui: &mut Ui, t: Option<&Table>, e: &mut Edits) {
    let Some(t) = t else {
        ui.label(RichText::new("no drone settings yet (GS mode, linked)").color(WARN));
        return;
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
        let name = t
            .rows
            .get(*row)
            .map_or("?", |r| plain_name(&r.id).unwrap_or(r.id.as_str()));
        ui.label(RichText::new(format!("{name}: refused, {why}")).color(BAD));
    }
}

/// The rows named in `ids`, in that order (empty: the whole table in the
/// drone's order). A row the drone does not have is left out.
fn rows_ui(ui: &mut Ui, ids: &[&str], t: Option<&Table>, e: &mut Edits, look: Look) -> Option<Cmd> {
    let t = t?;
    if e.text.len() != t.rows.len() {
        e.text = vec![String::new(); t.rows.len()];
    }
    let mut cmd = None;
    let mut row = |ui: &mut Ui, i: usize, r: &Row| {
        if let Some(c) = row_ui(ui, i, r, t, e, look) {
            cmd = Some(c);
        }
    };
    if ids.is_empty() {
        for (i, r) in t.rows.iter().enumerate() {
            row(ui, i, r);
        }
    } else {
        for id in ids {
            if let Some((i, r)) = t.rows.iter().enumerate().find(|(_, r)| r.id == *id) {
                row(ui, i, r);
            }
        }
    }
    cmd
}

/// `ids` folded under an "Advanced" header, closed until asked for.
fn advanced(ui: &mut Ui, salt: &str, add: impl FnOnce(&mut Ui) -> Option<Cmd>) -> Option<Cmd> {
    egui::CollapsingHeader::new("Advanced")
        .id_salt(salt)
        .show(ui, add)
        .body_returned
        .flatten()
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

/// The size, in egui points, of a surface [`surface`] is laid out for.
pub const SURFACE_POINTS: [f32; 2] = [800.0, 600.0];
/// Width of its tab rail.
const RAIL_W: f32 = 190.0;

/// The container for a surface of its own (the headset's menu quad): it
/// fills the whole context, opaque, the tabs in a rail on the left and the
/// scrolling body on the right. Drawn whether or not `p.open`: the front
/// end owns the surface and decides when it shows.
pub fn surface(ctx: &egui::Context, p: &mut Panel, i: PanelIn) -> PanelOut {
    let mut out = PanelOut::default();
    egui::CentralPanel::default()
        .frame(
            egui::Frame::NONE
                .fill(Color32::from_rgb(16, 18, 22))
                .inner_margin(14.0),
        )
        .show(ctx, |ui| {
            p.look.apply(ui);
            ui.horizontal_top(|ui| {
                let rail = egui::vec2(RAIL_W, ui.available_height());
                let close = ui
                    .allocate_ui_with_layout(
                        rail,
                        egui::Layout::top_down_justified(egui::Align::LEFT),
                        |ui| tabs(ui, p),
                    )
                    .inner;
                ui.separator();
                ui.vertical(|ui| {
                    egui::ScrollArea::vertical()
                        .auto_shrink(false)
                        .show(ui, |ui| {
                            out = body(ui, p, i);
                        });
                });
                out.close = close;
            });
        });
    out
}

/// The chosen tab's rows, into the caller's `ui` (its scroll area).
pub fn body(ui: &mut Ui, p: &mut Panel, i: PanelIn) -> PanelOut {
    let mut out = PanelOut::default();
    let look = p.look;
    let h = look.row_h;
    // A held set first: it is answered before anything else is changed.
    if let Some((row, value)) = p.confirm.clone() {
        let name = i
            .table
            .and_then(|t| t.rows.get(row))
            .map_or("?", |r| plain_name(&r.id).unwrap_or(r.id.as_str()));
        let warning = format!(
            "The drone is armed. {name} = {value} restarts the video: the picture stops \
             while the drone rebuilds it."
        );
        if let Some(go) = crate::cal::confirm_row(ui, &warning, "Apply", h) {
            if go {
                out.cmd = Some(Cmd::Set(row, value));
            }
            p.confirm = None;
        }
        ui.separator();
    }
    // The tab's own command; a set may be held back by `gate` below.
    let mut cmd = None;
    match p.tab {
        Tab::Flight => {
            if let Some((text, bad)) = &i.warning {
                ui.label(RichText::new(text).color(if *bad { BAD } else { WARN }));
            }
            ui.label(RichText::new(&i.link_info).monospace());
            ui.separator();
            let before = i.prefs.clone();
            if p.xr {
                view_ui(ui, i.prefs);
            }
            ui.checkbox(&mut i.prefs.stats, "Link statistics over the video");
            out.prefs_changed = *i.prefs != before;
        }
        Tab::Picture => {
            table_status(ui, i.table, i.edits);
            cmd = rows_ui(ui, &PICTURE, i.table, i.edits, look);
            let (t, e) = (i.table, &mut *i.edits);
            if let Some(c) = advanced(ui, "picture", |ui| rows_ui(ui, &PICTURE_ADV, t, e, look)) {
                cmd = Some(c);
            }
        }
        Tab::Link if p.chan_menu && !i.channels.is_empty() => {
            let m = channel_menu(ui, i.prefs, i.channels, i.scan, look);
            p.chan_menu = !m.back;
            if let Some(c) = m.choose {
                out.prefs_changed = true;
                cmd = Some(Cmd::Channels(c));
            } else if let Some(set) = m.rescan {
                cmd = Some(Cmd::ChannelScan(set));
            }
        }
        Tab::Link => {
            ui.label(RichText::new(&i.link_info).monospace());
            if !i.channels.is_empty() {
                if let Some(set) = channels_button(ui, i.prefs, i.channels, h) {
                    p.chan_menu = true;
                    cmd = Some(Cmd::ChannelScan(set));
                }
            }
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
                    "20 MHz reaches farther at the robust rungs (4-6 dB at MCS0); 40 MHz \
                     carries more per rung. Switching restarts the link for a few seconds; \
                     the drone needs no change.",
                )
                .small()
                .color(Color32::GRAY),
            );
            ui.separator();
            table_status(ui, i.table, i.edits);
            if let Some(c) = rows_ui(ui, &LINK, i.table, i.edits, look) {
                cmd = Some(c);
            }
            ui.separator();
            if let Some(c) = crate::cal::ui(ui, i.cal, i.table, &mut p.cal_confirm, h) {
                cmd = Some(c);
            }
            let (t, e, prefs) = (i.table, &mut *i.edits, &mut *i.prefs);
            let mut relink = false;
            let adv = advanced(ui, "link", |ui| {
                let c = rows_ui(ui, &LINK_ADV, t, e, look);
                relink = ui
                    .checkbox(&mut prefs.s3_demote, "Step down on enhance-layer loss")
                    .changed();
                ui.label(
                    RichText::new(
                        "On: any lost droppable frame steps the ladder down (early, \
                         cautious). Off: only base-layer loss does, so the bitrate climbs \
                         higher. Restarts the link.",
                    )
                    .small()
                    .color(Color32::GRAY),
                );
                c
            });
            if let Some(c) = adv {
                cmd = Some(c);
            }
            if relink {
                out.prefs_changed = true;
                out.relink = true;
            }
        }
        Tab::Recording => {
            ui.label("On the drone (microSD):");
            table_status(ui, i.table, i.edits);
            cmd = rows_ui(ui, &RECORDING, i.table, i.edits, look);
            ui.separator();
            ui.label(RichText::new("Ground DVR: not implemented yet").color(Color32::GRAY));
        }
        Tab::Screen => out.prefs_changed = screen_ui(ui, i.prefs, p.xr),
        Tab::System => {
            ui.label(format!("Palianytsia {}", pal_build::ID));
            ui.label(RichText::new(&i.log_info).small().color(Color32::GRAY));
            ui.separator();
            if let Some(t) = i.table {
                ui.horizontal_wrapped(|ui| {
                    let alive = match t.alive {
                        Some(true) => "heartbeat ok",
                        Some(false) => "no heartbeat",
                        None => "waiting for heartbeat",
                    };
                    ui.label(format!("drone parameters: list {}, {alive}", t.list));
                    if ui.add_enabled(t.linked, btn("refresh", h)).clicked() {
                        cmd = Some(Cmd::Refresh);
                    }
                });
                if t.mismatch {
                    ui.label(RichText::new("table mismatch: drone build differs").color(BAD));
                }
            }
            table_status(ui, i.table, i.edits);
            if let Some(c) = rows_ui(ui, &SYSTEM, i.table, i.edits, look) {
                cmd = Some(c);
            }
            let (t, e) = (i.table, &mut *i.edits);
            let all = egui::CollapsingHeader::new("All drone parameters")
                .default_open(p.table_open)
                .show(ui, |ui| rows_ui(ui, &[], t, e, look))
                .body_returned
                .flatten();
            if let Some(c) = all {
                cmd = Some(c);
            }
        }
    }
    if let Some(c) = gate(cmd, i.table, i.armed, &mut p.confirm) {
        out.cmd = Some(c);
    }
    out
}

/// Goggle or window: the headset's two ways to show the video.
fn view_ui(ui: &mut Ui, p: &mut HudPrefs) {
    ui.horizontal(|ui| {
        use crate::prefs::QuestView as V;
        ui.label("View");
        ui.radio_value(&mut p.quest_view, V::Goggle, "goggle");
        ui.radio_value(&mut p.quest_view, V::Window, "window in the room");
    });
}

/// The Screen tab: what the HUD shows and, on a headset, the video filter.
/// True when a pref changed.
fn screen_ui(ui: &mut Ui, p: &mut HudPrefs, xr: bool) -> bool {
    let before = p.clone();
    ui.checkbox(&mut p.dot, "Centre dot");
    ui.checkbox(&mut p.battery, "Battery");
    ui.checkbox(&mut p.timer, "Arm state and flight timer");
    ui.checkbox(&mut p.nav, "Altitude, speed, home, satellites");
    ui.checkbox(&mut p.link, "Link readout (RSSI, MCS)");
    ui.checkbox(&mut p.messages, "Flight controller messages");
    ui.add(egui::Slider::new(&mut p.scale, 0.6..=2.0).text("text size"));
    if xr {
        ui.separator();
        ui.horizontal(|ui| {
            use crate::prefs::VideoFilter as F;
            ui.label("Video filter");
            ui.radio_value(&mut p.video_filter, F::Off, "off");
            ui.radio_value(&mut p.video_filter, F::Sharpen, "sharpen");
            ui.radio_value(&mut p.video_filter, F::Supersample, "supersample");
        });
    }
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

    fn table_with(id: &str, apply: &str) -> Table {
        Table {
            linked: true,
            rows: vec![Row {
                id: id.into(),
                apply: apply.into(),
                value: Some("0".into()),
                ..Default::default()
            }],
            ..Default::default()
        }
    }

    #[test]
    fn armed_holds_a_set_that_restarts_the_video() {
        let t = table_with("REC_ENABLE", "rebuild");
        let set = || Some(Cmd::Set(0, "1".into()));
        let mut held = None;
        // Disarmed, and before any telemetry: straight out.
        assert_eq!(gate(set(), Some(&t), Some(false), &mut held), set());
        assert_eq!(gate(set(), Some(&t), None, &mut held), set());
        assert_eq!(held, None);
        // Armed: held for the question, nothing sent.
        assert_eq!(gate(set(), Some(&t), Some(true), &mut held), None);
        assert_eq!(held, Some((0, "1".to_string())));
    }

    #[test]
    fn armed_lets_through_what_does_not_stop_the_picture() {
        let knob = table_with("ENC_BITRATE_MAX", "knob");
        let mut held = None;
        let set = || Some(Cmd::Set(0, "9000".into()));
        assert_eq!(gate(set(), Some(&knob), Some(true), &mut held), set());
        let refresh = gate(Some(Cmd::Refresh), Some(&knob), Some(true), &mut held);
        assert_eq!(refresh, Some(Cmd::Refresh));
        assert_eq!(held, None);
    }

    #[test]
    fn each_tab_fits_the_quest_surface_and_names_known_rows() {
        // 8 rows fit the Quest menu surface without scrolling (pal-ttb.30);
        // the ground rows drawn beside the drone rows count too.
        let ground = |t: Tab| match t {
            Tab::Flight => 3,    // link line, view, statistics
            Tab::Link => 4,      // link line, channels, width, calibration
            Tab::Recording => 1, // ground DVR line
            Tab::System => 3,    // version, table state, the folded table
            Tab::Picture | Tab::Screen => 0,
        };
        let drone: [(Tab, &[&str]); 4] = [
            (Tab::Picture, &PICTURE),
            (Tab::Link, &LINK),
            (Tab::Recording, &RECORDING),
            (Tab::System, &SYSTEM),
        ];
        for (tab, ids) in drone {
            assert!(ids.len() + ground(tab) <= 8, "{tab:?}");
        }
        let all = [
            &PICTURE[..],
            &PICTURE_ADV,
            &LINK,
            &LINK_ADV,
            &RECORDING,
            &SYSTEM,
        ];
        for id in all.into_iter().flatten() {
            assert!(plain_name(id).is_some(), "{id} has no plain name");
        }
    }

    #[test]
    fn tabs_parse_by_name() {
        assert_eq!(Tab::parse("system"), Some(Tab::System));
        assert_eq!(Tab::parse("Flight"), Some(Tab::Flight));
        assert_eq!(Tab::parse("drone"), None);
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
