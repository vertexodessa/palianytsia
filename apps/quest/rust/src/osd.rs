//! What the headset draws with egui (spec 5.6 "OSD and menus"): pal-osd's
//! flight HUD (and the link statistics when enabled) on a transparent
//! surface the XR loop shows over the video quad. Redrawn when a stats line
//! arrives (10 Hz) or every 250 ms, never at video rate. The settings panel
//! (pal-osd's, as on the phone) opens with B or Y and is driven by the
//! controller ray; while it is open the surface redraws at 30 Hz so the
//! cursor follows the hand.
//!
//! The parts stand side by side: the `Console` holds the UI state and is
//! fed once per tick, a `Surface` only draws what its closure says.
use std::path::PathBuf;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::{Duration, Instant};

use openxr as xr;
use pal_osd::console::{Console, Feeds};
use pal_osd::panel::Panel;
use pal_osd::stall::Stall;

use crate::supervisor::{Hub, LinkUi};
use crate::surface::Surface;

/// The OSD quad is SPAN times the video quad in each direction, centred
/// on it, so stats chips can be parked beside the picture.
pub const SPAN: f32 = 1.5;

/// Swapchain size in pixels and egui points per pixel: the video part is
/// 1280x720 points, so pal-osd's sizes match a 720p screen.
pub const W: u32 = 2880;
pub const H: u32 = 1620;
const PPP: f32 = 1.5;

/// The OSD's size in egui points.
pub fn points() -> egui::Vec2 {
    egui::vec2(W as f32 / PPP, H as f32 / PPP)
}

/// Where the video quad lies on the OSD, in points.
pub fn video_rect() -> egui::Rect {
    let p = points();
    egui::Rect::from_center_size((p / 2.0).to_pos2(), p / SPAN)
}

/// What the frame loop tells the UI each frame.
#[derive(Default)]
pub struct UiIn {
    /// Button edges: open or close the settings, toggle the statistics.
    pub menu: bool,
    pub stats: bool,
    /// Trigger edge: Some(true) pressed, Some(false) released.
    pub click: Option<bool>,
    /// Where the ray meets the OSD surface, in its points.
    pub on_osd: Option<egui::Pos2>,
}

pub struct Osd {
    pub hud: Surface,
    link: Option<LinkUi>,
    console: Console,
    /// Last pointer position, for the cursor and the release event.
    pointer: Option<egui::Pos2>,
    /// Holds a pressed click still against hand jitter.
    lock: pal_osd::pointer::ClickLock,
    t0: Instant,
    last_draw: Option<Instant>,
    fps: f64,
    /// The stall state the last draw showed.
    drawn_stall: Stall,
}

impl Osd {
    /// The operator's video layer filter, read every frame by xr.
    pub fn video_filter(&self) -> pal_osd::prefs::VideoFilter {
        self.console.prefs.video_filter
    }

    /// The operator's view mode, read every frame by xr.
    pub fn view_mode(&self) -> pal_osd::prefs::QuestView {
        self.console.prefs.quest_view
    }

    pub fn panel_open(&self) -> bool {
        self.console.panel.open
    }
}

impl Osd {
    pub fn new(
        session: &xr::Session<xr::OpenGlEs>,
        gl: Arc<glow::Context>,
        prefs_path: PathBuf,
    ) -> Result<Self, String> {
        Ok(Self {
            hud: Surface::new(session, gl, W, H, PPP)?,
            link: None,
            console: Console::new(
                prefs_path,
                Panel {
                    // Finger-sized rows suit a hand-held ray too.
                    look: pal_osd::panel::Look::TOUCH,
                    xr: true,
                    ..Default::default()
                },
            ),
            pointer: None,
            lock: Default::default(),
            t0: Instant::now(),
            last_draw: None,
            fps: 0.0,
            drawn_stall: Stall::None,
        })
    }

    /// Drains the link's channels and applies the frame's input; redraws
    /// when something changed or the last draw is old.
    pub fn tick(&mut self, hub: &Hub, inp: UiIn) {
        let mut changed = false;
        if inp.menu {
            self.console.panel.open = !self.console.panel.open;
            changed = true;
        }
        if inp.stats {
            self.console.toggle_stats();
            changed = true;
        }
        if let Some(l) = hub.ui.lock().unwrap_or_else(|p| p.into_inner()).take() {
            log::info!(
                "osd: link {}, {}",
                l.shared.radio_label(l.info.width),
                l.info.log_info
            );
            self.console.new_session();
            self.link = Some(l);
        }
        let now = self.t0.elapsed().as_secs_f64();
        let mut fresh = false;
        if let Some(l) = &self.link {
            fresh = self.console.drain_stats(now, &l.ports.stats_rx);
            let feeds = Feeds {
                mav: &l.ports.mav_rx,
                params: &l.ports.params_rx,
                set_res: &l.ports.set_res_rx,
            };
            // Row state changes into logcat: a set that "hangs" shows here
            // whether the drone's answer arrived.
            for c in self.console.drain(now, feeds) {
                log::info!("{c}");
            }
            self.fps = self
                .console
                .on_frames(now, l.shared.frames.load(Ordering::Relaxed));
        }
        // Pointer events only while the panel is open: in flight the ray
        // and the trigger do nothing.
        let mut events = Vec::new();
        if self.console.panel.open {
            let button = |pos, pressed| egui::Event::PointerButton {
                pos,
                button: egui::PointerButton::Primary,
                pressed,
                modifiers: egui::Modifiers::default(),
            };
            match (inp.on_osd, self.pointer) {
                (Some(p), _) => {
                    let p = self.lock.place(p, inp.click);
                    events.push(egui::Event::PointerMoved(p));
                    if let Some(pressed) = inp.click {
                        events.push(button(p, pressed));
                    }
                }
                (None, Some(last)) => {
                    // Off the quad: finish a held click where it was.
                    if inp.click == Some(false) {
                        events.push(button(self.lock.place(last, Some(false)), false));
                    }
                    events.push(egui::Event::PointerGone);
                }
                (None, None) => {}
            }
            changed |= inp.on_osd != self.pointer || inp.click.is_some();
            self.pointer = inp.on_osd;
        } else {
            self.pointer = None;
        }
        // A stalled picture redraws at 10 Hz so its age keeps counting, and
        // at once when the stall state changes: the idle cadence alone
        // would show "NO VIDEO" up to 250 ms late and keep it after the
        // first new frame.
        let stall = self.console.stall();
        changed |= !stall.same_kind(&self.drawn_stall);
        let every = if self.console.panel.open {
            33
        } else if stall != Stall::None {
            100
        } else {
            250
        };
        let due = self
            .last_draw
            .is_none_or(|t| t.elapsed() >= Duration::from_millis(every));
        if fresh || due || changed {
            self.drawn_stall = stall;
            let mut status = hub.status.lock().unwrap_or_else(|p| p.into_inner()).clone();
            if let Some(l) = self.link.as_ref() {
                if status.is_empty() && l.shared.searching.load(Ordering::Relaxed) {
                    status = l.shared.radio_label(l.info.width);
                }
            }
            if let Err(e) = self.draw(hub, now, &status, events) {
                log::warn!("osd: {e}");
            }
            self.last_draw = Some(Instant::now());
        }
    }

    fn draw(
        &mut self,
        hub: &Hub,
        now: f64,
        status: &str,
        events: Vec<egui::Event>,
    ) -> Result<(), String> {
        let fps = self.fps;
        let kbps = self.link.as_ref().map_or(0.0, |l| l.shared.rx_kbps());
        let console = &mut self.console;
        let link_info = self.link.as_ref().map_or_else(
            || "no link".to_string(),
            |l| {
                format!(
                    "{}, key {}",
                    l.shared.radio_label(l.info.width),
                    console.hist.last().map_or("?", |s| s.key_fp.as_str())
                )
            },
        );
        let log_info = self.link.as_ref().map_or_else(String::new, |l| {
            format!(
                "{}\n\nadb pull /sdcard/Android/data/org.palianytsia.quest/files/logs",
                l.info.log_info
            )
        });
        let channels: &[u8] = self.link.as_ref().map_or(&[], |l| &l.info.channels);
        let cursor = self.pointer.filter(|_| console.panel.open);
        let mut pout = pal_osd::panel::PanelOut::default();
        let dec_errors = self
            .link
            .as_ref()
            .map_or(0, |l| l.shared.dec_errors.load(Ordering::Relaxed));
        let mut chips_moved = false;
        self.hud.draw(events, |ctx| {
            pout = console.show_panel(ctx, link_info.clone(), log_info.clone(), channels);
            let c = &mut *console;
            egui::CentralPanel::default()
                .frame(egui::Frame::NONE)
                .show(ctx, |ui| {
                    let status = (!status.is_empty()).then(|| {
                        egui::RichText::new(status)
                            .size(20.0)
                            .color(egui::Color32::from_rgb(240, 190, 60))
                            .background_color(egui::Color32::from_black_alpha(160))
                    });
                    let hud = pal_osd::panel::hud_rect(ui.ctx(), &c.panel, video_rect());
                    chips_moved = c.overlay(
                        ui,
                        now,
                        pal_osd::console::OverlayIn {
                            video: video_rect(),
                            hud,
                            chips_in_hud: true,
                            top_offset: 36.0,
                            status,
                            fps,
                            kbps,
                            dec_errors,
                        },
                    );
                });
            // The controller ray's end: no controller model is drawn, so
            // this dot is the only sign of where it points.
            if let Some(c) = cursor {
                ctx.layer_painter(egui::LayerId::new(
                    egui::Order::Tooltip,
                    egui::Id::new("cursor"),
                ))
                .circle(
                    c,
                    7.0,
                    egui::Color32::from_rgb(255, 255, 255),
                    egui::Stroke::new(2.0_f32, egui::Color32::BLACK),
                );
            }
        })?;
        self.console
            .settle(&mut pout, self.link.as_ref().map(|l| &l.ports.cmd_tx));
        if chips_moved && !pout.prefs_changed {
            self.console.save_prefs();
        }
        if pout.relink {
            hub.relink.store(true, Ordering::Relaxed);
        }
        Ok(())
    }
}
