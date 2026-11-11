//! What the headset draws with egui (spec 5.6 "OSD and menus"), on
//! surfaces of its own:
//! - the HUD: pal-osd's flight HUD (and the link statistics when enabled),
//!   transparent, shown over the video quad. Redrawn when a stats line
//!   arrives (10 Hz) or every 250 ms, never at video rate;
//! - the menu: pal-osd's settings on an opaque surface the frame loop hangs
//!   in front of the pilot while it is open. Redrawn on input, at most
//!   30 Hz, and not at all while closed;
//! - the pointer: a ray and a cursor, each drawn once; the frame loop moves
//!   their quads every frame, so they follow the hand at display rate.
//!
//! The parts stand side by side: the `Console` holds the UI state and is
//! fed once per tick, a `Surface` only draws what its closure says. Whether
//! the menu is open is the frame loop's fact (`UiIn::menu_open`); the panel
//! state here follows it.
use std::path::PathBuf;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::{Duration, Instant};

use openxr as xr;
use pal_osd::console::{Console, Feeds};
use pal_osd::panel::{Look, Panel, PanelOut, SURFACE_POINTS};
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

/// The menu surface: pal-osd's 800 x 600 points at 2 px per point. On the
/// 44-degree quad that is 36 px per degree, above what the panel resolves.
const MENU_PPP: f32 = 2.0;

/// The OSD's size in egui points.
pub fn points() -> egui::Vec2 {
    egui::vec2(W as f32 / PPP, H as f32 / PPP)
}

/// Where the video quad lies on the OSD, in points.
pub fn video_rect() -> egui::Rect {
    let p = points();
    egui::Rect::from_center_size((p / 2.0).to_pos2(), p / SPAN)
}

/// The surfaces the ray can point at.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Surf {
    Menu,
    Osd,
}

/// What the frame loop tells the UI each frame.
#[derive(Default)]
pub struct UiIn {
    /// The menu quad is up.
    pub menu_open: bool,
    /// Button edge: toggle the link statistics.
    pub stats: bool,
    /// Trigger edge: Some(true) pressed, Some(false) released.
    pub click: Option<bool>,
    /// Where the ray meets a surface, in that surface's points.
    pub on: Option<(Surf, egui::Pos2)>,
    /// Menu scroll for this frame, in points; positive scrolls up.
    pub scroll: f32,
}

/// What the UI asks of the frame loop.
#[derive(Default)]
pub struct UiOut {
    /// The menu's close button was pressed.
    pub close: bool,
    /// A press landed on a control: tick the controller.
    pub pulse: bool,
}

pub struct Osd {
    pub hud: Surface,
    pub menu: Surface,
    /// The pointer's sprites, drawn once.
    pub ray: Surface,
    pub cursor: Surface,
    link: Option<LinkUi>,
    console: Console,
    /// Where the pointer was last frame, for the events that end a visit.
    pointer: Option<(Surf, egui::Pos2)>,
    /// The trigger is down since a press on that surface.
    held: bool,
    /// The menu has a stand-in pointer so the stick can scroll it while
    /// the ray is elsewhere.
    parked: bool,
    /// Holds a pressed click still against hand jitter.
    lock: pal_osd::pointer::ClickLock,
    /// Pointer events not yet drawn: a surface redraws at most 30 times a
    /// second, the frame loop runs faster.
    menu_pending: Vec<egui::Event>,
    hud_pending: Vec<egui::Event>,
    t0: Instant,
    hud_drawn: Option<Instant>,
    menu_drawn: Option<Instant>,
    menu_was_open: bool,
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
}

fn button(pos: egui::Pos2, pressed: bool) -> egui::Event {
    egui::Event::PointerButton {
        pos,
        button: egui::PointerButton::Primary,
        pressed,
        modifiers: egui::Modifiers::default(),
    }
}

/// A sprite: one egui pass that paints `shape` on a clear surface.
fn sprite(
    session: &xr::Session<xr::OpenGlEs>,
    gl: &Arc<glow::Context>,
    w: u32,
    h: u32,
    paint: impl Fn(&egui::Painter, egui::Rect),
) -> Result<Surface, String> {
    let mut s = Surface::new(session, gl.clone(), w, h, 1.0)?;
    s.draw(Vec::new(), |ctx| {
        let layer = egui::LayerId::new(egui::Order::Background, egui::Id::new("sprite"));
        paint(&ctx.layer_painter(layer), ctx.screen_rect());
    })?;
    Ok(s)
}

impl Osd {
    pub fn new(
        session: &xr::Session<xr::OpenGlEs>,
        gl: Arc<glow::Context>,
        prefs_path: PathBuf,
    ) -> Result<Self, String> {
        let [mw, mh] = SURFACE_POINTS;
        let white = egui::Color32::WHITE;
        Ok(Self {
            hud: Surface::new(session, gl.clone(), W, H, PPP)?,
            menu: Surface::new(
                session,
                gl.clone(),
                (mw * MENU_PPP) as u32,
                (mh * MENU_PPP) as u32,
                MENU_PPP,
            )?,
            // A thin bar, solid in the middle, soft at the long edges.
            ray: sprite(session, &gl, 8, 64, |p, r| {
                p.rect_filled(r, 0.0, egui::Color32::from_white_alpha(70));
                p.rect_filled(r.shrink2(egui::vec2(2.0, 0.0)), 0.0, white);
            })?,
            // A white disc with a dark rim, readable on any background.
            cursor: sprite(session, &gl, 64, 64, |p, r| {
                p.circle(
                    r.center(),
                    24.0,
                    white,
                    egui::Stroke::new(6.0_f32, egui::Color32::BLACK),
                );
            })?,
            link: None,
            console: Console::new(
                prefs_path,
                Panel {
                    look: Look::HEADSET,
                    xr: true,
                    ..Default::default()
                },
            ),
            pointer: None,
            held: false,
            parked: false,
            lock: Default::default(),
            menu_pending: Vec::new(),
            hud_pending: Vec::new(),
            t0: Instant::now(),
            hud_drawn: None,
            menu_drawn: None,
            menu_was_open: false,
            fps: 0.0,
            drawn_stall: Stall::None,
        })
    }

    /// The pointer events of this frame for each surface. A surface the
    /// ray has left gets the release of a held press and a PointerGone;
    /// the one under the ray gets the move and the trigger edge.
    fn pointer_events(
        &mut self,
        on: Option<(Surf, egui::Pos2)>,
        click: Option<bool>,
    ) -> (Vec<egui::Event>, Vec<egui::Event>) {
        let (mut menu, mut hud) = (Vec::new(), Vec::new());
        let mut to = |s: Surf, e: egui::Event| match s {
            Surf::Menu => menu.push(e),
            Surf::Osd => hud.push(e),
        };
        if let Some((was, last)) = self.pointer {
            if on.map(|o| o.0) != Some(was) {
                if self.held {
                    to(was, button(self.lock.place(last, Some(false)), false));
                    self.held = false;
                }
                to(was, egui::Event::PointerGone);
            }
        }
        if let Some((s, p)) = on {
            let p = self.lock.place(p, click);
            to(s, egui::Event::PointerMoved(p));
            if let Some(pressed) = click {
                to(s, button(p, pressed));
                self.held = pressed;
            }
        }
        self.pointer = on;
        (menu, hud)
    }

    /// Drains the link's channels and applies the frame's input; redraws
    /// the surfaces whose content changed or whose last draw is old.
    pub fn tick(&mut self, hub: &Hub, inp: UiIn) -> UiOut {
        let mut out = UiOut::default();
        let mut hud_changed = false;
        if inp.stats {
            self.console.toggle_stats();
            hud_changed = true;
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
        // The frame loop owns "open"; the panel state follows it.
        self.console.panel.open = inp.menu_open;
        // The ray acts only while the menu is open: in flight it and the
        // trigger do nothing.
        let on = inp.on.filter(|_| inp.menu_open);
        let (mut menu_ev, mut hud_ev) = self.pointer_events(on, inp.click);
        let on_menu = matches!(on, Some((Surf::Menu, _)));
        if inp.menu_open && inp.scroll != 0.0 {
            // egui scrolls the area under the pointer: with the ray
            // elsewhere, park a pointer on the menu's body for the scroll.
            if !on_menu {
                let [w, h] = SURFACE_POINTS;
                menu_ev.push(egui::Event::PointerMoved(egui::pos2(w * 0.6, h * 0.5)));
                self.parked = true;
            }
            menu_ev.push(egui::Event::MouseWheel {
                unit: egui::MouseWheelUnit::Point,
                delta: egui::vec2(0.0, inp.scroll),
                modifiers: egui::Modifiers::default(),
            });
        } else if self.parked && !on_menu {
            menu_ev.push(egui::Event::PointerGone);
            self.parked = false;
        }
        if on_menu {
            self.parked = false;
        }
        let since = |t: Option<Instant>, ms: u64| {
            t.is_none_or(|t| t.elapsed() >= Duration::from_millis(ms))
        };
        if inp.menu_open {
            // A trigger edge is never dropped; moves and scrolls are drawn
            // at most 30 times a second; an idle menu still follows the
            // table (a set's answer) and finishes egui's animations.
            let edge = menu_ev
                .iter()
                .any(|e| matches!(e, egui::Event::PointerButton { .. }));
            let opened = !self.menu_was_open;
            if opened && !on_menu {
                // Where the ray was when the menu last closed is not where
                // it is now: no stale hover on the first picture.
                self.menu_pending.push(egui::Event::PointerGone);
            }
            // Events of frames that are not drawn wait for the next draw.
            self.menu_pending.append(&mut menu_ev);
            let active = !self.menu_pending.is_empty() || fresh;
            if edge
                || opened
                || (active && since(self.menu_drawn, 33))
                || since(self.menu_drawn, 100)
            {
                let events = std::mem::take(&mut self.menu_pending);
                match self.draw_menu(hub, events) {
                    Ok((close, used)) => {
                        out.close = close;
                        out.pulse = inp.click == Some(true) && on_menu && used;
                    }
                    Err(e) => log::warn!("menu: {e}"),
                }
                self.menu_drawn = Some(Instant::now());
            }
        } else {
            self.menu_drawn = None;
            self.menu_pending.clear();
        }
        self.menu_was_open = inp.menu_open;
        // A stalled picture redraws at 10 Hz so its age keeps counting, and
        // at once when the stall state changes: the idle cadence alone
        // would show "NO VIDEO" up to 250 ms late and keep it after the
        // first new frame.
        let stall = self.console.stall();
        hud_changed |= !stall.same_kind(&self.drawn_stall);
        // Chips are dragged with the ray on the HUD: follow it at 30 Hz,
        // never drop a trigger edge, keep the events of undrawn frames.
        let hud_edge = hud_ev
            .iter()
            .any(|e| matches!(e, egui::Event::PointerButton { .. }));
        self.hud_pending.append(&mut hud_ev);
        let pointed = !self.hud_pending.is_empty();
        let every = if stall != Stall::None { 100 } else { 250 };
        if fresh
            || hud_changed
            || hud_edge
            || (pointed && since(self.hud_drawn, 33))
            || since(self.hud_drawn, every)
        {
            self.drawn_stall = stall;
            let mut status = hub.status.lock().unwrap_or_else(|p| p.into_inner()).clone();
            if let Some(l) = self.link.as_ref() {
                if status.is_empty() && l.shared.searching.load(Ordering::Relaxed) {
                    status = l.shared.radio_label(l.info.width);
                }
            }
            let events = std::mem::take(&mut self.hud_pending);
            if let Err(e) = self.draw_hud(now, &status, events) {
                log::warn!("osd: {e}");
            }
            self.hud_drawn = Some(Instant::now());
        }
        out
    }

    /// The menu surface; returns (close pressed, egui used the pointer).
    fn draw_menu(&mut self, hub: &Hub, events: Vec<egui::Event>) -> Result<(bool, bool), String> {
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
        let mut pout = PanelOut::default();
        let drawn = self.menu.draw(events, |ctx| {
            pout = console.show_surface(ctx, link_info.clone(), log_info.clone(), channels);
        })?;
        self.console
            .settle(&mut pout, self.link.as_ref().map(|l| &l.ports.cmd_tx));
        if pout.relink {
            hub.relink.store(true, Ordering::Relaxed);
        }
        Ok((pout.close, drawn.used_pointer))
    }

    fn draw_hud(&mut self, now: f64, status: &str, events: Vec<egui::Event>) -> Result<(), String> {
        let fps = self.fps;
        let kbps = self.link.as_ref().map_or(0.0, |l| l.shared.rx_kbps());
        let console = &mut self.console;
        let dec_errors = self
            .link
            .as_ref()
            .map_or(0, |l| l.shared.dec_errors.load(Ordering::Relaxed));
        let mut chips_moved = false;
        self.hud.draw(events, |ctx| {
            egui::CentralPanel::default()
                .frame(egui::Frame::NONE)
                .show(ctx, |ui| {
                    let status = (!status.is_empty()).then(|| {
                        egui::RichText::new(status)
                            .size(20.0)
                            .color(egui::Color32::from_rgb(240, 190, 60))
                            .background_color(egui::Color32::from_black_alpha(160))
                    });
                    chips_moved = console.overlay(
                        ui,
                        now,
                        pal_osd::console::OverlayIn {
                            video: video_rect(),
                            // The menu is on a surface of its own: the HUD
                            // keeps the whole picture.
                            hud: video_rect(),
                            chips_in_hud: true,
                            top_offset: 36.0,
                            status,
                            fps,
                            kbps,
                            dec_errors,
                        },
                    );
                });
        })?;
        if chips_moved {
            self.console.save_prefs();
        }
        Ok(())
    }
}
