//! The OSD layer (spec 5.6 "OSD and menus"): pal-osd's flight HUD (and the
//! link statistics when enabled) drawn by egui into a GLES swapchain that
//! the XR loop submits as a transparent quad over the video quad. Redrawn
//! when a stats line arrives (10 Hz) or every 250 ms, never at video rate;
//! between redraws the compositor keeps showing the last released image.
//! The settings panel (pal-osd's, as on the phone) opens with B or Y and is
//! driven by the controller ray; while it is open the layer redraws at
//! 30 Hz so the cursor follows the hand.
use std::num::NonZeroU32;
use std::path::PathBuf;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::{Duration, Instant};

use glow::HasContext;
use openxr as xr;
use pal_osd::console::{Console, Feeds};
use pal_osd::panel::Panel;
use pal_osd::stall::Stall;

use crate::input::Frame;
use crate::supervisor::{Hub, LinkUi};

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

pub struct Osd {
    gl: Arc<glow::Context>,
    painter: egui_glow::Painter,
    ctx: egui::Context,
    fbo: glow::Framebuffer,
    pub sc: xr::Swapchain<xr::OpenGlEs>,
    images: Vec<u32>,
    /// A released image exists: the layer can be submitted.
    pub ready: bool,
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
        let sc = session
            .create_swapchain(&xr::SwapchainCreateInfo {
                create_flags: xr::SwapchainCreateFlags::EMPTY,
                usage_flags: xr::SwapchainUsageFlags::COLOR_ATTACHMENT
                    | xr::SwapchainUsageFlags::SAMPLED,
                format: glow::RGBA8,
                sample_count: 1,
                width: W,
                height: H,
                face_count: 1,
                array_size: 1,
                mip_count: 1,
            })
            .map_err(|e| format!("OSD swapchain: {e}"))?;
        let images = sc.enumerate_images().map_err(|e| e.to_string())?;
        let painter = egui_glow::Painter::new(gl.clone(), "", None, false)
            .map_err(|e| format!("OSD painter: {e}"))?;
        // SAFETY: the XR session's GLES context is current on this thread.
        let fbo = unsafe { gl.create_framebuffer() }?;
        Ok(Self {
            gl,
            painter,
            ctx: {
                let ctx = egui::Context::default();
                // A held trigger is still a click: a ray press is slower
                // than a tap (egui's 0.8 s is made for touch long-press).
                ctx.options_mut(|o| o.input_options.max_click_duration = 3.0);
                ctx
            },
            fbo,
            sc,
            images,
            ready: false,
            link: None,
            console: Console::new(
                prefs_path,
                Panel {
                    // Finger-sized rows suit a hand-held ray too.
                    touch: true,
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

    /// Drains the link's channels and applies the controller frame;
    /// redraws when something changed or the last draw is old.
    pub fn tick(&mut self, hub: &Hub, inp: Frame) {
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
            match (inp.pointer, self.pointer) {
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
            changed |= inp.pointer != self.pointer || inp.click.is_some();
            self.pointer = inp.pointer;
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
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(egui::Pos2::ZERO, points())),
            events,
            ..Default::default()
        };
        self.ctx.set_pixels_per_point(PPP);
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
        let out = self.ctx.run(input, |ctx| {
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
                    chips_moved = c.overlay(
                        ui,
                        now,
                        pal_osd::console::OverlayIn {
                            video: video_rect(),
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
        });
        self.console
            .settle(&mut pout, self.link.as_ref().map(|l| &l.ports.cmd_tx));
        if chips_moved && !pout.prefs_changed {
            self.console.save_prefs();
        }
        if pout.relink {
            hub.relink.store(true, Ordering::Relaxed);
        }
        let prims = self.ctx.tessellate(out.shapes, out.pixels_per_point);
        let idx = self.sc.acquire_image().map_err(|e| e.to_string())?;
        self.sc
            .wait_image(xr::Duration::INFINITE)
            .map_err(|e| e.to_string())?;
        let tex = NonZeroU32::new(self.images[idx as usize]).map(glow::NativeTexture);
        // SAFETY: the session's GLES context is current on this thread and
        // `tex` is a live swapchain image.
        unsafe {
            self.gl.bind_framebuffer(glow::FRAMEBUFFER, Some(self.fbo));
            self.gl.framebuffer_texture_2d(
                glow::FRAMEBUFFER,
                glow::COLOR_ATTACHMENT0,
                glow::TEXTURE_2D,
                tex,
                0,
            );
        }
        self.painter.clear([W, H], [0.0, 0.0, 0.0, 0.0]);
        self.painter.paint_and_update_textures(
            [W, H],
            out.pixels_per_point,
            &prims,
            &out.textures_delta,
        );
        // SAFETY: as above.
        unsafe {
            self.gl.bind_framebuffer(glow::FRAMEBUFFER, None);
        }
        self.sc.release_image().map_err(|e| e.to_string())?;
        self.ready = true;
        Ok(())
    }
}

impl Drop for Osd {
    fn drop(&mut self) {
        self.painter.destroy();
        // SAFETY: as in draw().
        unsafe { self.gl.delete_framebuffer(self.fbo) };
    }
}
