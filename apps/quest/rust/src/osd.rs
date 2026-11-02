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
use pal_osd::panel::{Panel, PanelIn};
use pal_osd::params::{Edits, Table};
use pal_osd::prefs::HudPrefs;
use pal_osd::{History, Snapshot};

use crate::input::Frame;
use crate::supervisor::{Hub, LinkUi};

/// Swapchain size in pixels and egui points per pixel: 1280x720 points,
/// so pal-osd's sizes match a 720p screen.
pub const W: u32 = 1920;
pub const H: u32 = 1080;
const PPP: f32 = 1.5;

/// The OSD's size in egui points.
pub fn points() -> egui::Vec2 {
    egui::vec2(W as f32 / PPP, H as f32 / PPP)
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
    hist: History,
    telem: pal_osd::telem::Telemetry,
    prefs: HudPrefs,
    prefs_path: PathBuf,
    panel: Panel,
    table: Option<Table>,
    edits: Edits,
    /// Last pointer position, for the cursor and the release event.
    pointer: Option<egui::Pos2>,
    t0: Instant,
    last_draw: Option<Instant>,
    fps: (f64, u64, Instant),
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
            ctx: egui::Context::default(),
            fbo,
            sc,
            images,
            ready: false,
            link: None,
            hist: History::default(),
            telem: Default::default(),
            prefs: HudPrefs::load(&prefs_path),
            prefs_path,
            panel: Panel {
                // Finger-sized rows suit a hand-held ray too.
                touch: true,
                ..Default::default()
            },
            table: None,
            edits: Edits::default(),
            pointer: None,
            t0: Instant::now(),
            last_draw: None,
            fps: (0.0, 0, Instant::now()),
        })
    }

    /// Drains the link's channels and applies the controller frame;
    /// redraws when something changed or the last draw is old.
    pub fn tick(&mut self, hub: &Hub, inp: Frame) {
        let mut changed = false;
        if inp.menu {
            self.panel.open = !self.panel.open;
            changed = true;
        }
        if inp.stats {
            self.prefs.stats = !self.prefs.stats;
            self.prefs.save(&self.prefs_path);
            changed = true;
        }
        if let Some(l) = hub.ui.lock().unwrap_or_else(|p| p.into_inner()).take() {
            log::info!(
                "osd: link {}, {}",
                l.shared.radio_label(l.info.width),
                l.info.log_info
            );
            // A new link is a new session: fresh history.
            self.hist = History::default();
            self.telem = Default::default();
            self.link = Some(l);
        }
        let now = self.t0.elapsed().as_secs_f64();
        let mut fresh = false;
        if let Some(l) = &self.link {
            while let Ok(s) = l.ports.stats_rx.try_recv() {
                if let Some(snap) = Snapshot::parse(&s) {
                    self.hist.push(now, snap);
                    fresh = true;
                }
            }
            while let Ok(p) = l.ports.mav_rx.try_recv() {
                self.telem.on_packet(&p, now);
            }
            while let Ok(t) = l.ports.params_rx.try_recv() {
                if let Some(t) = Table::parse(&t) {
                    // Row state changes into logcat: a set that "hangs"
                    // shows here whether the drone's answer arrived.
                    for (i, r) in t.rows.iter().enumerate() {
                        let old = self.table.as_ref().and_then(|o| o.rows.get(i));
                        if old.is_none_or(|o| o.state != r.state || o.value != r.value) {
                            log::info!(
                                "param {} = {:?}: {} {}",
                                r.id,
                                r.value.as_deref().unwrap_or("-"),
                                r.state,
                                r.status
                            );
                        }
                    }
                    self.table = Some(t);
                }
            }
            while let Ok(r) = l.ports.set_res_rx.try_recv() {
                self.edits.last = Some(r);
            }
            let frames = l.shared.frames.load(Ordering::Relaxed);
            let dt = self.fps.2.elapsed().as_secs_f64();
            if dt >= 1.0 {
                self.fps = (
                    frames.saturating_sub(self.fps.1) as f64 / dt,
                    frames,
                    Instant::now(),
                );
            }
        }
        // Pointer events only while the panel is open: in flight the ray
        // and the trigger do nothing.
        let mut events = Vec::new();
        if self.panel.open {
            let button = |pos, pressed| egui::Event::PointerButton {
                pos,
                button: egui::PointerButton::Primary,
                pressed,
                modifiers: egui::Modifiers::default(),
            };
            match (inp.pointer, self.pointer) {
                (Some(p), _) => {
                    events.push(egui::Event::PointerMoved(p));
                    if let Some(pressed) = inp.click {
                        events.push(button(p, pressed));
                    }
                }
                (None, Some(last)) => {
                    // Off the quad: finish a held click where it was.
                    if inp.click == Some(false) {
                        events.push(button(last, false));
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
        let every = if self.panel.open { 33 } else { 250 };
        let due = self
            .last_draw
            .is_none_or(|t| t.elapsed() >= Duration::from_millis(every));
        if fresh || due || changed {
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
        let (hist, telem, fps) = (&self.hist, &self.telem, self.fps.0);
        let kbps = self.link.as_ref().map_or(0.0, |l| l.shared.rx_kbps());
        let (panel, edits, prefs, table) = (
            &mut self.panel,
            &mut self.edits,
            &mut self.prefs,
            self.table.as_ref(),
        );
        let link_info = self.link.as_ref().map_or_else(
            || "no link".to_string(),
            |l| {
                format!(
                    "{}, key {}",
                    l.shared.radio_label(l.info.width),
                    hist.last().map_or("?", |s| s.key_fp.as_str())
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
        let cursor = self.pointer.filter(|_| panel.open);
        let mut pout = pal_osd::panel::PanelOut::default();
        let dec = self.link.as_ref().map_or((0, 0), |l| {
            (
                l.shared.frames.load(Ordering::Relaxed),
                l.shared.dec_errors.load(Ordering::Relaxed),
            )
        });
        let out = self.ctx.run(input, |ctx| {
            pout = pal_osd::panel::show(
                ctx,
                panel,
                PanelIn {
                    table,
                    edits,
                    prefs,
                    link_info: link_info.clone(),
                    log_info: log_info.clone(),
                    cal: hist.last().and_then(|s| s.cal.as_ref()),
                    channels,
                },
            );
            let prefs = &*prefs;
            egui::CentralPanel::default()
                .frame(egui::Frame::NONE)
                .show(ctx, |ui| {
                    let r = pal_osd::panel::hud_rect(ctx, panel, ui.max_rect());
                    pal_osd::hud::hud(ui, r, ui.max_rect().center(), telem, hist, now, prefs);
                    let mismatch = hist.last().is_some_and(|s| s.key_mismatch);
                    if prefs.stats || mismatch || !status.is_empty() {
                        let top = r.shrink(8.0).translate(egui::vec2(0.0, 36.0));
                        ui.allocate_new_ui(egui::UiBuilder::new().max_rect(top), |ui| {
                            if !status.is_empty() {
                                ui.label(
                                    egui::RichText::new(status)
                                        .size(20.0)
                                        .color(egui::Color32::from_rgb(240, 190, 60))
                                        .background_color(egui::Color32::from_black_alpha(160)),
                                );
                            }
                            if prefs.stats {
                                pal_osd::chips(ui, hist, fps, kbps, dec);
                            }
                            pal_osd::key_banner(ui, hist);
                        });
                    }
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
        if let Some(c) = pout.cmd {
            if let Some(l) = &self.link {
                let _ = l.ports.cmd_tx.send(c);
            }
        }
        if pout.relink {
            hub.relink.store(true, Ordering::Relaxed);
        }
        if pout.prefs_changed {
            self.prefs.save(&self.prefs_path);
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
            self.gl.viewport(0, 0, W as i32, H as i32);
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
