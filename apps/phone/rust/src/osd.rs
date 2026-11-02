//! The OSD (spec 5.5 last item): pal-osd's egui widgets drawn by Rust into a
//! second, transparent Surface above the video, so the phone shows the same
//! chips as pal-desktop and the Quest can reuse this for its OSD layer.
//! One thread owns the EGL context; it redraws when a stats line arrives
//! (10 Hz) and clears to transparent around the widgets.
use std::sync::atomic::Ordering;
use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use glow::HasContext;
use khronos_egl as egl;
use ndk::native_window::NativeWindow;
use pal_osd::console::{Console, Feeds};
use pal_osd::panel::Panel;
use pal_osd::params::Cmd;
use runtime::Shared;

pub struct OsdIn {
    pub win: NativeWindow,
    /// Android DisplayMetrics.density: egui points to pixels.
    pub density: f32,
    pub stats_rx: Receiver<String>,
    /// The newest stats line, also read by the Java side.
    pub last: Arc<Mutex<String>>,
    pub shared: Arc<Shared>,
    /// The drone's MAVLink packets (CoreIo::on_mav) for the flight HUD.
    pub mav_rx: Receiver<Vec<u8>>,
    /// Touches from the Activity, in Surface pixels.
    pub touch_rx: Receiver<Touch>,
    /// F18 drone settings: table snapshots in, commands out, answers in.
    pub params_rx: Receiver<String>,
    pub cmd_tx: Sender<Cmd>,
    pub set_res_rx: Receiver<(usize, Result<(), String>)>,
    /// Width, channel set and flight log place, for the Link and Logs tabs.
    pub info: pal_android::LinkInfo,
    /// HUD prefs file in the app's files dir.
    pub prefs_path: std::path::PathBuf,
}

/// One MotionEvent: action 0 down, 1 up, 2 move, 3 cancel.
pub struct Touch {
    pub action: i32,
    pub x: f32,
    pub y: f32,
}

/// Touches as egui pointer events (points).
fn touch_events(rx: &Receiver<Touch>, ppp: f32, ev: &mut Vec<egui::Event>) {
    while let Ok(t) = rx.try_recv() {
        let pos = egui::pos2(t.x / ppp, t.y / ppp);
        let button = |pressed| egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed,
            modifiers: egui::Modifiers::default(),
        };
        match t.action {
            0 => {
                ev.push(egui::Event::PointerMoved(pos));
                ev.push(button(true));
            }
            1 => {
                ev.push(egui::Event::PointerMoved(pos));
                ev.push(button(false));
                ev.push(egui::Event::PointerGone);
            }
            2 => ev.push(egui::Event::PointerMoved(pos)),
            _ => ev.push(egui::Event::PointerGone),
        }
    }
}

struct Gl {
    egl: egl::Instance<egl::Static>,
    d: egl::Display,
    s: egl::Surface,
    c: egl::Context,
}

fn make_gl(win: &NativeWindow) -> Result<(Gl, glow::Context), String> {
    let e = egl::Instance::new(egl::Static);
    let d = unsafe { e.get_display(egl::DEFAULT_DISPLAY) }.ok_or("no EGL display")?;
    e.initialize(d).map_err(|x| format!("eglInitialize: {x}"))?;
    let attrs = [
        egl::RENDERABLE_TYPE,
        0x0040, // EGL_OPENGL_ES3_BIT
        egl::SURFACE_TYPE,
        egl::WINDOW_BIT,
        egl::RED_SIZE,
        8,
        egl::GREEN_SIZE,
        8,
        egl::BLUE_SIZE,
        8,
        egl::ALPHA_SIZE,
        8,
        egl::NONE,
    ];
    let cfg = e
        .choose_first_config(d, &attrs)
        .map_err(|x| format!("eglChooseConfig: {x}"))?
        .ok_or("no EGL config with alpha")?;
    let c = e
        .create_context(d, cfg, None, &[egl::CONTEXT_CLIENT_VERSION, 3, egl::NONE])
        .map_err(|x| format!("eglCreateContext: {x}"))?;
    // SAFETY: the window outlives the surface (both live in run()).
    let s = unsafe {
        e.create_window_surface(d, cfg, win.ptr().as_ptr().cast(), None)
            .map_err(|x| format!("eglCreateWindowSurface: {x}"))?
    };
    e.make_current(d, Some(s), Some(s), Some(c))
        .map_err(|x| format!("eglMakeCurrent: {x}"))?;
    // SAFETY: a current GLES3 context on this thread.
    let gl = unsafe {
        glow::Context::from_loader_function(|n| {
            e.get_proc_address(n)
                .map_or(std::ptr::null(), |f| f as *const _)
        })
    };
    Ok((Gl { egl: e, d, s, c }, gl))
}

pub fn run(o: OsdIn) {
    let (g, gl) = match make_gl(&o.win) {
        Ok(x) => x,
        Err(e) => {
            log::error!("osd: {e}");
            // Without an OSD the stats still reach Java.
            while let Ok(s) = o.stats_rx.recv() {
                *o.last.lock().unwrap_or_else(|p| p.into_inner()) = s;
            }
            return;
        }
    };
    let gl = Arc::new(gl);
    let mut painter = match egui_glow::Painter::new(gl.clone(), "", None, false) {
        Ok(p) => p,
        Err(e) => {
            log::error!("osd: painter: {e}");
            return;
        }
    };
    let ctx = egui::Context::default();
    let t0 = Instant::now();
    let mut console = Console::new(
        o.prefs_path.clone(),
        Panel {
            touch: true,
            ..Default::default()
        },
    );
    loop {
        if o.shared.quit.load(Ordering::Relaxed) {
            break;
        }
        // Stats lines pace the redraw at 10 Hz; with the panel open, 30 Hz
        // keeps scrolling and steppers responsive.
        let wait = if console.panel.open { 33 } else { 100 };
        match o.stats_rx.recv_timeout(Duration::from_millis(wait)) {
            Ok(s) => {
                console.on_stats(t0.elapsed().as_secs_f64(), &s);
                *o.last.lock().unwrap_or_else(|p| p.into_inner()) = s;
            }
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => break,
        }
        let now = t0.elapsed().as_secs_f64();
        console.drain(
            now,
            Feeds {
                mav: &o.mav_rx,
                params: &o.params_rx,
                set_res: &o.set_res_rx,
            },
        );
        let frames = o.shared.frames.load(Ordering::Relaxed);
        let fps = console.fps(frames);
        let (w, h) = (o.win.width().max(1) as u32, o.win.height().max(1) as u32);
        let ppp = o.density.max(1.0);
        let mut events = Vec::new();
        touch_events(&o.touch_rx, ppp, &mut events);
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(w as f32 / ppp, h as f32 / ppp),
            )),
            events,
            ..Default::default()
        };
        ctx.set_pixels_per_point(ppp);
        let dec = (frames, o.shared.dec_errors.load(Ordering::Relaxed));
        let mut pout = pal_osd::panel::PanelOut::default();
        let mut stats_flip = false;
        let out = ctx.run(input, |ctx| {
            pal_osd::panel::gear(ctx, &mut console.panel);
            let link_info = format!(
                "{}, key {}",
                o.shared.radio_label(o.info.width),
                console.hist.last().map_or("?", |s| s.key_fp.as_str())
            );
            let log_info = format!(
                "{}\n\nadb pull /sdcard/Android/data/org.palianytsia.phone/files/logs",
                o.info.log_info
            );
            pout = console.show_panel(ctx, link_info, log_info, &o.info.channels);
            egui::CentralPanel::default()
                .frame(egui::Frame::NONE)
                .show(ctx, |ui| {
                    // The video SurfaceView fills the same rect, and
                    // MediaCodec scales the picture to it.
                    // A tap on the bare video toggles link statistics; the
                    // gear and the panel are Areas above, so they win.
                    let bg = ui.interact(ui.max_rect(), egui::Id::new("bg"), egui::Sense::click());
                    if bg.clicked() && !console.panel.open {
                        console.prefs.stats = !console.prefs.stats;
                        stats_flip = true;
                    }
                    let c = &console;
                    let hist = &c.hist;
                    let r = pal_osd::panel::hud_rect(ctx, &c.panel, ui.max_rect());
                    pal_osd::hud::hud(ui, r, ui.max_rect().center(), &c.telem, hist, now, &c.prefs);
                    let stats = c.prefs.stats;
                    let mismatch = hist.last().is_some_and(|s| s.key_mismatch);
                    let searching = o.shared.searching.load(Ordering::Relaxed);
                    if stats || mismatch || searching {
                        let top = r.shrink(8.0).translate(egui::vec2(0.0, 36.0));
                        ui.allocate_new_ui(egui::UiBuilder::new().max_rect(top), |ui| {
                            if searching {
                                ui.label(
                                    egui::RichText::new(o.shared.radio_label(o.info.width))
                                        .size(20.0)
                                        .color(egui::Color32::YELLOW),
                                );
                            }
                            if stats {
                                pal_osd::chips(ui, hist, fps, o.shared.rx_kbps(), dec);
                            }
                            pal_osd::key_banner(ui, hist);
                        });
                    }
                });
        });
        console.settle(&mut pout, Some(&o.cmd_tx));
        if stats_flip && !pout.prefs_changed {
            console.save_prefs();
        }
        if pout.relink {
            // The width applies at link start: end this link; the Activity
            // sees it stop and starts a new one with the saved prefs.
            pal_android::set_err("link settings changed");
            o.shared.quit.store(true, Ordering::Relaxed);
        }
        let prims = ctx.tessellate(out.shapes, out.pixels_per_point);
        unsafe {
            gl.viewport(0, 0, w as i32, h as i32);
        }
        painter.clear([w, h], [0.0, 0.0, 0.0, 0.0]);
        painter.paint_and_update_textures(
            [w, h],
            out.pixels_per_point,
            &prims,
            &out.textures_delta,
        );
        if let Err(e) = g.egl.swap_buffers(g.d, g.s) {
            log::warn!("osd: eglSwapBuffers: {e}");
            break;
        }
    }
    painter.destroy();
    let _ = g.egl.make_current(g.d, None, None, None);
    let _ = g.egl.destroy_surface(g.d, g.s);
    let _ = g.egl.destroy_context(g.d, g.c);
}
