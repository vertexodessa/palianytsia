//! The OSD (spec 5.5 last item): pal-osd's egui widgets drawn by Rust into a
//! second, transparent Surface above the video, so the phone shows the same
//! chips as pal-desktop and the Quest can reuse this for its OSD layer.
//! One thread owns the EGL context; it redraws when a stats line arrives
//! (10 Hz) and clears to transparent around the widgets.
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use glow::HasContext;
use khronos_egl as egl;
use ndk::native_window::NativeWindow;
use pal_osd::{History, Snapshot};
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
    /// Link statistics over the HUD; a tap on the screen toggles it.
    pub show_stats: Arc<AtomicBool>,
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
    let mut hist = History::default();
    let mut telem = pal_osd::telem::Telemetry::default();
    let t0 = Instant::now();
    let (mut fps, mut last_frames, mut fps_t) = (0.0, 0u64, Instant::now());
    loop {
        if o.shared.quit.load(Ordering::Relaxed) {
            break;
        }
        match o.stats_rx.recv_timeout(Duration::from_millis(250)) {
            Ok(s) => {
                if let Some(snap) = Snapshot::parse(&s) {
                    hist.push(t0.elapsed().as_secs_f64(), snap);
                }
                *o.last.lock().unwrap_or_else(|p| p.into_inner()) = s;
            }
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => break,
        }
        let now = t0.elapsed().as_secs_f64();
        while let Ok(p) = o.mav_rx.try_recv() {
            telem.on_packet(&p, now);
        }
        let frames = o.shared.frames.load(Ordering::Relaxed);
        let dt = fps_t.elapsed().as_secs_f64();
        if dt >= 1.0 {
            fps = (frames.saturating_sub(last_frames)) as f64 / dt;
            last_frames = frames;
            fps_t = Instant::now();
        }
        let (w, h) = (o.win.width().max(1) as u32, o.win.height().max(1) as u32);
        let ppp = o.density.max(1.0);
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(w as f32 / ppp, h as f32 / ppp),
            )),
            ..Default::default()
        };
        ctx.set_pixels_per_point(ppp);
        let dec = (frames, o.shared.dec_errors.load(Ordering::Relaxed));
        let out = ctx.run(input, |ctx| {
            egui::CentralPanel::default()
                .frame(egui::Frame::NONE)
                .show(ctx, |ui| {
                    // The video SurfaceView fills the same rect, and
                    // MediaCodec scales the picture to it.
                    let r = ui.max_rect();
                    pal_osd::hud::hud(ui, r, &telem, &hist, now);
                    let stats = o.show_stats.load(Ordering::Relaxed);
                    let mismatch = hist.last().is_some_and(|s| s.key_mismatch);
                    if stats || mismatch {
                        let top = r.shrink(8.0).translate(egui::vec2(0.0, 36.0));
                        ui.allocate_new_ui(egui::UiBuilder::new().max_rect(top), |ui| {
                            if stats {
                                pal_osd::chips(ui, &hist, fps, dec);
                            }
                            pal_osd::key_banner(ui, &hist);
                        });
                    }
                });
        });
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
