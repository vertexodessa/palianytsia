//! The OpenXR side, grown from spikes/quest-xr-probe (whose findings it
//! keeps: the ndk_context Activity swap, the zero fields of the surface
//! swapchain, the vertical flip). Two quad layers: the video, decoded by
//! MediaCodec straight into an XR_KHR_android_surface_swapchain, and the
//! OSD above it. In goggle mode they sit in VIEW space, head-locked like
//! real FPV goggles; in window mode they hang in LOCAL space over
//! passthrough. The frame loop never waits for video.
use std::ffi::CString;
use std::io::Read;
use std::ptr;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::{Duration, Instant};

use android_activity::{AndroidApp, MainEvent, PollEvent};
use glow::HasContext;
use openxr as xr;
use openxr::sys;

use crate::background::Background;
use crate::input::Input;
use crate::osd::Osd;
use crate::placement::{self, Grab, Placement, Pose, MAX_DIST_M, MIN_DIST_M};
use crate::supervisor::Hub;
use pal_osd::prefs::QuestView;

/// Video swapchain size: the decoder scales into it.
const VW: u32 = 1920;
const VH: u32 = 1080;
/// While the menu is open in window mode, the video part of the OSD quad
/// is at least this wide per metre of distance (50 degrees across): the
/// panel is drawn at a fixed share of the quad, unreadable on a small window.
const MENU_MIN_WIDTH_PER_M: f32 = 0.93;

#[derive(Clone, Copy)]
struct Extent {
    dist_m: f32,
    width_m: f32,
}

/// How big the video is and how far away, per view mode. The goggle
/// default is 2.0 m wide at 1.5 m, about 67 degrees across, the window's
/// 1.2 m, about 44. The thumbsticks change them and screen.json in the
/// files dir keeps them; the window's place in the room is not kept, it
/// starts in front of the pilot every launch.
struct Screen {
    goggle: Extent,
    window: Extent,
    path: std::path::PathBuf,
    /// Changed and not saved yet.
    dirty: bool,
}

impl Screen {
    fn load(path: std::path::PathBuf) -> Self {
        let mut s = Self {
            goggle: Extent {
                dist_m: 1.5,
                width_m: 2.0,
            },
            window: Extent {
                dist_m: 1.5,
                width_m: 1.2,
            },
            path,
            dirty: false,
        };
        if let Ok(t) = std::fs::read_to_string(&s.path) {
            let num = |k: &str| {
                t.split(&format!("\"{k}\":"))
                    .nth(1)
                    .and_then(|r| r.split([',', '}']).next())
                    .and_then(|v| v.trim().parse::<f32>().ok())
            };
            let read = |e: &mut Extent, dist: &str, width: &str| {
                if let Some(d) = num(dist) {
                    e.dist_m = d.clamp(MIN_DIST_M, MAX_DIST_M);
                }
                if let Some(w) = num(width) {
                    e.width_m = w.clamp(0.5, 6.0);
                }
            };
            read(&mut s.goggle, "dist_m", "width_m");
            read(&mut s.window, "window_dist_m", "window_width_m");
        }
        s
    }

    fn extent(&self, view: QuestView) -> Extent {
        match view {
            QuestView::Goggle => self.goggle,
            QuestView::Window => self.window,
        }
    }

    /// Stick Y per second: full deflection changes size or distance by
    /// about 60 % a second, proportional so it feels the same near and far.
    fn steer(&mut self, view: QuestView, size: f32, dist: f32, dt: f32) {
        if size == 0.0 && dist == 0.0 {
            if self.dirty {
                self.dirty = false;
                let j = format!(
                    "{{\"dist_m\":{:.3},\"width_m\":{:.3},\"window_dist_m\":{:.3},\"window_width_m\":{:.3}}}",
                    self.goggle.dist_m,
                    self.goggle.width_m,
                    self.window.dist_m,
                    self.window.width_m
                );
                if let Err(e) = std::fs::write(&self.path, j) {
                    log::warn!("screen.json: {e}");
                }
            }
            return;
        }
        let e = match view {
            QuestView::Goggle => &mut self.goggle,
            QuestView::Window => &mut self.window,
        };
        e.width_m = (e.width_m * (1.0 + 0.6 * size * dt)).clamp(0.5, 6.0);
        e.dist_m = (e.dist_m * (1.0 - 0.6 * dist * dt)).clamp(MIN_DIST_M, MAX_DIST_M);
        self.dirty = true;
    }
}

/// `space` located in `base` at `t`, when the tracker knows it.
fn locate(space: &xr::Space, base: &xr::Space, t: xr::Time) -> Option<Pose> {
    let loc = space.locate(base, t).ok()?;
    let ok = xr::SpaceLocationFlags::POSITION_VALID | xr::SpaceLocationFlags::ORIENTATION_VALID;
    if !loc.location_flags.contains(ok) {
        return None;
    }
    let (p, q) = (loc.pose.position, loc.pose.orientation);
    Some(Pose {
        pos: [p.x, p.y, p.z],
        rot: [q.x, q.y, q.z, q.w],
    })
}

/// Where the video hangs now and whether that is in LOCAL space (else
/// VIEW). A window not placed yet shows head-locked: no tracked head at the
/// first frames must not mean no video.
fn video_placement(view: QuestView, window: Option<Placement>, scr: &Screen) -> (bool, Placement) {
    match window.filter(|_| view == QuestView::Window) {
        Some(w) => (true, w),
        None => {
            let e = scr.extent(view);
            (false, Placement::in_view(e.dist_m, e.width_m))
        }
    }
}

/// The OSD quad, centred on the video.
fn osd_placement(video: &Placement, view: QuestView, menu: bool, scr: &Screen) -> Placement {
    let min = if view == QuestView::Window && menu {
        MENU_MIN_WIDTH_PER_M * scr.window.dist_m / video.width_m
    } else {
        1.0
    };
    video.scaled(crate::osd::SPAN * min.max(1.0))
}

struct Egl {
    egl: khronos_egl::Instance<khronos_egl::Static>,
    display: khronos_egl::Display,
    config: khronos_egl::Config,
    context: khronos_egl::Context,
}

/// The GLES context the session binds to; the OSD draws with it. A tiny
/// pbuffer keeps it current (all real targets are swapchain FBOs).
fn make_egl() -> Result<Egl, String> {
    use khronos_egl as egl;
    let e = egl::Instance::new(egl::Static);
    let d = unsafe { e.get_display(egl::DEFAULT_DISPLAY) }.ok_or("no EGL display")?;
    e.initialize(d).map_err(|x| format!("eglInitialize: {x}"))?;
    let attrs = [
        egl::RENDERABLE_TYPE,
        0x0040, // EGL_OPENGL_ES3_BIT
        egl::SURFACE_TYPE,
        egl::PBUFFER_BIT,
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
        .ok_or("no EGL config")?;
    let ctx = e
        .create_context(d, cfg, None, &[egl::CONTEXT_CLIENT_VERSION, 3, egl::NONE])
        .map_err(|x| format!("eglCreateContext: {x}"))?;
    let pb = e
        .create_pbuffer_surface(d, cfg, &[egl::WIDTH, 16, egl::HEIGHT, 16, egl::NONE])
        .map_err(|x| format!("pbuffer: {x}"))?;
    e.make_current(d, Some(pb), Some(pb), Some(ctx))
        .map_err(|x| format!("eglMakeCurrent: {x}"))?;
    Ok(Egl {
        egl: e,
        display: d,
        config: cfg,
        context: ctx,
    })
}

fn asset(app: &AndroidApp, name: &str) -> Result<String, String> {
    let c = CString::new(name).map_err(|e| e.to_string())?;
    let mut a = app
        .asset_manager()
        .open(&c)
        .ok_or_else(|| format!("asset {name} missing"))?;
    let mut s = String::new();
    a.read_to_string(&mut s)
        .map_err(|e| format!("asset {name}: {e}"))?;
    Ok(s)
}

/// Surface swapchain plus its Android Surface as a NativeWindow.
fn video_swapchain(
    inst: &xr::Instance,
    session: &xr::Session<xr::OpenGlEs>,
    vm: &jni::JavaVM,
) -> Result<(sys::Swapchain, ndk::native_window::NativeWindow), String> {
    let create = inst
        .exts()
        .khr_android_surface_swapchain
        .as_ref()
        .ok_or("no XR_KHR_android_surface_swapchain")?
        .create_swapchain_android_surface;
    // The Meta runtime accepts only usage 0 and zero counts here; any 1
    // is ERROR_VALIDATION_FAILURE with no log (spike, 2026-10-31).
    let ci = sys::SwapchainCreateInfo {
        ty: sys::SwapchainCreateInfo::TYPE,
        next: ptr::null(),
        create_flags: sys::SwapchainCreateFlags::EMPTY,
        usage_flags: sys::SwapchainUsageFlags::EMPTY,
        format: 0,
        sample_count: 0,
        width: VW,
        height: VH,
        face_count: 0,
        array_size: 0,
        mip_count: 0,
    };
    let mut sc = sys::Swapchain::NULL;
    let mut surface: sys::platform::jobject = ptr::null_mut();
    // SAFETY: a valid session handle and out-pointers.
    let r = unsafe { create(session.as_raw(), &ci, &mut sc, &mut surface) };
    if r != sys::Result::SUCCESS || surface.is_null() {
        return Err(format!("xrCreateSwapchainAndroidSurfaceKHR: {r:?}"));
    }
    let env = vm.attach_current_thread().map_err(|e| e.to_string())?;
    // SAFETY: `surface` is the runtime's live android.view.Surface.
    let win = unsafe {
        ndk::native_window::NativeWindow::from_surface(env.get_raw().cast(), surface.cast())
    }
    .ok_or("ANativeWindow_fromSurface returned null")?;
    log::info!("video swapchain {}x{}", win.width(), win.height());
    Ok((sc, win))
}

fn quad(
    at: &Placement,
    space: sys::Space,
    sc: sys::Swapchain,
    w: u32,
    h: u32,
    flags: sys::CompositionLayerFlags,
    next: *const std::ffi::c_void,
) -> sys::CompositionLayerQuad {
    let ([px, py, pz], [qx, qy, qz, qw]) = (at.pose.pos, at.pose.rot);
    sys::CompositionLayerQuad {
        ty: sys::CompositionLayerQuad::TYPE,
        next,
        layer_flags: flags,
        space,
        eye_visibility: sys::EyeVisibility::BOTH,
        sub_image: sys::SwapchainSubImage {
            swapchain: sc,
            image_rect: sys::Rect2Di {
                offset: sys::Offset2Di { x: 0, y: 0 },
                extent: sys::Extent2Di {
                    width: w as i32,
                    height: h as i32,
                },
            },
            image_array_index: 0,
        },
        pose: sys::Posef {
            orientation: sys::Quaternionf {
                x: qx,
                y: qy,
                z: qz,
                w: qw,
            },
            position: sys::Vector3f {
                x: px,
                y: py,
                z: pz,
            },
        },
        size: sys::Extent2Df {
            width: at.width_m,
            height: at.height_m(),
        },
    }
}

/// Consumes NativeActivity's input queue. Unread, the input dispatcher
/// times out on it and the system shows "isn't responding" over the
/// video (seen 2026-11-01). Controllers come through OpenXR actions, not
/// this queue.
fn drain_input(app: &AndroidApp) {
    if let Ok(mut it) = app.input_events_iter() {
        while it.next(|_| android_activity::InputStatus::Unhandled) {}
    }
}

/// The Activity in ndk_context while one `run` lasts. android-activity sets
/// the Application there once per process, and the Activity's global ref
/// dies with its AndroidApp; Quest re-creates the Activity in the same
/// process, and the next `xrInitializeLoaderKHR` aborted on the dead ref
/// (tombstones of 2026-11-01/02: "invalid global reference", "java_object
/// == null", 6 min into the process).
struct ActivityContext {
    vm: *mut std::ffi::c_void,
    app: *mut std::ffi::c_void,
}

impl ActivityContext {
    /// # Safety
    /// `activity` stays a valid global ref until the guard drops.
    unsafe fn swap_in(activity: *mut std::ffi::c_void) -> Self {
        let c = ndk_context::android_context();
        ndk_context::release_android_context();
        ndk_context::initialize_android_context(c.vm(), activity);
        Self {
            vm: c.vm(),
            app: c.context(),
        }
    }
}

impl Drop for ActivityContext {
    fn drop(&mut self) {
        // SAFETY: android-activity's leaked Application global ref.
        unsafe {
            ndk_context::release_android_context();
            ndk_context::initialize_android_context(self.vm, self.app);
        }
    }
}

/// Every extension the runtime offers, by name, into the log: the list in
/// the vault was read from VrDriver.apk strings, which shows what the
/// runtime knows, not what it gives this app.
fn log_extensions(entry: &xr::Entry) {
    let f = entry.fp().enumerate_instance_extension_properties;
    let mut n = 0u32;
    // SAFETY: the two-call idiom; the second call fills `n` entries.
    let names = unsafe {
        if f(ptr::null(), 0, &mut n, ptr::null_mut()) != sys::Result::SUCCESS {
            return;
        }
        let mut v = vec![
            sys::ExtensionProperties {
                ty: sys::ExtensionProperties::TYPE,
                next: ptr::null_mut(),
                extension_name: [0; sys::MAX_EXTENSION_NAME_SIZE],
                extension_version: 0,
            };
            n as usize
        ];
        if f(ptr::null(), n, &mut n, v.as_mut_ptr()) != sys::Result::SUCCESS {
            return;
        }
        v.truncate(n as usize);
        v.iter()
            .map(|p| {
                std::ffi::CStr::from_ptr(p.extension_name.as_ptr())
                    .to_string_lossy()
                    .into_owned()
            })
            .collect::<Vec<_>>()
    };
    log::info!("openxr extensions ({}): {}", names.len(), names.join(" "));
}

pub fn run(app: &AndroidApp) -> Result<(), String> {
    let toml = asset(app, "gs.toml")?;
    let ext_dir = app.external_data_path().ok_or("no external files dir")?;
    let int_dir = app.internal_data_path().ok_or("no files dir")?;
    let hub = Arc::new(Hub::default());
    let act = app.activity_as_ptr() as usize;
    let sup = {
        let hub = hub.clone();
        let log_dir = ext_dir.join("logs");
        let prefs = int_dir.join("hud.json");
        std::thread::Builder::new()
            .name("pal-supervisor".into())
            .spawn(move || crate::supervisor::run(hub, act, toml, log_dir, prefs))
            .map_err(|e| e.to_string())?
    };

    let entry = unsafe { xr::Entry::load() }.map_err(|e| format!("load loader: {e}"))?;
    entry
        .initialize_android_loader()
        .map_err(|e| format!("xrInitializeLoaderKHR: {e}"))?;
    let exts = entry
        .enumerate_extensions()
        .map_err(|e| format!("extensions: {e}"))?;
    log_extensions(&entry);
    if !exts.khr_android_surface_swapchain {
        return Err("runtime lacks XR_KHR_android_surface_swapchain".into());
    }
    // android-activity 0.6 puts the Application into ndk_context and
    // openxr hands that over as applicationActivity; the Meta runtime then
    // holds the session in IDLE for good. Give it the Activity.
    // SAFETY: the activity global ref lives as long as `app`, which outlives
    // this guard; the guard puts the Application back when run returns.
    let _ctx = unsafe { ActivityContext::swap_in(app.activity_as_ptr()) };
    let vm_ptr = ndk_context::android_context().vm();
    // SAFETY: the process's JavaVM.
    let vm = unsafe { jni::JavaVM::from_raw(vm_ptr.cast()) }.map_err(|e| e.to_string())?;
    let mut want = xr::ExtensionSet::default();
    want.khr_android_create_instance = true;
    want.khr_android_surface_swapchain = true;
    want.khr_opengl_es_enable = true;
    want.fb_display_refresh_rate = exts.fb_display_refresh_rate;
    want.fb_composition_layer_image_layout = exts.fb_composition_layer_image_layout;
    want.meta_performance_metrics = exts.meta_performance_metrics;
    want.fb_composition_layer_settings = exts.fb_composition_layer_settings;
    want.fb_passthrough = exts.fb_passthrough;
    log::info!("XR_FB_passthrough: {}", exts.fb_passthrough);
    let inst = entry
        .create_instance(
            &xr::ApplicationInfo {
                application_name: "palianytsia",
                application_version: 1,
                engine_name: "palianytsia",
                engine_version: 1,
                api_version: xr::Version::new(1, 0, 0),
            },
            &want,
            &[],
        )
        .map_err(|e| format!("create_instance: {e}"))?;
    let sys_id = inst
        .system(xr::FormFactor::HEAD_MOUNTED_DISPLAY)
        .map_err(|e| format!("system: {e}"))?;
    inst.graphics_requirements::<xr::OpenGlEs>(sys_id)
        .map_err(|e| format!("gles requirements: {e}"))?;
    // xrCreateSession checks the activity is resumed, focused and has a
    // window; created earlier, the session never leaves IDLE.
    let (mut resumed, mut window, mut focused) = (false, false, false);
    while !(resumed && window && focused) {
        let mut destroy = false;
        drain_input(app);
        app.poll_events(Some(Duration::from_millis(100)), |e| {
            if let PollEvent::Main(m) = &e {
                match m {
                    MainEvent::Resume { .. } => resumed = true,
                    MainEvent::InitWindow { .. } => window = true,
                    MainEvent::Pause => resumed = false,
                    MainEvent::TerminateWindow { .. } => window = false,
                    MainEvent::GainedFocus => focused = true,
                    MainEvent::LostFocus => focused = false,
                    MainEvent::Destroy => destroy = true,
                    _ => {}
                }
            }
        });
        if destroy {
            hub.quit.store(true, Ordering::Relaxed);
            let _ = sup.join();
            return Ok(());
        }
    }
    let egl = make_egl()?;
    let (session, _waiter, _stream) = unsafe {
        inst.create_session::<xr::OpenGlEs>(
            sys_id,
            &xr::opengles::SessionCreateInfo::Android {
                display: egl.display.as_ptr() as _,
                config: egl.config.as_ptr() as _,
                context: egl.context.as_ptr() as _,
            },
        )
    }
    .map_err(|e| format!("create_session: {e}"))?;
    if exts.fb_display_refresh_rate {
        let rates = session.enumerate_display_refresh_rates();
        log::info!("display refresh rates: {rates:?}");
        let top = rates.iter().flatten().copied().reduce(f32::max);
        if top.is_some_and(|t| t <= crate::rate::STOCK_MAX_HZ + 0.5) {
            log::info!(
                "up to 207 Hz cuts video latency ~5 ms: `adb shell setprop debug.oculus.allRefreshRates 1` (again after each reboot), then restart the app"
            );
        }
    }
    // SAFETY: the context made current in make_egl().
    let gl = Arc::new(unsafe {
        glow::Context::from_loader_function(|n| {
            egl.egl
                .get_proc_address(n)
                .map_or(ptr::null(), |f| f as *const _)
        })
    });
    let space = session
        .create_reference_space(xr::ReferenceSpaceType::VIEW, xr::Posef::IDENTITY)
        .map_err(|e| format!("space: {e}"))?;
    let local = session
        .create_reference_space(xr::ReferenceSpaceType::LOCAL, xr::Posef::IDENTITY)
        .map_err(|e| format!("local space: {e}"))?;

    let input = match Input::new(&inst, &session) {
        Ok(i) => Some(i),
        Err(e) => {
            log::error!("controllers: {e}");
            None
        }
    };
    let perf = crate::perf::PerfMetrics::new(&inst, &session);
    let mut scr = Screen::load(int_dir.join("screen.json"));
    let mut view = QuestView::Goggle;
    let mut background: Option<Background> = None;
    // None until placed: on launch and on every switch into the window mode.
    let mut window: Option<Placement> = None;
    let mut grab: Option<Grab> = None;
    let mut grip_was = false;
    let mut t_frame = Instant::now();
    let mut video: Option<sys::Swapchain> = None;
    let mut osd: Option<Osd> = None;
    let fp = inst.fp();
    let mut ev = xr::EventDataBuffer::new();
    let (mut running, mut quit) = (false, false);
    let (mut frames, mut t_log) = (0u64, Instant::now());
    let mut refresh = crate::rate::Refresh::new(Instant::now());
    while !quit {
        drain_input(app);
        app.poll_events(Some(Duration::ZERO), |e| {
            if let PollEvent::Main(MainEvent::Destroy) = &e {
                quit = true;
            }
        });
        while let Some(e) = inst.poll_event(&mut ev).map_err(|e| e.to_string())? {
            if let xr::Event::SessionStateChanged(s) = e {
                log::info!("xr state {:?}", s.state());
                match s.state() {
                    xr::SessionState::READY => {
                        session
                            .begin(xr::ViewConfigurationType::PRIMARY_STEREO)
                            .map_err(|e| e.to_string())?;
                        running = true;
                        // After READY: created earlier, the shell transition
                        // stalled in the spike.
                        if video.is_none() {
                            let (sc, win) = video_swapchain(&inst, &session, &vm)?;
                            *hub.win.lock().unwrap_or_else(|p| p.into_inner()) = Some(win);
                            video = Some(sc);
                        }
                        if osd.is_none() {
                            match Osd::new(&session, gl.clone(), int_dir.join("hud.json")) {
                                Ok(o) => osd = Some(o),
                                Err(e) => log::error!("{e}"),
                            }
                        }
                    }
                    xr::SessionState::STOPPING => {
                        session.end().map_err(|e| e.to_string())?;
                        running = false;
                    }
                    xr::SessionState::EXITING | xr::SessionState::LOSS_PENDING => quit = true,
                    _ => {}
                }
            }
        }
        if !running {
            std::thread::sleep(Duration::from_millis(10));
            continue;
        }
        if exts.fb_display_refresh_rate && refresh.due(Instant::now()) {
            let sh = hub.shared.lock().unwrap_or_else(|p| p.into_inner()).clone();
            let f = sh.as_ref().map_or(0, |s| s.frames.load(Ordering::Relaxed));
            let cam = sh.map_or(0, |s| s.cam_fps_x10.load(Ordering::Relaxed));
            let offered = session
                .enumerate_display_refresh_rates()
                .unwrap_or_default();
            if let Some((r, fps)) = refresh.tick(Instant::now(), f, cam, &offered) {
                match session.request_display_refresh_rate(r) {
                    Ok(()) => log::info!("display {r} Hz for video {fps:.0} fps"),
                    Err(e) => log::warn!("display {r} Hz refused: {e}"),
                }
            }
        }

        let mut fs = sys::FrameState {
            ty: sys::FrameState::TYPE,
            next: ptr::null_mut(),
            predicted_display_time: sys::Time::from_nanos(0),
            predicted_display_period: sys::Duration::from_nanos(0),
            should_render: sys::FALSE,
        };
        let wi = sys::FrameWaitInfo {
            ty: sys::FrameWaitInfo::TYPE,
            next: ptr::null(),
        };
        let bi = sys::FrameBeginInfo {
            ty: sys::FrameBeginInfo::TYPE,
            next: ptr::null(),
        };
        // SAFETY: a running session; the info structs live across the calls.
        unsafe {
            (fp.wait_frame)(session.as_raw(), &wi, &mut fs);
        }
        let wanted = osd.as_ref().map_or(view, |o| o.view_mode());
        if wanted != view {
            view = wanted;
            window = None;
            grab = None;
            background = None;
            if view == QuestView::Window && exts.fb_passthrough {
                match Background::new(&session) {
                    Ok(b) => background = Some(b),
                    Err(e) => log::error!("passthrough: {e}"),
                }
            }
            log::info!(
                "view {view:?}, passthrough {}",
                if background.is_some() { "on" } else { "off" }
            );
        }
        let t = fs.predicted_display_time;
        let head = (view == QuestView::Window)
            .then(|| locate(&space, &local, t))
            .flatten();
        if view == QuestView::Window && window.is_none() {
            window = head.map(|h| Placement::in_front_of(h, scr.window.dist_m, scr.window.width_m));
        }
        let menu = osd.as_ref().is_some_and(Osd::panel_open);
        let (in_local, video_at) = video_placement(view, window, &scr);
        let xr_space = if in_local { &local } else { &space };
        // Input at the predicted display time, so the ray matches the
        // frame it is drawn into.
        let frame = input.as_ref().map_or_else(Default::default, |i| {
            i.poll(
                &session,
                xr_space,
                t,
                &osd_placement(&video_at, view, menu, &scr),
                crate::osd::points(),
            )
        });
        let dt = t_frame.elapsed().as_secs_f32().min(0.1);
        t_frame = Instant::now();
        let (mut size, mut dist) = (frame.size, frame.dist);
        if view == QuestView::Window {
            if let (Some(w), Some(h)) = (window.as_mut(), head) {
                if frame.grip && !grip_was {
                    grab = frame.aim.and_then(|r| Grab::start(&r, w));
                }
                if let (Some(g), Some(r)) = (&grab, &frame.aim) {
                    w.pose = g.moved(r, h.pos);
                }
                if !frame.grip && grab.take().is_some() {
                    scr.window.dist_m = placement::distance(w.pose.pos, h.pos);
                    scr.dirty = true;
                }
            }
            // Only the window under the ray listens: a thumb resting on
            // a stick must not change it unasked in flight.
            let aimed = frame.aim.is_some_and(|r| video_at.hit(&r).is_some());
            if !aimed || grab.is_some() {
                (size, dist) = (0.0, 0.0);
            }
        }
        grip_was = frame.grip;
        let before = scr.extent(view).dist_m;
        scr.steer(view, size, dist, dt);
        if view == QuestView::Window {
            if let Some(w) = window.as_mut() {
                w.width_m = scr.window.width_m;
                w.pushed(before - scr.window.dist_m);
            }
        }
        if let Some(o) = osd.as_mut() {
            o.tick(&hub, frame);
        }
        // SAFETY: as above.
        unsafe {
            (fp.begin_frame)(session.as_raw(), &bi);
        }
        // The operator's filter for the video quad (OSD tab), chained
        // after the flip.
        use pal_osd::prefs::VideoFilter;
        use sys::CompositionLayerSettingsFlagsFB as LF;
        let filter = osd.as_ref().map_or(VideoFilter::Off, |o| o.video_filter());
        let settings = sys::CompositionLayerSettingsFB {
            ty: sys::CompositionLayerSettingsFB::TYPE,
            next: ptr::null(),
            layer_flags: match filter {
                VideoFilter::Off => LF::EMPTY,
                VideoFilter::Sharpen => LF::QUALITY_SHARPENING,
                VideoFilter::Supersample => LF::QUALITY_SUPER_SAMPLING,
            },
        };
        let settings_ptr = if exts.fb_composition_layer_settings && filter != VideoFilter::Off {
            &settings as *const _ as *const std::ffi::c_void
        } else {
            ptr::null()
        };
        // A decoder Surface stores rows top-down; OpenXR samples bottom-up.
        let flip = sys::CompositionLayerImageLayoutFB {
            ty: sys::CompositionLayerImageLayoutFB::TYPE,
            next: settings_ptr as *mut _,
            flags: sys::CompositionLayerImageLayoutFlagsFB::VERTICAL_FLIP,
        };
        let flip_ptr = if exts.fb_composition_layer_image_layout {
            &flip as *const _ as *const std::ffi::c_void
        } else {
            settings_ptr
        };
        // After this frame's grab and sticks.
        let (in_local, video_at) = video_placement(view, window, &scr);
        let layer_space = if in_local {
            local.as_raw()
        } else {
            space.as_raw()
        };
        let mut quads = Vec::with_capacity(2);
        if let Some(sc) = video {
            quads.push(quad(
                &video_at,
                layer_space,
                sc,
                VW,
                VH,
                sys::CompositionLayerFlags::EMPTY,
                flip_ptr,
            ));
        }
        if let Some(o) = osd.as_ref().filter(|o| o.ready) {
            quads.push(quad(
                &osd_placement(&video_at, view, o.panel_open(), &scr),
                layer_space,
                o.sc.as_raw(),
                crate::osd::W,
                crate::osd::H,
                sys::CompositionLayerFlags::BLEND_TEXTURE_SOURCE_ALPHA,
                ptr::null(),
            ));
        }
        let room = background.as_ref().map(Background::layer);
        // The room first: layers are composited in order.
        let layers: Vec<*const sys::CompositionLayerBaseHeader> = room
            .iter()
            .map(|r| r as *const _ as *const sys::CompositionLayerBaseHeader)
            .chain(
                quads
                    .iter()
                    .map(|q| q as *const _ as *const sys::CompositionLayerBaseHeader),
            )
            .collect();
        let render = fs.should_render != sys::FALSE;
        let ei = sys::FrameEndInfo {
            ty: sys::FrameEndInfo::TYPE,
            next: ptr::null(),
            display_time: fs.predicted_display_time,
            environment_blend_mode: sys::EnvironmentBlendMode::OPAQUE,
            layer_count: if render { layers.len() as u32 } else { 0 },
            layers: layers.as_ptr(),
        };
        // SAFETY: the layers and their chained structs outlive the call.
        let r = unsafe { (fp.end_frame)(session.as_raw(), &ei) };
        if r != sys::Result::SUCCESS {
            log::warn!("xr end_frame {r:?}");
        }
        frames += 1;
        if t_log.elapsed() >= Duration::from_secs(10) {
            log::info!(
                "xr {:.1} frames/s, period {} us",
                frames as f64 / t_log.elapsed().as_secs_f64(),
                fs.predicted_display_period.as_nanos() / 1000
            );
            if let Some(p) = &perf {
                log::info!("xr perf: {}", p.sample());
            }
            frames = 0;
            t_log = Instant::now();
        }
    }
    hub.quit.store(true, Ordering::Relaxed);
    drop(osd);
    // The supervisor stops the link (bounded 3 s) before it returns.
    let t0 = Instant::now();
    while !sup.is_finished() && t0.elapsed() < Duration::from_secs(5) {
        std::thread::sleep(Duration::from_millis(20));
    }
    if sup.is_finished() {
        let _ = sup.join();
    }
    // SAFETY: nothing draws any more.
    unsafe { gl.finish() };
    Ok(())
}
