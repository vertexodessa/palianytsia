//! The OpenXR side, grown from the quest-xr-probe spike (docs/spec.md has
//! its results), whose findings it keeps: the ndk_context Activity swap,
//! the zero fields of the surface swapchain, the vertical flip. Quad
//! layers only: the video, decoded by MediaCodec straight into an
//! XR_KHR_android_surface_swapchain, and the OSD above it. In goggle mode
//! they sit in VIEW space, head-locked like real FPV goggles; in window
//! mode they hang in LOCAL space over passthrough. While the settings menu
//! is open it is a third quad, placed in front of the head when it opens
//! and fixed in the room (LOCAL) in both modes, with the pointer's ray and
//! cursor as two more. Each frame the loop reads the controller, decides
//! what the ray points at and who gets the sticks, and builds the list of
//! quads once. The frame loop never waits for video.
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
use crate::input::{locate, Input};
use crate::osd::{Osd, Surf, UiIn};
use crate::placement::{
    self, Grab, Placement, Pose, Ray, SpaceKind, Sticks, Target, MAX_DIST_M, MIN_DIST_M,
};
use crate::supervisor::Hub;
use pal_osd::prefs::QuestView;

/// Video swapchain size: the decoder scales into it.
const VW: u32 = 1920;
const VH: u32 = 1080;
/// The pointer's ray is this wide and, when it points at nothing, this
/// long; the cursor is a disc this wide on the surface it points at.
const RAY_WIDTH_M: f32 = 0.003;
const RAY_FREE_M: f32 = 2.0;
const CURSOR_M: f32 = 0.012;
/// Full stick deflection scrolls the menu this many points a second.
const SCROLL_PT_S: f32 = 900.0;

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

/// Where the video hangs now and in which space. A window not placed yet
/// shows head-locked: no tracked head at the first frames must not mean no
/// video.
fn video_placement(
    view: QuestView,
    window: Option<Placement>,
    scr: &Screen,
) -> (SpaceKind, Placement) {
    match window.filter(|_| view == QuestView::Window) {
        Some(w) => (SpaceKind::Local, w),
        None => {
            let e = scr.extent(view);
            (SpaceKind::View, Placement::in_view(e.dist_m, e.width_m))
        }
    }
}

/// One quad layer of a frame, as data: where, in which space, which
/// swapchain. The frame loop builds the list once, after input and
/// steering; `Submit` only turns it into the runtime's structs.
#[derive(Clone, Copy)]
struct QuadSpec {
    space: SpaceKind,
    at: Placement,
    sc: sys::Swapchain,
    w: u32,
    h: u32,
    /// The decoder's surface: flipped, opaque, filtered as the pilot chose.
    /// Else an egui surface, blended by its alpha.
    video: bool,
}

/// The OSD quad: centred on the video, SPAN times its size, always.
fn osd_placement(video: &Placement) -> Placement {
    video.scaled(crate::osd::SPAN)
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

/// Until the activity is resumed, focused and has a window; false when it
/// is destroyed first.
fn wait_for_activity(app: &AndroidApp) -> bool {
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
            return false;
        }
    }
    true
}

/// The window mode's placement and grab.
#[derive(Default)]
struct WindowCtl {
    /// None until placed: on launch and on every switch into the mode.
    window: Option<Placement>,
    grab: Option<Grab>,
    grip_was: bool,
}

impl WindowCtl {
    fn reset(&mut self) {
        self.window = None;
        self.grab = None;
    }

    fn place(&mut self, head: Option<Pose>, scr: &Screen) {
        if self.window.is_none() {
            self.window =
                head.map(|h| Placement::in_front_of(h, scr.window.dist_m, scr.window.width_m));
        }
    }

    /// Grip moves the window; `aim` is the ray in LOCAL, where it hangs.
    fn step(&mut self, grip: bool, aim: Option<Ray>, head: Option<Pose>, scr: &mut Screen) {
        if let (Some(w), Some(h)) = (self.window.as_mut(), head) {
            if grip && !self.grip_was {
                self.grab = aim.and_then(|r| Grab::start(&r, w));
            }
            if let (Some(g), Some(r)) = (&self.grab, &aim) {
                w.pose = g.moved(r, h.pos);
            }
            if !grip && self.grab.take().is_some() {
                scr.window.dist_m = placement::distance(w.pose.pos, h.pos);
                scr.dirty = true;
            }
        }
        self.grip_was = grip;
    }

    fn carried(&self) -> bool {
        self.grab.is_some()
    }

    /// After the sticks moved the screen's extent.
    fn follow(&mut self, before: f32, scr: &Screen) {
        if let Some(w) = self.window.as_mut() {
            w.width_m = scr.window.width_m;
            w.pushed(before - scr.window.dist_m);
        }
    }
}

/// One frame's layers: the structs and their chains must stay alive
/// across xrEndFrame, so one function builds them and makes the call.
struct Submit<'a> {
    fp: &'a xr::raw::Instance,
    session: sys::Session,
    exts: &'a xr::ExtensionSet,
    space: &'a xr::Space,
    local: &'a xr::Space,
    background: Option<&'a Background>,
    /// The operator's filter for the video quad (Screen tab).
    filter: pal_osd::prefs::VideoFilter,
    /// Back to front.
    quads: &'a [QuadSpec],
}

impl Submit<'_> {
    fn end_frame(&self, fs: &sys::FrameState) {
        // The filter is chained after the flip.
        use pal_osd::prefs::VideoFilter;
        use sys::CompositionLayerSettingsFlagsFB as LF;
        let filter = self.filter;
        let settings = sys::CompositionLayerSettingsFB {
            ty: sys::CompositionLayerSettingsFB::TYPE,
            next: ptr::null(),
            layer_flags: match filter {
                VideoFilter::Off => LF::EMPTY,
                VideoFilter::Sharpen => LF::QUALITY_SHARPENING,
                VideoFilter::Supersample => LF::QUALITY_SUPER_SAMPLING,
            },
        };
        let settings_ptr = if self.exts.fb_composition_layer_settings && filter != VideoFilter::Off
        {
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
        let flip_ptr = if self.exts.fb_composition_layer_image_layout {
            &flip as *const _ as *const std::ffi::c_void
        } else {
            settings_ptr
        };
        let quads: Vec<sys::CompositionLayerQuad> = self
            .quads
            .iter()
            .map(|q| {
                let space = match q.space {
                    SpaceKind::Local => self.local.as_raw(),
                    SpaceKind::View => self.space.as_raw(),
                };
                let (flags, next) = if q.video {
                    (sys::CompositionLayerFlags::EMPTY, flip_ptr)
                } else {
                    (
                        sys::CompositionLayerFlags::BLEND_TEXTURE_SOURCE_ALPHA,
                        ptr::null(),
                    )
                };
                quad(&q.at, space, q.sc, q.w, q.h, flags, next)
            })
            .collect();
        let room = self.background.map(Background::layer);
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
        let r = unsafe { (self.fp.end_frame)(self.session, &ei) };
        if r != sys::Result::SUCCESS {
            log::warn!("xr end_frame {r:?}");
        }
    }
}

pub fn run(app: &AndroidApp) -> Result<(), String> {
    let toml = asset(app, "gs.toml")?;
    let ext_dir = app.external_data_path().ok_or("no external files dir")?;
    let int_dir = app.internal_data_path().ok_or("no files dir")?;
    let hub = Arc::new(Hub::default());
    let act = app.activity_as_ptr() as usize;
    // Read here: the supervisor must not touch ndk_context, which this
    // thread swaps below.
    let vm = ndk_context::android_context().vm() as usize;
    let sup = {
        let hub = hub.clone();
        let log_dir = ext_dir.join("logs");
        let prefs = int_dir.join("hud.json");
        std::thread::Builder::new()
            .name("pal-supervisor".into())
            .spawn(move || crate::supervisor::run(hub, act, vm, toml, log_dir, prefs))
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
    if !wait_for_activity(app) {
        hub.quit.store(true, Ordering::Relaxed);
        let _ = sup.join();
        return Ok(());
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
    let mut win = WindowCtl::default();
    // The settings menu's quad: Some while it is open. It is placed once,
    // when it opens (`placement::menu_at`), and again only when the runtime
    // recentres LOCAL under it.
    let mut menu: Option<(SpaceKind, Placement)> = None;
    let mut recentred = false;
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
            } else if let xr::Event::ReferenceSpaceChangePending(_) = e {
                recentred = true;
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
            win.reset();
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
        // The head in the room: the window and the menu are placed by it,
        // and the pointer's ray turns to it.
        let head = locate(&space, &local, t);
        if view == QuestView::Window {
            win.place(head, &scr);
        }
        // Input at the predicted display time, so the ray matches the
        // frame it is drawn into.
        let frame = input
            .as_ref()
            .map_or_else(Default::default, |i| i.poll(&session, &space, &local, t));
        if frame.menu {
            menu = match menu {
                Some(_) => None,
                None => Some(placement::menu_at(head, scr.extent(view).dist_m)),
            };
            log::info!("menu {}", if menu.is_some() { "open" } else { "closed" });
        } else if recentred && menu.is_some() {
            menu = Some(placement::menu_at(head, scr.extent(view).dist_m));
        }
        recentred = false;
        let (v_space, video_at) = video_placement(view, win.window, &scr);
        // What the ray points at, front to back.
        let mut targets = Vec::with_capacity(2);
        if let Some((space, at)) = menu {
            targets.push(Target {
                id: Surf::Menu,
                space,
                at,
                size: pal_osd::panel::SURFACE_POINTS,
            });
        }
        targets.push(Target {
            id: Surf::Osd,
            space: v_space,
            at: osd_placement(&video_at),
            size: crate::osd::points().into(),
        });
        let hit = placement::route(&frame.aim, &targets);
        let dt = t_frame.elapsed().as_secs_f32().min(0.1);
        t_frame = Instant::now();
        let window = (view == QuestView::Window).then(|| {
            win.step(frame.grip, frame.aim.local, head, &mut scr);
            let aimed = frame
                .aim
                .in_space(v_space)
                .is_some_and(|r| video_at.hit(&r).is_some());
            (aimed, win.carried())
        });
        win.grip_was = frame.grip;
        let mut scroll = 0.0;
        let (size, dist) =
            match placement::sticks(menu.is_some(), window, frame.right_y, frame.left_y) {
                Sticks::Screen { size, dist } => (size, dist),
                Sticks::Scroll(y) => {
                    scroll = y * SCROLL_PT_S * dt;
                    (0.0, 0.0)
                }
                Sticks::Idle => (0.0, 0.0),
            };
        let before = scr.extent(view).dist_m;
        scr.steer(view, size, dist, dt);
        if view == QuestView::Window {
            win.follow(before, &scr);
        }
        if let Some(o) = osd.as_mut() {
            let out = o.tick(
                &hub,
                UiIn {
                    menu_open: menu.is_some(),
                    stats: frame.stats,
                    click: frame.click,
                    on: hit.map(|h| (h.id, egui::pos2(h.pos[0], h.pos[1]))),
                    scroll,
                },
            );
            if out.close {
                menu = None;
                log::info!("menu closed");
            }
            if let (true, Some(i)) = (out.pulse, input.as_ref()) {
                i.pulse(&session);
            }
        }
        // SAFETY: as above.
        unsafe {
            (fp.begin_frame)(session.as_raw(), &bi);
        }
        // The frame's quads, back to front, after its grab and sticks.
        let (v_space, video_at) = video_placement(view, win.window, &scr);
        let mut quads = Vec::with_capacity(5);
        let egui_quad = |s: &crate::surface::Surface, space, at| QuadSpec {
            space,
            at,
            sc: s.sc.as_raw(),
            w: s.w,
            h: s.h,
            video: false,
        };
        if let Some(sc) = video {
            quads.push(QuadSpec {
                space: v_space,
                at: video_at,
                sc,
                w: VW,
                h: VH,
                video: true,
            });
        }
        if let Some(o) = osd.as_ref() {
            if o.hud.ready {
                quads.push(egui_quad(&o.hud, v_space, osd_placement(&video_at)));
            }
            if let Some((m_space, m_at)) = menu {
                if o.menu.ready {
                    quads.push(egui_quad(&o.menu, m_space, m_at));
                }
                // The pointer, from this frame's aim: its quads move at
                // display rate whatever the menu's own redraw rate.
                if let (Some(ray), Some(h)) = (frame.aim.local, head) {
                    let reach = hit.map_or(RAY_FREE_M, |x| x.reach);
                    if let Some(at) = placement::ray_quad(&ray, reach, h.pos, RAY_WIDTH_M) {
                        quads.push(egui_quad(&o.ray, SpaceKind::Local, at));
                    }
                }
                let on = hit.and_then(|x| Some((x, targets.iter().find(|t| t.id == x.id)?)));
                if let Some((x, tg)) = on {
                    let at = placement::cursor_quad(&tg.at, x.point, CURSOR_M);
                    quads.push(egui_quad(&o.cursor, tg.space, at));
                }
            }
        }
        Submit {
            fp,
            session: session.as_raw(),
            exts: &exts,
            space: &space,
            local: &local,
            background: background.as_ref(),
            filter: osd
                .as_ref()
                .map_or(Default::default(), |o| o.video_filter()),
            quads: &quads,
        }
        .end_frame(&fs);
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
    pal_android::join_within(sup, Duration::from_secs(5));
    // SAFETY: nothing draws any more.
    unsafe { gl.finish() };
    Ok(())
}
