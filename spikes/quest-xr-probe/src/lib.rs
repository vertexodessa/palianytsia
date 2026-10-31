//! Quest M0 spike. Logs to logcat under tag `palxr`.
//!
//! Stage 1: what the Quest 3 OpenXR runtime offers (extensions, views).
//! Stage 2: MediaCodec decodes H.265 straight into an XR_KHR_android_surface_swapchain
//! surface shown as a quad layer; no GL copy on our side. Input is an Annex-B file
//! with an AUD per access unit at `<external files dir>/palxr.h265`, fed at 60 fps.
use android_activity::{AndroidApp, MainEvent, PollEvent};
use log::{error, info, warn};
use openxr as xr;
use openxr::sys;
use std::collections::HashMap;
use std::ptr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

const W: u32 = 1920;
const H: u32 = 1080;
const FPS: u64 = 60;

fn probe(entry: &xr::Entry) -> Result<xr::ExtensionSet, String> {
    let exts = entry
        .enumerate_extensions()
        .map_err(|e| format!("enumerate: {e}"))?;
    info!(
        "KEY khr_android_surface_swapchain={} fb_android_surface_swapchain_create={} fb_swapchain_update_state_android_surface={} khr_opengl_es_enable={} fb_display_refresh_rate={}",
        exts.khr_android_surface_swapchain,
        exts.fb_android_surface_swapchain_create,
        exts.fb_swapchain_update_state_android_surface,
        exts.khr_opengl_es_enable,
        exts.fb_display_refresh_rate
    );
    Ok(exts)
}

struct Egl {
    display: *mut std::ffi::c_void,
    config: *mut std::ffi::c_void,
    context: *mut std::ffi::c_void,
}

/// The GLES binding the session requires. We never draw with it: the
/// decoder writes the swapchain surface directly.
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
        display: d.as_ptr(),
        config: cfg.as_ptr(),
        context: ctx.as_ptr(),
    })
}

/// Splits an Annex-B stream at AUD NAL units (type 35).
fn split_aus(d: &[u8]) -> Vec<&[u8]> {
    let mut starts = Vec::new();
    let mut i = 0;
    while i + 4 < d.len() {
        if d[i] == 0 && d[i + 1] == 0 && d[i + 2] == 1 && (d[i + 3] >> 1) & 0x3f == 35 {
            starts.push(if i > 0 && d[i - 1] == 0 { i - 1 } else { i });
            i += 4;
        } else {
            i += 1;
        }
    }
    starts.push(d.len());
    starts.windows(2).map(|w| &d[w[0]..w[1]]).collect()
}

fn pct(v: &mut [u64], p: f64) -> u64 {
    if v.is_empty() {
        return 0;
    }
    v.sort_unstable();
    v[((v.len() - 1) as f64 * p) as usize]
}

/// Decoder thread: feeds AUs at FPS, renders every output buffer to the surface,
/// logs queue-to-output latency once per second.
fn decode_loop(surface: usize, path: String, stop: Arc<AtomicBool>) {
    use ndk::media::media_codec::{
        DequeuedInputBufferResult as In, DequeuedOutputBufferInfoResult as Out, MediaCodec,
        MediaCodecDirection,
    };
    let data = match std::fs::read(&path) {
        Ok(d) => d,
        Err(e) => {
            error!("DEC read {path}: {e}");
            return;
        }
    };
    let aus = split_aus(&data);
    info!("DEC {} AUs from {path}", aus.len());

    let ctx = ndk_context::android_context();
    let vm = unsafe { jni::JavaVM::from_raw(ctx.vm().cast()) }.unwrap();
    let env = vm.attach_current_thread().unwrap();
    let win = unsafe {
        ndk::native_window::NativeWindow::from_surface(env.get_raw().cast(), surface as _)
    };
    let Some(win) = win else {
        error!("DEC ANativeWindow_fromSurface returned null");
        return;
    };
    info!("DEC window {}x{}", win.width(), win.height());

    let codec = MediaCodec::from_decoder_type("video/hevc").expect("no hevc decoder");
    let mut fmt = ndk::media::media_format::MediaFormat::new();
    fmt.set_str("mime", "video/hevc");
    fmt.set_i32("width", W as i32);
    fmt.set_i32("height", H as i32);
    fmt.set_i32("low-latency", 1);
    if let Err(e) = codec.configure(&fmt, Some(&win), MediaCodecDirection::Decoder) {
        error!("DEC configure: {e:?}");
        return;
    }
    codec.start().unwrap();
    info!("DEC started");

    let period = Duration::from_micros(1_000_000 / FPS);
    let t0 = Instant::now();
    let mut next = 0usize;
    let mut queued_at: HashMap<i64, Instant> = HashMap::new();
    let mut lat_us = Vec::new();
    let mut rendered = 0u64;
    let mut win_start = Instant::now();
    while !stop.load(Ordering::Relaxed) {
        let due = t0 + period * next as u32;
        if Instant::now() >= due {
            if let Ok(In::Buffer(mut b)) = codec.dequeue_input_buffer(Duration::ZERO) {
                let au = aus[next % aus.len()];
                let dst = b.buffer_mut();
                let n = au.len().min(dst.len());
                for (d, s) in dst[..n].iter_mut().zip(au) {
                    d.write(*s);
                }
                let pts = (next as u64) * 1_000_000 / FPS;
                queued_at.insert(pts as i64, Instant::now());
                codec.queue_input_buffer(b, 0, n, pts, 0).unwrap();
                next += 1;
            }
        }
        match codec.dequeue_output_buffer(Duration::from_millis(1)) {
            Ok(Out::Buffer(ob)) => {
                let pts = ob.info().presentation_time_us();
                if let Some(t) = queued_at.remove(&pts) {
                    lat_us.push(t.elapsed().as_micros() as u64);
                }
                codec.release_output_buffer(ob, true).unwrap();
                rendered += 1;
            }
            Ok(Out::OutputFormatChanged) => info!("DEC format changed"),
            Ok(_) => {}
            Err(e) => warn!("DEC dequeue_output: {e:?}"),
        }
        if win_start.elapsed() >= Duration::from_secs(1) {
            let secs = win_start.elapsed().as_secs_f64();
            info!(
                "DEC fps={:.1} queued={} dec_lat_us p50={} p95={} max={}",
                rendered as f64 / secs,
                next,
                pct(&mut lat_us, 0.5),
                pct(&mut lat_us, 0.95),
                lat_us.iter().max().copied().unwrap_or(0)
            );
            rendered = 0;
            lat_us.clear();
            win_start = Instant::now();
        }
    }
    let _ = codec.stop();
}

fn run(app: &AndroidApp) -> Result<(), String> {
    let entry = unsafe { xr::Entry::load() }.map_err(|e| format!("load loader: {e}"))?;
    entry
        .initialize_android_loader()
        .map_err(|e| format!("xrInitializeLoaderKHR: {e}"))?;
    let exts = probe(&entry)?;
    if !exts.khr_android_surface_swapchain {
        return Err("no XR_KHR_android_surface_swapchain".into());
    }
    let mut want = xr::ExtensionSet::default();
    want.khr_android_create_instance = true;
    want.khr_android_surface_swapchain = true;
    want.khr_opengl_es_enable = true;
    want.fb_display_refresh_rate = exts.fb_display_refresh_rate;
    let inst = entry
        .create_instance(
            &xr::ApplicationInfo {
                application_name: "palxr-probe",
                application_version: 1,
                engine_name: "palianytsia",
                engine_version: 1,
                api_version: xr::Version::new(1, 0, 0),
            },
            &want,
            &[],
        )
        .map_err(|e| format!("create_instance: {e}"))?;
    let props = inst.properties().map_err(|e| e.to_string())?;
    info!("RUNTIME {} {}", props.runtime_name, props.runtime_version);
    let sys_id = inst
        .system(xr::FormFactor::HEAD_MOUNTED_DISPLAY)
        .map_err(|e| format!("system: {e}"))?;
    let req = inst
        .graphics_requirements::<xr::OpenGlEs>(sys_id)
        .map_err(|e| format!("gles requirements: {e}"))?;
    info!(
        "GLES req min={} max={}",
        req.min_api_version_supported, req.max_api_version_supported
    );
    let egl = make_egl()?;
    let (session, _waiter, _stream) = unsafe {
        inst.create_session::<xr::OpenGlEs>(
            sys_id,
            &xr::opengles::SessionCreateInfo::Android {
                display: egl.display as _,
                config: egl.config as _,
                context: egl.context as _,
            },
        )
    }
    .map_err(|e| format!("create_session: {e}"))?;
    let space = session
        .create_reference_space(xr::ReferenceSpaceType::LOCAL, xr::Posef::IDENTITY)
        .map_err(|e| format!("space: {e}"))?;

    let create = inst
        .exts()
        .khr_android_surface_swapchain
        .as_ref()
        .unwrap()
        .create_swapchain_android_surface;
    // The runtime gives no reason for ERROR_VALIDATION_FAILURE, so try the
    // plausible field combinations and log each verdict.
    let u_sc = sys::SwapchainUsageFlags::SAMPLED | sys::SwapchainUsageFlags::COLOR_ATTACHMENT;
    let variants: [(&str, sys::SwapchainUsageFlags, u32, u32, u32, u32); 6] = [
        ("usage=S|C all1", u_sc, 1, 1, 1, 1),
        ("usage=S all1", sys::SwapchainUsageFlags::SAMPLED, 1, 1, 1, 1),
        ("usage=0 all1", sys::SwapchainUsageFlags::EMPTY, 1, 1, 1, 1),
        ("usage=0 all0", sys::SwapchainUsageFlags::EMPTY, 0, 0, 0, 0),
        ("usage=S|C all0", u_sc, 0, 0, 0, 0),
        ("usage=S|C samp0 rest1", u_sc, 0, 1, 1, 1),
    ];
    let mut sc = sys::Swapchain::NULL;
    let mut surface: sys::platform::jobject = ptr::null_mut();
    for (name, usage, samples, faces, arr, mips) in variants {
        let ci = sys::SwapchainCreateInfo {
            ty: sys::SwapchainCreateInfo::TYPE,
            next: ptr::null(),
            create_flags: sys::SwapchainCreateFlags::EMPTY,
            usage_flags: usage,
            format: 0,
            sample_count: samples,
            width: W,
            height: H,
            face_count: faces,
            array_size: arr,
            mip_count: mips,
        };
        let r = unsafe { create(session.as_raw(), &ci, &mut sc, &mut surface) };
        info!("SURFACE try {name}: {r:?}");
        if r == sys::Result::SUCCESS && !surface.is_null() {
            break;
        }
    }
    if surface.is_null() {
        return Err("xrCreateSwapchainAndroidSurfaceKHR: every variant failed".into());
    }
    info!("SURFACE swapchain ok, surface={surface:p}");

    let path = app
        .external_data_path()
        .map(|p| p.join("palxr.h265").display().to_string())
        .unwrap_or_default();
    let stop = Arc::new(AtomicBool::new(false));
    let surf = surface as usize;
    let dec = {
        let stop = stop.clone();
        std::thread::spawn(move || decode_loop(surf, path, stop))
    };

    let fp = inst.fp();
    let mut ev = xr::EventDataBuffer::new();
    let mut running = false;
    let mut quit = false;
    let mut frames = 0u64;
    let mut t = Instant::now();
    while !quit {
        app.poll_events(Some(Duration::ZERO), |e| {
            if let PollEvent::Main(MainEvent::Destroy) = e {
                quit = true;
            }
        });
        while let Some(e) = inst.poll_event(&mut ev).map_err(|e| e.to_string())? {
            if let xr::Event::SessionStateChanged(s) = e {
                info!("XR state {:?}", s.state());
                match s.state() {
                    xr::SessionState::READY => {
                        session
                            .begin(xr::ViewConfigurationType::PRIMARY_STEREO)
                            .map_err(|e| e.to_string())?;
                        running = true;
                        if exts.fb_display_refresh_rate {
                            let rates = session.enumerate_display_refresh_rates();
                            info!("XR refresh rates {rates:?}");
                            match session.request_display_refresh_rate(120.0) {
                                Ok(()) => info!("XR asked for 120 Hz"),
                                Err(e) => warn!("XR 120 Hz refused: {e}"),
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
        unsafe {
            (fp.wait_frame)(session.as_raw(), &wi, &mut fs);
            (fp.begin_frame)(session.as_raw(), &bi);
        }
        let quad = sys::CompositionLayerQuad {
            ty: sys::CompositionLayerQuad::TYPE,
            next: ptr::null(),
            layer_flags: sys::CompositionLayerFlags::EMPTY,
            space: space.as_raw(),
            eye_visibility: sys::EyeVisibility::BOTH,
            sub_image: sys::SwapchainSubImage {
                swapchain: sc,
                image_rect: sys::Rect2Di {
                    offset: sys::Offset2Di { x: 0, y: 0 },
                    extent: sys::Extent2Di {
                        width: W as i32,
                        height: H as i32,
                    },
                },
                image_array_index: 0,
            },
            pose: sys::Posef {
                orientation: sys::Quaternionf {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
                position: sys::Vector3f {
                    x: 0.0,
                    y: 0.0,
                    z: -2.0,
                },
            },
            size: sys::Extent2Df {
                width: 3.2,
                height: 1.8,
            },
        };
        let layers = [&quad as *const _ as *const sys::CompositionLayerBaseHeader];
        let render = fs.should_render != sys::FALSE;
        let ei = sys::FrameEndInfo {
            ty: sys::FrameEndInfo::TYPE,
            next: ptr::null(),
            display_time: fs.predicted_display_time,
            environment_blend_mode: sys::EnvironmentBlendMode::OPAQUE,
            layer_count: u32::from(render),
            layers: layers.as_ptr(),
        };
        let r = unsafe { (fp.end_frame)(session.as_raw(), &ei) };
        if r != sys::Result::SUCCESS {
            warn!("XR end_frame {r:?}");
        }
        frames += 1;
        if t.elapsed() >= Duration::from_secs(5) {
            info!(
                "XR frames/s={:.1} period_ns={}",
                frames as f64 / t.elapsed().as_secs_f64(),
                fs.predicted_display_period.as_nanos()
            );
            frames = 0;
            t = Instant::now();
        }
    }
    stop.store(true, Ordering::Relaxed);
    let _ = dec.join();
    Ok(())
}

#[no_mangle]
fn android_main(app: AndroidApp) {
    android_logger::init_once(
        android_logger::Config::default()
            .with_max_level(log::LevelFilter::Info)
            .with_tag("palxr"),
    );
    match run(&app) {
        Ok(()) => info!("PROBE OK"),
        Err(e) => error!("PROBE FAIL {e}"),
    }
}
