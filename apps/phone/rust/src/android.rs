use std::sync::atomic::Ordering;
use std::sync::mpsc::{self, Receiver};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use decode::mediacodec::MediaCodecDecoder;
use decode::Decoder;
use jni::objects::{JClass, JObject, JString};
use jni::sys::{jint, jlong, jstring};
use jni::JNIEnv;
use pal_core::refresh::IdrDriver;
use pal_core::rust_core::{RustCore, RustOpts};
use pal_core::Mode;
use runtime::{CoreLoop, LiveCfg, Shared, Sink};

/// Same rendezvous nonce as the desktop: replays stay comparable.
const RZ_NONCE: u32 = 0x5041_4c31;

static LAST_ERR: Mutex<String> = Mutex::new(String::new());

fn set_err(e: impl Into<String>) {
    let e = e.into();
    log::error!("{e}");
    *LAST_ERR.lock().unwrap_or_else(|p| p.into_inner()) = e;
}

struct Phone {
    shared: Arc<Shared>,
    stats_rx: Receiver<String>,
    last_stats: String,
    core: Option<JoinHandle<()>>,
    dec: Option<JoinHandle<()>>,
    // Kept alive for the core loop's F18 channels.
    _params_rx: Receiver<String>,
    _cmd_tx: mpsc::Sender<pal_osd::params::Cmd>,
    _set_res_rx: Receiver<(usize, Result<(), String>)>,
    _mav_rx: Receiver<Vec<u8>>,
}

fn start(fd: i32, win: ndk::native_window::NativeWindow, toml: &str) -> Result<Phone, String> {
    let cfg = pal_core::config::parse_config_str(toml).map_err(|e| e.to_string())?;
    let ch = cfg
        .radio
        .pin
        .or(cfg.radio.channels.first().copied())
        .ok_or("config: no channel")?;
    let width = cfg.radio.width;
    let dec = MediaCodecDecoder::new(win, 1920, 1080)?;

    let (au_tx, au_rx) = mpsc::sync_channel::<(u64, u64, Vec<u8>)>(8);
    let (stats_tx, stats_rx) = mpsc::sync_channel(4);
    let (params_tx, params_rx) = mpsc::sync_channel(2);
    let (cmd_tx, cmd_rx) = mpsc::channel();
    let (set_res_tx, set_res_rx) = mpsc::sync_channel(8);
    let (mav_tx, mav_rx) = mpsc::sync_channel(256);
    let shared = Arc::new(Shared::default());
    let sink = Sink {
        au_tx,
        mav_tx,
        udp: None,
        card: None,
        log: None,
        now_us: 0,
        pts_off_us: None,
        shared: shared.clone(),
        idr: Some(IdrDriver::default()),
    };
    // RustCore is built on the core thread (GsCore is not Send); a config
    // it refuses still shows up in lastError().
    let mk_cl = move |core: Box<dyn pal_core::GsCore>| CoreLoop {
        core,
        sink,
        stats_tx,
        params_tx,
        cmd_rx,
        set_res_tx,
        last_params_us: 0,
        lat: false,
        last_lat_us: 0,
        rx_full_seen: 0,
        stall_mark: (0, 0),
        last_stats_us: 0,
        last_log_us: 0,
    };

    let sh = shared.clone();
    let dec_t = std::thread::Builder::new()
        .name("pal-decode".into())
        .spawn(move || decode_loop(dec, au_rx, &sh))
        .map_err(|e| e.to_string())?;
    let core_t = std::thread::Builder::new()
        .name("pal-core".into())
        .spawn(move || {
            let opts = RustOpts {
                mode: Mode::Gs,
                adaptive_gap: true,
                rz_nonce: RZ_NONCE,
                channel: ch,
            };
            let cl = match RustCore::new(&cfg, opts) {
                Ok(c) => mk_cl(Box::new(c)),
                Err(e) => return set_err(format!("core: {e}")),
            };
            let lc = LiveCfg {
                select: devourer_sys::Select::Fd(fd),
                ch,
                width,
                gs: true,
            };
            match runtime::run_live(cl, lc, |chip| {
                log::info!("card {chip} on ch {ch} / {width} MHz")
            }) {
                Ok(()) => log::info!("core loop ended"),
                Err(e) => set_err(e),
            }
        })
        .map_err(|e| e.to_string())?;
    Ok(Phone {
        shared,
        stats_rx,
        last_stats: String::new(),
        core: Some(core_t),
        dec: Some(dec_t),
        _params_rx: params_rx,
        _cmd_tx: cmd_tx,
        _set_res_rx: set_res_rx,
        _mav_rx: mav_rx,
    })
}

/// AUs into MediaCodec; decoder errors go back to the IDR driver the same
/// way the desktop's decode thread reports them.
fn decode_loop(mut dec: MediaCodecDecoder, au_rx: Receiver<(u64, u64, Vec<u8>)>, sh: &Shared) {
    let mut errs = 0;
    while let Ok((n, _t, au)) = au_rx.recv() {
        dec.decode(&au);
        let (f, e) = dec.counters();
        sh.frames.store(f, Ordering::Relaxed);
        if e > errs {
            errs = e;
            sh.dec_err_au.store(n, Ordering::Relaxed);
            sh.dec_errors.store(e, Ordering::Release);
        }
    }
    log::info!("decode loop ended ({} stale frames dropped)", dec.stale());
}

fn jstr(env: &mut JNIEnv, s: &str) -> jstring {
    env.new_string(s)
        .map(|j| j.into_raw())
        .unwrap_or(std::ptr::null_mut())
}

#[no_mangle]
pub extern "system" fn Java_org_palianytsia_phone_Native_start(
    mut env: JNIEnv,
    _c: JClass,
    fd: jint,
    surface: JObject,
    toml: JString,
) -> jlong {
    android_logger::init_once(
        android_logger::Config::default()
            .with_max_level(log::LevelFilter::Info)
            .with_tag("palianytsia"),
    );
    let toml: String = match env.get_string(&toml) {
        Ok(s) => s.into(),
        Err(e) => {
            set_err(format!("config string: {e}"));
            return 0;
        }
    };
    // SAFETY: a live android.view.Surface from the Activity's SurfaceHolder.
    let win = unsafe {
        ndk::native_window::NativeWindow::from_surface(
            env.get_raw().cast(),
            surface.as_raw().cast(),
        )
    };
    let Some(win) = win else {
        set_err("ANativeWindow_fromSurface returned null");
        return 0;
    };
    match start(fd, win, &toml) {
        Ok(p) => Box::into_raw(Box::new(p)) as jlong,
        Err(e) => {
            set_err(e);
            0
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_org_palianytsia_phone_Native_stats(
    mut env: JNIEnv,
    _c: JClass,
    h: jlong,
) -> jstring {
    if h == 0 {
        return jstr(&mut env, "");
    }
    // SAFETY: h came from start() and stop() has not run (the Activity
    // calls both on the UI thread).
    let p = unsafe { &mut *(h as *mut Phone) };
    while let Ok(s) = p.stats_rx.try_recv() {
        p.last_stats = s;
    }
    let s = p.last_stats.clone();
    jstr(&mut env, &s)
}

#[no_mangle]
pub extern "system" fn Java_org_palianytsia_phone_Native_lastError(
    mut env: JNIEnv,
    _c: JClass,
) -> jstring {
    let e = LAST_ERR.lock().unwrap_or_else(|p| p.into_inner()).clone();
    jstr(&mut env, &e)
}

#[no_mangle]
pub extern "system" fn Java_org_palianytsia_phone_Native_stop(_env: JNIEnv, _c: JClass, h: jlong) {
    if h == 0 {
        return;
    }
    // SAFETY: as in stats(); ownership returns here exactly once.
    let mut p = unsafe { Box::from_raw(h as *mut Phone) };
    p.shared.quit.store(true, Ordering::Relaxed);
    // Bounded join (spec 5.5: 3 s): a wedged USB run is left behind rather
    // than hanging the UI thread.
    let t0 = Instant::now();
    if let Some(c) = p.core.take() {
        while !c.is_finished() && t0.elapsed() < Duration::from_secs(3) {
            std::thread::sleep(Duration::from_millis(20));
        }
        if c.is_finished() {
            let _ = c.join();
        } else {
            log::warn!("core thread did not stop in 3 s; leaving it");
        }
    }
    if let Some(d) = p.dec.take() {
        if d.is_finished() || p.core.is_none() {
            // The decoder ends when the core loop drops the AU sender.
            let _ = d.join();
        }
    }
}
