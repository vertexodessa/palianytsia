use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use jni::objects::{JClass, JObject, JString};
use jni::sys::{jint, jlong, jstring};
use jni::JNIEnv;
use pal_android::{clear_err, last_error, set_err, Link, LinkCfg};

struct Phone {
    link: Option<Link>,
    fd: i32,
    /// The card, opened by the first link's core thread and kept across
    /// the relinks a settings change asks for (the bring-up takes seconds).
    cards: Arc<Mutex<Option<Arc<runtime::CardSet>>>>,
    args: Args,
    /// Newest stats line, written by the OSD thread.
    last: Arc<Mutex<String>>,
    osd: Option<JoinHandle<()>>,
    /// Touches from the Activity (pixels) into the OSD thread's egui.
    touch_tx: mpsc::Sender<crate::osd::Touch>,
    /// Set by the OSD when the operator changed a start-time setting.
    relink: Arc<AtomicBool>,
    /// Panel tab carried from one OSD to the next across a relink.
    reopen: Arc<Mutex<Option<pal_osd::panel::Tab>>>,
}

/// What every link on this connection starts with.
#[derive(Clone)]
struct Args {
    win: ndk::native_window::NativeWindow,
    osd_win: ndk::native_window::NativeWindow,
    density: f32,
    toml: String,
    prefs_path: std::path::PathBuf,
    log_dir: std::path::PathBuf,
}

fn start(fd: i32, args: Args) -> Result<Phone, String> {
    let mut p = Phone {
        link: None,
        fd,
        cards: Arc::default(),
        args,
        last: Arc::new(Mutex::new(String::new())),
        osd: None,
        touch_tx: mpsc::channel().0,
        relink: Arc::new(AtomicBool::new(false)),
        reopen: Arc::default(),
    };
    p.launch()?;
    Ok(p)
}

impl Phone {
    /// A link on the open card, with its OSD thread.
    fn launch(&mut self) -> Result<(), String> {
        let a = &self.args;
        let (link, ports) = Link::start(
            LinkCfg {
                cards: pal_android::Cards::Kept {
                    fds: vec![self.fd],
                    slot: self.cards.clone(),
                },
                toml: a.toml.clone(),
                log_dir: a.log_dir.clone(),
                pick: pal_android::link_pick(&a.prefs_path),
            },
            a.win.clone(),
        )?;
        let (touch_tx, touch_rx) = mpsc::channel();
        let osd_in = crate::osd::OsdIn {
            mav_rx: ports.mav_rx,
            touch_rx,
            params_rx: ports.params_rx,
            cmd_tx: ports.cmd_tx,
            set_res_rx: ports.set_res_rx,
            prefs_path: a.prefs_path.clone(),
            info: link.info.clone(),
            win: a.osd_win.clone(),
            density: a.density,
            stats_rx: ports.stats_rx,
            last: self.last.clone(),
            shared: link.shared.clone(),
            relink: self.relink.clone(),
            reopen: self.reopen.clone(),
        };
        let osd_t = match std::thread::Builder::new()
            .name("pal-osd".into())
            .spawn(move || crate::osd::run(osd_in))
        {
            Ok(t) => t,
            Err(e) => {
                link.stop();
                return Err(e.to_string());
            }
        };
        self.link = Some(link);
        self.osd = Some(osd_t);
        self.touch_tx = touch_tx;
        Ok(())
    }

    /// Ends the link and its OSD thread; false when the core thread did not
    /// finish in time (it may still hold the card).
    fn halt(&mut self) -> bool {
        if let Some(l) = self.link.as_ref() {
            l.request_quit();
        }
        // The OSD thread draws into a Surface the Activity may be about to
        // destroy: it checks quit every 100 ms at most.
        if let Some(o) = self.osd.take() {
            let t1 = Instant::now();
            while !o.is_finished() && t1.elapsed() < Duration::from_secs(1) {
                std::thread::sleep(Duration::from_millis(10));
            }
            if o.is_finished() {
                let _ = o.join();
            }
        }
        self.link.take().is_none_or(Link::stop)
    }
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
    osd: JObject,
    density: jni::sys::jfloat,
    toml: JString,
    files_dir: JString,
    log_dir: JString,
) -> jlong {
    android_logger::init_once(
        android_logger::Config::default()
            .with_max_level(log::LevelFilter::Info)
            .with_tag("palianytsia"),
    );
    // A retry starts clean: the previous run's error is no longer news.
    clear_err();
    let toml: String = match env.get_string(&toml) {
        Ok(s) => s.into(),
        Err(e) => {
            set_err(format!("config string: {e}"));
            return 0;
        }
    };
    let dir: String = env
        .get_string(&files_dir)
        .map(Into::into)
        .unwrap_or_default();
    let prefs_path = std::path::Path::new(&dir).join("hud.json");
    let log_dir: String = env.get_string(&log_dir).map(Into::into).unwrap_or_default();
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
    // SAFETY: as above, the transparent OSD SurfaceView's Surface.
    let osd_win = unsafe {
        ndk::native_window::NativeWindow::from_surface(env.get_raw().cast(), osd.as_raw().cast())
    };
    let Some(osd_win) = osd_win else {
        set_err("OSD ANativeWindow_fromSurface returned null");
        return 0;
    };
    let args = Args {
        win,
        osd_win,
        density,
        toml,
        prefs_path,
        log_dir: log_dir.into(),
    };
    match start(fd, args) {
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
    let p = unsafe { &*(h as *mut Phone) };
    let s = p.last.lock().unwrap_or_else(|e| e.into_inner()).clone();
    jstr(&mut env, &s)
}

#[no_mangle]
pub extern "system" fn Java_org_palianytsia_phone_Native_touch(
    _env: JNIEnv,
    _c: JClass,
    h: jlong,
    action: jint,
    x: jni::sys::jfloat,
    y: jni::sys::jfloat,
) {
    if h == 0 {
        return;
    }
    // SAFETY: as in stats().
    let p = unsafe { &*(h as *mut Phone) };
    let _ = p.touch_tx.send(crate::osd::Touch { action, x, y });
}

/// False once the core thread has returned (card gone, USB error): the
/// Activity then stops this handle and starts over.
#[no_mangle]
pub extern "system" fn Java_org_palianytsia_phone_Native_running(
    _env: JNIEnv,
    _c: JClass,
    h: jlong,
) -> jni::sys::jboolean {
    if h == 0 {
        return 0;
    }
    // SAFETY: as in stats().
    let p = unsafe { &*(h as *mut Phone) };
    u8::from(p.link.as_ref().is_some_and(Link::running))
}

/// After `running` turned false: when the operator's settings ended the
/// link, starts the next one on the same open card and returns true. False
/// otherwise (a USB error, a wedged core): the Activity stops and starts over.
#[no_mangle]
pub extern "system" fn Java_org_palianytsia_phone_Native_relink(
    _env: JNIEnv,
    _c: JClass,
    h: jlong,
) -> jni::sys::jboolean {
    if h == 0 {
        return 0;
    }
    // SAFETY: as in stats(); the UI thread is the only caller.
    let p = unsafe { &mut *(h as *mut Phone) };
    if !p.relink.swap(false, Ordering::Relaxed) {
        return 0;
    }
    let t0 = Instant::now();
    if !p.halt() {
        set_err("link settings changed; the old link did not stop");
        return 0;
    }
    clear_err();
    match p.launch() {
        Ok(()) => {
            log::info!("relink on the open card in {} ms", t0.elapsed().as_millis());
            1
        }
        Err(e) => {
            set_err(e);
            0
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_org_palianytsia_phone_Native_lastError(
    mut env: JNIEnv,
    _c: JClass,
) -> jstring {
    jstr(&mut env, &last_error())
}

#[no_mangle]
pub extern "system" fn Java_org_palianytsia_phone_Native_stop(_env: JNIEnv, _c: JClass, h: jlong) {
    if h == 0 {
        return;
    }
    // SAFETY: as in stats(); ownership returns here exactly once.
    let mut p = unsafe { Box::from_raw(h as *mut Phone) };
    p.halt();
}
