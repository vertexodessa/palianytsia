//! Keeps one `pal_android::Link` running on whatever card is attached:
//! waits for the decoder window and the card, starts the link, watches it,
//! and starts over 3 s after the card leaves or the core gives up (spec
//! 5.5 USB robustness). XR session state never reaches here: with the
//! headset lifted the link and the flight log carry on.
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use pal_android::{clear_err, last_error, Link, LinkCfg, Ports};
use runtime::Shared;

use crate::usb::{Find, Usb};

/// What the OSD needs from a freshly started link.
pub struct LinkUi {
    pub ports: Ports,
    pub shared: Arc<Shared>,
    pub radio: String,
    pub log_info: String,
}

/// Shared between the supervisor, the XR loop and the OSD.
#[derive(Default)]
pub struct Hub {
    /// The surface swapchain's window, set by the XR loop once it exists.
    pub win: Mutex<Option<ndk::native_window::NativeWindow>>,
    /// A new link's ports, taken by the OSD.
    pub ui: Mutex<Option<LinkUi>>,
    /// The running link's counters ("None" between links).
    pub shared: Mutex<Option<Arc<Shared>>>,
    /// One line for the OSD while there is no link ("" when linked).
    pub status: Mutex<String>,
    pub quit: AtomicBool,
}

impl Hub {
    fn set_status(&self, s: impl Into<String>) {
        let s = s.into();
        let mut cur = self.status.lock().unwrap_or_else(|p| p.into_inner());
        if *cur != s {
            if !s.is_empty() {
                log::info!("{s}");
            }
            *cur = s;
        }
    }
}

const RETRY: Duration = Duration::from_secs(3);

pub fn run(hub: Arc<Hub>, act: usize, toml: String, log_dir: PathBuf) {
    let ctx = ndk_context::android_context();
    // SAFETY: the VM pointer ndk_context holds is the process's JavaVM.
    let vm = match unsafe { jni::JavaVM::from_raw(ctx.vm().cast()) } {
        Ok(v) => v,
        Err(e) => return log::error!("supervisor: VM: {e}"),
    };
    let mut env = match vm.attach_current_thread_permanently() {
        Ok(e) => e,
        Err(e) => return log::error!("supervisor: attach: {e}"),
    };
    // SAFETY: activity_as_ptr is a global ref that lives as long as the app.
    let act = unsafe { jni::objects::JObject::from_raw(act as jni::sys::jobject) };
    let usb = match Usb::new(&mut env, act) {
        Ok(u) => u,
        Err(e) => return log::error!("supervisor: UsbManager: {e}"),
    };
    let mut asked = Vec::new();
    let nap = |d: Duration| {
        let t0 = std::time::Instant::now();
        while t0.elapsed() < d && !hub.quit.load(Ordering::Relaxed) {
            std::thread::sleep(Duration::from_millis(100));
        }
    };
    while !hub.quit.load(Ordering::Relaxed) {
        let win = hub.win.lock().unwrap_or_else(|p| p.into_inner()).clone();
        let Some(win) = win else {
            nap(Duration::from_millis(200));
            continue;
        };
        let found = env.with_local_frame(32, |env| usb.find(env, &mut asked));
        let open = match found {
            Err(e) => {
                let _ = env.exception_clear();
                hub.set_status(format!("USB scan failed: {e}"));
                nap(RETRY);
                continue;
            }
            Ok(Find::None) => {
                hub.set_status("no RTL8812 card");
                nap(RETRY);
                continue;
            }
            Ok(Find::Waiting(id)) => {
                hub.set_status(format!("card {id}: waiting for USB permission"));
                nap(Duration::from_secs(1));
                continue;
            }
            Ok(Find::Ready(o)) => o,
        };
        clear_err();
        hub.set_status(format!("card {} up, starting", open.id));
        let cfg = LinkCfg {
            fd: open.fd,
            toml: toml.clone(),
            log_dir: log_dir.clone(),
        };
        let (link, ports) = match Link::start(cfg, win) {
            Ok(x) => x,
            Err(e) => {
                Usb::close(&mut env, open);
                hub.set_status(format!("start failed: {e}; retrying"));
                nap(RETRY);
                continue;
            }
        };
        *hub.shared.lock().unwrap_or_else(|p| p.into_inner()) = Some(link.shared.clone());
        *hub.ui.lock().unwrap_or_else(|p| p.into_inner()) = Some(LinkUi {
            ports,
            shared: link.shared.clone(),
            radio: link.radio.clone(),
            log_info: link.log_info.clone(),
        });
        hub.set_status("");
        let why = loop {
            nap(Duration::from_millis(500));
            if hub.quit.load(Ordering::Relaxed) {
                break "app closing".to_string();
            }
            if !link.running() {
                break format!("link stopped: {}", last_error());
            }
            match env.with_local_frame(32, |env| usb.present(env, &open.name)) {
                Ok(false) => break "card detached".to_string(),
                Ok(true) => {}
                Err(_) => {
                    let _ = env.exception_clear();
                }
            }
        };
        log::warn!("supervisor: {why}");
        *hub.shared.lock().unwrap_or_else(|p| p.into_inner()) = None;
        link.stop();
        Usb::close(&mut env, open);
        hub.set_status(format!("{why}; retrying"));
        nap(RETRY);
    }
}
