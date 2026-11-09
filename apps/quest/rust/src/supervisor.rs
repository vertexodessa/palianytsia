//! Keeps one `pal_android::Link` running on whatever cards are attached
//! (up to `MAX_CARDS`; mabur's multi-card GS: all receive, the channel core
//! scouts on one): waits for the decoder window and the cards, starts the
//! link, watches it, and starts over 3 s after a card leaves or the core
//! gives up (spec 5.5 USB robustness), at once when a card is added. XR session state never reaches here: with the
//! headset lifted the link and the flight log carry on.
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use pal_android::{clear_err, last_error, Cards, Link, LinkCfg, Ports};
use runtime::Shared;

use crate::usb::{Open, Usb};

/// What the OSD needs from a freshly started link.
/// How the supervisor goes on after a link ends.
#[derive(Clone, Copy)]
enum Reopen {
    /// A start-time setting changed: a new link on the cards still open.
    SameCards,
    /// The cards stay attached (a card added): reopen at once.
    Now,
    /// Something went away or failed: wait RETRY first.
    AfterRetry,
}

pub struct LinkUi {
    pub ports: Ports,
    pub shared: Arc<Shared>,
    pub info: pal_android::LinkInfo,
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
    /// The OSD changed a start-time setting (width, channel): restart the
    /// link at once with the new prefs.
    pub relink: AtomicBool,
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
/// Two cards is what the Quest's port powers (a hub with two 8812s).
const MAX_CARDS: usize = 2;
/// With one card ready and another still waiting for its permission
/// dialog, start without it after this long.
const PERM_WAIT: Duration = Duration::from_secs(20);

fn ids(opens: &[Open]) -> String {
    opens
        .iter()
        .map(|o| o.id.as_str())
        .collect::<Vec<_>>()
        .join(" + ")
}

pub fn run(hub: Arc<Hub>, act: usize, vm: usize, toml: String, log_dir: PathBuf, prefs: PathBuf) {
    // SAFETY: `vm` is the process's JavaVM, read on the XR thread before
    // that thread swaps ndk_context.
    let vm = match unsafe { jni::JavaVM::from_raw(vm as *mut _) } {
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
    // Since when a card has been waiting for its permission dialog.
    let mut perm_since: Option<std::time::Instant> = None;
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
        // Which cards are there and which may be opened.
        let scan = env.with_local_frame(32, |env| {
            let mut devs = usb.cards(env)?;
            devs.truncate(MAX_CARDS);
            let mut ready = Vec::new();
            let mut waiting = Vec::new();
            for d in devs {
                if usb.permitted(env, &d.name, &mut asked)? {
                    ready.push(d);
                } else {
                    waiting.push(d);
                }
            }
            Ok::<_, jni::errors::Error>((ready, waiting))
        });
        let (ready, waiting) = match scan {
            Err(e) => {
                let _ = env.exception_clear();
                hub.set_status(format!("USB scan failed: {e}"));
                nap(RETRY);
                continue;
            }
            Ok(x) => x,
        };
        if ready.is_empty() && waiting.is_empty() {
            perm_since = None;
            hub.set_status("no RTL8812 card");
            nap(RETRY);
            continue;
        }
        if !waiting.is_empty() {
            let t0 = *perm_since.get_or_insert_with(std::time::Instant::now);
            let w = waiting.iter().map(|d| d.id.as_str()).collect::<Vec<_>>();
            if ready.is_empty() || t0.elapsed() < PERM_WAIT {
                hub.set_status(format!(
                    "card {}: waiting for USB permission ({} ready)",
                    w.join(", "),
                    ready.len()
                ));
                nap(Duration::from_secs(1));
                continue;
            }
            log::warn!("starting without {}: no USB permission", w.join(", "));
        } else {
            perm_since = None;
        }
        let mut opens = Vec::new();
        for d in &ready {
            match env.with_local_frame(32, |env| usb.open(env, d)) {
                Ok(Ok(o)) => opens.push(o),
                Ok(Err(e)) => log::warn!("usb: {e}"),
                Err(e) => {
                    let _ = env.exception_clear();
                    log::warn!("usb: open {}: {e}", d.id);
                }
            }
        }
        if opens.is_empty() {
            hub.set_status("card would not open; retrying");
            nap(RETRY);
            continue;
        }
        clear_err();
        hub.set_status(format!("card {} up, starting", ids(&opens)));
        let fds: Vec<i32> = opens.iter().map(|o| o.fd).collect();
        // The bring-up (seconds per card) runs once per attach: a restart
        // for new settings runs on the cards still open.
        let set = match pal_android::open_cards(&fds, &toml, pal_android::link_pick(&prefs)) {
            Ok(s) => s,
            Err(e) => {
                for o in opens {
                    Usb::close(&mut env, o);
                }
                hub.set_status(format!("start failed: {e}; retrying"));
                nap(RETRY);
                continue;
            }
        };
        log::info!(
            "supervisor: {} card(s) open: {} ({})",
            opens.len(),
            ids(&opens),
            set.chips()
        );
        let (why, reopen) = loop {
            let cfg = LinkCfg {
                cards: Cards::Open(set.clone()),
                toml: toml.clone(),
                log_dir: log_dir.clone(),
                pick: pal_android::link_pick(&prefs),
            };
            let (link, ports) = match Link::start(cfg, win.clone()) {
                Ok(x) => x,
                Err(e) => break (format!("start failed: {e}"), Reopen::AfterRetry),
            };
            *hub.shared.lock().unwrap_or_else(|p| p.into_inner()) = Some(link.shared.clone());
            *hub.ui.lock().unwrap_or_else(|p| p.into_inner()) = Some(LinkUi {
                ports,
                shared: link.shared.clone(),
                info: link.info.clone(),
            });
            hub.set_status("");
            // Each end of the link says, where it is decided, whether the
            // cards are still there to reopen at once.
            let (why, reopen) = loop {
                nap(Duration::from_millis(500));
                if hub.quit.load(Ordering::Relaxed) {
                    break ("app closing".to_string(), Reopen::AfterRetry);
                }
                if hub.relink.swap(false, Ordering::Relaxed) {
                    break ("link settings changed".to_string(), Reopen::SameCards);
                }
                if !link.running() {
                    break (
                        format!("link stopped: {}", last_error()),
                        Reopen::AfterRetry,
                    );
                }
                let check = env.with_local_frame(32, |env| {
                    for o in &opens {
                        if !usb.present(env, &o.name)? {
                            return Ok(Some((
                                format!("card {} detached", o.id),
                                Reopen::AfterRetry,
                            )));
                        }
                    }
                    if opens.len() < MAX_CARDS {
                        for d in usb.cards(env)? {
                            if opens.iter().any(|o| o.name == d.name) {
                                continue;
                            }
                            if usb.permitted(env, &d.name, &mut asked)? {
                                return Ok(Some((format!("card {} added", d.id), Reopen::Now)));
                            }
                        }
                    }
                    Ok::<_, jni::errors::Error>(None)
                });
                match check {
                    Ok(Some(end)) => break end,
                    Ok(None) => {}
                    Err(_) => {
                        let _ = env.exception_clear();
                    }
                }
            };
            log::warn!("supervisor: {why}");
            *hub.shared.lock().unwrap_or_else(|p| p.into_inner()) = None;
            let clean = link.stop();
            if matches!(reopen, Reopen::SameCards) && clean && !hub.quit.load(Ordering::Relaxed) {
                hub.set_status(format!("{why}, restarting"));
                continue;
            }
            // A core thread left wedged may still be on the cards: reopen
            // them from scratch.
            break (why, if clean { reopen } else { Reopen::AfterRetry });
        };
        // The cards close before their fds do.
        drop(set);
        for o in opens {
            Usb::close(&mut env, o);
        }
        match reopen {
            Reopen::Now | Reopen::SameCards => {
                hub.set_status(format!("{why}, restarting"));
                nap(Duration::from_millis(300));
            }
            Reopen::AfterRetry => {
                hub.set_status(format!("{why}; retrying"));
                nap(RETRY);
            }
        }
    }
}
