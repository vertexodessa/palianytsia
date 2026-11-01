use std::path::PathBuf;
use std::sync::atomic::Ordering;
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use decode::mediacodec::MediaCodecDecoder;
use decode::Decoder;
use pal_core::refresh::IdrDriver;
use pal_core::rust_core::{RustCore, RustOpts};
use pal_core::Mode;
use pal_osd::params::Cmd;
use runtime::{CoreLoop, LiveCfg, Shared, Sink};

/// Same rendezvous nonce as the desktop: replays stay comparable.
const RZ_NONCE: u32 = 0x5041_4c31;

static LAST_ERR: Mutex<String> = Mutex::new(String::new());

/// The newest error of any link thread ("" when none since `clear_err`).
pub fn last_error() -> String {
    LAST_ERR.lock().unwrap_or_else(|p| p.into_inner()).clone()
}

pub fn set_err(e: impl Into<String>) {
    let e = e.into();
    log::error!("{e}");
    *LAST_ERR.lock().unwrap_or_else(|p| p.into_inner()) = e;
}

pub fn clear_err() {
    LAST_ERR.lock().unwrap_or_else(|p| p.into_inner()).clear();
}

pub struct LinkCfg {
    /// usbfs fds from UsbDeviceConnection.getFileDescriptor(), one per
    /// card; the first is the link card, the rest receive (and scout) with
    /// it under the channel core.
    pub fds: Vec<i32>,
    pub toml: String,
    /// Flight log root; one `fl-NNNN_*` directory per start.
    pub log_dir: PathBuf,
    /// Video link width (HudPrefs::link_bw): every ladder rung at this bw.
    pub link_bw: u8,
}

/// What the UI side reads and sends while the link runs.
pub struct Ports {
    /// Stats JSON lines at 10 Hz (with the ladder view merged in).
    pub stats_rx: Receiver<String>,
    /// F18 table snapshots, commands, and the core's answers to Set.
    pub params_rx: Receiver<String>,
    pub cmd_tx: Sender<Cmd>,
    pub set_res_rx: Receiver<(usize, Result<(), String>)>,
    /// The drone's MAVLink packets, for the flight HUD.
    pub mav_rx: Receiver<Vec<u8>>,
}

pub struct Link {
    pub shared: Arc<Shared>,
    /// Channel width; the channel itself is `shared.radio_label(width)`,
    /// which follows the drone search.
    pub width: u8,
    /// Where the flight log goes, or why there is none.
    pub log_info: String,
    core: Option<JoinHandle<()>>,
    dec: Option<JoinHandle<()>>,
}

impl Link {
    pub fn start(
        c: LinkCfg,
        win: ndk::native_window::NativeWindow,
    ) -> Result<(Link, Ports), String> {
        let mut c = c;
        match pal_core::config::with_link_bw(&c.toml, c.link_bw) {
            Ok(t) => c.toml = t,
            // A 40 MHz pick on a 20 MHz radio config: fly the config as is.
            Err(e) => log::warn!("link width {} MHz not applied: {e}", c.link_bw),
        }
        let cfg = pal_core::config::parse_config_str(&c.toml).map_err(|e| e.to_string())?;
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
        if let Some(r) = cfg.link.ladder_cfg.ladder.last() {
            shared.video_bw.store(r.bw as u8, Ordering::Relaxed);
        }
        // Same layout as pal-desktop --log-dir (pal log reads it), bodies
        // capped at 256 MB per session so a replay can rerun a bad flight.
        let (log, log_info) = match flightlog::session::Session::create(
            &c.log_dir,
            std::time::SystemTime::now(),
            256 << 20,
        ) {
            Ok(s) => {
                let info = format!("flight log {}", s.dir().display());
                (Some(s), info)
            }
            Err(e) => (
                None,
                format!("flight log off: {}: {e}", c.log_dir.display()),
            ),
        };
        log::info!("{log_info}");
        let sink = Sink {
            au_tx,
            mav_tx,
            udp: None,
            card: None,
            log,
            now_us: 0,
            pts_off_us: None,
            shared: shared.clone(),
            idr: Some(IdrDriver::default()),
        };
        // RustCore is built on the core thread (GsCore is not Send); a
        // config it refuses still shows up in last_error().
        let ports = runtime::LoopPorts {
            stats_tx,
            params_tx,
            cmd_rx,
            set_res_tx,
        };
        let mk_cl = move |core: Box<dyn pal_core::GsCore>| CoreLoop::new(core, sink, ports, false);
        let sh = shared.clone();
        let dec_t = std::thread::Builder::new()
            .name("pal-decode".into())
            .spawn(move || decode_loop(dec, au_rx, &sh))
            .map_err(|e| e.to_string())?;
        let fd = *c.fds.first().ok_or("no card fd")?;
        let extra: Vec<devourer_sys::Select> = c.fds[1..]
            .iter()
            .map(|&f| devourer_sys::Select::Fd(f))
            .collect();
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
                // radio.channel = "auto": the channel core drives the card
                // (finds the drone, moves the link to the cleanest member,
                // hops), in the Rust core that replaces the one above.
                // Pinned: the runtime holds the card on `ch` and transmits.
                let drive = match cfg.radio.pin {
                    None => runtime::Drive::Core {
                        attach: runtime::dev_card::rust_channel_core(cfg.clone(), opts, width),
                        extra,
                    },
                    Some(_) => runtime::Drive::Runtime {
                        gs: true,
                        search: None,
                    },
                };
                let lc = LiveCfg {
                    select: devourer_sys::Select::Fd(fd),
                    ch,
                    width,
                    drive,
                };
                match runtime::run_live(cl, lc, |chip| {
                    log::info!("card {chip} on ch {ch} / {width} MHz")
                }) {
                    Ok(()) => log::info!("core loop ended"),
                    Err(e) => set_err(e),
                }
            })
            .map_err(|e| e.to_string())?;
        Ok((
            Link {
                shared,
                width,
                log_info,
                core: Some(core_t),
                dec: Some(dec_t),
            },
            Ports {
                stats_rx,
                params_rx,
                cmd_tx,
                set_res_rx,
                mav_rx,
            },
        ))
    }

    /// False once the core thread has returned (card gone, USB error).
    pub fn running(&self) -> bool {
        self.core.as_ref().is_some_and(|c| !c.is_finished())
    }

    /// Asks every thread to quit (UI threads that watch `shared.quit` too).
    pub fn request_quit(&self) {
        self.shared.quit.store(true, Ordering::Relaxed);
    }

    /// Bounded stop (spec 5.5: 3 s): a core thread wedged in USB is left
    /// behind rather than hanging the caller, and then so is the decoder,
    /// which only ends when the core loop drops the AU sender.
    pub fn stop(mut self) {
        self.request_quit();
        let t0 = Instant::now();
        let mut core_done = true;
        if let Some(c) = self.core.take() {
            while !c.is_finished() && t0.elapsed() < Duration::from_secs(3) {
                std::thread::sleep(Duration::from_millis(20));
            }
            core_done = c.is_finished();
            if core_done {
                let _ = c.join();
            } else {
                log::warn!("core thread did not stop in 3 s; leaving it");
            }
        }
        if let Some(d) = self.dec.take() {
            if core_done {
                let _ = d.join();
            } else {
                log::warn!("decoder left with the wedged core thread");
            }
        }
    }
}

/// AUs into MediaCodec; decoder errors go back to the IDR driver the same
/// way the desktop's decode thread reports them.
fn decode_loop(mut dec: MediaCodecDecoder, au_rx: Receiver<(u64, u64, Vec<u8>)>, sh: &Shared) {
    let mut errs = 0;
    // Once a second: a line when AUs came in but no frame came out, so a
    // stall on the glass leaves its cause in logcat.
    let (mut t, mut aus, mut f0, mut s0, mut n0) = (Instant::now(), 0u64, 0u64, 0u64, 0u64);
    loop {
        let got = match au_rx.recv_timeout(Duration::from_millis(250)) {
            Ok(x) => Some(x),
            Err(mpsc::RecvTimeoutError::Timeout) => None,
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        };
        if let Some((n, _t, au)) = got {
            aus += 1;
            dec.decode(&au);
            let (_, e) = dec.counters();
            if e > errs {
                errs = e;
                sh.dec_err_au.store(n, Ordering::Relaxed);
                sh.dec_errors.store(e, Ordering::Release);
            }
        }
        let (f, _) = dec.counters();
        sh.frames.store(f, Ordering::Relaxed);
        if t.elapsed() >= Duration::from_secs(1) {
            let (st, ni) = (dec.stale(), dec.no_input());
            // Under 5 AUs is decoder warm-up after start or a heal, not a
            // stall: a stuck second at 90 fps brings ~90.
            if aus >= 5 && f == f0 {
                let drops = sh.au_dropped.load(Ordering::Relaxed);
                log::warn!(
                    "video stalled: {aus} AUs in, 0 frames out; stale {} no_input {} errors {errs} au_drops {drops}",
                    st - s0,
                    ni - n0,
                );
                sh.event(
                    "video_stall",
                    serde_json::json!({"aus_in": aus, "stale": st - s0, "no_input": ni - n0,
                        "dec_errors": errs, "au_drops": drops}),
                );
            } else if aus == 0 && sh.frames.load(Ordering::Relaxed) > 0 {
                log::warn!("video stalled: no AUs from the core for 1 s");
                sh.event("video_stall", serde_json::json!({"aus_in": 0}));
            }
            (t, aus, f0, s0, n0) = (Instant::now(), 0, f, st, ni);
        }
    }
    log::info!("decode loop ended ({} stale frames dropped)", dec.stale());
}
