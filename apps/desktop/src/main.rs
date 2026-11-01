//! pal-desktop: the native Linux GS (spec 5.7). Source is a devourer card
//! (`--live`) or a capture replayed at 1x (`--replay bodies.bin`); the core
//! runs on its own thread, AUs go to a decode thread through an 8-slot queue
//! (drop-newest + counter when full), and the UI only reads the newest frame
//! and the newest stats line.
//!
//!   pal-desktop -c gs.toml --replay bodies.bin [--mode gs|spotter] [--loop]
//!   pal-desktop [-c gs.toml] [--live] [--ch N --width 20|40] [--mode gs|spotter] [--card N]
//!     live is the default; config from -c, $PAL_CONFIG, or configs/bench-ch40.toml;
//!     channel/width from its radio section, mode gs
//!   common: [--core rust|ref] [--log-dir DIR] [--snapshot out.ppm --secs N]
//!           [--mav-udp HOST:PORT]   drone MAVLink to a GCS (QGroundControl) and back
//!           [--idr | --no-idr]      GS IDR requests (default: on live, off replay)
//!           [--hwdec auto|vdpau|vaapi|cuda|sw]  decoder (default auto: first hwaccel
//!                                   that opens, else software)
//!           [--params]              open the drone settings window (P toggles; GS only)
//!           [--lat]                 per-second latency breakdown on stderr (printed
//!                                   anyway while a stage is over its threshold)
//!
//! `--snapshot` saves one rendered window after N seconds and quits: the
//! headless check that video and OSD actually draw.
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, SyncSender, TrySendError};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use decode::Decoder;
use flightlog::capture::{self, Record};
use flightlog::session::Session;
use pal_core::refresh::IdrDriver;
use pal_core::rust_core::{RustCore, RustOpts};
use pal_core::{Au, CoreIo, GsCore, Mode, RxBody};
use pal_osd::{History, Snapshot};
use pal_ref::{RefCore, RefOpts};
use pal_wire::dot11;

const TICK_US: u64 = 2000;
const RZ_NONCE: u32 = 0x5041_4c31;

#[derive(Clone, Default)]
struct Opts {
    config: Option<PathBuf>,
    replay: Option<PathBuf>,
    live: bool,
    ch: u8,
    width: u8,
    card: i32,
    mode: Option<Mode>,
    rust: bool,
    looping: bool,
    log_dir: Option<PathBuf>,
    snapshot: Option<PathBuf>,
    secs: f64,
    mav_udp: Option<String>,
    /// Live: IDR requests on (`--no-idr` off). Replay: off (`--idr` on),
    /// as `pal replay`, so one capture gives the same stats in both apps.
    idr: Option<bool>,
    /// libavcodec hwaccel ("vdpau", "vaapi", "cuda"); "" = software.
    hwdec: String,
    /// Open the settings window at start (P toggles it).
    params: bool,
    /// Per-second latency breakdown on stderr (RX queue, capture->AU,
    /// decode, display).
    lat: bool,
}

fn die(msg: &str) -> ! {
    eprintln!("pal-desktop: {msg}");
    std::process::exit(2)
}

fn parse() -> Opts {
    let mut o = Opts {
        hwdec: "auto".into(),
        rust: true,
        secs: 5.0,
        ..Opts::default()
    };
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        let mut val = || {
            it.next()
                .unwrap_or_else(|| die(&format!("{a} needs a value")))
        };
        match a.as_str() {
            "-c" => o.config = Some(val().into()),
            "--replay" => o.replay = Some(val().into()),
            "--live" => o.live = true,
            "--ch" => o.ch = val().parse().unwrap_or_else(|_| die("--ch")),
            "--width" => o.width = val().parse().unwrap_or_else(|_| die("--width")),
            "--card" => o.card = val().parse().unwrap_or_else(|_| die("--card")),
            "--mode" => {
                o.mode = Some(match val().as_str() {
                    "gs" => Mode::Gs,
                    "spotter" => Mode::Spotter,
                    m => die(&format!("--mode gs|spotter, got {m}")),
                })
            }
            "--core" => o.rust = val() != "ref",
            "--loop" => o.looping = true,
            "--log-dir" => o.log_dir = Some(val().into()),
            "--snapshot" => o.snapshot = Some(val().into()),
            "--secs" => o.secs = val().parse().unwrap_or_else(|_| die("--secs")),
            "--mav-udp" => o.mav_udp = Some(val()),
            "--idr" => o.idr = Some(true),
            "--no-idr" => o.idr = Some(false),
            "--hwdec" => o.hwdec = val(),
            "--params" => o.params = true,
            "--lat" => o.lat = true,
            s => die(&format!("unknown argument {s}")),
        }
    }
    if o.live && o.replay.is_some() {
        die("--live and --replay are exclusive");
    }
    // No source given: live, the everyday case (`cargo run -p pal-desktop`).
    o.live = o.replay.is_none();
    if o.config.is_none() {
        // -c, else $PAL_CONFIG, else the bench rig's config in this repo.
        let p = std::env::var_os("PAL_CONFIG")
            .map(PathBuf::from)
            .unwrap_or_else(|| {
                PathBuf::from(concat!(
                    env!("CARGO_MANIFEST_DIR"),
                    "/../../configs/bench-ch40.toml"
                ))
            });
        eprintln!("pal-desktop: config {}", p.display());
        o.config = Some(p);
    }
    if o.live {
        o.mode.get_or_insert(Mode::Gs);
        // Channel and width from the config unless given: radio.channel's
        // pin, else the first of radio.channels.
        let cfg = o.config.as_ref().expect("config");
        let c = pal_core::config::load_config(cfg, None)
            .unwrap_or_else(|e| die(&format!("{}: {e}", cfg.display())));
        if o.ch == 0 {
            o.ch = c
                .radio
                .pin
                .or(c.radio.channels.first().copied())
                .unwrap_or(0);
        }
        if o.width == 0 {
            o.width = c.radio.width;
        }
    }
    o
}

fn make_core(o: &Opts, channel: u8) -> Box<dyn GsCore> {
    let cfg = o
        .config
        .clone()
        .unwrap_or_else(|| die("-c <gs.toml> is required"));
    let mode = o.mode.unwrap_or(Mode::Spotter);
    if o.rust {
        let c = pal_core::config::load_config(&cfg, None).unwrap_or_else(|e| die(&e.to_string()));
        let opts = RustOpts {
            mode,
            adaptive_gap: true,
            rz_nonce: RZ_NONCE,
            channel,
        };
        return Box::new(RustCore::new(&c, opts).unwrap_or_else(|e| die(&e)));
    }
    let opts = RefOpts {
        mode,
        adaptive_gap: true,
        rz_nonce: RZ_NONCE,
        channel,
    };
    Box::new(RefCore::new(&cfg, opts).unwrap_or_else(|e| die(&format!("core: {e}"))))
}

/// Counters the UI shows that no stats line carries.
#[derive(Default)]
struct Shared {
    au_dropped: AtomicU64,
    frames: AtomicU64,
    dec_errors: AtomicU64,
    /// Sink AU number of the AU decoded when `dec_errors` last rose.
    dec_err_au: AtomicU64,
    ended: AtomicBool,
    /// The window closed: the core loop returns, so the flight log flushes.
    quit: AtomicBool,
    /// The core loop has returned and dropped its Session.
    core_done: AtomicBool,
    lat: Lat,
}

/// `--lat` probes. Stamps are `lat_us()`; *_max are per report window and
/// reset by the report.
#[derive(Default)]
struct Lat {
    rx_sent: AtomicU64,
    rx_full: AtomicU64,
    rx_recv: AtomicU64,
    body_lag_max: AtomicU64,
    cap_au_max: AtomicU64,
    cap_au_last: AtomicU64,
    au_dec_max: AtomicU64,
    dec_us_sum: AtomicU64,
    dec_n: AtomicU64,
    aus_in: AtomicU64,
    show_max: AtomicU64,
    shown: AtomicU64,
    skipped_rgba: AtomicU64,
}

fn lat_us() -> u64 {
    static START: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    START.get_or_init(Instant::now).elapsed().as_micros() as u64
}

struct Frame {
    w: usize,
    h: usize,
    rgba: Vec<u8>,
    seq: u64,
    /// lat_us() when it came out of the decoder.
    t_dec: u64,
}

struct Sink {
    /// AUs to the decoder, tagged with their number (`IdrDriver::aus`) and
    /// the lat_us() they were queued at.
    au_tx: SyncSender<(u64, u64, Vec<u8>)>,
    /// The core's capture-clock offset (stats `pts_off_us`), for --lat.
    pts_off_us: Option<i64>,
    mav_tx: SyncSender<Vec<u8>>,
    /// GCS bridge: inbound packets go to `peer`, whatever it sends comes back.
    udp: Option<(std::net::UdpSocket, std::net::SocketAddr)>,
    card: Option<Arc<devourer_sys::Card>>,
    log: Option<Session>,
    now_us: u64,
    shared: Arc<Shared>,
    /// GS mode only: AU damage and decoder errors into IDR requests.
    idr: Option<IdrDriver>,
}

impl CoreIo for Sink {
    fn on_au(&mut self, a: Au) {
        let n = self.idr.as_mut().map_or(0, |i| {
            i.on_au(&a, self.now_us);
            i.aus()
        });
        if let Some(l) = self.log.as_mut() {
            let _ = l.au(self.now_us, &a);
        }
        if let Some(off) = self.pts_off_us {
            // webgs cap_to_complete_us: capture stamp to this AU's completion.
            let cap = (self.now_us as i64 + off) as u32;
            let d = cap.wrapping_sub(a.pts_us) as i32;
            let lat = &self.shared.lat;
            lat.cap_au_max.fetch_max(d.max(0) as u64, Ordering::Relaxed);
            lat.cap_au_last.store(d.max(0) as u64, Ordering::Relaxed);
        }
        self.shared.lat.aus_in.fetch_add(1, Ordering::Relaxed);
        if let Err(TrySendError::Full(_)) = self.au_tx.try_send((n, lat_us(), a.data)) {
            self.shared.au_dropped.fetch_add(1, Ordering::Relaxed);
            // A dropped AU breaks the reference chain until an IRAP: ask for
            // one now instead of waiting for the decoder to report damage.
            if let Some(i) = self.idr.as_mut() {
                i.r.request(pal_core::refresh::Reason::Decoder, self.now_us / 1000);
            }
        }
    }
    fn on_mav(&mut self, pkt: &[u8]) {
        let _ = self.mav_tx.try_send(pkt.to_vec());
        if let Some((sock, peer)) = &self.udp {
            let _ = sock.send_to(pkt, peer);
        }
    }
    fn send(&mut self, rc_body: &[u8]) {
        if let Some(c) = &self.card {
            c.send_control(rc_body);
        }
    }
}

struct CoreLoop {
    core: Box<dyn GsCore>,
    sink: Sink,
    stats_tx: SyncSender<String>,
    /// F18: the parameter table to the UI (2 Hz), commands and set results back.
    params_tx: SyncSender<String>,
    cmd_rx: Receiver<pal_osd::params::Cmd>,
    set_res_tx: SyncSender<(usize, Result<(), String>)>,
    last_params_us: u64,
    lat: bool,
    last_lat_us: u64,
    rx_full_seen: u64,
    /// (frames, au drops) at the last report, for the stall warning.
    stall_mark: (u64, u64),
    last_stats_us: u64,
    last_log_us: u64,
}

impl CoreLoop {
    fn rx(&mut self, b: &RxBody) {
        self.sink.now_us = b.mono_us;
        if let Some(l) = self.sink.log.as_mut() {
            let _ = l.body(b);
        }
        self.core.on_rx(b, &mut self.sink);
    }
    fn tick(&mut self, now: u64) {
        self.sink.now_us = now;
        if let Some((sock, _)) = &self.sink.udp {
            let mut buf = [0u8; 2048];
            while let Ok((n, _)) = sock.recv_from(&mut buf) {
                self.core.mav_push_outbound(&buf[..n]);
            }
        }
        if let Some(l) = self.sink.log.as_mut() {
            let _ = l.tick(now);
        }
        if let Some(i) = self.sink.idr.as_mut() {
            // Errors first (Acquire pairs with the decoder's Release): the AU
            // number read after it is the same rise's or a newer one.
            let sh = &self.sink.shared;
            let e = sh.dec_errors.load(Ordering::Acquire);
            i.on_decoder_errors(e, sh.dec_err_au.load(Ordering::Relaxed), now);
        }
        while let Ok(c) = self.cmd_rx.try_recv() {
            match c {
                pal_osd::params::Cmd::Set(row, text) => {
                    let r = self.core.param_set(row, &text);
                    eprintln!("pal-desktop: param set {row} {text:?}: {r:?}");
                    let _ = self.set_res_tx.try_send((row, r));
                }
                pal_osd::params::Cmd::Refresh => self.core.param_refresh(),
            }
        }
        self.core.tick(now, &mut self.sink);
        if now >= self.last_params_us + 500_000 {
            self.last_params_us = now;
            let _ = self.params_tx.try_send(self.core.params_json(now));
        }
        if let Some(i) = self.sink.idr.as_mut() {
            let evs = i.step(now, &mut *self.core);
            if let Some(l) = self.sink.log.as_mut() {
                let _ = l.refresh_events(&evs);
            }
        }
        // 10 Hz to the UI, 1 Hz to the flight log.
        if now >= self.last_stats_us + 100_000 {
            self.last_stats_us = now;
            let line = self.core.stats_json();
            if self.sink.log.is_some() && now >= self.last_log_us + 1_000_000 {
                self.last_log_us = now;
                if let Some(l) = self.sink.log.as_mut() {
                    let _ = l.stats(now, &line);
                }
            }
            self.sink.pts_off_us = line
                .split("\"pts_off_us\":")
                .nth(1)
                .and_then(|r| r.split([',', '}']).next())
                .and_then(|v| v.parse().ok());
            let _ = self.stats_tx.try_send(line);
        }
        if now >= self.last_lat_us + 1_000_000 {
            self.last_lat_us = now;
            let l = &self.sink.shared.lat;
            // Without --lat the line still prints when a stage goes bad, so
            // an intermittent multi-second lag leaves its breakdown behind.
            // A decoder that stopped returning (a hung hwaccel driver) shows
            // as AUs dropping while the frame count stands still.
            let frames_now = self.sink.shared.frames.load(Ordering::Relaxed);
            let drops_now = self.sink.shared.au_dropped.load(Ordering::Relaxed);
            if drops_now > self.stall_mark.1 && frames_now == self.stall_mark.0 {
                eprintln!(
                    "pal-desktop: decoder stalled ({frames_now} frames, AUs dropping); a hung hwaccel driver? try --hwdec sw or another backend"
                );
            }
            self.stall_mark = (frames_now, drops_now);
            let bad = l.body_lag_max.load(Ordering::Relaxed) > 200_000
                || l.cap_au_max.load(Ordering::Relaxed) > 500_000
                || l.au_dec_max.load(Ordering::Relaxed) > 300_000
                || l.show_max.load(Ordering::Relaxed) > 300_000
                || l.rx_full.load(Ordering::Relaxed) > self.rx_full_seen;
            self.rx_full_seen = l.rx_full.load(Ordering::Relaxed);
            let take = |a: &AtomicU64| a.swap(0, Ordering::Relaxed);
            if !self.lat && !bad {
                for a in [
                    &l.body_lag_max,
                    &l.cap_au_max,
                    &l.au_dec_max,
                    &l.show_max,
                    &l.dec_us_sum,
                    &l.dec_n,
                    &l.shown,
                ] {
                    take(a);
                }
                return;
            }
            let (sent, full, recv) = (
                l.rx_sent.load(Ordering::Relaxed),
                l.rx_full.load(Ordering::Relaxed),
                l.rx_recv.load(Ordering::Relaxed),
            );
            let (dec_sum, dec_n) = (take(&l.dec_us_sum), take(&l.dec_n));
            let frames = self.sink.shared.frames.load(Ordering::Relaxed);
            eprintln!(
                "lat: rx_queue={} rx_full={full} body_lag_max={}ms cap_au={}ms(max {}) au_to_frame_max={}ms dec_avg={:.1}ms dec_holds={} frame_to_screen_max={}ms shown/s={} au_drops={} rgba_skipped={}",
                sent.saturating_sub(recv + full),
                take(&l.body_lag_max) / 1000,
                l.cap_au_last.load(Ordering::Relaxed) / 1000,
                take(&l.cap_au_max) / 1000,
                take(&l.au_dec_max) / 1000,
                dec_sum as f64 / dec_n.max(1) as f64 / 1000.0,
                l.aus_in.load(Ordering::Relaxed).saturating_sub(frames + self.sink.shared.au_dropped.load(Ordering::Relaxed)),
                take(&l.show_max) / 1000,
                take(&l.shown),
                self.sink.shared.au_dropped.load(Ordering::Relaxed),
                l.skipped_rgba.load(Ordering::Relaxed),
            );
        }
    }
}

fn mono_us(t0: Instant) -> u64 {
    1_000_000 + t0.elapsed().as_micros() as u64
}

/// Replay at 1x on the capture's own clock: each record waits until its
/// stamp, measured from the first record, has passed on the wall clock.
fn run_replay(mut cl: CoreLoop, o: &Opts, path: PathBuf, repaint: egui::Context) {
    loop {
        let f =
            std::fs::File::open(&path).unwrap_or_else(|e| die(&format!("{}: {e}", path.display())));
        let r = capture::Reader::new(std::io::BufReader::new(f))
            .unwrap_or_else(|e| die(&e.to_string()));
        let wall0 = Instant::now();
        let mut t0: Option<u64> = None;
        for rec in r {
            let Ok(rec) = rec else { break };
            let t = match &rec {
                Record::Body(b) => b.mono_us,
                Record::Tick(t) => *t,
            };
            let base = *t0.get_or_insert(t);
            let due = Duration::from_micros(t.saturating_sub(base));
            // In slices, so a closed window is noticed inside a long gap.
            while let Some(w) = due.checked_sub(wall0.elapsed()) {
                if cl.sink.shared.quit.load(Ordering::Relaxed) {
                    return;
                }
                std::thread::sleep(w.min(Duration::from_millis(50)));
            }
            match rec {
                Record::Body(b) => cl.rx(&b),
                Record::Tick(t) => cl.tick(t),
            }
        }
        if !o.looping || cl.sink.shared.quit.load(Ordering::Relaxed) {
            break;
        }
        // A new pass is a new session: fresh core, as after a restart.
        cl.core = make_core(o, 0);
        cl.last_stats_us = 0;
    }
    if let Some(l) = cl.sink.log.as_mut() {
        let _ = l.flush();
    }
    cl.sink.shared.ended.store(true, Ordering::Relaxed);
    repaint.request_repaint();
}

fn run_live(mut cl: CoreLoop, o: Opts) {
    let mut card = devourer_sys::Card::open(&devourer_sys::Config {
        select: devourer_sys::Select::Scan { index: o.card },
        channel: o.ch,
        width_mhz: o.width,
    })
    .unwrap_or_else(|e| die(&format!("card: {e}")));
    eprintln!(
        "pal-desktop: card {} on ch {} / {} MHz",
        card.chip().unwrap_or("?".into()),
        o.ch,
        o.width
    );
    let t0 = Instant::now();
    let (tx, rx) = mpsc::sync_channel::<RxBody>(4096);
    let ch = o.ch;
    let shared_rx = cl.sink.shared.clone();
    card.start_rx(move |f| {
        if let dot11::Verdict::Body(p) = dot11::parse(f.frame, f.crc_err, f.data_rate) {
            shared_rx.lat.rx_sent.fetch_add(1, Ordering::Relaxed);
            let r = tx.try_send(RxBody {
                card_id: 0,
                mono_us: mono_us(t0),
                rssi: f.rssi,
                snr: f.snr,
                evm: f.evm,
                phy_valid: f.physt,
                crc_ok: !f.crc_err,
                mac_seq: p.mac_seq,
                tsfl: f.tsfl,
                mcs: p.mcs,
                rx_channel: ch,
                body: f.frame[p.body_off..].to_vec(),
            });
            if r.is_err() {
                shared_rx.lat.rx_full.fetch_add(1, Ordering::Relaxed);
            }
        }
    })
    .unwrap_or_else(|e| die(&e));
    let card = Arc::new(card);
    if o.mode == Some(Mode::Gs) {
        cl.sink.card = Some(card.clone());
    }
    let mut next_tick = mono_us(t0);
    loop {
        let wait = Duration::from_micros(next_tick.saturating_sub(mono_us(t0)));
        match rx.recv_timeout(wait) {
            Ok(b) => {
                let l = &cl.sink.shared.lat;
                l.rx_recv.fetch_add(1, Ordering::Relaxed);
                l.body_lag_max
                    .fetch_max(mono_us(t0).saturating_sub(b.mono_us), Ordering::Relaxed);
                cl.rx(&b)
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(mpsc::RecvTimeoutError::Disconnected) => die("RX thread ended"),
        }
        let now = mono_us(t0);
        if now >= next_tick {
            cl.tick(now);
            next_tick = now + TICK_US;
        }
        if cl.sink.shared.quit.load(Ordering::Relaxed) {
            return;
        }
    }
}

fn run_decode(
    au_rx: Receiver<(u64, u64, Vec<u8>)>,
    out: Arc<Mutex<Option<Frame>>>,
    shared: Arc<Shared>,
    repaint: egui::Context,
    hwdec: String,
) {
    let mut dec = decode::AvDecoder::open_or_sw(0, if hwdec == "sw" { "" } else { &hwdec })
        .unwrap_or_else(|e| die(&format!("decode: {e}")));
    let mut buf = Vec::new();
    let mut seq = 0u64;
    let mut errs = 0;
    // One AU of lookahead: a frame whose successor is already queued is
    // never shown, so it skips the GPU download and RGBA conversion (the
    // bulk of the per-frame cost at 90 fps).
    let mut next: Option<(u64, u64, Vec<u8>)> = None;
    while let Some((n, t_enq, au)) = next.take().or_else(|| au_rx.recv().ok()) {
        let t_dec0 = lat_us();
        let got = dec.decode(&au);
        let t_dec1 = lat_us();
        shared
            .lat
            .dec_us_sum
            .fetch_add(t_dec1 - t_dec0, Ordering::Relaxed);
        shared.lat.dec_n.fetch_add(1, Ordering::Relaxed);
        if got > 0 {
            shared
                .lat
                .au_dec_max
                .fetch_max(t_dec1.saturating_sub(t_enq), Ordering::Relaxed);
        }
        let (f, e) = dec.counters();
        if e > errs {
            errs = e;
            shared.dec_err_au.store(n, Ordering::Relaxed);
            shared.dec_errors.store(e, Ordering::Release);
        }
        if got == 0 {
            continue;
        }
        shared.frames.store(f, Ordering::Relaxed);
        if let Ok(q) = au_rx.try_recv() {
            next = Some(q);
            shared.lat.skipped_rgba.fetch_add(1, Ordering::Relaxed);
            continue;
        }
        if let Some((w, h)) = dec.rgba(&mut buf) {
            seq += 1;
            let mut slot = out.lock().unwrap_or_else(|p| p.into_inner());
            // Keep only the newest frame; reuse the old buffer for the next one.
            let old = slot.replace(Frame {
                w,
                h,
                rgba: std::mem::take(&mut buf),
                seq,
                t_dec: t_dec1,
            });
            if let Some(old) = old {
                buf = old.rgba;
            }
            drop(slot);
            repaint.request_repaint();
        }
    }
}

struct App {
    mav_rx: Receiver<Vec<u8>>,
    telem: pal_osd::telem::Telemetry,
    frame: Arc<Mutex<Option<Frame>>>,
    shared: Arc<Shared>,
    stats_rx: Receiver<String>,
    params_rx: Receiver<String>,
    cmd_tx: mpsc::Sender<pal_osd::params::Cmd>,
    set_res_rx: Receiver<(usize, Result<(), String>)>,
    params: Option<pal_osd::params::Table>,
    edits: pal_osd::params::Edits,
    show_params: bool,
    hist: History,
    tex: Option<egui::TextureHandle>,
    shown_seq: u64,
    t0: Instant,
    fps: f64,
    fps_mark: (Instant, u64),
    show_monitor: bool,
    snapshot: Option<(PathBuf, f64)>,
    snapshot_sent: bool,
}

fn write_ppm(path: &PathBuf, img: &egui::ColorImage) -> std::io::Result<()> {
    use std::io::Write;
    let mut f = std::io::BufWriter::new(std::fs::File::create(path)?);
    write!(f, "P6\n{} {}\n255\n", img.size[0], img.size[1])?;
    for p in &img.pixels {
        f.write_all(&[p.r(), p.g(), p.b()])?;
    }
    f.flush()
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _f: &mut eframe::Frame) {
        let now = self.t0.elapsed().as_secs_f64();
        while let Ok(p) = self.mav_rx.try_recv() {
            self.telem.on_packet(&p, now);
        }
        while let Ok(line) = self.stats_rx.try_recv() {
            if let Some(s) = Snapshot::parse(&line) {
                self.hist.push(now, s);
            }
        }
        if let Some(fr) = self
            .frame
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .as_ref()
        {
            if fr.seq != self.shown_seq {
                self.shown_seq = fr.seq;
                let l = &self.shared.lat;
                l.show_max
                    .fetch_max(lat_us().saturating_sub(fr.t_dec), Ordering::Relaxed);
                l.shown.fetch_add(1, Ordering::Relaxed);
                let img = egui::ColorImage::from_rgba_unmultiplied([fr.w, fr.h], &fr.rgba);
                match self.tex.as_mut() {
                    Some(t) => t.set(img, egui::TextureOptions::LINEAR),
                    None => {
                        self.tex =
                            Some(ctx.load_texture("video", img, egui::TextureOptions::LINEAR))
                    }
                }
            }
        }
        let frames = self.shared.frames.load(Ordering::Relaxed);
        if self.fps_mark.0.elapsed() >= Duration::from_secs(1) {
            self.fps = (frames - self.fps_mark.1) as f64 / self.fps_mark.0.elapsed().as_secs_f64();
            self.fps_mark = (Instant::now(), frames);
        }
        // Hotkeys only while no text field has focus (typing a value must
        // not toggle panels).
        let typing = ctx.memory(|m| m.focused().is_some());
        if !typing && ctx.input(|i| i.key_pressed(egui::Key::M)) {
            self.show_monitor = !self.show_monitor;
        }
        if !typing && ctx.input(|i| i.key_pressed(egui::Key::P)) {
            self.show_params = !self.show_params;
        }
        while let Ok(l) = self.params_rx.try_recv() {
            if let Some(t) = pal_osd::params::Table::parse(&l) {
                self.params = Some(t);
            }
        }
        while let Ok(r) = self.set_res_rx.try_recv() {
            self.edits.last = Some(r);
        }
        if self.show_params {
            if let Some(t) = &self.params {
                let mut open = true;
                egui::Window::new("drone settings (P)")
                    .open(&mut open)
                    .default_pos([24.0, 120.0])
                    .show(ctx, |ui| {
                        if let Some(c) = pal_osd::params::widget(ui, t, &mut self.edits) {
                            let _ = self.cmd_tx.send(c);
                        }
                    });
                self.show_params = open;
            }
        }
        if self.show_monitor {
            egui::SidePanel::right("monitor")
                .default_width(320.0)
                .show(ctx, |ui| {
                    ui.heading("monitor (M)");
                    pal_osd::monitor(ui, &self.hist, now);
                    ui.separator();
                    ui.label(format!(
                        "decoded {}  dec errors {}  AU drops {}",
                        frames,
                        self.shared.dec_errors.load(Ordering::Relaxed),
                        self.shared.au_dropped.load(Ordering::Relaxed)
                    ));
                    if self.shared.ended.load(Ordering::Relaxed) {
                        ui.label("replay ended");
                    }
                });
        }
        egui::CentralPanel::default()
            .frame(egui::Frame::NONE.fill(egui::Color32::BLACK))
            .show(ctx, |ui| {
                let area = ui.max_rect();
                if let Some(t) = &self.tex {
                    let [w, h] = t.size().map(|v| v as f32);
                    let scale = (area.width() / w).min(area.height() / h);
                    let r = egui::Rect::from_center_size(
                        area.center(),
                        egui::vec2(w * scale, h * scale),
                    );
                    ui.painter().image(
                        t.id(),
                        r,
                        egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
                        egui::Color32::WHITE,
                    );
                }
                let decode = (frames, self.shared.dec_errors.load(Ordering::Relaxed));
                ui.allocate_new_ui(egui::UiBuilder::new().max_rect(area.shrink(8.0)), |ui| {
                    pal_osd::chips(ui, &self.hist, self.fps, decode);
                    pal_osd::key_banner(ui, &self.hist);
                });
            });
        egui::Area::new(egui::Id::new("telem"))
            .anchor(egui::Align2::LEFT_BOTTOM, egui::vec2(8.0, -8.0))
            .show(ctx, |ui| pal_osd::telem::widget(ui, &self.telem, now));
        if let Some((path, secs)) = self.snapshot.clone() {
            if !self.snapshot_sent && now >= secs {
                self.snapshot_sent = true;
                ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(egui::UserData::default()));
            }
            let shot = ctx.input(|i| {
                i.events.iter().find_map(|e| match e {
                    egui::Event::Screenshot { image, .. } => Some(image.clone()),
                    _ => None,
                })
            });
            if let Some(img) = shot {
                write_ppm(&path, &img).unwrap_or_else(|e| die(&format!("{}: {e}", path.display())));
                eprintln!(
                    "pal-desktop: snapshot {} ({}x{}), frames {frames}",
                    path.display(),
                    img.size[0],
                    img.size[1]
                );
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
        }
        ctx.request_repaint_after(Duration::from_millis(100));
    }
}

fn main() {
    let o = parse();
    // Every dropped AU breaks the reference chain until the next intra
    // refresh (a 2-slot queue dropped 6 AUs at replay start and the
    // decoder logged missing POCs), so the queue absorbs decoder warm-up.
    let (au_tx, au_rx) = mpsc::sync_channel::<(u64, u64, Vec<u8>)>(8);
    let (stats_tx, stats_rx) = mpsc::sync_channel::<String>(64);
    let (params_tx, params_rx) = mpsc::sync_channel::<String>(4);
    let (cmd_tx, cmd_rx) = mpsc::channel::<pal_osd::params::Cmd>();
    let (set_res_tx, set_res_rx) = mpsc::sync_channel(8);
    let (mav_tx, mav_rx) = mpsc::sync_channel::<Vec<u8>>(256);
    let udp = o.mav_udp.as_ref().map(|a| {
        let peer: std::net::SocketAddr = a.parse().unwrap_or_else(|_| die("--mav-udp HOST:PORT"));
        let sock =
            std::net::UdpSocket::bind("0.0.0.0:0").unwrap_or_else(|e| die(&format!("udp: {e}")));
        sock.set_nonblocking(true)
            .unwrap_or_else(|e| die(&format!("udp: {e}")));
        eprintln!(
            "pal-desktop: MAVLink bridge {} -> {peer}",
            sock.local_addr().map_or("?".into(), |a| a.to_string())
        );
        (sock, peer)
    });
    let shared = Arc::new(Shared::default());
    let frame = Arc::new(Mutex::new(None));
    let log = o.log_dir.as_ref().map(|d| {
        let cap = if o.live { 512 << 20 } else { 0 };
        let s = Session::create(d, std::time::SystemTime::now(), cap)
            .unwrap_or_else(|e| die(&format!("{}: {e}", d.display())));
        eprintln!("pal-desktop: flight log {}", s.dir().display());
        s
    });
    let channel = if o.live { o.ch } else { 0 };
    let sink = Sink {
        au_tx,
        mav_tx,
        udp,
        card: None,
        log,
        now_us: 0,
        pts_off_us: None,
        shared: shared.clone(),
        idr: (o.mode == Some(Mode::Gs) && o.idr.unwrap_or(o.live)).then(IdrDriver::default),
    };
    let native = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 760.0])
            .with_title("Palianytsia"),
        ..Default::default()
    };
    let snapshot = o.snapshot.clone().map(|p| (p, o.secs));
    let shared_main = shared.clone();
    let res = eframe::run_native(
        "Palianytsia",
        native,
        Box::new(move |cc| {
            let ctx = cc.egui_ctx.clone();
            {
                let (frame, shared, ctx) = (frame.clone(), shared.clone(), ctx.clone());
                let hwdec = o.hwdec.clone();
                std::thread::Builder::new()
                    .name("decode".into())
                    .spawn(move || run_decode(au_rx, frame, shared, ctx, hwdec))
                    .expect("decode thread");
            }
            {
                let o = o.clone();
                let ctx = ctx.clone();
                std::thread::Builder::new()
                    .name("core".into())
                    .spawn(move || {
                        // Built here: pal-ref's core is not Send.
                        let cl = CoreLoop {
                            core: make_core(&o, channel),
                            sink,
                            stats_tx,
                            params_tx,
                            cmd_rx,
                            set_res_tx,
                            last_params_us: 0,
                            lat: o.lat,
                            last_lat_us: 0,
                            rx_full_seen: 0,
                            stall_mark: (0, 0),
                            last_stats_us: 0,
                            last_log_us: 0,
                        };
                        let shared = cl.sink.shared.clone();
                        match o.replay.clone() {
                            Some(p) => run_replay(cl, &o, p, ctx),
                            None => run_live(cl, o),
                        }
                        // `cl` (and its Session) is dropped by now.
                        shared.core_done.store(true, Ordering::Release);
                    })
                    .expect("core thread");
            }
            Ok(Box::new(App {
                mav_rx,
                telem: Default::default(),
                frame,
                shared,
                stats_rx,
                params_rx,
                cmd_tx,
                set_res_rx,
                params: None,
                edits: Default::default(),
                show_params: o.params,
                hist: History::default(),
                tex: None,
                shown_seq: 0,
                t0: Instant::now(),
                fps: 0.0,
                fps_mark: (Instant::now(), 0),
                show_monitor: true,
                snapshot,
                snapshot_sent: false,
            }))
        }),
    );
    // The process exit would skip the core thread's drops: the flight log's
    // buffered tail was lost on every window close (and on --snapshot).
    shared_main.quit.store(true, Ordering::Relaxed);
    let until = Instant::now() + Duration::from_secs(2);
    while !shared_main.core_done.load(Ordering::Acquire) && Instant::now() < until {
        std::thread::sleep(Duration::from_millis(10));
    }
    if let Err(e) = res {
        die(&format!("eframe: {e}"));
    }
}
