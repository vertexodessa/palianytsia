//! pal-desktop: the native Linux GS (spec 5.7). Source is a devourer card
//! (`--live`) or a capture replayed at 1x (`--replay bodies.bin`); the core
//! runs on its own thread, AUs go to a decode thread through an 8-slot queue
//! (drop-newest + counter when full), and the UI only reads the newest frame
//! and the newest stats line.
//!
//!   pal-desktop -c gs.toml --replay bodies.bin [--mode gs|spotter] [--loop]
//!   pal-desktop -c gs.toml --live --ch 144 --width 40 [--mode gs] [--card N]
//!   common: [--core rust|ref] [--log-dir DIR] [--snapshot out.ppm --secs N]
//!           [--mav-udp HOST:PORT]   drone MAVLink to a GCS (QGroundControl) and back
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
}

fn die(msg: &str) -> ! {
    eprintln!("pal-desktop: {msg}");
    std::process::exit(2)
}

fn parse() -> Opts {
    let mut o = Opts {
        ch: 149,
        width: 20,
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
            s => die(&format!("unknown argument {s}")),
        }
    }
    if o.live == o.replay.is_some() {
        die("exactly one of --live or --replay FILE");
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
    ended: AtomicBool,
}

struct Frame {
    w: usize,
    h: usize,
    rgba: Vec<u8>,
    seq: u64,
}

struct Sink {
    au_tx: SyncSender<Vec<u8>>,
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
        if let Some(i) = self.idr.as_mut() {
            i.on_au(&a, self.now_us);
        }
        if let Some(l) = self.log.as_mut() {
            let _ = l.au(self.now_us, &a);
        }
        if let Err(TrySendError::Full(_)) = self.au_tx.try_send(a.data) {
            self.shared.au_dropped.fetch_add(1, Ordering::Relaxed);
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
            i.on_decoder_errors(self.sink.shared.dec_errors.load(Ordering::Relaxed), now);
        }
        self.core.tick(now, &mut self.sink);
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
            let _ = self.stats_tx.try_send(line);
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
            if let Some(w) = due.checked_sub(wall0.elapsed()) {
                std::thread::sleep(w);
            }
            match rec {
                Record::Body(b) => cl.rx(&b),
                Record::Tick(t) => cl.tick(t),
            }
        }
        if !o.looping {
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
    card.start_rx(move |f| {
        if let dot11::Verdict::Body(p) = dot11::parse(f.frame, f.crc_err, f.data_rate) {
            let _ = tx.try_send(RxBody {
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
            Ok(b) => cl.rx(&b),
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(mpsc::RecvTimeoutError::Disconnected) => die("RX thread ended"),
        }
        let now = mono_us(t0);
        if now >= next_tick {
            cl.tick(now);
            next_tick = now + TICK_US;
        }
    }
}

fn run_decode(
    au_rx: Receiver<Vec<u8>>,
    out: Arc<Mutex<Option<Frame>>>,
    shared: Arc<Shared>,
    repaint: egui::Context,
) {
    let mut dec = decode::AvDecoder::open(0).unwrap_or_else(|e| die(&format!("decode: {e}")));
    let mut buf = Vec::new();
    let mut seq = 0u64;
    while let Ok(au) = au_rx.recv() {
        if dec.decode(&au) == 0 {
            continue;
        }
        let (f, e) = dec.counters();
        shared.frames.store(f, Ordering::Relaxed);
        shared.dec_errors.store(e, Ordering::Relaxed);
        if let Some((w, h)) = dec.rgba(&mut buf) {
            seq += 1;
            let mut slot = out.lock().unwrap_or_else(|p| p.into_inner());
            // Keep only the newest frame; reuse the old buffer for the next one.
            let old = slot.replace(Frame {
                w,
                h,
                rgba: std::mem::take(&mut buf),
                seq,
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
        if ctx.input(|i| i.key_pressed(egui::Key::M)) {
            self.show_monitor = !self.show_monitor;
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
    let (au_tx, au_rx) = mpsc::sync_channel::<Vec<u8>>(8);
    let (stats_tx, stats_rx) = mpsc::sync_channel::<String>(64);
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
        shared: shared.clone(),
        idr: (o.mode == Some(Mode::Gs)).then(IdrDriver::default),
    };
    let native = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 760.0])
            .with_title("Palianytsia"),
        ..Default::default()
    };
    let snapshot = o.snapshot.clone().map(|p| (p, o.secs));
    let res = eframe::run_native(
        "Palianytsia",
        native,
        Box::new(move |cc| {
            let ctx = cc.egui_ctx.clone();
            {
                let (frame, shared, ctx) = (frame.clone(), shared.clone(), ctx.clone());
                std::thread::Builder::new()
                    .name("decode".into())
                    .spawn(move || run_decode(au_rx, frame, shared, ctx))
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
                            last_stats_us: 0,
                            last_log_us: 0,
                        };
                        match o.replay.clone() {
                            Some(p) => run_replay(cl, &o, p, ctx),
                            None => run_live(cl, o),
                        }
                    })
                    .expect("core thread");
            }
            Ok(Box::new(App {
                mav_rx,
                telem: Default::default(),
                frame,
                shared,
                stats_rx,
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
    if let Err(e) = res {
        die(&format!("eframe: {e}"));
    }
}
