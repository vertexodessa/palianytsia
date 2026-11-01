//! The GS runtime shared by pal-desktop and the phone/Quest apps (spec 5.3
//! `runtime`): the devourer RX thread, the core loop (GsCore + IdrDriver +
//! flight log + stats/params out, commands in) and the counters a platform
//! decoder feeds back. Decoding and drawing stay in the apps.
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, SyncSender, TrySendError};
use std::sync::Arc;
use std::time::{Duration, Instant};

use flightlog::session::Session;
use pal_core::refresh::IdrDriver;
use pal_core::{Au, CoreIo, GsCore, RxBody};
use pal_wire::dot11;

/// Core tick period.
pub const TICK_US: u64 = 2000;

/// Monotonic stamp for --lat probes (process-wide).
pub fn lat_us() -> u64 {
    static START: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    START.get_or_init(Instant::now).elapsed().as_micros() as u64
}

/// Counters the UI shows that no stats line carries.
#[derive(Default)]
pub struct Shared {
    pub au_dropped: AtomicU64,
    pub frames: AtomicU64,
    pub dec_errors: AtomicU64,
    /// Sink AU number of the AU decoded when `dec_errors` last rose.
    pub dec_err_au: AtomicU64,
    pub ended: AtomicBool,
    /// The window closed: the core loop returns, so the flight log flushes.
    pub quit: AtomicBool,
    /// The core loop has returned and dropped its Session.
    pub core_done: AtomicBool,
    pub lat: Lat,
}

/// `--lat` probes. Stamps are `lat_us()`; *_max are per report window and
/// reset by the report.
#[derive(Default)]
pub struct Lat {
    pub rx_sent: AtomicU64,
    pub rx_full: AtomicU64,
    pub rx_recv: AtomicU64,
    pub body_lag_max: AtomicU64,
    pub cap_au_max: AtomicU64,
    pub cap_au_last: AtomicU64,
    pub au_dec_max: AtomicU64,
    pub dec_us_sum: AtomicU64,
    pub dec_n: AtomicU64,
    pub aus_in: AtomicU64,
    pub show_max: AtomicU64,
    pub shown: AtomicU64,
    pub skipped_rgba: AtomicU64,
}

pub struct Sink {
    /// AUs to the decoder, tagged with their number (`IdrDriver::aus`) and
    /// the lat_us() they were queued at.
    pub au_tx: SyncSender<(u64, u64, Vec<u8>)>,
    /// The core's capture-clock offset (stats `pts_off_us`), for --lat.
    pub pts_off_us: Option<i64>,
    pub mav_tx: SyncSender<Vec<u8>>,
    /// GCS bridge: inbound packets go to `peer`, whatever it sends comes back.
    pub udp: Option<(std::net::UdpSocket, std::net::SocketAddr)>,
    pub card: Option<Arc<devourer_sys::Card>>,
    pub log: Option<Session>,
    pub now_us: u64,
    pub shared: Arc<Shared>,
    /// GS mode only: AU damage and decoder errors into IDR requests.
    pub idr: Option<IdrDriver>,
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

pub struct CoreLoop {
    pub core: Box<dyn GsCore>,
    pub sink: Sink,
    pub stats_tx: SyncSender<String>,
    /// F18: the parameter table to the UI (2 Hz), commands and set results back.
    pub params_tx: SyncSender<String>,
    pub cmd_rx: Receiver<pal_osd::params::Cmd>,
    pub set_res_tx: SyncSender<(usize, Result<(), String>)>,
    pub last_params_us: u64,
    pub lat: bool,
    pub last_lat_us: u64,
    pub rx_full_seen: u64,
    /// (frames, au drops) at the last report, for the stall warning.
    pub stall_mark: (u64, u64),
    pub last_stats_us: u64,
    pub last_log_us: u64,
}

impl CoreLoop {
    pub fn rx(&mut self, b: &RxBody) {
        self.sink.now_us = b.mono_us;
        if let Some(l) = self.sink.log.as_mut() {
            let _ = l.body(b);
        }
        self.core.on_rx(b, &mut self.sink);
    }
    pub fn tick(&mut self, now: u64) {
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
                    eprintln!("pal: param set {row} {text:?}: {r:?}");
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
            // The ladder view goes to the UI only: the flight log keeps the
            // webgs stats line both cores agree on.
            let ui_line = match self.core.ladder_json() {
                Some(l) if line.ends_with('}') => {
                    format!("{},\"ladder\":{l}}}", &line[..line.len() - 1])
                }
                _ => line,
            };
            let _ = self.stats_tx.try_send(ui_line);
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
                    "pal: decoder stalled ({frames_now} frames, AUs dropping); a hung hwaccel driver? try --hwdec sw or another backend"
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

pub fn mono_us(t0: Instant) -> u64 {
    1_000_000 + t0.elapsed().as_micros() as u64
}

/// Where the card comes from and how to tune it.
pub struct LiveCfg {
    pub select: devourer_sys::Select,
    pub ch: u8,
    pub width: u8,
    /// GS mode: the card also transmits the core's RC frames.
    pub gs: bool,
}

/// Opens the card and runs the core loop on this thread until
/// `shared.quit`; `on_open` gets the chip name once the card is up.
pub fn run_live(mut cl: CoreLoop, o: LiveCfg, on_open: impl FnOnce(&str)) -> Result<(), String> {
    let mut card = devourer_sys::Card::open(&devourer_sys::Config {
        select: o.select,
        channel: o.ch,
        width_mhz: o.width,
    })
    .map_err(|e| format!("card: {e}"))?;
    on_open(&card.chip().unwrap_or("?".into()));
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
    })?;
    let card = Arc::new(card);
    if o.gs {
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
            Err(mpsc::RecvTimeoutError::Disconnected) => return Err("RX thread ended".into()),
        }
        let now = mono_us(t0);
        if now >= next_tick {
            cl.tick(now);
            next_tick = now + TICK_US;
        }
        if cl.sink.shared.quit.load(Ordering::Relaxed) {
            return Ok(());
        }
    }
}
