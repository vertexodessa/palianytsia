//! The GS runtime shared by pal-desktop and the phone/Quest apps (spec 5.3
//! `runtime`): the devourer RX thread, the core loop (GsCore + IdrDriver +
//! flight log + stats/params out, commands in) and the counters a platform
//! decoder feeds back. Decoding and drawing stay in the apps.
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicU8, Ordering};
use std::sync::mpsc::{self, Receiver, SyncSender, TrySendError};
use std::sync::{Arc, Mutex};
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
    /// Events from other threads (decoder stalls) for the flight log; the
    /// core loop drains them on its tick.
    pub events: Mutex<Vec<(&'static str, serde_json::Value)>>,
    /// The channel the card listens on; 0 before the card is up.
    pub channel: AtomicU8,
    /// The card is sweeping the channel set for the drone.
    pub searching: AtomicBool,
}

impl Shared {
    /// "ch 40 / 40 MHz", or "searching for the drone: ch 64 / 40 MHz".
    pub fn radio_label(&self, width: u8) -> String {
        let ch = self.channel.load(Ordering::Relaxed);
        if self.searching.load(Ordering::Relaxed) {
            format!("searching for the drone: ch {ch} / {width} MHz")
        } else {
            format!("ch {ch} / {width} MHz")
        }
    }

    pub fn event(&self, ev: &'static str, detail: serde_json::Value) {
        let mut q = self.events.lock().unwrap_or_else(|p| p.into_inner());
        // A wedged core loop must not grow this without bound.
        if q.len() < 256 {
            q.push((ev, detail));
        }
    }
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
        let evs = std::mem::take(
            &mut *self
                .sink
                .shared
                .events
                .lock()
                .unwrap_or_else(|p| p.into_inner()),
        );
        if let Some(l) = self.sink.log.as_mut() {
            let _ = l.tick(now);
            for (ev, d) in evs {
                let _ = l.event(now, ev, d);
            }
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
                    // Android has no stderr: the flight log keeps it.
                    self.sink.shared.event(
                        "param_set",
                        serde_json::json!({"row": row, "value": text,
                            "result": match &r { Ok(()) => "sent".to_string(), Err(e) => e.clone() }}),
                    );
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
                    // A killed app (Android swipe, OOM) skips Drop: at most
                    // one second of log is lost, a few writes a second.
                    let _ = l.flush();
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

/// Builds a core for the channel the drone was found on.
pub type Remake = Box<dyn FnMut(u8) -> Result<Box<dyn GsCore>, String>>;

/// Where the card comes from and how to tune it.
pub struct LiveCfg {
    pub select: devourer_sys::Select,
    pub ch: u8,
    pub width: u8,
    /// GS mode: the card also transmits the core's RC frames.
    pub gs: bool,
    /// The channel set to sweep for the drone (radio.channel = "auto"):
    /// at start, and again after `LOST_US` without a drone frame. Empty =
    /// stay on `ch`. Needs `remake`: a core's DISC proposes the channel it
    /// was built for, and a core on the wrong one would steer the drone.
    pub search: Vec<u8>,
    pub remake: Option<Remake>,
}

/// Listening time per channel. The drone sends video frames continuously
/// (and its 1 Hz telemetry when idle), so 3 frames in 400 ms is certain
/// on its channel and impossible elsewhere: SA-matched and CRC-good.
pub const DWELL_MS: u64 = 400;
pub const FOUND_HITS: u32 = 3;
/// No drone frame for this long while searching is enabled: sweep again.
pub const LOST_US: u64 = 8_000_000;

/// Sweeps `set` (starting at `first`) until `hits` grows by FOUND_HITS
/// within one dwell. Bodies queued meanwhile are dropped: none may reach a
/// core built for another channel. None when asked to quit.
#[allow(clippy::too_many_arguments)]
fn search(
    card: &devourer_sys::Card,
    set: &[u8],
    first: u8,
    width: u8,
    cur: &AtomicU8,
    hits: &AtomicU32,
    rx: &Receiver<RxBody>,
    shared: &Shared,
) -> Result<Option<u8>, String> {
    let mut order = vec![first];
    order.extend(set.iter().copied().filter(|&c| c != first));
    shared.searching.store(true, Ordering::Relaxed);
    let t = Instant::now();
    let mut rounds = 0u32;
    let found = 'sweep: loop {
        for &ch in &order {
            if shared.quit.load(Ordering::Relaxed) {
                break 'sweep None;
            }
            // Blind the RX stamp across the move, as maburgs' retune does.
            cur.store(0, Ordering::Relaxed);
            card.set_channel(ch, width)?;
            shared.channel.store(ch, Ordering::Relaxed);
            // Frames already in the USB pipeline belong to the old channel.
            std::thread::sleep(Duration::from_millis(5));
            while rx.try_recv().is_ok() {}
            let h0 = hits.load(Ordering::Relaxed);
            cur.store(ch, Ordering::Relaxed);
            let end = Instant::now() + Duration::from_millis(DWELL_MS);
            while Instant::now() < end {
                let _ = rx.recv_timeout(Duration::from_millis(20));
                if hits.load(Ordering::Relaxed).wrapping_sub(h0) >= FOUND_HITS {
                    break 'sweep Some(ch);
                }
            }
        }
        rounds += 1;
        if rounds == 1 {
            shared.event("search", serde_json::json!({"set": set, "first": first}));
        }
    };
    shared.searching.store(false, Ordering::Relaxed);
    if let Some(ch) = found {
        shared.event(
            "drone_found",
            serde_json::json!({"ch": ch, "ms": t.elapsed().as_millis() as u64, "rounds": rounds}),
        );
    }
    Ok(found)
}

/// Opens the card and runs the core loop on this thread until
/// `shared.quit`; `on_open` gets the chip name once the card is up.
pub fn run_live(
    mut cl: CoreLoop,
    mut o: LiveCfg,
    on_open: impl FnOnce(&str),
) -> Result<(), String> {
    let mut card = devourer_sys::Card::open(&devourer_sys::Config {
        select: o.select.clone(),
        channel: o.ch,
        width_mhz: o.width,
    })
    .map_err(|e| format!("card: {e}"))?;
    on_open(&card.chip().unwrap_or("?".into()));
    let t0 = Instant::now();
    let (tx, rx) = mpsc::sync_channel::<RxBody>(4096);
    // The channel RX frames are stamped with; 0 while the card retunes.
    let cur = Arc::new(AtomicU8::new(o.ch));
    // CRC-good drone data frames (SA-matched by dot11::parse) since open.
    let hits = Arc::new(AtomicU32::new(0));
    let shared_rx = cl.sink.shared.clone();
    let (cur_rx, hits_rx) = (cur.clone(), hits.clone());
    card.start_rx(move |f| {
        let ch = cur_rx.load(Ordering::Relaxed);
        if ch == 0 {
            return;
        }
        if let dot11::Verdict::Body(p) = dot11::parse(f.frame, f.crc_err, f.data_rate) {
            if !f.crc_err && f.frame[0] & 0xFC == 0x88 {
                hits_rx.fetch_add(1, Ordering::Relaxed);
            }
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
    let shared = cl.sink.shared.clone();
    shared.channel.store(o.ch, Ordering::Relaxed);
    let searching = !o.search.is_empty() && o.remake.is_some();
    let mut core_ch = o.ch;
    // Sweeps, then rebuilds the core when the drone is elsewhere.
    let mut find = |cl: &mut CoreLoop, core_ch: &mut u8| -> Result<bool, String> {
        let from = *core_ch;
        let Some(ch) = search(&card, &o.search, from, o.width, &cur, &hits, &rx, &shared)? else {
            return Ok(false);
        };
        if ch != *core_ch {
            let remake = o.remake.as_mut().expect("remake");
            cl.core = remake(ch)?;
            *core_ch = ch;
        }
        Ok(true)
    };
    if searching && !find(&mut cl, &mut core_ch)? {
        return Ok(());
    }
    if o.gs {
        cl.sink.card = Some(card.clone());
    }
    let mut seen = (hits.load(Ordering::Relaxed), mono_us(t0));
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
        let h = hits.load(Ordering::Relaxed);
        if h != seen.0 {
            seen = (h, now);
        } else if searching && now.saturating_sub(seen.1) >= LOST_US {
            // The core stays silent while the card sweeps: its DISC on a
            // foreign channel could move a drone that is listening there.
            cl.sink
                .shared
                .event("drone_lost", serde_json::json!({"ch": core_ch}));
            let card_tx = cl.sink.card.take();
            if !find(&mut cl, &mut core_ch)? {
                return Ok(());
            }
            cl.sink.card = card_tx;
            seen = (hits.load(Ordering::Relaxed), mono_us(t0));
        }
    }
}
