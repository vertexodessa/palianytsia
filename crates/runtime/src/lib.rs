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

pub mod dev_card;
pub mod stats_line;

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
    /// The video link's width (the ladder's bw); 0 = the receiver's width.
    /// Equal to it unless a config flies 20 MHz rungs on a 40 MHz card.
    pub video_bw: AtomicU8,
    /// Video bytes out of the core (completed AUs), for the bitrate readout.
    pub au_bytes: AtomicU64,
    /// (window start, au_bytes then, last rate) behind rx_kbps().
    rate: Mutex<Option<(Instant, u64, f64)>>,
}

impl Shared {
    /// "ch 40 / 40 MHz" (the video width), or "searching for the drone:
    /// ch 64 / 40 MHz" (the receiver's width while it sweeps).
    pub fn radio_label(&self, width: u8) -> String {
        let ch = self.channel.load(Ordering::Relaxed);
        if self.searching.load(Ordering::Relaxed) {
            return format!("searching for the drone: ch {ch} / {width} MHz");
        }
        match self.video_bw.load(Ordering::Relaxed) {
            0 => format!("ch {ch} / {width} MHz"),
            v if v == width => format!("ch {ch} / {v} MHz"),
            v => format!("ch {ch} / {v} MHz video (RX {width})"),
        }
    }

    /// Received video bitrate, kbit/s, over the last full second (UI rate).
    pub fn rx_kbps(&self) -> f64 {
        let bytes = self.au_bytes.load(Ordering::Relaxed);
        let mut r = self.rate.lock().unwrap_or_else(|p| p.into_inner());
        let (t0, b0, last) = *r.get_or_insert((Instant::now(), bytes, 0.0));
        let dt = t0.elapsed().as_secs_f64();
        if dt >= 1.0 {
            let k = bytes.saturating_sub(b0) as f64 * 8.0 / dt / 1000.0;
            *r = Some((Instant::now(), bytes, k));
            return k;
        }
        last
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
        self.shared
            .au_bytes
            .fetch_add(a.data.len() as u64, Ordering::Relaxed);
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
    ports: LoopPorts,
    lat: bool,
    // The loop's own schedule and marks; no caller sets or reads them.
    last_params_us: u64,
    last_lat_us: u64,
    rx_full_seen: u64,
    /// (frames, au drops) at the last report, for the stall warning.
    stall_mark: (u64, u64),
    last_stats_us: u64,
    last_log_us: u64,
    /// Calibration bookkeeping the core does not keep.
    cal: CalUi,
}

/// The loop's channels to the UI.
pub struct LoopPorts {
    pub stats_tx: SyncSender<String>,
    /// F18: the parameter table to the UI (2 Hz), commands and set results back.
    pub params_tx: SyncSender<String>,
    pub cmd_rx: Receiver<pal_osd::params::Cmd>,
    pub set_res_tx: SyncSender<(usize, Result<(), String>)>,
}

/// What the loop adds around the core's calibration: the last start's
/// refusal for the UI, the last state for the flight-log events, and
/// whether the current core has been told where cal.log goes.
#[derive(Default)]
struct CalUi {
    refusal: Option<String>,
    last_state: String,
    log_dir_set: bool,
}

impl CoreLoop {
    /// `lat`: the --lat latency report on stderr.
    pub fn new(core: Box<dyn GsCore>, sink: Sink, ports: LoopPorts, lat: bool) -> Self {
        Self {
            core,
            sink,
            ports,
            lat,
            last_params_us: 0,
            last_lat_us: 0,
            rx_full_seen: 0,
            stall_mark: (0, 0),
            last_stats_us: 0,
            last_log_us: 0,
            cal: CalUi::default(),
        }
    }

    /// The core clock starts over (a replay's next pass): the stats line is
    /// due at once again. Only that schedule rewinds, as before.
    pub fn restart_stats_clock(&mut self) {
        self.last_stats_us = 0;
    }

    /// Swaps in a new core (a remake after a search, the channel core's
    /// attach, a replay pass); it is told where cal.log goes at its first tick.
    pub fn set_core(&mut self, core: Box<dyn GsCore>) {
        self.core = core;
        self.cal.log_dir_set = false;
        self.cal.last_state.clear();
    }

    fn cal_cmd(&mut self, start: bool) {
        let r = if start {
            self.core.cal_start(&[])
        } else {
            self.core.cal_abort();
            Ok(())
        };
        eprintln!(
            "pal: calibration {}: {r:?}",
            if start { "start" } else { "abort" }
        );
        self.sink.shared.event(
            if start { "cal_start" } else { "cal_abort" },
            serde_json::json!({"result": match &r { Ok(()) => "ok".to_string(), Err(e) => e.clone() }}),
        );
        self.cal.refusal = r.err();
    }

    /// The core's calibration JSON with the refusal spliced in; a state
    /// change goes to the flight log (with the result table on the way out).
    fn cal_line(&mut self) -> Option<serde_json::Value> {
        let j = self.core.cal_json()?;
        let v: serde_json::Value = serde_json::from_str(&j).ok()?;
        let st = v
            .get("state")
            .and_then(|s| s.as_str())
            .unwrap_or("")
            .to_string();
        if st != self.cal.last_state {
            if !self.cal.last_state.is_empty() || st != "idle" {
                eprintln!("pal: calibration {} -> {st}", self.cal.last_state);
                self.sink.shared.event("cal", v.clone());
            }
            self.cal.last_state = st;
        }
        Some(stats_line::with_refusal(v, self.cal.refusal.as_deref()))
    }

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
        while let Ok(c) = self.ports.cmd_rx.try_recv() {
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
                    let _ = self.ports.set_res_tx.try_send((row, r));
                }
                pal_osd::params::Cmd::Refresh => self.core.param_refresh(),
                pal_osd::params::Cmd::CalStart => self.cal_cmd(true),
                pal_osd::params::Cmd::CalAbort => self.cal_cmd(false),
            }
        }
        if !self.cal.log_dir_set {
            self.cal.log_dir_set = true;
            if let Some(l) = self.sink.log.as_ref() {
                self.core.set_cal_log_dir(l.dir());
            }
        }
        self.core.tick(now, &mut self.sink);
        if now >= self.last_params_us + 500_000 {
            self.last_params_us = now;
            let _ = self.ports.params_tx.try_send(self.core.params_json(now));
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
            let stats = stats_line::parse(&line);
            self.sink.pts_off_us = stats.as_ref().and_then(stats_line::pts_off_us);
            // The ladder and calibration views go to the UI only: the
            // flight log keeps the webgs stats line both cores agree on.
            let ladder = self.core.ladder_json();
            let cal = self.cal_line();
            let ui = match stats {
                Some(s) => stats_line::ui_line(s, ladder.as_deref(), cal),
                None => line,
            };
            let _ = self.ports.stats_tx.try_send(ui);
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

/// A core that drives the card itself (mabur's C++ GS with its channel
/// core): it retunes, hops and transmits through the card, and every
/// received frame passes through it first for its counters.
pub trait RxTap: Send + Sync {
    /// Every frame with its parse verdict. None = not a body for the core;
    /// else the rx_channel to stamp (0 = mid-retune, unknown).
    fn on_rx(&self, f: &devourer_sys::RxFrame, v: &dot11::Verdict) -> Option<u8>;
    /// The channel the card is on now.
    fn channel(&self) -> u8;
}

pub struct Attached {
    pub core: Box<dyn GsCore>,
    /// One per card the core drives, in card order; a card without a tap
    /// (past the end) has its frames dropped.
    pub taps: Vec<Arc<dyn RxTap>>,
    /// The core's own clock; RX stamps and ticks use it.
    pub clock: fn() -> u64,
}

/// Builds such a core once the cards are open and receiving (card 0 is
/// `LiveCfg::select`, then `LiveCfg::extra` in order).
pub type Attach = Box<dyn FnOnce(&[Arc<devourer_sys::Card>]) -> Result<Attached, String>>;

/// Where the card comes from and how to tune it.
pub struct LiveCfg {
    pub select: devourer_sys::Select,
    pub ch: u8,
    pub width: u8,
    pub drive: Drive,
}

/// Who tunes the card(s), decided once by the app.
pub enum Drive {
    /// The runtime: the card stays on `LiveCfg::ch`, or sweeps for the
    /// drone when `search` is given. `gs`: the card also transmits the
    /// core's RC frames.
    Runtime { gs: bool, search: Option<Search> },
    /// A core that owns the cards (see `RxTap`): it finds the drone,
    /// retunes and transmits itself. `extra`: more cards next to the
    /// first (mabur's multi-card GS: every card receives, the channel core
    /// scouts on one).
    Core {
        attach: Attach,
        extra: Vec<devourer_sys::Select>,
    },
}

/// The channel set to sweep for the drone (radio.channel = "auto"): at
/// start, and again after `LOST_US` without a drone frame. An empty set =
/// stay on the channel. `remake` builds the core for the channel found: a
/// core's DISC proposes the channel it was built for, and a core on the
/// wrong one would steer the drone.
pub struct Search {
    pub set: Vec<u8>,
    pub remake: Remake,
}

type TapClock = (Vec<Arc<dyn RxTap>>, fn() -> u64);

fn mono_default() -> u64 {
    static START: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    1_000_000 + START.get_or_init(Instant::now).elapsed().as_micros() as u64
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

/// Opens the card(s) and runs the core loop on this thread until
/// `shared.quit`; `on_open` gets the chip names once the cards are up.
pub fn run_live(cl: CoreLoop, o: LiveCfg, on_open: impl FnOnce(&str)) -> Result<(), String> {
    match o.drive {
        Drive::Core { attach, extra } => {
            let mut selects = vec![o.select];
            selects.extend(extra);
            run_core_owned(cl, selects, o.ch, o.width, attach, on_open)
        }
        Drive::Runtime { gs, search } => {
            run_runtime_owned(cl, o.select, o.ch, o.width, gs, search, on_open)
        }
    }
}

/// Says what a received frame's body is stamped with: Some((rx_channel,
/// mono_us)) forwards it to the core, None drops it. Called on the card's
/// RX thread with the card's roster index.
type Stamp =
    Arc<dyn Fn(u8, &devourer_sys::RxFrame, &dot11::Verdict) -> Option<(u8, u64)> + Send + Sync>;

/// The core's view of one received frame: `f` as the card delivered it,
/// `p` its dot11 body, stamped with the card, the core clock and the
/// channel it was heard on. The one place that maps the chip's RX fields.
pub fn rx_body(
    card_id: u8,
    f: &devourer_sys::RxFrame,
    p: &dot11::Parsed,
    mono_us: u64,
    rx_channel: u8,
) -> RxBody {
    RxBody {
        card_id,
        mono_us,
        rssi: f.rssi,
        snr: f.snr,
        evm: f.evm,
        phy_valid: f.physt,
        crc_ok: !f.crc_err,
        mac_seq: p.mac_seq,
        tsfl: f.tsfl,
        mcs: p.mcs,
        rx_channel,
        body: f.frame[p.body_off..].to_vec(),
    }
}

/// Opens the cards; each RX thread hands its bodies, as `stamp` stamps
/// them, to `tx`. Card 0 must open; a further card that will not come up
/// costs diversity, not the link: it is skipped. Ids stay contiguous over
/// the cards that opened (the core's roster).
fn open_cards(
    selects: Vec<devourer_sys::Select>,
    ch: u8,
    width: u8,
    stamp: Stamp,
    tx: SyncSender<RxBody>,
    shared: &Arc<Shared>,
) -> Result<(Vec<Arc<devourer_sys::Card>>, String), String> {
    let mut cards = Vec::with_capacity(selects.len());
    let mut chips = Vec::with_capacity(selects.len());
    for (i, sel) in selects.into_iter().enumerate() {
        let mut card = match devourer_sys::Card::open(&devourer_sys::Config {
            select: sel,
            channel: ch,
            width_mhz: width,
        }) {
            Ok(c) => c,
            Err(e) if i == 0 => return Err(format!("card: {e}")),
            Err(e) => {
                eprintln!("pal: card {i} skipped: {e}");
                shared.event(
                    "card_skipped",
                    serde_json::json!({"card": i, "err": e.to_string()}),
                );
                continue;
            }
        };
        chips.push(card.chip().unwrap_or("?".into()));
        let (tx, shared, stamp) = (tx.clone(), shared.clone(), stamp.clone());
        let card_id = cards.len() as u8;
        card.start_rx(move |f| {
            let v = dot11::parse(f.frame, f.crc_err, f.data_rate);
            let Some((ch, now)) = stamp(card_id, f, &v) else {
                return;
            };
            if let dot11::Verdict::Body(p) = v {
                shared.lat.rx_sent.fetch_add(1, Ordering::Relaxed);
                let r = tx.try_send(rx_body(card_id, f, &p, now, ch));
                if r.is_err() {
                    shared.lat.rx_full.fetch_add(1, Ordering::Relaxed);
                }
            }
        })?;
        cards.push(Arc::new(card));
    }
    Ok((cards, chips.join(" + ")))
}

/// A core that owns the cards: every frame passes its card's tap first
/// (counters, the channel stamp), the core retunes and transmits itself.
fn run_core_owned(
    mut cl: CoreLoop,
    selects: Vec<devourer_sys::Select>,
    ch: u8,
    width: u8,
    attach: Attach,
    on_open: impl FnOnce(&str),
) -> Result<(), String> {
    let (tx, rx) = mpsc::sync_channel::<RxBody>(4096);
    // Set once the core exists; frames before it are dropped.
    let tap: Arc<std::sync::OnceLock<TapClock>> = Arc::default();
    let tap_rx = tap.clone();
    let stamp: Stamp = Arc::new(move |card_id, f, v| {
        let (taps, clock) = tap_rx.get()?;
        let ch = taps.get(usize::from(card_id))?.on_rx(f, v)?;
        Some((ch, clock()))
    });
    let shared = cl.sink.shared.clone();
    let (cards, chips) = open_cards(selects, ch, width, stamp, tx, &shared)?;
    on_open(&chips);
    shared.channel.store(ch, Ordering::Relaxed);
    let at = attach(&cards)?;
    cl.set_core(at.core);
    let clock = at.clock;
    // The link card's channel is the one the UI shows.
    let link = at.taps.first().cloned();
    let _ = tap.set((at.taps, at.clock));
    let r = pump(
        &mut cl,
        &rx,
        clock,
        |_, _| {
            if let Some(t) = &link {
                shared.channel.store(t.channel(), Ordering::Relaxed);
            }
        },
        |_, _| Ok(true),
    );
    // The core's threads use the cards: they stop before those close.
    drop(cl);
    drop(link);
    drop(cards);
    r
}

/// The runtime holds the card: on `ch`, or sweeping `search` for the
/// drone and rebuilding the core for the channel it is found on.
fn run_runtime_owned(
    mut cl: CoreLoop,
    select: devourer_sys::Select,
    ch: u8,
    width: u8,
    gs: bool,
    search: Option<Search>,
    on_open: impl FnOnce(&str),
) -> Result<(), String> {
    let (tx, rx) = mpsc::sync_channel::<RxBody>(4096);
    // The channel RX frames are stamped with; 0 while the card retunes.
    let cur = Arc::new(AtomicU8::new(ch));
    // CRC-good drone data frames (SA-matched by dot11::parse) since open.
    let hits = Arc::new(AtomicU32::new(0));
    let (cur_rx, hits_rx) = (cur.clone(), hits.clone());
    let stamp: Stamp = Arc::new(move |_, f, v| {
        let ch = cur_rx.load(Ordering::Relaxed);
        if ch == 0 {
            return None;
        }
        if matches!(v, dot11::Verdict::Body(_)) && !f.crc_err && f.frame[0] & 0xFC == 0x88 {
            hits_rx.fetch_add(1, Ordering::Relaxed);
        }
        Some((ch, mono_default()))
    });
    let shared = cl.sink.shared.clone();
    let (cards, chips) = open_cards(vec![select], ch, width, stamp, tx, &shared)?;
    on_open(&chips);
    shared.channel.store(ch, Ordering::Relaxed);
    let card = &cards[0];
    let clock: fn() -> u64 = mono_default;
    let mut srch = search.filter(|s| !s.set.is_empty());
    let searching = srch.is_some();
    let mut core_ch = ch;
    // Sweeps, then rebuilds the core when the drone is elsewhere.
    let mut find = |cl: &mut CoreLoop, core_ch: &mut u8| -> Result<bool, String> {
        let s = srch.as_mut().expect("searching");
        let from = *core_ch;
        let Some(ch) = self::search(card, &s.set, from, width, &cur, &hits, &rx, &shared)? else {
            return Ok(false);
        };
        if ch != *core_ch {
            let core = (s.remake)(ch)?;
            cl.set_core(core);
            *core_ch = ch;
        }
        Ok(true)
    };
    if searching && !find(&mut cl, &mut core_ch)? {
        return Ok(());
    }
    if gs {
        cl.sink.card = Some(card.clone());
    }
    let mut seen = (hits.load(Ordering::Relaxed), clock());
    pump(
        &mut cl,
        &rx,
        clock,
        |_, _| {},
        |cl, now| {
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
                if !find(cl, &mut core_ch)? {
                    return Ok(false);
                }
                cl.sink.card = card_tx;
                seen = (hits.load(Ordering::Relaxed), clock());
            }
            Ok(true)
        },
    )
}

/// Bodies into the core and its ticks on `clock` until `shared.quit`.
/// `on_tick` runs right after each tick; `after` once per wake past the
/// quit check, and ends the loop by returning false.
fn pump(
    cl: &mut CoreLoop,
    rx: &Receiver<RxBody>,
    clock: fn() -> u64,
    mut on_tick: impl FnMut(&mut CoreLoop, u64),
    mut after: impl FnMut(&mut CoreLoop, u64) -> Result<bool, String>,
) -> Result<(), String> {
    let mut next_tick = clock();
    loop {
        let wait = Duration::from_micros(next_tick.saturating_sub(clock()));
        match rx.recv_timeout(wait) {
            Ok(b) => {
                let l = &cl.sink.shared.lat;
                l.rx_recv.fetch_add(1, Ordering::Relaxed);
                l.body_lag_max
                    .fetch_max(clock().saturating_sub(b.mono_us), Ordering::Relaxed);
                cl.rx(&b)
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(mpsc::RecvTimeoutError::Disconnected) => return Err("RX thread ended".into()),
        }
        let now = clock();
        if now >= next_tick {
            cl.tick(now);
            next_tick = now + TICK_US;
            on_tick(cl, now);
        }
        if cl.sink.shared.quit.load(Ordering::Relaxed) {
            return Ok(());
        }
        if !after(cl, now)? {
            return Ok(());
        }
    }
}
