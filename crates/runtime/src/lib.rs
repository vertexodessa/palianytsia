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

pub mod cam_fps;
pub mod dev_card;
pub mod panic_hook;
pub mod rf_sim;
pub mod stats_line;

pub use panic_hook::install_panic_hook;

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
    /// The camera's frame rate x10 from AU capture stamps (`cam_fps`); 0
    /// until known. Not in the stats line: M1 compares that byte for byte.
    pub cam_fps_x10: AtomicU32,
}

impl Shared {
    /// The decoder's error total after AU `au`: on a rise, the AU first and
    /// then, with Release, the total `CoreLoop` reads with Acquire.
    pub fn note_dec_errors(&self, au: u64, total: u64) {
        if total > self.dec_errors.load(Ordering::Relaxed) {
            self.dec_err_au.store(au, Ordering::Relaxed);
            self.dec_errors.store(total, Ordering::Release);
        }
    }

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

/// One report's window counters, taken and reset together: whether the
/// report prints or not, the next second starts from zero.
struct LatWindow {
    body_lag_max: u64,
    cap_au_max: u64,
    au_dec_max: u64,
    show_max: u64,
    dec_us_sum: u64,
    dec_n: u64,
    shown: u64,
}

impl Lat {
    fn take_window(&self) -> LatWindow {
        let take = |a: &AtomicU64| a.swap(0, Ordering::Relaxed);
        LatWindow {
            body_lag_max: take(&self.body_lag_max),
            cap_au_max: take(&self.cap_au_max),
            au_dec_max: take(&self.au_dec_max),
            show_max: take(&self.show_max),
            dec_us_sum: take(&self.dec_us_sum),
            dec_n: take(&self.dec_n),
            shown: take(&self.shown),
        }
    }
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
    pub cam_fps: cam_fps::CamFps,
}

impl CoreIo for Sink {
    fn on_au(&mut self, a: Au) {
        let n = self.idr.as_mut().map_or(0, |i| {
            i.on_au(&a, self.now_us);
            i.aus()
        });
        if let Some(l) = self.log.as_mut() {
            // A failed write stops the session; `CoreLoop::log_err` tells.
            let _ = l.au(self.now_us, &a);
        }
        if let Some(off) = self.pts_off_us {
            let d = pal_core::uplink::cap_to_complete_us(a.pts_us, self.now_us, off).max(0) as u64;
            let lat = &self.shared.lat;
            lat.cap_au_max.fetch_max(d, Ordering::Relaxed);
            lat.cap_au_last.store(d, Ordering::Relaxed);
        }
        if let Some(f) = self.cam_fps.push(a.pts_us) {
            let x10 = (f * 10.0).round() as u32;
            self.shared.cam_fps_x10.store(x10, Ordering::Relaxed);
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
        if rf_sim::active().is_some_and(|s| !s.tx_passes()) {
            return;
        }
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
    /// Per-card deaf/hearing flips for the flight log.
    deaf: pal_osd::DeafWatch,
    /// Core time (ms) of the last ladder transition logged as rung_reason.
    rung_seen_ms: Option<i64>,
    /// The flight log's write error has been reported.
    log_err_told: bool,
    /// The drone's parameter table as last read, for the flight log.
    params: ParamLog,
    /// The cards the core drives, in card order (`watch_cards`).
    taps: Vec<Arc<dyn RxTap>>,
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
        warn_low_log_space(&sink);
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
            deaf: pal_osd::DeafWatch::default(),
            rung_seen_ms: None,
            log_err_told: false,
            params: ParamLog::default(),
            taps: Vec::new(),
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
            serde_json::json!({"result": result_text(&r, "ok")}),
        );
        self.cal.refusal = r.err();
    }

    /// The core's calibration JSON with the refusal spliced in; a state
    /// change goes to the flight log (with the result table on the way out).
    /// The ladder's last transition with its reason, once per transition:
    /// the stats line has the rung but not why it moved (a demote with no
    /// loss in sight was unexplained, 2026-11-03). Sampled with the stats
    /// line, so of two moves within one tick only the second shows.
    fn log_rung_reason(&mut self, now_us: u64, v: &serde_json::Value) {
        let (Some(age), Some(reason)) = (
            v.get("last_age_ms").and_then(serde_json::Value::as_i64),
            v.get("last_reason").and_then(serde_json::Value::as_str),
        ) else {
            return;
        };
        let at_ms = ((now_us / 1000) as i64).saturating_sub(age);
        // The age is rounded: the same transition reads up to 1 ms apart.
        if self
            .rung_seen_ms
            .is_some_and(|seen| at_ms.abs_diff(seen) <= 2)
        {
            return;
        }
        self.rung_seen_ms = Some(at_ms);
        let g = |k: &str| v.get(k).cloned().unwrap_or(serde_json::Value::Null);
        self.sink.shared.event(
            "rung_reason",
            serde_json::json!({"reason": reason, "from": g("last_from"), "to": g("last_to"),
                "age_ms": age, "u": g("u"), "probe": g("probe"), "probe_u": g("probe_u")}),
        );
    }

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
    /// One pass of the core loop: each step does what is due at `now`.
    pub fn tick(&mut self, now: u64) {
        self.sink.now_us = now;
        self.drain_mav_udp();
        self.drain_events(now);
        self.feed_decoder_errors(now);
        while let Ok(c) = self.ports.cmd_rx.try_recv() {
            self.apply_cmd(c);
        }
        self.hand_cal_log_dir();
        self.core.tick(now, &mut self.sink);
        self.log_chan_records(now);
        self.log_fec_rows();
        self.publish_params(now);
        self.step_idr(now);
        self.publish_stats(now);
        self.report_lat(now);
    }

    fn drain_mav_udp(&mut self) {
        if let Some((sock, _)) = &self.sink.udp {
            let mut buf = [0u8; 2048];
            while let Ok((n, _)) = sock.recv_from(&mut buf) {
                self.core.mav_push_outbound(&buf[..n]);
            }
        }
    }

    fn drain_events(&mut self, now: u64) {
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
    }

    fn feed_decoder_errors(&mut self, now: u64) {
        if let Some(i) = self.sink.idr.as_mut() {
            // Errors first (Acquire pairs with the decoder's Release): the AU
            // number read after it is the same rise's or a newer one.
            let sh = &self.sink.shared;
            let e = sh.dec_errors.load(Ordering::Acquire);
            i.on_decoder_errors(e, sh.dec_err_au.load(Ordering::Relaxed), now);
        }
    }

    fn apply_cmd(&mut self, c: pal_osd::params::Cmd) {
        match c {
            pal_osd::params::Cmd::Set(row, text) => {
                let r = self.core.param_set(row, &text);
                eprintln!("pal: param set {row} {text:?}: {r:?}");
                // Android has no stderr: the flight log keeps it. Row
                // numbers move between releases; the id does not.
                let (id, prev) = self.params.row(row);
                self.sink.shared.event(
                    "param_set",
                    serde_json::json!({"row": row, "id": id, "prev": prev, "value": text,
                        "result": result_text(&r, "sent")}),
                );
                let _ = self.ports.set_res_tx.try_send((row, r));
            }
            pal_osd::params::Cmd::Refresh => self.core.param_refresh(),
            pal_osd::params::Cmd::CalStart => self.cal_cmd(true),
            pal_osd::params::Cmd::CalAbort => self.cal_cmd(false),
            pal_osd::params::Cmd::ChannelScan(chs) => {
                let r = self.core.chan_scan(&chs);
                eprintln!("pal: channel scan {chs:?}: {r:?}");
            }
            pal_osd::params::Cmd::Channels(chs) => {
                let r = self.core.set_channels(&chs);
                eprintln!("pal: channels {chs:?}: {r:?}");
                self.sink.shared.event(
                    "channels",
                    serde_json::json!({"channels": chs, "result": result_text(&r, "ok")}),
                );
            }
        }
    }

    fn hand_cal_log_dir(&mut self) {
        if !self.cal.log_dir_set {
            self.cal.log_dir_set = true;
            if let Some(l) = self.sink.log.as_ref() {
                self.core.set_cal_log_dir(l.dir());
            }
        }
    }

    /// The channel core's records into the flight log, each under its own
    /// event name; dropped unread when there is no log.
    /// Taken every tick with or without a log, like the channel records,
    /// so the core's queue stays short.
    fn log_fec_rows(&mut self) {
        let rows = self.core.take_fec_rows();
        if let Some(l) = self.sink.log.as_mut() {
            let _ = l.fec_rows(&rows);
        }
    }

    fn log_chan_records(&mut self, now: u64) {
        let recs = self.core.take_chan_records();
        let Some(l) = self.sink.log.as_mut() else {
            return;
        };
        for r in recs {
            if let Ok(serde_json::Value::Object(mut m)) = serde_json::from_str(&r) {
                let ev = m.remove("ev");
                let ev = ev
                    .as_ref()
                    .and_then(serde_json::Value::as_str)
                    .unwrap_or("chan");
                let _ = l.event(now, ev, m.into());
            }
        }
    }

    fn publish_params(&mut self, now: u64) {
        if now >= self.last_params_us + 500_000 {
            self.last_params_us = now;
            let j = self.core.params_json(now);
            if let Some(values) = self.params.note(&j) {
                self.sink.shared.event("params", values);
            }
            let _ = self.ports.params_tx.try_send(j);
        }
    }

    fn step_idr(&mut self, now: u64) {
        if let Some(i) = self.sink.idr.as_mut() {
            let evs = i.step(now, &mut *self.core);
            if let Some(l) = self.sink.log.as_mut() {
                let _ = l.refresh_events(&evs);
            }
        }
    }

    /// 10 Hz to the UI, 1 Hz to the flight log.
    fn publish_stats(&mut self, now: u64) {
        if now < self.last_stats_us + 100_000 {
            return;
        }
        self.last_stats_us = now;
        let line = self.core.stats_json();
        let stats = stats_line::parse(&line);
        let off = self.card_off_us(stats.as_ref());
        if self.sink.log.is_some() && now >= self.last_log_us + 1_000_000 {
            self.last_log_us = now;
            self.log_stats(now, &line, &off);
            if let Some(s) = rf_sim::active() {
                self.sink.shared.event("rf_sim", s.snapshot());
            }
        }
        self.sink.pts_off_us = stats.as_ref().and_then(stats_line::pts_off_us);
        // The ladder and calibration views go to the UI only: the
        // flight log keeps the webgs stats line both cores agree on.
        let json = |t: Option<String>| t.and_then(|t| serde_json::from_str(&t).ok());
        let ladder = json(self.core.ladder_json());
        if let Some(l) = &ladder {
            self.log_rung_reason(now, l);
        }
        let scan = json(self.core.chan_scan_json());
        let cal = self.cal_line();
        let log_err = self.log_err();
        let off = (!off.is_empty()).then(|| off.into());
        let ui = match stats {
            Some(s) => stats_line::ui_line(
                s,
                [
                    ("ladder", ladder),
                    ("chscan", scan),
                    ("log_err", log_err),
                    ("card_off_us", off),
                    ("rf_sim", rf_sim::active().map(|s| s.snapshot())),
                ],
                cal,
            ),
            None => line,
        };
        let _ = self.ports.stats_tx.try_send(ui);
    }

    /// The error that stopped the flight log, for the UI line. The write
    /// calls' own results are dropped on purpose: the session latches the
    /// first error and this reads it once per stats pass. Told once (stderr
    /// and an event); the log itself can no longer take it.
    fn log_err(&mut self) -> Option<serde_json::Value> {
        let text = self.sink.log.as_ref()?.write_error()?.to_string();
        if !self.log_err_told {
            self.log_err_told = true;
            eprintln!("pal: flight log stopped: {text}");
            self.sink
                .shared
                .event("log_error", serde_json::json!({ "error": text }));
        }
        Some(text.into())
    }

    /// Each card's time off the transmitting card's channel, from the
    /// cards' taps; empty for a core that does not drive the cards.
    fn card_off_us(&self, stats: Option<&serde_json::Value>) -> Vec<u64> {
        let tx = stats
            .and_then(|s| s.get("tx_card"))
            .and_then(serde_json::Value::as_u64)
            .map_or(0, |t| t as usize);
        self.taps.iter().map(|t| t.off_link_us(tx)).collect()
    }

    /// The cards a core drives (`run_core_on`), for their time off the
    /// link channel.
    pub fn watch_cards(&mut self, taps: Vec<Arc<dyn RxTap>>) {
        self.taps = taps;
    }

    /// The flight log's stats line, then the deaf-card check it feeds.
    fn log_stats(&mut self, now: u64, line: &str, off: &[u64]) {
        if let Some(l) = self.sink.log.as_mut() {
            let _ = l.stats(now, line);
            // A killed app (Android swipe, OOM) skips Drop: at most
            // one second of log is lost, a few writes a second.
            let _ = l.flush();
        }
        let mut cards = serde_json::from_str(line).map_or(Vec::new(), |v| pal_osd::cards_of(&v));
        for (c, &o) in cards.iter_mut().zip(off) {
            c.off_link_us = o;
        }
        for c in self.deaf.push(now as f64 / 1e6, &cards) {
            let ev = if c.deaf { "card_deaf" } else { "card_hearing" };
            eprintln!(
                "pal: card {} {ev}: {:.1} frames/s, best card {:.0}",
                c.card, c.fps, c.best_fps
            );
            self.sink.shared.event(
                ev,
                serde_json::json!({"card": c.card, "fps": (c.fps * 10.0).round() / 10.0,
                    "best_fps": c.best_fps.round()}),
            );
        }
    }

    /// Once a second. Without --lat the line still prints when a stage goes
    /// bad, so an intermittent multi-second lag leaves its breakdown behind.
    fn report_lat(&mut self, now: u64) {
        if now < self.last_lat_us + 1_000_000 {
            return;
        }
        self.last_lat_us = now;
        let sh = &self.sink.shared;
        let l = &sh.lat;
        let frames = sh.frames.load(Ordering::Relaxed);
        let au_drops = sh.au_dropped.load(Ordering::Relaxed);
        // A decoder that stopped returning (a hung hwaccel driver) shows
        // as AUs dropping while the frame count stands still.
        if au_drops > self.stall_mark.1 && frames == self.stall_mark.0 {
            eprintln!(
                "pal: decoder stalled ({frames} frames, AUs dropping); a hung hwaccel driver? try --hwdec sw or another backend"
            );
        }
        self.stall_mark = (frames, au_drops);
        let full = l.rx_full.load(Ordering::Relaxed);
        let w = l.take_window();
        let log_max = self.sink.log.as_mut().map_or(0, Session::take_call_max_us);
        let bad = w.body_lag_max > 200_000
            || log_max > 20_000
            || w.cap_au_max > 500_000
            || w.au_dec_max > 300_000
            || w.show_max > 300_000
            || full > self.rx_full_seen;
        self.rx_full_seen = full;
        if !self.lat && !bad {
            return;
        }
        let (sent, recv) = (
            l.rx_sent.load(Ordering::Relaxed),
            l.rx_recv.load(Ordering::Relaxed),
        );
        eprintln!(
            "lat: rx_queue={} rx_full={full} body_lag_max={}ms cap_au={}ms(max {}) au_to_frame_max={}ms dec_avg={:.1}ms dec_holds={} frame_to_screen_max={}ms shown/s={} au_drops={au_drops} rgba_skipped={} log_write_max={:.1}ms",
            sent.saturating_sub(recv + full),
            w.body_lag_max / 1000,
            l.cap_au_last.load(Ordering::Relaxed) / 1000,
            w.cap_au_max / 1000,
            w.au_dec_max / 1000,
            w.dec_us_sum as f64 / w.dec_n.max(1) as f64 / 1000.0,
            l.aus_in.load(Ordering::Relaxed).saturating_sub(frames + au_drops),
            w.show_max / 1000,
            w.shown,
            l.skipped_rgba.load(Ordering::Relaxed),
            log_max as f64 / 1000.0,
        );
    }
}

/// The drone's parameter table for the flight log. Without it a session
/// does not say what the drone ran (encoder floor, power mode, FEC
/// window), and an A/B read from two sessions credits the wrong change.
#[derive(Default)]
struct ParamLog {
    /// (id, value) per table row, from the last `params_json`.
    rows: Vec<(String, Option<String>)>,
    /// The {id: value} map last written; None until the next listing.
    logged: Option<serde_json::Map<String, serde_json::Value>>,
}

impl ParamLog {
    /// Reads one `params_json`. Returns the snapshot to log: the first
    /// full read after a listing (each link-up re-lists), then whenever a
    /// value differs from the last one written.
    fn note(&mut self, json: &str) -> Option<serde_json::Value> {
        let v: serde_json::Value = serde_json::from_str(json).ok()?;
        let s = |x: &serde_json::Value| x.as_str().map(str::to_string);
        self.rows = v
            .get("rows")?
            .as_array()?
            .iter()
            .map(|r| (s(&r["id"]).unwrap_or_default(), s(&r["value"])))
            .collect();
        let list = v.get("list").and_then(serde_json::Value::as_str);
        if !matches!(list, Some("complete" | "partial")) {
            // Unlinked or listing: the next table is a new snapshot.
            self.logged = None;
            return None;
        }
        let values: serde_json::Map<_, _> = self
            .rows
            .iter()
            .filter_map(|(id, v)| Some((id.clone(), v.clone()?.into())))
            .collect();
        if values.is_empty() || self.logged.as_ref() == Some(&values) {
            return None;
        }
        self.logged = Some(values.clone());
        Some(serde_json::json!({"list": list, "values": values}))
    }

    /// Row `row`'s id and its value before a set.
    fn row(&self, row: usize) -> (Option<&str>, Option<&str>) {
        self.rows
            .get(row)
            .map_or((None, None), |(id, v)| (Some(id.as_str()), v.as_deref()))
    }
}

/// A command's outcome for the flight log.
fn result_text(r: &Result<(), String>, ok: &str) -> String {
    match r {
        Ok(()) => ok.to_string(),
        Err(e) => e.clone(),
    }
}

/// Warns once at start when the log's volume has less room than the
/// capture cap: a full disk stops the whole log, not only the capture.
fn warn_low_log_space(sink: &Sink) {
    let Some(l) = &sink.log else { return };
    let cap = l.capture_cap();
    let Some(free) = free_bytes(l.dir()) else {
        return;
    };
    if cap > 0 && free < cap {
        eprintln!(
            "pal: flight log volume has {} MB free, less than the {} MB capture cap",
            free >> 20,
            cap >> 20
        );
        sink.shared.event(
            "log_space",
            serde_json::json!({"free_bytes": free, "capture_cap": cap}),
        );
    }
}

/// Bytes an unprivileged writer may still use on the volume holding `p`.
fn free_bytes(p: &std::path::Path) -> Option<u64> {
    use std::os::unix::ffi::OsStrExt;
    let c = std::ffi::CString::new(p.as_os_str().as_bytes()).ok()?;
    // SAFETY: statvfs is plain old data that the call fills in; zeroed is a
    // valid value of it.
    let mut st: libc::statvfs = unsafe { std::mem::zeroed() };
    // SAFETY: `c` is NUL-terminated and outlives the call; `st` is writable.
    if unsafe { libc::statvfs(c.as_ptr(), &mut st) } != 0 {
        return None;
    }
    // Both fields are 32-bit on 32-bit targets.
    #[allow(clippy::useless_conversion)]
    Some(u64::from(st.f_bavail).saturating_mul(u64::from(st.f_frsize)))
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
    /// Microseconds this card has spent on another channel than card
    /// `link` (or mid-retune) since the core attached; 0 where unknown.
    fn off_link_us(&self, _link: usize) -> u64 {
        0
    }
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
    /// The runtime keeps the card on `LiveCfg::ch`. `gs`: the card also
    /// transmits the core's RC frames.
    Pinned { gs: bool },
    /// The runtime sweeps for the drone; the core never transmits.
    Sweep(Search),
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
/// start, and again after `LOST_US` without a drone frame. `remake` builds the core for the channel found: a
/// core's DISC proposes the channel it was built for, and a core on the
/// wrong one would steer the drone.
pub struct Search {
    pub set: Vec<u8>,
    pub remake: Remake,
}

type TapClock = (Vec<Arc<dyn RxTap>>, fn() -> u64);

pub fn mono_default() -> u64 {
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
/// Passes over the set before the sweep also listens on the rest of the
/// 5 GHz grid: a drone follows a GS to any grid channel and parks there on
/// boot, as in the channel core's search (mabur 39039eac).
pub const GRID_AFTER_ROUNDS: u32 = 2;

mod sweep;

/// Sweeps `set` (starting at `first`) until `hits` grows by FOUND_HITS
/// within one dwell. Bodies queued meanwhile are dropped: none may reach a
/// core built for another channel. None when asked to quit. `tuned` is the
/// channel and width the card is on, kept between sweeps.
#[allow(clippy::too_many_arguments)]
fn search(
    io: &mut impl sweep::SweepIo,
    set: &[u8],
    first: u8,
    width: u8,
    tuned: &mut (u8, u8),
    cur: &AtomicU8,
    hits: &AtomicU32,
    shared: &Shared,
) -> Result<Option<u8>, String> {
    let mut order = vec![first];
    order.extend(set.iter().copied().filter(|&c| c != first));
    shared.searching.store(true, Ordering::Relaxed);
    let t = io.now_us();
    let mut rounds = 0u32;
    // pump's count: what the sweep takes off the queue must not read as a
    // backlog on the latency line.
    let taken = || shared.lat.rx_recv.fetch_add(1, Ordering::Relaxed);
    let found = 'sweep: loop {
        for &ch in &order {
            if shared.quit.load(Ordering::Relaxed) {
                break 'sweep None;
            }
            // Blind the RX stamp across the move, as maburgs' retune does.
            cur.store(0, Ordering::Relaxed);
            match dev_card::retune_plan(width, tuned.1, tuned.0, ch) {
                Some(w) => {
                    io.set_monitor(ch, w)?;
                    tuned.1 = w;
                }
                None if io.fast_retune(ch) => {}
                None => return Err(format!("fast_retune({ch}) failed")),
            }
            tuned.0 = ch;
            shared.channel.store(ch, Ordering::Relaxed);
            // Frames already in the USB pipeline belong to the old channel.
            io.sleep(Duration::from_millis(5));
            while io.try_take() {
                taken();
            }
            let h0 = hits.load(Ordering::Relaxed);
            cur.store(ch, Ordering::Relaxed);
            let end = io.now_us() + DWELL_MS * 1000;
            while io.now_us() < end {
                if io.take_within(Duration::from_millis(20)) {
                    taken();
                }
                if hits.load(Ordering::Relaxed).wrapping_sub(h0) >= FOUND_HITS {
                    break 'sweep Some(ch);
                }
            }
        }
        rounds += 1;
        if rounds == 1 {
            shared.event("search", serde_json::json!({"set": set, "first": first}));
        }
        if rounds == GRID_AFTER_ROUNDS {
            let grid: Vec<u8> = pal_wire::ht40::grid_5g(width)
                .into_iter()
                .filter(|c| !order.contains(c))
                .collect();
            shared.event("search_grid", serde_json::json!({"channels": grid.len()}));
            order.extend(grid);
        }
    };
    shared.searching.store(false, Ordering::Relaxed);
    if let Some(ch) = found {
        shared.event(
            "drone_found",
            serde_json::json!({"ch": ch, "ms": (io.now_us() - t) / 1000, "rounds": rounds}),
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
            let set = CardSet::open(selects, o.ch, o.width)?;
            on_open(set.chips());
            run_core_on(cl, &set, o.ch, o.width, attach)
        }
        Drive::Pinned { gs } => {
            let set = CardSet::open(vec![o.select], o.ch, o.width)?;
            on_open(set.chips());
            run_runtime_owned(cl, &set, o.ch, o.width, gs, None)
        }
        Drive::Sweep(search) => {
            let set = CardSet::open(vec![o.select], o.ch, o.width)?;
            on_open(set.chips());
            run_runtime_owned(cl, &set, o.ch, o.width, false, Some(search))
        }
    }
}

/// Says what a received frame's body is stamped with: Some((rx_channel,
/// mono_us)) forwards it to the core, None drops it. Called on the card's
/// RX thread with the card's roster index.
pub type Stamp =
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

/// Where the cards' RX threads deliver while a session runs: its body
/// channel, its stamp, its counters. None between sessions: frames drop.
struct Route {
    tx: SyncSender<RxBody>,
    stamp: Stamp,
    shared: Arc<Shared>,
}

/// Open cards that outlive a session. Opening a card is the slow part of a
/// start (an RTL8822E takes ~4.8 s on the Quest); a session that only
/// changes settings (the video width) runs on the cards the last one left,
/// retuned to its own channel and width. Ids are contiguous over the cards
/// that opened (the core's roster).
pub struct CardSet {
    cards: Vec<Arc<devourer_sys::Card>>,
    chips: String,
    /// Further cards that would not open: (select index, error).
    skipped: Vec<(usize, String)>,
    route: Arc<std::sync::RwLock<Option<Route>>>,
    /// Sessions run so far: the first finds the cards where open put them.
    sessions: AtomicU32,
}

impl CardSet {
    /// Card 0 must open; a further card that will not come up costs
    /// diversity, not the link: it is skipped.
    pub fn open(selects: Vec<devourer_sys::Select>, ch: u8, width: u8) -> Result<Self, String> {
        let route: Arc<std::sync::RwLock<Option<Route>>> = Arc::default();
        let mut cards = Vec::with_capacity(selects.len());
        let mut chips = Vec::with_capacity(selects.len());
        let mut skipped = Vec::new();
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
                    skipped.push((i, e.to_string()));
                    continue;
                }
            };
            chips.push(card.chip().unwrap_or("?".into()));
            let route = route.clone();
            let card_id = cards.len() as u8;
            card.start_rx(move |f| {
                let r = route.read().unwrap_or_else(|p| p.into_inner());
                let Some(r) = r.as_ref() else { return };
                let v = dot11::parse(f.frame, f.crc_err, f.data_rate);
                // Bench RF simulator: before the stamp, so the channel
                // core's tap and the core both see the simulated air.
                let mut buf = Vec::new();
                let g;
                let (f, v) = match rf_sim::active() {
                    Some(sim) => {
                        let body = match &v {
                            dot11::Verdict::Body(p) => Some((p.body_off, p.mcs)),
                            _ => None,
                        };
                        let Some(x) = sim.rx(f, body, &mut buf) else {
                            return;
                        };
                        g = x;
                        (&g, dot11::parse(g.frame, g.crc_err, g.data_rate))
                    }
                    None => (f, v),
                };
                let Some((ch, now)) = (r.stamp)(card_id, f, &v) else {
                    return;
                };
                if let dot11::Verdict::Body(p) = v {
                    r.shared.lat.rx_sent.fetch_add(1, Ordering::Relaxed);
                    if r.tx.try_send(rx_body(card_id, f, &p, now, ch)).is_err() {
                        r.shared.lat.rx_full.fetch_add(1, Ordering::Relaxed);
                    }
                }
            })?;
            cards.push(Arc::new(card));
        }
        Ok(CardSet {
            cards,
            chips: chips.join(" + "),
            skipped,
            route,
            sessions: AtomicU32::new(0),
        })
    }

    /// "8822e + 8812a": the chips that opened, in roster order.
    pub fn cards(&self) -> &[Arc<devourer_sys::Card>] {
        &self.cards
    }

    pub fn chips(&self) -> &str {
        &self.chips
    }

    /// Starts a session: its card chips and skipped-card events, the cards on (`ch`,
    /// `width`) when an earlier session left them elsewhere, and the RX
    /// threads delivering to `tx` as `stamp` says until the guard drops.
    pub fn begin(
        &self,
        ch: u8,
        width: u8,
        stamp: Stamp,
        tx: SyncSender<RxBody>,
        shared: &Arc<Shared>,
    ) -> Result<SessionRoute<'_>, String> {
        shared.event("cards", serde_json::json!({"chips": self.chips}));
        for (i, e) in &self.skipped {
            shared.event("card_skipped", serde_json::json!({"card": i, "err": e}));
        }
        if self.sessions.fetch_add(1, Ordering::Relaxed) > 0 {
            for (i, c) in self.cards.iter().enumerate() {
                c.ctl()
                    .set_monitor(ch, width)
                    .map_err(|e| format!("card {i}: {e}"))?;
            }
        }
        *self.route.write().unwrap_or_else(|p| p.into_inner()) = Some(Route {
            tx,
            stamp,
            shared: shared.clone(),
        });
        Ok(SessionRoute(self))
    }
}

/// A session's hold on the cards' RX: dropped, frames drop until the next
/// session begins.
pub struct SessionRoute<'a>(&'a CardSet);

impl Drop for SessionRoute<'_> {
    fn drop(&mut self) {
        *self.0.route.write().unwrap_or_else(|p| p.into_inner()) = None;
    }
}

/// A core that owns the cards: every frame passes its card's tap first
/// (counters, the channel stamp), the core retunes and transmits itself.
/// `set` is the caller's: it stays open when this returns, for the next
/// session (`run_live` drops it at once).
pub fn run_core_on(
    mut cl: CoreLoop,
    set: &CardSet,
    ch: u8,
    width: u8,
    attach: Attach,
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
    let _route = set.begin(ch, width, stamp, tx, &shared)?;
    shared.channel.store(ch, Ordering::Relaxed);
    let at = attach(&set.cards)?;
    cl.set_core(at.core);
    cl.watch_cards(at.taps.clone());
    let clock = at.clock;
    // The link card's channel is the one the UI shows.
    let link = at.taps.first().cloned();
    let _ = tap.set((at.taps, at.clock));
    let r = pump(
        &mut cl,
        &rx,
        clock,
        &shared,
        |_, _| {
            if let Some(t) = &link {
                shared.channel.store(t.channel(), Ordering::Relaxed);
            }
        },
        |_, _| Ok(true),
    );
    // The core's threads use the cards: they stop before the next session
    // (or the set's owner) takes them.
    drop(cl);
    drop(link);
    r
}

/// The runtime holds the card: on `ch`, or sweeping `search` for the
/// drone and rebuilding the core for the channel it is found on.
fn run_runtime_owned(
    mut cl: CoreLoop,
    set: &CardSet,
    ch: u8,
    width: u8,
    gs: bool,
    search: Option<Search>,
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
    let _route = set.begin(ch, width, stamp, tx, &shared)?;
    shared.channel.store(ch, Ordering::Relaxed);
    let card = &set.cards[0];
    let clock: fn() -> u64 = mono_default;
    let mut srch = search;
    let searching = srch.is_some();
    let mut core_ch = ch;
    let mut tuned = (ch, width);
    // Sweeps, then rebuilds the core when the drone is elsewhere.
    let mut find = |cl: &mut CoreLoop, core_ch: &mut u8| -> Result<bool, String> {
        let s = srch.as_mut().expect("searching");
        let from = *core_ch;
        let mut io = sweep::LiveSweep { card, rx: &rx };
        let Some(ch) = self::search(
            &mut io, &s.set, from, width, &mut tuned, &cur, &hits, &shared,
        )?
        else {
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
    let mut seen = sweep::Heard::new(hits.load(Ordering::Relaxed), clock());
    let r = pump(
        &mut cl,
        &rx,
        clock,
        &shared,
        |_, _| {},
        |cl, now| {
            if seen.lost(hits.load(Ordering::Relaxed), now) && searching {
                cl.sink
                    .shared
                    .event("drone_lost", serde_json::json!({"ch": core_ch}));
                if !find(cl, &mut core_ch)? {
                    return Ok(false);
                }
                seen = sweep::Heard::new(hits.load(Ordering::Relaxed), clock());
            }
            Ok(true)
        },
    );
    drop(cl);
    r
}

/// What `pump` drives: the bodies off the queue and the 2 ms ticks.
pub trait BodyLoop {
    fn rx(&mut self, b: &RxBody);
    fn tick(&mut self, now: u64);
}

impl BodyLoop for CoreLoop {
    fn rx(&mut self, b: &RxBody) {
        CoreLoop::rx(self, b);
    }
    fn tick(&mut self, now: u64) {
        CoreLoop::tick(self, now);
    }
}

/// Bodies into `l` and its ticks on `clock` until `shared.quit`.
/// `on_tick` runs right after each tick; `after` once per wake past the
/// quit check, and ends the loop by returning false.
pub fn pump<L: BodyLoop>(
    l: &mut L,
    rx: &Receiver<RxBody>,
    clock: fn() -> u64,
    shared: &Shared,
    mut on_tick: impl FnMut(&mut L, u64),
    mut after: impl FnMut(&mut L, u64) -> Result<bool, String>,
) -> Result<(), String> {
    let mut next_tick = clock();
    loop {
        let wait = Duration::from_micros(next_tick.saturating_sub(clock()));
        match rx.recv_timeout(wait) {
            Ok(b) => {
                shared.lat.rx_recv.fetch_add(1, Ordering::Relaxed);
                shared
                    .lat
                    .body_lag_max
                    .fetch_max(clock().saturating_sub(b.mono_us), Ordering::Relaxed);
                l.rx(&b)
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(mpsc::RecvTimeoutError::Disconnected) => return Err("RX thread ended".into()),
        }
        let now = clock();
        if now >= next_tick {
            l.tick(now);
            next_tick = now + TICK_US;
            on_tick(l, now);
        }
        if shared.quit.load(Ordering::Relaxed) {
            return Ok(());
        }
        if !after(l, now)? {
            return Ok(());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    fn table(list: &str, min_iqp: Option<&str>) -> String {
        serde_json::json!({"list": list, "rows": [
            {"id": "RADIO_PWR_MODE", "value": "offset"},
            {"id": "VENC_MIN_IQP", "value": min_iqp},
        ]})
        .to_string()
    }

    /// The table is logged once per listing and again when a value moves;
    /// a set names the parameter and what it was before.
    #[test]
    fn the_drone_parameters_are_logged_per_listing_and_on_change() {
        let mut p = ParamLog::default();
        assert_eq!(p.note(&table("pending", None)), None, "still listing");
        let first = p.note(&table("complete", Some("34"))).expect("first table");
        assert_eq!(first["values"]["VENC_MIN_IQP"], "34");
        assert_eq!(first["values"]["RADIO_PWR_MODE"], "offset");
        assert_eq!(p.note(&table("complete", Some("34"))), None, "same values");
        assert_eq!(p.row(1), (Some("VENC_MIN_IQP"), Some("34")));
        assert_eq!(p.row(9), (None, None));
        let moved = p
            .note(&table("complete", Some("42")))
            .expect("a value moved");
        assert_eq!(moved["values"]["VENC_MIN_IQP"], "42");
        // Link lost and found: the next listing logs the table again.
        assert_eq!(p.note(&table("none", None)), None);
        assert!(p.note(&table("complete", Some("42"))).is_some());
        assert_eq!(p.note("not json"), None);
    }

    /// The formula Sink computed inline before it called
    /// pal_core::uplink::cap_to_complete_us: kept to prove the two agree.
    fn inline_before(now_us: u64, off: i64, pts: u32) -> u64 {
        let cap = (now_us as i64 + off) as u32;
        (cap.wrapping_sub(pts) as i32).max(0) as u64
    }

    #[test]
    fn cap_latency_matches_the_formula_it_replaced() {
        let nows = [
            1_000_000u64,
            5_123_456_789,
            u64::from(u32::MAX) - 3,
            1 << 40,
        ];
        let offs = [0i64, -123_456_789, 987_654_321, -(1 << 33), 1 << 33];
        let ptss = [0u32, 1, 0x7fff_ffff, 0x8000_0000, u32::MAX, 123_456_789];
        for &n in &nows {
            for &o in &offs {
                for &p in &ptss {
                    let lib = pal_core::uplink::cap_to_complete_us(p, n, o).max(0) as u64;
                    assert_eq!(lib, inline_before(n, o, p), "now {n} off {o} pts {p}");
                }
            }
        }
    }

    /// A volume that takes `left` more bytes over all the session's line
    /// files, then fails every write as a full disk would.
    #[derive(Default)]
    struct Disk {
        left: u64,
        failed: bool,
        calls_after: u64,
    }

    struct FailAfter(Arc<Mutex<Disk>>, flightlog::session::LineWriter);

    impl Write for FailAfter {
        fn write(&mut self, b: &[u8]) -> std::io::Result<usize> {
            let mut d = self.0.lock().unwrap();
            if d.failed {
                d.calls_after += 1;
            }
            if d.failed || (b.len() as u64) > d.left {
                d.failed = true;
                return Err(std::io::Error::from_raw_os_error(libc::ENOSPC));
            }
            d.left -= b.len() as u64;
            self.1.write(b)
        }
        fn flush(&mut self) -> std::io::Result<()> {
            let mut d = self.0.lock().unwrap();
            if d.failed {
                d.calls_after += 1;
                return Err(std::io::Error::from_raw_os_error(libc::ENOSPC));
            }
            self.1.flush()
        }
    }

    /// One AU per tick and a stats line, like a live session.
    struct AuCore;

    impl GsCore for AuCore {
        fn on_rx(&mut self, _: &RxBody, _: &mut dyn CoreIo) {}
        fn tick(&mut self, _: u64, io: &mut dyn CoreIo) {
            io.on_au(Au {
                data: vec![0; 100],
                complete: true,
                ..Au::default()
            });
        }
        fn stats_json(&self) -> String {
            r#"{"session":true,"rung":0}"#.into()
        }
        fn link_op(&self) -> (i32, i32) {
            (0, 20)
        }
    }

    /// A CoreLoop with every port's far end held, so sends land.
    struct Rig {
        cl: CoreLoop,
        shared: Arc<Shared>,
        stats_rx: mpsc::Receiver<String>,
        _far: Box<dyn std::any::Any>,
    }

    fn rig(core: Box<dyn GsCore>, log: Option<Session>) -> Rig {
        let (au_tx, au_rx) = mpsc::sync_channel(1 << 16);
        let (mav_tx, mav_rx) = mpsc::sync_channel(1);
        let shared = Arc::new(Shared::default());
        let sink = Sink {
            au_tx,
            pts_off_us: None,
            mav_tx,
            udp: None,
            card: None,
            log,
            now_us: 0,
            shared: shared.clone(),
            idr: None,
            cam_fps: Default::default(),
        };
        let (stats_tx, stats_rx) = mpsc::sync_channel(1 << 12);
        let (params_tx, params_rx) = mpsc::sync_channel(1 << 12);
        let (cmd_tx, cmd_rx) = mpsc::sync_channel(1);
        let (set_res_tx, set_res_rx) = mpsc::sync_channel(1);
        let ports = LoopPorts {
            stats_tx,
            params_tx,
            cmd_rx,
            set_res_tx,
        };
        Rig {
            cl: CoreLoop::new(core, sink, ports, false),
            shared,
            stats_rx,
            _far: Box::new((au_rx, mav_rx, params_rx, cmd_tx, set_res_rx)),
        }
    }

    /// A core whose ladder moved at each of `moves` (core ms). The age it
    /// reports wobbles by 1 ms, as the rounded age of the real core does.
    struct LadderCore {
        now_ms: i64,
        moves: Vec<i64>,
    }

    impl GsCore for LadderCore {
        fn on_rx(&mut self, _: &RxBody, _: &mut dyn CoreIo) {}
        fn tick(&mut self, now_us: u64, _: &mut dyn CoreIo) {
            self.now_ms = (now_us / 1000) as i64;
        }
        fn stats_json(&self) -> String {
            r#"{"session":true,"rung":0}"#.into()
        }
        fn link_op(&self) -> (i32, i32) {
            (0, 20)
        }
        fn ladder_json(&self) -> Option<String> {
            let Some(at) = self.moves.iter().rev().find(|&&m| m <= self.now_ms) else {
                return Some(r#"{"rung":0,"top":4}"#.into());
            };
            let age = self.now_ms - at + (self.now_ms / 100) % 2;
            Some(format!(
                r#"{{"rung":1,"top":4,"last_reason":"demote_loss","last_from":2,"last_to":1,"last_age_ms":{age}}}"#
            ))
        }
    }

    #[test]
    fn each_ladder_transition_is_logged_once() {
        let Rig { mut cl, shared, .. } = rig(
            Box::new(LadderCore {
                now_ms: 0,
                moves: vec![1530, 3270],
            }),
            None,
        );
        let mut told = Vec::new();
        let mut t = 1_000_000;
        while t < 5_000_000 {
            cl.tick(t);
            told.extend(shared.events.lock().unwrap().drain(..));
            t += TICK_US;
        }
        let ages: Vec<_> = told
            .iter()
            .filter(|(ev, _)| *ev == "rung_reason")
            .map(|(_, v)| v["age_ms"].as_i64().unwrap())
            .collect();
        // Seen with the first 10 Hz stats line after each move, at 1.6 s
        // and 3.3 s; the odd tenth carries the wobble's 1 ms. Every later
        // line reads the same move 1 ms earlier or not, and stays quiet.
        assert_eq!(ages, [70, 31], "{told:?}");
    }

    #[test]
    fn free_space_reads_the_volume_or_nothing() {
        assert!(free_bytes(&std::env::temp_dir()).is_some_and(|b| b > 0));
        assert_eq!(free_bytes(std::path::Path::new("/no/such/dir")), None);
    }

    #[test]
    fn a_full_disk_stops_the_log_once_and_the_ui_line_says_so() {
        let root = std::env::temp_dir().join(format!("pal-rt-logerr-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        let run = flightlog::session::Run {
            app: "test",
            core: "rust",
            mode: "gs",
            config: None,
        };
        let mut log = Session::create(&root, std::time::SystemTime::now(), 0, &run).unwrap();
        let disk = Arc::new(Mutex::new(Disk {
            left: 4000,
            ..Disk::default()
        }));
        log.wrap_writers(|_, w| Box::new(FailAfter(disk.clone(), w)));
        let Rig {
            mut cl,
            shared,
            stats_rx,
            ..
        } = rig(Box::new(AuCore), Some(log));
        let mut told = Vec::new();
        let mut t = 1_000_000;
        // 5 s of ticks: the au.log lines alone pass 4000 bytes in well
        // under a second, so the disk fills early and stays full.
        while t < 6_000_000 {
            cl.tick(t);
            // What the next tick would drain into the log, taken first.
            told.extend(shared.events.lock().unwrap().drain(..));
            t += TICK_US;
        }
        drop(cl);
        let logged: Vec<_> = told.iter().filter(|(ev, _)| *ev == "log_error").collect();
        assert_eq!(logged.len(), 1, "{told:?}");
        assert!(
            logged[0].1["error"]
                .as_str()
                .is_some_and(|e| e.contains("No space left")),
            "{:?}",
            logged[0]
        );
        let d = disk.lock().unwrap();
        assert!(d.failed);
        assert_eq!(d.calls_after, 0, "writes after the first failure");
        let last = stats_rx.try_iter().last().expect("UI lines");
        let v: serde_json::Value = serde_json::from_str(&last).unwrap();
        assert!(
            v["log_err"]
                .as_str()
                .is_some_and(|e| e.contains("No space left")),
            "{last}"
        );
        let _ = std::fs::remove_dir_all(&root);
    }
}
