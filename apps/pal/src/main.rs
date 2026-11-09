//! pal: the Linux CLI (spec 5.3). Live RX/TX through devourer, replay of a
//! capture, and the flight log. `--core ref` (default) runs mabur's C++
//! WebGs through pal-ref, `--core rust` the Rust core; captures are what the
//! two are diffed on.
//!
//!   pal live   -c gs.toml --ch 144 --width 40 [--mode gs|spotter] [--secs 30]
//!              [--card N] [--capture bodies.bin] [--dump out.h265] [--aus out.aus]
//!              [--log-dir DIR [--capture-mb N]] [--core ref|rust] [--param-set T_S,ROW,TEXT ...]
//!   pal replay <bodies.bin> -c gs.toml [--mode gs|spotter] [--dump out.h265]
//!              [--aus out.aus] [--sends out.sends] [--log-dir DIR]
//!              [--drop-pct N --seed S] [--core ref|rust] [--decode [--hwdec vdpau|vaapi|cuda]]
//!              [--param-set T_S,ROW,TEXT ...]   F18 sets; prints `param` results and `params`
//!   pal log    <fl-NNNN_...>   summarise one flight-log directory
//!
//! Gs mode transmits: the core's RC frames (DISC, RCF, NACK, refresh) go out
//! on the same card, and the drone follows this host.
//!
//! Both use a fixed frame-gap timeout and a pinned rendezvous nonce so a
//! replay of a capture reproduces the live run's AUs exactly.
use flightlog::capture::{self, Record};
use flightlog::session::{self, Session};
use pal_core::refresh::IdrDriver;
use pal_core::rust_core::RustCore;
use pal_core::CoreOpts;
use pal_core::{Au, CoreIo, GsCore, Mode, RxBody, RZ_NONCE};
use pal_ref::RefCore;
use pal_wire::dot11;
use std::io::{BufReader, BufWriter, Write};
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc;
use std::sync::Arc;
use std::time::{Duration, Instant};

#[derive(Default)]
struct Opts {
    config: Option<PathBuf>,
    ch: u8,
    width: u8,
    secs: u64,
    card: i32,
    capture: Option<PathBuf>,
    dump: Option<PathBuf>,
    aus: Option<PathBuf>,
    sends: Option<PathBuf>,
    mav: Option<PathBuf>,
    log_dir: Option<PathBuf>,
    decode: bool,
    /// libavcodec hwaccel for `--decode` ("vdpau", "vaapi", "cuda"); "" = software.
    hwdec: String,
    /// Replay: parameter sets `T_S,ROW,TEXT` (seconds after the first tick).
    param_sets: Vec<(f64, usize, String)>,
    capture_mb: u64,
    mode: Option<Mode>,
    rust: bool,
    drop_pct: u32,
    seed: u64,
    /// Live: IDR requests on (`--no-idr` off). Replay: off (`--idr` on).
    idr: Option<bool>,
    pos: Vec<PathBuf>,
}

fn parse(args: impl Iterator<Item = String>) -> Opts {
    let mut o = Opts {
        ch: 149,
        width: 20,
        secs: 30,
        ..Default::default()
    };
    let mut it = args;
    while let Some(a) = it.next() {
        let mut val = || {
            it.next()
                .unwrap_or_else(|| die(&format!("{a} needs a value")))
        };
        match a.as_str() {
            "-c" => o.config = Some(val().into()),
            "--ch" => o.ch = val().parse().unwrap_or_else(|_| die("--ch")),
            "--width" => o.width = val().parse().unwrap_or_else(|_| die("--width")),
            "--secs" => o.secs = val().parse().unwrap_or_else(|_| die("--secs")),
            "--card" => o.card = val().parse().unwrap_or_else(|_| die("--card")),
            "--capture" => o.capture = Some(val().into()),
            "--dump" => o.dump = Some(val().into()),
            "--aus" => o.aus = Some(val().into()),
            "--sends" => o.sends = Some(val().into()),
            "--mav" => o.mav = Some(val().into()),
            "--log-dir" => o.log_dir = Some(val().into()),
            "--decode" => o.decode = true,
            "--hwdec" => o.hwdec = val(),
            "--param-set" => {
                let v = val();
                let mut it = v.splitn(3, ',');
                let (Some(t), Some(r), Some(x)) = (it.next(), it.next(), it.next()) else {
                    die("--param-set T_S,ROW,TEXT")
                };
                o.param_sets.push((
                    t.parse().unwrap_or_else(|_| die("--param-set T_S")),
                    r.parse().unwrap_or_else(|_| die("--param-set ROW")),
                    x.to_string(),
                ));
            }
            "--idr" => o.idr = Some(true),
            "--no-idr" => o.idr = Some(false),
            "--capture-mb" => o.capture_mb = val().parse().unwrap_or_else(|_| die("--capture-mb")),
            "--mode" => {
                o.mode = Some(match val().as_str() {
                    "gs" => Mode::Gs,
                    "spotter" => Mode::Spotter,
                    m => die(&format!("--mode gs|spotter, got {m}")),
                })
            }
            "--drop-pct" => o.drop_pct = val().parse().unwrap_or_else(|_| die("--drop-pct")),
            "--seed" => o.seed = val().parse().unwrap_or_else(|_| die("--seed")),
            "--core" => {
                o.rust = match val().as_str() {
                    "rust" => true,
                    "ref" => false,
                    c => die(&format!("--core rust|ref, got {c}")),
                }
            }
            s if s.starts_with('-') => die(&format!("unknown flag {s}")),
            _ => o.pos.push(a.into()),
        }
    }
    o
}

fn die(msg: &str) -> ! {
    eprintln!("pal: {msg}");
    std::process::exit(2)
}

/// AU outputs: Annex-B concatenation for players, LP file for `cmp`
/// (u32 len | sid | flags|0x04 complete | u32 pts_us | data, as webgs replay).
struct Sink {
    dump: Option<BufWriter<std::fs::File>>,
    aus_file: Option<BufWriter<std::fs::File>>,
    /// Every RC body handed to `send`, LP (u32 len | bytes), for `cmp`.
    sends_file: Option<BufWriter<std::fs::File>>,
    /// Inbound MAVLink packets from the drone, LP (u32 len | bytes).
    mav_file: Option<BufWriter<std::fs::File>>,
    mav: u64,
    log: Option<Session>,
    /// `--decode`: every AU through libavcodec (frames/errors in the summary).
    dec: Option<decode::AvDecoder>,
    /// Core clock of the call in progress, for the AU log.
    now_us: u64,
    aus: u64,
    truncated: u64,
    bytes: u64,
    card: Option<Arc<devourer_sys::Card>>,
    sends: u64,
    send_fail: u64,
    idr: Option<IdrDriver>,
}

impl Sink {
    fn new(o: &Opts) -> Self {
        let mk = |p: &Option<PathBuf>| {
            p.as_ref().map(|p| {
                BufWriter::new(
                    std::fs::File::create(p)
                        .unwrap_or_else(|e| die(&format!("{}: {e}", p.display()))),
                )
            })
        };
        Sink {
            dump: mk(&o.dump),
            aus_file: mk(&o.aus),
            sends_file: mk(&o.sends),
            mav_file: mk(&o.mav),
            mav: 0,
            log: o.log_dir.as_ref().map(|d| {
                let s = Session::create(d, std::time::SystemTime::now(), o.capture_mb << 20)
                    .unwrap_or_else(|e| die(&format!("{}: {e}", d.display())));
                eprintln!("pal: flight log {}", s.dir().display());
                s
            }),
            now_us: 0,
            dec: o.decode.then(|| {
                decode::AvDecoder::open_or_sw(0, &o.hwdec)
                    .unwrap_or_else(|e| die(&format!("decode: {e}")))
            }),
            aus: 0,
            truncated: 0,
            bytes: 0,
            card: None,
            sends: 0,
            send_fail: 0,
            idr: None,
        }
    }
    fn finish(&mut self) {
        for w in [
            self.dump.as_mut(),
            self.aus_file.as_mut(),
            self.sends_file.as_mut(),
            self.mav_file.as_mut(),
        ]
        .into_iter()
        .flatten()
        {
            w.flush().unwrap();
        }
        if let Some(l) = self.log.as_mut() {
            l.flush().unwrap();
        }
    }

    /// After each core tick: IDR retries and the epoch into the core.
    fn idr_step(&mut self, now_us: u64, core: &mut dyn GsCore) {
        let Some(d) = self.idr.as_mut() else { return };
        let evs = d.step(now_us, core);
        if let Some(l) = self.log.as_mut() {
            l.refresh_events(&evs).unwrap();
        }
    }

    /// Once a second of core clock: the stats line into the flight log.
    fn log_stats(&mut self, now_us: u64, last: &mut u64, core: &dyn GsCore) {
        if self.log.is_some() && now_us >= *last + 1_000_000 {
            *last = now_us;
            let line = core.stats_json();
            self.log
                .as_mut()
                .map(|l| l.stats(now_us, &line))
                .transpose()
                .unwrap();
        }
    }
}

impl CoreIo for Sink {
    fn on_mav(&mut self, pkt: &[u8]) {
        self.mav += 1;
        if let Some(w) = self.mav_file.as_mut() {
            w.write_all(&(pkt.len() as u32).to_le_bytes()).unwrap();
            w.write_all(pkt).unwrap();
        }
    }
    fn on_au(&mut self, a: Au) {
        if let Some(l) = self.log.as_mut() {
            l.au(self.now_us, &a).unwrap();
        }
        if let Some(i) = self.idr.as_mut() {
            i.on_au(&a, self.now_us);
        }
        if let Some(d) = self.dec.as_mut() {
            d.decode(&a.data);
            if let Some(i) = self.idr.as_mut() {
                // Synchronous decode: the errors are this AU's.
                let n = i.aus();
                i.on_decoder_errors(d.counters().1, n, self.now_us);
            }
        }
        self.aus += 1;
        self.bytes += a.data.len() as u64;
        if !a.complete {
            self.truncated += 1;
        }
        if let Some(w) = self.dump.as_mut() {
            w.write_all(&a.data).unwrap();
        }
        if let Some(w) = self.aus_file.as_mut() {
            let flags = a.flags | if a.complete { 0x04 } else { 0 };
            w.write_all(&(a.data.len() as u32).to_le_bytes()).unwrap();
            w.write_all(&[a.sid, flags]).unwrap();
            w.write_all(&a.pts_us.to_le_bytes()).unwrap();
            w.write_all(&a.data).unwrap();
        }
    }
    fn send(&mut self, rc_body: &[u8]) {
        self.sends += 1;
        if let Some(w) = self.sends_file.as_mut() {
            w.write_all(&(rc_body.len() as u32).to_le_bytes()).unwrap();
            w.write_all(rc_body).unwrap();
        }
        if let Some(c) = &self.card {
            if !c.send_control(rc_body) {
                self.send_fail += 1;
            }
        }
    }
}

fn core(o: &Opts, channel: u8) -> Box<dyn GsCore> {
    let cfg = o
        .config
        .clone()
        .unwrap_or_else(|| die("-c <gs.toml> is required"));
    let opts = CoreOpts {
        mode: o.mode.unwrap_or(Mode::Spotter),
        adaptive_gap: false,
        rz_nonce: RZ_NONCE,
        channel,
    };
    if o.rust {
        let c = pal_core::config::load_config(&cfg, None).unwrap_or_else(|e| die(&e.to_string()));
        return Box::new(RustCore::new(&c, opts).unwrap_or_else(|e| die(&e)));
    }
    Box::new(RefCore::new(&cfg, opts).unwrap_or_else(|e| die(&format!("core: {e}"))))
}

/// The `--param-set` entries still to send, in time order.
struct ParamScript(std::iter::Peekable<std::vec::IntoIter<(f64, usize, String)>>);

impl ParamScript {
    fn new(sets: &[(f64, usize, String)]) -> Self {
        let mut sets = sets.to_vec();
        sets.sort_by(|a, b| a.0.total_cmp(&b.0));
        Self(sets.into_iter().peekable())
    }

    /// Sends each entry whose time `due` says has come. `tools/m1-replay.sh`
    /// compares the printed lines.
    fn fire(&mut self, core: &mut dyn GsCore, due: impl Fn(f64) -> bool) {
        while let Some((ts, row, text)) = self.0.next_if(|(ts, _, _)| due(*ts)) {
            let r = core.param_set(row, &text);
            println!("param t={ts} row={row} text={text} -> {r:?}");
        }
    }
}

/// `pal live`'s side of the runtime's pump: capture and flight log around
/// the core, and the per-second figures.
struct Live {
    core: Box<dyn GsCore>,
    sink: Sink,
    cap: Option<capture::Writer<BufWriter<std::fs::File>>>,
    t0: Instant,
    sets: ParamScript,
    print_params: bool,
    last_params: String,
    last_stats: u64,
    bodies: u64,
    rssi_sum: f64,
    rssi_n: u64,
    mcs: u8,
}

impl runtime::BodyLoop for Live {
    fn rx(&mut self, b: &RxBody) {
        if b.phy_valid && b.crc_ok {
            self.rssi_sum += f64::from(b.rssi[0]) - 110.0;
            self.rssi_n += 1;
        }
        if b.mcs != 255 {
            self.mcs = b.mcs;
        }
        if let Some(c) = self.cap.as_mut() {
            c.body(b).unwrap();
        }
        self.bodies += 1;
        self.sink.now_us = b.mono_us;
        if let Some(l) = self.sink.log.as_mut() {
            l.body(b).unwrap();
        }
        self.core.on_rx(b, &mut self.sink);
    }

    fn tick(&mut self, now: u64) {
        if let Some(c) = self.cap.as_mut() {
            c.tick(now).unwrap();
        }
        self.sink.now_us = now;
        if let Some(l) = self.sink.log.as_mut() {
            l.tick(now).unwrap();
        }
        let el = self.t0.elapsed().as_secs_f64();
        self.sets.fire(&mut *self.core, |ts| el >= ts);
        self.core.tick(now, &mut self.sink);
        if self.print_params {
            // Live: print each change of a row's state as it happens.
            let p = self.core.params_json(now);
            if p != self.last_params {
                print_param_changes(&self.last_params, &p, el);
                self.last_params = p;
            }
        }
        self.sink.idr_step(now, &mut *self.core);
        self.sink.log_stats(now, &mut self.last_stats, &*self.core);
    }
}

fn live(o: Opts) {
    let core = core(&o, o.ch);
    let mut sink = Sink::new(&o);
    let cap = o.capture.as_ref().map(|p| {
        capture::Writer::new(BufWriter::with_capacity(
            1 << 20,
            std::fs::File::create(p).unwrap(),
        ))
        .unwrap()
    });
    let set = runtime::CardSet::open(
        vec![devourer_sys::Select::Scan { index: o.card }],
        o.ch,
        o.width,
    )
    .unwrap_or_else(|e| die(&e));
    eprintln!("pal: card {} on ch {} / {} MHz", set.chips(), o.ch, o.width);

    let t0 = Instant::now();
    let clock = runtime::mono_default;
    let (tx, rx) = mpsc::sync_channel::<RxBody>(4096);
    let (foreign, rx_frames) = (Arc::new(AtomicU64::new(0)), Arc::new(AtomicU64::new(0)));
    let shared = Arc::new(runtime::Shared::default());
    let stamp: runtime::Stamp = {
        let (foreign, rx_frames, ch) = (foreign.clone(), rx_frames.clone(), o.ch);
        Arc::new(move |_, _, v| {
            rx_frames.fetch_add(1, Ordering::Relaxed);
            if matches!(v, dot11::Verdict::Foreign) {
                foreign.fetch_add(1, Ordering::Relaxed);
            }
            Some((ch, clock()))
        })
    };
    let route = set
        .begin(o.ch, o.width, stamp, tx, &shared)
        .unwrap_or_else(|e| die(&e));
    if o.mode == Some(Mode::Gs) {
        sink.card = set.cards().first().cloned();
        if o.idr != Some(false) {
            sink.idr = Some(IdrDriver::default());
        }
    }

    let mut live = Live {
        core,
        sink,
        cap,
        t0,
        sets: ParamScript::new(&o.param_sets),
        print_params: !o.param_sets.is_empty(),
        last_params: String::new(),
        last_stats: 0,
        bodies: 0,
        rssi_sum: 0.0,
        rssi_n: 0,
        mcs: 255,
    };
    let end = Duration::from_secs(o.secs);
    let mut next_report = t0 + Duration::from_secs(1);
    let mut last_aus = 0u64;
    runtime::pump(
        &mut live,
        &rx,
        clock,
        &shared,
        |_, _| {},
        |l, _| {
            if Instant::now() >= next_report {
                next_report += Duration::from_secs(1);
                eprintln!(
                    "pal: t={:>3}s bodies={} aus/s={} truncated={} rssi={:.1} mcs={} rx_frames={} foreign={} dropped={} sends={} send_fail={}",
                    l.t0.elapsed().as_secs(),
                    l.bodies,
                    l.sink.aus - last_aus,
                    l.sink.truncated,
                    if l.rssi_n > 0 { l.rssi_sum / l.rssi_n as f64 } else { f64::NAN },
                    l.mcs,
                    rx_frames.load(Ordering::Relaxed),
                    foreign.load(Ordering::Relaxed),
                    shared.lat.rx_full.load(Ordering::Relaxed),
                    l.sink.sends,
                    l.sink.send_fail
                );
                last_aus = l.sink.aus;
                l.rssi_sum = 0.0;
                l.rssi_n = 0;
            }
            Ok(l.t0.elapsed() < end)
        },
    )
    .unwrap_or_else(|e| die(&e));
    live.sink.card = None;
    drop(route); // no more bodies join the queue; what is in it still counts
    while let Ok(b) = rx.try_recv() {
        runtime::BodyLoop::rx(&mut live, &b);
    }
    if let Some(c) = live.cap.as_mut() {
        c.flush().unwrap();
    }
    let bodies = live.bodies;
    live.sink.finish();
    println!(
        "live bodies={bodies} aus={} truncated={} au_bytes={} dropped={} sends={} send_fail={} stats={}",
        live.sink.aus,
        live.sink.truncated,
        live.sink.bytes,
        shared.lat.rx_full.load(Ordering::Relaxed),
        live.sink.sends,
        live.sink.send_fail,
        live.core.stats_json()
    );
}

fn replay(o: Opts) {
    let [path] = <[PathBuf; 1]>::try_from(o.pos.clone())
        .unwrap_or_else(|_| die("replay needs one <bodies.bin>"));
    let mut core = core(&o, 0);
    let mut sink = Sink::new(&o);
    if o.idr == Some(true) {
        if o.mode == Some(Mode::Gs) {
            sink.idr = Some(IdrDriver::default());
        } else {
            eprintln!("pal: --idr needs --mode gs (Spotter sends no RCF); ignored");
        }
    }
    let f = std::fs::File::open(&path).unwrap_or_else(|e| die(&format!("{}: {e}", path.display())));
    let r = capture::Reader::new(BufReader::with_capacity(1 << 20, f))
        .unwrap_or_else(|e| die(&e.to_string()));
    let (mut bodies, mut ticks, mut dropped) = (0u64, 0u64, 0u64);
    let mut last_stats = 0u64;
    let mut t_first: Option<u64> = None;
    let mut sets = ParamScript::new(&o.param_sets);
    let mut last_t = 0u64;
    // Synthetic loss: splitmix64 on a fixed seed, so both cores see the
    // same surviving bodies.
    let mut rng = o.seed.wrapping_add(pal_wire::sw::SPLITMIX_GAMMA);
    for rec in r {
        let rec = match rec {
            Ok(rec) => rec,
            // A killed app can leave half a record; the flight before it counts.
            Err(e) if capture::is_torn_tail(&e) => {
                eprintln!("pal: capture ends in a torn record, stopping there: {e}");
                break;
            }
            Err(e) => die(&format!("capture: {e}")),
        };
        match rec {
            Record::Body(b) => {
                if o.drop_pct > 0
                    && (pal_wire::sw::splitmix64(&mut rng) % 100) < u64::from(o.drop_pct)
                {
                    dropped += 1;
                    continue;
                }
                bodies += 1;
                sink.now_us = b.mono_us;
                core.on_rx(&b, &mut sink);
            }
            Record::Tick(t) => {
                ticks += 1;
                sink.now_us = t;
                last_t = t;
                let t0 = *t_first.get_or_insert(t);
                sets.fire(&mut *core, |ts| t >= t0 + (ts * 1e6) as u64);
                core.tick(t, &mut sink);
                sink.idr_step(t, &mut *core);
                sink.log_stats(t, &mut last_stats, &*core);
            }
        }
    }
    sink.finish();
    println!(
        "replay bodies={bodies} dropped={dropped} ticks={ticks} aus={} truncated={} au_bytes={} sends={} mav={}",
        sink.aus, sink.truncated, sink.bytes, sink.sends, sink.mav
    );
    if let Some(d) = sink.dec.as_ref() {
        let (f, e) = d.counters();
        println!("decode frames={f} errors={e} hw_frames={}", d.hw_frames());
    }
    println!("stats {}", core.stats_json());
    if o.mode == Some(Mode::Gs) {
        println!("params {}", core.params_json(last_t));
    }
}

/// Rows whose value, state or status changed between two params tables.
fn print_param_changes(old: &str, new: &str, t_s: f64) {
    let rows = |s: &str| -> Vec<serde_json::Value> {
        serde_json::from_str::<serde_json::Value>(s)
            .ok()
            .and_then(|v| v.get("rows").and_then(|r| r.as_array().cloned()))
            .unwrap_or_default()
    };
    let (a, b) = (rows(old), rows(new));
    for (i, r) in b.iter().enumerate() {
        let pick = |r: &serde_json::Value| {
            (
                r.get("value").cloned(),
                r.get("state").cloned(),
                r.get("status").cloned(),
            )
        };
        if a.get(i).map(pick) != Some(pick(r)) {
            println!(
                "param t={t_s:.1} {} value={} state={} status={}",
                r["id"], r["value"], r["state"], r["status"]
            );
        }
    }
}

fn log_summary(o: Opts) {
    let [dir] = <[PathBuf; 1]>::try_from(o.pos)
        .unwrap_or_else(|_| die("log needs one fl-NNNN_... directory"));
    let s = session::summarize(&dir).unwrap_or_else(|e| die(&format!("{}: {e}", dir.display())));
    println!("{}", dir.display());
    println!(
        "  span {:.1} s, {} stats lines, capture {}",
        s.span_s,
        s.stats_lines,
        if s.has_capture { "yes" } else { "no" }
    );
    let pct = |n: u64| {
        if s.aus > 0 {
            100.0 * n as f64 / s.aus as f64
        } else {
            0.0
        }
    };
    println!(
        "  AUs {} ({:.1} MB, {} IDR): truncated {} ({:.1} %, base {}), salvaged {}",
        s.aus,
        s.au_bytes as f64 / 1e6,
        s.idr,
        s.truncated,
        pct(s.truncated),
        s.truncated_base,
        s.salvaged
    );
    for (t, to) in &s.session_edges {
        println!("  session -> {to} at {:.3} s", *t as f64 / 1e6);
    }
    let ev: Vec<String> = s.events.iter().map(|(k, n)| format!("{k} {n}")).collect();
    println!(
        "  events: {}",
        if ev.is_empty() {
            "none".into()
        } else {
            ev.join(", ")
        }
    );
    if let Some(v) = &s.last_stats {
        println!("  last stats: {v}");
    }
}

fn main() {
    let mut args = std::env::args().skip(1);
    match args.next().as_deref() {
        Some("live") => live(parse(args)),
        Some("replay") => replay(parse(args)),
        Some("log") => log_summary(parse(args)),
        _ => die("usage: pal live|replay|log ... (see src/main.rs)"),
    }
}
