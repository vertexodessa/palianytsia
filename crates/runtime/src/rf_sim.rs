//! Bench-only RF loss simulator: what an attenuator and a fading path would
//! do to the drone's frames, applied where a card hands a frame to the
//! session (`CardSet`), before the channel core's tap and the core see it,
//! and to the GS's RC sends. Off unless a front end installs one
//! (`pal live --rf-sim`, `pal-desktop --rf-sim`, Android `debug.pal.rf_sim`).
//!
//! The model, per own frame (foreign frames pass untouched):
//! - `atten`: dB taken off the frame's RSSI and SNR, so the ladder and the
//!   link-health view read a weaker link, and a loss chance from the margin
//!   to the frame's MCS sensitivity: 50 % at the sensitivity, ~10 % 2.2 dB
//!   above it, ~90 % 2.2 dB below (logistic, 1 dB scale). A fixed value or
//!   a triangle `A-B/S` (A to B and back over S seconds).
//! - `fade`: `EVERY_S/LEN_MS` fades at random, exponential gaps and
//!   lengths with those means; a fade adds `fade_db` (default: every frame
//!   lost) and drops RC sends.
//! - `loss`: a flat percentage on top.
//! - `crc`: the percentage of lost frames delivered FCS-bad with 1 to 4
//!   bytes of the body flipped instead of vanishing (a chip near its
//!   sensitivity hands those up; the core salvages from them).
//! - `up`: a flat percentage of RC sends dropped.
//!
//! The core's stats line is not touched (the parity gates compare it); the
//! counters go to the UI line as `rf_sim` and to the flight log as an
//! `rf_sim` event each second, so a log taken under the simulator says so.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, RwLock};
use std::time::Instant;

use devourer_sys::RxFrame;

/// HT MCS 0..7 sensitivity at 20 MHz, one stream, dBm: typical RTL88xx
/// datasheet values. 40 MHz is 3 dB worse, MCS 8..15 (two streams) another 3.
const SENS_20_DBM: [f64; 8] = [-92.0, -89.0, -87.0, -84.0, -80.0, -76.0, -74.0, -72.0];

#[derive(Clone, Debug, PartialEq)]
pub struct Spec {
    /// (from, to, period_s); from == to for a fixed attenuation.
    pub atten: (f64, f64, f64),
    pub loss_pct: f64,
    /// (mean gap s, mean length ms); None = no fades.
    pub fade: Option<(f64, f64)>,
    /// None = a fade loses every frame.
    pub fade_db: Option<f64>,
    pub crc_pct: f64,
    pub up_pct: f64,
    pub seed: u64,
}

impl Default for Spec {
    fn default() -> Self {
        Spec {
            atten: (0.0, 0.0, 0.0),
            loss_pct: 0.0,
            fade: None,
            fade_db: None,
            crc_pct: 25.0,
            up_pct: 0.0,
            seed: 1,
        }
    }
}

impl Spec {
    /// `atten=40-70/60,loss=5,fade=8/300,fade_db=25,crc=25,up=2,seed=7`; any
    /// subset, comma separated.
    pub fn parse(s: &str) -> Result<Spec, String> {
        let mut o = Spec::default();
        let num = |k: &str, v: &str| -> Result<f64, String> {
            v.trim()
                .parse::<f64>()
                .ok()
                .filter(|x| x.is_finite() && *x >= 0.0)
                .ok_or_else(|| format!("rf_sim: {k}={v}: not a number >= 0"))
        };
        let pct = |k: &str, v: &str| -> Result<f64, String> {
            let x = num(k, v)?;
            if x > 100.0 {
                return Err(format!("rf_sim: {k}={v}: above 100"));
            }
            Ok(x)
        };
        for kv in s.split(',').map(str::trim).filter(|x| !x.is_empty()) {
            let (k, v) = kv
                .split_once('=')
                .ok_or_else(|| format!("rf_sim: {kv}: expected key=value"))?;
            match k.trim() {
                "atten" => {
                    o.atten = match v.split_once('/') {
                        None => {
                            let a = num(k, v)?;
                            (a, a, 0.0)
                        }
                        Some((range, per)) => {
                            let (a, b) = range
                                .split_once('-')
                                .ok_or_else(|| format!("rf_sim: atten={v}: expected A-B/S"))?;
                            let p = num(k, per)?;
                            if p <= 0.0 {
                                return Err(format!("rf_sim: atten={v}: period must be > 0"));
                            }
                            (num(k, a)?, num(k, b)?, p)
                        }
                    }
                }
                "loss" => o.loss_pct = pct(k, v)?,
                "fade" => {
                    let (e, l) = v
                        .split_once('/')
                        .ok_or_else(|| format!("rf_sim: fade={v}: expected EVERY_S/LEN_MS"))?;
                    let (e, l) = (num(k, e)?, num(k, l)?);
                    if e <= 0.0 || l <= 0.0 {
                        return Err(format!("rf_sim: fade={v}: both must be > 0"));
                    }
                    o.fade = Some((e, l));
                }
                "fade_db" => o.fade_db = Some(num(k, v)?),
                "crc" => o.crc_pct = pct(k, v)?,
                "up" => o.up_pct = pct(k, v)?,
                "seed" => {
                    o.seed = v
                        .trim()
                        .parse()
                        .map_err(|_| format!("rf_sim: seed={v}: not an integer"))?
                }
                _ => return Err(format!("rf_sim: unknown key {k}")),
            }
        }
        Ok(o)
    }

    /// The attenuation `t_s` seconds in: a triangle from `from` to `to`
    /// and back over the period.
    pub fn atten_at(&self, t_s: f64) -> f64 {
        let (a, b, p) = self.atten;
        if p <= 0.0 || a == b {
            return a;
        }
        let ph = (t_s / p).fract();
        let tri = if ph < 0.5 { ph * 2.0 } else { 2.0 - ph * 2.0 };
        a + (b - a) * tri
    }
}

/// The loss chance of a frame `margin_db` above its MCS sensitivity.
pub fn loss_at_margin(margin_db: f64) -> f64 {
    1.0 / (1.0 + margin_db.exp())
}

/// Sensitivity of HT MCS `mcs` (255 = unknown, read as MCS 0) at `bw40`.
pub fn sensitivity_dbm(mcs: u8, bw40: bool) -> f64 {
    let m = if mcs == 255 { 0 } else { mcs };
    let mut s = SENS_20_DBM[usize::from(m % 8)];
    if bw40 {
        s += 3.0;
    }
    if (8..16).contains(&m) {
        s += 3.0;
    }
    s
}

struct State {
    rng: u64,
    fading: bool,
    /// Seconds since start of the next fade edge.
    next_edge_s: f64,
}

impl State {
    fn unit(&mut self) -> f64 {
        (pal_wire::sw::splitmix64(&mut self.rng) >> 11) as f64 / (1u64 << 53) as f64
    }
    fn chance(&mut self, pct: f64) -> bool {
        pct > 0.0 && self.unit() * 100.0 < pct
    }
    /// An exponential draw with mean `mean`.
    fn exp(&mut self, mean: f64) -> f64 {
        -mean * (1.0 - self.unit()).ln()
    }
}

#[derive(Default)]
struct Counts {
    rx: AtomicU64,
    rx_lost: AtomicU64,
    rx_corrupt: AtomicU64,
    tx: AtomicU64,
    tx_lost: AtomicU64,
    fades: AtomicU64,
}

pub struct RfSim {
    spec: Spec,
    text: String,
    t0: Instant,
    st: Mutex<State>,
    n: Counts,
}

/// What happens to one received frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Fate {
    /// Delivered, `db` taken off its RSSI and SNR.
    Pass {
        db: f64,
    },
    Lost,
    /// Delivered FCS-bad with bytes flipped.
    Corrupt {
        db: f64,
    },
}

impl RfSim {
    pub fn new(spec: Spec, text: &str) -> Self {
        let mut st = State {
            rng: spec.seed,
            fading: false,
            next_edge_s: f64::INFINITY,
        };
        if let Some((every, _)) = spec.fade {
            st.next_edge_s = st.exp(every);
        }
        RfSim {
            spec,
            text: text.to_string(),
            t0: Instant::now(),
            st: Mutex::new(st),
            n: Counts::default(),
        }
    }

    pub fn spec(&self) -> &Spec {
        &self.spec
    }

    fn now_s(&self) -> f64 {
        self.t0.elapsed().as_secs_f64()
    }

    /// Advances the fade state to `t_s`; true while in a fade.
    fn fading(&self, st: &mut State, t_s: f64) -> bool {
        let Some((every, len_ms)) = self.spec.fade else {
            return false;
        };
        while t_s >= st.next_edge_s {
            st.fading = !st.fading;
            let mean = if st.fading {
                self.n.fades.fetch_add(1, Ordering::Relaxed);
                len_ms / 1000.0
            } else {
                every
            };
            st.next_edge_s += st.exp(mean).max(1e-6);
        }
        st.fading
    }

    /// The fate of a frame heard at `rssi_dbm` (best chain; None when the
    /// PHY status is not valid) at HT `mcs` and `bw40`, `t_s` seconds in.
    pub fn fate_at(&self, t_s: f64, rssi_dbm: Option<f64>, mcs: u8, bw40: bool) -> Fate {
        let mut st = self.st.lock().unwrap_or_else(|e| e.into_inner());
        let fade = self.fading(&mut st, t_s);
        let mut db = self.spec.atten_at(t_s);
        let mut lost = false;
        if fade {
            match self.spec.fade_db {
                Some(f) => db += f,
                None => lost = true,
            }
        }
        if !lost && db > 0.0 {
            if let Some(r) = rssi_dbm {
                let m = r - db - sensitivity_dbm(mcs, bw40);
                lost = st.unit() < loss_at_margin(m);
            }
        }
        if !lost {
            lost = st.chance(self.spec.loss_pct);
        }
        if !lost {
            return Fate::Pass { db };
        }
        if st.chance(self.spec.crc_pct) {
            Fate::Corrupt { db }
        } else {
            Fate::Lost
        }
    }

    /// Applies the model to a frame the card delivered: None = lost; else
    /// the frame as the session should see it (`buf` holds a corrupted
    /// copy). `body` is the body's (offset, HT MCS) for own frames, None
    /// for anything else, which passes untouched.
    pub fn rx<'a>(
        &self,
        f: &RxFrame<'a>,
        body: Option<(usize, u8)>,
        buf: &'a mut Vec<u8>,
    ) -> Option<RxFrame<'a>> {
        let Some((off, mcs)) = body else {
            return Some(*f);
        };
        self.n.rx.fetch_add(1, Ordering::Relaxed);
        let best = f.rssi.iter().copied().filter(|&v| v > 0).max();
        let rssi = best.filter(|_| f.physt).map(|v| f64::from(v) - 110.0);
        let fate = self.fate_at(self.now_s(), rssi, mcs, f.bw40);
        let (db, corrupt) = match fate {
            Fate::Lost => {
                self.n.rx_lost.fetch_add(1, Ordering::Relaxed);
                return None;
            }
            Fate::Pass { db } => (db, false),
            Fate::Corrupt { db } => (db, true),
        };
        let d = db.round().clamp(0.0, 127.0) as u8;
        let mut g = *f;
        for (r, s) in g.rssi.iter_mut().zip(g.snr.iter_mut()) {
            if *r > 0 {
                *r = r.saturating_sub(d).max(1);
                *s = s.saturating_sub(d as i8).max(0);
            }
        }
        if corrupt {
            // The FCS is the last 4 bytes; flip inside the body only so
            // the frame still parses as the drone's.
            let end = f.frame.len().saturating_sub(4);
            if end <= off {
                self.n.rx_lost.fetch_add(1, Ordering::Relaxed);
                return None;
            }
            self.n.rx_corrupt.fetch_add(1, Ordering::Relaxed);
            buf.clear();
            buf.extend_from_slice(f.frame);
            let mut st = self.st.lock().unwrap_or_else(|e| e.into_inner());
            let k = 1 + (st.unit() * 4.0) as usize;
            for _ in 0..k.min(4) {
                let i = off + (st.unit() * (end - off) as f64) as usize;
                let bit = 1u8 << ((st.unit() * 8.0) as u32).min(7);
                buf[i.min(end - 1)] ^= bit;
            }
            drop(st);
            g.frame = buf.as_slice();
            g.crc_err = true;
        }
        Some(g)
    }

    /// Whether an RC send goes out (false = lost on the uplink).
    pub fn tx_passes(&self) -> bool {
        self.n.tx.fetch_add(1, Ordering::Relaxed);
        let mut st = self.st.lock().unwrap_or_else(|e| e.into_inner());
        let lost = self.fading(&mut st, self.now_s()) || st.chance(self.spec.up_pct);
        if lost {
            self.n.tx_lost.fetch_add(1, Ordering::Relaxed);
        }
        !lost
    }

    /// The counters and the current attenuation, for the UI line and the
    /// flight log.
    pub fn snapshot(&self) -> serde_json::Value {
        let t = self.now_s();
        let fading = {
            let mut st = self.st.lock().unwrap_or_else(|e| e.into_inner());
            self.fading(&mut st, t)
        };
        let g = |a: &AtomicU64| a.load(Ordering::Relaxed);
        serde_json::json!({
            "spec": self.text,
            "atten_db": (self.spec.atten_at(t) * 10.0).round() / 10.0,
            "fading": fading,
            "rx": g(&self.n.rx),
            "rx_lost": g(&self.n.rx_lost),
            "rx_corrupt": g(&self.n.rx_corrupt),
            "tx": g(&self.n.tx),
            "tx_lost": g(&self.n.tx_lost),
            "fades": g(&self.n.fades),
        })
    }
}

static ACTIVE: RwLock<Option<Arc<RfSim>>> = RwLock::new(None);

/// Installs the process's simulator (None removes it). One per process:
/// it stands for the air between the drone and this GS, whichever session
/// or card is using it.
pub fn install(sim: Option<RfSim>) {
    if let Some(s) = &sim {
        eprintln!("pal: RF SIMULATOR ACTIVE ({}): bench only", s.text);
        log::warn!("RF simulator active: {}", s.text);
    }
    *ACTIVE.write().unwrap_or_else(|e| e.into_inner()) = sim.map(Arc::new);
}

/// Parses and installs `spec`; empty = none.
pub fn install_spec(spec: &str) -> Result<(), String> {
    if spec.trim().is_empty() {
        install(None);
        return Ok(());
    }
    install(Some(RfSim::new(Spec::parse(spec)?, spec.trim())));
    Ok(())
}

pub fn active() -> Option<Arc<RfSim>> {
    ACTIVE.read().unwrap_or_else(|e| e.into_inner()).clone()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_every_key_and_refuses_junk() {
        let s =
            Spec::parse("atten=40-70/60, loss=5,fade=8/300,fade_db=25,crc=10,up=2,seed=7").unwrap();
        assert_eq!(s.atten, (40.0, 70.0, 60.0));
        assert_eq!(s.loss_pct, 5.0);
        assert_eq!(s.fade, Some((8.0, 300.0)));
        assert_eq!(s.fade_db, Some(25.0));
        assert_eq!((s.crc_pct, s.up_pct, s.seed), (10.0, 2.0, 7));
        assert_eq!(Spec::parse("atten=55").unwrap().atten, (55.0, 55.0, 0.0));
        for bad in [
            "atten=-3",
            "loss=101",
            "fade=8",
            "fade=0/10",
            "foo=1",
            "loss",
            "atten=1-2/0",
        ] {
            assert!(Spec::parse(bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn triangle_runs_there_and_back() {
        let s = Spec::parse("atten=40-60/20").unwrap();
        assert_eq!(s.atten_at(0.0), 40.0);
        assert_eq!(s.atten_at(5.0), 50.0);
        assert_eq!(s.atten_at(10.0), 60.0);
        assert_eq!(s.atten_at(15.0), 50.0);
        assert_eq!(s.atten_at(20.0), 40.0);
    }

    #[test]
    fn margin_curve_and_sensitivity() {
        assert!((loss_at_margin(0.0) - 0.5).abs() < 1e-9);
        assert!(loss_at_margin(2.2) < 0.11 && loss_at_margin(2.2) > 0.09);
        assert!(loss_at_margin(-2.2) > 0.89);
        assert_eq!(sensitivity_dbm(0, false), -92.0);
        assert_eq!(sensitivity_dbm(255, false), -92.0);
        assert_eq!(sensitivity_dbm(4, true), -77.0);
        assert_eq!(sensitivity_dbm(12, false), -77.0);
    }

    fn rate(sim: &RfSim, t: f64, rssi: f64, mcs: u8, n: usize) -> f64 {
        let lost = (0..n)
            .filter(|i| {
                !matches!(
                    sim.fate_at(t + *i as f64 * 1e-6, Some(rssi), mcs, false),
                    Fate::Pass { .. }
                )
            })
            .count();
        lost as f64 / n as f64
    }

    /// The point of the model: at one attenuation a low MCS survives
    /// where a high one does not, so a demote helps as on real air.
    #[test]
    fn loss_depends_on_mcs_at_one_attenuation() {
        let sim = RfSim::new(Spec::parse("atten=50").unwrap(), "atten=50");
        // -30 dBm on the bench, 50 dB off: -80 dBm; MCS 4 sits at its
        // sensitivity, MCS 1 has 9 dB of margin.
        let m4 = rate(&sim, 0.0, -30.0, 4, 20_000);
        let m1 = rate(&sim, 0.0, -30.0, 1, 20_000);
        assert!((m4 - 0.5).abs() < 0.02, "mcs4 {m4}");
        assert!(m1 < 0.001, "mcs1 {m1}");
    }

    #[test]
    fn flat_loss_and_fades_cost_what_they_say() {
        let sim = RfSim::new(Spec::parse("loss=10,seed=3").unwrap(), "");
        let r = rate(&sim, 0.0, -30.0, 7, 50_000);
        assert!((r - 0.10).abs() < 0.01, "{r}");
        // Fades every 1 s for 100 ms on average: about 9 % of the time.
        let sim = RfSim::new(Spec::parse("fade=1/100,crc=0,seed=5").unwrap(), "");
        let mut lost = 0;
        let n = 600_000;
        for i in 0..n {
            let t = i as f64 * 1e-3;
            if sim.fate_at(t, Some(-30.0), 1, false) == Fate::Lost {
                lost += 1;
            }
        }
        let frac = lost as f64 / n as f64;
        assert!((frac - 0.0909).abs() < 0.015, "{frac}");
        assert!(sim.n.fades.load(Ordering::Relaxed) > 400);
    }

    #[test]
    fn rx_lowers_rssi_and_corrupts_only_the_body() {
        let sim = RfSim::new(Spec::parse("atten=10,loss=100,crc=100").unwrap(), "");
        let frame: Vec<u8> = (0..60u8).collect();
        let f = RxFrame {
            frame: &frame,
            crc_err: false,
            physt: true,
            data_rate: 0,
            rssi: [80, 0],
            snr: [30, 0],
            evm: [0, 0],
            tsfl: 0,
            bw40: false,
            stbc: false,
            sgi: false,
        };
        let mut buf = Vec::new();
        let g = sim
            .rx(&f, Some((26, 1)), &mut buf)
            .expect("corrupt, not lost");
        assert!(g.crc_err);
        assert_eq!(g.rssi, [70, 0]);
        assert_eq!(g.snr, [20, 0]);
        assert_eq!(&g.frame[..26], &frame[..26]);
        assert_eq!(&g.frame[56..], &frame[56..]);
        assert_ne!(&g.frame[26..56], &frame[26..56]);
        // Foreign frames are not the drone's signal: untouched.
        let mut buf = Vec::new();
        let h = sim.rx(&f, None, &mut buf).unwrap();
        assert_eq!(h.frame, &frame[..]);
        assert!(!h.crc_err);
    }
}
