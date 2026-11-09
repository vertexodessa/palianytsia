//! OSD widgets (spec section 6, feature `ui`) drawn from the core's `stats_json` line, so
//! every app and both cores share one OSD. Reads only the snapshot it is
//! handed: no locks toward the RX or core path.
#[cfg(feature = "ui")]
pub mod cal;
pub mod channels;
#[cfg(feature = "ui")]
pub mod console;
#[cfg(feature = "ui")]
pub mod hud;
#[cfg(feature = "ui")]
pub mod panel;
pub mod params;
#[cfg(feature = "ui")]
pub mod pointer;
pub mod prefs;
#[cfg(feature = "ui")]
pub mod telem;

use std::collections::VecDeque;

use serde_json::Value;

#[cfg(feature = "ui")]
mod widgets;
#[cfg(feature = "ui")]
pub use widgets::*;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CardRx {
    pub rssi: Option<i64>,
    pub video: u64,
    /// Every frame the card delivered, CRC failures included.
    pub frames: u64,
    pub crc_fail: u64,
}

/// The cards chip's window in seconds: long enough that a 5 ms scout dwell
/// or one late stats line does not move it, short enough to show a card
/// that just stopped.
pub const CARD_WINDOW_S: f64 = 3.0;

/// One card over the last `CARD_WINDOW_S`.
#[derive(Clone, Debug, PartialEq)]
pub struct CardRate {
    pub rssi: Option<i64>,
    pub fps: f64,
    /// None while the card delivered nothing in the window.
    pub crc_pct: Option<f64>,
    pub deaf: bool,
}

/// The best card must deliver this many frames/s before another one can be
/// called deaf: below it (search, a dark drone) every card is quiet.
const DEAF_MIN_BEST_FPS: f64 = 50.0;
/// A card on the link channel hears the same air as its neighbour; under a
/// tenth of the best card's frames it is not receiving (2026-11-02 Quest:
/// card 0 at 0.1 frames/s against card 1 at 230 with no USB error).
const DEAF_SHARE: f64 = 0.1;

/// Per-card rates from two snapshots `dt` seconds apart; empty when the
/// card count changed between them.
pub fn card_rates(old: &[CardRx], new: &[CardRx], dt: f64) -> Vec<CardRate> {
    if old.len() != new.len() || dt <= 0.0 {
        return Vec::new();
    }
    let raw: Vec<(f64, u64, u64)> = old
        .iter()
        .zip(new)
        .map(|(a, b)| {
            let f = b.frames.saturating_sub(a.frames);
            (f as f64 / dt, f, b.crc_fail.saturating_sub(a.crc_fail))
        })
        .collect();
    let best = raw.iter().map(|r| r.0).fold(0.0, f64::max);
    raw.iter()
        .zip(new)
        .map(|(&(fps, f, crc), c)| CardRate {
            rssi: c.rssi,
            fps,
            crc_pct: (f > 0).then(|| 100.0 * crc.min(f) as f64 / f as f64),
            deaf: best >= DEAF_MIN_BEST_FPS && fps < DEAF_SHARE * best,
        })
        .collect()
}

/// A card going deaf or hearing again, for the flight log.
#[derive(Clone, Debug, PartialEq)]
pub struct DeafChange {
    pub card: usize,
    pub deaf: bool,
    pub fps: f64,
    pub best_fps: f64,
}

/// Watches the per-card counters for the flight log the way the cards
/// chip does (same window, same rule), so a deaf card leaves an event
/// behind and not only a red chip nobody was looking at.
#[derive(Default)]
pub struct DeafWatch {
    rows: VecDeque<(f64, Vec<CardRx>)>,
    deaf: Vec<bool>,
}

impl DeafWatch {
    /// Feeds one snapshot; returns the cards whose state flipped. A
    /// changed card count starts over with every card hearing.
    pub fn push(&mut self, t_s: f64, cards: &[CardRx]) -> Vec<DeafChange> {
        if self.deaf.len() != cards.len() {
            self.rows.clear();
            self.deaf = vec![false; cards.len()];
        }
        self.rows.push_back((t_s, cards.to_vec()));
        while self
            .rows
            .front()
            .is_some_and(|(t, _)| t_s - t > CARD_WINDOW_S)
        {
            self.rows.pop_front();
        }
        let (t0, old) = &self.rows[0];
        if cards.len() < 2 || t_s <= *t0 {
            return Vec::new();
        }
        let rates = card_rates(old, cards, t_s - t0);
        let best = rates.iter().map(|r| r.fps).fold(0.0, f64::max);
        let mut out = Vec::new();
        for (i, r) in rates.iter().enumerate() {
            if r.deaf != self.deaf[i] {
                self.deaf[i] = r.deaf;
                out.push(DeafChange {
                    card: i,
                    deaf: r.deaf,
                    fps: r.fps,
                    best_fps: best,
                });
            }
        }
        out
    }
}

/// "c0 DEAF   0/s  c1 TX -45 dBm 230/s crc 6 %": each card's frames/s
/// and CRC failures over the last seconds, its RSSI, and which one
/// transmits. A deaf card's RSSI is left out: it is the last one heard.
pub fn cards_line(rates: &[CardRate], tx: Option<u64>) -> String {
    rates
        .iter()
        .enumerate()
        .map(|(i, c)| {
            let t = if tx == Some(i as u64) { " TX" } else { "" };
            let fps = c.fps.round() as i64;
            if c.deaf {
                return format!("c{i}{t} DEAF {fps:>3}/s");
            }
            let r = c.rssi.map_or("  -".into(), |r| format!("{r:>3}"));
            let crc = c
                .crc_pct
                .map_or(String::new(), |p| format!(" crc {:>2.0} %", p));
            format!("c{i}{t} {r} dBm {fps:>3}/s{crc}")
        })
        .collect::<Vec<_>>()
        .join("  ")
}

fn u(v: &Value, k: &str) -> u64 {
    v.get(k).and_then(Value::as_u64).unwrap_or(0)
}

/// The per-card receive counters of a parsed stats line.
pub fn cards_of(v: &Value) -> Vec<CardRx> {
    v.get("cards")
        .and_then(Value::as_array)
        .map(|a| {
            a.iter()
                .map(|c| CardRx {
                    rssi: c.get("rssi").and_then(Value::as_i64),
                    video: u(c, "video"),
                    frames: u(c, "frames"),
                    crc_fail: u(c, "crc_fail"),
                })
                .collect()
        })
        .unwrap_or_default()
}
