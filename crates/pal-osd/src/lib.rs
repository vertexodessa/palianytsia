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
mod json;
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

use crate::json::{list, opt_i64, u64_or};

#[cfg(feature = "ui")]
mod widgets;
#[cfg(feature = "ui")]
pub use widgets::*;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CardRx {
    pub rssi: Option<i64>,
    /// Every frame the card delivered, CRC failures included.
    pub frames: u64,
    pub crc_fail: u64,
    /// Microseconds the card has spent off the link card's channel (UI
    /// stats `card_off_us`, from the runtime's retunes); 0 where unknown.
    pub off_link_us: u64,
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
    /// The card sat on the link channel long enough in the window to be
    /// called deaf or hearing; `fps` is then per second on that channel.
    pub judged: bool,
}

/// The best card must deliver this many frames/s before another one can be
/// called deaf: below it (search, a dark drone) every card is quiet.
const DEAF_MIN_BEST_FPS: f64 = 50.0;
/// A card on the link channel hears the same air as its neighbour; under a
/// tenth of the best card's frames it is not receiving (2026-11-02 Quest:
/// card 0 at 0.1 frames/s against card 1 at 230 with no USB error).
const DEAF_SHARE: f64 = 0.1;
/// A card is judged only on the time it spent on the link channel, and
/// only with this much of it in the window: a scout card's prelude burst
/// keeps it off the link for over a second of each burst (four 320 ms
/// dwells on a Quest with two cards), and its frames over the whole window
/// then read as a deaf card. Half a second keeps the first verdict on a
/// card that never leaves the link at the second 1 Hz sample, as before.
const MIN_ON_LINK_S: f64 = 0.5;

/// Per-card rates from two snapshots `dt` seconds apart; empty when the
/// card count changed between them.
pub fn card_rates(old: &[CardRx], new: &[CardRx], dt: f64) -> Vec<CardRate> {
    if old.len() != new.len() || dt <= 0.0 {
        return Vec::new();
    }
    // (fps, judged, frames, crc failures)
    let raw: Vec<(f64, bool, u64, u64)> = old
        .iter()
        .zip(new)
        .map(|(a, b)| {
            let f = b.frames.saturating_sub(a.frames);
            let away = b.off_link_us.saturating_sub(a.off_link_us) as f64 / 1e6;
            let on = (dt - away).max(0.0);
            let judged = on >= MIN_ON_LINK_S;
            let fps = f as f64 / if judged { on } else { dt };
            (fps, judged, f, b.crc_fail.saturating_sub(a.crc_fail))
        })
        .collect();
    let best = raw.iter().filter(|r| r.1).map(|r| r.0).fold(0.0, f64::max);
    raw.iter()
        .zip(new)
        .map(|(&(fps, judged, f, crc), c)| CardRate {
            rssi: c.rssi,
            fps,
            crc_pct: (f > 0).then(|| 100.0 * crc.min(f) as f64 / f as f64),
            deaf: judged && best >= DEAF_MIN_BEST_FPS && fps < DEAF_SHARE * best,
            judged,
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
        let best = rates
            .iter()
            .filter(|r| r.judged)
            .map(|r| r.fps)
            .fold(0.0, f64::max);
        let mut out = Vec::new();
        for (i, r) in rates.iter().enumerate() {
            // A card away from the link keeps its last verdict.
            if r.judged && r.deaf != self.deaf[i] {
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

/// The per-card receive counters of a parsed stats line, with each card's
/// time off the link channel where the line has it.
pub fn cards_of(v: &Value) -> Vec<CardRx> {
    let off = list(v, "card_off_us", |o| o.as_u64().unwrap_or(0));
    let mut cards = list(v, "cards", |c| CardRx {
        rssi: opt_i64(c, "rssi"),
        frames: u64_or(c, "frames", 0),
        crc_fail: u64_or(c, "crc_fail", 0),
        off_link_us: 0,
    });
    for (c, o) in cards.iter_mut().zip(off) {
        c.off_link_us = o;
    }
    cards
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The flight log's stats lines on the Quest came 1.007 to 1.016 s
    /// apart (1.013 on average), so the 3 s window spans two intervals.
    const STEP_S: f64 = 1.013;

    /// Two cards sampled as the flight log does, sample `t` at
    /// `t * STEP_S`: card 0 on the link at 1500 frames/s; card 1 at
    /// `on_fps` while on the link and silent while away; `away(t)` is its
    /// time off the link in the interval ending at sample `t`, seconds.
    fn run(
        samples: u64,
        on_fps: f64,
        away: impl Fn(u64) -> f64,
        report_away: bool,
    ) -> Vec<(u64, DeafChange)> {
        let mut w = DeafWatch::default();
        let (mut f1, mut off1) = (0.0, 0.0);
        let mut out = Vec::new();
        for t in 0..=samples {
            if t > 0 {
                let a = away(t).clamp(0.0, STEP_S);
                f1 += on_fps * (STEP_S - a);
                off1 += a;
            }
            let ts = t as f64 * STEP_S;
            let cards = [
                CardRx {
                    frames: (1500.0 * ts) as u64,
                    ..CardRx::default()
                },
                CardRx {
                    frames: f1 as u64,
                    off_link_us: if report_away { (off1 * 1e6) as u64 } else { 0 },
                    ..CardRx::default()
                },
            ];
            out.extend(w.push(ts, &cards).into_iter().map(|c| (t, c)));
        }
        out
    }

    /// The prelude as a Quest with two cards ran it: bursts of four
    /// 320 ms dwells 7.4 s apart (rounded to 7 s here), each taking the
    /// scout about 1.3 s off the link, split over two seconds.
    fn prelude(t: u64) -> f64 {
        match t % 7 {
            3 => 0.6,
            4 => 0.7,
            _ => 0.0,
        }
    }

    #[test]
    fn a_scouting_card_is_not_called_deaf() {
        // 300 frames/s on the link, as card 1 heard between bursts.
        let today = run(25, 300.0, prelude, false);
        assert!(
            today.iter().any(|(_, c)| c.card == 1 && c.deaf),
            "the model must show the false flag of the whole-window rate"
        );
        let now = run(25, 300.0, prelude, true);
        assert!(now.is_empty(), "{now:?}");
        // Away for one second in every three.
        let third = run(25, 300.0, |t| if t % 3 == 0 { 1.0 } else { 0.0 }, true);
        assert!(third.is_empty(), "{third:?}");
    }

    #[test]
    fn a_silent_card_on_the_link_is_flagged_as_soon_as_before() {
        let today = run(5, 0.0, |_| 0.0, false);
        let now = run(5, 0.0, |_| 0.0, true);
        assert_eq!(today.len(), 1);
        assert_eq!(now.len(), 1);
        let (t, c) = &now[0];
        assert_eq!((*t, c.card, c.deaf), (today[0].0, 1, true));
        assert_eq!(*t, 1, "the second sample");
        assert_eq!(c.fps, 0.0);
    }

    #[test]
    fn a_deaf_card_that_scouts_is_still_flagged() {
        // Silent everywhere and sent scouting a second in every three.
        let now = run(25, 0.0, |t| if t % 3 == 1 { 1.0 } else { 0.0 }, true);
        assert_eq!(now.len(), 1, "{now:?}");
        assert!(now[0].1.deaf && now[0].1.card == 1);
        assert!(now[0].0 <= 3, "flagged at t={}", now[0].0);
        // In the prelude pattern it is flagged before the first burst and
        // stays flagged through every one.
        let pre = run(25, 0.0, prelude, true);
        assert_eq!(pre.len(), 1, "{pre:?}");
        assert_eq!(pre[0].0, 1);
    }

    #[test]
    fn a_card_away_the_whole_window_keeps_its_verdict() {
        // Hearing, then parked off the link (a scout between bursts): no
        // verdict, so no flip either way.
        let now = run(12, 1500.0, |t| if t > 4 { 1.0 } else { 0.0 }, true);
        assert!(now.is_empty(), "{now:?}");
    }

    #[test]
    fn cards_of_reads_the_off_link_times() {
        let v: Value = serde_json::from_str(
            r#"{"cards":[{"frames":5},{"frames":7}],"card_off_us":[0,1300000]}"#,
        )
        .unwrap();
        let c = cards_of(&v);
        assert_eq!((c[0].off_link_us, c[1].off_link_us), (0, 1_300_000));
        let v: Value = serde_json::from_str(r#"{"cards":[{"frames":5}]}"#).unwrap();
        assert_eq!(cards_of(&v)[0].off_link_us, 0);
    }
}
