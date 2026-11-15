//! The preflight checklist (spec F22, pal-ttb.13): what decides a flight
//! on the ground, each row green, amber or red with one line of what to
//! do. Rows are pure functions of the latest stats snapshot, the card
//! rates and the FC telemetry, so every front end shows the same list.
//! A red row never blocks anything: the GS does not stand between the
//! pilot and the sticks.
use crate::telem::Telemetry;
use crate::{CardRate, Snapshot};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Level {
    Ok,
    Warn,
    Bad,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Check {
    pub level: Level,
    /// Short name: "cards", "key", ...
    pub what: &'static str,
    /// What was seen and, when not green, what to do.
    pub text: String,
}

fn row(level: Level, what: &'static str, text: impl Into<String>) -> Check {
    Check {
        level,
        what,
        text: text.into(),
    }
}

/// Cards on the same air within this many dB of each other; more is an
/// antenna off or cross-polarised (2026-09-19 Quest: one antenna of four
/// LHCP, 13 dB down, found only after the flights).
pub const CARD_SPREAD_DB: i64 = 8;
/// Uplink delivery (the drone's count of our RC frames) below which an
/// order may wait a feedback period or more.
pub const UPLINK_WARN_PCT: i64 = 90;
pub const UPLINK_BAD_PCT: i64 = 70;
/// Drone SoC temperature: the bench drone without its fan reached 99 C
/// and shut down in about 14 minutes.
pub const TEMP_WARN_C: i64 = 75;
pub const TEMP_BAD_C: i64 = 85;
/// Satellites for a usable home point.
pub const MIN_SATS: u8 = 6;
/// Loss after FEC on the ground: with the drone a few metres away anything
/// above this is interference or a receiver problem.
pub const RESIDUAL_WARN: f64 = 0.01;

/// The checklist for one snapshot. `rates` are the per-card rates
/// (`History::card_rates`, empty with one card), `telem_fresh` whether FC
/// telemetry arrived within its stale window.
pub fn checks(s: &Snapshot, rates: &[CardRate], t: &Telemetry, telem_fresh: bool) -> Vec<Check> {
    let mut v = Vec::new();
    // The link itself, and the build pair.
    if let Some((drone, app)) = s.peer_rc {
        v.push(row(
            Level::Bad,
            "link",
            format!("drone speaks RC {drone}, app RC {app}: update one of them"),
        ));
    } else if s.key_mismatch {
        v.push(row(Level::Bad, "link", "pairing key mismatch: pair again"));
    } else if !s.session {
        v.push(row(Level::Bad, "link", "no drone link"));
    } else {
        v.push(row(Level::Ok, "link", format!("linked, {} AUs", s.aus)));
    }
    if s.key_fp == "default" {
        v.push(row(
            Level::Warn,
            "key",
            "default pairing key: any GS with it can fly this drone's link",
        ));
    }
    if let Some(u) = &s.update {
        let (level, text) = match u.state.as_str() {
            "current" => (Level::Ok, "drone build matches the app".to_string()),
            "custom_wait" => (Level::Ok, "drone runs a custom build".to_string()),
            "updating" | "updated" | "waiting" | "busy" => {
                (Level::Warn, format!("drone update {}", u.state))
            }
            other => (
                Level::Warn,
                format!("drone build differs from the app ({other})"),
            ),
        };
        v.push(row(level, "build", text));
        if u.fallback {
            v.push(row(
                Level::Warn,
                "build",
                "the drone's boot guard restored its last proven build",
            ));
        }
    }
    // Receivers.
    if let Some((i, _)) = rates.iter().enumerate().find(|(_, r)| r.deaf) {
        v.push(row(
            Level::Bad,
            "cards",
            format!("card {i} hears nothing: reseat its cable and antennas"),
        ));
    } else if !rates.is_empty() {
        let heard: Vec<(usize, i64)> = rates
            .iter()
            .enumerate()
            .filter(|(_, r)| r.judged)
            .filter_map(|(i, r)| r.rssi.map(|x| (i, x)))
            .collect();
        let best = heard.iter().max_by_key(|h| h.1);
        let worst = heard.iter().min_by_key(|h| h.1);
        match (best, worst) {
            (Some(b), Some(w)) if b.1 - w.1 > CARD_SPREAD_DB => v.push(row(
                Level::Warn,
                "cards",
                format!(
                    "card {} is {} dB below card {}: check its antennas and their polarisation",
                    w.0,
                    b.1 - w.1,
                    b.0
                ),
            )),
            _ => v.push(row(
                Level::Ok,
                "cards",
                format!("{} cards receiving", rates.len()),
            )),
        }
    }
    if let Some(r) = s.residual.filter(|r| *r > RESIDUAL_WARN) {
        v.push(row(
            Level::Warn,
            "loss",
            format!(
                "{:.1} % lost after FEC on the ground: busy channel or a receiver problem",
                100.0 * r
            ),
        ));
    }
    if let Some(p) = s.uplink_pct {
        let level = if p < UPLINK_BAD_PCT {
            Level::Bad
        } else if p < UPLINK_WARN_PCT {
            Level::Warn
        } else {
            Level::Ok
        };
        v.push(row(
            level,
            "uplink",
            format!("drone hears {p} % of our frames"),
        ));
    }
    // The drone itself.
    if let Some(c) = s.drone_temp_c {
        let level = if c >= TEMP_BAD_C {
            Level::Bad
        } else if c >= TEMP_WARN_C {
            Level::Warn
        } else {
            Level::Ok
        };
        let text = if level == Level::Ok {
            format!("drone {c} C")
        } else {
            format!("drone {c} C: check its fan or airflow")
        };
        v.push(row(level, "temp", text));
    }
    // The flight controller.
    if !telem_fresh {
        v.push(row(Level::Warn, "fc", "no FC telemetry"));
    } else if let Some(n) = t.sats {
        if n < MIN_SATS {
            v.push(row(
                Level::Warn,
                "gps",
                format!("{n} satellites: wait for a fix"),
            ));
        } else if t.home.is_none() {
            v.push(row(
                Level::Warn,
                "gps",
                format!("{n} satellites, no home point"),
            ));
        } else {
            v.push(row(Level::Ok, "gps", format!("{n} satellites, home set")));
        }
    }
    // What this GS records.
    if let Some(e) = &s.log_err {
        v.push(row(Level::Bad, "log", format!("flight log stopped: {e}")));
    }
    if let Some(d) = s.dvr.as_ref().filter(|d| d.state == "error") {
        v.push(row(
            Level::Bad,
            "dvr",
            format!("DVR failed: {}", d.error.as_deref().unwrap_or("?")),
        ));
    }
    v
}

/// The worst level of the list (Ok for an empty one).
pub fn worst(v: &[Check]) -> Level {
    v.iter().map(|c| c.level).max().unwrap_or(Level::Ok)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::update::UpdateView;

    fn linked() -> Snapshot {
        Snapshot {
            session: true,
            aus: 100,
            key_fp: "a1b2".into(),
            ..Snapshot::default()
        }
    }

    fn rate(rssi: i64, deaf: bool) -> CardRate {
        CardRate {
            rssi: Some(rssi),
            fps: if deaf { 0.0 } else { 200.0 },
            crc_pct: None,
            deaf,
            judged: true,
        }
    }

    fn level_of(v: &[Check], what: &str) -> Option<Level> {
        v.iter().find(|c| c.what == what).map(|c| c.level)
    }

    #[test]
    fn a_healthy_ground_is_all_green() {
        let mut s = linked();
        s.uplink_pct = Some(98);
        s.drone_temp_c = Some(60);
        s.update = Some(UpdateView {
            state: "current".into(),
            ..UpdateView::default()
        });
        let t = Telemetry {
            sats: Some(12),
            home: Some((52.5, 13.4)),
            ..Telemetry::default()
        };
        let v = checks(&s, &[rate(-30, false), rate(-33, false)], &t, true);
        assert_eq!(worst(&v), Level::Ok, "{v:?}");
        for w in ["link", "build", "cards", "uplink", "temp", "gps"] {
            assert_eq!(level_of(&v, w), Some(Level::Ok), "{w}");
        }
    }

    #[test]
    fn each_ground_fault_shows_its_row() {
        // No link.
        let v = checks(&Snapshot::default(), &[], &Telemetry::default(), false);
        assert_eq!(level_of(&v, "link"), Some(Level::Bad));
        assert_eq!(level_of(&v, "fc"), Some(Level::Warn));
        // RC version and key.
        let mut s = linked();
        s.peer_rc = Some((19, 18));
        assert_eq!(
            level_of(&checks(&s, &[], &Telemetry::default(), true), "link"),
            Some(Level::Bad)
        );
        let mut s = linked();
        s.key_fp = "default".into();
        assert_eq!(
            level_of(&checks(&s, &[], &Telemetry::default(), true), "key"),
            Some(Level::Warn)
        );
        // A deaf card is red, an antenna 13 dB down amber (the Quest LHCP case).
        let s = linked();
        let t = Telemetry::default();
        assert_eq!(
            level_of(
                &checks(&s, &[rate(-30, false), rate(-90, true)], &t, true),
                "cards"
            ),
            Some(Level::Bad)
        );
        let v = checks(&s, &[rate(-30, false), rate(-43, false)], &t, true);
        assert_eq!(level_of(&v, "cards"), Some(Level::Warn));
        assert!(
            v.iter()
                .any(|c| c.text.contains("card 1 is 13 dB below card 0")),
            "{v:?}"
        );
        assert_eq!(
            level_of(
                &checks(&s, &[rate(-30, false), rate(-38, false)], &t, true),
                "cards"
            ),
            Some(Level::Ok)
        );
        // Uplink and temperature thresholds.
        for (p, l) in [(95, Level::Ok), (80, Level::Warn), (60, Level::Bad)] {
            let mut s = linked();
            s.uplink_pct = Some(p);
            assert_eq!(
                level_of(&checks(&s, &[], &t, true), "uplink"),
                Some(l),
                "{p}"
            );
        }
        for (c, l) in [(70, Level::Ok), (80, Level::Warn), (95, Level::Bad)] {
            let mut s = linked();
            s.drone_temp_c = Some(c);
            assert_eq!(level_of(&checks(&s, &[], &t, true), "temp"), Some(l), "{c}");
        }
        // GPS: few satellites, then no home.
        let few = Telemetry {
            sats: Some(4),
            ..Telemetry::default()
        };
        assert_eq!(
            level_of(&checks(&s, &[], &few, true), "gps"),
            Some(Level::Warn)
        );
        let nohome = Telemetry {
            sats: Some(10),
            ..Telemetry::default()
        };
        assert_eq!(
            level_of(&checks(&s, &[], &nohome, true), "gps"),
            Some(Level::Warn)
        );
        // Recording failures.
        let mut s = linked();
        s.log_err = Some("disk full".into());
        assert_eq!(
            level_of(&checks(&s, &[], &t, true), "log"),
            Some(Level::Bad)
        );
    }
}
