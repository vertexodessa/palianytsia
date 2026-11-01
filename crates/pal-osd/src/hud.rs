//! The flight OSD: what a pilot looks at. Centre dot, battery, flight timer,
//! altitude/speed/home/sats, FC messages and one compact link readout,
//! painted straight over the video rect with shadowed text so it reads on
//! any background. The detailed link statistics (`chips`) are a separate,
//! off-by-default view.
use egui::{Align2, Color32, FontId, Pos2, Rect, Stroke, Ui, Vec2};

use crate::prefs::HudPrefs;
use crate::telem::Telemetry;
use crate::{History, SIGNAL_HOLD_S};

const WHITE: Color32 = Color32::from_rgb(240, 240, 240);
const WARN: Color32 = Color32::from_rgb(240, 190, 60);
const BAD: Color32 = Color32::from_rgb(240, 80, 70);
const OK: Color32 = Color32::from_rgb(110, 220, 140);

/// Per-cell voltage thresholds for the battery colour (LiPo under load).
const CELL_WARN_V: f32 = 3.5;
const CELL_BAD_V: f32 = 3.3;
/// Telemetry older than this reads as lost.
const STALE_S: f64 = 3.0;
/// How long an FC message stays on screen.
const TEXT_S: f64 = 5.0;

fn text(ui: &Ui, pos: Pos2, align: Align2, s: &str, size: f32, c: Color32) -> Rect {
    let p = ui.painter();
    let f = FontId::monospace(size);
    // A 1-point black shadow keeps white text readable over sky and snow.
    for d in [Vec2::new(1.0, 1.0), Vec2::new(-1.0, 1.0)] {
        p.text(pos + d, align, s, f.clone(), Color32::from_black_alpha(220));
    }
    p.text(pos, align, s, f, c)
}

fn mmss(s: f64) -> String {
    let s = s.max(0.0) as u64;
    format!("{:02}:{:02}", s / 60, s % 60)
}

/// Paints the HUD into `r` (the video rect, in points).
pub fn hud(ui: &Ui, r: Rect, t: &Telemetry, h: &History, now_s: f64, pf: &HudPrefs) {
    let base = (r.height() / 22.0).clamp(11.0, 22.0) * pf.scale;
    let pad = base * 0.8;
    let p = ui.painter();

    let c = r.center();
    if pf.dot {
        p.circle(c, base * 0.18, WHITE, Stroke::new(1.0_f32, Color32::BLACK));
    }

    let fresh = t.last_s.is_some_and(|s| now_s - s <= STALE_S);
    let link = h.last();
    let session = link.is_some_and(|s| s.session);

    // Top left: arm state and flight timer.
    let tl = r.left_top() + Vec2::new(pad, pad);
    if !pf.timer {
    } else if t.last_s.is_none() {
        text(ui, tl, Align2::LEFT_TOP, "NO TELEMETRY", base, WARN);
    } else {
        let (arm, col) = if t.armed {
            ("ARMED", BAD)
        } else {
            ("DISARMED", OK)
        };
        let a = text(ui, tl, Align2::LEFT_TOP, arm, base, col);
        text(
            ui,
            a.right_top() + Vec2::new(base * 0.6, 0.0),
            Align2::LEFT_TOP,
            &mmss(t.flight_s(now_s)),
            base,
            WHITE,
        );
        if !fresh {
            text(
                ui,
                a.left_bottom() + Vec2::new(0.0, base * 0.2),
                Align2::LEFT_TOP,
                "TELEMETRY LOST",
                base * 0.8,
                BAD,
            );
        }
    }

    // Top right: one compact link readout (RSSI, MCS); details live in the
    // stats view.
    let tr = r.right_top() + Vec2::new(-pad, pad);
    match (session, h.signal(SIGNAL_HOLD_S)) {
        (true, _) if !pf.link => {}
        (true, Some((rssi, snr))) => {
            let mcs = link.map_or(-1, |s| s.mcs);
            let col = if snr > 15.0 { WHITE } else { WARN };
            text(
                ui,
                tr,
                Align2::RIGHT_TOP,
                &format!("{rssi:.0}dBm MCS{mcs}"),
                base * 0.9,
                col,
            );
        }
        (true, None) => {
            text(ui, tr, Align2::RIGHT_TOP, "NO SIGNAL", base * 0.9, BAD);
        }
        (false, _) => {
            text(ui, tr, Align2::RIGHT_TOP, "NO LINK", base, BAD);
        }
    }

    // Bottom left: battery, large.
    if pf.battery {
        battery(ui, r, t, base, pad);
    }
    if pf.nav {
        nav(ui, r, t, base, pad);
    }
    if pf.messages {
        messages(ui, c, r, t, base, now_s);
    }
}

fn battery(ui: &Ui, r: Rect, t: &Telemetry, base: f32, pad: f32) {
    let bl = r.left_bottom() + Vec2::new(pad, -pad);
    let cell = t.cell_v();
    let bcol = match cell {
        Some(v) if v < CELL_BAD_V => BAD,
        Some(v) if v < CELL_WARN_V => WARN,
        _ => WHITE,
    };
    let mut line2 = Vec::new();
    if let Some(v) = cell {
        line2.push(format!("{v:.2}V/c"));
    }
    if let Some(a) = t.batt_a {
        line2.push(format!("{a:.1}A"));
    }
    if let Some(m) = t.batt_mah {
        line2.push(format!("{m}mAh"));
    }
    let l2 = text(
        ui,
        bl,
        Align2::LEFT_BOTTOM,
        &line2.join("  "),
        base * 0.85,
        WHITE,
    );
    let pack = t.batt_v.map_or("--.-V".into(), |v| format!("{v:.1}V"));
    text(
        ui,
        l2.left_top() - Vec2::new(0.0, base * 0.15),
        Align2::LEFT_BOTTOM,
        &pack,
        base * 1.6,
        bcol,
    );
}

fn nav(ui: &Ui, r: Rect, t: &Telemetry, base: f32, pad: f32) {
    // Bottom right: altitude, speed, home, sats (top to bottom).
    let mut rows = Vec::new();
    if let Some(a) = t.alt_rel_m {
        rows.push(format!("ALT {a:.0}m"));
    }
    if let Some(v) = t.groundspeed_ms {
        rows.push(format!("SPD {:.0}km/h", v * 3.6));
    }
    if let Some(d) = t.home_distance_m() {
        rows.push(format!("HOME {d:.0}m"));
    }
    if let Some(s) = t.sats {
        rows.push(format!("SAT {s}"));
    }
    let mut y = r.bottom() - pad;
    for row in rows.iter().rev() {
        let b = text(
            ui,
            Pos2::new(r.right() - pad, y),
            Align2::RIGHT_BOTTOM,
            row,
            base,
            WHITE,
        );
        y = b.top() - base * 0.15;
    }
}

fn messages(ui: &Ui, c: Pos2, r: Rect, t: &Telemetry, base: f32, now_s: f64) {
    // Below centre: the newest FC message, red for warnings and worse.
    if let Some((ts, sev, msg)) = t.texts.back() {
        if now_s - ts <= TEXT_S {
            let col = if *sev <= 4 { BAD } else { WHITE };
            text(
                ui,
                c + Vec2::new(0.0, r.height() * 0.22),
                Align2::CENTER_CENTER,
                msg,
                base,
                col,
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::mmss;

    #[test]
    fn timer_format() {
        assert_eq!(mmss(0.0), "00:00");
        assert_eq!(mmss(83.9), "01:23");
        assert_eq!(mmss(-5.0), "00:00");
    }
}
