//! Flight telemetry from the drone's MAVLink stream (T_MAV): the FC's
//! messages as maburd's MSP bridge emits them (`common/src/msp_telemetry.cpp`:
//! HEARTBEAT, SYS_STATUS, ATTITUDE, GLOBAL_POSITION_INT, RC_CHANNELS,
//! VFR_HUD, BATTERY_STATUS, STATUSTEXT; GPS_RAW_INT when the FC has GPS).
//! Field offsets are the MAVLink wire order (common.xml); payloads arrive
//! zero-extended to full length from the parser.
use std::collections::VecDeque;

use egui::{Color32, RichText, Ui};
use pal_wire::mavlink::{Framing, Parser};

const MSG_HEARTBEAT: u32 = 0;
const MSG_SYS_STATUS: u32 = 1;
const MSG_GPS_RAW_INT: u32 = 24;
const MSG_ATTITUDE: u32 = 30;
const MSG_GLOBAL_POSITION_INT: u32 = 33;
const MSG_VFR_HUD: u32 = 74;
const MSG_BATTERY_STATUS: u32 = 147;
const MSG_STATUSTEXT: u32 = 253;
const ARMED: u8 = 128;
const FC_SYS: u8 = 1;
const FC_COMP: u8 = 1;

fn f32_at(p: &[u8], o: usize) -> f32 {
    f32::from_le_bytes([p[o], p[o + 1], p[o + 2], p[o + 3]])
}
fn i32_at(p: &[u8], o: usize) -> i32 {
    i32::from_le_bytes([p[o], p[o + 1], p[o + 2], p[o + 3]])
}
fn u16_at(p: &[u8], o: usize) -> u16 {
    u16::from_le_bytes([p[o], p[o + 1]])
}
fn i16_at(p: &[u8], o: usize) -> i16 {
    i16::from_le_bytes([p[o], p[o + 1]])
}

#[derive(Clone, Debug, Default)]
pub struct Telemetry {
    pub last_s: Option<f64>,
    pub packets: u64,
    pub armed: bool,
    pub custom_mode: u32,
    pub vehicle_type: u8,
    pub roll_deg: f32,
    pub pitch_deg: f32,
    pub yaw_deg: f32,
    pub alt_rel_m: Option<f32>,
    pub groundspeed_ms: Option<f32>,
    pub climb_ms: Option<f32>,
    pub heading_deg: Option<i16>,
    pub throttle_pct: Option<u16>,
    pub batt_v: Option<f32>,
    pub batt_a: Option<f32>,
    pub batt_mah: Option<i32>,
    pub batt_pct: Option<i8>,
    pub sats: Option<u8>,
    pub fix_type: u8,
    pub lat_lon: Option<(f64, f64)>,
    pub home: Option<(f64, f64)>,
    pub texts: VecDeque<(f64, u8, String)>,
}

/// Great-circle distance in metres.
pub fn distance_m(a: (f64, f64), b: (f64, f64)) -> f64 {
    let (la1, la2) = (a.0.to_radians(), b.0.to_radians());
    let dla = la2 - la1;
    let dlo = (b.1 - a.1).to_radians();
    let h = (dla / 2.0).sin().powi(2) + la1.cos() * la2.cos() * (dlo / 2.0).sin().powi(2);
    2.0 * 6_371_000.0 * h.sqrt().asin()
}

impl Telemetry {
    /// One whole MAVLink packet (as `CoreIo::on_mav` delivers it).
    pub fn on_packet(&mut self, pkt: &[u8], now_s: f64) {
        let mut p = Parser::default();
        let ok = pkt
            .iter()
            .fold(false, |ok, &b| ok | (p.feed(b) == Framing::Ok));
        if !ok {
            return;
        }
        let m = p.message();
        let pl = &m.payload;
        if m.msgid == MSG_STATUSTEXT {
            let raw = &pl[1..51];
            let len = raw.iter().position(|&c| c == 0).unwrap_or(50);
            self.texts.push_back((
                now_s,
                pl[0],
                String::from_utf8_lossy(&raw[..len]).into_owned(),
            ));
            while self.texts.len() > 5 {
                self.texts.pop_front();
            }
            return;
        }
        if (m.sysid, m.compid) != (FC_SYS, FC_COMP) {
            return; // maburd's own component (191) speaks params, not flight state
        }
        self.packets += 1;
        self.last_s = Some(now_s);
        match m.msgid {
            MSG_HEARTBEAT => {
                self.custom_mode = u32::from_le_bytes([pl[0], pl[1], pl[2], pl[3]]);
                self.vehicle_type = pl[4];
                let armed = pl[6] & ARMED != 0;
                // Home is where the drone arms with a fix.
                if armed && !self.armed {
                    self.home = self.lat_lon;
                }
                self.armed = armed;
            }
            MSG_SYS_STATUS => {
                let mv = u16_at(pl, 14);
                self.batt_v = (mv != u16::MAX).then(|| f32::from(mv) / 1000.0);
                let ca = i16_at(pl, 16);
                self.batt_a = (ca >= 0).then(|| f32::from(ca) / 100.0);
                let pct = pl[30] as i8;
                self.batt_pct = (pct >= 0).then_some(pct);
            }
            MSG_GPS_RAW_INT => {
                self.fix_type = pl[28];
                self.sats = (pl[29] != u8::MAX).then_some(pl[29]);
            }
            MSG_ATTITUDE => {
                self.roll_deg = f32_at(pl, 4).to_degrees();
                self.pitch_deg = f32_at(pl, 8).to_degrees();
                self.yaw_deg = f32_at(pl, 12).to_degrees();
            }
            MSG_GLOBAL_POSITION_INT => {
                let (lat, lon) = (i32_at(pl, 4), i32_at(pl, 8));
                if lat != 0 || lon != 0 {
                    let ll = (f64::from(lat) / 1e7, f64::from(lon) / 1e7);
                    self.lat_lon = Some(ll);
                    if self.home.is_none() {
                        self.home = Some(ll);
                    }
                }
                self.alt_rel_m = Some(i32_at(pl, 16) as f32 / 1000.0);
            }
            MSG_VFR_HUD => {
                self.groundspeed_ms = Some(f32_at(pl, 4));
                self.climb_ms = Some(f32_at(pl, 12));
                self.heading_deg = Some(i16_at(pl, 16));
                self.throttle_pct = Some(u16_at(pl, 18));
            }
            MSG_BATTERY_STATUS => {
                let mah = i32_at(pl, 0);
                self.batt_mah = (mah >= 0).then_some(mah);
            }
            _ => {}
        }
    }

    pub fn home_distance_m(&self) -> Option<f64> {
        Some(distance_m(self.home?, self.lat_lon?))
    }
}

fn opt<T: std::fmt::Display>(v: Option<T>, unit: &str) -> String {
    v.map_or("-".into(), |v| format!("{v}{unit}"))
}

/// The telemetry block: armed, attitude, altitude, speed, battery, GPS, texts.
pub fn widget(ui: &mut Ui, t: &Telemetry, now_s: f64) {
    let stale = t.last_s.is_none_or(|s| now_s - s > 3.0);
    egui::Frame::NONE
        .fill(Color32::from_black_alpha(160))
        .corner_radius(4.0)
        .inner_margin(6.0)
        .show(ui, |ui| {
            ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
            ui.set_min_width(250.0);
            if t.last_s.is_none() {
                ui.label(RichText::new("no FC telemetry").color(Color32::GRAY));
                return;
            }
            let (arm, c) = if t.armed {
                ("ARMED", Color32::from_rgb(230, 80, 70))
            } else {
                ("disarmed", Color32::from_rgb(90, 200, 120))
            };
            ui.horizontal(|ui| {
                ui.label(RichText::new(arm).strong().color(c));
                if stale {
                    ui.label(RichText::new("stale").color(Color32::from_rgb(230, 180, 60)));
                }
            });
            let mono = |ui: &mut Ui, s: String| ui.label(RichText::new(s).monospace());
            mono(
                ui,
                format!(
                    "att  R{:+5.0} P{:+5.0} Y{:4.0}",
                    t.roll_deg, t.pitch_deg, t.yaw_deg
                ),
            );
            mono(
                ui,
                format!(
                    "alt  {}  vs {}",
                    opt(t.alt_rel_m.map(|a| format!("{a:.1}")), " m"),
                    opt(t.climb_ms.map(|c| format!("{c:+.1}")), " m/s")
                ),
            );
            mono(
                ui,
                format!(
                    "spd  {}  thr {}",
                    opt(t.groundspeed_ms.map(|v| format!("{:.0}", v * 3.6)), " km/h"),
                    opt(t.throttle_pct, " %")
                ),
            );
            mono(
                ui,
                format!(
                    "bat  {}  {}  {}",
                    opt(t.batt_v.map(|v| format!("{v:.2}")), " V"),
                    opt(t.batt_a.map(|a| format!("{a:.1}")), " A"),
                    opt(t.batt_mah, " mAh")
                ),
            );
            mono(
                ui,
                format!(
                    "gps  {} sats  home {}",
                    opt(t.sats, ""),
                    opt(t.home_distance_m().map(|d| format!("{d:.0}")), " m")
                ),
            );
            for (_, sev, txt) in &t.texts {
                let c = if *sev <= 3 {
                    Color32::from_rgb(230, 80, 70)
                } else {
                    Color32::LIGHT_GRAY
                };
                ui.label(RichText::new(txt).small().color(c));
            }
        });
}

#[cfg(test)]
mod tests {
    use super::*;
    use pal_wire::mavlink::pack_v2;

    #[test]
    fn heartbeat_attitude_battery() {
        let mut t = Telemetry::default();
        let mut hb = [0u8; 9];
        hb[4] = 2;
        hb[6] = ARMED | 1;
        t.on_packet(&pack_v2(MSG_HEARTBEAT, 1, 1, 0, &hb), 1.0);
        let mut att = [0u8; 28];
        att[4..8].copy_from_slice(&(0.5f32).to_le_bytes());
        t.on_packet(&pack_v2(MSG_ATTITUDE, 1, 1, 1, &att), 1.1);
        let mut ss = [0u8; 31];
        ss[14..16].copy_from_slice(&16_100u16.to_le_bytes());
        ss[16..18].copy_from_slice(&1234i16.to_le_bytes());
        ss[30] = 80;
        t.on_packet(&pack_v2(MSG_SYS_STATUS, 1, 1, 2, &ss), 1.2);
        assert!(t.armed);
        assert!((t.roll_deg - 28.6479).abs() < 1e-3);
        assert_eq!(t.batt_v, Some(16.1));
        assert_eq!(t.batt_a, Some(12.34));
        assert_eq!(t.batt_pct, Some(80));
        // maburd's own component is ignored for flight state.
        t.on_packet(&pack_v2(MSG_HEARTBEAT, 1, 191, 3, &[0; 9]), 1.3);
        assert!(t.armed);
    }

    #[test]
    fn haversine() {
        let d = distance_m((52.52, 13.405), (52.53, 13.405));
        assert!((d - 1112.0).abs() < 2.0, "{d}");
    }
}
