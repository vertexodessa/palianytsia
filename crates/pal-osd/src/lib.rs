//! OSD widgets (spec section 6) drawn from the core's `stats_json` line, so
//! every app and both cores share one OSD. Reads only the snapshot it is
//! handed: no locks toward the RX or core path.
pub mod cal;
pub mod console;
pub mod hud;
pub mod panel;
pub mod params;
pub mod prefs;
pub mod telem;

use std::collections::VecDeque;

use egui::{Align2, Color32, FontId, Pos2, Rect, Stroke, Ui, Vec2};
use serde_json::Value;

/// The fields the OSD shows, parsed from one stats line.
#[derive(Clone, Debug, Default)]
pub struct Snapshot {
    pub mode: String,
    pub session: bool,
    pub key_mismatch: bool,
    pub rung: i64,
    pub mcs: i64,
    pub bw: i64,
    pub drone_state: Option<i64>,
    pub rssi_dbm: Option<f64>,
    pub snr_db: Option<f64>,
    pub bodies: u64,
    pub aus: u64,
    pub trunc: u64,
    pub salvaged: u64,
    pub sends: u64,
    pub resets: u64,
    /// Fingerprint of our pairing key; "default" for `mabur-default-00`.
    pub key_fp: String,
    pub rtt_ms: Option<f64>,
    /// "none" | "wait" | "ok" | ... (UplinkDelivery), with its window %.
    pub uplink: String,
    pub uplink_pct: Option<i64>,
    /// Link health: loss before FEC, and what FEC left (post-FEC), 0..1.
    pub pre_fec_loss: Option<f64>,
    pub residual: Option<f64>,
    /// Per-card receive (two or more cards only): smoothed RSSI and the
    /// video bodies the card has delivered.
    pub cards: Vec<CardRx>,
    /// The card the uplink transmits on (two or more cards only).
    pub tx_card: Option<u64>,
    pub drone_temp_c: Option<i64>,
    /// NACK repairs: symbols filled so far, and the current rate.
    pub nack_fill: u64,
    pub nack_fill_pps: Option<f64>,
    /// Rust core only (`GsCore::ladder_json`, merged in by the app).
    pub ladder: Option<Ladder>,
    /// The TX-power calibration block (UI stats line only).
    pub cal: Option<cal::CalView>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct CardRx {
    pub rssi: Option<i64>,
    pub video: u64,
}

/// "c0 TX -45 dBm 52 %  c1 -50 dBm 48 %": each card's RSSI and its share
/// of the video bodies received so far (both cards hear most of them, so
/// the shares say which one hears more), and which one transmits.
pub fn cards_line(cards: &[CardRx], tx: Option<u64>) -> String {
    let total: u64 = cards.iter().map(|c| c.video).sum();
    cards
        .iter()
        .enumerate()
        .map(|(i, c)| {
            let r = c.rssi.map_or("  -".into(), |r| format!("{r:>3}"));
            let share = (100 * c.video).checked_div(total).unwrap_or(0);
            let t = if tx == Some(i as u64) { " TX" } else { "" };
            format!("c{i}{t} {r} dBm {share:>2} %")
        })
        .collect::<Vec<_>>()
        .join("  ")
}

/// What the next rung promote waits on (`GsCore::ladder_json`).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Ladder {
    pub rung: i64,
    pub top: i64,
    /// None on the top rung.
    pub up: Option<LadderUp>,
    /// Last rung change: reason, from, to, age.
    pub last: Option<(String, i64, i64, i64)>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct LadderUp {
    pub u: f64,
    pub up_util: f64,
    pub clean_ms: i64,
    pub clean_need_ms: i64,
    pub probe: String,
    pub probe_rung: i64,
    pub probe_u: f64,
    pub probe_max_util: f64,
    pub streak: u64,
    pub streak_need: u64,
    pub penalty_ms: i64,
}

impl Ladder {
    fn parse(v: &Value) -> Option<Self> {
        let i = |k: &str| v.get(k).and_then(Value::as_i64);
        let f = |k: &str| v.get(k).and_then(Value::as_f64).unwrap_or(0.0);
        let up = v.get("probe").and_then(Value::as_str).map(|p| LadderUp {
            u: f("u"),
            up_util: f("up_util"),
            clean_ms: i("clean_ms").unwrap_or(0),
            clean_need_ms: i("clean_need_ms").unwrap_or(0),
            probe: p.to_string(),
            probe_rung: i("probe_rung").unwrap_or(-1),
            probe_u: f("probe_u"),
            probe_max_util: f("probe_max_util"),
            streak: u(v, "streak"),
            streak_need: u(v, "streak_need"),
            penalty_ms: i("penalty_ms").unwrap_or(0),
        });
        let last = v.get("last_reason").and_then(Value::as_str).map(|r| {
            (
                r.to_string(),
                i("last_from").unwrap_or(-1),
                i("last_to").unwrap_or(-1),
                i("last_age_ms").unwrap_or(0),
            )
        });
        Some(Self {
            rung: i("rung")?,
            top: i("top").unwrap_or(0),
            up,
            last,
        })
    }

    /// The chip text and whether the promote is blocked right now, in the
    /// order the controller checks: penalty, s1 loss, s1 clean run, probe.
    pub fn summary(&self) -> (String, bool) {
        let Some(u) = &self.up else {
            return ("top rung".into(), false);
        };
        if u.penalty_ms > 0 {
            return (
                format!(
                    "r{} blocked {:.1} s",
                    self.rung + 1,
                    u.penalty_ms as f64 / 1000.0
                ),
                true,
            );
        }
        if u.u >= u.up_util {
            return (format!("loss u {:.2} > {:.2}", u.u, u.up_util), true);
        }
        let clean = format!(
            "clean {:.1}/{:.1} s",
            u.clean_ms.min(u.clean_need_ms) as f64 / 1000.0,
            u.clean_need_ms as f64 / 1000.0
        );
        let probe = match u.probe.as_str() {
            "off" => "probe off".to_string(),
            "clean" => format!("probe r{} {}/{}", u.probe_rung, u.streak, u.streak_need),
            "lossy" => format!(
                "probe r{} lossy u {:.2} > {:.2}",
                u.probe_rung, u.probe_u, u.probe_max_util
            ),
            other => format!("probe r{} {other}", u.probe_rung),
        };
        let blocked = u.clean_ms < u.clean_need_ms
            || (u.probe != "off" && (u.probe != "clean" || u.streak < u.streak_need));
        (format!("{clean}  {probe}"), blocked)
    }
}

fn u(v: &Value, k: &str) -> u64 {
    v.get(k).and_then(Value::as_u64).unwrap_or(0)
}

impl Snapshot {
    pub fn parse(line: &str) -> Option<Self> {
        let v: Value = serde_json::from_str(line).ok()?;
        Some(Self {
            mode: v
                .get("mode")
                .and_then(Value::as_str)
                .unwrap_or("?")
                .to_string(),
            session: v.get("session").and_then(Value::as_bool).unwrap_or(false),
            key_mismatch: v
                .get("key_mismatch")
                .and_then(Value::as_bool)
                .unwrap_or(false),
            rung: v.get("rung").and_then(Value::as_i64).unwrap_or(-1),
            mcs: v.get("mcs").and_then(Value::as_i64).unwrap_or(-1),
            bw: v.get("bw").and_then(Value::as_i64).unwrap_or(0),
            drone_state: v.get("drone_state").and_then(Value::as_i64),
            rssi_dbm: v.get("rssi_dbm").and_then(Value::as_f64),
            snr_db: v.get("snr_db").and_then(Value::as_f64),
            bodies: u(&v, "bodies"),
            aus: u(&v, "aus"),
            trunc: u(&v, "trunc"),
            salvaged: u(&v, "salvaged"),
            sends: u(&v, "sends"),
            resets: u(&v, "resets"),
            key_fp: v
                .get("key_fp")
                .and_then(Value::as_str)
                .unwrap_or("")
                .to_string(),
            rtt_ms: v.get("rtt_ms").and_then(Value::as_f64),
            uplink: v
                .get("uplink")
                .and_then(Value::as_str)
                .unwrap_or("none")
                .to_string(),
            uplink_pct: v.get("uplink_pct").and_then(Value::as_i64),
            pre_fec_loss: v.get("pre_fec_loss").and_then(Value::as_f64),
            residual: v.get("residual").and_then(Value::as_f64),
            cards: v
                .get("cards")
                .and_then(Value::as_array)
                .map(|a| {
                    a.iter()
                        .map(|c| CardRx {
                            rssi: c.get("rssi").and_then(Value::as_i64),
                            video: u(c, "video"),
                        })
                        .collect()
                })
                .unwrap_or_default(),
            tx_card: v.get("tx_card").and_then(Value::as_u64),
            drone_temp_c: v.get("drone_temp_c").and_then(Value::as_i64),
            nack_fill: v.get("nack").map_or(0, |n| u(n, "fill")),
            nack_fill_pps: v
                .get("nack")
                .and_then(|n| n.get("fill_pps"))
                .and_then(Value::as_f64),
            ladder: v.get("ladder").and_then(Ladder::parse),
            cal: v.get("cal").and_then(cal::CalView::parse),
        })
    }
}

/// Drone `Telem.state` names (mabur rc_proto).
pub fn drone_state_name(s: i64) -> &'static str {
    match s {
        0 => "boot",
        1 => "rendezvous",
        2 => "linked",
        3 => "failsafe",
        _ => "?",
    }
}

const OK: Color32 = Color32::from_rgb(90, 200, 120);
const WARN: Color32 = Color32::from_rgb(230, 180, 60);
const BAD: Color32 = Color32::from_rgb(230, 80, 70);

trait Severity {
    /// At least WARN (BAD stays BAD).
    fn max_warn(self) -> Self;
}
impl Severity for Color32 {
    fn max_warn(self) -> Self {
        if self == BAD {
            BAD
        } else {
            WARN
        }
    }
}

/// Per-second history for the 60 s monitor plots.
#[derive(Default)]
pub struct History {
    rows: VecDeque<(f64, Snapshot)>,
}

impl History {
    pub const SPAN_S: f64 = 60.0;
    pub fn push(&mut self, t_s: f64, s: Snapshot) {
        self.rows.push_back((t_s, s));
        while self
            .rows
            .front()
            .is_some_and(|(t, _)| t_s - t > Self::SPAN_S)
        {
            self.rows.pop_front();
        }
    }
    pub fn last(&self) -> Option<&Snapshot> {
        self.rows.back().map(|(_, s)| s)
    }
    /// Per-interval deltas of a counter, as (t, value per second).
    fn rate(&self, f: impl Fn(&Snapshot) -> u64) -> Vec<(f64, f64)> {
        self.rows
            .iter()
            .zip(self.rows.iter().skip(1))
            .filter_map(|((t0, a), (t1, b))| {
                let dt = t1 - t0;
                (dt > 0.0).then(|| (*t1, f(b).saturating_sub(f(a)) as f64 / dt))
            })
            .collect()
    }
    /// The newest (rssi, snr) pair no older than `hold_s` before the last
    /// row. A stats window with no drone frame in it reports both as null,
    /// so taking only the last row made the chip blink "no signal" on a
    /// healthy link.
    pub fn signal(&self, hold_s: f64) -> Option<(f64, f64)> {
        let t_last = self.rows.back()?.0;
        self.rows
            .iter()
            .rev()
            .take_while(|(t, _)| t_last - t <= hold_s)
            .find_map(|(_, s)| s.rssi_dbm.zip(s.snr_db))
    }
    fn series(&self, f: impl Fn(&Snapshot) -> Option<f64>) -> Vec<(f64, f64)> {
        self.rows
            .iter()
            .filter_map(|(t, s)| f(s).map(|v| (*t, v)))
            .collect()
    }
}

fn chip(ui: &mut Ui, label: &str, text: &str, color: Color32) {
    chip_frame(ui, label, text, color);
}

fn chip_hover(ui: &mut Ui, label: &str, text: &str, color: Color32, hover: &str) {
    chip_frame(ui, label, text, color).on_hover_text(hover);
}

fn chip_frame(ui: &mut Ui, label: &str, text: &str, color: Color32) -> egui::Response {
    let text = &steady(ui, label, text);
    // horizontal_wrapped cannot see a Frame's width before it is drawn, so
    // on a narrow screen (the phone, ~890 points) the last chips ran off
    // the edge. Measure the two labels and break the row ourselves.
    let style = ui.style().clone();
    let w = |t: egui::RichText| {
        egui::WidgetText::from(t)
            .into_galley(
                ui,
                Some(egui::TextWrapMode::Extend),
                f32::INFINITY,
                egui::TextStyle::Body,
            )
            .size()
            .x
    };
    let need = w(egui::RichText::new(label).small())
        + w(egui::RichText::new(text).monospace())
        + style.spacing.item_spacing.x
        + 2.0 * 6.0;
    if need > ui.available_size_before_wrap().x && ui.cursor().min.x > ui.max_rect().min.x + 1.0 {
        ui.end_row();
    }
    egui::Frame::NONE
        .fill(Color32::from_black_alpha(160))
        .corner_radius(4.0)
        .inner_margin(Vec2::new(6.0, 3.0))
        .show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new(label).small().color(Color32::GRAY));
                ui.label(egui::RichText::new(text).monospace().color(color));
            });
        })
        .response
}

/// `text` padded to the longest text this chip has shown: a chip that
/// shrinks when a number loses a digit moves every chip after it, so a chip
/// only ever grows (monospace, so characters are columns).
fn steady(ui: &Ui, label: &str, text: &str) -> String {
    let id = egui::Id::new(("chip-width", label));
    let n = text.chars().count();
    let w = ui.ctx().data_mut(|d| {
        let m = d.get_temp_mut_or(id, 0usize);
        *m = (*m).max(n);
        *m
    });
    format!("{text:<w$}")
}

/// What the pairing-key banner says, if anything (F5): a mismatch is an
/// error (the drone ignores us), the default key a standing warning
/// (anyone holding `mabur-default-00` can command the drone).
pub fn key_warning(s: &Snapshot) -> Option<(String, bool)> {
    if s.key_mismatch {
        Some((
            format!(
                "KEY MISMATCH: the drone answers under another pairing key (ours {})",
                s.key_fp
            ),
            true,
        ))
    } else if s.key_fp == "default" {
        Some((
            "DEFAULT LINK KEY: anyone with mabur-default-00 can command this drone".into(),
            false,
        ))
    } else {
        None
    }
}

/// The key banner across the top of the video, when `key_warning` has one.
pub fn key_banner(ui: &mut Ui, h: &History) {
    let Some((text, bad)) = h.last().and_then(key_warning) else {
        return;
    };
    let (fill, fg) = if bad {
        (BAD, Color32::WHITE)
    } else {
        (
            Color32::from_rgba_unmultiplied(230, 180, 60, 200),
            Color32::BLACK,
        )
    };
    egui::Frame::NONE
        .fill(fill)
        .corner_radius(4.0)
        .inner_margin(Vec2::new(10.0, 5.0))
        .show(ui, |ui| {
            ui.label(egui::RichText::new(text).strong().color(fg));
        });
}

fn pct(v: f64) -> String {
    format!("{:.1} %", 100.0 * v)
}

/// Chars of the "up" chip: its longest text, "clean 1.5/1.5 s  probe r1
/// lossy u 0.26 > 0.05".
const UP_WIDTH: usize = 46;

/// How long the signal chip keeps the last rssi/SNR through null windows.
pub(crate) const SIGNAL_HOLD_S: f64 = 1.5;

/// The top OSD row: link, signal, loss, uplink side.
/// `kbps`: the received video bitrate (runtime::Shared::rx_kbps).
pub fn chips(ui: &mut Ui, h: &History, fps: f64, kbps: f64, decode: (u64, u64)) {
    let Some(s) = h.last() else {
        chip(ui, "core", "waiting for stats", WARN);
        return;
    };
    ui.horizontal_wrapped(|ui| {
        let cal = s.cal.as_ref().and_then(cal::CalView::hud_line);
        let (link, c) = if s.key_mismatch {
            ("KEY MISMATCH".to_string(), BAD)
        } else if let Some(line) = cal {
            // A run owns the air: no video until it ends, by design.
            (line, WARN)
        } else if s.mode == "spotter" {
            (format!("spotter MCS{} {}M", s.mcs, s.bw), OK)
        } else if s.session {
            (format!("rung {} MCS{} {}M", s.rung, s.mcs, s.bw), OK)
        } else {
            ("beaconing".to_string(), WARN)
        };
        chip(ui, "link", &link, c);
        let held = h.signal(SIGNAL_HOLD_S);
        let sig = match held {
            Some((r, n)) => format!("{r:>4.0} dBm  SNR {n:>2.0}"),
            None => "no signal".into(),
        };
        chip(
            ui,
            "signal",
            &sig,
            if held.is_some_and(|(_, n)| n > 15.0) {
                OK
            } else {
                WARN
            },
        );
        if s.cards.len() > 1 {
            chip(ui, "cards", &cards_line(&s.cards, s.tx_card), OK);
        }
        let total = s.aus + s.trunc;
        let pct = if total > 0 {
            100.0 * s.trunc as f64 / total as f64
        } else {
            0.0
        };
        let mut loss = format!("trunc {pct:>4.1} %  salv {:>4}", s.salvaged);
        if let Some(p) = s.pre_fec_loss {
            loss += &format!("  pre {}", self::pct(p));
        }
        if let Some(r) = s.residual {
            loss += &format!("  post {}", self::pct(r));
        }
        chip(
            ui,
            "loss",
            &loss,
            if pct < 2.0 {
                OK
            } else if pct < 10.0 {
                WARN
            } else {
                BAD
            },
        );
        chip(
            ui,
            "video",
            &format!(
                "{fps:>3.0} fps  {:>4.1} Mbit/s  dec err {:>3}",
                kbps / 1000.0,
                decode.1
            ),
            OK,
        );
        if let Some(d) = s.drone_state {
            let mut c = if d == 2 { OK } else { WARN };
            let mut t = drone_state_name(d).to_string();
            if let Some(tc) = s.drone_temp_c {
                t += &format!("  {tc} C");
                if tc >= 85 {
                    c = BAD;
                } else if tc >= 75 {
                    c = c.max_warn();
                }
            }
            chip(ui, "drone", &t, c);
        }
        if s.mode == "gs" {
            let mut up = match s.uplink_pct {
                Some(p) => format!("{} {p:>3} %", s.uplink),
                None => s.uplink.clone(),
            };
            if let Some(r) = s.rtt_ms {
                up += &format!("  rtt {r:>4.0} ms");
            }
            let c = match (s.uplink.as_str(), s.uplink_pct) {
                ("ok", Some(p)) if p >= 80 => OK,
                ("ok", _) | ("wait", _) | ("none", _) => WARN,
                _ => BAD,
            };
            chip(ui, "uplink", &up, c);
            let mut nack = format!("fill {:>5}", s.nack_fill);
            if let Some(r) = s.nack_fill_pps {
                nack += &format!("  {r:>4.0}/s");
            }
            chip(ui, "nack", &nack, OK);
        }
        if let (true, Some(l)) = (s.session, s.ladder.as_ref()) {
            let (txt, blocked) = l.summary();
            let mut hover = format!("rung {}/{}", l.rung, l.top);
            if let Some((r, f, t, age)) = &l.last {
                hover += &format!(
                    "\nlast change: {r} {f} -> {t}, {:.1} s ago",
                    *age as f64 / 1000.0
                );
            }
            // Last in the row and padded: its text length changes every
            // window, and a chip before others would make them all jump.
            chip_hover(
                ui,
                "up",
                &format!("{txt:<UP_WIDTH$}"),
                if blocked { WARN } else { OK },
                &hover,
            );
        }
    });
}

fn plot(ui: &mut Ui, title: &str, pts: &[(f64, f64)], now: f64, color: Color32) {
    let (rect, _) =
        ui.allocate_exact_size(Vec2::new(ui.available_width(), 70.0), egui::Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, 3.0, Color32::from_black_alpha(150));
    let font = FontId::monospace(11.0);
    let last = pts.last().map_or("-".into(), |(_, v)| format!("{v:.1}"));
    p.text(
        rect.left_top() + Vec2::new(4.0, 2.0),
        Align2::LEFT_TOP,
        format!("{title} {last}"),
        font,
        Color32::GRAY,
    );
    if pts.len() < 2 {
        return;
    }
    let (lo, hi) = pts.iter().fold((f64::MAX, f64::MIN), |(a, b), (_, v)| {
        (a.min(*v), b.max(*v))
    });
    let (lo, hi) = if hi - lo < 1e-6 {
        (lo - 1.0, hi + 1.0)
    } else {
        (lo, hi)
    };
    let inner = Rect::from_min_max(
        rect.min + Vec2::new(2.0, 16.0),
        rect.max - Vec2::new(2.0, 2.0),
    );
    let map = |(t, v): (f64, f64)| {
        let x = inner.right() - ((now - t) / History::SPAN_S) as f32 * inner.width();
        let y = inner.bottom() - ((v - lo) / (hi - lo)) as f32 * inner.height();
        Pos2::new(x.max(inner.left()), y)
    };
    let line: Vec<Pos2> = pts.iter().copied().map(map).collect();
    p.add(egui::Shape::line(line, Stroke::new(1.5_f32, color)));
}

/// 60 s monitor plots: signal, loss, rung.
pub fn monitor(ui: &mut Ui, h: &History, now: f64) {
    plot(ui, "SNR dB", &h.series(|s| s.snr_db), now, OK);
    plot(
        ui,
        "RSSI dBm",
        &h.series(|s| s.rssi_dbm),
        now,
        Color32::LIGHT_BLUE,
    );
    plot(ui, "truncated AU/s", &h.rate(|s| s.trunc), now, BAD);
    plot(
        ui,
        "rung",
        &h.series(|s| (s.rung >= 0).then_some(s.rung as f64)),
        now,
        WARN,
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_rust_core_line() {
        let s = Snapshot::parse(r#"{"mode":"gs","session":true,"rung":2,"mcs":3,"bw":40,"drone_state":2,"snr_db":30.5,"rssi_dbm":-40.0,"aus":10,"trunc":1}"#)
            .expect("parse");
        assert!(s.session);
        assert_eq!((s.rung, s.mcs, s.aus, s.trunc), (2, 3, 10, 1));
        let mut h = History::default();
        h.push(0.0, s.clone());
        h.push(1.0, Snapshot { trunc: 4, ..s });
        assert_eq!(h.rate(|s| s.trunc), vec![(1.0, 3.0)]);
    }

    #[test]
    fn parses_new_fields_and_key_warnings() {
        let two = Snapshot::parse(r#"{"cards":[{"frames":10,"video":30,"crc_fail":0,"rssi":-45},{"frames":9,"video":10,"crc_fail":1,"rssi":null}]}"#)
            .unwrap();
        assert_eq!(
            cards_line(&two.cards, None),
            "c0 -45 dBm 75 %  c1   - dBm 25 %"
        );
        assert_eq!(
            cards_line(&two.cards, Some(1)),
            "c0 -45 dBm 75 %  c1 TX   - dBm 25 %"
        );
        let s = Snapshot::parse(r#"{"mode":"gs","key_fp":"default","key_mismatch":false,"rtt_ms":6.1,"uplink":"ok","uplink_pct":92,"pre_fec_loss":0.085,"residual":0.0106,"drone_temp_c":46,"nack":{"fill":48,"fill_pps":0.0}}"#)
            .expect("parse");
        assert_eq!(
            (s.uplink.as_str(), s.uplink_pct, s.drone_temp_c),
            ("ok", Some(92), Some(46))
        );
        assert_eq!((s.nack_fill, s.rtt_ms), (48, Some(6.1)));
        let (t, bad) = key_warning(&s).expect("default key warns");
        assert!(t.starts_with("DEFAULT LINK KEY") && !bad);
        let m = Snapshot {
            key_mismatch: true,
            key_fp: "1a2b".into(),
            ..s.clone()
        };
        let (t, bad) = key_warning(&m).expect("mismatch");
        assert!(t.contains("ours 1a2b") && bad);
        let own = Snapshot {
            key_fp: "1a2b".into(),
            ..s
        };
        assert!(key_warning(&own).is_none());
    }

    #[test]
    fn signal_holds_through_null_windows() {
        let mut h = History::default();
        let row = |r: &str| Snapshot::parse(&format!(r#"{{"mode":"gs"{r}}}"#)).expect("row");
        h.push(0.0, row(r#","rssi_dbm":-40.0,"snr_db":30.0"#));
        h.push(0.1, row(""));
        h.push(1.0, row(""));
        assert_eq!(h.signal(SIGNAL_HOLD_S), Some((-40.0, 30.0)));
        h.push(1.6, row(""));
        assert_eq!(h.signal(SIGNAL_HOLD_S), None);
    }

    #[test]
    fn ladder_summary_names_the_blocking_stage() {
        let row = |l: &str| {
            Snapshot::parse(&format!(r#"{{"mode":"gs","ladder":{l}}}"#))
                .and_then(|s| s.ladder)
                .expect("ladder")
        };
        let base = r#""rung":0,"top":4,"u":0.0,"up_util":0.15,"clean_ms":2000,"clean_need_ms":1500,"probe_rung":1,"probe_u":0.0,"probe_max_util":0.05,"streak_need":90,"penalty_ms":0"#;
        let l = row(&format!(r#"{{{base},"probe":"clean","streak":48}}"#));
        assert_eq!(
            l.summary(),
            ("clean 1.5/1.5 s  probe r1 48/90".into(), true)
        );
        let l = row(&format!(r#"{{{base},"probe":"lossy","streak":0}}"#));
        assert!(l.summary().0.contains("lossy") && l.summary().1);
        assert!(l.summary().0.len() <= UP_WIDTH);
        let l = row(&format!(r#"{{{base},"probe":"clean","streak":95}}"#));
        assert!(!l.summary().1);
        let l = row(
            r#"{"rung":4,"top":4,"last_reason":"promote_probed","last_from":3,"last_to":4,"last_age_ms":900}"#,
        );
        assert_eq!(l.summary(), ("top rung".into(), false));
        assert_eq!(l.last.as_ref().map(|x| x.2), Some(4));
    }
}
