//! The egui part of the crate: colours, history, chips, monitor.
use super::*;
use egui::{Align2, Color32, FontId, Pos2, Rect, Stroke, Ui, Vec2};

use crate::json::{bool_or, opt_f64, opt_i64, str_or, u64_or};

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
    pub aus: u64,
    pub trunc: u64,
    pub salvaged: u64,
    /// Keyframe request epochs sent so far (first tries and retries).
    pub idr_req: u64,
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
    /// frames the card has delivered.
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
    /// The operator's channel scan (UI stats line only).
    pub chscan: Option<channels::ScanView>,
    /// The write error that stopped the flight log (UI stats line only).
    pub log_err: Option<String>,
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
        let i = |k: &str| opt_i64(v, k);
        let f = |k: &str| opt_f64(v, k).unwrap_or(0.0);
        let up = v.get("probe").and_then(Value::as_str).map(|p| LadderUp {
            u: f("u"),
            up_util: f("up_util"),
            clean_ms: i("clean_ms").unwrap_or(0),
            clean_need_ms: i("clean_need_ms").unwrap_or(0),
            probe: p.to_string(),
            probe_rung: i("probe_rung").unwrap_or(-1),
            probe_u: f("probe_u"),
            probe_max_util: f("probe_max_util"),
            streak: u64_or(v, "streak", 0),
            streak_need: u64_or(v, "streak_need", 0),
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

impl Snapshot {
    pub fn parse(line: &str) -> Option<Self> {
        let v: Value = serde_json::from_str(line).ok()?;
        Some(Self {
            mode: str_or(&v, "mode", "?"),
            session: bool_or(&v, "session", false),
            key_mismatch: bool_or(&v, "key_mismatch", false),
            rung: opt_i64(&v, "rung").unwrap_or(-1),
            mcs: opt_i64(&v, "mcs").unwrap_or(-1),
            bw: opt_i64(&v, "bw").unwrap_or(0),
            drone_state: opt_i64(&v, "drone_state"),
            rssi_dbm: opt_f64(&v, "rssi_dbm"),
            snr_db: opt_f64(&v, "snr_db"),
            aus: u64_or(&v, "aus", 0),
            trunc: u64_or(&v, "trunc", 0),
            salvaged: u64_or(&v, "salvaged", 0),
            idr_req: u64_or(&v, "idr_req", 0),
            key_fp: str_or(&v, "key_fp", ""),
            rtt_ms: opt_f64(&v, "rtt_ms"),
            uplink: str_or(&v, "uplink", "none"),
            uplink_pct: opt_i64(&v, "uplink_pct"),
            pre_fec_loss: opt_f64(&v, "pre_fec_loss"),
            residual: opt_f64(&v, "residual"),
            cards: cards_of(&v),
            tx_card: v.get("tx_card").and_then(Value::as_u64),
            drone_temp_c: opt_i64(&v, "drone_temp_c"),
            nack_fill: v.get("nack").map_or(0, |n| u64_or(n, "fill", 0)),
            nack_fill_pps: v.get("nack").and_then(|n| opt_f64(n, "fill_pps")),
            ladder: v.get("ladder").and_then(Ladder::parse),
            cal: v.get("cal").and_then(cal::CalView::parse),
            chscan: v.get("chscan").and_then(channels::ScanView::parse),
            log_err: v.get("log_err").and_then(Value::as_str).map(str::to_string),
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

/// The UI stats lines (10 Hz from the core loop) of the last 60 s, for the
/// monitor plots.
#[derive(Default)]
pub struct History {
    rows: VecDeque<(f64, Snapshot)>,
}

impl History {
    pub const SPAN_S: f64 = 60.0;
    pub fn push(&mut self, t_s: f64, s: Snapshot) {
        // Only the newest row's views are read; about 600 rows are kept.
        if let Some((_, prev)) = self.rows.back_mut() {
            prev.ladder = None;
            prev.cal = None;
            prev.chscan = None;
        }
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
    /// Per-card rates over the last `CARD_WINDOW_S` (empty with fewer
    /// than two cards or one snapshot).
    pub fn card_rates(&self) -> Vec<CardRate> {
        let Some((t1, new)) = self.rows.back() else {
            return Vec::new();
        };
        if new.cards.len() < 2 {
            return Vec::new();
        }
        let start = self.rows.partition_point(|(t, _)| t1 - t > CARD_WINDOW_S);
        let Some((t0, old)) = self.rows.get(start) else {
            return Vec::new();
        };
        card_rates(&old.cards, &new.cards, t1 - t0)
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
    /// How much counter `f` grew over the last `window_s` (0 with one row).
    pub fn grew(&self, window_s: f64, f: impl Fn(&Snapshot) -> u64) -> u64 {
        let Some((t1, new)) = self.rows.back() else {
            return 0;
        };
        let start = self.rows.partition_point(|(t, _)| t1 - t > window_s);
        self.rows
            .get(start)
            .map_or(0, |(_, old)| f(new).saturating_sub(f(old)))
    }
    fn series(&self, f: impl Fn(&Snapshot) -> Option<f64>) -> Vec<(f64, f64)> {
        self.rows
            .iter()
            .filter_map(|(t, s)| f(s).map(|v| (*t, v)))
            .collect()
    }
}

/// One stats chip: what it says, before placement.
struct Chip {
    label: &'static str,
    /// Columns of its text, the longest text it shows in flight, so a chip
    /// never changes size when a number gains or loses a digit and the
    /// operator can park it next to another without the two colliding
    /// later. A calibration line or a many-card row can still be longer and
    /// widens its chip while it lasts.
    cols: usize,
    text: String,
    color: Color32,
    hover: Option<String>,
}

/// One card's entry in the cards chip: "c0 TX -100 dBm 999/s crc 99 %".
const CARD_COLS: usize = 29;

/// Size of a chip whose text is `cols` monospace columns.
fn chip_size(ui: &Ui, label: &str, cols: usize) -> Vec2 {
    let galley = |t: egui::RichText| {
        egui::WidgetText::from(t)
            .into_galley(
                ui,
                Some(egui::TextWrapMode::Extend),
                f32::INFINITY,
                egui::TextStyle::Body,
            )
            .size()
    };
    let l = galley(egui::RichText::new(label).small());
    let t = galley(egui::RichText::new(" ".repeat(cols.max(1))).monospace());
    Vec2::new(
        l.x + t.x + ui.spacing().item_spacing.x + 2.0 * 6.0,
        l.y.max(t.y) + 2.0 * 3.0,
    )
}

fn draw_chip(ui: &mut Ui, c: &Chip, cols: usize) -> egui::Response {
    egui::Frame::NONE
        .fill(Color32::from_black_alpha(160))
        .corner_radius(4.0)
        .inner_margin(Vec2::new(6.0, 3.0))
        .show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new(c.label).small().color(Color32::GRAY));
                ui.label(
                    egui::RichText::new(format!("{:<cols$}", c.text))
                        .monospace()
                        .color(c.color),
                );
            });
        })
        .response
}

/// Lays the chips out and draws each in its own Area: an unmoved chip
/// takes the next place in rows that wrap at the right edge of `region`, a
/// moved one sits at its `pos` offset from `region`'s top-left, which may
/// be outside `region` (anywhere on the egui screen). Dragging a chip moves
/// it, a double click puts it back in the row. Returns the height of the
/// rows and whether `pos` changed (the caller saves the prefs).
fn place_chips(
    ui: &Ui,
    region: Rect,
    chips: &[Chip],
    pos: &mut std::collections::BTreeMap<String, [f32; 2]>,
) -> (f32, bool) {
    let gap = ui.spacing().item_spacing;
    let (mut x, mut y, mut row_h) = (region.min.x, region.min.y, 0.0f32);
    let mut changed = false;
    for c in chips {
        let cols = c.cols.max(c.text.chars().count());
        let size = chip_size(ui, c.label, cols);
        let at = if let Some(&[dx, dy]) = pos.get(c.label) {
            region.min + Vec2::new(dx, dy)
        } else {
            if x + size.x > region.max.x && x > region.min.x + 1.0 {
                x = region.min.x;
                y += row_h + gap.y;
                row_h = 0.0;
            }
            let p = Pos2::new(x, y);
            x += size.x + gap.x;
            row_h = row_h.max(size.y);
            p
        };
        let id = egui::Id::new(("pal-chip", c.label));
        egui::Area::new(id)
            .order(egui::Order::Middle)
            .fixed_pos(at)
            .show(ui.ctx(), |ui| {
                let r = draw_chip(ui, c, cols);
                let r = ui.interact(r.rect, id.with("drag"), egui::Sense::click_and_drag());
                if r.dragged() {
                    let d = r.drag_delta();
                    let o = at - region.min + d;
                    pos.insert(c.label.to_string(), [o.x, o.y]);
                }
                if r.drag_stopped() {
                    changed = true;
                }
                if r.double_clicked() && pos.remove(c.label).is_some() {
                    changed = true;
                }
                if let Some(h) = &c.hover {
                    r.on_hover_text(h.as_str());
                }
            });
    }
    let h = if row_h > 0.0 {
        y + row_h - region.min.y
    } else {
        0.0
    };
    (h, changed)
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

/// The keyframes chip: request epochs counted over this window, and the
/// count from which it shows.
const KEYFRAME_WINDOW_S: f64 = 10.0;
const KEYFRAME_STORM: u64 = 5;

/// How long the signal chip keeps the last rssi/SNR through null windows.
pub(crate) const SIGNAL_HOLD_S: f64 = 1.5;

/// The stats chips: link, signal, loss, uplink side, laid out from the top
/// of `ui`'s rect (see `place_chips`; `pos` is `HudPrefs::chip_pos`). The
/// rows' height is allocated in `ui`, so what the caller adds next lands
/// below them. Returns true when the operator moved a chip: save the prefs.
/// `kbps`: the received video bitrate (runtime::Shared::rx_kbps).
pub fn chips(
    ui: &mut Ui,
    pos: &mut std::collections::BTreeMap<String, [f32; 2]>,
    h: &History,
    fps: f64,
    kbps: f64,
    dec_errors: u64,
) -> bool {
    let mut out: Vec<Chip> = Vec::new();
    let mut chip =
        |label: &'static str, cols: usize, text: &str, color: Color32, hover: Option<String>| {
            out.push(Chip {
                label,
                cols,
                text: text.to_string(),
                color,
                hover,
            })
        };
    if let Some(s) = h.last() {
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
        // "rung 12 MCS7 40M", "spotter MCS7 40M"
        chip("link", 16, &link, c, None);
        let held = h.signal(SIGNAL_HOLD_S);
        let sig = match held {
            Some((r, n)) => format!("{r:>4.0} dBm  SNR {n:>2.0}"),
            None => "no signal".into(),
        };
        chip(
            "signal",
            16, // "-100 dBm  SNR 30"
            &sig,
            if held.is_some_and(|(_, n)| n > 15.0) {
                OK
            } else {
                WARN
            },
            None,
        );
        let rates = h.card_rates();
        if !rates.is_empty() {
            let deaf = rates.iter().any(|c| c.deaf);
            chip(
                "cards",
                rates.len() * CARD_COLS + 2 * rates.len().saturating_sub(1),
                &cards_line(&rates, s.tx_card),
                if deaf { BAD } else { OK },
                None,
            );
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
            "loss",
            49, // "trunc 100.0 %  salv 9999  pre 100.0 %  post 100.0 %"
            &loss,
            if pct < 2.0 {
                OK
            } else if pct < 10.0 {
                WARN
            } else {
                BAD
            },
            None,
        );
        chip(
            "video",
            33, // "120 fps  99.9 Mbit/s  dec err 999"
            &format!(
                "{fps:>3.0} fps  {:>4.1} Mbit/s  dec err {:>3}",
                kbps / 1000.0,
                dec_errors
            ),
            OK,
            None,
        );
        // A keyframe storm: one request with its retries sends up to about
        // 10 in 10 s at the longest retry gap (1 s), so 5 or more is either
        // several losses close together or a request that does not heal.
        let asks = h.grew(KEYFRAME_WINDOW_S, |s| s.idr_req);
        if asks >= KEYFRAME_STORM {
            chip(
                "keyframes",
                21, // "keyframes 99 in 10 s"
                &format!("keyframes {asks:>2} in 10 s"),
                if asks >= 2 * KEYFRAME_STORM {
                    BAD
                } else {
                    WARN
                },
                None,
            );
        }
        if let Some(d) = s.drone_state {
            let mut c = if d == 2 { OK } else { WARN };
            let mut t = drone_state_name(d).to_string();
            if let Some(tc) = s.drone_temp_c {
                t += &format!("  {tc} C");
                if tc >= 85 {
                    c = BAD;
                } else if tc >= 75 {
                    c = WARN;
                }
            }
            // "rendezvous  100 C"
            chip("drone", 17, &t, c, None);
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
            // "wait 100 %  rtt 1000 ms"
            chip("uplink", 23, &up, c, None);
            let mut nack = format!("fill {:>5}", s.nack_fill);
            if let Some(r) = s.nack_fill_pps {
                nack += &format!("  {r:>4.0}/s");
            }
            // "fill 99999  9999/s"
            chip("nack", 18, &nack, OK, None);
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
            chip(
                "up",
                UP_WIDTH,
                &txt,
                if blocked { WARN } else { OK },
                Some(hover),
            );
        }
        if let Some(e) = &s.log_err {
            // Last in the row: it appears mid-flight and must not push the
            // other chips around.
            chip("log", 0, &format!("STOPPED: {e}"), BAD, None);
        }
    } else {
        chip("core", 0, "waiting for stats", WARN, None);
    }
    let region = ui.max_rect();
    let (height, moved) = place_chips(ui, region, &out, pos);
    ui.allocate_space(Vec2::new(region.width(), height));
    moved
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

    /// One egui frame of the chips over a 1280x720 screen, `region` the
    /// chips' rect; returns whether `chips` reported a move.
    fn chip_frame(
        ctx: &egui::Context,
        region: Rect,
        pos: &mut std::collections::BTreeMap<String, [f32; 2]>,
        h: &History,
        events: Vec<egui::Event>,
    ) -> bool {
        let input = egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1280.0, 720.0))),
            events,
            ..Default::default()
        };
        let mut moved = false;
        let _ = ctx.run(input, |ctx| {
            egui::CentralPanel::default()
                .frame(egui::Frame::NONE)
                .show(ctx, |ui| {
                    ui.allocate_new_ui(egui::UiBuilder::new().max_rect(region), |ui| {
                        moved = chips(ui, pos, h, 120.0, 9000.0, 0);
                    });
                });
        });
        moved
    }

    fn chip_rect(ctx: &egui::Context, label: &str) -> Rect {
        ctx.memory(|m| m.area_rect(egui::Id::new(("pal-chip", label))))
            .expect("chip drawn")
    }

    #[test]
    fn the_keyframes_chip_shows_from_five_requests_in_ten_seconds() {
        let ctx = egui::Context::default();
        let region = Rect::from_min_size(Pos2::new(100.0, 100.0), Vec2::new(600.0, 400.0));
        let mut pos = Default::default();
        let drawn = |ctx: &egui::Context| {
            ctx.memory(|m| m.area_rect(egui::Id::new(("pal-chip", "keyframes"))))
                .is_some()
        };
        let snap = |idr_req| Snapshot {
            mode: "gs".into(),
            session: true,
            idr_req,
            ..Default::default()
        };
        let mut h = History::default();
        // Three in the last 10 s: no chip.
        for (t, n) in [(0.0, 10), (5.0, 12), (10.0, 14), (15.0, 15)] {
            h.push(t, snap(n));
        }
        assert_eq!(h.grew(10.0, |s| s.idr_req), 3);
        chip_frame(&ctx, region, &mut pos, &h, vec![]);
        assert!(!drawn(&ctx));
        // The window now starts at the 10 s row (14): five, the threshold.
        h.push(16.0, snap(19));
        assert_eq!(h.grew(10.0, |s| s.idr_req), 5);
        chip_frame(&ctx, region, &mut pos, &h, vec![]);
        assert!(drawn(&ctx));
    }

    #[test]
    fn chips_keep_their_size_and_drag_anywhere() {
        let ctx = egui::Context::default();
        let region = Rect::from_min_size(Pos2::new(100.0, 100.0), Vec2::new(600.0, 400.0));
        let mut pos = Default::default();
        let mut h = History::default();
        let snap = |trunc, salv| Snapshot {
            mode: "gs".into(),
            session: true,
            aus: 1000,
            trunc,
            salvaged: salv,
            uplink: "ok".into(),
            ..Default::default()
        };
        h.push(0.0, snap(0, 0));
        chip_frame(&ctx, region, &mut pos, &h, vec![]);
        chip_frame(&ctx, region, &mut pos, &h, vec![]);
        let loss = chip_rect(&ctx, "loss");
        let video = chip_rect(&ctx, "video");
        // More digits and the pre/post fields do not resize the chip.
        h.push(
            1.0,
            Snapshot {
                pre_fec_loss: Some(0.123),
                residual: Some(0.01),
                ..snap(150, 1234)
            },
        );
        chip_frame(&ctx, region, &mut pos, &h, vec![]);
        assert_eq!(chip_rect(&ctx, "loss"), loss);
        // The rows wrap inside the region.
        assert!(loss.max.x <= region.max.x && video.min.x >= region.min.x);
        assert!(video.min.y > loss.min.y || video.min.x > loss.max.x);

        // Drag the video chip to the screen's left of the region.
        let p0 = video.center();
        let mv = |p: Pos2| egui::Event::PointerMoved(p);
        let btn = |p: Pos2, pressed| egui::Event::PointerButton {
            pos: p,
            button: egui::PointerButton::Primary,
            pressed,
            modifiers: Default::default(),
        };
        chip_frame(&ctx, region, &mut pos, &h, vec![mv(p0)]);
        chip_frame(&ctx, region, &mut pos, &h, vec![btn(p0, true)]);
        let mut moved = false;
        for i in 1..=10 {
            let p = p0 + Vec2::new(-15.0 * i as f32, 20.0 * i as f32);
            moved |= chip_frame(&ctx, region, &mut pos, &h, vec![mv(p)]);
        }
        let end = p0 + Vec2::new(-150.0, 200.0);
        moved |= chip_frame(&ctx, region, &mut pos, &h, vec![btn(end, false)]);
        assert!(moved, "a drag reports a move");
        let [dx, dy] = pos["video"];
        let want = video.min - region.min + Vec2::new(-150.0, 200.0);
        assert!(
            (dx - want.x).abs() < 0.5 && (dy - want.y).abs() < 0.5,
            "{dx},{dy} vs {want:?}"
        );
        chip_frame(&ctx, region, &mut pos, &h, vec![]);
        let r = chip_rect(&ctx, "video");
        assert!(r.min.x < region.min.x, "outside the region: {r:?}");
        assert_eq!(r.size(), video.size());

        // A double click puts it back in the row.
        let c = r.center();
        let mut back = false;
        for _ in 0..2 {
            back |= chip_frame(&ctx, region, &mut pos, &h, vec![mv(c), btn(c, true)]);
            back |= chip_frame(&ctx, region, &mut pos, &h, vec![btn(c, false)]);
        }
        assert!(back && pos.is_empty());
    }

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
        assert_eq!((two.cards[0].frames, two.cards[1].crc_fail), (10, 1));
        assert_eq!(two.log_err, None);
        let stopped =
            Snapshot::parse(r#"{"log_err":"No space left on device (os error 28)"}"#).unwrap();
        assert_eq!(
            stopped.log_err.as_deref(),
            Some("No space left on device (os error 28)")
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

    fn card(rssi: Option<i64>, frames: u64, crc_fail: u64) -> CardRx {
        CardRx {
            rssi,
            frames,
            crc_fail,
            ..CardRx::default()
        }
    }

    #[test]
    fn deaf_watch_reports_each_flip_once() {
        let mut w = DeafWatch::default();
        // 1 s apart; card 1 hears 230/s throughout, card 0 stops at t=3.
        let at = |t: u64, c0: u64| [card(None, c0, 0), card(None, 230 * t, 0)];
        assert!(w.push(0.0, &at(0, 0)).is_empty());
        for t in 1..=3 {
            assert!(w.push(t as f64, &at(t, 230 * t)).is_empty(), "t={t}");
        }
        let mut flips = Vec::new();
        for t in 4..=9 {
            flips.extend(w.push(t as f64, &at(t, 690)));
        }
        assert_eq!(flips.len(), 1);
        assert_eq!((flips[0].card, flips[0].deaf), (0, true));
        assert_eq!(flips[0].fps, 0.0);
        // It hears again: one flip back.
        let back: Vec<_> = (10..=13)
            .flat_map(|t| w.push(t as f64, &at(t, 690 + 230 * (t - 9))))
            .collect();
        assert_eq!(back.len(), 1);
        assert_eq!((back[0].card, back[0].deaf), (0, false));
        // A card set change starts over without a flip.
        assert!(w.push(14.0, &[card(None, 0, 0)]).is_empty());
    }

    #[test]
    fn a_card_that_stopped_hearing_reads_deaf() {
        // The 2026-11-02 Quest session: card 0 at ~0.1 frames/s, card 1 at
        // ~230 with 7 % CRC failures.
        let old = [card(None, 97, 0), card(Some(-50), 19_086, 1_000)];
        let new = [card(None, 97, 0), card(Some(-48), 19_776, 1_048)];
        let r = card_rates(&old, &new, 3.0);
        assert!(r[0].deaf && !r[1].deaf);
        assert_eq!(r[0].crc_pct, None);
        assert_eq!(r[1].fps, 230.0);
        assert_eq!(
            cards_line(&r, Some(1)),
            "c0 DEAF   0/s  c1 TX -48 dBm 230/s crc  7 %"
        );
    }

    #[test]
    fn quiet_air_or_a_changed_card_set_names_no_card_deaf() {
        // No drone: both cards hear only the neighbours.
        let r = card_rates(
            &[card(None, 0, 0); 2],
            &[card(None, 30, 3), card(None, 0, 0)],
            1.0,
        );
        assert!(r.iter().all(|c| !c.deaf));
        // A second card was just added.
        assert!(card_rates(&[card(None, 0, 0)], &[card(None, 9, 0); 2], 1.0).is_empty());
        // Both cards on the link, one weaker: not deaf.
        let r = card_rates(
            &[card(None, 0, 0); 2],
            &[card(Some(-40), 230, 9), card(Some(-60), 60, 20)],
            1.0,
        );
        assert!(r.iter().all(|c| !c.deaf));
        assert_eq!(
            cards_line(&r, None),
            "c0 -40 dBm 230/s crc  4 %  c1 -60 dBm  60/s crc 33 %"
        );
    }

    #[test]
    fn history_rates_span_the_card_window() {
        let snap = |f0, f1| Snapshot {
            cards: vec![card(Some(-40), f0, 0), card(Some(-41), f1, 0)],
            ..Snapshot::default()
        };
        let mut h = History::default();
        assert!(h.card_rates().is_empty());
        h.push(0.0, snap(0, 0));
        h.push(1.0, snap(100, 100));
        h.push(2.0, snap(300, 300));
        h.push(4.0, snap(700, 300));
        // Window 3 s: from t=1 to t=4.
        let r = h.card_rates();
        assert_eq!((r[0].fps, r[1].fps), (200.0, 200.0 / 3.0));
        assert!(!r[1].deaf);
        h.push(6.0, snap(1100, 300));
        let r = h.card_rates();
        assert!(r[1].deaf, "card 1 heard nothing for 4 s: {r:?}");
    }
}
