//! mabur `hop_verdict.{h,cpp}`: one verdict per hop window from the link's
//! loss and every card's NHM airtime. Impaired + weak = fade; two routes to
//! interfered, both reading foreign NHM airtime: sustained occupancy over
//! `blocked_pct` (`occupied`), and a lower floor (`detect_pct`) confirmed
//! by loss (`detected`). The recovered reference freezes on the first
//! impaired window and thaws after 3 healthy ones.

use std::collections::VecDeque;

use super::hop::HOP_SETTLE_BLANK_MS;
use super::radio::SNR_RAW_TO_DB;
use crate::config::{BusyCfg, HopCfg};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Verdict {
    #[default]
    Healthy,
    Fade,
    Interfered,
    Unknown,
}

impl Verdict {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Healthy => "healthy",
            Self::Fade => "fade",
            Self::Interfered => "interfered",
            Self::Unknown => "unknown",
        }
    }
}

// 0x08/0x10/0x40 are unassigned (retired contended/raised/starved); 0x04,
// once the fading bit, is `detected`.
pub const EV_IMPAIRED: u8 = 1;
pub const EV_WEAK: u8 = 2;
pub const EV_DETECTED: u8 = 4;
pub const EV_BLOCKED: u8 = 32;
pub const EV_OCCUPIED: u8 = 128;

/// This window's evidence is a hop-route reading: `blocked` (the occupied
/// route's per-window reading) or `detected` (the detect route).
pub fn trigger_evidence(ev: u8) -> bool {
    ev & (EV_BLOCKED | EV_DETECTED) != 0
}

pub fn rssi_raw_to_dbm(raw: f64) -> f64 {
    if raw > 0.0 {
        raw - 110.0
    } else {
        0.0
    }
}
pub fn snr_raw_to_db(raw: f64) -> f64 {
    raw * SNR_RAW_TO_DB
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VerdictCardIn {
    pub valid: bool,
    pub foreign: u32,
    pub crc_fail: u32,
    pub fa: u32,
    pub cca: u32,
    pub rssi_dbm: f64,
    pub snr_db: f64,
    pub snr_valid: bool,
    pub busy_valid: bool,
    pub nhm_busy_pct: f64,
    pub own_air_pct: f64,
}

impl Default for VerdictCardIn {
    fn default() -> Self {
        Self {
            valid: false,
            foreign: 0,
            crc_fail: 0,
            fa: 0,
            cca: 0,
            rssi_dbm: 0.0,
            snr_db: 0.0,
            snr_valid: true,
            busy_valid: false,
            nhm_busy_pct: 0.0,
            own_air_pct: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct VerdictLinkIn {
    /// 0..1 over the window.
    pub pre_fec_loss: f64,
    pub recovered: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VerdictOut {
    pub v: Verdict,
    pub evidence: u8,
    /// -1 while healthy.
    pub ref_rung: i32,
    /// Interfered in >= persist of the last 3 windows.
    pub trigger: bool,
    pub t_start_ms: f64,
    pub t_ms: f64,
    pub ref_frozen: bool,
    /// The first impaired, non-weak window of a frozen episode that is the
    /// lead-in of either route to `interfered` (the rung-store blank's
    /// arming edge, `hop_blank.h`).
    pub first_blocked: bool,
}

impl Default for VerdictOut {
    fn default() -> Self {
        Self {
            v: Verdict::Healthy,
            evidence: 0,
            ref_rung: -1,
            trigger: false,
            t_start_ms: 0.0,
            t_ms: 0.0,
            ref_frozen: false,
            first_blocked: false,
        }
    }
}

/// One detect-ring slot per window, reading-less windows included.
#[derive(Clone, Copy, Debug)]
struct DetectSlot {
    reading: bool,
    /// Minimum-across-cards foreign busy %.
    foreign: f64,
    /// `loss_impaired`, never on a window starting inside the landing settle.
    lossy: bool,
}

#[derive(Clone, Debug)]
pub struct HopVerdict {
    cfg: HopCfg,
    busy: BusyCfg,
    rec_hist: VecDeque<f64>,
    frozen: bool,
    prev_ms: f64,
    have_prev: bool,
    seen_blocked: bool,
    ref_rec: f64,
    ref_rung: i32,
    healthy_streak: i32,
    /// Consecutive blocked windows with a busy reading; a reading-less
    /// window holds it. Survives a hop's landing on purpose.
    blocked_streak: i32,
    detect_ring: VecDeque<DetectSlot>,
    have_landed: bool,
    landed_ms: f64,
    recent_interfered: VecDeque<bool>,
}

fn mean(v: &VecDeque<f64>) -> f64 {
    if v.is_empty() {
        0.0
    } else {
        v.iter().sum::<f64>() / v.len() as f64
    }
}

impl HopVerdict {
    pub fn new(cfg: HopCfg, busy: BusyCfg) -> Self {
        Self {
            cfg,
            busy,
            rec_hist: VecDeque::new(),
            frozen: false,
            prev_ms: 0.0,
            have_prev: false,
            seen_blocked: false,
            ref_rec: 0.0,
            ref_rung: -1,
            healthy_streak: 0,
            blocked_streak: 0,
            detect_ring: VecDeque::new(),
            have_landed: false,
            landed_ms: 0.0,
            recent_interfered: VecDeque::new(),
        }
    }

    /// The per-hop thaw (VerifyPass). rec_hist survives: it is the trailing
    /// baseline, not per-hop state.
    pub fn reset(&mut self) {
        self.frozen = false;
        self.ref_rec = 0.0;
        self.ref_rung = -1;
        self.healthy_streak = 0;
        self.seen_blocked = false;
        self.recent_interfered.clear();
        self.blocked_streak = 0;
        self.detect_ring.clear();
        self.have_landed = false;
    }

    /// A new in-session run: the blocked streak and the detect ring are
    /// per-run state.
    pub fn new_session(&mut self) {
        self.blocked_streak = 0;
        self.detect_ring.clear();
        self.have_landed = false;
    }

    /// A hop landed (Confirm) at `now_ms`: the ring held the old channel's
    /// readings. The blocked streak carries, so an occupied target fails
    /// verify on its first blocked reading.
    pub fn landed(&mut self, now_ms: f64) {
        self.detect_ring.clear();
        self.landed_ms = now_ms;
        self.have_landed = true;
    }

    pub fn ref_rung(&self) -> i32 {
        self.ref_rung
    }

    /// A live ladder swap inside a frozen episode: the caller translates
    /// the frozen rung to the new ladder. No-op while healthy.
    pub fn remap_ref_rung(&mut self, rung: i32) {
        if self.ref_rung >= 0 {
            self.ref_rung = rung;
        }
    }

    /// The ring holds a detection: a majority of reading slots, their
    /// median foreign % >= detect_pct, and at least `lossy_needed` lossy
    /// slots. With an even count the median is the mean of the two middle
    /// readings.
    fn ring_reads_detect(&self, lossy_needed: i32) -> bool {
        let n = self.cfg.verdict.detect_windows;
        let mut fg: Vec<f64> = Vec::new();
        let mut lossy = 0;
        for s in &self.detect_ring {
            if s.reading {
                fg.push(s.foreign);
            }
            if s.lossy {
                lossy += 1;
            }
        }
        if lossy < lossy_needed || (fg.len() as i32) < (n + 1) / 2 {
            return false;
        }
        fg.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let mid = fg.len() / 2;
        let med = if fg.len().is_multiple_of(2) {
            (fg[mid] + fg[mid - 1]) / 2.0
        } else {
            fg[mid]
        };
        med >= self.cfg.verdict.detect_pct
    }

    /// The thaw hold: a detection in the making (one lossy slot is enough).
    fn ring_holds_detection(&self) -> bool {
        self.ring_reads_detect(1)
    }

    pub fn window(
        &mut self,
        now_ms: f64,
        cards: &[VerdictCardIn],
        link: &VerdictLinkIn,
        rung: i32,
    ) -> VerdictOut {
        let hist_n = (5000 / self.cfg.window_ms).max(1) as usize;
        let mut best: Option<usize> = None;
        for (i, c) in cards.iter().enumerate() {
            if c.valid && best.is_none_or(|b| c.rssi_dbm > cards[b].rssi_dbm) {
                best = Some(i);
            }
        }
        let mut o = VerdictOut {
            t_start_ms: if self.have_prev { self.prev_ms } else { now_ms },
            t_ms: now_ms,
            ..Default::default()
        };
        self.prev_ms = now_ms;
        self.have_prev = true;
        let Some(best) = best else {
            o.v = Verdict::Unknown;
            o.ref_frozen = self.frozen;
            return o;
        };
        let hv = self.cfg.verdict.clone();
        // blocked = the MINIMUM foreign reading across cards with one: a
        // receive-weak diversity card reads our own faint frames as foreign.
        let mut min_foreign: Option<f64> = None;
        for c in cards.iter().filter(|c| c.valid && c.busy_valid) {
            let foreign_pct = (c.nhm_busy_pct - c.own_air_pct).max(0.0);
            min_foreign = Some(min_foreign.map_or(foreign_pct, |m| m.min(foreign_pct)));
        }
        let have_busy_reading = min_foreign.is_some();
        let min_foreign_busy_pct = min_foreign.unwrap_or(0.0);
        let blocked = have_busy_reading && min_foreign_busy_pct >= self.busy.blocked_pct;
        if have_busy_reading {
            self.blocked_streak = if blocked { self.blocked_streak + 1 } else { 0 };
        }
        let occupied = self.blocked_streak >= hv.occupied_windows;
        let rec_ref = if self.frozen {
            self.ref_rec
        } else {
            mean(&self.rec_hist)
        };
        let loss_impaired = link.pre_fec_loss * 100.0 > hv.loss_pct
            || (rec_ref > 0.0
                && link.recovered as f64 > hv.recovered_x * rec_ref
                && link.recovered >= hv.recovered_min.max(0) as u32);
        // A window starting inside the landing settle keeps its reading but
        // is never the ring's loss window: the retune gap's repairs.
        let settle_debris = self.have_landed && o.t_start_ms < self.landed_ms + HOP_SETTLE_BLANK_MS;
        self.detect_ring.push_back(DetectSlot {
            reading: have_busy_reading,
            foreign: min_foreign_busy_pct,
            lossy: loss_impaired && !settle_debris,
        });
        while self.detect_ring.len() > hv.detect_windows.max(1) as usize {
            self.detect_ring.pop_front();
        }
        let detected = self.detect_ring.len() == hv.detect_windows as usize
            && self.ring_reads_detect(hv.detect_impaired);
        let impaired = loss_impaired || occupied || detected;
        let b = &cards[best];
        let weak = b.rssi_dbm < hv.weak_rssi_dbm as f64
            && (!b.snr_valid || b.snr_db < hv.weak_snr_db as f64);
        let flag = |on: bool, bit: u8| if on { bit } else { 0 };
        o.evidence = flag(impaired, EV_IMPAIRED)
            | flag(weak, EV_WEAK)
            | flag(detected, EV_DETECTED)
            | flag(blocked, EV_BLOCKED)
            | flag(occupied, EV_OCCUPIED);
        // weak gates first; each route needs this window's own reading.
        o.v = if !impaired {
            Verdict::Healthy
        } else if weak {
            Verdict::Fade
        } else if (occupied && blocked) || (detected && have_busy_reading) {
            Verdict::Interfered
        } else {
            Verdict::Unknown
        };
        if impaired && !self.frozen {
            self.frozen = true;
            self.ref_rung = rung;
            self.ref_rec = rec_ref;
        }
        let lead_in = blocked
            || detected
            || (loss_impaired && have_busy_reading && min_foreign_busy_pct >= hv.detect_pct);
        if impaired && lead_in && !weak && !self.seen_blocked {
            self.seen_blocked = true;
            o.first_blocked = true;
        }
        self.healthy_streak = if o.v == Verdict::Healthy {
            self.healthy_streak + 1
        } else {
            0
        };
        if self.frozen && self.healthy_streak >= 3 && !self.ring_holds_detection() {
            self.frozen = false;
            self.ref_rung = -1;
            self.seen_blocked = false;
        }
        if !self.frozen {
            self.rec_hist.push_back(link.recovered as f64);
            if self.rec_hist.len() > hist_n {
                self.rec_hist.pop_front();
            }
        }
        o.ref_rung = self.ref_rung;
        o.ref_frozen = self.frozen;
        self.recent_interfered.push_back(o.v == Verdict::Interfered);
        if self.recent_interfered.len() > 3 {
            self.recent_interfered.pop_front();
        }
        o.trigger =
            self.recent_interfered.iter().filter(|&&x| x).count() as i32 >= self.cfg.persist;
        o
    }
}

#[cfg(test)]
mod tests {
    //! A selection of mabur `test_hop_verdict.cpp`; the whole verdict is
    //! held to mabur's by pal-ref `tests/verdict_diff.rs`.
    use super::*;

    fn card(rssi: f64, snr: f64) -> VerdictCardIn {
        VerdictCardIn {
            valid: true,
            rssi_dbm: rssi,
            snr_db: snr,
            ..Default::default()
        }
    }
    fn busy(nhm: f64, own: f64) -> VerdictCardIn {
        VerdictCardIn {
            busy_valid: true,
            nhm_busy_pct: nhm,
            own_air_pct: own,
            ..card(-61.0, 30.0)
        }
    }
    fn low_jam() -> [VerdictCardIn; 2] {
        [busy(45.0, 35.0), busy(45.0, 35.0)]
    }
    fn jam() -> [VerdictCardIn; 2] {
        [busy(95.0, 3.0), busy(95.0, 3.0)]
    }
    fn link(loss: f64, recovered: u32) -> VerdictLinkIn {
        VerdictLinkIn {
            pre_fec_loss: loss,
            recovered,
        }
    }
    fn hv() -> HopVerdict {
        HopVerdict::new(HopCfg::default(), BusyCfg::default())
    }
    /// 5 s of clean windows at rung 5 to build the references.
    fn warm(v: &mut HopVerdict) -> f64 {
        let mut t = 0.0;
        for _ in 0..40 {
            v.window(
                t,
                &[card(-61.0, 30.0), card(-61.0, 29.0)],
                &link(0.0, 20),
                5,
            );
            t += 150.0;
        }
        t
    }
    fn has(o: &VerdictOut, bit: u8) -> bool {
        o.evidence & bit != 0
    }
    const OCC_N: i32 = 5;
    const DET_N: i32 = 5;

    #[test]
    fn loss_with_foreign_frames_is_not_a_trigger() {
        let mut v = hv();
        let t = warm(&mut v);
        let o = v.window(t, &[card(-55.0, 33.0); 2], &link(0.06, 80), 5);
        assert_eq!(o.v, Verdict::Unknown);
        assert!(!o.trigger && has(&o, EV_IMPAIRED));
        assert_eq!(o.ref_rung, 5);
    }

    #[test]
    fn fade_is_fade_not_interfered() {
        let mut v = hv();
        let t = warm(&mut v);
        let o = v.window(
            t,
            &[card(-90.0, 8.0), card(-86.0, 10.0)],
            &link(0.05, 90),
            1,
        );
        assert_eq!(o.v, Verdict::Fade);
        assert!(has(&o, EV_WEAK));
    }

    #[test]
    fn occupied_fires_after_occupied_windows_with_zero_loss() {
        let mut v = hv();
        let mut t = warm(&mut v);
        for _ in 1..OCC_N {
            let o = v.window(t, &jam(), &link(0.0, 20), 5);
            assert_eq!(o.v, Verdict::Healthy);
            assert!(!has(&o, EV_OCCUPIED));
            t += 150.0;
        }
        let o = v.window(t, &jam(), &link(0.0, 20), 5);
        assert_eq!(o.v, Verdict::Interfered);
        assert!(has(&o, EV_OCCUPIED) && has(&o, EV_IMPAIRED) && !has(&o, EV_DETECTED));
        let o = v.window(t + 150.0, &jam(), &link(0.0, 20), 4);
        assert!(o.trigger);
        assert_eq!(o.ref_rung, 5);
    }

    #[test]
    fn no_busy_reading_holds_the_occupied_count() {
        let mut v = hv();
        let mut t = warm(&mut v);
        for _ in 1..OCC_N {
            v.window(t, &jam(), &link(0.0, 20), 5);
            t += 150.0;
        }
        v.window(t, &[card(-61.0, 30.0); 2], &link(0.0, 20), 5);
        t += 150.0;
        v.window(t, &[VerdictCardIn::default(); 2], &link(0.0, 20), 5);
        t += 150.0;
        let o = v.window(t, &jam(), &link(0.0, 20), 5);
        assert_eq!(o.v, Verdict::Interfered);
    }

    #[test]
    fn detected_fires_with_one_loss_window_inside_a_low_foreign_run() {
        let mut v = hv();
        let mut t = warm(&mut v);
        v.new_session();
        let o = v.window(t, &low_jam(), &link(0.04, 20), 5);
        assert_eq!((o.v, o.ref_rung), (Verdict::Unknown, 5));
        t += 150.0;
        for _ in 1..DET_N - 1 {
            assert!(!has(
                &v.window(t, &low_jam(), &link(0.0, 20), 4),
                EV_DETECTED
            ));
            t += 150.0;
        }
        let o = v.window(t, &low_jam(), &link(0.0, 20), 4);
        assert_eq!(o.v, Verdict::Interfered);
        assert!(has(&o, EV_DETECTED) && !has(&o, EV_BLOCKED) && !has(&o, EV_OCCUPIED));
        assert_eq!(o.ref_rung, 5, "the thaw was held while the ring detected");
    }

    #[test]
    fn detected_needs_a_loss_window() {
        let mut v = hv();
        let mut t = warm(&mut v);
        let mut o = VerdictOut::default();
        for _ in 0..DET_N + 2 {
            o = v.window(t, &low_jam(), &link(0.0, 20), 5);
            t += 150.0;
        }
        assert_eq!(o.v, Verdict::Healthy);
    }

    #[test]
    fn settle_debris_never_confirms_the_detect_ring() {
        let mut v = hv();
        let mut t = warm(&mut v);
        v.landed(t - 150.0);
        assert!(!has(
            &v.window(t, &low_jam(), &link(0.04, 20), 5),
            EV_DETECTED
        ));
        t += 150.0;
        for _ in 1..DET_N + 2 {
            assert!(!has(
                &v.window(t, &low_jam(), &link(0.0, 20), 4),
                EV_DETECTED
            ));
            t += 150.0;
        }
    }

    #[test]
    fn landed_keeps_the_blocked_streak_and_the_frozen_rung() {
        let mut v = hv();
        let mut t = warm(&mut v);
        for _ in 1..OCC_N {
            v.window(t, &jam(), &link(0.04, 20), 5);
            t += 150.0;
        }
        v.landed(t);
        let o = v.window(t, &jam(), &link(0.0, 20), 3);
        assert!(has(&o, EV_OCCUPIED) && !has(&o, EV_DETECTED));
        assert!(o.ref_frozen);
        assert_eq!(o.ref_rung, 5);
    }

    #[test]
    fn first_blocked_fires_once_per_frozen_episode() {
        let mut v = hv();
        let mut t = warm(&mut v);
        let o = v.window(t, &jam(), &link(0.05, 20), 5);
        assert!(o.first_blocked);
        t += 150.0;
        for _ in 0..OCC_N + 2 {
            assert!(!v.window(t, &jam(), &link(0.05, 20), 4).first_blocked);
            t += 150.0;
        }
        // A zero-loss blocked window is no lead-in: nothing is impaired yet.
        let mut v = hv();
        let t = warm(&mut v);
        assert!(!v.window(t, &jam(), &link(0.0, 20), 5).first_blocked);
    }

    #[test]
    fn trigger_evidence_names_both_routes() {
        assert!(trigger_evidence(EV_BLOCKED) && trigger_evidence(EV_DETECTED));
        assert!(!trigger_evidence(EV_IMPAIRED | EV_WEAK | EV_OCCUPIED));
    }

    #[test]
    fn raw_units_convert_to_dbm_and_db() {
        assert_eq!(rssi_raw_to_dbm(40.0), -70.0);
        assert_eq!(rssi_raw_to_dbm(0.0), 0.0);
        assert_eq!(snr_raw_to_db(24.0), 12.0);
    }
}
