//! mabur `hop_verdict.{h,cpp}`: one verdict per hop window from the link's
//! loss and every card's RF counters. Impaired + weak = fade; impaired +
//! blocked (NHM) or contended/raised without a fade = interfered. The
//! trailing references (RSSI median, recovered and AU means) freeze on the
//! first impaired window and thaw after 3 healthy ones.

use std::collections::VecDeque;

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

pub const EV_IMPAIRED: u8 = 1;
pub const EV_WEAK: u8 = 2;
pub const EV_FADING: u8 = 4;
pub const EV_CONTENDED: u8 = 8;
pub const EV_RAISED: u8 = 16;
pub const EV_BLOCKED: u8 = 32;
pub const EV_STARVED: u8 = 64;

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
    pub starved: bool,
    pub au_count: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VerdictOut {
    pub v: Verdict,
    pub evidence: u8,
    /// -1 while healthy.
    pub ref_rung: i32,
    /// Interfered in >= persist of the last 3 windows.
    pub trigger: bool,
    pub ref_rssi_dbm: f64,
    pub d_rssi_db: f64,
    pub t_start_ms: f64,
    pub t_ms: f64,
    pub ref_frozen: bool,
    pub first_interfered: bool,
}

impl Default for VerdictOut {
    fn default() -> Self {
        Self {
            v: Verdict::Healthy,
            evidence: 0,
            ref_rung: -1,
            trigger: false,
            ref_rssi_dbm: 0.0,
            d_rssi_db: 0.0,
            t_start_ms: 0.0,
            t_ms: 0.0,
            ref_frozen: false,
            first_interfered: false,
        }
    }
}

#[derive(Clone, Debug)]
pub struct HopVerdict {
    cfg: HopCfg,
    busy: BusyCfg,
    n_cards: usize,
    rssi_hist: Vec<VecDeque<f64>>,
    rec_hist: VecDeque<f64>,
    au_hist: VecDeque<f64>,
    frozen: bool,
    prev_ms: f64,
    have_prev: bool,
    seen_interfered: bool,
    ref_rssi: Vec<f64>,
    ref_rec: f64,
    ref_au: f64,
    ref_rung: i32,
    healthy_streak: i32,
    recent_interfered: VecDeque<bool>,
}

fn median(v: &VecDeque<f64>) -> f64 {
    if v.is_empty() {
        return 0.0;
    }
    let mut s: Vec<f64> = v.iter().copied().collect();
    s.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let n = s.len();
    if n % 2 == 1 {
        s[n / 2]
    } else {
        (s[n / 2 - 1] + s[n / 2]) / 2.0
    }
}

fn mean(v: &VecDeque<f64>) -> f64 {
    if v.is_empty() {
        0.0
    } else {
        v.iter().sum::<f64>() / v.len() as f64
    }
}

impl HopVerdict {
    pub fn new(cfg: HopCfg, busy: BusyCfg, n_cards: usize) -> Self {
        Self {
            cfg,
            busy,
            n_cards,
            rssi_hist: vec![VecDeque::new(); n_cards],
            rec_hist: VecDeque::new(),
            au_hist: VecDeque::new(),
            frozen: false,
            prev_ms: 0.0,
            have_prev: false,
            seen_interfered: false,
            ref_rssi: Vec::new(),
            ref_rec: 0.0,
            ref_au: 0.0,
            ref_rung: -1,
            healthy_streak: 0,
            recent_interfered: VecDeque::new(),
        }
    }

    pub fn reset(&mut self) {
        self.frozen = false;
        self.ref_rec = 0.0;
        self.ref_au = 0.0;
        self.ref_rung = -1;
        self.ref_rssi.clear();
        self.healthy_streak = 0;
        self.seen_interfered = false;
        self.recent_interfered.clear();
    }

    pub fn new_session(&mut self) {
        self.au_hist.clear();
        self.ref_au = 0.0;
    }

    pub fn ref_rung(&self) -> i32 {
        self.ref_rung
    }

    pub fn window(
        &mut self,
        now_ms: f64,
        cards: &[VerdictCardIn],
        link: &VerdictLinkIn,
        rung: i32,
    ) -> VerdictOut {
        let w_s = self.cfg.window_ms as f64 / 1000.0;
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
        let hv = &self.cfg.verdict;
        let rec_ref = if self.frozen {
            self.ref_rec
        } else {
            mean(&self.rec_hist)
        };
        let au_ref = if self.frozen {
            self.ref_au
        } else {
            mean(&self.au_hist)
        };
        let starved = link.starved
            || (hv.starved_frac > 0.0
                && au_ref >= 2.0
                && (link.au_count as f64) < hv.starved_frac * au_ref);
        let impaired = link.pre_fec_loss * 100.0 > hv.loss_pct
            || (rec_ref > 0.0
                && link.recovered as f64 > hv.recovered_x * rec_ref
                && link.recovered >= hv.recovered_min.max(0) as u32)
            || starved;
        let b = &cards[best];
        let weak = b.rssi_dbm < hv.weak_rssi_dbm as f64
            && (!b.snr_valid || b.snr_db < hv.weak_snr_db as f64);
        let frozen_ref = if self.frozen && best < self.ref_rssi.len() {
            self.ref_rssi[best]
        } else {
            0.0
        };
        let rssi_ref = if self.frozen {
            if frozen_ref != 0.0 {
                frozen_ref
            } else {
                b.rssi_dbm
            }
        } else if self.rssi_hist[best].is_empty() {
            b.rssi_dbm
        } else {
            median(&self.rssi_hist[best])
        };
        let fading = b.rssi_dbm < rssi_ref - hv.fading_drop_db as f64;
        let (mut contended, mut raised) = (false, false);
        let mut min_foreign_busy: Option<f64> = None;
        for c in cards.iter().filter(|c| c.valid) {
            contended = contended || c.foreign as f64 / w_s > hv.foreign_pps as f64;
            raised = raised || c.fa as f64 / w_s > hv.fa_pps as f64;
            if c.busy_valid {
                let foreign_pct = (c.nhm_busy_pct - c.own_air_pct).max(0.0);
                min_foreign_busy =
                    Some(min_foreign_busy.map_or(foreign_pct, |m| m.min(foreign_pct)));
            }
        }
        let blocked = min_foreign_busy.is_some_and(|m| m >= self.busy.blocked_pct);
        let flag = |on: bool, bit: u8| if on { bit } else { 0 };
        o.evidence = flag(impaired, EV_IMPAIRED)
            | flag(weak, EV_WEAK)
            | flag(fading, EV_FADING)
            | flag(contended, EV_CONTENDED)
            | flag(raised, EV_RAISED)
            | flag(blocked, EV_BLOCKED)
            | flag(starved, EV_STARVED);
        o.v = if !impaired {
            Verdict::Healthy
        } else if weak {
            Verdict::Fade
        } else if blocked || ((contended || raised) && !fading) {
            Verdict::Interfered
        } else {
            Verdict::Unknown
        };
        if impaired && !self.frozen {
            self.frozen = true;
            self.ref_rung = rung;
            self.ref_rec = rec_ref;
            self.ref_au = au_ref;
            if self.ref_rssi.len() != self.n_cards {
                self.ref_rssi = vec![0.0; self.n_cards];
            }
            for i in 0..self.n_cards {
                if !self.rssi_hist[i].is_empty() {
                    self.ref_rssi[i] = median(&self.rssi_hist[i]);
                } else if i < cards.len() && cards[i].valid {
                    self.ref_rssi[i] = cards[i].rssi_dbm;
                }
            }
        }
        if o.v == Verdict::Interfered && !self.seen_interfered {
            self.seen_interfered = true;
            o.first_interfered = true;
        }
        self.healthy_streak = if o.v == Verdict::Healthy {
            self.healthy_streak + 1
        } else {
            0
        };
        if self.frozen && self.healthy_streak >= 3 {
            self.frozen = false;
            self.ref_rung = -1;
            self.seen_interfered = false;
        }
        if !self.frozen {
            for (i, c) in cards.iter().enumerate().take(self.n_cards) {
                if c.valid {
                    let h = &mut self.rssi_hist[i];
                    h.push_back(c.rssi_dbm);
                    if h.len() > hist_n {
                        h.pop_front();
                    }
                }
            }
            for (h, v) in [
                (&mut self.rec_hist, link.recovered as f64),
                (&mut self.au_hist, link.au_count as f64),
            ] {
                h.push_back(v);
                if h.len() > hist_n {
                    h.pop_front();
                }
            }
        }
        o.ref_rung = self.ref_rung;
        o.ref_rssi_dbm = rssi_ref;
        o.d_rssi_db = b.rssi_dbm - rssi_ref;
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
    use super::*;

    fn cfg() -> HopCfg {
        HopCfg::default()
    }
    fn bcfg() -> BusyCfg {
        BusyCfg::default()
    }
    fn card(rssi: f64, snr: f64, foreign_ps: f64, fa_ps: f64, crc_ps: f64) -> VerdictCardIn {
        let n = |ps: f64| (ps * 0.15 + 0.5) as u32;
        VerdictCardIn {
            valid: true,
            rssi_dbm: rssi,
            snr_db: snr,
            foreign: n(foreign_ps),
            fa: n(fa_ps),
            cca: n(fa_ps),
            crc_fail: n(crc_ps),
            ..Default::default()
        }
    }
    fn c(rssi: f64, snr: f64, foreign_ps: f64, fa_ps: f64) -> VerdictCardIn {
        card(rssi, snr, foreign_ps, fa_ps, 0.0)
    }
    fn link(loss: f64, recovered: u32) -> VerdictLinkIn {
        VerdictLinkIn {
            pre_fec_loss: loss,
            recovered,
            ..Default::default()
        }
    }
    fn dead() -> VerdictCardIn {
        VerdictCardIn::default()
    }
    fn clean() -> [VerdictCardIn; 2] {
        [c(-61.0, 30.0, 0.0, 4.0), c(-61.0, 29.0, 0.0, 4.0)]
    }
    fn jam() -> [VerdictCardIn; 2] {
        [c(-55.0, 33.0, 237.0, 2.0), c(-55.0, 33.0, 237.0, 2.0)]
    }
    fn fade_cards() -> [VerdictCardIn; 2] {
        [
            card(-90.0, 8.0, 0.0, 0.0, 30.0),
            card(-86.0, 10.0, 0.0, 10.0, 5.0),
        ]
    }
    /// 5 s of clean windows at rung 5 to build the references.
    fn warm(v: &mut HopVerdict) -> f64 {
        let mut t = 0.0;
        for _ in 0..40 {
            v.window(t, &clean(), &link(0.0, 20), 5);
            t += 150.0;
        }
        t
    }
    fn hv(n: usize) -> HopVerdict {
        HopVerdict::new(cfg(), bcfg(), n)
    }
    fn has(o: &VerdictOut, bit: u8) -> bool {
        o.evidence & bit != 0
    }

    #[test]
    fn baseline_is_healthy() {
        let mut v = hv(2);
        let t = warm(&mut v);
        let o = v.window(t, &clean(), &link(0.005, 20), 5);
        assert_eq!(o.v, Verdict::Healthy);
        assert!(!o.trigger);
        assert_eq!(o.ref_rung, -1);
    }

    #[test]
    fn co_channel_802_11_neighbour_is_interfered_after_two_windows() {
        let mut v = hv(2);
        let t = warm(&mut v);
        let o1 = v.window(t, &jam(), &link(0.06, 80), 5);
        assert_eq!(o1.v, Verdict::Interfered);
        assert!(!o1.trigger);
        assert_eq!(o1.ref_rung, 5);
        assert!(has(&o1, EV_IMPAIRED) && has(&o1, EV_CONTENDED) && !has(&o1, EV_RAISED));
        let o2 = v.window(t + 150.0, &jam(), &link(0.06, 80), 4);
        assert!(o2.trigger);
        assert_eq!(o2.ref_rung, 5);
    }

    #[test]
    fn o4_raised_floor_is_interfered_even_with_rssi_up() {
        let mut v = hv(2);
        let t = warm(&mut v);
        let o = v.window(
            t,
            &[
                card(-48.0, 30.0, 0.0, 744.0, 130.0),
                card(-50.0, 27.0, 0.0, 388.0, 130.0),
            ],
            &link(0.85, 5),
            0,
        );
        assert_eq!(o.v, Verdict::Interfered);
        assert!(has(&o, EV_RAISED) && !has(&o, EV_FADING));
    }

    #[test]
    fn fade_is_fade_not_interfered() {
        let mut v = hv(2);
        let t = warm(&mut v);
        let o = v.window(t, &fade_cards(), &link(0.05, 90), 1);
        assert_eq!(o.v, Verdict::Fade);
        assert!(has(&o, EV_WEAK) && !o.trigger);
    }

    #[test]
    fn loss_with_no_domain_evidence_is_unknown() {
        let mut v = hv(2);
        let t = warm(&mut v);
        let o = v.window(t, &clean(), &link(0.08, 100), 5);
        assert_eq!(o.v, Verdict::Unknown);
        assert!(!o.trigger);
    }

    #[test]
    fn off_channel_blocking_reads_as_raised() {
        let mut v = hv(2);
        let t = warm(&mut v);
        let o = v.window(
            t,
            &[
                card(-59.0, 28.0, 0.0, 209.0, 94.0),
                card(-59.0, 28.0, 0.0, 299.0, 130.0),
            ],
            &link(0.04, 70),
            4,
        );
        assert_eq!(o.v, Verdict::Interfered);
        assert!(has(&o, EV_RAISED));
    }

    #[test]
    fn recovered_rate_alone_marks_impaired_and_its_reference_freezes() {
        let mut v = hv(2);
        let t = warm(&mut v);
        let o1 = v.window(t, &jam(), &link(0.0, 100), 5);
        assert!(has(&o1, EV_IMPAIRED));
        for i in 1..30 {
            v.window(t + 150.0 * i as f64, &jam(), &link(0.0, 100), 0);
        }
        let o2 = v.window(t + 150.0 * 30.0, &jam(), &link(0.0, 100), 0);
        assert!(has(&o2, EV_IMPAIRED));
    }

    #[test]
    fn skipped_card_does_not_contribute_and_one_card_fade_covered_is_healthy() {
        let mut v = hv(2);
        let t = warm(&mut v);
        let o = v.window(t, &[c(-61.0, 30.0, 0.0, 4.0), dead()], &link(0.004, 20), 5);
        assert_eq!(o.v, Verdict::Healthy);
        let o2 = v.window(
            t + 150.0,
            &[c(-88.0, 9.0, 0.0, 4.0), c(-61.0, 30.0, 0.0, 4.0)],
            &link(0.004, 20),
            5,
        );
        assert_eq!(o2.v, Verdict::Healthy);
    }

    #[test]
    fn references_thaw_after_three_healthy_windows() {
        let mut v = hv(2);
        let t = warm(&mut v);
        v.window(t, &jam(), &link(0.06, 80), 5);
        assert_eq!(v.ref_rung(), 5);
        for i in 1..=3 {
            v.window(t + 150.0 * i as f64, &clean(), &link(0.0, 20), 3);
        }
        assert_eq!(v.ref_rung(), -1);
    }

    #[test]
    fn frozen_reference_for_a_mid_dwell_card_is_not_fabricated_from_zero() {
        let mut v = hv(2);
        let mut t = 0.0;
        for _ in 0..40 {
            v.window(t, &[c(-61.0, 30.0, 0.0, 4.0), dead()], &link(0.0, 20), 5);
            t += 150.0;
        }
        let o1 = v.window(t, &[c(-55.0, 33.0, 237.0, 2.0), dead()], &link(0.06, 80), 5);
        assert_eq!(o1.v, Verdict::Interfered);
        assert_eq!(v.ref_rung(), 5);
        let o2 = v.window(
            t + 150.0,
            &[c(-55.0, 33.0, 237.0, 2.0), c(-40.0, 30.0, 0.0, 4.0)],
            &link(0.06, 80),
            5,
        );
        assert!(!has(&o2, EV_FADING));
    }

    #[test]
    fn weak_takes_priority_over_contended_when_not_fading() {
        let mut v = hv(2);
        let mut t = 0.0;
        for _ in 0..40 {
            v.window(
                t,
                &[c(-80.0, 10.0, 0.0, 4.0), c(-80.0, 10.0, 0.0, 4.0)],
                &link(0.0, 20),
                5,
            );
            t += 150.0;
        }
        let o = v.window(
            t,
            &[c(-80.0, 10.0, 237.0, 2.0), c(-80.0, 10.0, 237.0, 2.0)],
            &link(0.06, 20),
            5,
        );
        assert_eq!(o.v, Verdict::Fade);
        assert!(!o.trigger);
        assert!(
            has(&o, EV_IMPAIRED)
                && has(&o, EV_WEAK)
                && has(&o, EV_CONTENDED)
                && !has(&o, EV_FADING)
        );
    }

    #[test]
    fn every_window_carries_the_span_it_was_measured_over() {
        let mut v = hv(2);
        let o1 = v.window(1000.0, &clean(), &link(0.0, 20), 5);
        assert!(o1.t_start_ms == 1000.0 && o1.t_ms == 1000.0);
        let o2 = v.window(1150.0, &clean(), &link(0.0, 20), 5);
        assert!(o2.t_start_ms == 1000.0 && o2.t_ms == 1150.0);
        let o3 = v.window(1300.0, &clean(), &link(0.0, 20), 5);
        assert!(o3.t_start_ms == 1150.0 && o3.t_ms == 1300.0);
        let o4 = v.window(1450.0, &[dead(), dead()], &link(0.0, 20), 5);
        assert!(o4.v == Verdict::Unknown && o4.t_start_ms == 1300.0 && o4.t_ms == 1450.0);
    }

    #[test]
    fn first_interfered_fires_once_per_frozen_episode() {
        let mut v = hv(2);
        let mut t = warm(&mut v);
        let f = v.window(t, &fade_cards(), &link(0.05, 90), 1);
        assert!(f.v == Verdict::Fade && f.ref_frozen && !f.first_interfered);
        t += 150.0;
        let o1 = v.window(t, &jam(), &link(0.06, 80), 5);
        assert!(o1.v == Verdict::Interfered && o1.first_interfered);
        t += 150.0;
        let o2 = v.window(t, &jam(), &link(0.06, 80), 5);
        assert!(o2.v == Verdict::Interfered && !o2.first_interfered);
        for i in 1..=3 {
            v.window(t + 150.0 * i as f64, &clean(), &link(0.0, 20), 5);
        }
        let o3 = v.window(t + 600.0, &jam(), &link(0.06, 80), 5);
        assert!(o3.first_interfered);
        v.reset();
        let o4 = v.window(t + 750.0, &jam(), &link(0.06, 80), 5);
        assert!(o4.first_interfered);
    }

    #[test]
    fn ref_frozen_marks_the_impaired_episode() {
        let mut v = hv(2);
        let mut t = warm(&mut v);
        assert!(!v.window(t, &clean(), &link(0.0, 20), 5).ref_frozen);
        t += 150.0;
        let o = v.window(t, &jam(), &link(0.06, 80), 5);
        assert_eq!(o.v, Verdict::Interfered);
        assert!(o.ref_frozen);
        t += 150.0;
        assert!(v.window(t, &clean(), &link(0.0, 20), 5).ref_frozen);
        t += 150.0;
        assert!(v.window(t, &clean(), &link(0.0, 20), 5).ref_frozen);
        t += 150.0;
        assert!(!v.window(t, &clean(), &link(0.0, 20), 5).ref_frozen);
    }

    #[test]
    fn reset_thaws_the_snapshot_and_clears_the_latched_trigger() {
        let mut v = hv(2);
        let t = warm(&mut v);
        let o1 = v.window(t, &jam(), &link(0.06, 80), 5);
        let o2 = v.window(t + 150.0, &jam(), &link(0.06, 80), 5);
        assert!(o1.ref_frozen && o2.trigger && v.ref_rung() == 5);
        v.reset();
        assert_eq!(v.ref_rung(), -1);
        let o3 = v.window(t + 300.0, &clean(), &link(0.0, 20), 5);
        assert!(!o3.trigger && !o3.ref_frozen && o3.v == Verdict::Healthy);
    }

    #[test]
    fn raw_units_convert_to_dbm_and_db() {
        assert_eq!(rssi_raw_to_dbm(64.0), -46.0);
        assert_eq!(snr_raw_to_db(66.0), 33.0);
        assert_eq!(rssi_raw_to_dbm(0.0), 0.0);
    }

    #[test]
    fn converted_edge_of_range_reading_is_fade() {
        let mut v = hv(2);
        let t = warm(&mut v);
        let (r, s) = (rssi_raw_to_dbm(28.0), snr_raw_to_db(20.0));
        let o = v.window(
            t,
            &[c(r, s, 0.0, 4.0), c(r, s, 0.0, 4.0)],
            &link(0.06, 80),
            5,
        );
        assert_eq!(o.v, Verdict::Fade);
        assert!(has(&o, EV_WEAK));
        let raw = v.window(
            t + 150.0,
            &[c(28.0, 20.0, 0.0, 4.0), c(28.0, 20.0, 0.0, 4.0)],
            &link(0.06, 80),
            5,
        );
        assert!(!has(&raw, EV_WEAK));
    }

    fn busy_card(busy: f64, own: f64, valid: bool) -> VerdictCardIn {
        VerdictCardIn {
            valid: true,
            rssi_dbm: -48.0,
            snr_db: 30.0,
            busy_valid: valid,
            nhm_busy_pct: busy,
            own_air_pct: own,
            ..Default::default()
        }
    }
    fn bc(busy: f64, own: f64) -> VerdictCardIn {
        busy_card(busy, own, true)
    }

    #[test]
    fn long_frame_jam_is_interfered_via_blocked() {
        let mut v = hv(1);
        let mut o = VerdictOut::default();
        let mut t = 0.0;
        for _ in 0..3 {
            t += 150.0;
            o = v.window(t, &[bc(95.0, 2.0)], &link(0.80, 0), 0);
        }
        assert_eq!(o.v, Verdict::Interfered);
        assert!(has(&o, EV_BLOCKED) && o.trigger);
    }

    #[test]
    fn blocked_threshold_comes_from_busy_cfg() {
        let b = BusyCfg {
            blocked_pct: 30.0,
            ..bcfg()
        };
        let (mut low, mut def) = (HopVerdict::new(cfg(), b, 1), hv(1));
        let (mut ol, mut od) = (VerdictOut::default(), VerdictOut::default());
        let mut t = 0.0;
        for _ in 0..3 {
            t += 150.0;
            ol = low.window(t, &[bc(40.0, 2.0)], &link(0.80, 0), 0);
            od = def.window(t, &[bc(40.0, 2.0)], &link(0.80, 0), 0);
        }
        assert!(has(&ol, EV_BLOCKED));
        assert!(!has(&od, EV_BLOCKED));
    }

    #[test]
    fn busy_but_healthy_link_stays_healthy() {
        let o = hv(1).window(150.0, &[bc(95.0, 2.0)], &link(0.0, 0), 0);
        assert_eq!(o.v, Verdict::Healthy);
    }

    #[test]
    fn own_airtime_is_subtracted() {
        let o = hv(1).window(150.0, &[bc(60.0, 45.0)], &link(0.20, 0), 0);
        assert!(!has(&o, EV_BLOCKED));
        assert_eq!(o.v, Verdict::Unknown);
    }

    #[test]
    fn blocked_beats_fading_but_not_weak() {
        let bad = link(0.5, 0);
        {
            let mut v = hv(1);
            let mut t = 0.0;
            for _ in 0..10 {
                t += 150.0;
                v.window(t, &[bc(0.0, 2.0)], &VerdictLinkIn::default(), 0);
            }
            let c = VerdictCardIn {
                rssi_dbm: -63.0,
                ..bc(100.0, 2.0)
            };
            let o = v.window(t + 150.0, &[c], &bad, 0);
            assert!(has(&o, EV_FADING));
            assert_eq!(o.v, Verdict::Interfered);
        }
        {
            let c = VerdictCardIn {
                rssi_dbm: -85.0,
                snr_db: 5.0,
                ..bc(100.0, 2.0)
            };
            assert_eq!(hv(1).window(150.0, &[c], &bad, 0).v, Verdict::Fade);
        }
    }

    #[test]
    fn no_busy_reading_is_todays_verdict() {
        let o = hv(1).window(150.0, &[busy_card(100.0, 0.0, false)], &link(0.8, 0), 0);
        assert_eq!(o.v, Verdict::Unknown);
        assert!(!has(&o, EV_BLOCKED));
    }

    #[test]
    fn weak_diversity_card_alone_is_not_blocked() {
        let o = hv(2).window(150.0, &[bc(70.0, 5.0), bc(70.0, 68.0)], &link(0.2, 0), 0);
        assert!(!has(&o, EV_BLOCKED));
        assert_ne!(o.v, Verdict::Interfered);
    }

    #[test]
    fn interferer_seen_by_both_cards_is_blocked() {
        let o = hv(2).window(150.0, &[bc(95.0, 2.0), bc(95.0, 2.0)], &link(0.2, 0), 0);
        assert!(has(&o, EV_BLOCKED));
        assert_eq!(o.v, Verdict::Interfered);
    }

    #[test]
    fn card_without_reading_does_not_veto() {
        let o = hv(2).window(
            150.0,
            &[bc(95.0, 2.0), busy_card(100.0, 0.0, false)],
            &link(0.2, 0),
            0,
        );
        assert!(has(&o, EV_BLOCKED));
        assert_eq!(o.v, Verdict::Interfered);
    }

    /// Trailing recovered mean ~0.2/window.
    fn warm_quiet(v: &mut HopVerdict) -> f64 {
        let mut t = 0.0;
        for i in 0..40 {
            v.window(t, &clean(), &link(0.0, u32::from(i % 5 == 0)), 5);
            t += 150.0;
        }
        t
    }

    #[test]
    fn recovered_below_floor_is_not_impaired() {
        let mut v = hv(2);
        let t = warm_quiet(&mut v);
        let o = v.window(t, &clean(), &link(0.0, 2), 5);
        assert!(!has(&o, EV_IMPAIRED));
        assert_eq!(o.v, Verdict::Healthy);
    }

    #[test]
    fn recovered_at_floor_is_impaired() {
        let mut v = hv(2);
        let t = warm_quiet(&mut v);
        assert!(has(&v.window(t, &clean(), &link(0.0, 12), 5), EV_IMPAIRED));
        let mut v8 = hv(2);
        let t = warm_quiet(&mut v8);
        assert!(has(&v8.window(t, &clean(), &link(0.0, 8), 5), EV_IMPAIRED));
    }

    #[test]
    fn recovered_min_zero_disables_floor() {
        let mut c0 = cfg();
        c0.verdict.recovered_min = 0;
        let mut v = HopVerdict::new(c0, bcfg(), 2);
        let t = warm_quiet(&mut v);
        assert!(has(&v.window(t, &clean(), &link(0.0, 2), 5), EV_IMPAIRED));
    }

    fn starved() -> VerdictLinkIn {
        VerdictLinkIn {
            starved: true,
            ..Default::default()
        }
    }

    #[test]
    fn starved_and_blocked_is_interfered() {
        let mut v = hv(2);
        let t = warm(&mut v);
        let o = v.window(t, &[bc(98.0, 0.0), bc(98.0, 0.0)], &starved(), 5);
        assert_eq!(o.v, Verdict::Interfered);
        assert!(has(&o, EV_STARVED) && has(&o, EV_BLOCKED) && has(&o, EV_IMPAIRED));
    }

    #[test]
    fn starved_alone_is_unknown() {
        let mut v = hv(2);
        let t = warm(&mut v);
        let mut o = VerdictOut::default();
        for i in 0..3 {
            o = v.window(t + 150.0 * i as f64, &clean(), &starved(), 5);
        }
        assert_eq!(o.v, Verdict::Unknown);
        assert!(has(&o, EV_STARVED) && !o.trigger);
    }

    #[test]
    fn starved_windows_do_not_feed_the_trailing_references() {
        let mut v = hv(2);
        let mut t = warm(&mut v);
        for _ in 0..30 {
            assert!(v.window(t, &clean(), &starved(), 5).ref_frozen);
            t += 150.0;
        }
        for _ in 0..3 {
            v.window(t, &clean(), &link(0.0, 20), 5);
            t += 150.0;
        }
        assert!(!has(&v.window(t, &clean(), &link(0.0, 12), 5), EV_IMPAIRED));
    }

    fn au_link(au: u32) -> VerdictLinkIn {
        VerdictLinkIn {
            recovered: 20,
            au_count: au,
            ..Default::default()
        }
    }
    fn warm_au(v: &mut HopVerdict, au: u32, n: usize) -> f64 {
        let mut t = 0.0;
        for _ in 0..n {
            v.window(t, &clean(), &au_link(au), 5);
            t += 150.0;
        }
        t
    }
    fn busy2() -> [VerdictCardIn; 2] {
        [bc(98.0, 0.0), bc(98.0, 0.0)]
    }

    #[test]
    fn au_collapse_is_starved() {
        let mut v = hv(2);
        let t = warm_au(&mut v, 9, 10);
        let o = v.window(t, &busy2(), &au_link(1), 5);
        assert!(has(&o, EV_STARVED) && has(&o, EV_IMPAIRED));
        assert_eq!(o.v, Verdict::Interfered);
    }

    #[test]
    fn half_rate_is_not_starved() {
        let mut v = hv(2);
        let t = warm_au(&mut v, 9, 10);
        let o = v.window(t, &busy2(), &au_link(4), 5);
        assert!(!has(&o, EV_STARVED));
        assert_eq!(o.v, Verdict::Healthy);
    }

    #[test]
    fn no_au_baseline_is_not_starved() {
        let o = hv(2).window(0.0, &busy2(), &au_link(0), 5);
        assert!(!has(&o, EV_STARVED));
        assert_eq!(o.v, Verdict::Healthy);
        let mut w = hv(2);
        let t = warm_au(&mut w, 1, 10);
        assert!(!has(&w.window(t, &busy2(), &au_link(0), 5), EV_STARVED));
    }

    #[test]
    fn starved_frac_zero_disables_au_rule() {
        let mut c0 = cfg();
        c0.verdict.starved_frac = 0.0;
        let mut v = HopVerdict::new(c0, bcfg(), 2);
        let t = warm_au(&mut v, 9, 10);
        let o = v.window(t, &busy2(), &au_link(1), 5);
        assert!(!has(&o, EV_STARVED));
        assert_eq!(o.v, Verdict::Healthy);
    }

    #[test]
    fn frozen_au_reference_is_used_while_frozen() {
        let mut v = hv(2);
        let mut t = warm_au(&mut v, 9, 33);
        for _ in 0..30 {
            let o = v.window(t, &busy2(), &au_link(1), 5);
            assert!(has(&o, EV_STARVED) && o.ref_frozen);
            t += 150.0;
        }
        v.reset();
        assert!(has(&v.window(t, &busy2(), &au_link(1), 5), EV_STARVED));
    }

    #[test]
    fn new_session_clears_au_baseline() {
        let mut v = hv(2);
        let mut t = warm_au(&mut v, 9, 10);
        v.new_session();
        for _ in 0..3 {
            assert!(!has(&v.window(t, &busy2(), &au_link(0), 5), EV_STARVED));
            t += 150.0;
        }
        assert!(has(&v.window(t, &clean(), &link(0.0, 100), 5), EV_IMPAIRED));
    }

    #[test]
    fn weak_without_a_real_snr_fires_on_rssi_alone() {
        let mut v = hv(2);
        let t = warm(&mut v);
        let relay = VerdictCardIn {
            snr_valid: false,
            ..card(-90.0, 40.0, 0.0, 0.0, 30.0)
        };
        let o = v.window(t, &[relay, dead()], &link(0.05, 90), 1);
        assert_eq!(o.v, Verdict::Fade);
        assert!(has(&o, EV_WEAK));
        let mut u = hv(2);
        let t = warm(&mut u);
        let o2 = u.window(
            t,
            &[card(-90.0, 40.0, 0.0, 0.0, 30.0), dead()],
            &link(0.05, 90),
            1,
        );
        assert!(!has(&o2, EV_WEAK));
    }
}
