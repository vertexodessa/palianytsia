//! mabur `inflight_scout.{h,cpp}`: the ~10 ms mid-flight dwell, one timed
//! visit to a candidate on the spare card and back to `back` before
//! returning, so the live video on the other card sees nothing. A failed
//! retune must not strand the card: the dwell always tries to return, and
//! a failed return flags the record and books no visit (the caller's retune
//! loop off `plan.desired(card)` reconciles). Pure apart from the injected
//! radio, clock and sleep; ChannelCore owns the thread.

use std::sync::Arc;

use super::hop_ranker::HopVisit;
use super::radio::{nhm_busy_pct, nhm_period_4us, NhmBusyRead, ScoutRadio};
use super::scout::{ScoutDwell, FLAG_NHM_MISSING, FLAG_READ_FAILED, FLAG_RETUNE_FAILED};
use crate::config::ht40_offset;

#[derive(Clone, Debug, PartialEq)]
pub struct InflightScoutCfg {
    pub observe_ms: i32,
    pub period_ms: i32,
    /// The dwell set: radio.channels; the card's own channel is skipped.
    pub candidates: Vec<u8>,
    /// The link width: the dwell keeps the card's tuning; recorded.
    pub width_mhz: u8,
    pub busy_dbm: i32,
}

impl Default for InflightScoutCfg {
    fn default() -> Self {
        Self {
            observe_ms: 5,
            period_ms: 333,
            candidates: Vec::new(),
            width_mhz: 20,
            busy_dbm: -83,
        }
    }
}

pub type NowUsFn = Box<dyn Fn() -> i64 + Send>;
pub type SleepFn = Box<dyn Fn(i32) + Send>;

pub struct InflightScout {
    cfg: InflightScoutCfg,
    radio: Arc<dyn ScoutRadio>,
    now_us: NowUsFn,
    sleep_ms: SleepFn,
    next_idx: usize,
    seq: u64,
}

impl InflightScout {
    pub fn new(
        cfg: InflightScoutCfg,
        radio: Arc<dyn ScoutRadio>,
        now_us: NowUsFn,
        sleep_ms: SleepFn,
    ) -> Self {
        Self {
            cfg,
            radio,
            now_us,
            sleep_ms,
            next_idx: 0,
            seq: 0,
        }
    }

    /// The core picks the dwell card per cycle.
    /// The link width moved (a live width flip): later dwells record it.
    pub fn set_width_mhz(&mut self, w: u8) {
        self.cfg.width_mhz = w;
    }

    pub fn set_radio(&mut self, radio: Arc<dyn ScoutRadio>) {
        self.radio = radio;
    }

    pub fn cfg(&self) -> &InflightScoutCfg {
        &self.cfg
    }

    /// One dwell on `ch`, back to `back`. The record is always filled; the
    /// visit is Some exactly when both retunes succeeded (otherwise the
    /// record carries FLAG_RETUNE_FAILED): ChannelCore pairs records and
    /// visits by this.
    pub fn dwell(&mut self, ch: u8, back: u8) -> (ScoutDwell, Option<HopVisit>) {
        let w40 = self.cfg.width_mhz == 40;
        let mut d = ScoutDwell {
            seq: self.seq,
            ch,
            width_mhz: if w40 { 40 } else { 20 },
            offset: if w40 { ht40_offset(ch) } else { 0 },
            in_session: true,
            ..Default::default()
        };
        self.seq += 1;
        let r = &self.radio;
        let t0 = (self.now_us)();
        d.t_start_ms = t0 / 1000;
        if !r.retune(ch) {
            d.flags |= FLAG_RETUNE_FAILED;
            let _ = r.retune(back);
            d.t_end_ms = (self.now_us)() / 1000;
            return (d, None);
        }
        let t1 = (self.now_us)();
        let _ = r.read_energy_scout();
        let period = nhm_period_4us(self.cfg.observe_ms);
        let armed = r.arm_nhm_busy(period);
        let f0 = r.frames();
        let t2 = (self.now_us)();
        (self.sleep_ms)(self.cfg.observe_ms);
        let t3 = (self.now_us)();
        let nb = if armed {
            r.read_nhm_busy()
        } else {
            NhmBusyRead::default()
        };
        let busy = if nb.valid && nb.period == period {
            nhm_busy_pct(&nb, self.cfg.busy_dbm)
        } else {
            None
        };
        d.busy_valid = busy.is_some();
        d.busy_pct = busy.unwrap_or(0.0);
        let e = r.read_energy_scout();
        let f1 = r.frames();
        let t4 = (self.now_us)();
        let back_ok = r.retune(back);
        let t5 = (self.now_us)();

        d.retune_us = t1 - t0;
        d.settle_ms = 0; // this dwell is the ~10 ms budget itself
        d.observe_ms = (t3 - t2) / 1000;
        d.t_end_ms = t5 / 1000;
        d.valid_fa = e.fa_valid;
        d.fa_ofdm = e.fa_ofdm;
        d.cca_ofdm = e.cca_ofdm;
        d.valid_igi = e.igi_valid;
        d.igi = e.igi;
        d.valid_nhm = e.nhm_valid;
        if !e.nhm_valid {
            d.flags |= FLAG_NHM_MISSING;
        }
        if !e.fa_valid {
            d.flags |= FLAG_READ_FAILED;
        }
        let own = f1.own.wrapping_sub(f0.own) as u32;
        let foreign = f1.foreign.wrapping_sub(f0.foreign) as u32;
        d.dvr_frames = own;
        d.frames = own.wrapping_add(foreign);
        d.floor_valid = e.floor_valid;
        d.floor_dbm = e.floor_dbm;
        d.to_us = t1 - t0;
        d.read_us = t4 - t3;
        d.back_us = t5 - t4;
        if !back_ok {
            d.flags |= FLAG_RETUNE_FAILED;
            return (d, None);
        }
        let v = HopVisit {
            ch,
            t_ms: t0 as f64 / 1000.0,
            fa: e.fa_ofdm,
            cca: e.cca_ofdm,
            own,
            foreign,
            busy_valid: d.busy_valid,
            busy_pct: d.busy_pct,
            ..Default::default()
        };
        (d, Some(v))
    }

    /// The operator narrowed or widened `radio.use`: the dwells go to the
    /// new set from its first member.
    pub fn set_candidates(&mut self, set: Vec<u8>) {
        self.cfg.candidates = set;
        self.next_idx = 0;
    }

    /// Round-robin over the dwell set in config order, skipping the channel
    /// the card is on; None when that is the only one.
    pub fn next_candidate(&mut self, skip: u8) -> Option<u8> {
        let set = &self.cfg.candidates;
        for _ in 0..set.len() {
            let ch = set[self.next_idx];
            self.next_idx = (self.next_idx + 1) % set.len();
            if ch != skip {
                return Some(ch);
            }
        }
        None
    }

    /// Every channel of the set except `back`, back to back: one record per
    /// dwell appended to `records`, the visits of the ones that held.
    pub fn burst(&mut self, back: u8, records: &mut Vec<ScoutDwell>) -> Vec<HopVisit> {
        let set = self.cfg.candidates.clone();
        let mut visits = Vec::with_capacity(set.len());
        for ch in set.into_iter().filter(|&c| c != back) {
            let (d, v) = self.dwell(ch, back);
            visits.extend(v);
            records.push(d);
        }
        visits
    }
}

#[cfg(test)]
mod tests {
    use super::super::hop_ranker::HopRanker;
    use super::super::radio::{ScoutEnergy, ScoutFrames};
    use super::*;
    use crate::config::{BusyCfg, HopCfg};
    use std::sync::atomic::{AtomicI64, Ordering::SeqCst};
    use std::sync::Mutex;

    #[derive(Default)]
    struct St {
        log: Vec<String>,
        ch: u8,
        fa_next: u32,
        own: u64,
        foreign: u64,
        retunes: u32,
        armed: u16,
    }

    /// own/foreign are cumulative like the chip's; the sleep bumps foreign
    /// so the dwell's delta is real.
    #[derive(Default)]
    struct FakeRadio {
        st: Mutex<St>,
        fail_ch: Option<u8>,
        fail_nth: Option<u32>,
        nhm: bool,
        busy_136: bool,
    }

    impl FakeRadio {
        fn on(ch: u8) -> Self {
            let r = Self::default();
            r.st.lock().unwrap().ch = ch;
            r
        }
        fn s(&self) -> std::sync::MutexGuard<'_, St> {
            self.st.lock().unwrap()
        }
    }

    impl ScoutRadio for FakeRadio {
        fn retune(&self, c: u8) -> bool {
            let mut s = self.s();
            s.log.push(format!("retune {c}"));
            s.retunes += 1;
            if self.fail_ch == Some(c) || self.fail_nth == Some(s.retunes) {
                return false;
            }
            s.ch = c;
            true
        }
        fn read_energy(&self, _: bool) -> ScoutEnergy {
            self.s().log.push("read_full".into());
            ScoutEnergy::default()
        }
        fn read_energy_scout(&self) -> ScoutEnergy {
            let mut s = self.s();
            s.log.push("read_scout".into());
            let fa = if self.busy_136 {
                if s.ch == 136 {
                    1
                } else {
                    50
                }
            } else {
                s.fa_next
            };
            ScoutEnergy {
                fa_valid: true,
                fa_ofdm: fa,
                cca_ofdm: fa,
                ..Default::default()
            }
        }
        fn frames(&self) -> ScoutFrames {
            let s = self.s();
            ScoutFrames {
                own: s.own,
                foreign: s.foreign,
                own_air_us: 0,
            }
        }
        fn arm_nhm_busy(&self, p: u16) -> bool {
            if !self.nhm {
                return false;
            }
            let mut s = self.s();
            s.log.push("arm".into());
            s.armed = p;
            true
        }
        fn read_nhm_busy(&self) -> NhmBusyRead {
            let mut s = self.s();
            s.log.push("nhm".into());
            let mut r = NhmBusyRead {
                valid: true,
                period: s.armed,
                ..Default::default()
            };
            r.buckets[0] = 55;
            r.buckets[11] = 200;
            r
        }
    }

    fn cfg() -> InflightScoutCfg {
        InflightScoutCfg {
            candidates: vec![136, 120, 149, 165],
            ..Default::default()
        }
    }

    /// The scout on `r`; each sleep advances the clock and, with `bump`,
    /// lands 2 foreign frames.
    fn scout(
        c: InflightScoutCfg,
        r: &Arc<FakeRadio>,
        bump: bool,
    ) -> (InflightScout, Arc<AtomicI64>) {
        let t = Arc::new(AtomicI64::new(0));
        let (tn, ts, rs) = (t.clone(), t.clone(), r.clone());
        let s = InflightScout::new(
            c,
            r.clone(),
            Box::new(move || tn.load(SeqCst)),
            Box::new(move |ms| {
                ts.fetch_add(ms as i64 * 1000, SeqCst);
                if bump {
                    rs.s().foreign += 2;
                }
            }),
        );
        (s, t)
    }

    #[test]
    fn dwell_sequence_and_record() {
        let r = Arc::new(FakeRadio::on(136));
        let (mut s, _) = scout(cfg(), &r, true);
        r.s().fa_next = 13;
        r.s().foreign = 100; // a nonzero f0 tells a delta from an absolute count
        let (d, v) = s.dwell(149, 136);
        let v = v.unwrap();
        assert_eq!(
            r.s().log,
            vec!["retune 149", "read_scout", "read_scout", "retune 136"]
        );
        assert_eq!(r.s().ch, 136);
        assert!(d.in_session && d.observe_ms == 5 && d.fa_ofdm == 13);
        assert!(v.ch == 149 && v.fa == 13 && v.foreign == 2 && !v.busy_valid);
    }

    #[test]
    fn dwell_arms_and_reads_busy_for_the_observe_span() {
        let r = Arc::new(FakeRadio {
            nhm: true,
            ..FakeRadio::on(136)
        });
        let (mut s, _) = scout(cfg(), &r, false);
        let (d, v) = s.dwell(149, 136);
        let v = v.unwrap();
        assert_eq!(
            r.s().log,
            vec![
                "retune 149",
                "read_scout",
                "arm",
                "nhm",
                "read_scout",
                "retune 136"
            ]
        );
        assert!(v.busy_valid && (v.busy_pct - 100.0 * 200.0 / 255.0).abs() < 1e-9);
        assert!(d.busy_valid);
    }

    #[test]
    fn round_robin_and_burst() {
        let r = Arc::new(FakeRadio::on(136));
        let (mut s, _) = scout(cfg(), &r, true);
        let got: Vec<_> = (0..4).map(|_| s.next_candidate(136)).collect();
        assert_eq!(got, vec![Some(120), Some(149), Some(165), Some(120)]);
        let mut recs = Vec::new();
        let visits = s.burst(136, &mut recs);
        assert!(visits.len() == 3 && recs.len() == 3 && r.s().ch == 136);
    }

    #[test]
    fn rotation_visits_every_member_except_the_one_the_card_is_on() {
        let r = Arc::new(FakeRadio::on(136));
        let (mut s, _) = scout(cfg(), &r, false);
        let got: Vec<u8> = (0..8).map(|_| s.next_candidate(149).unwrap()).collect();
        assert_eq!(got, vec![136, 120, 165, 136, 120, 165, 136, 120]);
    }

    #[test]
    fn rotation_never_returns_the_skipped_channel() {
        let r = Arc::new(FakeRadio::on(136));
        let (mut s, _) = scout(cfg(), &r, false);
        for skip in [120, 149, 165, 136] {
            for _ in 0..9 {
                assert_ne!(s.next_candidate(skip), Some(skip));
            }
        }
    }

    #[test]
    fn nothing_to_dwell_on_is_none() {
        let r = Arc::new(FakeRadio::on(136));
        let (mut s, _) = scout(InflightScoutCfg::default(), &r, false);
        assert!(s.next_candidate(136).is_none() && s.next_candidate(144).is_none());
        let mut recs = Vec::new();
        assert!(s.burst(136, &mut recs).is_empty() && recs.is_empty() && r.s().log.is_empty());
    }

    #[test]
    fn burst_covers_the_whole_set_and_skips_back() {
        let r = Arc::new(FakeRadio::on(149));
        let (mut s, _) = scout(cfg(), &r, true);
        let mut recs = Vec::new();
        let v = s.burst(149, &mut recs);
        assert_eq!(recs.len(), 3);
        assert_eq!(
            v.iter().map(|v| v.ch).collect::<Vec<_>>(),
            vec![136, 120, 165]
        );
        assert_eq!(r.s().ch, 149);
    }

    #[test]
    fn a_clean_member_becomes_a_ranked_hop_target_after_a_hop() {
        let r = Arc::new(FakeRadio {
            busy_136: true,
            ..FakeRadio::on(149)
        });
        let (mut s, t) = scout(cfg(), &r, false);
        let mut ranker = HopRanker::new(
            HopCfg::default(),
            BusyCfg::default(),
            vec![120, 149, 165, 136],
            0,
            20,
        );
        for _ in 0..6 {
            let ch = s.next_candidate(149).unwrap();
            if let (_, Some(v)) = s.dwell(ch, 149) {
                ranker.add(v);
            }
        }
        assert_eq!(
            ranker.best(t.load(SeqCst) as f64 / 1000.0, 149, &[], false),
            Some(136)
        );
    }

    #[test]
    fn failed_retune_returns_none_and_leaves_card_on_back() {
        let r = Arc::new(FakeRadio {
            fail_ch: Some(149),
            ..FakeRadio::on(0)
        });
        let (mut s, _) = scout(cfg(), &r, true);
        let (d, v) = s.dwell(149, 136);
        assert!(v.is_none() && d.flags & FLAG_RETUNE_FAILED != 0);
        assert_eq!(r.s().ch, 136);
    }

    #[test]
    fn failed_return_retune_flags_and_refuses_the_visit() {
        let r = Arc::new(FakeRadio {
            fail_nth: Some(2),
            ..FakeRadio::on(136)
        });
        let (mut s, _) = scout(cfg(), &r, true);
        r.s().fa_next = 13;
        let (d, v) = s.dwell(149, 136);
        assert!(v.is_none() && d.flags & FLAG_RETUNE_FAILED != 0);
        assert_eq!(d.fa_ofdm, 13);
        assert_eq!(r.s().ch, 149);
    }

    #[test]
    fn set_radio_repoints_the_next_dwell() {
        let (r1, r2) = (Arc::new(FakeRadio::on(200)), Arc::new(FakeRadio::on(210)));
        let (mut s, _) = scout(cfg(), &r1, false);
        s.set_radio(r2.clone());
        let (_, v) = s.dwell(149, 136);
        assert!(v.is_some());
        assert!(r1.s().log.is_empty());
        let log = r2.s().log.clone();
        assert!(log.len() == 4 && log[0] == "retune 149" && log[3] == "retune 136");
        assert_eq!(r2.s().ch, 136);
    }

    #[test]
    fn success_iff_visit_and_no_retune_flag() {
        for (fail_ch, fail_nth, ok) in [
            (Some(149), None, false),
            (None, Some(2), false),
            (None, None, true),
        ] {
            let r = Arc::new(FakeRadio {
                fail_ch,
                fail_nth,
                ..FakeRadio::on(136)
            });
            let (mut s, _) = scout(cfg(), &r, true);
            let (d, v) = s.dwell(149, 136);
            assert_eq!(v.is_some(), ok);
            assert_eq!(d.flags & FLAG_RETUNE_FAILED == 0, ok);
            if let Some(v) = v {
                assert_eq!(v.ch, 149);
            }
        }
    }

    #[test]
    fn dwell_record_carries_the_tuned_width_and_pair_offset() {
        let r = Arc::new(FakeRadio::on(136));
        let c40 = InflightScoutCfg {
            candidates: vec![144],
            width_mhz: 40,
            ..Default::default()
        };
        let (mut s, _) = scout(c40, &r, false);
        let (d, _) = s.dwell(144, 136);
        assert!(d.width_mhz == 40 && d.offset == 2);
        let c20 = InflightScoutCfg {
            candidates: vec![149],
            ..Default::default()
        };
        let (mut s20, _) = scout(c20, &r, false);
        let (d, _) = s20.dwell(149, 136);
        assert!(d.width_mhz == 20 && d.offset == 0);
    }
}
