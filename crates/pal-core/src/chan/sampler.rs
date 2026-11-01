//! mabur `window_sampler.{h,cpp}`: the channel-health window's card reads,
//! off the core thread. Every hop.window_ms it samples each card (NHM busy
//! read, FA/CCA delta, frame counters, NHM re-arm) and hands the core one
//! `WindowRecord` through a one-slot mailbox. The reads are synchronous USB
//! register I/O sharing devourer's per-device lock with the card's coex
//! tick, which the core thread must never wait on.
//!
//! Threaded (`start`/`stop`) or stepped (`step` from the core tick). Owns
//! the per-card `NhmWindowTracker` (a reading counts only if this sampler
//! armed it, same channel, no dwell or core retune since) and the per-card
//! device-use lock (`hold_card`), which card stop/reopen, the core's dwell
//! burst and the in-flight scout's dwell take too. Relay survey reads and
//! the calibration gate are not ported (USB cards only, no calibration).

use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, AtomicU64, AtomicU8, Ordering::*};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::JoinHandle;

use super::gate::verdict_card_usable;
use super::radio::{nhm_busy_pct, LinkCard, NhmWindowTracker, ScoutEnergy, ScoutFrames};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CardSample {
    /// Ready, not blocked, on the op channel.
    pub valid: bool,
    /// None = no NHM evidence this window.
    pub nhm_busy_pct: Option<f64>,
    /// read_energy_scout(): fa/cca since the previous read.
    pub energy: ScoutEnergy,
    /// Counters at the sample instant.
    pub frames: ScoutFrames,
    pub channel: u8,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct WindowRecord {
    /// The core's ms clock at the sample instant.
    pub t_ms: u64,
    pub cards: Vec<CardSample>,
    /// The reset generation it was sampled under.
    pub epoch: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WindowSamplerCfg {
    pub window_ms: i32,
    /// nhm_period_4us(window_ms - 10)
    pub nhm_period_4us: u16,
    pub busy_dbm: i32,
}

impl Default for WindowSamplerCfg {
    fn default() -> Self {
        Self {
            window_ms: 150,
            nhm_period_4us: 0,
            busy_dbm: -83,
        }
    }
}

/// Facts the core thread owns and writes; the sampler only reads them.
/// card_blocked = the boot scout owns the card; a dwell in progress is
/// dwell_busy/dwell_card (-1 = none).
#[derive(Debug)]
pub struct WindowGates {
    pub in_session: AtomicBool,
    pub cal_running: AtomicBool,
    pub op: AtomicU8,
    pub dwell_busy: AtomicBool,
    pub dwell_card: AtomicI32,
    pub dwell_gen: Vec<AtomicU32>,
    pub tune_gen: Vec<AtomicU32>,
    pub card_blocked: Vec<AtomicBool>,
}

impl WindowGates {
    pub fn new(n_cards: usize) -> Self {
        Self {
            in_session: AtomicBool::new(false),
            cal_running: AtomicBool::new(false),
            op: AtomicU8::new(0),
            dwell_busy: AtomicBool::new(false),
            dwell_card: AtomicI32::new(-1),
            dwell_gen: (0..n_cards).map(|_| AtomicU32::new(0)).collect(),
            tune_gen: (0..n_cards).map(|_| AtomicU32::new(0)).collect(),
            card_blocked: (0..n_cards).map(|_| AtomicBool::new(false)).collect(),
        }
    }
}

pub type NowMsFn = Box<dyn Fn() -> u64 + Send>;
pub type SleepMsFn = Box<dyn Fn(i32) + Send>;

/// Shared between the core and the sampling thread (`Arc`). The thread
/// holds a clone, so call `stop()` before dropping the cards.
pub struct WindowSampler {
    cards: Vec<Arc<dyn LinkCard>>,
    cfg: WindowSamplerCfg,
    gates: Arc<WindowGates>,
    card_mu: Vec<Mutex<()>>,
    /// The sampling side only.
    trackers: Mutex<Vec<NhmWindowTracker>>,
    reset_req: AtomicBool,
    /// Bumped by reset(); pins publish() to its pass.
    epoch: AtomicU32,
    /// Stepped cadence (core thread).
    last_ms: AtomicU64,
    mailbox: Mutex<Option<WindowRecord>>,
    dropped: AtomicU64,
    run: AtomicBool,
    th: Mutex<Option<JoinHandle<()>>>,
}

impl WindowSampler {
    pub fn new(
        cards: Vec<Arc<dyn LinkCard>>,
        cfg: WindowSamplerCfg,
        gates: Arc<WindowGates>,
    ) -> Self {
        let n = cards.len();
        Self {
            cards,
            cfg,
            gates,
            card_mu: (0..n).map(|_| Mutex::new(())).collect(),
            trackers: Mutex::new(vec![NhmWindowTracker::default(); n]),
            reset_req: AtomicBool::new(false),
            epoch: AtomicU32::new(0),
            last_ms: AtomicU64::new(0),
            mailbox: Mutex::new(None),
            dropped: AtomicU64::new(0),
            run: AtomicBool::new(false),
            th: Mutex::new(None),
        }
    }

    pub fn active(&self) -> bool {
        self.gates.in_session.load(Relaxed) && !self.gates.cal_running.load(Relaxed)
    }

    pub fn sample_once(&self, now_ms: u64) -> WindowRecord {
        let mut trackers = self.trackers.lock().unwrap();
        if self.reset_req.swap(false, AcqRel) {
            trackers.iter_mut().for_each(|t| t.invalidate());
        }
        let mut rec = WindowRecord {
            t_ms: now_ms,
            epoch: self.epoch.load(Acquire),
            cards: vec![CardSample::default(); self.cards.len()],
        };
        let g = &self.gates;
        let op = g.op.load(Relaxed);
        for (i, fe) in self.cards.iter().enumerate() {
            let _card = self.card_mu[i].lock().unwrap();
            let s = &mut rec.cards[i];
            let blocked = g.card_blocked[i].load(Acquire)
                || (g.dwell_busy.load(Acquire) && g.dwell_card.load(Acquire) == i as i32);
            if !verdict_card_usable(fe.ready(), blocked, fe.channel(), op) {
                trackers[i].invalidate();
                continue;
            }
            let nb = fe.read_nhm_busy();
            let dgen = g.dwell_gen[i].load(Acquire);
            let tgen = g.tune_gen[i].load(Acquire);
            if trackers[i].usable(&nb, fe.channel(), dgen, tgen) {
                s.nhm_busy_pct = nhm_busy_pct(&nb, self.cfg.busy_dbm);
            }
            s.energy = fe.read_energy_scout();
            s.frames = fe.frames();
            s.channel = fe.channel();
            if fe.arm_nhm_busy(self.cfg.nhm_period_4us) {
                trackers[i].armed(s.channel, self.cfg.nhm_period_4us, dgen, tgen);
            } else {
                trackers[i].invalidate();
            }
            s.valid = true;
        }
        rec
    }

    /// Card i's device-use lock, held while the guard lives. Never call
    /// stop() while holding it: the pass stop() joins may wait on it.
    pub fn hold_card(&self, i: usize) -> MutexGuard<'_, ()> {
        self.card_mu[i].lock().unwrap()
    }

    /// The hop-inactive edge: drop every arm and the mailbox, bump the
    /// epoch so a pass in flight can never republish its pre-edge record,
    /// and restart the stepped cadence.
    pub fn reset(&self, now_ms: u64) {
        self.reset_req.store(true, Release);
        self.last_ms.store(now_ms, Relaxed);
        let mut mb = self.mailbox.lock().unwrap();
        *mb = None;
        self.epoch.fetch_add(1, Release);
    }

    /// Stepped mode: publishes when a window is due and the gate is active.
    pub fn step(&self, now_ms: u64) {
        if !self.active()
            || now_ms.wrapping_sub(self.last_ms.load(Relaxed)) < self.cfg.window_ms.max(0) as u64
        {
            return;
        }
        self.step_now(now_ms);
    }

    /// Sample and publish now, whatever the cadence, and restart it.
    pub fn step_now(&self, now_ms: u64) {
        self.last_ms.store(now_ms, Relaxed);
        self.publish(self.sample_once(now_ms));
    }

    /// Threaded mode: sleep(window_ms) between passes.
    pub fn start(self: &Arc<Self>, now_ms: NowMsFn, sleep_ms: SleepMsFn) {
        if self.run.swap(true, AcqRel) {
            return;
        }
        let me = self.clone();
        let h = std::thread::Builder::new()
            .name("pal-window".into())
            .spawn(move || {
                while me.run.load(Acquire) {
                    sleep_ms(me.cfg.window_ms);
                    if !me.run.load(Acquire) {
                        break;
                    }
                    if me.active() {
                        me.publish(me.sample_once(now_ms()));
                    }
                }
            })
            .expect("spawn window sampler");
        *self.th.lock().unwrap() = Some(h);
    }

    /// Idempotent; joins.
    pub fn stop(&self) {
        self.run.store(false, Release);
        if let Some(h) = self.th.lock().unwrap().take() {
            let _ = h.join();
        }
    }

    /// Newest wins; a replaced record counts as dropped and its FA/CCA
    /// delta folds into the newer one; a record from before a reset() is
    /// discarded silently.
    pub fn publish(&self, mut r: WindowRecord) {
        let mut mb = self.mailbox.lock().unwrap();
        if r.epoch != self.epoch.load(Acquire) {
            return;
        }
        if let Some(old) = mb.as_ref() {
            self.dropped.fetch_add(1, Relaxed);
            for (o, c) in old.cards.iter().zip(r.cards.iter_mut()) {
                if o.valid && c.valid && o.channel == c.channel {
                    c.energy.fa_ofdm = c.energy.fa_ofdm.wrapping_add(o.energy.fa_ofdm);
                    c.energy.cca_ofdm = c.energy.cca_ofdm.wrapping_add(o.energy.cca_ofdm);
                }
            }
        }
        *mb = Some(r);
    }

    /// Core thread.
    pub fn take(&self) -> Option<WindowRecord> {
        self.mailbox.lock().unwrap().take()
    }

    pub fn dropped(&self) -> u64 {
        self.dropped.load(Relaxed)
    }
}

#[cfg(test)]
mod tests {
    use super::super::fake::{FakeCard, FakeClock};
    use super::super::radio::NhmBusyRead;
    use super::*;
    use std::sync::Condvar;
    use std::time::{Duration, Instant};

    struct Rig {
        cards: Vec<Arc<FakeCard>>,
        gates: Arc<WindowGates>,
        s: Arc<WindowSampler>,
    }

    fn cfg(window_ms: i32) -> WindowSamplerCfg {
        WindowSamplerCfg {
            window_ms,
            nhm_period_4us: 35000,
            busy_dbm: -83,
        }
    }

    impl Rig {
        fn new(n: usize) -> Self {
            Self::with(n, 150)
        }
        fn with(n: usize, window_ms: i32) -> Self {
            let clk = FakeClock::default();
            let cards: Vec<Arc<FakeCard>> =
                (0..n).map(|_| FakeCard::new(clk.clone(), 40)).collect();
            let gates = Arc::new(WindowGates::new(n));
            gates.in_session.store(true, SeqCst);
            gates.op.store(40, SeqCst);
            let dyn_cards: Vec<Arc<dyn LinkCard>> = cards
                .iter()
                .map(|c| c.clone() as Arc<dyn LinkCard>)
                .collect();
            let s = Arc::new(WindowSampler::new(dyn_cards, cfg(window_ms), gates.clone()));
            Self { cards, gates, s }
        }
        fn nhm(&self, i: usize, period: u16) {
            let mut c = self.cards[i].s();
            c.nhm_arm_ok = true;
            c.nhm_read = busy_read(period);
        }
    }

    /// Half the buckets sit above -83 dBm.
    fn busy_read(period: u16) -> NhmBusyRead {
        NhmBusyRead {
            valid: true,
            period,
            buckets: [10; 12],
            ..Default::default()
        }
    }

    #[test]
    fn one_pass_samples_every_card_in_order() {
        let g = Rig::new(2);
        g.cards[0].s().fa_next = 7;
        g.cards[1].s().fa_next = 9;
        g.cards[0].s().fr.own = 100;
        g.cards[1].s().fr.own = 200;
        let r = g.s.sample_once(1000);
        assert_eq!(r.cards.len(), 2);
        assert_eq!(r.t_ms, 1000);
        assert!(r.cards[0].valid && r.cards[1].valid);
        assert!(r.cards[0].energy.fa_ofdm == 7 && r.cards[1].energy.fa_ofdm == 9);
        assert!(r.cards[0].frames.own == 100 && r.cards[1].frames.own == 200);
        assert_eq!(r.cards[0].channel, 40);
        let c = g.cards[0].calls();
        assert!(c.len() >= 2 && c[0] == "read_nhm" && c.last().unwrap() == "arm_nhm 35000");
    }

    #[test]
    fn nhm_pct_needs_our_own_arm() {
        let g = Rig::new(1);
        g.nhm(0, 35000);
        assert!(g.s.sample_once(1000).cards[0].nhm_busy_pct.is_none());
        let second = g.s.sample_once(1150);
        assert_eq!(
            second.cards[0].nhm_busy_pct,
            nhm_busy_pct(&busy_read(35000), -83)
        );
        assert!(second.cards[0].nhm_busy_pct.is_some());
    }

    #[test]
    fn dwell_tune_channel_or_period_change_empties_nhm_pct() {
        let g = Rig::new(1);
        g.nhm(0, 35000);
        g.s.sample_once(1000);
        g.gates.dwell_gen[0].fetch_add(1, SeqCst);
        assert!(g.s.sample_once(1150).cards[0].nhm_busy_pct.is_none());
        g.gates.tune_gen[0].fetch_add(1, SeqCst);
        assert!(g.s.sample_once(1300).cards[0].nhm_busy_pct.is_none());
        g.cards[0].s().nhm_read = busy_read(1250);
        assert!(g.s.sample_once(1450).cards[0].nhm_busy_pct.is_none());
        g.cards[0].s().nhm_read = busy_read(35000);
        assert!(g.s.sample_once(1600).cards[0].nhm_busy_pct.is_some());
    }

    #[test]
    fn blocked_off_op_or_not_ready_card_is_invalid_and_disarmed() {
        let g = Rig::new(2);
        g.nhm(0, 35000);
        g.nhm(1, 35000);
        g.s.sample_once(1000);
        g.gates.card_blocked[0].store(true, SeqCst);
        g.gates.dwell_busy.store(true, SeqCst);
        g.gates.dwell_card.store(1, SeqCst);
        let r = g.s.sample_once(1150);
        assert!(!r.cards[0].valid && !r.cards[1].valid);
        g.gates.card_blocked[0].store(false, SeqCst);
        g.gates.dwell_busy.store(false, SeqCst);
        g.gates.dwell_card.store(-1, SeqCst);
        let r = g.s.sample_once(1300);
        assert!(r.cards[0].valid && r.cards[0].nhm_busy_pct.is_none());
        g.cards[0].s().ch = 64;
        assert!(!g.s.sample_once(1450).cards[0].valid);
        g.cards[0].s().ch = 40;
        g.cards[0].s().is_ready = false;
        assert!(!g.s.sample_once(1600).cards[0].valid);
    }

    #[test]
    fn reset_drops_arms_and_the_mailbox() {
        let g = Rig::new(1);
        g.nhm(0, 35000);
        g.s.publish(g.s.sample_once(1000));
        g.s.reset(1100);
        assert!(g.s.take().is_none());
        assert!(g.s.sample_once(1250).cards[0].nhm_busy_pct.is_none());
    }

    #[test]
    fn record_sampled_before_reset_is_not_published() {
        let g = Rig::new(1);
        let r = g.s.sample_once(1000);
        g.s.reset(1100);
        g.s.publish(r);
        assert!(g.s.take().is_none());
        assert_eq!(g.s.dropped(), 0);
        g.s.publish(g.s.sample_once(1250));
        assert!(g.s.take().is_some());
    }

    #[test]
    fn mailbox_keeps_newest_and_counts_dropped() {
        let g = Rig::new(1);
        g.s.publish(g.s.sample_once(1000));
        g.s.publish(g.s.sample_once(1150));
        assert_eq!(g.s.dropped(), 1);
        assert_eq!(g.s.take().unwrap().t_ms, 1150);
        assert!(g.s.take().is_none());
    }

    #[test]
    fn stepped_cadence_is_one_record_per_window_while_active() {
        let g = Rig::new(1);
        g.s.reset(1000);
        g.s.step(1100);
        assert!(g.s.take().is_none());
        g.s.step(1150);
        assert!(g.s.take().is_some());
        g.s.step(1160);
        assert!(g.s.take().is_none());
        g.gates.in_session.store(false, SeqCst);
        g.s.step(1400);
        assert!(g.s.take().is_none());
        g.gates.in_session.store(true, SeqCst);
        g.s.step_now(1500);
        assert!(g.s.take().is_some());
        g.s.step(1600);
        assert!(g.s.take().is_none());
    }

    #[test]
    fn thread_publishes_and_stops() {
        let g = Rig::with(1, 20);
        let t0 = Instant::now();
        g.s.start(
            Box::new(move || t0.elapsed().as_millis() as u64),
            Box::new(|ms| std::thread::sleep(Duration::from_millis(ms as u64))),
        );
        std::thread::sleep(Duration::from_millis(120));
        g.s.stop();
        g.s.stop();
        assert!(g.s.take().is_some());
        assert!(g.s.dropped() >= 1);
    }

    #[test]
    fn hold_card_waits_for_an_in_flight_pass() {
        let g = Rig::with(1, 20);
        let card = g.cards[0].clone();
        card.s().block_next_read = true;
        g.s.start(
            Box::new(|| 1000),
            Box::new(|ms| std::thread::sleep(Duration::from_millis(ms as u64))),
        );
        drop(card.cv.wait_while(card.s(), |s| !s.in_read).unwrap());
        let sync = Arc::new((Mutex::new((false, false)), Condvar::new())); // (held, done)
        let (s2, y2) = (g.s.clone(), sync.clone());
        let holder = std::thread::spawn(move || {
            let _h = s2.hold_card(0);
            y2.0.lock().unwrap().0 = true;
            y2.1.notify_all();
            drop(y2.1.wait_while(y2.0.lock().unwrap(), |st| !st.1).unwrap());
        });
        {
            let (st, to) = sync
                .1
                .wait_timeout_while(sync.0.lock().unwrap(), Duration::from_millis(100), |st| {
                    !st.0
                })
                .unwrap();
            assert!(to.timed_out() && !st.0);
        }
        card.s().release = true;
        card.cv.notify_all();
        {
            let mut st = sync
                .1
                .wait_while(sync.0.lock().unwrap(), |st| !st.0)
                .unwrap();
            st.1 = true;
        }
        sync.1.notify_all();
        holder.join().unwrap();
        g.s.stop();
    }

    #[test]
    fn hold_card_excludes_a_pass() {
        let g = Rig::new(1);
        let card = g.cards[0].clone();
        let hold = g.s.hold_card(0);
        std::thread::scope(|sc| {
            let pass = sc.spawn(|| g.s.publish(g.s.sample_once(1000)));
            {
                let (st, to) = card
                    .cv
                    .wait_timeout_while(card.s(), Duration::from_millis(100), |s| !s.touched)
                    .unwrap();
                assert!(to.timed_out() && !st.touched);
            }
            drop(hold);
            drop(card.cv.wait_while(card.s(), |s| !s.touched).unwrap());
            pass.join().unwrap();
        });
        assert!(g.s.take().is_some());
    }

    #[test]
    fn replaced_record_folds_its_energy_delta_into_the_newer_one() {
        let g = Rig::new(2);
        let rec = |t: u64, fa0: u32, cca0: u32, v1: bool, fa1: u32| {
            let mut r = WindowRecord {
                t_ms: t,
                cards: vec![CardSample::default(); 2],
                epoch: 0,
            };
            r.cards[0].valid = true;
            r.cards[0].energy.fa_ofdm = fa0;
            r.cards[0].energy.cca_ofdm = cca0;
            r.cards[1].valid = v1;
            r.cards[1].energy.fa_ofdm = fa1;
            r
        };
        g.s.publish(rec(1000, 7, 3, false, 50));
        g.s.publish(rec(1150, 5, 2, true, 4));
        let r = g.s.take().unwrap();
        assert_eq!(r.t_ms, 1150);
        assert!(r.cards[0].energy.fa_ofdm == 12 && r.cards[0].energy.cca_ofdm == 5);
        assert_eq!(r.cards[1].energy.fa_ofdm, 4);
        assert_eq!(g.s.dropped(), 1);
    }
}
