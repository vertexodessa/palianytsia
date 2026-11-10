//! The card devourer-sys brought up, as the Rust channel core's LinkCard
//! (pal-ref's `PalCard` in Rust, itself PixelPilot's MaburCard: mabur's
//! RadioFrontend minus what the host owns). Threads: the RX thread
//! (`on_rx`, lock-free), the core thread (`send_control`, width resync),
//! the channel core's scout and in-flight threads (retunes, energy and NHM
//! reads). Every device call holds the card's control lock (`Card::ctl`);
//! the channel atomics change under it, so two retunes from two threads
//! cannot leave `channel` naming the wrong one.
use std::sync::atomic::{AtomicU64, AtomicU8, Ordering};
use std::sync::{Arc, Mutex, Weak};

use pal_core::chan::core::{MsClock, Sleeper, UsClock};
use pal_core::chan::own_air::{ht_mcs_of_rate, OwnAirAcc};
use pal_core::chan::radio::{
    CardCaps, LinkCard, NhmBusyRead, ScoutEnergy, ScoutFrames, ScoutRadio,
};
use pal_core::config::{ht40_offset, Config};
use pal_core::rust_core::{ChanEnv, RustCore};
use pal_core::CoreOpts;
use pal_wire::dot11;

use crate::{mono_default, Attach, Attached, RxTap};

fn to_scout(e: devourer_sys::Energy) -> ScoutEnergy {
    ScoutEnergy {
        fa_valid: e.valid_fa,
        cca_ofdm: e.cca_ofdm,
        fa_ofdm: e.fa_ofdm,
        igi_valid: e.valid_igi,
        igi: e.igi,
        nhm_valid: e.valid_nhm,
        floor_valid: e.valid_floor,
        floor_dbm: e.floor_dbm,
    }
}

/// How long each pair of the core's cards sat on different channels (a card
/// mid-retune is on none), so a card's frame rate can be judged on its
/// time on the link card's channel only. Taken on retunes and by the core
/// loop's stats pass, never on the RX path.
pub(crate) struct ChannelBook {
    inner: Mutex<Book>,
}

struct Book {
    at_us: u64,
    /// Per card; 0 while it retunes.
    ch: Vec<u8>,
    /// `apart_us[i * n + j]`: cards i and j on different channels.
    apart_us: Vec<u64>,
}

impl Book {
    fn advance(&mut self, now_us: u64) {
        let dt = now_us.saturating_sub(self.at_us);
        self.at_us = self.at_us.max(now_us);
        let n = self.ch.len();
        for i in 0..n {
            for j in 0..n {
                let (a, b) = (self.ch[i], self.ch[j]);
                if i != j && (a == 0 || b == 0 || a != b) {
                    self.apart_us[i * n + j] += dt;
                }
            }
        }
    }
}

impl ChannelBook {
    pub(crate) fn new(n: usize, ch: u8, now_us: u64) -> Self {
        Self {
            inner: Mutex::new(Book {
                at_us: now_us,
                ch: vec![ch; n],
                apart_us: vec![0; n * n],
            }),
        }
    }

    fn book(&self) -> std::sync::MutexGuard<'_, Book> {
        self.inner.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Card `card` is on `ch` (0 = retuning) from `now_us` on.
    pub(crate) fn set(&self, now_us: u64, card: usize, ch: u8) {
        let mut b = self.book();
        b.advance(now_us);
        if let Some(c) = b.ch.get_mut(card) {
            *c = ch;
        }
    }

    /// Time cards `card` and `other` spent apart up to `now_us`.
    pub(crate) fn apart_us(&self, now_us: u64, card: usize, other: usize) -> u64 {
        let mut b = self.book();
        b.advance(now_us);
        let n = b.ch.len();
        if card < n && other < n {
            b.apart_us[card * n + other]
        } else {
            0
        }
    }
}

pub struct DevCard {
    // Weak: the card's RX handler holds this (the tap), so a strong ref
    // would keep the card open forever.
    card: Weak<devourer_sys::Card>,
    /// The card's index in the core's roster (log lines only).
    idx: usize,
    caps: CardCaps,
    rx_frames: AtomicU64,
    foreign: AtomicU64,
    own: AtomicU64,
    own_air_us: AtomicU64,
    tx_frames: AtomicU64,
    tx_fail: AtomicU64,
    /// RX thread only; the lock is never contended.
    own_air: Mutex<OwnAirAcc>,
    channel: AtomicU8,
    width: AtomicU8,
    /// What the chip is tuned at: `width`, except 20 while a 40 MHz link
    /// sits on a channel without a pair (165).
    tuned_w: AtomicU8,
    /// The stamp RX bodies get; 0 while a retune is in flight.
    rx_channel: AtomicU8,
    /// Shared by the core's cards: where each one is tuned, over time.
    book: Arc<ChannelBook>,
}

impl DevCard {
    /// RadioFrontend::start_ after InitWrite: the caps and which energy
    /// counters this chip has.
    pub(crate) fn new(
        card: &Arc<devourer_sys::Card>,
        idx: usize,
        ch: u8,
        width: u8,
        book: Arc<ChannelBook>,
    ) -> Self {
        let (ac, e) = {
            let c = card.ctl();
            (c.caps(), c.rx_energy(true))
        };
        let caps = CardCaps {
            valid: ac.supported,
            chip: if ac.chip.is_empty() {
                "?".into()
            } else {
                ac.chip
            },
            gen: if ac.gen.is_empty() {
                "?".into()
            } else {
                ac.gen
            },
            tx_chains: ac.tx_chains,
            rx_chains: ac.rx_chains,
            bw_mask: ac.bw_mask,
            tune5g_lo: ac.tune5g_lo,
            tune5g_hi: ac.tune5g_hi,
            fast_retune: ac.fast_retune,
            fa_ok: e.valid_fa,
            igi_ok: e.valid_igi,
            nhm_ok: e.valid_nhm,
            floor_ok: e.valid_floor,
            snr_ok: true,
        };
        Self {
            card: Arc::downgrade(card),
            idx,
            caps,
            rx_frames: AtomicU64::new(0),
            foreign: AtomicU64::new(0),
            own: AtomicU64::new(0),
            own_air_us: AtomicU64::new(0),
            tx_frames: AtomicU64::new(0),
            tx_fail: AtomicU64::new(0),
            own_air: Mutex::new(OwnAirAcc::default()),
            channel: AtomicU8::new(ch),
            width: AtomicU8::new(if width == 40 { 40 } else { 20 }),
            tuned_w: AtomicU8::new(if width == 40 { 40 } else { 20 }),
            rx_channel: AtomicU8::new(ch),
            book,
        }
    }

    /// The RX stamp blinds for a move; the book counts the card as away.
    fn begin_move(&self) {
        self.rx_channel.store(0, Ordering::Release);
        self.book.set(mono_default(), self.idx, 0);
    }

    /// The card under its control lock; None once it is closed.
    fn with_ctl<R>(&self, f: impl FnOnce(&mut devourer_sys::Ctl) -> R) -> Option<R> {
        let card = self.card.upgrade()?;
        let mut c = card.ctl();
        Some(f(&mut c))
    }
}

impl RxTap for DevCard {
    /// RadioFrontend::on_packet's counters.
    fn on_rx(&self, f: &devourer_sys::RxFrame, v: &dot11::Verdict) -> Option<u8> {
        self.rx_frames.fetch_add(1, Ordering::Relaxed);
        match v {
            dot11::Verdict::Short => return None,
            dot11::Verdict::Foreign => {
                self.foreign.fetch_add(1, Ordering::Relaxed);
                return None;
            }
            dot11::Verdict::Body(_) => {}
        }
        if !f.crc_err {
            self.own.fetch_add(1, Ordering::Relaxed);
            let mut a = self.own_air.lock().unwrap_or_else(|e| e.into_inner());
            a.on_frame(
                f.frame.len(),
                ht_mcs_of_rate(f.data_rate),
                f.physt,
                if f.bw40 { 40 } else { 20 },
                f.stbc,
                f.sgi,
            );
            self.own_air_us.store(a.total_us(), Ordering::Relaxed);
        }
        Some(self.rx_channel.load(Ordering::Acquire))
    }
    fn channel(&self) -> u8 {
        LinkCard::channel(self)
    }
    fn off_link_us(&self, link: usize) -> u64 {
        self.book.apart_us(mono_default(), self.idx, link)
    }
}

impl DevCard {
    /// Ends a move, control lock held. `channel` follows only a move that
    /// worked: the retry must plan from where the card really is, or a failed
    /// HT40 side change is retried as a FastRetune that keeps the old side.
    fn end_move(&self, ch: u8, ok: bool) {
        if ok {
            self.channel.store(ch, Ordering::Release);
        }
        let now = self.channel.load(Ordering::Acquire);
        self.rx_channel.store(now, Ordering::Release);
        self.book.set(mono_default(), self.idx, now);
    }

    /// The full channel set (`set_monitor`) at `w`, control lock held;
    /// `width()` is not touched (a 40 MHz link on 165 tunes 20 and stays a
    /// 40 MHz link). `tuned_w` moves only on success: after a failure the
    /// next retune must still see the width mismatch and try the full set.
    fn tune_full(&self, c: &devourer_sys::Ctl, ch: u8, w: u8) -> bool {
        self.begin_move();
        let r = c.set_monitor(ch, w);
        self.end_move(ch, r.is_ok());
        match r {
            Ok(()) => {
                let was = self.tuned_w.swap(w, Ordering::AcqRel);
                eprintln!(
                    "pal radio: card {} width {was} -> {w} MHz on ch {ch}",
                    self.idx
                );
                true
            }
            Err(e) => {
                eprintln!("pal radio: card {}: {e}", self.idx);
                false
            }
        }
    }
}

/// How a move from `from` to `to` must be tuned: None = FastRetune (it
/// keeps width and HT40 offset), Some(w) = the full channel set at `w`.
/// The operator may choose channels on both sides of the pair grid (36 is
/// HT40+, 40 HT40-) and 165, which has no pair; at 40 MHz those moves cannot
/// keep the offset.
pub(crate) fn retune_plan(width: u8, tuned_w: u8, from: u8, to: u8) -> Option<u8> {
    let w = if width == 40 && ht40_offset(to) == 0 {
        20
    } else {
        width
    };
    (w != tuned_w || (w == 40 && ht40_offset(to) != ht40_offset(from))).then_some(w)
}

impl ScoutRadio for DevCard {
    fn retune(&self, ch: u8) -> bool {
        self.with_ctl(|c| {
            let from = self.channel.load(Ordering::Acquire);
            let tuned = self.tuned_w.load(Ordering::Acquire);
            if let Some(w) = retune_plan(self.width(), tuned, from, ch) {
                return self.tune_full(c, ch, w);
            }
            // Blind the body stamp for the move: a frame picked up during
            // it reads 0 = unknown, not the new channel.
            self.begin_move();
            let ok = c.fast_retune(ch);
            self.end_move(ch, ok);
            ok
        })
        .unwrap_or(false)
    }
    fn read_energy(&self, with_nhm: bool) -> ScoutEnergy {
        self.with_ctl(|c| to_scout(c.rx_energy(with_nhm)))
            .unwrap_or_default()
    }
    fn read_energy_scout(&self) -> ScoutEnergy {
        self.with_ctl(|c| to_scout(c.rx_energy_scout()))
            .unwrap_or_default()
    }
    fn frames(&self) -> ScoutFrames {
        ScoutFrames {
            own: self.own.load(Ordering::Relaxed),
            foreign: self.foreign.load(Ordering::Relaxed),
            own_air_us: self.own_air_us.load(Ordering::Relaxed),
        }
    }
    fn arm_nhm_busy(&self, period_4us: u16) -> bool {
        self.with_ctl(|c| c.arm_nhm_busy(period_4us))
            .unwrap_or(false)
    }
    fn read_nhm_busy(&self) -> NhmBusyRead {
        self.with_ctl(|c| {
            let b = c.read_nhm_busy();
            NhmBusyRead {
                valid: b.valid,
                buckets: b.buckets,
                duration: b.duration,
                period: b.period,
            }
        })
        .unwrap_or_default()
    }
}

impl LinkCard for DevCard {
    fn ready(&self) -> bool {
        true
    }
    fn alive(&self) -> bool {
        true
    }
    fn caps(&self) -> CardCaps {
        self.caps.clone()
    }
    fn rx_frames(&self) -> u64 {
        self.rx_frames.load(Ordering::Relaxed)
    }
    fn foreign(&self) -> u64 {
        self.foreign.load(Ordering::Relaxed)
    }
    fn send_control(&self, body: &[u8]) -> bool {
        let ok = self.with_ctl(|c| c.send_control(body)).unwrap_or(false);
        if ok { &self.tx_frames } else { &self.tx_fail }.fetch_add(1, Ordering::Relaxed);
        ok
    }
    fn tx_frames(&self) -> u64 {
        self.tx_frames.load(Ordering::Relaxed)
    }
    fn tx_fail(&self) -> u64 {
        self.tx_fail.load(Ordering::Relaxed)
    }
    fn set_width(&self, ch: u8, w: u8) -> bool {
        if w == 40 && ht40_offset(ch) == 0 {
            return false;
        }
        // Under the lock and only on success: a retune must not plan from a
        // width the chip has not reached.
        self.with_ctl(|c| {
            let ok = self.tune_full(c, ch, w);
            if ok {
                self.width.store(w, Ordering::Release);
            }
            ok
        })
        .unwrap_or(false)
    }
    fn channel(&self) -> u8 {
        self.channel.load(Ordering::Acquire)
    }
    fn width(&self) -> u8 {
        self.width.load(Ordering::Acquire)
    }
    fn tuned_central(&self) -> i32 {
        self.with_ctl(|c| c.tuned_central()).unwrap_or(-1)
    }
    fn can_scout(&self) -> bool {
        true
    }
}

/// Builds the Rust GS with its channel core on the cards once they are open
/// at `opts.channel` / `width` (pal-ref's `live::channel_core` for the Rust
/// core). Card 0 is the link card; with two or more the channel core
/// scouts on another and every card's bodies feed the aggregator. The
/// core's threads run on real time: `mono_default`, the clock the RX
/// stamps and ticks use.
pub fn rust_channel_core(cfg: Config, opts: CoreOpts, width: u8) -> Attach {
    Box::new(move |cards| {
        let book = Arc::new(ChannelBook::new(cards.len(), opts.channel, mono_default()));
        let dcs: Vec<Arc<DevCard>> = cards
            .iter()
            .enumerate()
            .map(|(i, c)| Arc::new(DevCard::new(c, i, opts.channel, width, book.clone())))
            .collect();
        let now_us: UsClock = Arc::new(mono_default);
        let now_ms: MsClock = Arc::new(|| mono_default() / 1000);
        let sleep: Sleeper = Arc::new(|ms| {
            if ms > 0 {
                std::thread::sleep(std::time::Duration::from_millis(ms as u64));
            }
        });
        let env = ChanEnv {
            cards: dcs.iter().map(|d| d.clone() as Arc<dyn LinkCard>).collect(),
            now_ms,
            now_us,
            sleep,
            threaded: true,
        };
        let core = RustCore::with_cards(&cfg, opts, env)?;
        Ok(Attached {
            core: Box::new(core),
            taps: dcs.into_iter().map(|d| d as Arc<dyn RxTap>).collect(),
            clock: mono_default,
        })
    })
}

#[cfg(test)]
mod book_tests {
    use super::ChannelBook;

    #[test]
    fn a_scout_dwell_counts_as_time_apart_from_the_link_card() {
        let b = ChannelBook::new(2, 100, 1_000_000);
        // Card 1: a 2 ms retune, 320 ms on 153, a 2 ms retune back.
        b.set(2_000_000, 1, 0);
        b.set(2_002_000, 1, 153);
        b.set(2_322_000, 1, 0);
        b.set(2_324_000, 1, 100);
        assert_eq!(b.apart_us(3_000_000, 1, 0), 324_000);
        assert_eq!(b.apart_us(3_000_000, 0, 1), 324_000);
        assert_eq!(b.apart_us(3_000_000, 1, 1), 0);
        // The link card hops and the scout follows 50 ms later.
        b.set(4_000_000, 0, 0);
        b.set(4_001_000, 0, 36);
        b.set(4_051_000, 1, 36);
        assert_eq!(b.apart_us(5_000_000, 1, 0), 324_000 + 51_000);
        // A card the book does not have reads 0.
        assert_eq!(b.apart_us(5_000_000, 2, 0), 0);
    }
}

#[cfg(test)]
mod retune_tests {
    use super::retune_plan;

    #[test]
    fn fast_retune_only_where_it_keeps_width_and_offset() {
        assert_eq!(retune_plan(20, 20, 36, 40), None);
        assert_eq!(retune_plan(40, 40, 40, 64), None, "both HT40-");
        assert_eq!(retune_plan(40, 40, 40, 36), Some(40), "HT40- to HT40+");
        assert_eq!(retune_plan(40, 40, 161, 165), Some(20), "165 has no pair");
        assert_eq!(
            retune_plan(40, 20, 165, 161),
            Some(40),
            "back to 40 on a pair"
        );
        assert_eq!(retune_plan(40, 20, 165, 165), None);
    }
}
