//! The card devourer-sys brought up, as the Rust channel core's LinkCard
//! (pal-ref's `PalCard` in Rust, itself PixelPilot's MaburCard: mabur's
//! RadioFrontend minus what the host owns). Threads: the RX thread
//! (`on_rx`, lock-free), the core thread (`send_control`, width resync),
//! the channel core's scout and in-flight threads (retunes, energy and NHM
//! reads). Every device call holds the card's control lock (`Card::ctl`);
//! the channel atomics change under it, so two retunes from two threads
//! cannot leave `channel` naming the wrong one.
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicU8, Ordering};
use std::sync::{Arc, Mutex, Weak};

use pal_core::chan::core::{MsClock, Sleeper, UsClock};
use pal_core::chan::own_air::{ht_mcs_of_rate, OwnAirAcc};
use pal_core::chan::radio::{
    CardCaps, LinkCard, NhmBusyRead, ScoutEnergy, ScoutFrames, ScoutRadio,
};
use pal_core::config::{ht40_offset, Config};
use pal_core::rust_core::{ChanEnv, RustCore, RustOpts};
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

pub struct DevCard {
    // Weak: the card's RX handler holds this (the tap), so a strong ref
    // would keep the card open forever.
    card: Weak<devourer_sys::Card>,
    /// The card's index in the core's roster (log lines only).
    idx: usize,
    caps: CardCaps,
    ready: AtomicBool,
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
}

impl DevCard {
    /// RadioFrontend::start_ after InitWrite: the caps and which energy
    /// counters this chip has.
    pub fn new(card: &Arc<devourer_sys::Card>, idx: usize, ch: u8, width: u8) -> Self {
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
            ready: AtomicBool::new(true),
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
        }
    }

    /// The card under its control lock; None once it is closed or down.
    fn with_ctl<R>(&self, f: impl FnOnce(&devourer_sys::Ctl) -> R) -> Option<R> {
        if !self.ready() {
            return None;
        }
        let card = self.card.upgrade()?;
        let c = card.ctl();
        Some(f(&c))
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
}

impl DevCard {
    /// The full channel set (`set_monitor`) at `w`; `width()` is not
    /// touched (a 40 MHz link on 165 tunes 20 and stays a 40 MHz link).
    fn tune_full(&self, ch: u8, w: u8) -> bool {
        let was = self.tuned_w.load(Ordering::Acquire);
        self.tuned_w.store(w, Ordering::Release);
        self.with_ctl(|c| {
            self.rx_channel.store(0, Ordering::Release);
            let r = c.set_monitor(ch, w);
            self.channel.store(ch, Ordering::Release);
            self.rx_channel.store(ch, Ordering::Release);
            match r {
                Ok(()) => {
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
        })
        .unwrap_or(false)
    }
}

/// How a move from `from` to `to` must be tuned: None = FastRetune (it
/// keeps width and HT40 offset), Some(w) = the full channel set at `w`.
/// The operator may choose channels on both sides of the pair grid (36 is
/// HT40+, 40 HT40-) and 165, which has no pair; at 40 MHz those moves cannot
/// keep the offset.
fn retune_plan(width: u8, tuned_w: u8, from: u8, to: u8) -> Option<u8> {
    let w = if width == 40 && ht40_offset(to) == 0 {
        20
    } else {
        width
    };
    (w != tuned_w || (w == 40 && ht40_offset(to) != ht40_offset(from))).then_some(w)
}

impl ScoutRadio for DevCard {
    fn retune(&self, ch: u8) -> bool {
        let from = self.channel.load(Ordering::Acquire);
        if let Some(w) = retune_plan(self.width(), self.tuned_w.load(Ordering::Acquire), from, ch) {
            return self.tune_full(ch, w);
        }
        self.with_ctl(|c| {
            // Blind the body stamp for the move: a frame picked up during
            // it reads 0 = unknown, not the new channel.
            self.rx_channel.store(0, Ordering::Release);
            let ok = c.fast_retune(ch);
            self.channel.store(ch, Ordering::Release);
            self.rx_channel.store(ch, Ordering::Release);
            ok
        })
        .unwrap_or(false)
    }
    fn retune_width(&self, ch: u8, width_mhz: u8) -> bool {
        self.set_width(ch, width_mhz)
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
        self.ready.load(Ordering::Acquire)
    }
    fn alive(&self) -> bool {
        self.ready()
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
        self.width.store(w, Ordering::Release);
        self.tune_full(ch, w)
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
pub fn rust_channel_core(cfg: Config, opts: RustOpts, width: u8) -> Attach {
    Box::new(move |cards| {
        let dcs: Vec<Arc<DevCard>> = cards
            .iter()
            .enumerate()
            .map(|(i, c)| Arc::new(DevCard::new(c, i, opts.channel, width)))
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
