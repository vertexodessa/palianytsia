//! The uplink TX card (maburgs `gs/src/tx_selector.{h,cpp}`): the card that
//! hears the drone best carries the RCFs, with a 3 dB margin held for 2 s
//! before a switch and an immediate move off a dead card. It compares RSSI,
//! not SNR: a relay's SNR is RSSI plus a constant, while RSSI is honest on
//! every card type.

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CardSnapshot {
    pub alive: bool,
    /// Best-chain RSSI EMA, raw (dBm + 110), as `CardTrack::rf.rssi_ema`.
    pub rssi_ema: f64,
    pub last_frame_us: u64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TxSelectorCfg {
    /// A card index, or -1 = pick by RSSI.
    pub pinned: i32,
    pub switch_margin_db: f64,
    pub hold_ms: u64,
    pub card_dead_ms: u64,
}

impl Default for TxSelectorCfg {
    /// maburgs main.cpp's values.
    fn default() -> Self {
        Self {
            pinned: -1,
            switch_margin_db: 3.0,
            hold_ms: 2000,
            card_dead_ms: 1500,
        }
    }
}

/// `radio.tx_card` when such a card exists, else -1 (auto); maburgs
/// card_scan.h `effective_tx_card`.
pub fn effective_tx_card(configured: i32, n_cards: usize) -> i32 {
    if configured >= 0 && (configured as usize) < n_cards {
        configured
    } else {
        -1
    }
}

#[derive(Clone, Debug)]
pub struct TxSelector {
    cfg: TxSelectorCfg,
    n_cards: usize,
    selected: usize,
    challenger: Option<usize>,
    challenge_since_us: u64,
    switches: u64,
}

impl TxSelector {
    pub fn new(cfg: TxSelectorCfg, n_cards: usize) -> Self {
        let selected = match usize::try_from(cfg.pinned) {
            Ok(p) if p < n_cards => p,
            _ => 0,
        };
        Self {
            cfg,
            n_cards,
            selected,
            challenger: None,
            challenge_since_us: 0,
            switches: 0,
        }
    }

    pub fn selected(&self) -> usize {
        self.selected
    }

    pub fn switches(&self) -> u64 {
        self.switches
    }

    fn dead(&self, c: &CardSnapshot, now_us: u64) -> bool {
        if !c.alive {
            return true;
        }
        // A body drained after the loop-top stamp carries a mono_us past
        // the ms-floored now: a frame from this very tick is alive, never
        // a wrap.
        if c.last_frame_us >= now_us {
            return false;
        }
        now_us - c.last_frame_us > self.cfg.card_dead_ms * 1000
    }

    /// The strongest live card; all dead = keep the current one (it
    /// transmits anyway).
    fn best_alive(&self, cards: &[CardSnapshot], now_us: u64) -> usize {
        let mut best: Option<usize> = None;
        for (i, c) in cards.iter().enumerate().take(self.n_cards) {
            if self.dead(c, now_us) {
                continue;
            }
            if best.is_none_or(|b| c.rssi_ema > cards[b].rssi_ema) {
                best = Some(i);
            }
        }
        best.unwrap_or(self.selected)
    }

    pub fn update(&mut self, cards: &[CardSnapshot], now_us: u64) -> usize {
        if let Ok(p) = usize::try_from(self.cfg.pinned) {
            if p < cards.len() && !self.dead(&cards[p], now_us) {
                self.selected = p;
                return p;
            }
            // The pinned card is dead: pick by RSSI until it returns.
        }
        let cur = cards[self.selected];
        if self.dead(&cur, now_us) {
            let next = self.best_alive(cards, now_us);
            if next != self.selected {
                self.selected = next;
                self.switches += 1;
            }
            self.challenger = None;
            return self.selected;
        }
        let cand = self.best_alive(cards, now_us);
        if cand != self.selected && cards[cand].rssi_ema >= cur.rssi_ema + self.cfg.switch_margin_db
        {
            if self.challenger != Some(cand) {
                self.challenger = Some(cand);
                self.challenge_since_us = now_us;
            } else if now_us - self.challenge_since_us >= self.cfg.hold_ms * 1000 {
                self.selected = cand;
                self.switches += 1;
                self.challenger = None;
            }
        } else {
            self.challenger = None;
        }
        self.selected
    }
}

#[cfg(test)]
mod tests {
    //! mabur tests/test_tx_selector.cpp, case for case.
    use super::*;

    /// Raw RSSI = dBm + 110: 50 = -60 dBm.
    fn snap(rssi_raw: f64, last_us: u64) -> CardSnapshot {
        CardSnapshot {
            alive: true,
            rssi_ema: rssi_raw,
            last_frame_us: last_us,
        }
    }

    fn sel(pinned: i32) -> TxSelector {
        TxSelector::new(
            TxSelectorCfg {
                pinned,
                ..TxSelectorCfg::default()
            },
            2,
        )
    }

    fn stamp(cards: &mut [CardSnapshot], t: u64) {
        for c in cards {
            c.last_frame_us = t;
        }
    }

    #[test]
    fn hysteresis_switch_needs_sustained_margin() {
        let mut s = sel(-1);
        let mut cards = [snap(50.0, 0), snap(51.0, 0)];
        assert_eq!(s.update(&cards, 0), 0);
        for t in (0..5_000_000).step_by(100_000) {
            stamp(&mut cards, t);
            assert_eq!(s.update(&cards, t), 0); // +1 dB: never
        }
        cards[1].rssi_ema = 54.0; // +4 dB
        let t = 5_000_000;
        assert_eq!(s.update(&cards, t), 0);
        stamp(&mut cards, t + 1_000_000);
        assert_eq!(s.update(&cards, t + 1_000_000), 0);
        stamp(&mut cards, t + 2_100_000);
        assert_eq!(s.update(&cards, t + 2_100_000), 1);
        assert_eq!(s.switches(), 1);
    }

    #[test]
    fn equal_rssi_never_switches() {
        let mut s = sel(-1);
        let mut cards = [snap(48.0, 0), snap(48.0, 0)];
        for t in (0..10_000_000).step_by(100_000) {
            stamp(&mut cards, t);
            // Jitter inside the margin.
            cards[1].rssi_ema = 48.0 + if (t / 100_000) % 2 == 1 { 2.9 } else { -2.9 };
            assert_eq!(s.update(&cards, t), 0);
        }
        assert_eq!(s.switches(), 0);
    }

    #[test]
    fn challenge_resets_if_margin_drops() {
        let mut s = sel(-1);
        let mut cards = [snap(50.0, 0), snap(54.0, 0)];
        s.update(&cards, 0);
        cards[1].rssi_ema = 51.0;
        stamp(&mut cards, 1_000_000);
        assert_eq!(s.update(&cards, 1_000_000), 0);
        cards[1].rssi_ema = 54.0;
        stamp(&mut cards, 2_000_000);
        assert_eq!(s.update(&cards, 2_000_000), 0);
        stamp(&mut cards, 3_500_000);
        assert_eq!(s.update(&cards, 3_500_000), 0);
    }

    #[test]
    fn dead_card_switches_immediately() {
        let mut s = sel(-1);
        let mut cards = [snap(55.0, 0), snap(50.0, 0)];
        assert_eq!(s.update(&cards, 0), 0);
        cards[1].last_frame_us = 2_000_000;
        assert_eq!(s.update(&cards, 2_000_000), 1);
        cards[0].alive = false;
        assert_eq!(s.update(&cards, 2_100_000), 1);
    }

    #[test]
    fn pin_overrides_unless_dead() {
        let mut s = sel(1);
        let mut cards = [snap(60.0, 0), snap(40.0, 0)];
        assert_eq!(s.update(&cards, 0), 1);
        cards[1].alive = false;
        assert_eq!(s.update(&cards, 100_000), 0);
    }

    #[test]
    fn relay_with_high_synthetic_snr_but_lower_rssi_loses() {
        let mut s = sel(-1);
        // -55 dBm USB vs -65 dBm relay.
        let mut cards = [snap(55.0, 0), snap(45.0, 0)];
        for t in (0..5_000_000).step_by(100_000) {
            stamp(&mut cards, t);
            assert_eq!(s.update(&cards, t), 0);
        }
    }

    #[test]
    fn future_frame_stamp_is_not_dead() {
        let mut s = sel(-1);
        let mut cards = [snap(50.0, 0), snap(50.0, 0)];
        for now in (1_000_000..3_000_000).step_by(100_000) {
            cards[0].last_frame_us = now + 500;
            cards[1].last_frame_us = now;
            assert_eq!(s.update(&cards, now), 0);
        }
        assert_eq!(s.switches(), 0);
    }

    #[test]
    fn effective_tx_card_falls_back_to_auto() {
        assert_eq!(effective_tx_card(1, 2), 1);
        assert_eq!(effective_tx_card(2, 2), -1);
        assert_eq!(effective_tx_card(-1, 2), -1);
    }
}
