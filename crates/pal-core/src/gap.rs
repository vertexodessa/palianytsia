//! Rate-aware FrameStream gap timeout (mabur `gs/src/gap_timeout_policy.h`):
//! timeout = clamp(w / send_rate + margin, floor, cap). A lost symbol is
//! repairable while the TX window still covers it, which at low rungs lasts
//! longer than the fixed 50 ms. cap 0 disables the stretch.

const HALF_LIFE_S: f64 = 2.0;
const MIN_RATE: f64 = 0.5;
const MARGIN_MS: f64 = 15.0;
const DEFAULT_WINDOW: usize = 32;

#[derive(Clone, Copy, Debug)]
struct S {
    seeded: bool,
    have_rate: bool,
    last_seq: u64,
    last_ms: u64,
    rate: f64,
    window: usize,
}

pub struct GapTimeoutPolicy {
    floor_ms: u64,
    cap_ms: u64,
    s: [S; 2],
}

impl GapTimeoutPolicy {
    pub fn new(floor_ms: u64, cap_ms: u64) -> Self {
        let s = S {
            seeded: false,
            have_rate: false,
            last_seq: 0,
            last_ms: 0,
            rate: 0.0,
            window: DEFAULT_WINDOW,
        };
        Self {
            floor_ms,
            cap_ms,
            s: [s; 2],
        }
    }

    /// ~1 Hz per sid with the decoder's newest virtual seq and repair window.
    pub fn update(&mut self, sid: usize, newest_seq: u64, repair_window: usize, now_ms: u64) {
        let Some(s) = self.s.get_mut(sid) else { return };
        if repair_window > 0 {
            s.window = repair_window;
        }
        if !s.seeded || newest_seq < s.last_seq {
            s.seeded = true;
            s.last_seq = newest_seq;
            s.last_ms = now_ms;
            return;
        }
        if now_ms <= s.last_ms {
            return;
        }
        let dt_s = (now_ms - s.last_ms) as f64 / 1000.0;
        let inst = (newest_seq - s.last_seq) as f64 / dt_s;
        let alpha = 1.0 - 0.5f64.powf(dt_s / HALF_LIFE_S);
        s.rate += alpha * (inst - s.rate);
        s.have_rate = true;
        s.last_seq = newest_seq;
        s.last_ms = now_ms;
    }

    /// Microseconds one symbol takes at the estimated send rate; 0 while the
    /// rate is unknown or near silence, or with the stretch disabled, where
    /// `timeout_ms` already decides alone.
    pub fn us_per_symbol(&self, sid: usize) -> u32 {
        match self.s.get(sid) {
            Some(s) if self.cap_ms != 0 && s.have_rate && s.rate >= MIN_RATE => {
                (1e6 / s.rate) as u32
            }
            _ => 0,
        }
    }
    pub fn cap_ms(&self) -> u64 {
        self.cap_ms
    }
    pub const fn margin_ms() -> u32 {
        MARGIN_MS as u32
    }

    pub fn timeout_ms(&self, sid: usize) -> u64 {
        let Some(s) = self.s.get(sid) else {
            return self.floor_ms;
        };
        if self.cap_ms == 0 || !s.have_rate {
            return self.floor_ms;
        }
        if s.rate < MIN_RATE {
            return self.cap_ms;
        }
        let span = 1000.0 * s.window as f64 / s.rate + MARGIN_MS;
        // C++ truncates to int, then clamps.
        (span as i64).clamp(self.floor_ms as i64, self.cap_ms as i64) as u64
    }
}

/// A drone (maburd) restart seen as a large backwards step of T_TELEM's
/// tlm_seq (mabur `gs/src/drone_restart.h`). Never fires on the first
/// sample; tolerates reorder up to 100; a 16-bit wrap is forward; holds off
/// 10 s after a fire so old-run stragglers do not split one restart.
#[derive(Default)]
pub struct DroneRestartDetector {
    have: bool,
    last: u16,
    fired: bool,
    fired_ms: f64,
}

impl DroneRestartDetector {
    pub const TOLERANCE: u32 = 100;
    pub const HOLDOFF_MS: f64 = 10000.0;

    pub fn on_telem(&mut self, tlm_seq: u16, now_ms: f64) -> bool {
        if !self.have {
            self.have = true;
            self.last = tlm_seq;
            return false;
        }
        let fwd = u32::from(tlm_seq.wrapping_sub(self.last));
        self.last = tlm_seq;
        let back = if fwd == 0 { 0 } else { 65536 - fwd };
        if fwd <= 32768 || back <= Self::TOLERANCE {
            return false;
        }
        if self.fired && now_ms - self.fired_ms < Self::HOLDOFF_MS {
            return false;
        }
        self.fired = true;
        self.fired_ms = now_ms;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn restart_rules() {
        let mut d = DroneRestartDetector::default();
        assert!(!d.on_telem(5000, 0.0));
        assert!(!d.on_telem(4950, 10.0)); // reorder
        assert!(!d.on_telem(5100, 20.0));
        assert!(d.on_telem(3, 30.0));
        assert!(!d.on_telem(65000, 40.0)); // stragglers inside holdoff... climbing
        let mut w = DroneRestartDetector::default();
        w.on_telem(65530, 0.0);
        assert!(!w.on_telem(2, 1.0)); // wrap is forward
    }

    #[test]
    fn gap_policy() {
        let mut g = GapTimeoutPolicy::new(50, 150);
        assert_eq!(g.timeout_ms(0), 50);
        g.update(0, 0, 32, 0);
        g.update(0, 1000, 32, 1000);
        // 1000 syms/s * alpha(0.29) = 292.9/s: 32/292.9 s + 15 ms = 124 ms
        assert_eq!(g.timeout_ms(0), 124);
        g.update(0, 1000, 32, 30000);
        assert_eq!(g.timeout_ms(0), 150);
    }

    #[test]
    fn us_per_symbol_follows_the_send_rate() {
        let mut g = GapTimeoutPolicy::new(50, 150);
        assert_eq!(g.us_per_symbol(0), 0); // no rate yet
        g.update(0, 0, 64, 0);
        g.update(0, 1000, 64, 1000);
        // 292.9 syms/s, the rate of the gap_policy test above
        assert_eq!(g.us_per_symbol(0), 3414);
        assert_eq!(g.us_per_symbol(1), 0); // sids are independent
        assert_eq!(g.us_per_symbol(2), 0); // out of range
        g.update(0, 1000, 64, 30000); // silence: the rate falls under MIN_RATE
        assert_eq!(g.us_per_symbol(0), 0);

        let mut off = GapTimeoutPolicy::new(50, 0);
        off.update(0, 0, 64, 0);
        off.update(0, 1000, 64, 1000);
        assert_eq!(off.us_per_symbol(0), 0); // stretch disabled
    }
}
