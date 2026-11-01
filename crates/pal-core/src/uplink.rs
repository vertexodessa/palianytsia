//! GS-side readouts from T_TELEM: control-loop RTT and the drone/GS pts
//! clock offset (mabur `gs/src/rtt_estimator.cpp`), and the share of our
//! RCFs the drone heard (`gs/src/uplink_delivery.cpp`).

const RING_CAP: usize = 64;
const OFF_WINDOW: usize = 32;
const NEG_CLAMP_US: i64 = -1000;
const MAX_PLAUSIBLE_US: i64 = 3_000_000;
const MAX_AGE_MS: u16 = 1000;
const EWMA_ALPHA: f64 = 0.2;

#[derive(Clone, Copy, Default)]
struct Sent {
    seq: u16,
    t_us: u64,
    used: bool,
}

pub struct RttEstimator {
    ring: [Sent; RING_CAP],
    ring_next: usize,
    rtt_ewma_us: f64,
    rtt_min_us: f64,
    n: u32,
    off_win: [(i64, i64); OFF_WINDOW],
    off_next: usize,
    off_n: usize,
}

impl Default for RttEstimator {
    fn default() -> Self {
        Self {
            ring: [Sent::default(); RING_CAP],
            ring_next: 0,
            rtt_ewma_us: 0.0,
            rtt_min_us: 0.0,
            n: 0,
            off_win: [(0, 0); OFF_WINDOW],
            off_next: 0,
            off_n: 0,
        }
    }
}

impl RttEstimator {
    pub fn on_rcf_sent(&mut self, seq: u16, t_us: u64) {
        self.ring[self.ring_next] = Sent {
            seq,
            t_us,
            used: true,
        };
        self.ring_next = (self.ring_next + 1) % RING_CAP;
    }
    pub fn on_telem(
        &mut self,
        seq_echo: u16,
        echo_valid: bool,
        age_ms: u16,
        pts_at_build_us: u64,
        rx_us: u64,
    ) -> bool {
        if !echo_valid || age_ms >= MAX_AGE_MS {
            return false;
        }
        let Some(hit) = self.ring.iter().find(|s| s.used && s.seq == seq_echo) else {
            return false;
        };
        if rx_us < hit.t_us {
            return false;
        }
        let mut rtt = (rx_us - hit.t_us) as i64 - i64::from(age_ms) * 1000;
        if !(NEG_CLAMP_US..=MAX_PLAUSIBLE_US).contains(&rtt) {
            return false;
        }
        rtt = rtt.max(0);
        let r = rtt as f64;
        if self.n == 0 {
            self.rtt_ewma_us = r;
            self.rtt_min_us = r;
        } else {
            self.rtt_ewma_us += EWMA_ALPHA * (r - self.rtt_ewma_us);
            self.rtt_min_us = self.rtt_min_us.min(r);
        }
        self.n += 1;
        if pts_at_build_us != 0 {
            let off = pts_at_build_us as i64 - rx_us as i64 + rtt / 2;
            self.off_win[self.off_next] = (rtt, off);
            self.off_next = (self.off_next + 1) % OFF_WINDOW;
            self.off_n = (self.off_n + 1).min(OFF_WINDOW);
        }
        true
    }
    pub fn has_rtt(&self) -> bool {
        self.n > 0
    }
    pub fn rtt_ms(&self) -> f64 {
        self.rtt_ewma_us / 1000.0
    }
    pub fn rtt_min_ms(&self) -> f64 {
        self.rtt_min_us / 1000.0
    }
    pub fn has_offset(&self) -> bool {
        self.off_n > 0
    }
    /// Offset of the min-RTT sample in the window.
    pub fn pts_off_us(&self) -> i64 {
        let mut best = 0;
        for i in 1..self.off_n {
            if self.off_win[i].0 < self.off_win[best].0 {
                best = i;
            }
        }
        self.off_win[best].1
    }
}

/// Capture-to-complete latency: the completion time mapped into the
/// drone's u32 pts clock, minus the AU's pts (web_gs.cpp).
pub fn cap_to_complete_us(pts32: u32, t_complete_us: u64, pts_off_us: i64) -> i64 {
    let cap_in_pts = t_complete_us.wrapping_add(pts_off_us as u64) as u32;
    i64::from(cap_in_pts.wrapping_sub(pts32) as i32)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UplinkState {
    None,
    Wait,
    Ok,
    Low,
    Dead,
    Idle,
    Stale,
}

impl UplinkState {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::Wait => "wait",
            Self::Ok => "ok",
            Self::Low => "low",
            Self::Dead => "dead",
            Self::Idle => "idle",
            Self::Stale => "stale",
        }
    }
}

const WINDOW_MS: u64 = 10000;
const MIN_SPAN_MS: u64 = 9000;
const STALE_MS: u64 = 3000;
const IDLE_MS: u64 = 1000;
const DEAD_GRACE_MS: u64 = 2000;
const LOW_BELOW_PCT: i32 = 80;
const MIN_RCFS_IN_WINDOW: u64 = 20;
const STATE_RENDEZVOUS: u8 = 1;
const STATE_FAILSAFE: u8 = 3;
const CAP: usize = 16;

#[derive(Default)]
pub struct UplinkDelivery {
    ring: std::collections::VecDeque<(u64, u32, u64)>,
    sent: u64,
    have_sent: bool,
    last_sent_ms: u64,
    run_start_ms: u64,
    have_telem: bool,
    last_seq: u16,
    last_rx: u32,
    drone_state: u8,
    last_telem_ms: u64,
    pct: Option<i32>,
}

impl UplinkDelivery {
    pub fn on_rcf_sent(&mut self, now: u64) {
        if !self.have_sent || now - self.last_sent_ms > IDLE_MS {
            self.run_start_ms = now;
        }
        self.have_sent = true;
        self.last_sent_ms = now;
        self.sent += 1;
    }
    pub fn on_telem(&mut self, tlm_seq: u16, rcf_rx: u32, drone_state: u8, now: u64) {
        if self.have_telem && tlm_seq == self.last_seq {
            return;
        }
        if self.have_telem && (rcf_rx < self.last_rx || tlm_seq.wrapping_sub(self.last_seq) > 32767)
        {
            self.ring.clear();
            self.pct = None;
        }
        self.have_telem = true;
        self.last_seq = tlm_seq;
        self.last_rx = rcf_rx;
        self.drone_state = drone_state;
        self.last_telem_ms = now;
        if self.ring.len() == CAP {
            self.ring.pop_front();
        }
        self.ring.push_back((now, rcf_rx, self.sent));
        while self.ring.len() > 1 && self.ring[0].0 + WINDOW_MS + 500 < now {
            self.ring.pop_front();
        }
        if self.ring.len() == 1 {
            self.pct = None;
            return;
        }
        let base = self.ring[0];
        let cur = *self.ring.back().expect("cur");
        if cur.0 - base.0 < MIN_SPAN_MS {
            return;
        }
        let d_sent = cur.2 - base.2;
        if d_sent < MIN_RCFS_IN_WINDOW {
            self.pct = None;
            return;
        }
        let ratio = f64::from(cur.1.wrapping_sub(base.1)) / d_sent as f64;
        self.pct = Some(((100.0 * ratio).round() as i32).clamp(0, 100));
    }
    pub fn view(&self, now: u64) -> (UplinkState, Option<i32>) {
        let s = if !self.have_telem {
            UplinkState::None
        } else if now > self.last_telem_ms && now - self.last_telem_ms > STALE_MS {
            UplinkState::Stale
        } else if !self.have_sent || (now > self.last_sent_ms && now - self.last_sent_ms > IDLE_MS)
        {
            UplinkState::Idle
        } else if ((self.drone_state == STATE_RENDEZVOUS || self.drone_state == STATE_FAILSAFE)
            && self.last_telem_ms >= self.run_start_ms + DEAD_GRACE_MS)
            || self.pct == Some(0)
        {
            UplinkState::Dead
        } else {
            match self.pct {
                None => UplinkState::Wait,
                Some(p) if p < LOW_BELOW_PCT => UplinkState::Low,
                Some(_) => UplinkState::Ok,
            }
        };
        (s, self.pct)
    }
}
