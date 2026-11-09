//! Display refresh rate for a video frame rate (spec 5.6 "Frame rate").
//! Host-built so its tests run with the workspace's.
use std::time::{Duration, Instant};

/// Camera modes the drone runs (VENC_MODE presets), lowest first.
const NOMINAL: &[f64] = &[30.0, 60.0, 90.0, 100.0, 120.0];

/// The display rate for a measured video fps, or None while it is unknown.
///
/// The measured rate counts decoded frames, so loss only lowers it: it
/// rounds up to the camera mode it came from. 5 % above a mode still counts
/// as that mode: a window after a stall decodes its backlog late. A mode the display divides
/// evenly gets that rate (120 Hz for 120 or 60, 90 Hz for 90 or 30). 100 fps
/// divides none of 72/80/90/120: 120 Hz shows every frame (one in five
/// twice) on an 8.3 ms period, where 90 Hz drops one frame in ten; it got
/// 90 Hz until 2026-11-03, a decoded 99.x fps falling under a `> 100` band.
///
/// A runtime that offers rates above 120 Hz (`debug.oculus.allRefreshRates`
/// set over adb) gets its highest: the compositor latch wait and the panel's
/// scan-to-backlight time both shrink with the period. Measured on the
/// Quest 3 with 120 fps video, codec input to backlight flash p50 18.75 ms
/// at 120 Hz, 13.53 ms at 207 Hz; frames replaced before the latch 17 % ->
/// 3.2 %.
pub fn rate_for(fps: f64, offered: &[f32]) -> Option<f32> {
    if fps <= MIN_FPS {
        return None;
    }
    let top = offered.iter().copied().reduce(f32::max)?;
    if top > STOCK_MAX_HZ + 0.5 {
        return Some(top);
    }
    let nominal = NOMINAL
        .iter()
        .copied()
        .find(|n| fps <= n * 1.05)
        .unwrap_or(120.0);
    let want = match nominal as u32 {
        30 | 90 => 90.0,
        _ => 120.0,
    };
    offered
        .iter()
        .copied()
        .find(|r| (r - want).abs() < 1.0)
        .or_else(|| offered.iter().copied().reduce(f32::max))
}

/// The highest rate a stock runtime offers.
pub const STOCK_MAX_HZ: f32 = 120.0;

/// At or under this a window is an outage, not a frame rate.
const MIN_FPS: f64 = 20.0;

/// Windows the estimate spans (2 s each in xr).
const WINDOWS: usize = 5;

/// The video fps as the best of the last few windows: loss only lowers a
/// window's count, so the maximum is the camera mode's rate. One window
/// alone flapped the display 90/120 Hz every 2 s on a 100 fps link losing
/// a few percent (decoded 89-99 fps, 2026-11-03).
#[derive(Default)]
pub struct FpsEstimate {
    recent: std::collections::VecDeque<f64>,
}

impl FpsEstimate {
    /// A window with no video to speak of (an outage) says nothing about
    /// the camera mode: it keeps the windows from before, so the display
    /// does not drop to 90 Hz for the seconds video takes to come back.
    pub fn push(&mut self, fps: f64) -> f64 {
        if fps <= MIN_FPS {
            return self.recent.iter().copied().fold(0.0, f64::max);
        }
        if self.recent.len() == WINDOWS {
            self.recent.pop_front();
        }
        self.recent.push_back(fps);
        self.recent.iter().copied().fold(0.0, f64::max)
    }
}

/// How often the display is matched to the video.
const REFRESH_PERIOD: Duration = Duration::from_secs(2);

/// The refresh-rate policy of the frame loop: every 2 s the camera rate from
/// the AU stamps, else (stamps not in yet) the decoded fps, and a request
/// only when the rate moved.
pub struct Refresh {
    current: f32,
    last_t: Instant,
    last_frames: u64,
    est: FpsEstimate,
}

impl Refresh {
    pub fn new(now: Instant) -> Self {
        Self {
            current: 0.0,
            last_t: now,
            last_frames: 0,
            est: FpsEstimate::default(),
        }
    }

    pub fn due(&self, now: Instant) -> bool {
        now.duration_since(self.last_t) >= REFRESH_PERIOD
    }

    /// The (rate, video fps) to request, if any. A link restart resets
    /// `frames`, so the delta saturates.
    pub fn tick(
        &mut self,
        now: Instant,
        frames: u64,
        cam_x10: u32,
        offered: &[f32],
    ) -> Option<(f32, f64)> {
        let dt = now.duration_since(self.last_t).as_secs_f64();
        let decoded = self
            .est
            .push(frames.saturating_sub(self.last_frames) as f64 / dt);
        self.last_t = now;
        self.last_frames = frames;
        let fps = if cam_x10 > 0 {
            f64::from(cam_x10) / 10.0
        } else {
            decoded
        };
        let r = rate_for(fps, offered)?;
        if (r - self.current).abs() <= 0.5 {
            return None;
        }
        self.current = r;
        Some((r, fps))
    }
}

#[cfg(test)]
mod tests {
    use super::{rate_for, FpsEstimate, Refresh};
    use std::time::{Duration, Instant};

    const QUEST3: &[f32] = &[72.0, 80.0, 90.0, 120.0];

    fn every_2s(r: &mut Refresh, t: &mut Instant) -> Instant {
        *t += Duration::from_secs(2);
        assert!(r.due(*t));
        *t
    }

    #[test]
    fn refresh_prefers_the_camera_rate_over_decoded_frames() {
        let mut t = Instant::now();
        let mut r = Refresh::new(t);
        let now = every_2s(&mut r, &mut t);
        // 180 decoded in 2 s reads 90 fps; the stamps say 120.
        assert_eq!(r.tick(now, 180, 1200, QUEST3), Some((120.0, 120.0)));
    }

    #[test]
    fn refresh_falls_back_to_decoded_frames() {
        let mut t = Instant::now();
        let mut r = Refresh::new(t);
        let now = every_2s(&mut r, &mut t);
        assert_eq!(r.tick(now, 180, 0, QUEST3), Some((90.0, 90.0)));
    }

    #[test]
    fn refresh_requests_only_a_change() {
        let mut t = Instant::now();
        let mut r = Refresh::new(t);
        let now = every_2s(&mut r, &mut t);
        assert!(r.tick(now, 0, 1200, QUEST3).is_some());
        let now = every_2s(&mut r, &mut t);
        assert_eq!(r.tick(now, 0, 1200, QUEST3), None);
        let now = every_2s(&mut r, &mut t);
        assert_eq!(r.tick(now, 0, 900, QUEST3), Some((90.0, 90.0)));
    }

    #[test]
    fn refresh_survives_the_counter_reset_of_a_link_restart() {
        let mut t = Instant::now();
        let mut r = Refresh::new(t);
        let now = every_2s(&mut r, &mut t);
        assert_eq!(r.tick(now, 180, 0, QUEST3), Some((90.0, 90.0)));
        // The counter restarts at 0: no huge delta, the outage keeps the mode.
        let now = every_2s(&mut r, &mut t);
        assert_eq!(r.tick(now, 10, 0, QUEST3), None);
    }

    #[test]
    fn refresh_waits_for_its_period() {
        let t = Instant::now();
        let r = Refresh::new(t);
        assert!(!r.due(t + Duration::from_millis(1999)));
        assert!(r.due(t + Duration::from_secs(2)));
    }

    #[test]
    fn spec_modes() {
        assert_eq!(rate_for(120.0, QUEST3), Some(120.0));
        assert_eq!(rate_for(60.0, QUEST3), Some(120.0));
        assert_eq!(rate_for(90.0, QUEST3), Some(90.0));
        assert_eq!(rate_for(30.0, QUEST3), Some(90.0));
    }

    #[test]
    fn hundred_fps_gets_120_even_measured_just_under() {
        assert_eq!(rate_for(100.0, QUEST3), Some(120.0));
        assert_eq!(rate_for(99.4, QUEST3), Some(120.0));
        assert_eq!(rate_for(101.5, QUEST3), Some(120.0));
    }

    #[test]
    fn loss_lowers_the_count_not_the_mode() {
        // 90 fps with 12 % loss still picks 90 Hz, not 80.
        assert_eq!(rate_for(79.0, QUEST3), Some(90.0));
        assert_eq!(rate_for(57.0, QUEST3), Some(120.0));
    }

    #[test]
    fn unknown_or_missing() {
        assert_eq!(rate_for(0.0, QUEST3), None);
        assert_eq!(rate_for(15.0, QUEST3), None);
        assert_eq!(rate_for(90.0, &[72.0, 120.0]), Some(120.0));
        assert_eq!(rate_for(90.0, &[]), None);
    }

    #[test]
    fn unlocked_rates_take_the_highest_for_any_video() {
        let all: Vec<f32> = (72..=207).map(|r| r as f32).collect();
        assert_eq!(rate_for(120.0, &all), Some(207.0));
        assert_eq!(rate_for(30.0, &all), Some(207.0));
        assert_eq!(rate_for(15.0, &all), None);
    }

    #[test]
    fn a_lossy_100_fps_link_holds_120_hz() {
        let mut e = FpsEstimate::default();
        let seen: Vec<_> = [99.0, 91.0, 93.0, 89.0, 96.0, 63.0, 91.0, 99.0]
            .iter()
            .map(|f| rate_for(e.push(*f), QUEST3))
            .collect();
        assert!(seen.iter().all(|r| *r == Some(120.0)), "{seen:?}");
    }

    #[test]
    fn a_mode_change_down_follows_within_five_windows() {
        let mut e = FpsEstimate::default();
        for _ in 0..5 {
            e.push(100.0);
        }
        let r: Vec<_> = (0..5).map(|_| rate_for(e.push(90.0), QUEST3)).collect();
        assert_eq!(r[3], Some(120.0));
        assert_eq!(r[4], Some(90.0));
    }

    #[test]
    fn an_outage_keeps_the_mode() {
        let mut e = FpsEstimate::default();
        for _ in 0..5 {
            e.push(99.0);
        }
        let r: Vec<_> = [0.0, 0.0, 0.0, 12.0, 88.0, 95.0]
            .iter()
            .map(|f| rate_for(e.push(*f), QUEST3))
            .collect();
        assert!(r.iter().all(|x| *x == Some(120.0)), "{r:?}");
    }
}
