//! The camera's frame rate from the capture stamps of the AUs that arrive.
//!
//! Counting decoded frames reads a lossy link as a slower camera: a 100 fps
//! link losing ~10 % decoded 89-99 fps per 2 s and put the Quest on 90 Hz
//! (2026-11-03). A lost AU only widens one stamp step to two periods, so the
//! low end of the steps is the period whatever the loss.

use std::collections::VecDeque;

/// Steps the estimate spans (~0.6 s at 100 fps).
const STEPS: usize = 64;
/// Recompute after this many new steps; sorting 64 per AU buys nothing.
const EVERY: usize = 16;
/// A step this long is an outage or a restart, not a frame period.
const MAX_STEP_US: u32 = 200_000;

#[derive(Default)]
pub struct CamFps {
    last: Option<u32>,
    steps: VecDeque<u32>,
    fresh: usize,
}

impl CamFps {
    /// Feeds one AU's capture stamp (`Au::pts_us`, wraps at u32); returns a
    /// new estimate in fps once enough steps are in, else None.
    pub fn push(&mut self, pts_us: u32) -> Option<f64> {
        let prev = self.last.replace(pts_us);
        let step = pts_us.wrapping_sub(prev?);
        // 0: slices of one frame; past MAX_STEP_US (or "negative", i.e. a
        // huge wrapped value): reorder, outage, drone restart.
        if step == 0 || step >= MAX_STEP_US {
            return None;
        }
        if self.steps.len() == STEPS {
            self.steps.pop_front();
        }
        self.steps.push_back(step);
        self.fresh += 1;
        if self.steps.len() < STEPS || self.fresh < EVERY {
            return None;
        }
        self.fresh = 0;
        let mut v: Vec<u32> = self.steps.iter().copied().collect();
        v.sort_unstable();
        // 10th percentile: loss only adds long steps, and stamp jitter of a
        // few short ones should not set the rate either.
        let p = v[STEPS / 10];
        Some(1e6 / p as f64)
    }
}

#[cfg(test)]
mod tests {
    use super::CamFps;

    fn run(stamps: impl Iterator<Item = u32>) -> Option<f64> {
        let mut c = CamFps::default();
        stamps.filter_map(|s| c.push(s)).last()
    }

    #[test]
    fn steady_100_fps() {
        let f = run((0..200u32).map(|i| i * 10_000)).unwrap();
        assert!((f - 100.0).abs() < 0.01, "{f}");
    }

    #[test]
    fn thirty_percent_loss_still_reads_the_camera() {
        // Every third and every seventh frame lost.
        let f = run((0..400u32)
            .filter(|i| i % 3 != 0 && i % 7 != 0)
            .map(|i| i * 11_111))
        .unwrap();
        assert!((f - 90.0).abs() < 0.1, "{f}");
    }

    #[test]
    fn wraps_and_ignores_slices_and_gaps() {
        let base = u32::MAX - 500_000;
        let stamps = (0..200u32).flat_map(|i| {
            let t = base.wrapping_add(i * 8_333);
            // Two slices per frame, and a 3 s outage half way.
            let t = if i >= 100 {
                t.wrapping_add(3_000_000)
            } else {
                t
            };
            [t, t]
        });
        let f = run(stamps).unwrap();
        assert!((f - 120.0).abs() < 0.1, "{f}");
    }

    #[test]
    fn nothing_before_a_full_window() {
        assert_eq!(run((0..60u32).map(|i| i * 10_000)), None);
    }
}
