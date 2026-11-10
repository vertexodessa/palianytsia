//! The age of the picture on the glass. When video stops, the decoder's
//! last frame stays up and looks live; a pilot flying on it reacts late.
//! Every front end feeds the decoder's running frame count here once per
//! OSD tick, so the stall marker is one rule on all three.

/// Above this age the picture is late: a frame-age counter appears. At
/// 90 fps it is 13 frame intervals. Both thresholds are a first guess for
/// the bench to settle.
pub const LATE_S: f64 = 0.15;
/// Above this age the picture is called dead: "NO VIDEO" and a red frame.
pub const NO_VIDEO_S: f64 = 0.5;

/// How stale the shown picture is, with its age in seconds.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum Stall {
    #[default]
    None,
    Late(f64),
    NoVideo(f64),
}

impl Stall {
    /// The classification without the age, for a redraw decision that
    /// must react to a change of state at once.
    pub fn same_kind(&self, o: &Stall) -> bool {
        std::mem::discriminant(self) == std::mem::discriminant(o)
    }
}

/// When the decoder's frame count last moved.
#[derive(Clone, Copy, Debug, Default)]
pub struct FrameAge {
    frames: u64,
    /// None until the first frame ever: before video has flowed once there
    /// is no frozen picture to warn about.
    since: Option<f64>,
}

impl FrameAge {
    /// The decoder's running count at `now_s`. A lower count is a new link
    /// restarting its counter, not a new frame: the picture on the glass
    /// is still the old one, so its age keeps running.
    pub fn note(&mut self, now_s: f64, frames: u64) {
        if frames > self.frames {
            self.since = Some(now_s);
        }
        self.frames = frames;
    }

    pub fn stall(&self, now_s: f64) -> Stall {
        let Some(t) = self.since else {
            return Stall::None;
        };
        let age = (now_s - t).max(0.0);
        if age > NO_VIDEO_S {
            Stall::NoVideo(age)
        } else if age > LATE_S {
            Stall::Late(age)
        } else {
            Stall::None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nothing_before_the_first_frame() {
        let mut a = FrameAge::default();
        a.note(0.0, 0);
        assert_eq!(a.stall(10.0), Stall::None);
    }

    #[test]
    fn thresholds() {
        let mut a = FrameAge::default();
        a.note(1.0, 5);
        assert_eq!(a.stall(1.0), Stall::None);
        assert_eq!(a.stall(1.1), Stall::None);
        assert!(matches!(a.stall(1.2), Stall::Late(t) if (t - 0.2).abs() < 1e-9));
        assert!(
            matches!(a.stall(1.5), Stall::Late(_)),
            "0.5 s is still late"
        );
        assert!(matches!(a.stall(2.25), Stall::NoVideo(t) if (t - 1.25).abs() < 1e-9));
    }

    #[test]
    fn a_new_frame_clears_and_the_same_count_does_not() {
        let mut a = FrameAge::default();
        a.note(0.0, 5);
        a.note(1.0, 5);
        assert!(matches!(a.stall(1.0), Stall::NoVideo(_)));
        a.note(1.0, 6);
        assert_eq!(a.stall(1.0), Stall::None);
    }

    #[test]
    fn a_restarted_count_is_not_a_frame() {
        let mut a = FrameAge::default();
        a.note(0.0, 900);
        a.note(1.0, 0);
        assert!(matches!(a.stall(1.0), Stall::NoVideo(_)));
        // The new link's first frame clears it.
        a.note(2.0, 1);
        assert_eq!(a.stall(2.0), Stall::None);
    }

    #[test]
    fn same_kind_ignores_the_age() {
        assert!(Stall::Late(0.2).same_kind(&Stall::Late(0.4)));
        assert!(!Stall::Late(0.2).same_kind(&Stall::NoVideo(0.6)));
        assert!(!Stall::None.same_kind(&Stall::Late(0.2)));
    }
}
