//! Touch controllers through OpenXR actions (spec 5.6 "Input"), as facts
//! about the device and nothing else: where the right controller aims (in
//! VIEW and in LOCAL), the trigger edge, the button edges, both thumbstick
//! Y values, the grip. What the ray points at and who the sticks serve is
//! decided by the frame loop (`placement::route`, `placement::sticks`).
//! Bindings: trigger clicks, B or Y opens the settings menu, A or X toggles
//! link statistics, right grip carries the window in window mode.
use openxr as xr;

use crate::placement::Pose;
use crate::placement::{rotate, Ray, Rays};

/// `space` located in `base` at `t`, when the tracker knows it.
pub fn locate(space: &xr::Space, base: &xr::Space, t: xr::Time) -> Option<Pose> {
    let loc = space.locate(base, t).ok()?;
    let ok = xr::SpaceLocationFlags::POSITION_VALID | xr::SpaceLocationFlags::ORIENTATION_VALID;
    if !loc.location_flags.contains(ok) {
        return None;
    }
    let (p, q) = (loc.pose.position, loc.pose.orientation);
    Some(Pose {
        pos: [p.x, p.y, p.z],
        rot: [q.x, q.y, q.z, q.w],
    })
}

pub struct Input {
    set: xr::ActionSet,
    aim: xr::Action<xr::Posef>,
    click: xr::Action<bool>,
    menu: xr::Action<bool>,
    stats: xr::Action<bool>,
    right_stick: xr::Action<xr::Vector2f>,
    left_stick: xr::Action<xr::Vector2f>,
    grip: xr::Action<bool>,
    haptic: xr::Action<xr::Haptic>,
    aim_space: xr::Space,
}

/// One frame's input.
#[derive(Default)]
pub struct Frame {
    /// The right controller's aim; a space is None when the tracker cannot
    /// place the controller in it.
    pub aim: Rays,
    /// Trigger edge this frame: Some(true) pressed, Some(false) released.
    pub click: Option<bool>,
    pub menu: bool,
    pub stats: bool,
    /// Thumbstick Y, -1..1, 0 inside the dead zone.
    pub right_y: f32,
    pub left_y: f32,
    /// The right grip is held.
    pub grip: bool,
}

impl Input {
    pub fn new(inst: &xr::Instance, session: &xr::Session<xr::OpenGlEs>) -> xr::Result<Self> {
        let set = inst.create_action_set("gs", "Ground station", 0)?;
        let aim = set.create_action::<xr::Posef>("aim", "Pointer", &[])?;
        let click = set.create_action::<bool>("click", "Click", &[])?;
        let menu = set.create_action::<bool>("menu", "Settings", &[])?;
        let stats = set.create_action::<bool>("stats", "Link statistics", &[])?;
        let right_stick =
            set.create_action::<xr::Vector2f>("size", "Screen size, menu scroll", &[])?;
        let left_stick = set.create_action::<xr::Vector2f>("dist", "Screen distance", &[])?;
        let grip = set.create_action::<bool>("grip", "Carry the window", &[])?;
        let haptic = set.create_action::<xr::Haptic>("haptic", "Click feedback", &[])?;
        let p = |s: &str| inst.string_to_path(s);
        inst.suggest_interaction_profile_bindings(
            p("/interaction_profiles/oculus/touch_controller")?,
            &[
                xr::Binding::new(&aim, p("/user/hand/right/input/aim/pose")?),
                xr::Binding::new(&click, p("/user/hand/right/input/trigger/value")?),
                xr::Binding::new(&menu, p("/user/hand/right/input/b/click")?),
                xr::Binding::new(&menu, p("/user/hand/left/input/y/click")?),
                xr::Binding::new(&stats, p("/user/hand/right/input/a/click")?),
                xr::Binding::new(&stats, p("/user/hand/left/input/x/click")?),
                xr::Binding::new(&right_stick, p("/user/hand/right/input/thumbstick")?),
                xr::Binding::new(&left_stick, p("/user/hand/left/input/thumbstick")?),
                xr::Binding::new(&grip, p("/user/hand/right/input/squeeze/value")?),
                xr::Binding::new(&haptic, p("/user/hand/right/output/haptic")?),
            ],
        )?;
        session.attach_action_sets(&[&set])?;
        let aim_space = aim.create_space(session.clone(), xr::Path::NULL, xr::Posef::IDENTITY)?;
        Ok(Self {
            set,
            aim,
            click,
            menu,
            stats,
            right_stick,
            left_stick,
            grip,
            haptic,
            aim_space,
        })
    }

    /// `view` and `local`: the two reference spaces quads hang in.
    pub fn poll(
        &self,
        session: &xr::Session<xr::OpenGlEs>,
        view: &xr::Space,
        local: &xr::Space,
        t: xr::Time,
    ) -> Frame {
        let mut f = Frame::default();
        if session
            .sync_actions(&[xr::ActiveActionSet::new(&self.set)])
            .is_err()
        {
            return f;
        }
        let edge = |a: &xr::Action<bool>| {
            a.state(session, xr::Path::NULL)
                .ok()
                .filter(|s| s.is_active && s.changed_since_last_sync)
                .map(|s| s.current_state)
        };
        f.click = edge(&self.click);
        f.menu = edge(&self.menu) == Some(true);
        f.stats = edge(&self.stats) == Some(true);
        let stick = |a: &xr::Action<xr::Vector2f>| {
            a.state(session, xr::Path::NULL)
                .ok()
                .filter(|s| s.is_active)
                // A dead zone: a resting thumb must not creep the screen.
                .map_or(0.0, |s| {
                    if s.current_state.y.abs() < 0.2 {
                        0.0
                    } else {
                        s.current_state.y
                    }
                })
        };
        f.right_y = stick(&self.right_stick);
        f.left_y = stick(&self.left_stick);
        f.grip = self
            .grip
            .state(session, xr::Path::NULL)
            .is_ok_and(|s| s.is_active && s.current_state);
        if !self.aim.is_active(session, xr::Path::NULL).unwrap_or(false) {
            return f;
        }
        let ray = |base: &xr::Space| {
            locate(&self.aim_space, base, t).map(|p| Ray {
                origin: p.pos,
                // Forward is -Z rotated by the aim.
                dir: rotate(p.rot, [0.0, 0.0, -1.0]),
            })
        };
        f.aim = Rays {
            view: ray(view),
            local: ray(local),
        };
        f
    }

    /// A short tick in the right hand: a press landed on a control.
    #[allow(dead_code)] // called from the menu commit on
    pub fn pulse(&self, session: &xr::Session<xr::OpenGlEs>) {
        let v = xr::HapticVibration::new()
            .amplitude(0.3)
            .frequency(xr::FREQUENCY_UNSPECIFIED)
            .duration(xr::Duration::from_nanos(20_000_000));
        if let Err(e) = self.haptic.apply_feedback(session, xr::Path::NULL, &v) {
            log::warn!("haptic: {e}");
        }
    }
}
