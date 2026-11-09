//! Touch controllers through OpenXR actions (spec 5.6 "Input"): the right
//! controller's aim ray is a pointer on the OSD quad, the trigger clicks,
//! B or Y opens the settings panel, A or X toggles link statistics; the
//! right thumbstick resizes the screen and the left one moves it nearer or
//! farther; the right grip carries the window in window mode.
use openxr as xr;

use crate::placement::Pose;
use crate::placement::{rotate, Placement, Ray};

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
    size: xr::Action<xr::Vector2f>,
    dist: xr::Action<xr::Vector2f>,
    grip: xr::Action<bool>,
    aim_space: xr::Space,
}

/// One frame's input, in the OSD's egui points.
#[derive(Default)]
pub struct Frame {
    /// Where the aim ray meets the OSD quad, None when it misses.
    pub pointer: Option<egui::Pos2>,
    /// Trigger edge this frame: Some(true) pressed, Some(false) released.
    pub click: Option<bool>,
    pub menu: bool,
    pub stats: bool,
    /// Thumbstick Y, -1..1: right = screen size, left = distance.
    pub size: f32,
    pub dist: f32,
    /// The right controller's aim, in the space given to `poll`.
    pub aim: Option<Ray>,
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
        let size = set.create_action::<xr::Vector2f>("size", "Screen size", &[])?;
        let dist = set.create_action::<xr::Vector2f>("dist", "Screen distance", &[])?;
        let grip = set.create_action::<bool>("grip", "Carry the window", &[])?;
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
                xr::Binding::new(&size, p("/user/hand/right/input/thumbstick")?),
                xr::Binding::new(&dist, p("/user/hand/left/input/thumbstick")?),
                xr::Binding::new(&grip, p("/user/hand/right/input/squeeze/value")?),
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
            size,
            dist,
            grip,
            aim_space,
        })
    }

    /// `space`: the one both quads live in; `osd` is where the OSD quad
    /// hangs in it; `pts` is the OSD's size in points.
    pub fn poll(
        &self,
        session: &xr::Session<xr::OpenGlEs>,
        space: &xr::Space,
        t: xr::Time,
        osd: &Placement,
        pts: egui::Vec2,
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
        f.size = stick(&self.size);
        f.dist = stick(&self.dist);
        f.grip = self
            .grip
            .state(session, xr::Path::NULL)
            .is_ok_and(|s| s.is_active && s.current_state);
        let active = self.aim.is_active(session, xr::Path::NULL).unwrap_or(false);
        if !active {
            return f;
        }
        let Some(p) = locate(&self.aim_space, space, t) else {
            return f;
        };
        let ray = Ray {
            origin: p.pos,
            // Forward is -Z rotated by the aim.
            dir: rotate(p.rot, [0.0, 0.0, -1.0]),
        };
        f.aim = Some(ray);
        if let Some((_, [u, v])) = osd.hit(&ray) {
            f.pointer = Some(egui::pos2(u * pts.x, v * pts.y));
        }
        f
    }
}
