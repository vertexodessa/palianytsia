//! Touch controllers through OpenXR actions (spec 5.6 "Input"): the right
//! controller's aim ray is a pointer on the OSD quad, the trigger clicks,
//! B or Y opens the settings panel, A or X toggles link statistics.
use openxr as xr;

pub struct Input {
    set: xr::ActionSet,
    aim: xr::Action<xr::Posef>,
    click: xr::Action<bool>,
    menu: xr::Action<bool>,
    stats: xr::Action<bool>,
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
}

impl Input {
    pub fn new(inst: &xr::Instance, session: &xr::Session<xr::OpenGlEs>) -> xr::Result<Self> {
        let set = inst.create_action_set("gs", "Ground station", 0)?;
        let aim = set.create_action::<xr::Posef>("aim", "Pointer", &[])?;
        let click = set.create_action::<bool>("click", "Click", &[])?;
        let menu = set.create_action::<bool>("menu", "Settings", &[])?;
        let stats = set.create_action::<bool>("stats", "Link statistics", &[])?;
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
            aim_space,
        })
    }

    /// `view`: the VIEW space both quads live in; the quad is `w_m` wide,
    /// 16:9, centred `dist_m` ahead; `pts` is the OSD's size in points.
    pub fn poll(
        &self,
        session: &xr::Session<xr::OpenGlEs>,
        view: &xr::Space,
        t: xr::Time,
        dist_m: f32,
        w_m: f32,
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
        let active = self.aim.is_active(session, xr::Path::NULL).unwrap_or(false);
        if !active {
            return f;
        }
        let Ok(loc) = self.aim_space.locate(view, t) else {
            return f;
        };
        let ok = xr::SpaceLocationFlags::POSITION_VALID | xr::SpaceLocationFlags::ORIENTATION_VALID;
        if !loc.location_flags.contains(ok) {
            return f;
        }
        let (p, q) = (loc.pose.position, loc.pose.orientation);
        // Forward is -Z rotated by q.
        let d = rotate(q, [0.0, 0.0, -1.0]);
        if d[2] >= -1e-3 {
            return f;
        }
        let t = (-dist_m - p.z) / d[2];
        let (hx, hy) = (p.x + t * d[0], p.y + t * d[1]);
        let h_m = w_m * 9.0 / 16.0;
        let u = (hx + w_m / 2.0) / w_m;
        let v = (h_m / 2.0 - hy) / h_m;
        if (0.0..=1.0).contains(&u) && (0.0..=1.0).contains(&v) {
            f.pointer = Some(egui::pos2(u * pts.x, v * pts.y));
        }
        f
    }
}

/// v rotated by the unit quaternion q.
fn rotate(q: xr::Quaternionf, v: [f32; 3]) -> [f32; 3] {
    let (x, y, z, w) = (q.x, q.y, q.z, q.w);
    // t = 2 * cross(q.xyz, v); v' = v + w * t + cross(q.xyz, t)
    let t = [
        2.0 * (y * v[2] - z * v[1]),
        2.0 * (z * v[0] - x * v[2]),
        2.0 * (x * v[1] - y * v[0]),
    ];
    [
        v[0] + w * t[0] + (y * t[2] - z * t[1]),
        v[1] + w * t[1] + (z * t[0] - x * t[2]),
        v[2] + w * t[2] + (x * t[1] - y * t[0]),
    ]
}

#[cfg(test)]
mod tests {
    #[test]
    fn identity_and_yaw() {
        let id = openxr::Quaternionf {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        };
        assert_eq!(super::rotate(id, [0.0, 0.0, -1.0]), [0.0, 0.0, -1.0]);
        // 90 degrees about +Y turns -Z into -X.
        let s = std::f32::consts::FRAC_1_SQRT_2;
        let yaw = openxr::Quaternionf {
            x: 0.0,
            y: s,
            z: 0.0,
            w: s,
        };
        let r = super::rotate(yaw, [0.0, 0.0, -1.0]);
        assert!((r[0] + 1.0).abs() < 1e-5 && r[1].abs() < 1e-5 && r[2].abs() < 1e-5);
    }
}
