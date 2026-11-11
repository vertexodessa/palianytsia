//! Where a quad hangs and how a controller ray meets it. Plain arrays and
//! no OpenXR types, so host builds run the tests. A placement is relative
//! to some space; the caller says which one (VIEW for goggle mode, LOCAL
//! for the window) and nothing here depends on it.
pub type V3 = [f32; 3];
/// x, y, z, w.
pub type Quat = [f32; 4];

pub const MIN_DIST_M: f32 = 0.6;
pub const MAX_DIST_M: f32 = 5.0;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub pos: V3,
    pub rot: Quat,
}

/// A quad; it lies in its local XY plane and faces +Z.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Placement {
    pub pose: Pose,
    pub width_m: f32,
    /// Height over width.
    pub aspect: f32,
}

/// The video's and the OSD's shape.
pub const WIDE: f32 = 9.0 / 16.0;

/// The reference space a placement or a ray is given in: VIEW moves with
/// the head, LOCAL stays in the room.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpaceKind {
    View,
    Local,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ray {
    pub origin: V3,
    /// Unit length.
    pub dir: V3,
}

const IDENTITY: Quat = [0.0, 0.0, 0.0, 1.0];

fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn scale(a: V3, k: f32) -> V3 {
    [a[0] * k, a[1] * k, a[2] * k]
}

fn len(a: V3) -> f32 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

pub fn distance(a: V3, b: V3) -> f32 {
    len(sub(a, b))
}

/// v rotated by the unit quaternion q.
pub fn rotate(q: Quat, v: V3) -> V3 {
    let [x, y, z, w] = q;
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

fn inverse(q: Quat) -> Quat {
    [-q[0], -q[1], -q[2], q[3]]
}

/// The rotation that turns a quad at `pos` towards `head`, upright: yaw and
/// pitch, never roll.
pub fn facing(pos: V3, head: V3) -> Quat {
    let d = sub(head, pos);
    let yaw = d[0].atan2(d[2]);
    let pitch = -d[1].atan2(d[0].hypot(d[2]));
    let (sy, cy) = (yaw / 2.0).sin_cos();
    let (sp, cp) = (pitch / 2.0).sin_cos();
    // yaw * pitch
    [cy * sp, sy * cp, -sy * sp, cy * cp]
}

/// Where the head faces, levelled: unit length, no vertical part. The
/// head's pitch is left out, so looking down at the controllers still
/// means "ahead".
fn level_forward(head: Quat) -> V3 {
    let flat = |v: V3| {
        let l = v[0].hypot(v[2]);
        (l > 0.05).then(|| [v[0] / l, 0.0, v[2] / l])
    };
    let forward = rotate(head, [0.0, 0.0, -1.0]);
    // Straight down, the top of the head points where the face would;
    // straight up, the opposite way.
    let up = rotate(head, [0.0, 1.0, 0.0]);
    flat(forward)
        .or_else(|| flat(scale(up, -forward[1].signum())))
        .unwrap_or([0.0, 0.0, -1.0])
}

impl Placement {
    /// Goggle mode: straight ahead in VIEW space.
    pub fn in_view(dist_m: f32, width_m: f32) -> Self {
        Self {
            pose: Pose {
                pos: [0.0, 0.0, -dist_m],
                rot: IDENTITY,
            },
            width_m,
            aspect: WIDE,
        }
    }

    /// At eye height in the direction the head faces, turned to the head:
    /// a pilot who looks down at the controllers at launch still gets the
    /// window ahead, not on the floor.
    pub fn in_front_of(head: Pose, dist_m: f32, width_m: f32) -> Self {
        let pos = add(head.pos, scale(level_forward(head.rot), dist_m));
        Self {
            pose: Pose {
                pos,
                rot: facing(pos, head.pos),
            },
            width_m,
            aspect: WIDE,
        }
    }

    pub fn height_m(&self) -> f32 {
        self.width_m * self.aspect
    }

    /// The same centre and orientation, `k` times the size.
    pub fn scaled(&self, k: f32) -> Self {
        Self {
            width_m: self.width_m * k,
            ..*self
        }
    }

    /// Moves along the quad's normal, towards the viewer when `by` > 0.
    pub fn pushed(&mut self, by: f32) {
        let n = rotate(self.pose.rot, [0.0, 0.0, 1.0]);
        self.pose.pos = add(self.pose.pos, scale(n, by));
    }

    /// Distance along the ray and (u, v) on the quad, (0, 0) its top left,
    /// when the ray meets the front of the quad.
    pub fn hit(&self, ray: &Ray) -> Option<(f32, [f32; 2])> {
        let inv = inverse(self.pose.rot);
        let o = rotate(inv, sub(ray.origin, self.pose.pos));
        let d = rotate(inv, ray.dir);
        if d[2] >= -1e-3 {
            return None;
        }
        let t = -o[2] / d[2];
        if t <= 0.0 {
            return None;
        }
        let h = self.height_m();
        let u = (o[0] + t * d[0] + self.width_m / 2.0) / self.width_m;
        let v = (h / 2.0 - (o[1] + t * d[1])) / h;
        ((0.0..=1.0).contains(&u) && (0.0..=1.0).contains(&v)).then_some((t, [u, v]))
    }
}

/// The controller's aim in each space it could be located in.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Rays {
    pub view: Option<Ray>,
    pub local: Option<Ray>,
}

impl Rays {
    pub fn in_space(&self, s: SpaceKind) -> Option<Ray> {
        match s {
            SpaceKind::View => self.view,
            SpaceKind::Local => self.local,
        }
    }
}

/// A surface the ray can point at: where it hangs, in which space, and its
/// size in the units its pointer events use (egui points).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Target<T> {
    pub id: T,
    pub space: SpaceKind,
    pub at: Placement,
    pub size: [f32; 2],
}

/// Where the ray met a target.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hit<T> {
    pub id: T,
    /// On the surface, in its own units, (0, 0) the top left.
    pub pos: [f32; 2],
    /// The same point in the target's space, for the cursor quad.
    pub point: V3,
    /// Metres from the ray's origin.
    pub reach: f32,
}

/// The first of `targets` the ray meets: they are given front to back, so
/// a surface drawn over another also takes the pointer from it.
pub fn route<T: Copy>(rays: &Rays, targets: &[Target<T>]) -> Option<Hit<T>> {
    targets.iter().find_map(|t| {
        let ray = rays.in_space(t.space)?;
        let (reach, [u, v]) = t.at.hit(&ray)?;
        Some(Hit {
            id: t.id,
            pos: [u * t.size[0], v * t.size[1]],
            point: add(ray.origin, scale(ray.dir, reach)),
            reach,
        })
    })
}

/// The settings menu's quad: 44 x 34 degrees at 1.1 m, its centre 20
/// degrees below the eye line, so its top edge is 3 degrees below it and
/// the horizon stays clear.
pub const MENU_WIDTH_M: f32 = 0.89;
pub const MENU_ASPECT: f32 = 0.75;
pub const MENU_DIST_M: f32 = 1.1;
pub const MENU_PITCH_DEG: f32 = 20.0;

/// Where the menu hangs, decided once when it opens. With a head pose:
/// fixed in the room (LOCAL), in the direction the head faces, turned to
/// the head. Without one: in VIEW, the same place relative to the eyes,
/// and it stays there until it is opened again. It keeps 0.1 m in front
/// of a video nearer than the menu's own distance.
pub fn menu_at(head: Option<Pose>, video_dist_m: f32) -> (SpaceKind, Placement) {
    let dist = MENU_DIST_M.min(video_dist_m - 0.1).max(MIN_DIST_M);
    let (s, c) = MENU_PITCH_DEG.to_radians().sin_cos();
    let (space, eye, ahead) = match head {
        Some(h) => (SpaceKind::Local, h.pos, level_forward(h.rot)),
        None => (SpaceKind::View, [0.0; 3], [0.0, 0.0, -1.0]),
    };
    let pos = add(eye, add(scale(ahead, dist * c), [0.0, -dist * s, 0.0]));
    let at = Placement {
        pose: Pose {
            pos,
            rot: facing(pos, eye),
        },
        width_m: MENU_WIDTH_M,
        aspect: MENU_ASPECT,
    };
    (space, at)
}

fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn dot(a: V3, b: V3) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// The rotation whose X, Y and Z axes are the given orthonormal vectors.
fn from_axes(x: V3, y: V3, z: V3) -> Quat {
    let tr = x[0] + y[1] + z[2];
    let q = if tr > 0.0 {
        let s = (tr + 1.0).sqrt() * 2.0;
        [
            (y[2] - z[1]) / s,
            (z[0] - x[2]) / s,
            (x[1] - y[0]) / s,
            s / 4.0,
        ]
    } else if x[0] > y[1] && x[0] > z[2] {
        let s = (1.0 + x[0] - y[1] - z[2]).sqrt() * 2.0;
        [
            s / 4.0,
            (y[0] + x[1]) / s,
            (z[0] + x[2]) / s,
            (y[2] - z[1]) / s,
        ]
    } else if y[1] > z[2] {
        let s = (1.0 + y[1] - x[0] - z[2]).sqrt() * 2.0;
        [
            (y[0] + x[1]) / s,
            s / 4.0,
            (z[1] + y[2]) / s,
            (z[0] - x[2]) / s,
        ]
    } else {
        let s = (1.0 + z[2] - x[0] - y[1]).sqrt() * 2.0;
        [
            (z[0] + x[2]) / s,
            (z[1] + y[2]) / s,
            s / 4.0,
            (x[1] - y[0]) / s,
        ]
    };
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
}

/// The pointer ray as a thin quad: `width_m` wide, from the ray's origin
/// to `reach` metres along it, turned about the ray to face `head`. None
/// when the head is on the ray's own line (no facing to choose).
pub fn ray_quad(ray: &Ray, reach: f32, head: V3, width_m: f32) -> Option<Placement> {
    let mid = add(ray.origin, scale(ray.dir, reach / 2.0));
    let to_head = sub(head, mid);
    // The part of "towards the head" that is square to the ray.
    let z = sub(to_head, scale(ray.dir, dot(to_head, ray.dir)));
    let l = len(z);
    if l < 1e-4 || reach <= 0.0 {
        return None;
    }
    let z = scale(z, 1.0 / l);
    let y = ray.dir;
    Some(Placement {
        pose: Pose {
            pos: mid,
            rot: from_axes(cross(y, z), y, z),
        },
        width_m,
        aspect: reach / width_m,
    })
}

/// The cursor: a small square lying on a surface at `point`, lifted 2 mm
/// off it towards the viewer so the two do not fight.
pub fn cursor_quad(on: &Placement, point: V3, size_m: f32) -> Placement {
    let n = rotate(on.pose.rot, [0.0, 0.0, 1.0]);
    Placement {
        pose: Pose {
            pos: add(point, scale(n, 0.002)),
            rot: on.pose.rot,
        },
        width_m: size_m,
        aspect: 1.0,
    }
}

/// Who the thumbsticks serve this frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Sticks {
    /// The screen: right stick its size, left its distance.
    Screen {
        size: f32,
        dist: f32,
    },
    /// The open menu: the right stick scrolls it.
    Scroll(f32),
    Idle,
}

/// One decision per frame. An open menu takes the sticks; a window listens
/// only while the ray is on it and it is not being carried (a thumb
/// resting on a stick must not change it unasked in flight); the goggle
/// screen always listens.
pub fn sticks(menu_open: bool, window: Option<(bool, bool)>, right_y: f32, left_y: f32) -> Sticks {
    if menu_open {
        return if right_y == 0.0 {
            Sticks::Idle
        } else {
            Sticks::Scroll(right_y)
        };
    }
    match window {
        Some((aimed, carried)) if !aimed || carried => Sticks::Idle,
        _ => Sticks::Screen {
            size: right_y,
            dist: left_y,
        },
    }
}

/// A window held by the ray: it keeps its distance along the ray and the
/// point that was grabbed stays under the ray.
#[derive(Clone, Copy, Debug)]
pub struct Grab {
    reach: f32,
    offset: V3,
}

impl Grab {
    pub fn start(ray: &Ray, p: &Placement) -> Option<Self> {
        let (reach, _) = p.hit(ray)?;
        let at = add(ray.origin, scale(ray.dir, reach));
        Some(Self {
            reach,
            offset: sub(p.pose.pos, at),
        })
    }

    /// The window's pose for this ray, turned to the head and kept within
    /// the distance limits.
    pub fn moved(&self, ray: &Ray, head: V3) -> Pose {
        let wanted = add(add(ray.origin, scale(ray.dir, self.reach)), self.offset);
        let v = sub(wanted, head);
        let l = len(v);
        let v = if l < 1e-3 {
            [0.0, 0.0, -MIN_DIST_M]
        } else {
            scale(v, l.clamp(MIN_DIST_M, MAX_DIST_M) / l)
        };
        let pos = add(head, v);
        Pose {
            pos,
            rot: facing(pos, head),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pal_osd::panel::SURFACE_POINTS as MENU_POINTS;

    fn near(a: V3, b: V3) -> bool {
        distance(a, b) < 1e-4
    }

    fn yaw90() -> Quat {
        let s = std::f32::consts::FRAC_1_SQRT_2;
        [0.0, s, 0.0, s]
    }

    fn head(rot: Quat) -> Pose {
        Pose {
            pos: [0.5, 1.6, 0.2],
            rot,
        }
    }

    #[test]
    fn rotate_identity_and_yaw() {
        assert_eq!(rotate(IDENTITY, [0.0, 0.0, -1.0]), [0.0, 0.0, -1.0]);
        // 90 degrees about +Y turns -Z into -X.
        assert!(near(rotate(yaw90(), [0.0, 0.0, -1.0]), [-1.0, 0.0, 0.0]));
    }

    #[test]
    fn the_quad_faces_the_head() {
        for (pos, h) in [
            ([0.0, 0.0, -2.0], [0.0, 0.0, 0.0]),
            ([3.0, 1.0, -1.0], [0.0, 1.6, 0.0]),
            ([-2.0, 3.0, 2.0], [0.5, 0.2, 0.0]),
        ] {
            let n = rotate(facing(pos, h), [0.0, 0.0, 1.0]);
            assert!(near(n, scale(sub(h, pos), 1.0 / distance(h, pos))), "{n:?}");
            // No roll: the quad's X axis stays level.
            assert!(rotate(facing(pos, h), [1.0, 0.0, 0.0])[1].abs() < 1e-4);
        }
    }

    #[test]
    fn placed_ahead_at_eye_height_whatever_the_pitch() {
        let p = Placement::in_front_of(head(yaw90()), 2.0, 1.2);
        assert!(near(p.pose.pos, [-1.5, 1.6, 0.2]));
        // Pitched 80 degrees down, facing -Z: still straight ahead.
        let (s, c) = (-40f32).to_radians().sin_cos();
        let p = Placement::in_front_of(head([s, 0.0, 0.0, c]), 2.0, 1.2);
        assert!(near(p.pose.pos, [0.5, 1.6, -1.8]), "{:?}", p.pose.pos);
        // Exactly straight down: the top of the head gives the direction.
        let (s, c) = (-45f32).to_radians().sin_cos();
        let p = Placement::in_front_of(head([s, 0.0, 0.0, c]), 2.0, 1.2);
        assert!(near(p.pose.pos, [0.5, 1.6, -1.8]), "{:?}", p.pose.pos);
    }

    #[test]
    fn hit_in_goggle_placement_matches_the_old_plane_test() {
        let p = Placement::in_view(1.5, 3.0);
        let ray = Ray {
            origin: [0.0, 0.0, 0.0],
            dir: [0.0, 0.0, -1.0],
        };
        let (t, uv) = p.hit(&ray).unwrap();
        assert!((t - 1.5).abs() < 1e-4 && near([uv[0], uv[1], 0.0], [0.5, 0.5, 0.0]));
        // A ray up and to the right lands in the top right quarter.
        let up_right = Ray {
            origin: [0.5, 0.2, 0.0],
            dir: [0.0, 0.0, -1.0],
        };
        let (_, uv) = p.hit(&up_right).unwrap();
        assert!(uv[0] > 0.5 && uv[1] < 0.5);
    }

    #[test]
    fn hit_misses() {
        let p = Placement::in_view(1.5, 1.0);
        let off = Ray {
            origin: [2.0, 0.0, 0.0],
            dir: [0.0, 0.0, -1.0],
        };
        let away = Ray {
            origin: [0.0, 0.0, 0.0],
            dir: [0.0, 0.0, 1.0],
        };
        let beyond = Ray {
            origin: [0.0, 0.0, -3.0],
            dir: [0.0, 0.0, -1.0],
        };
        assert!(p.hit(&off).is_none() && p.hit(&away).is_none() && p.hit(&beyond).is_none());
    }

    #[test]
    fn hit_on_a_turned_window() {
        let pos = [-2.0, 1.6, 0.0];
        let p = Placement {
            pose: Pose {
                pos,
                rot: facing(pos, [0.0, 1.6, 0.0]),
            },
            width_m: 1.0,
            aspect: WIDE,
        };
        let ray = Ray {
            origin: [0.0, 1.6, 0.0],
            dir: [-1.0, 0.0, 0.0],
        };
        let (t, uv) = p.hit(&ray).unwrap();
        assert!((t - 2.0).abs() < 1e-4 && (uv[0] - 0.5).abs() < 1e-4);
    }

    #[test]
    fn a_grabbed_window_keeps_the_point_under_the_ray() {
        let p = Placement::in_view(2.0, 2.0);
        let start = Ray {
            origin: [0.2, 0.0, 0.0],
            dir: [0.0, 0.0, -1.0],
        };
        let g = Grab::start(&start, &p).unwrap();
        // Unmoved ray: the window stays (head on the axis, so no turn).
        let same = g.moved(&start, [0.0, 0.0, 0.0]);
        assert!(near(same.pos, p.pose.pos));
        // Controller 0.3 m right: so does the window.
        let right = Ray {
            origin: [0.5, 0.0, 0.0],
            ..start
        };
        assert!(near(g.moved(&right, [0.0, 0.0, 0.0]).pos, [0.3, 0.0, -2.0]));
    }

    #[test]
    fn a_grabbed_window_stays_in_the_distance_limits() {
        let p = Placement::in_view(2.0, 2.0);
        let start = Ray {
            origin: [0.0, 0.0, 0.0],
            dir: [0.0, 0.0, -1.0],
        };
        let g = Grab::start(&start, &p).unwrap();
        let far = Ray {
            origin: [0.0, 0.0, -10.0],
            ..start
        };
        let got = g.moved(&far, [0.0, 0.0, 0.0]);
        assert!((distance(got.pos, [0.0; 3]) - MAX_DIST_M).abs() < 1e-4);
    }

    fn deg(rad: f32) -> f32 {
        rad.to_degrees()
    }

    /// Yaw and pitch of `p` seen from `eye`: yaw 0 = -Z, positive to the
    /// right; pitch positive up.
    fn bearing(eye: V3, p: V3) -> (f32, f32) {
        let d = sub(p, eye);
        (deg(d[0].atan2(-d[2])), deg(d[1].atan2(d[0].hypot(d[2]))))
    }

    #[test]
    fn the_menu_opens_ahead_and_below_the_eye_line() {
        // Facing -X (yaw 90), pitched anywhere: ahead is still -X.
        let h = head(yaw90());
        let (space, m) = menu_at(Some(h), 1.5);
        assert_eq!(space, SpaceKind::Local);
        let to = sub(m.pose.pos, h.pos);
        assert!((len(to) - MENU_DIST_M).abs() < 1e-4);
        assert!(to[0] < 0.0 && to[2].abs() < 1e-4, "{to:?}");
        let pitch = deg(to[1].atan2(to[0].hypot(to[2])));
        assert!((pitch + MENU_PITCH_DEG).abs() < 0.01, "{pitch}");
        // It faces the head.
        let n = rotate(m.pose.rot, [0.0, 0.0, 1.0]);
        assert!(near(n, scale(sub(h.pos, m.pose.pos), 1.0 / MENU_DIST_M)));
        // 44 x 34 degrees.
        let w = 2.0 * deg((m.width_m / 2.0 / MENU_DIST_M).atan());
        let hh = 2.0 * deg((m.height_m() / 2.0 / MENU_DIST_M).atan());
        assert!(
            (w - 44.0).abs() < 2.0 && (hh - 34.0).abs() < 2.0,
            "{w} {hh}"
        );
        // Its top edge is below the eye line.
        let top = add(
            m.pose.pos,
            rotate(m.pose.rot, [0.0, m.height_m() / 2.0, 0.0]),
        );
        assert!(bearing(h.pos, top).1 < -2.0);
    }

    #[test]
    fn the_menu_does_not_follow_the_head() {
        // Placed once: the same call with the pose at open gives the same
        // place, and nothing in it reads a later head pose.
        let h = head(IDENTITY);
        let (_, a) = menu_at(Some(h), 1.5);
        let (_, b) = menu_at(Some(h), 1.5);
        assert_eq!(a, b);
        let turned = menu_at(Some(head(yaw90())), 1.5).1;
        assert!(!near(a.pose.pos, turned.pose.pos));
    }

    #[test]
    fn without_a_head_pose_the_menu_is_head_locked() {
        let (space, m) = menu_at(None, 1.5);
        assert_eq!(space, SpaceKind::View);
        let (yaw, pitch) = bearing([0.0; 3], m.pose.pos);
        assert!(yaw.abs() < 0.01 && (pitch + MENU_PITCH_DEG).abs() < 0.01);
        assert!((len(m.pose.pos) - MENU_DIST_M).abs() < 1e-4);
    }

    #[test]
    fn the_menu_stays_in_front_of_a_near_screen() {
        let d = |video| len(menu_at(None, video).1.pose.pos);
        assert!((d(1.5) - MENU_DIST_M).abs() < 1e-4);
        assert!((d(1.0) - 0.9).abs() < 1e-4);
        assert!((d(MIN_DIST_M) - MIN_DIST_M).abs() < 1e-4);
    }

    #[test]
    fn menu_text_and_targets_are_big_enough() {
        // pal-osd owns the surface's size in points and its look; this
        // crate owns the quad. Together they give the angles a pilot sees.
        let look = pal_osd::panel::Look::HEADSET;
        // One point at the centre of the surface, in degrees.
        let pt = deg((MENU_WIDTH_M / MENU_POINTS[0] / MENU_DIST_M).atan());
        // egui's body and small text sizes.
        let (body, small) = (14.0 * look.text, 10.0 * look.text);
        assert!(body * pt >= 1.3, "body {}", body * pt);
        assert!(small * pt >= 1.0, "small {}", small * pt);
        assert!(look.row_h * pt >= 3.0, "row {}", look.row_h * pt);
        assert!((MENU_POINTS[1] / MENU_POINTS[0] - MENU_ASPECT).abs() < 1e-6);
    }

    fn ahead() -> Ray {
        Ray {
            origin: [0.0, 0.0, 0.0],
            dir: [0.0, 0.0, -1.0],
        }
    }

    #[test]
    fn the_front_target_takes_the_pointer() {
        let back = Target {
            id: 'o',
            space: SpaceKind::View,
            at: Placement::in_view(1.5, 3.0),
            size: [1920.0, 1080.0],
        };
        let front = Target {
            id: 'm',
            space: SpaceKind::Local,
            at: Placement {
                aspect: MENU_ASPECT,
                ..Placement::in_view(1.0, MENU_WIDTH_M)
            },
            size: MENU_POINTS,
        };
        let both = Rays {
            view: Some(ahead()),
            local: Some(ahead()),
        };
        let hit = route(&both, &[front, back]).unwrap();
        assert_eq!(hit.id, 'm');
        assert!((hit.pos[0] - 400.0).abs() < 1e-2 && (hit.pos[1] - 300.0).abs() < 1e-2);
        assert!(near(hit.point, [0.0, 0.0, -1.0]) && (hit.reach - 1.0).abs() < 1e-4);
        // A ray past the menu's edge falls through to the OSD behind it.
        let right = Ray {
            origin: [0.8, 0.0, 0.0],
            ..ahead()
        };
        let past = Rays {
            view: Some(right),
            local: Some(right),
        };
        assert_eq!(route(&past, &[front, back]).unwrap().id, 'o');
    }

    #[test]
    fn a_target_is_tested_with_the_ray_of_its_own_space() {
        // The menu in the room, the OSD on the head: the head has turned,
        // so the same controller aim is two different rays.
        let osd = Target {
            id: 'o',
            space: SpaceKind::View,
            at: Placement::in_view(1.5, 3.0),
            size: [1920.0, 1080.0],
        };
        let menu = Target {
            id: 'm',
            space: SpaceKind::Local,
            at: Placement::in_view(1.0, 0.5),
            size: MENU_POINTS,
        };
        let away = Ray {
            origin: [0.0, 0.0, 0.0],
            dir: [1.0, 0.0, 0.0],
        };
        let rays = Rays {
            view: Some(ahead()),
            local: Some(away),
        };
        assert_eq!(route(&rays, &[menu, osd]).unwrap().id, 'o');
        // No ray in a space: its targets cannot be hit.
        let local_only = Rays {
            view: None,
            local: Some(ahead()),
        };
        assert_eq!(route(&local_only, &[osd]), None);
        assert_eq!(route(&local_only, &[menu, osd]).unwrap().id, 'm');
    }

    #[test]
    fn the_ray_quad_runs_along_the_ray_and_faces_the_head() {
        let ray = Ray {
            origin: [0.2, 1.0, 0.0],
            dir: [0.0, 0.0, -1.0],
        };
        let head = [0.0, 1.6, 0.0];
        let q = ray_quad(&ray, 2.0, head, 0.003).unwrap();
        assert!(near(q.pose.pos, [0.2, 1.0, -1.0]));
        assert!((q.height_m() - 2.0).abs() < 1e-4);
        // Its long axis is the ray; its normal is square to the ray and
        // leans to the head.
        assert!(near(rotate(q.pose.rot, [0.0, 1.0, 0.0]), ray.dir));
        let n = rotate(q.pose.rot, [0.0, 0.0, 1.0]);
        assert!(dot(n, ray.dir).abs() < 1e-4 && dot(n, sub(head, q.pose.pos)) > 0.0);
        // The head on the ray's line: no facing to choose.
        assert!(ray_quad(&ray, 2.0, [0.2, 1.0, 5.0], 0.003).is_none());
    }

    #[test]
    fn the_cursor_lies_on_the_surface() {
        let on = Placement::in_view(1.5, 3.0);
        let c = cursor_quad(&on, [0.3, 0.1, -1.5], 0.012);
        assert!(near(c.pose.pos, [0.3, 0.1, -1.498]));
        assert_eq!(c.pose.rot, on.pose.rot);
        assert!((c.height_m() - 0.012).abs() < 1e-6);
    }

    #[test]
    fn who_gets_the_sticks() {
        // Goggle: the screen, always.
        let screen = Sticks::Screen {
            size: 0.5,
            dist: -1.0,
        };
        assert_eq!(sticks(false, None, 0.5, -1.0), screen);
        // Window: only under the ray and not while carried.
        assert_eq!(sticks(false, Some((true, false)), 0.5, -1.0), screen);
        assert_eq!(sticks(false, Some((false, false)), 0.5, -1.0), Sticks::Idle);
        assert_eq!(sticks(false, Some((true, true)), 0.5, -1.0), Sticks::Idle);
        // An open menu takes the right stick and nothing moves the screen.
        assert_eq!(sticks(true, None, 0.5, -1.0), Sticks::Scroll(0.5));
        assert_eq!(sticks(true, Some((true, false)), 0.0, -1.0), Sticks::Idle);
    }

    #[test]
    fn pushed_moves_towards_the_viewer() {
        let mut p = Placement::in_view(2.0, 1.0);
        p.pushed(0.5);
        assert!(near(p.pose.pos, [0.0, 0.0, -1.5]));
    }
}
