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

/// A 16:9 quad; it lies in its local XY plane and faces +Z.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Placement {
    pub pose: Pose,
    pub width_m: f32,
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

impl Placement {
    /// Goggle mode: straight ahead in VIEW space.
    pub fn in_view(dist_m: f32, width_m: f32) -> Self {
        Self {
            pose: Pose {
                pos: [0.0, 0.0, -dist_m],
                rot: IDENTITY,
            },
            width_m,
        }
    }

    /// At eye height in the direction the head faces, turned to the head.
    /// The head's pitch is left out: a pilot who looks down at the
    /// controllers at launch still gets the window ahead, not on the floor.
    pub fn in_front_of(head: Pose, dist_m: f32, width_m: f32) -> Self {
        let flat = |v: V3| {
            let l = v[0].hypot(v[2]);
            (l > 0.05).then(|| [v[0] / l, 0.0, v[2] / l])
        };
        let forward = rotate(head.rot, [0.0, 0.0, -1.0]);
        // Straight down, the top of the head points where the face would;
        // straight up, the opposite way.
        let up = rotate(head.rot, [0.0, 1.0, 0.0]);
        let dir = flat(forward)
            .or_else(|| flat(scale(up, -forward[1].signum())))
            .unwrap_or([0.0, 0.0, -1.0]);
        let pos = add(head.pos, scale(dir, dist_m));
        Self {
            pose: Pose {
                pos,
                rot: facing(pos, head.pos),
            },
            width_m,
        }
    }

    pub fn height_m(&self) -> f32 {
        self.width_m * 9.0 / 16.0
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

    #[test]
    fn pushed_moves_towards_the_viewer() {
        let mut p = Placement::in_view(2.0, 1.0);
        p.pushed(0.5);
        assert!(near(p.pose.pos, [0.0, 0.0, -1.5]));
    }
}
