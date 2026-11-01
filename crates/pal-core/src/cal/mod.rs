//! TX-power calibration, GS side (mabur gs/src/cal_*): docs/calibration-port.md.
pub mod analysis;
pub mod log;
pub mod picker;
pub mod plan;
pub mod session;

use crate::json::J;
use session::{sweep_token, CalSnapshot};

/// The UI's view of a run (maburgs' sideport link.cal plus the result
/// table `maburcal` prints): one JSON object.
/// `capable`: linked to a drone that advertises CAP_CALIBRATE.
pub fn status_json(s: &CalSnapshot, sweeps: &[pal_wire::cal::SweepSpec], capable: bool) -> String {
    let mut j = J::obj();
    j.set("state", s.state.name());
    j.set("capable", capable);
    j.set("running", s.state.running());
    j.set(
        "sweep_k",
        if s.sweep_n > 0 {
            s.sweep_i as u32 + 1
        } else {
            0
        },
    );
    j.set("sweep_n", s.sweep_n as u32);
    j.set("kind", s.kind_name());
    j.set("bw", s.cur.bw);
    j.set("cell_i", s.cell_i);
    j.set("cell_n", s.cell_n);
    j.set("channel", s.channel);
    j.set("pick", s.pick_note.as_str());
    j.set("stamp_mismatch", s.stamp_mismatch);
    j.set("fail_reason", s.fail_reason.as_str());
    let mut tokens = J::arr();
    for sp in sweeps {
        tokens.push(sweep_token(sp));
    }
    j.set("sweeps", tokens);
    let mut widths = J::arr();
    for (wi, w) in s.width.iter().enumerate() {
        if !w.have_walls && !w.have_eff && !w.eff_unknown {
            continue;
        }
        let mut o = J::obj();
        o.set("bw", if wi == 1 { 40u32 } else { 20u32 });
        o.set("applied", w.applied);
        o.set("apply_reason", w.apply_reason.as_str());
        o.set("eff_unknown", w.eff_unknown);
        let mut rates = J::arr();
        for r in 0..8 {
            let mut x = J::obj();
            x.set("mcs", r as u32);
            let wall = w.walls[r];
            x.set(
                "wall",
                (w.have_walls && wall.wall != analysis::NO_WALL).then_some(wall.wall),
            );
            x.set("wall_flags", wall.flags);
            let park = w.park[r];
            x.set(
                "park",
                (park != i32::from(pal_wire::cal::WALL_LEAVE)).then_some(park),
            );
            x.set(
                "verify_pct",
                (w.verify_pct[r] >= 0).then_some(w.verify_pct[r]),
            );
            x.set(
                "eff",
                (w.have_eff && w.eff_x1000[r] != 0).then_some(f64::from(w.eff_x1000[r]) / 1000.0),
            );
            x.set("cap_flags", w.cap[r].flags);
            rates.push(x);
        }
        o.set("rates", rates);
        widths.push(o);
    }
    j.set("widths", widths);
    j.dump()
}
#[cfg(test)]
mod session_tests;
