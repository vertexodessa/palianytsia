//! The Rust hop verdict (pal-core `chan::verdict`) against mabur's C++
//! `HopVerdict`, window by window on random scripts: every output field
//! must match. The scripts walk the regions the verdict branches on
//! (foreign airtime around detect_pct and blocked_pct, loss around
//! loss_pct, recovered bursts, weak cards, reading-less windows, landings,
//! resets and new sessions), so each route to `interfered` is exercised.
use std::os::raw::c_int;

mod common;
use common::Rng;

use pal_core::chan::verdict::{HopVerdict, VerdictCardIn, VerdictLinkIn};
use pal_core::config::{BusyCfg, HopCfg};
use pal_ref as _;

#[repr(C)]
struct VCfg {
    window_ms: c_int,
    persist: c_int,
    blocked_pct: f64,
    loss_pct: f64,
    recovered_x: f64,
    recovered_min: c_int,
    weak_rssi_dbm: c_int,
    weak_snr_db: c_int,
    occupied_windows: c_int,
    detect_pct: f64,
    detect_windows: c_int,
    detect_impaired: c_int,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct VCard {
    valid: c_int,
    snr_valid: c_int,
    busy_valid: c_int,
    rssi_dbm: f64,
    snr_db: f64,
    nhm_busy_pct: f64,
    own_air_pct: f64,
}

#[repr(C)]
#[derive(Default, Debug, PartialEq)]
struct VOut {
    v: c_int,
    evidence: c_int,
    ref_rung: c_int,
    trigger: c_int,
    ref_frozen: c_int,
    first_blocked: c_int,
    t_start_ms: f64,
    t_ms: f64,
}

#[repr(C)]
struct CppVerdict {
    _p: [u8; 0],
}

extern "C" {
    fn pal_verdict_new(c: *const VCfg) -> *mut CppVerdict;
    fn pal_verdict_free(p: *mut CppVerdict);
    fn pal_verdict_reset(p: *mut CppVerdict);
    fn pal_verdict_new_session(p: *mut CppVerdict);
    fn pal_verdict_landed(p: *mut CppVerdict, now_ms: f64);
    fn pal_verdict_remap(p: *mut CppVerdict, rung: c_int);
    #[allow(clippy::too_many_arguments)]
    fn pal_verdict_window(
        p: *mut CppVerdict,
        now_ms: f64,
        cards: *const VCard,
        n: usize,
        loss: f64,
        recovered: u32,
        rung: c_int,
        out: *mut VOut,
    );
}

fn cfgs(r: &mut Rng) -> (HopCfg, BusyCfg) {
    let mut h = HopCfg::default();
    let mut b = BusyCfg::default();
    if r.chance(0.5) {
        h.persist = 1 + r.below(3) as i32;
        h.verdict.occupied_windows = 1 + r.below(8) as i32;
        h.verdict.detect_windows = 2 + r.below(7) as i32;
        h.verdict.detect_impaired = 1 + r.below(h.verdict.detect_windows as u64) as i32;
        b.blocked_pct = 10.0 + r.below(50) as f64;
        h.verdict.detect_pct = 1.0 + r.below(b.blocked_pct as u64) as f64;
        h.verdict.recovered_min = r.below(12) as i32;
    }
    (h, b)
}

fn card(r: &mut Rng, regime: u64, h: &HopCfg, b: &BusyCfg) -> VerdictCardIn {
    let busy = match regime {
        0 => r.unit() * 4.0,
        1 => h.verdict.detect_pct + (r.unit() - 0.5) * 6.0,
        2 => b.blocked_pct + (r.unit() - 0.5) * 10.0,
        _ => r.unit() * 100.0,
    };
    VerdictCardIn {
        valid: !r.chance(0.08),
        busy_valid: !r.chance(0.2),
        snr_valid: !r.chance(0.1),
        rssi_dbm: if r.chance(0.15) { -85.0 } else { -60.0 } + r.unit() * 6.0,
        snr_db: if r.chance(0.15) { 8.0 } else { 28.0 } + r.unit() * 4.0,
        nhm_busy_pct: busy.max(0.0),
        own_air_pct: r.unit() * 8.0,
        ..Default::default()
    }
}

fn to_c(c: &VerdictCardIn) -> VCard {
    VCard {
        valid: c.valid.into(),
        snr_valid: c.snr_valid.into(),
        busy_valid: c.busy_valid.into(),
        rssi_dbm: c.rssi_dbm,
        snr_db: c.snr_db,
        nhm_busy_pct: c.nhm_busy_pct,
        own_air_pct: c.own_air_pct,
    }
}

#[test]
fn rust_verdict_matches_mabur_on_random_scripts() {
    let mut r = Rng(0x9E37_79B9_7F4A_7C15);
    let mut interfered = [0usize; 2];
    for script in 0..300 {
        let (h, b) = cfgs(&mut r);
        let vc = VCfg {
            window_ms: h.window_ms,
            persist: h.persist,
            blocked_pct: b.blocked_pct,
            loss_pct: h.verdict.loss_pct,
            recovered_x: h.verdict.recovered_x,
            recovered_min: h.verdict.recovered_min,
            weak_rssi_dbm: h.verdict.weak_rssi_dbm,
            weak_snr_db: h.verdict.weak_snr_db,
            occupied_windows: h.verdict.occupied_windows,
            detect_pct: h.verdict.detect_pct,
            detect_windows: h.verdict.detect_windows,
            detect_impaired: h.verdict.detect_impaired,
        };
        let cpp = unsafe { pal_verdict_new(&vc) };
        let mut rs = HopVerdict::new(h.clone(), b.clone());
        let n_cards = 1 + r.below(3) as usize;
        let mut t = 0.0;
        let mut regime = 0;
        let mut rung = 5;
        for step in 0..400 {
            if r.chance(0.04) {
                regime = r.below(4);
            }
            match r.below(200) {
                0 => {
                    rs.reset();
                    unsafe { pal_verdict_reset(cpp) };
                }
                1 => {
                    rs.new_session();
                    unsafe { pal_verdict_new_session(cpp) };
                }
                2..=4 => {
                    rs.landed(t);
                    unsafe { pal_verdict_landed(cpp, t) };
                }
                5 => {
                    let k = r.below(6) as i32;
                    rs.remap_ref_rung(k);
                    unsafe { pal_verdict_remap(cpp, k) };
                }
                _ => {}
            }
            let cards: Vec<VerdictCardIn> =
                (0..n_cards).map(|_| card(&mut r, regime, &h, &b)).collect();
            let link = VerdictLinkIn {
                pre_fec_loss: if r.chance(0.3) { r.unit() * 0.08 } else { 0.0 },
                recovered: if r.chance(0.2) {
                    r.below(120) as u32
                } else {
                    r.below(25) as u32
                },
            };
            if r.chance(0.1) {
                rung = r.below(6) as i32;
            }
            let o = rs.window(t, &cards, &link, rung);
            let cc: Vec<VCard> = cards.iter().map(to_c).collect();
            let mut co = VOut::default();
            unsafe {
                pal_verdict_window(
                    cpp,
                    t,
                    cc.as_ptr(),
                    cc.len(),
                    link.pre_fec_loss,
                    link.recovered,
                    rung,
                    &mut co,
                )
            };
            let ro = VOut {
                v: o.v as c_int,
                evidence: o.evidence.into(),
                ref_rung: o.ref_rung,
                trigger: o.trigger.into(),
                ref_frozen: o.ref_frozen.into(),
                first_blocked: o.first_blocked.into(),
                t_start_ms: o.t_start_ms,
                t_ms: o.t_ms,
            };
            assert_eq!(ro, co, "script {script} step {step}");
            if co.v == 2 {
                interfered[usize::from(co.evidence & 4 != 0)] += 1;
            }
            t += h.window_ms as f64 * (0.8 + r.unit() * 0.4);
        }
        unsafe { pal_verdict_free(cpp) };
    }
    // Both routes to `interfered` (occupied, detected) were walked.
    assert!(interfered[0] > 100 && interfered[1] > 100, "{interfered:?}");
}
