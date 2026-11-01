//! mabur's tests/test_cal_session.cpp, minus the channel-pick cases (the
//! pick is not ported).

use std::path::{Path, PathBuf};

use pal_wire::cal::{
    build_payload, park_rel_for, parse_payload, AckResult, Applied, ApplyCmd, CalMsg, FrameInfo,
    Kind, SweepAck, SweepCmd, SweepDone, SweepSpec, FLAG_LDPC, FLAG_STBC, PAYLOAD_LEN, REL_MIN,
    WALL_LEAVE,
};
use pal_wire::rc::TagCtx;

use super::analysis::*;
use super::log::CalLog;
use super::plan::*;
use super::session::*;

const FLAGS: u8 = FLAG_LDPC | FLAG_STBC;
const POWER_AIR: usize = 26 + PAYLOAD_LEN + 4;
const ROWS: i32 = SWEEP_HI - SWEEP_LO + 1;
const WALLS: [i32; 8] = [20, 10, 5, 0, -5, -10, -20, NO_WALL];
const LEAVE: i32 = WALL_LEAVE as i32;

fn sp(kind: Kind, bw: u8) -> SweepSpec {
    SweepSpec { kind, bw }
}

fn session() -> CalSession {
    let mut s = CalSession::new(CalSessionCfg::default(), CalLog::none());
    s.set_peer(true, true, TagCtx::default());
    s
}

fn start(s: &mut CalSession, nonce: u32, sweeps: &[SweepSpec], now: u64) {
    s.start(nonce, sweeps.to_vec(), now, 0, Pick::Pinned)
        .unwrap();
}

/// Every frame goes through the real stamp codec, as the RX path does.
fn stamp(k: Kind, rate: u8, idx: i32, bw: u8, flags: u8, seq: u32) -> FrameInfo {
    parse_payload(&build_payload(
        k,
        rate,
        idx as i8,
        bw,
        flags,
        seq,
        PAYLOAD_LEN,
    ))
    .unwrap()
}

fn sweep(m: Option<CalMsg>) -> Option<SweepCmd> {
    match m {
        Some(CalMsg::Sweep(c)) => Some(c),
        _ => None,
    }
}
fn apply(m: Option<CalMsg>) -> Option<ApplyCmd> {
    match m {
        Some(CalMsg::Apply(a)) => Some(a),
        _ => None,
    }
}
fn abort_nonce(m: &Option<CalMsg>) -> Option<u32> {
    match m {
        Some(CalMsg::Abort(a)) => Some(a.nonce),
        _ => None,
    }
}

fn ack_r(s: &mut CalSession, nonce: u32, seq: u8, now: u64, flags: u8, result: AckResult) {
    let a = SweepAck {
        nonce,
        seq,
        result,
        start_in_ms: 0,
        flags,
    };
    s.on_mav(&CalMsg::Ack(a), now);
}
fn ack(s: &mut CalSession, nonce: u32, seq: u8, now: u64) {
    ack_r(s, nonce, seq, now, FLAGS, AckResult::Accepted);
}
fn applied(s: &mut CalSession, nonce: u32, seq: u8, bw: u8, ok: bool, reason: &str, now: u64) {
    let a = Applied {
        nonce,
        seq,
        bw,
        ok,
        reason: reason.into(),
    };
    s.on_mav(&CalMsg::Applied(a), now);
}
fn frame(s: &mut CalSession, card: usize, f: FrameInfo, crc_ok: bool, air: usize, now: u64) {
    s.on_cal_frame(card, &f, -60, crc_ok, air, now);
}

/// Rate r delivers `per_cell` frames in every cell up to walls[r], nothing
/// above; NO_WALL delivers nothing.
fn feed_power(s: &mut CalSession, bw: u8, walls: &[i32; 8], now: u64, flags: u8, per_cell: u32) {
    for r in 0..8u8 {
        for idx in SWEEP_LO..=SWEEP_HI {
            if walls[r as usize] == NO_WALL || idx > walls[r as usize] {
                continue;
            }
            for k in 0..per_cell {
                frame(
                    s,
                    0,
                    stamp(Kind::Power, r, idx, bw, flags, k),
                    true,
                    POWER_AIR,
                    now,
                );
            }
        }
    }
}

fn power_done(nonce: u32, seq: u8, per_cell: u32) -> CalMsg {
    CalMsg::Done(SweepDone {
        nonce,
        seq,
        offered: [ROWS as u32 * per_cell; 8],
        blocked_pct: [0; 8],
    })
}

/// One power sweep from its SweepCmd to the first ApplyCmd.
fn run_power(
    s: &mut CalSession,
    nonce: u32,
    seq: u8,
    bw: u8,
    walls: &[i32; 8],
    t: u64,
) -> ApplyCmd {
    let cmd = sweep(s.due_mav(t)).unwrap();
    assert!(cmd.kind == Kind::Power && cmd.bw == bw && cmd.seq == seq);
    ack(s, nonce, seq, t + 10);
    feed_power(s, bw, walls, t + 20, FLAGS, u32::from(POWER_FRAMES));
    s.on_mav(&power_done(nonce, seq, u32::from(POWER_FRAMES)), t + 30);
    let a = apply(s.due_mav(t + 40)).unwrap();
    assert_eq!(a.seq, seq);
    a
}

fn fresh(name: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("pal-calsess-{}-{name}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    d
}
fn log_lines(d: &Path) -> Vec<String> {
    std::fs::read_to_string(d.join("cal.log"))
        .unwrap_or_default()
        .lines()
        .map(str::to_string)
        .collect()
}

fn verify_close(at: u64, cells: usize) -> u64 {
    at + u64::from(verify_duration_ms(cells)) + CalSessionCfg::default().phase_slack_ms
}

#[test]
fn start_sorts_and_dedups_the_sweep_list() {
    let mut s = session();
    let list = [
        sp(Kind::Capacity, 40),
        sp(Kind::Power, 20),
        sp(Kind::Capacity, 20),
        sp(Kind::Power, 40),
        sp(Kind::Power, 20),
        sp(Kind::Capacity, 40),
    ];
    start(&mut s, 1, &list, 0);
    assert_eq!(
        s.sweeps(),
        [
            sp(Kind::Power, 20),
            sp(Kind::Capacity, 20),
            sp(Kind::Power, 40),
            sp(Kind::Capacity, 40)
        ]
    );
    assert_eq!((s.snapshot().sweep_n, s.snapshot().sweep_i), (4, 0));
    let cmd = sweep(s.due_mav(0)).unwrap();
    assert_eq!(cmd, make_power_sweep(1, 0, 20));

    let mut t = session();
    assert!(t.start(2, vec![], 0, 0, Pick::Pinned).is_err());
    assert!(t
        .start(2, vec![sp(Kind::Verify, 20)], 0, 0, Pick::Pinned)
        .is_err());
    assert!(t
        .start(2, vec![sp(Kind::Power, 30)], 0, 0, Pick::Pinned)
        .is_err());
    assert_eq!(t.state(), State::Idle);
}

#[test]
fn start_refuses_without_capability_or_link() {
    let mut s = CalSession::new(CalSessionCfg::default(), CalLog::none());
    let one = vec![sp(Kind::Power, 20)];
    s.set_peer(false, true, TagCtx::default());
    assert!(s
        .start(1, one.clone(), 0, 0, Pick::Pinned)
        .unwrap_err()
        .contains("link"));
    s.set_peer(true, false, TagCtx::default());
    assert!(s
        .start(1, one.clone(), 0, 0, Pick::Pinned)
        .unwrap_err()
        .contains("CAP_CALIBRATE"));
    s.set_peer(true, true, TagCtx::default());
    s.start(1, one.clone(), 0, 0, Pick::Pinned).unwrap();
    assert!(s.start(2, one, 10, 0, Pick::Pinned).is_err());
}

#[test]
fn the_pick_word_and_channel_reach_the_snapshot() {
    let mut s = session();
    s.start(7, vec![sp(Kind::Power, 20)], 0, 149, Pick::Pinned)
        .unwrap();
    assert_eq!(s.state(), State::AwaitAck);
    assert_eq!(
        (s.snapshot().pick_note.as_str(), s.snapshot().channel),
        ("pinned", 149)
    );
    assert!(sweep(s.due_mav(0)).is_some());
    let mut t = session();
    t.start(8, vec![sp(Kind::Power, 20)], 0, 36, Pick::OneCard)
        .unwrap();
    assert_eq!(t.snapshot().pick_note, "onecard");
}

#[test]
fn sweep_command_repeats_until_acked_then_silence_opens() {
    let cfg = CalSessionCfg::default();
    let mut s = session();
    start(&mut s, 42, &[sp(Kind::Power, 20)], 0);
    assert!(sweep(s.due_mav(0)).is_some());
    assert!(s.due_mav(100).is_none());
    assert!(sweep(s.due_mav(200)).is_some());
    assert!(!s.radio_silent(250));
    ack(&mut s, 42, 1, 280); // another sweep's ack
    assert_eq!(s.state(), State::AwaitAck);
    ack(&mut s, 41, 0, 290); // another run's ack
    assert_eq!(s.state(), State::AwaitAck);
    let a = SweepAck {
        nonce: 42,
        seq: 0,
        result: AckResult::Accepted,
        start_in_ms: 200,
        flags: FLAGS,
    };
    s.on_mav(&CalMsg::Ack(a), 300);
    assert_eq!(s.state(), State::Sweep);
    assert!(s.due_mav(400).is_none());
    assert!(s.radio_silent(400));
    let end = 300 + 200 + u64::from(sweep_duration_ms(&make_power_sweep(42, 0, 20)));
    assert!(s.due_mav(end - 1).is_none());
    assert_eq!(s.state(), State::Sweep);
    assert!(s.due_mav(end).is_none());
    assert_eq!(s.state(), State::AwaitDone);
    assert!(s.radio_silent(end + cfg.phase_slack_ms - 1));
    assert!(!s.radio_silent(end + cfg.phase_slack_ms));
    s.due_mav(end + cfg.phase_slack_ms);
    assert_ne!(s.state(), State::AwaitDone);
    assert!(!s.radio_silent(end + cfg.phase_slack_ms));
}

#[test]
fn a_frame_with_the_wrong_width_or_flags_is_not_tallied() {
    let mut s = session();
    start(&mut s, 12, &[sp(Kind::Power, 20)], 0);
    assert!(s.due_mav(0).is_some());
    ack(&mut s, 12, 0, 10);
    frame(
        &mut s,
        0,
        stamp(Kind::Power, 0, 0, 40, FLAGS, 0),
        true,
        POWER_AIR,
        20,
    );
    frame(
        &mut s,
        0,
        stamp(Kind::Power, 0, 0, 20, FLAG_STBC, 0),
        true,
        POWER_AIR,
        20,
    );
    frame(
        &mut s,
        0,
        stamp(Kind::Capacity, 0, 0, 20, FLAGS, 0),
        true,
        POWER_AIR,
        20,
    );
    frame(
        &mut s,
        0,
        stamp(Kind::Verify, 0, 0, 20, FLAGS, 0),
        true,
        POWER_AIR,
        20,
    );
    assert_eq!(s.cell_received(0, 0, 0), 0);
    assert_eq!(s.snapshot().stamp_mismatch, 4);
    frame(
        &mut s,
        0,
        stamp(Kind::Power, 0, 0, 20, FLAGS, 0),
        true,
        POWER_AIR,
        20,
    );
    assert_eq!(s.cell_received(0, 0, 0), 1);
    assert_eq!(s.snapshot().stamp_mismatch, 4);
    frame(
        &mut s,
        1,
        stamp(Kind::Power, 0, 0, 20, FLAGS, 0),
        false,
        POWER_AIR,
        20,
    );
    assert_eq!(s.cell_received(0, 0, 1), 0);
}

#[test]
fn done_closes_the_window_early() {
    let mut s = session();
    start(&mut s, 13, &[sp(Kind::Power, 20)], 0);
    assert!(s.due_mav(0).is_some());
    ack(&mut s, 13, 0, 10);
    feed_power(&mut s, 20, &WALLS, 20, FLAGS, 20);
    assert!(s.radio_silent(1000));
    s.on_mav(&power_done(99, 0, 20), 1000);
    assert!(s.radio_silent(1000));
    s.on_mav(&power_done(13, 0, 20), 1000);
    assert!(!s.radio_silent(1000));
    assert_eq!(s.state(), State::Apply);
    assert!(apply(s.due_mav(1000)).is_some());
}

#[test]
fn power_sweep_yields_an_apply_with_leave_for_undetermined() {
    let mut s = session();
    start(&mut s, 14, &[sp(Kind::Power, 20)], 0);
    let a = run_power(&mut s, 14, 0, 20, &WALLS, 0);
    assert!(a.nonce == 14 && a.bw == 20 && a.has_walls && !a.has_eff);
    for (w, want) in a.walls.iter().zip(&WALLS).take(7) {
        assert_eq!(i32::from(*w), *want);
    }
    assert_eq!(a.walls[7], WALL_LEAVE);
    assert_eq!(a.legacy, 20);
    let snap = s.snapshot();
    assert!(snap.width[0].have_walls);
    assert_ne!(snap.width[0].walls[7].flags & FLAG_UNDETERMINED, 0);
    assert_eq!(snap.width[0].walls[0].wall, 20);
    assert!(s.due_mav(100).is_none());
    assert!(apply(s.due_mav(240)).is_some());
}

#[test]
fn expected_per_cell_comes_from_the_done() {
    let mut s = session();
    start(&mut s, 15, &[sp(Kind::Power, 20)], 0);
    assert!(s.due_mav(0).is_some());
    ack(&mut s, 15, 0, 10);
    feed_power(&mut s, 20, &WALLS, 20, FLAGS, 10);
    s.on_mav(&power_done(15, 0, 10), 30);
    let a = apply(s.due_mav(40)).unwrap();
    assert_eq!((a.walls[0], a.walls[6]), (20, -20));
}

#[test]
fn applied_ok_opens_verify_and_tallies_by_rate() {
    let mut s = session();
    start(&mut s, 16, &[sp(Kind::Power, 20)], 0);
    let a = run_power(&mut s, 16, 0, 20, &WALLS, 0);
    let parks = park_rel_for(&a.walls, 1.0);
    applied(&mut s, 16, 0, 40, true, "", 100); // other width
    assert_eq!(s.state(), State::Apply);
    applied(&mut s, 16, 0, 20, true, "", 100);
    assert_eq!(s.state(), State::Verify);
    assert!(s.radio_silent(100));
    assert!(s.due_mav(400).is_none());
    for k in 0..20 {
        frame(
            &mut s,
            0,
            stamp(Kind::Verify, 0, parks[0], 20, FLAGS, k),
            true,
            POWER_AIR,
            200,
        );
    }
    for k in 0..10 {
        let f = stamp(Kind::Verify, 1, parks[1] + 1, 20, FLAGS, k);
        frame(&mut s, 1, f, true, POWER_AIR, 200);
    }
    frame(
        &mut s,
        0,
        stamp(Kind::Verify, 2, parks[2], 40, FLAGS, 0),
        true,
        POWER_AIR,
        200,
    );
    assert_eq!(s.snapshot().stamp_mismatch, 1);
    let close = verify_close(100, 7);
    assert!(s.radio_silent(close - 1));
    s.due_mav(close);
    assert!(!s.radio_silent(close));
    assert_eq!(s.state(), State::Done);
    let w = &s.snapshot().width[0];
    assert!(w.applied);
    assert_eq!(
        (
            w.verify_pct[0],
            w.verify_pct[1],
            w.verify_pct[2],
            w.verify_pct[7]
        ),
        (100, 50, 0, -1)
    );
}

#[test]
fn refused_apply_fails_with_the_drones_reason() {
    let mut s = session();
    start(
        &mut s,
        17,
        &[sp(Kind::Power, 20), sp(Kind::Capacity, 20)],
        0,
    );
    run_power(&mut s, 17, 0, 20, &WALLS, 0);
    applied(&mut s, 17, 0, 20, false, "config line not found", 100);
    assert_eq!(s.state(), State::Failed);
    let snap = s.snapshot();
    assert!(snap.fail_reason.contains("config line not found"));
    assert!(!snap.width[0].applied);
    assert_eq!(snap.width[0].apply_reason, "config line not found");
    assert!(!s.radio_silent(100));
}

#[test]
fn capacity_sweep_yields_eff_and_zero_for_tx_bound() {
    let cfg = CalSessionCfg::default();
    let mut s = session();
    start(&mut s, 18, &[sp(Kind::Capacity, 20)], 0);
    let cmd = sweep(s.due_mav(0)).unwrap();
    assert!(cmd.kind == Kind::Capacity && cmd.dwell_ms == CAP_DWELL_MS);
    ack_r(&mut s, 18, 0, 10, FLAG_STBC, AckResult::Accepted);
    const AIR: usize = 1500;
    for r in 0..2 {
        for k in 0..900 {
            frame(
                &mut s,
                0,
                stamp(Kind::Capacity, r, 0, 20, FLAG_STBC, k),
                true,
                AIR,
                100,
            );
        }
    }
    s.due_mav(100);
    let d = SweepDone {
        nonce: 18,
        seq: 0,
        offered: [1000, 1000, 0, 0, 0, 0, 0, 0],
        blocked_pct: [80, 10, 0, 0, 0, 0, 0, 0],
    };
    s.on_mav(&CalMsg::Done(d), 200);
    let a = apply(s.due_mav(210)).unwrap();
    assert!(a.has_eff && !a.has_walls && a.bw == 20);
    let c0 = CapCell {
        offered: 1000,
        blocked_pct: 80,
        frames: [900, 0],
        air_bits: [900 * AIR as u64 * 8, 0],
        rssi_dbm: [-60, 0],
        have_rssi: [true, false],
        ..CapCell::default()
    };
    let r0 = analyze_capacity(&c0, 0, 20, u32::from(CAP_DWELL_MS), &cfg.th);
    assert_eq!(r0.flags & FLAG_TX_BOUND, 0);
    assert_eq!(a.eff_x1000[0], (r0.eff * 1000.0).round() as u16);
    assert!(a.eff_x1000[0] > 0);
    assert!(a.eff_x1000[1..].iter().all(|&e| e == 0));
    let w = &s.snapshot().width[0];
    assert!(w.have_eff);
    assert_ne!(w.cap[1].flags & FLAG_TX_BOUND, 0);
    applied(&mut s, 18, 0, 20, true, "", 300);
    assert_eq!(s.state(), State::Done);
    assert!(!s.radio_silent(300));
}

const CAP_AIR: usize = 1435;
const CAP_FRAMES: u32 = 1212;
const CAP_SPAN_MS: u64 = 2300;

fn cap_rx_ms(t0: u64, k: u32) -> u64 {
    t0 + CAP_SPAN_MS * u64::from(k) / u64::from(CAP_FRAMES - 1)
}
fn cap_done(nonce: u32, seq: u8, offered: u32) -> CalMsg {
    CalMsg::Done(SweepDone {
        nonce,
        seq,
        offered: [offered, 0, 0, 0, 0, 0, 0, 0],
        blocked_pct: [90, 0, 0, 0, 0, 0, 0, 0],
    })
}
fn p_row(d: &Path) -> String {
    log_lines(d)
        .into_iter()
        .rfind(|l| l.starts_with("P 0 "))
        .unwrap_or_default()
}
fn logged(name: &str) -> (PathBuf, CalSession) {
    let d = fresh(name);
    let mut s = CalSession::new(CalSessionCfg::default(), CalLog::open(&d));
    s.set_peer(true, true, TagCtx::default());
    (d, s)
}

#[test]
fn capacity_eff_is_timed_by_the_frames_receive_stamps() {
    let (d, mut s) = logged("rx_stamp");
    start(&mut s, 40, &[sp(Kind::Capacity, 20)], 0);
    assert!(sweep(s.due_mav(0)).is_some());
    ack_r(&mut s, 40, 0, 10, FLAG_STBC, AckResult::Accepted);
    for k in 0..CAP_FRAMES {
        let rx = cap_rx_ms(100, k);
        s.due_mav(if k == CAP_FRAMES - 10 {
            rx + 300
        } else {
            rx + 2
        });
        frame(
            &mut s,
            0,
            stamp(Kind::Capacity, 0, 0, 20, FLAG_STBC, k),
            true,
            CAP_AIR,
            rx,
        );
    }
    s.due_mav(2400);
    s.on_mav(&cap_done(40, 0, CAP_FRAMES), 2800);
    let a = apply(s.due_mav(2810)).unwrap();
    assert_eq!(a.eff_x1000[0], 930);
    drop(s);
    assert_eq!(p_row(&d), "P 0 1212 1212 13913760 930 90 0 2300");
}

#[test]
fn a_capacity_sweep_listens_through_the_drones_tx_queue_drain() {
    let cfg = CalSessionCfg::default();
    let (d, mut s) = logged("drain");
    start(
        &mut s,
        41,
        &[sp(Kind::Capacity, 20), sp(Kind::Power, 40)],
        0,
    );
    assert!(sweep(s.due_mav(0)).is_some());
    ack_r(&mut s, 41, 0, 10, FLAG_STBC, AckResult::Accepted);
    let mut done_sent = false;
    for k in 0..CAP_FRAMES {
        let rx = cap_rx_ms(100, k);
        if !done_sent && rx >= 2100 {
            s.on_mav(&cap_done(41, 0, CAP_FRAMES), rx);
            done_sent = true;
        }
        assert!(apply(s.due_mav(rx)).is_none());
        assert!(s.radio_silent(rx));
        frame(
            &mut s,
            0,
            stamp(Kind::Capacity, 0, 0, 20, FLAG_STBC, k),
            true,
            CAP_AIR,
            rx,
        );
    }
    assert!(done_sent);
    let tick = cap_rx_ms(100, CAP_FRAMES - 1) + 1;
    assert!(apply(s.due_mav(tick)).is_none());
    assert!(apply(s.due_mav(tick + cfg.tail_quiet_ms - 1)).is_none());
    let a = apply(s.due_mav(tick + cfg.tail_quiet_ms)).unwrap();
    assert_eq!(a.eff_x1000[0], 930);
    assert_eq!(s.snapshot().stamp_mismatch, 0);
    drop(s);
    assert_eq!(p_row(&d), "P 0 1212 1212 13913760 930 90 0 2300");
}

#[test]
fn a_stalled_core_loop_cannot_fake_a_quiet_tail() {
    let cfg = CalSessionCfg::default();
    let (d, mut s) = logged("stall");
    start(&mut s, 43, &[sp(Kind::Capacity, 20)], 0);
    assert!(sweep(s.due_mav(0)).is_some());
    ack_r(&mut s, 43, 0, 10, FLAG_STBC, AckResult::Accepted);
    const SPLIT: u32 = 1180;
    for k in 0..SPLIT {
        let rx = cap_rx_ms(100, k);
        s.due_mav(rx);
        frame(
            &mut s,
            0,
            stamp(Kind::Capacity, 0, 0, 20, FLAG_STBC, k),
            true,
            CAP_AIR,
            rx,
        );
    }
    let seen = cap_rx_ms(100, SPLIT - 1);
    s.on_mav(&cap_done(43, 0, CAP_FRAMES), seen);
    assert!(apply(s.due_mav(seen + 80)).is_none());
    for k in SPLIT..CAP_FRAMES {
        let rx = cap_rx_ms(100, k);
        assert!(rx < seen + 80);
        frame(
            &mut s,
            0,
            stamp(Kind::Capacity, 0, 0, 20, FLAG_STBC, k),
            true,
            CAP_AIR,
            rx,
        );
    }
    let tick = seen + 85;
    assert!(apply(s.due_mav(tick)).is_none());
    assert!(apply(s.due_mav(tick + cfg.tail_quiet_ms - 1)).is_none());
    let a = apply(s.due_mav(tick + cfg.tail_quiet_ms)).unwrap();
    assert_eq!(a.eff_x1000[0], 930);
    assert_eq!(s.snapshot().stamp_mismatch, 0);
    drop(s);
    assert_eq!(p_row(&d), "P 0 1212 1212 13913760 930 90 0 2300");
}

#[test]
fn a_power_sweep_still_closes_at_its_done() {
    let mut s = session();
    start(&mut s, 42, &[sp(Kind::Power, 20)], 0);
    assert!(s.due_mav(0).is_some());
    ack(&mut s, 42, 0, 10);
    feed_power(&mut s, 20, &WALLS, 1000, FLAGS, 20);
    s.on_mav(&power_done(42, 0, 20), 1000);
    assert_eq!(s.state(), State::Apply);
}

fn cap_run(s: &mut CalSession, nonce: u32, got: &[u32], offered: [u32; 8], blocked: [u8; 8]) {
    start(s, nonce, &[sp(Kind::Capacity, 20)], 0);
    assert!(sweep(s.due_mav(0)).is_some());
    ack_r(s, nonce, 0, 10, FLAG_STBC, AckResult::Accepted);
    for (r, &n) in got.iter().enumerate() {
        for k in 0..n {
            frame(
                s,
                0,
                stamp(Kind::Capacity, r as u8, 0, 20, FLAG_STBC, k),
                true,
                1500,
                100,
            );
        }
    }
    s.due_mav(100);
    let d = SweepDone {
        nonce,
        seq: 0,
        offered,
        blocked_pct: blocked,
    };
    s.on_mav(&CalMsg::Done(d), 200);
}

#[test]
fn a_capacity_point_that_lost_over_half_its_frames_is_not_written() {
    let mut s = session();
    cap_run(
        &mut s,
        19,
        &[10, 499, 600],
        [1000, 1000, 1000, 0, 0, 0, 0, 0],
        [80, 80, 80, 0, 0, 0, 0, 0],
    );
    let a = apply(s.due_mav(210)).unwrap();
    assert!(a.has_eff);
    assert!(a.eff_x1000[0] == 0 && a.eff_x1000[1] == 0 && a.eff_x1000[2] > 0);
    let w = &s.snapshot().width[0];
    assert!(w.eff_x1000[0] == 0 && w.eff_x1000[1] == 0 && w.eff_x1000[2] > 0);
    let both = FLAG_LOST | FLAG_LOSSY;
    assert_eq!(w.cap[0].flags & both, both);
    assert_eq!(w.cap[1].flags & both, both);
    assert_eq!(w.cap[2].flags & both, FLAG_LOSSY);
}

#[test]
fn a_capacity_sweep_that_lost_every_point_writes_nothing() {
    let mut s = session();
    cap_run(&mut s, 20, &[5], [1000; 8], [80; 8]);
    assert!(s.snapshot().width[0].eff_x1000.iter().all(|&e| e == 0));
}

#[test]
fn a_receive_stamp_older_than_the_last_tick_fires_no_timeout() {
    let mut s = session();
    start(&mut s, 21, &[sp(Kind::Power, 20)], 1000);
    assert!(sweep(s.due_mav(1005)).is_some());
    s.on_mav(&CalMsg::Done(SweepDone::default()), 999);
    assert_eq!(s.state(), State::AwaitAck);
    frame(
        &mut s,
        0,
        stamp(Kind::Verify, 0, 0, 20, FLAGS, 0),
        true,
        POWER_AIR,
        998,
    );
    assert_eq!(s.state(), State::AwaitAck);
    ack(&mut s, 21, 0, 2000);
    feed_power(&mut s, 20, &WALLS, 2010, FLAGS, 20);
    s.on_mav(&power_done(21, 0, 20), 2020);
    assert!(apply(s.due_mav(2030)).is_some());
    assert_eq!(s.state(), State::Apply);
    ack(&mut s, 0, 0, 2001);
    assert_eq!(s.state(), State::Apply);
}

#[test]
fn done_sends_three_aborts_so_the_drone_resumes_at_once() {
    let mut s = session();
    start(&mut s, 35, &[sp(Kind::Power, 20)], 0);
    run_power(&mut s, 35, 0, 20, &WALLS, 0);
    applied(&mut s, 35, 0, 20, true, "", 100);
    let close = verify_close(100, 7);
    let mut at = vec![];
    for t in (close..close + 2000).step_by(50) {
        let m = s.due_mav(t);
        if let Some(n) = abort_nonce(&m) {
            assert_eq!(n, 35);
            at.push(t);
        }
        assert!(sweep(m).is_none());
    }
    assert_eq!(s.state(), State::Done);
    assert_eq!(at.len(), 3);
    assert_eq!((at[1] - at[0], at[2] - at[1]), (200, 200));
}

#[test]
fn a_repeated_done_after_analyze_is_ignored() {
    let mut s = session();
    start(
        &mut s,
        37,
        &[sp(Kind::Power, 20), sp(Kind::Capacity, 20)],
        0,
    );
    let a = run_power(&mut s, 37, 0, 20, &WALLS, 0);
    s.on_mav(&power_done(37, 0, 20), 50);
    assert_eq!(s.state(), State::Apply);
    let again = apply(s.due_mav(240)).unwrap();
    assert_eq!(again, a);
    applied(&mut s, 37, 0, 20, true, "", 250);
    assert_eq!(s.state(), State::Verify);
    s.on_mav(&power_done(37, 0, 20), 260);
    assert_eq!(s.state(), State::Verify);
    assert!(s.radio_silent(260));
    let next = verify_close(250, 7);
    assert_eq!(sweep(s.due_mav(next)).unwrap().seq, 1);
    s.on_mav(&power_done(37, 0, 20), next + 10);
    assert_eq!(s.state(), State::AwaitAck);
    assert!(s.snapshot().width[0].have_walls);
    assert_eq!(s.snapshot().width[0].walls[0].wall, 20);
}

#[test]
fn a_stale_applied_for_the_power_seq_does_not_satisfy_the_capacity_apply() {
    let mut s = session();
    start(
        &mut s,
        36,
        &[sp(Kind::Power, 20), sp(Kind::Capacity, 20)],
        0,
    );
    run_power(&mut s, 36, 0, 20, &WALLS, 0);
    applied(&mut s, 36, 0, 20, true, "", 100);
    let t = verify_close(100, 7);
    let cmd = sweep(s.due_mav(t)).unwrap();
    assert!(cmd.kind == Kind::Capacity && cmd.seq == 1);
    ack(&mut s, 36, 1, t + 10);
    for k in 0..100 {
        frame(
            &mut s,
            0,
            stamp(Kind::Capacity, 0, 0, 20, FLAGS, k),
            true,
            1500,
            t + 20,
        );
    }
    s.due_mav(t + 20);
    let d = SweepDone {
        nonce: 36,
        seq: 1,
        offered: [100; 8],
        blocked_pct: [90; 8],
    };
    s.on_mav(&CalMsg::Done(d), t + 100);
    let a = apply(s.due_mav(t + 110)).unwrap();
    assert!(a.seq == 1 && a.bw == 20 && a.has_eff);
    applied(&mut s, 36, 0, 20, true, "", t + 120);
    assert_eq!(s.state(), State::Apply);
    assert!(apply(s.due_mav(t + 310)).is_some());
    applied(&mut s, 36, 1, 20, true, "", t + 320);
    assert_eq!(s.state(), State::Done);
}

#[test]
fn a_crc_bad_frame_is_not_an_implicit_ack() {
    let mut s = session();
    start(&mut s, 37, &[sp(Kind::Power, 20)], 0);
    assert!(s.due_mav(0).is_some());
    frame(
        &mut s,
        0,
        stamp(Kind::Power, 0, SWEEP_LO, 20, FLAG_LDPC, 0),
        false,
        POWER_AIR,
        50,
    );
    assert_eq!(s.state(), State::AwaitAck);
    assert!(sweep(s.due_mav(200)).is_some());
    frame(
        &mut s,
        0,
        stamp(Kind::Power, 0, SWEEP_LO, 20, FLAGS, 0),
        true,
        POWER_AIR,
        250,
    );
    assert_eq!(s.state(), State::Sweep);
    assert_eq!(s.snapshot().flags, FLAGS);
}

#[test]
fn capacity_without_done_writes_no_eff() {
    let cfg = CalSessionCfg::default();
    let mut s = session();
    start(&mut s, 19, &[sp(Kind::Capacity, 20)], 0);
    assert!(s.due_mav(0).is_some());
    ack_r(&mut s, 19, 0, 10, FLAG_STBC, AckResult::Accepted);
    for k in 0..900 {
        frame(
            &mut s,
            0,
            stamp(Kind::Capacity, 0, 0, 20, FLAG_STBC, k),
            true,
            1500,
            100,
        );
    }
    s.due_mav(100);
    let close =
        10 + u64::from(sweep_duration_ms(&make_capacity_sweep(19, 0, 20))) + cfg.phase_slack_ms;
    for t in (200..=close + 2000).step_by(100) {
        assert!(apply(s.due_mav(t)).is_none());
    }
    assert_eq!(s.state(), State::Done);
    let w = &s.snapshot().width[0];
    assert!(w.eff_unknown && !w.have_eff);
    assert!(w.eff_x1000.iter().all(|&e| e == 0));
}

#[test]
fn legacy_comes_from_the_20_table_when_both_are_swept() {
    let mut s = session();
    start(&mut s, 20, &[sp(Kind::Power, 40), sp(Kind::Power, 20)], 0);
    let a20 = run_power(&mut s, 20, 0, 20, &WALLS, 0);
    assert_eq!(a20.legacy, 20);
    applied(&mut s, 20, 0, 20, true, "", 100);
    let t = verify_close(100, 7);
    let w40 = [15, 10, 5, 0, -5, -10, -20, -30];
    let a40 = run_power(&mut s, 20, 1, 40, &w40, t);
    assert!(a40.bw == 40 && a40.walls[0] == 15 && a40.legacy == WALL_LEAVE);
    let mut u = session();
    start(&mut u, 21, &[sp(Kind::Power, 40)], 0);
    assert_eq!(run_power(&mut u, 21, 0, 40, &w40, 0).legacy, 15);
}

#[test]
fn ack_timeout_fails_without_an_apply() {
    let cfg = CalSessionCfg::default();
    let mut s = session();
    start(&mut s, 22, &[sp(Kind::Power, 20)], 0);
    let (mut applies, mut aborts) = (0, 0);
    for t in (0..=cfg.ack_timeout_ms + 1000).step_by(50) {
        let m = s.due_mav(t);
        aborts += i32::from(abort_nonce(&m).is_some());
        applies += i32::from(apply(m).is_some());
    }
    assert_eq!(s.state(), State::Failed);
    assert!(s.snapshot().fail_reason.contains("ack"));
    assert_eq!((applies, aborts), (0, 3));
    assert!(!s.radio_silent(cfg.ack_timeout_ms + 1000));
}

#[test]
fn refused_ack_fails_with_its_reason() {
    for (res, word) in [
        (AckResult::AnchorUnreadable, "anchor_unreadable"),
        (AckResult::Busy, "busy"),
        (AckResult::Unsupported, "unsupported"),
    ] {
        let mut s = session();
        start(&mut s, 23, &[sp(Kind::Power, 20)], 0);
        assert!(s.due_mav(0).is_some());
        ack_r(&mut s, 23, 0, 10, FLAGS, res);
        assert_eq!(s.state(), State::Failed);
        assert!(s.snapshot().fail_reason.contains(word));
        assert!(!s.radio_silent(10));
    }
}

#[test]
fn apply_timeout_fails() {
    let cfg = CalSessionCfg::default();
    let mut s = session();
    start(&mut s, 24, &[sp(Kind::Power, 20)], 0);
    run_power(&mut s, 24, 0, 20, &WALLS, 0);
    let mut applies = 1;
    for t in (50..40 + cfg.apply_timeout_ms).step_by(50) {
        applies += i32::from(apply(s.due_mav(t)).is_some());
    }
    assert_eq!(s.state(), State::Apply);
    assert!(applies >= 20);
    s.due_mav(40 + cfg.apply_timeout_ms);
    assert_eq!(s.state(), State::Failed);
    assert!(s.snapshot().fail_reason.contains("apply"));
}

#[test]
fn abort_emits_three_aborts_and_reopens_the_air() {
    let mut s = session();
    start(&mut s, 25, &[sp(Kind::Power, 20)], 0);
    assert!(s.due_mav(0).is_some());
    ack(&mut s, 25, 0, 10);
    assert!(s.radio_silent(100));
    s.abort("operator abort");
    assert!(!s.radio_silent(100));
    assert_eq!(s.state(), State::Idle);
    assert!(!s.running());
    assert_eq!(abort_nonce(&s.due_mav(110)), Some(25));
    assert!(s.due_mav(120).is_none());
    assert!(s.due_mav(309).is_none());
    assert!(abort_nonce(&s.due_mav(310)).is_some());
    assert!(s.due_mav(400).is_none());
    assert_eq!(abort_nonce(&s.due_mav(510)), Some(25));
    assert!(s.due_mav(710).is_none());
    assert!(s.due_mav(1000).is_none());
    assert_eq!(s.snapshot().fail_reason, "operator abort");
    ack(&mut s, 25, 0, 130);
    assert_eq!(s.state(), State::Idle);
}

#[test]
fn an_abort_before_anything_reached_the_drone_sends_nothing() {
    let mut s = session();
    start(&mut s, 27, &[sp(Kind::Power, 20)], 0);
    s.abort("operator abort");
    assert_eq!(s.state(), State::Idle);
    assert!(s.due_mav(10).is_none());
}

#[test]
fn implicit_ack_from_the_first_matching_frame() {
    let mut s = session();
    start(&mut s, 28, &[sp(Kind::Power, 20)], 0);
    assert!(s.due_mav(0).is_some());
    frame(
        &mut s,
        0,
        stamp(Kind::Capacity, 0, 0, 20, FLAG_STBC, 0),
        true,
        1500,
        50,
    );
    frame(
        &mut s,
        0,
        stamp(Kind::Power, 0, SWEEP_LO, 40, FLAG_STBC, 0),
        true,
        POWER_AIR,
        60,
    );
    assert_eq!(s.state(), State::AwaitAck);
    frame(
        &mut s,
        0,
        stamp(Kind::Power, 0, SWEEP_LO, 20, FLAG_STBC, 0),
        true,
        POWER_AIR,
        100,
    );
    assert_eq!(s.state(), State::Sweep);
    assert!(s.radio_silent(100));
    assert_eq!(s.snapshot().flags, FLAG_STBC);
    assert_eq!(s.cell_received(0, SWEEP_LO, 0), 1);
    assert!(s.due_mav(300).is_none());
    frame(
        &mut s,
        0,
        stamp(Kind::Power, 0, SWEEP_LO, 20, FLAGS, 0),
        true,
        POWER_AIR,
        110,
    );
    assert_eq!(s.cell_received(0, SWEEP_LO, 0), 1);
}

#[test]
fn verify_parks_match_the_drone_rule() {
    let cfg = CalSessionCfg {
        margin_db: 7.0, // 28 steps: pushes a low wall past REL_MIN
        ..CalSessionCfg::default()
    };
    let mut s = CalSession::new(cfg, CalLog::none());
    s.set_peer(true, true, TagCtx::default());
    start(&mut s, 29, &[sp(Kind::Power, 20)], 0);
    let mut walls = [NO_WALL; 8];
    walls[0] = SWEEP_LO;
    walls[1] = 30;
    let a = run_power(&mut s, 29, 0, 20, &walls, 0);
    assert_eq!(i32::from(a.walls[0]), SWEEP_LO);
    applied(&mut s, 29, 0, 20, true, "", 100);
    assert_eq!(s.state(), State::Verify);
    let drone = park_rel_for(&a.walls, cfg.margin_db);
    let w = &s.snapshot().width[0];
    assert_eq!(w.park, drone);
    assert_eq!((w.park[0], w.park[1], w.park[2]), (REL_MIN, 30 - 28, LEAVE));
    let close =
        100 + u64::from(verify_duration_ms(verify_cells(&drone).len())) + cfg.phase_slack_ms;
    assert!(s.radio_silent(close - 1));
    assert!(!s.radio_silent(close));
}

#[test]
fn snapshot_tracks_sweep_and_cell_progress() {
    let mut s = session();
    start(
        &mut s,
        30,
        &[sp(Kind::Capacity, 20), sp(Kind::Power, 20)],
        0,
    );
    assert!(s.due_mav(0).is_some());
    ack(&mut s, 30, 0, 10);
    let snap = s.snapshot();
    assert_eq!(snap.state, State::Sweep);
    assert_eq!((snap.sweep_i, snap.sweep_n), (0, 2));
    assert_eq!(snap.cur, sp(Kind::Power, 20));
    assert_eq!(
        (snap.cell_n, snap.cell_i, snap.flags),
        (8 * ROWS as u32, 0, FLAGS)
    );
    frame(
        &mut s,
        0,
        stamp(Kind::Power, 1, SWEEP_LO, 20, FLAGS, 0),
        true,
        POWER_AIR,
        20,
    );
    assert_eq!(s.snapshot().cell_i, ROWS as u32 + 1);
    frame(
        &mut s,
        0,
        stamp(Kind::Power, 1, SWEEP_LO + 1, 20, FLAGS, 0),
        true,
        POWER_AIR,
        30,
    );
    assert_eq!(s.snapshot().cell_i, ROWS as u32 + 2);
    s.on_mav(&power_done(30, 0, 20), 40);
    let cmd = sweep(s.due_mav(50)).unwrap();
    assert!(cmd.kind == Kind::Capacity && cmd.seq == 1);
    ack(&mut s, 30, 1, 60);
    let snap = s.snapshot();
    assert_eq!(
        (snap.sweep_i, snap.cur.kind, snap.cell_n),
        (1, Kind::Capacity, 8)
    );
    frame(
        &mut s,
        0,
        stamp(Kind::Capacity, 3, 0, 20, FLAGS, 0),
        true,
        1500,
        70,
    );
    assert_eq!(s.snapshot().cell_i, 4);
}

#[test]
fn tag_context_is_frozen_at_start() {
    let ctx = |vtx| TagCtx {
        vrx_nonce: 11,
        vtx_nonce: vtx,
        seq32: 0,
    };
    let mut s = CalSession::new(CalSessionCfg::default(), CalLog::none());
    s.set_peer(true, true, ctx(22));
    start(&mut s, 31, &[sp(Kind::Power, 20)], 0);
    s.set_peer(true, true, ctx(33));
    s.set_peer(false, true, ctx(33));
    assert_eq!((s.tag_ctx().vrx_nonce, s.tag_ctx().vtx_nonce), (11, 22));
    s.abort("test");
    s.set_peer(true, true, ctx(44));
    start(&mut s, 32, &[sp(Kind::Power, 20)], 10);
    assert_eq!(s.tag_ctx().vtx_nonce, 44);
}

#[test]
fn cal_log_gets_sweep_cell_wall_cap_verify_and_applied_rows() {
    let d = fresh("log");
    let mut s = CalSession::new(CalSessionCfg::default(), CalLog::open(&d));
    s.set_peer(true, true, TagCtx::default());
    s.start(
        77,
        vec![sp(Kind::Capacity, 20), sp(Kind::Power, 20)],
        0,
        36,
        Pick::Pinned,
    )
    .unwrap();
    run_power(&mut s, 77, 0, 20, &WALLS, 0);
    applied(&mut s, 77, 0, 20, true, "", 100);
    let parks = park_rel_for(&[20, 10, 5, 0, -5, -10, -20, WALL_LEAVE], 1.0);
    for k in 0..20 {
        frame(
            &mut s,
            0,
            stamp(Kind::Verify, 0, parks[0], 20, FLAGS, k),
            true,
            POWER_AIR,
            200,
        );
    }
    let t = verify_close(100, 7);
    assert_eq!(sweep(s.due_mav(t)).unwrap().kind, Kind::Capacity);
    ack(&mut s, 77, 1, t + 10);
    for k in 0..100 {
        frame(
            &mut s,
            0,
            stamp(Kind::Capacity, 0, 0, 20, FLAGS, k),
            true,
            1500,
            t + 20,
        );
    }
    s.due_mav(t + 20);
    let done = SweepDone {
        nonce: 77,
        seq: 1,
        offered: [100; 8],
        blocked_pct: [90; 8],
    };
    s.on_mav(&CalMsg::Done(done), t + 100);
    assert!(apply(s.due_mav(t + 110)).is_some());
    applied(&mut s, 77, 1, 20, true, "", t + 120);
    assert_eq!(s.state(), State::Done);

    let lines = log_lines(&d);
    let mut tags: Vec<String> = vec![];
    for l in &lines {
        let tag = l.split(' ').next().unwrap().to_string();
        if tags.last() != Some(&tag) {
            tags.push(tag);
        }
    }
    assert_eq!(
        tags,
        ["callog", "R", "S", "C", "W", "A", "V", "S", "P", "A"]
    );
    assert_eq!(lines[1], "R 77 1.00 36 pinned 1");
    assert_eq!(lines[2], "S 0 power 20 20");
    let count = |c: char| lines.iter().filter(|l| l.starts_with(c)).count();
    assert_eq!(count('C'), 8 * ROWS as usize);
    assert_eq!((count('W'), count('V'), count('P')), (8, 7, 8));
    assert!(lines.iter().any(|l| l == "S 1 capacity 20 2000"));
}

#[test]
fn null_log_sink_is_inert() {
    let mut s = session();
    start(&mut s, 34, &[sp(Kind::Power, 20)], 0);
    run_power(&mut s, 34, 0, 20, &WALLS, 0);
    applied(&mut s, 34, 0, 20, true, "", 100);
    s.due_mav(verify_close(100, 7));
    assert_eq!(s.state(), State::Done);
}
