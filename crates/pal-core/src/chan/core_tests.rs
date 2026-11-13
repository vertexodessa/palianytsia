//! mabur `tests/test_channel_core.cpp`, minus the relay cases (not
//! ported).

use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering::SeqCst};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

use pal_wire::rc::{self, DiscAck, Rcf};

use super::*;
use crate::chan::fake::{FakeCard, FakeClock};
use crate::chan::plan::MoveReason;
use crate::chan::radio::{nhm_period_4us, NhmBusyRead};
use crate::config::{load_config, Config};
use crate::ladder::LinkHealth;
use crate::vrx::{VrxCfg, VrxState};

fn bundle() -> Config {
    load_config(
        Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../third_party/mabur/gs/bundle/maburgs.default.toml"
        )),
        None,
    )
    .unwrap()
}

#[derive(Default)]
struct Rec {
    lines: Vec<String>,
    moves: Vec<MoveEvent>,
    hops: Vec<HopEvent>,
    dwells: Vec<(usize, ScoutDwell)>,
    picks: Vec<Option<u8>>,
    verdicts: usize,
}

impl ChannelSink for Rec {
    fn dwell(&mut self, _: f64, card: usize, d: &ScoutDwell) {
        self.dwells.push((card, *d));
    }
    fn pick(&mut self, _: f64, p: Option<u8>, _: u64, _: &[RankEntry], _: i32) {
        self.picks.push(p);
    }
    fn moved(&mut self, e: &MoveEvent) {
        self.moves.push(*e);
    }
    fn verdict(&mut self, _: f64, _: &VerdictOut, _: &[VerdictCardIn], _: &VerdictLinkIn) {
        self.verdicts += 1;
    }
    fn hop(&mut self, e: &HopEvent) {
        self.hops.push(e.clone());
    }
    fn log(&mut self, l: &str) {
        self.lines.push(l.to_string());
    }
}

/// N fake cards, the bundle config, a VrxController, a recording sink, an
/// injected clock, no threads.
struct Rig {
    clk: FakeClock,
    cfg: Config,
    cards: Vec<Arc<FakeCard>>,
    sink: Arc<Mutex<Rec>>,
    stored: Arc<Mutex<Vec<u8>>>,
    store_ok: Arc<AtomicBool>,
    vrx: VrxController,
    core: ChannelCore,
    agg: Aggregator,
    /// {may_send(0), may_send(1)} sampled inside the scout's sleep: the
    /// synchronous stand-in for the core thread reading the gate while the
    /// scout thread is mid-dwell.
    gate_obs: Arc<Mutex<Vec<(bool, bool)>>>,
    /// decode_one(): per-card next mac_seq.
    seq_next: Vec<u16>,
    /// Linked ticks feed link video (mabur Rig::video).
    video: bool,
}

fn junk_body(card: u8, seq: u16) -> crate::RxBody {
    crate::RxBody {
        card_id: card,
        mac_seq: seq,
        crc_ok: true,
        phy_valid: true,
        rssi: [38, 40],
        snr: [10, 25],
        mono_us: 1000 * u64::from(seq),
        // Not SBI, not RC: misroutes, still walks the seq.
        body: vec![0; 20],
        ..Default::default()
    }
}

impl Rig {
    fn new(n: usize, pinned: bool, start: u8) -> Self {
        Self::with_use(n, pinned, start, &[])
    }
    fn with_use(n: usize, pinned: bool, start: u8, use_: &[u8]) -> Self {
        Self::build(n, pinned, start, use_, 40)
    }
    /// `width` = the link's receiver width (cfg.width_mhz).
    fn build(n: usize, pinned: bool, start: u8, use_: &[u8], width: u8) -> Self {
        let mut cfg = bundle();
        cfg.radio.channels = vec![40, 64, 112, 144];
        cfg.radio.use_ = use_.to_vec();
        if pinned {
            cfg.radio.pin = Some(if start != 0 { start } else { 40 });
        }
        let start_ch = if start != 0 { start } else { 40 };
        let clk = FakeClock::default();
        let cards: Vec<Arc<FakeCard>> = (0..n)
            .map(|i| {
                let c = FakeCard::new(clk.clone(), start_ch);
                // The boot scout card opens at 20, every other at the link width.
                c.s().width_mhz = if i == n - 1 { 20 } else { width };
                c
            })
            .collect();
        let vrx = VrxController::new(VrxCfg::from_config(&cfg, start_ch, 0));
        let cc = ChannelCoreCfg {
            radio: cfg.radio.clone(),
            width_mhz: width,
            hop: cfg.hop.clone(),
            key: cfg.link.key,
            start_ch,
            threaded: false,
            store_name: "the-store".into(),
            ..Default::default()
        };
        let sink = Arc::new(Mutex::new(Rec::default()));
        let stored = Arc::new(Mutex::new(Vec::new()));
        let store_ok = Arc::new(AtomicBool::new(true));
        let gate_obs = Arc::new(Mutex::new(Vec::new()));
        let scout_slot: Arc<OnceLock<(Arc<Scout>, usize)>> = Arc::new(OnceLock::new());
        let (st, ok) = (stored.clone(), store_ok.clone());
        let (c1, c2, c3) = (clk.clone(), clk.clone(), clk.clone());
        let (obs, slot) = (gate_obs.clone(), scout_slot.clone());
        let core = ChannelCore::new(
            cc,
            cards
                .iter()
                .map(|c| c.clone() as Arc<dyn LinkCard>)
                .collect(),
            Box::new(sink.clone()),
            Box::new(move |ch| {
                st.lock().unwrap().push(ch);
                ok.load(SeqCst)
            }),
            Arc::new(move || c1.ms()),
            Arc::new(move || c2.ms() * 1000),
            Arc::new(move |ms| {
                c3.advance(ms as u64);
                // The gate while the scout owns its card (it always does
                // inside its own sleep).
                if let Some((s, sc)) = slot.get() {
                    let may = |card| !((card == *sc && !s.beaconing()) || s.quiet());
                    obs.lock().unwrap().push((may(0), may(1)));
                }
            }),
        );
        if let (Some(s), Some(sc)) = (core.scout(), core.scout_card()) {
            let _ = scout_slot.set((s.clone(), sc));
        }
        Rig {
            agg: Aggregator::new(&cfg.uep_layers(), 32, n, 0),
            clk,
            cfg,
            cards,
            sink,
            stored,
            store_ok,
            vrx,
            core,
            gate_obs,
            seq_next: Vec::new(),
            video: true,
        }
    }
    fn auto(n: usize) -> Self {
        Self::new(n, false, 0)
    }
    fn ms(&self) -> f64 {
        self.clk.ms() as f64
    }
    fn tick_tx(&mut self, session: bool, tx: usize) -> ChannelTickOut {
        // A linked tick carries link video, as on the air; video = false is
        // a session a DISC_ACK alone opened (the drone's encoder not up yet).
        if session && self.video {
            self.vrx.on_video(self.ms());
        }
        let inp = ChannelTickIn {
            now_ms: self.ms(),
            in_session: session,
            cal_running: false,
            tx_card: tx,
            agg: Some(&self.agg),
        };
        self.core.tick(&inp, &mut self.vrx)
    }
    /// One tick, then the clock steps 10 ms.
    fn tick(&mut self, session: bool) -> ChannelTickOut {
        let out = self.tick_tx(session, 0);
        self.clk.advance(10);
        out
    }
    fn rec(&self) -> std::sync::MutexGuard<'_, Rec> {
        self.sink.lock().unwrap()
    }
    fn has_line(&self, needle: &str) -> bool {
        self.rec().lines.iter().any(|l| l.contains(needle))
    }
    fn count_lines(&self, needle: &str) -> usize {
        self.rec()
            .lines
            .iter()
            .filter(|l| l.contains(needle))
            .count()
    }
    fn has_hop(&self, kind: &str) -> bool {
        self.rec().hops.iter().any(|e| e.kind == kind)
    }
    fn has_move(&self, r: MoveReason) -> bool {
        self.rec().moves.iter().any(|e| e.reason == r)
    }
    fn ack(&self, agreed: u8) -> DiscAck {
        DiscAck {
            vrx_nonce: self.vrx.rz_nonce(),
            vtx_nonce: 1,
            agreed_channel: agreed,
            seq: 1,
            ..Default::default()
        }
    }
    /// The DISC_ACK that opened the session arrived on `rx_ch`.
    fn session_opened_on(&mut self, rx_ch: u8, agreed: u8) {
        self.core.on_rc_body(rx_ch);
        let ack = self.ack(agreed);
        let now = self.ms();
        self.core.on_session_opened(&ack, now, &mut self.vrx);
    }
    /// The vrx to SESSION on op (a synthetic ack).
    fn link_up(&mut self) {
        let mut ack = self.ack(self.core.op());
        ack.chip_caps = rc::CAP_FRAME_WIRE;
        self.vrx.on_rc_frame(&rc::pack_disc_ack(&ack), self.ms());
        let healthy = LinkHealth {
            sample_valid: true,
            ..Default::default()
        };
        for _ in 0..200 {
            if self.vrx.link_state() == VrxState::Session {
                break;
            }
            self.clk.advance(10);
            self.vrx.on_video(self.ms());
            self.vrx.step(self.ms(), &healthy);
        }
        assert_eq!(self.vrx.link_state(), VrxState::Session);
    }
    /// One RCF built and "sent".
    fn pump_rcf(&mut self) -> Option<Rcf> {
        let healthy = LinkHealth {
            sample_valid: true,
            ..Default::default()
        };
        for _ in 0..20 {
            self.clk.advance(self.cfg.link.feedback_ms as u64);
            self.vrx.on_video(self.ms());
            let out = self.vrx.step(self.ms(), &healthy);
            self.tick_tx(true, 0);
            match out {
                Some(o) if !o.is_disc => {
                    self.core.note_sent(true, true);
                    return rc::parse_rcf(&o.frame);
                }
                _ => {}
            }
        }
        None
    }
    /// One decoded frame per card, walking its seq. A jammed window still
    /// decodes some of the drone's frames; the window fill gives a card
    /// that decoded nothing no busy reading at all. The body misroutes (not
    /// SBI, not RC): no FEC loss, no recovered symbols.
    fn decode_one(&mut self) {
        self.seq_next
            .resize(self.cards.len().max(self.seq_next.len()), 100);
        for i in 0..self.cards.len() {
            let seq = self.seq_next[i];
            self.seq_next[i] = seq.wrapping_add(1);
            self.agg.on_rx_body(&junk_body(i as u8, seq));
        }
    }
    /// The op's air held by a foreign transmitter: NHM reads 100 % busy on
    /// every card, armed at the sampler's op-window period, so a scout
    /// dwell (its own period) reads nothing. A card the test gave NHM air
    /// of its own (nhm_any_period) keeps it. Returns what to restore.
    fn op_jam(&self) -> Vec<(bool, NhmBusyRead)> {
        let mut full = NhmBusyRead {
            valid: true,
            period: nhm_period_4us(self.cfg.hop.window_ms - 10),
            ..Default::default()
        };
        full.buckets[11] = 100;
        self.cards
            .iter()
            .map(|c| {
                let mut s = c.s();
                let saved = (s.nhm_arm_ok, s.nhm_read);
                if !s.nhm_any_period {
                    s.nhm_arm_ok = true;
                    s.nhm_read = full;
                }
                saved
            })
            .collect()
    }
    fn op_unjam(&self, saved: Vec<(bool, NhmBusyRead)>) {
        for (c, (ok, rd)) in self.cards.iter().zip(saved) {
            let mut s = c.s();
            if !s.nhm_any_period {
                s.nhm_arm_ok = ok;
                s.nhm_read = rd;
            }
        }
    }
    /// The windows a blocked op needs on top of the old three-window
    /// trigger shape: `interfered` needs the op blocked for
    /// occupied_windows consecutive windows with a busy reading.
    /// A calibration round's air: op's NHM busy 25 % (below blocked_pct)
    /// on card 0 whatever period is armed; on the scout card 64 and 144
    /// read 25 %, 112 is the clean member at 5 %.
    fn cal_air(&self) {
        {
            let mut s = self.cards[0].s();
            s.nhm_arm_ok = true;
            s.nhm_any_period = true;
            s.nhm_read.valid = true;
            s.nhm_read.buckets[11] = 25;
            s.nhm_read.buckets[0] = 75;
        }
        let mut s = self.cards[1].s();
        s.nhm_busy_pct_on[64] = 25;
        s.nhm_busy_pct_on[112] = 5;
        s.nhm_busy_pct_on[144] = 25;
    }
    fn occ_lead(&self) -> usize {
        (self.cfg.hop.verdict.occupied_windows - 1).max(0) as usize
    }
    /// Interfered verdict windows: the op jammed (op_jam), foreign frames
    /// added alongside as context only. This rig's aggregator never decodes
    /// a real body, so the only route to `impaired` is occupancy: the first
    /// window arms NHM, the occupied_windows-th blocked one after it is the
    /// first interfered, the next the trigger (persist 2).
    fn interfere(&mut self, windows: usize) {
        let saved = self.op_jam();
        for _ in 0..windows {
            for c in &self.cards {
                c.s().fr.foreign += 40;
            }
            for _ in 0..self.cfg.hop.window_ms / 10 + 1 {
                self.vrx.on_video(self.ms());
                self.decode_one();
                self.tick_tx(true, 0);
                self.clk.advance(10);
            }
        }
        self.op_unjam(saved);
    }
    /// Interfered windows until the controller orders, bounded: the order
    /// waits on a burst ranking the candidates.
    fn interfere_until_order(&mut self) {
        for _ in 0..self.occ_lead() + 12 {
            if self.has_hop("order") {
                break;
            }
            self.interfere(1);
        }
        assert!(self.has_hop("order"));
    }
    /// An auto rig frozen on 40 without a hop. Two cards: no dwell ever
    /// completes, so max_ms freezes op in place. One card: link-up freezes
    /// it ("one-card linked").
    fn freeze_auto(&mut self) {
        if self.cards.len() >= 2 {
            self.cards.last().unwrap().s().retune_ok = false;
            for _ in 0..4000 {
                if !self.core.pick_open() {
                    break;
                }
                self.tick(false);
            }
            self.cards.last().unwrap().s().retune_ok = true;
        } else {
            self.link_up();
            for _ in 0..100 {
                if !self.core.pick_open() {
                    break;
                }
                self.tick(true);
            }
        }
        assert!(!self.core.pick_open());
        assert_eq!(self.core.op(), 40);
    }
    /// Linked healthy ticks past verify_ms.
    fn verify_window(&mut self) {
        for _ in 0..self.cfg.hop.verify_ms / 10 + 30 {
            self.vrx.on_video(self.ms());
            self.tick(true);
        }
    }
    fn busy_op(&self) {
        for c in &self.cards {
            c.s().cca_per_ms_on[40] = 50;
        }
    }
    fn ch(&self, i: usize) -> u8 {
        self.cards[i].s().ch
    }
    fn until(&mut self, n: usize, session: bool, needle: &str) {
        for _ in 0..n {
            if self.has_line(needle) {
                break;
            }
            self.tick(session);
        }
    }
}

#[test]
fn constructs_two_cards_auto_scouting() {
    let g = Rig::auto(2);
    let s = g.core.snapshot(0);
    assert_eq!(s.scan_state, "scouting");
    assert_eq!(s.scan_rounds, 0);
    assert!(s.scan_pick.is_none());
    assert_eq!(s.hop.state, "idle");
    assert!(s.energy.len() == 2 && s.dwell.len() == 2);
    assert_eq!(g.core.op(), 40);
    assert!(!g.core.hopping());
}

#[test]
fn constructs_pinned_is_off_with_pick_latched() {
    let g = Rig::new(2, true, 64);
    let s = g.core.snapshot(0);
    assert_eq!(s.scan_state, "off");
    assert_eq!(s.scan_pick, Some(64));
    assert!(!g.core.pick_open());
}

#[test]
fn hop_state_name_covers_every_state() {
    assert_eq!(hop_state_name(HopState::Idle), "idle");
    assert_eq!(hop_state_name(HopState::Ordered), "ordered");
    assert_eq!(hop_state_name(HopState::Verifying), "verifying");
    assert_eq!(hop_state_name(HopState::Hold), "hold");
}

/// Drop when the scout owns a card AND (the frame is for the scout card
/// while it is not beaconing, OR any card while the scout is in a quiet
/// observe).
#[test]
fn may_send_two_cards_drops_scout_card_unless_beaconing() {
    let g = Rig::auto(2);
    assert!(g.core.may_send(0));
    assert!(!g.core.may_send(1));
    assert_eq!(g.core.snapshot(0).scout_gated_sends, 1);
    for _ in 0..6 {
        g.core.run_scout_step();
    }
    let obs = g.gate_obs.lock().unwrap();
    assert!(obs.iter().any(|o| o.1), "scout card beaconing passes");
    assert!(
        obs.iter().any(|o| !o.0),
        "quiet observe holds the link card too"
    );
}

#[test]
fn may_send_counts_gated_sends() {
    let g = Rig::auto(1);
    assert!(!g.core.may_send(0)); // prelude: silent
    assert_eq!(g.core.snapshot(0).scout_gated_sends, 1);
    assert!(!g.core.may_send(0));
    assert_eq!(g.core.snapshot(0).scout_gated_sends, 2);
}

#[test]
fn disc_targets_follow_scan_disc_targets_while_scout_owns() {
    let g = Rig::auto(2);
    // The link card always; the scout card only while beaconing.
    assert_eq!(g.core.disc_targets(0), vec![0]);
}

#[test]
fn disc_for_card_retags_members_and_grid_channels() {
    let g = Rig::auto(2);
    let d = rc::Disc {
        vrx_nonce: 7,
        op_channel: 40,
        op_width: 40,
        seq: 1,
        ..Default::default()
    };
    let frame = rc::pack_disc(&d, &g.cfg.link.key);
    g.cards[1].s().ch = 64;
    let f1 = g.core.disc_for_card(&frame, 1);
    assert_eq!(rc::parse_disc(&f1).unwrap().op_channel, 64);
    let ok = |f: &[u8]| rc::verify_control(f, &g.cfg.link.key, &rc::TagCtx::default());
    assert_eq!(ok(&f1), ok(&frame)); // re-tagged, not just patched
    g.cards[1].s().ch = 60; // a grid channel outside the set
    let f2 = g.core.disc_for_card(&frame, 1);
    assert_eq!(rc::parse_disc(&f2).unwrap().op_channel, 60);
    g.cards[1].s().ch = 68; // off the grid
    assert_eq!(g.core.disc_for_card(&frame, 1), frame);
}

/// The grid search found the drone outside the set: the link forms there.
#[test]
fn session_opened_on_a_grid_channel_outside_the_set_moves_op_there() {
    let mut g = Rig::auto(2);
    g.session_opened_on(100, 100);
    assert_eq!(g.core.op(), 100);
    assert_eq!(g.vrx.proposal(), 100);
    assert!(g.has_line("drone found on 100 (op 40): the link forms there (outside the set)"));
}

#[test]
fn session_opened_elsewhere_moves_op_there() {
    let mut g = Rig::auto(2);
    g.session_opened_on(64, 64);
    assert_eq!(g.core.op(), 64);
    assert_eq!(g.vrx.proposal(), 64);
    assert!(g.has_line("drone found on 64 (op 40): the link forms there"));
    g.tick(true);
    assert!(g.has_move(MoveReason::LinkFound));
}

#[test]
fn session_opened_with_key_mismatch_or_foreign_nonce_is_ignored() {
    let mut g = Rig::auto(2);
    g.core.on_rc_body(64);
    let mut ack = g.ack(64);
    ack.flags = rc::ACK_KEY_MISMATCH;
    g.core.on_session_opened(&ack, 1000.0, &mut g.vrx);
    assert_eq!(g.core.op(), 40);
    ack.flags = 0;
    ack.vrx_nonce = g.vrx.rz_nonce().wrapping_add(1);
    g.core.on_session_opened(&ack, 1000.0, &mut g.vrx);
    assert_eq!(g.core.op(), 40);
    g.core.on_rc_body(68); // off the grid
    ack.vrx_nonce = g.vrx.rz_nonce();
    g.core.on_session_opened(&ack, 1000.0, &mut g.vrx);
    assert_eq!(g.core.op(), 40);
}

#[test]
fn disc_targets_is_tx_when_scout_owns_nothing() {
    let mut g = Rig::new(2, true, 40);
    g.tick_tx(true, 1); // linked: release_scout() false, search off
    g.tick_tx(true, 1);
    assert_eq!(g.core.disc_targets(1), vec![1]);
}

/// One card, auto: silent one_card_ms, then AckPrelude commits the prelude
/// ranking (no link) and the first op window runs.
#[test]
fn one_card_prelude_commits_before_first_disc() {
    let mut g = Rig::auto(1);
    g.cards[0].s().nhm_busy_pct_on[40] = 100; // 40 blocked, the rest unread
    g.until(800, false, "one-card prelude ranking picks");
    assert!(g.has_line("one-card prelude ranking picks"));
    assert_ne!(g.core.op(), 40);
    assert_eq!(g.stored.lock().unwrap().last(), Some(&g.core.op()));
    assert_eq!(g.vrx.proposal(), g.core.op());
    assert!(g.has_move(MoveReason::Commit));
    assert_eq!(g.core.snapshot(0).scan_state, "scouting");
}

/// radio.use narrowed to one member that nothing has ranked yet: the
/// prelude still commits to it, never to the unmeasured op outside it.
#[test]
fn one_card_prelude_never_picks_outside_radio_use() {
    let mut g = Rig::with_use(1, false, 0, &[112]);
    g.until(800, false, "one-card prelude ranking picks");
    assert!(
        g.has_line("one-card prelude ranking picks 112"),
        "{:?}",
        g.rec().lines
    );
    assert_eq!(g.core.op(), 112);
}

#[test]
fn two_card_no_link_commits_at_maturity() {
    let mut g = Rig::auto(2);
    g.cards[1].s().nhm_busy_pct_on[40] = 100; // op's pair blocked
    g.until(3000, false, "pick frozen on");
    assert!(g.has_line("pick frozen on"));
    assert!(g.has_line("(commit)"));
    assert!(g.has_line("maburgs channel: commit 40 -> "));
    assert_ne!(g.core.op(), 40);
    assert!(g.rec().picks.last().unwrap().is_some());
    assert_eq!(g.ch(0), g.core.op()); // the mechanical retune
}

#[test]
fn store_failure_is_logged_not_fatal() {
    let mut g = Rig::auto(2);
    g.store_ok.store(false, SeqCst);
    g.cards[1].s().nhm_busy_pct_on[40] = 100; // op's pair blocked
    g.until(3000, false, "pick frozen on");
    assert!(g.has_line("pick frozen on"));
    assert!(g.has_line("maburgs channel: could not write the-store"));
    assert_ne!(g.core.op(), 40);
}

#[test]
fn max_ms_freezes_unmeasured_in_place() {
    let mut g = Rig::auto(2);
    g.cards[1].s().retune_ok = false; // the scout never completes a dwell
    g.until(4000, false, "pick frozen on");
    assert!(g.has_line("(max_ms)"));
    assert_eq!(g.core.op(), 40);
}

#[test]
fn scout_card_death_while_working_freezes_pick() {
    let mut g = Rig::auto(2);
    g.tick(false);
    g.cards[1].s().is_alive = false;
    g.core.on_card_died(1);
    assert!(g.has_line("scout card 1 died at"));
    g.tick(false);
    assert!(g.has_line("(scout card died)"));
    assert!(!g.core.pick_open());
}

/// Pinned + linked: the scout parks and gives up the card, so the core's
/// width resync is the only thing that can fix a card reopened at 20 MHz.
#[test]
fn reopened_scout_card_at_20_gets_width_resync() {
    let mut g = Rig::new(2, true, 40);
    for _ in 0..10 {
        g.tick(true);
    }
    assert_eq!(g.cards[1].s().width_mhz, 40); // parked at radio.width by the scout
    g.cards[1].s().is_alive = false;
    g.core.on_card_died(1);
    {
        let mut s = g.cards[1].s();
        s.is_alive = true;
        s.ch = 40;
        s.width_mhz = 20;
        s.calls.clear();
    }
    g.core.on_card_reopened(1);
    for _ in 0..50 {
        g.tick(true);
    }
    assert!(g.cards[1]
        .calls()
        .iter()
        .any(|c| c.starts_with("set_width")));
    assert_eq!(g.cards[1].s().width_mhz, 40);
}

#[test]
fn mechanical_retune_skips_scout_card_and_follows_desired() {
    let mut g = Rig::auto(2);
    g.tick(false);
    g.tick(false);
    assert!(g.core.scout_owns_card(1));
    g.session_opened_on(64, 64);
    g.cards[1].s().calls.clear();
    g.tick_tx(true, 0);
    assert_eq!(g.ch(0), 64);
    assert!(!g.cards[1].calls().iter().any(|c| c == "retune 64"));
    assert!(g.core.scout_owns_card(1));
}

/// Pinned = static: an interfered op produces a verdict but no order, no
/// escape, no exhausted hold.
#[test]
fn pinned_two_card_never_hops_on_interference() {
    let mut g = Rig::new(2, true, 40);
    g.link_up();
    for _ in 0..5 {
        g.tick(true);
    }
    g.busy_op();
    let lead = g.occ_lead();
    g.interfere(lead + 6);
    assert_eq!(g.core.snapshot(0).hop.verdict, "interfered");
    assert!(g.rec().hops.is_empty());
    assert_eq!(g.core.snapshot(0).hop.state, "idle");
    assert!(g.ch(0) == 40 && g.ch(1) == 40);
    assert_eq!(g.pump_rcf().unwrap().hop_ch, 0);
    let dwells = g.rec().dwells.len();
    g.core.run_inflight_step();
    g.tick(true);
    assert_eq!(g.rec().dwells.len(), dwells);
    assert_eq!(g.ch(1), 40);
}

#[test]
fn pinned_one_card_never_hops_on_interference() {
    let mut g = Rig::new(1, true, 40);
    g.link_up();
    for _ in 0..5 {
        g.tick(true);
    }
    g.busy_op();
    let lead = g.occ_lead();
    g.interfere(lead + 6);
    assert_eq!(g.core.snapshot(0).hop.verdict, "interfered");
    assert!(g.rec().hops.is_empty());
    assert_eq!(g.ch(0), 40);
    assert_eq!(g.pump_rcf().unwrap().hop_ch, 0);
}

/// mabur boot_relocate_waits_for_video_on_a_disc_only_session: a freshly
/// booted drone acks DISC before its encoder runs; the relocate waits for a
/// video run, and a trickle (one body every 660 ms) is not one.
#[test]
fn boot_relocate_waits_for_video_on_a_disc_only_session() {
    let mut g = Rig::new(2, true, 40);
    g.video = false;
    g.session_opened_on(64, 64);
    assert_eq!(g.core.op(), 64);
    for _ in 0..90 {
        g.tick(true);
    }
    assert!(!g.has_line("relocate 64 -> 40 placed"));
    for i in 0..200 {
        if i % 66 == 0 {
            let now = g.ms();
            g.vrx.on_video(now);
        }
        g.tick(true);
    }
    assert!(!g.has_line("relocate 64 -> 40 placed"));
    assert!(!g.has_line("did not land"));
    g.video = true;
    for _ in 0..100 {
        if g.has_line("relocate 64 -> 40 placed") {
            break;
        }
        g.tick(true);
    }
    assert!(g.has_line("relocate 64 -> 40 placed"));
}

/// The relocation onto the pin landed, then the verify window read the pin
/// interfered: pinned, the confirm lands it with no verify.
#[test]
fn pinned_relocation_is_not_failed_by_a_dirty_pin() {
    let mut g = Rig::new(2, true, 40);
    g.session_opened_on(64, 64);
    g.link_up();
    g.until(200, true, "relocate 64 -> 40 placed");
    assert!(g.has_line("relocate 64 -> 40 placed"));
    assert_eq!(g.pump_rcf().unwrap().hop_ch, 40);
    g.core.note_video(40);
    g.tick(true);
    assert!(g.has_hop("lead_confirm"));
    g.busy_op();
    let lead = g.occ_lead();
    g.interfere(lead + 8);
    assert!(g.has_hop("verify_pass"));
    assert!(!g.has_hop("verify_fail"));
    assert!(!g.has_line("did not land"));
    let s = g.core.snapshot(0);
    assert_eq!(s.hop.holds, 0);
    assert_eq!(s.hop.state, "idle");
    assert!(g.core.op() == 40 && g.ch(0) == 40 && g.ch(1) == 40);
    assert_eq!(s.hop.verdict, "interfered");
}

/// radio.use narrows where the link may sit, not where the drone is
/// searched: a link found outside it (here op 40, frozen unmeasured) is
/// moved to a usable member, and the reactive candidates are only those.
#[test]
fn a_link_outside_radio_use_is_moved_into_it() {
    let mut g = Rig::with_use(2, false, 0, &[64, 112]);
    g.cards[1].s().retune_ok = false; // the scout never completes a dwell
    g.until(4000, false, "pick frozen on");
    assert_eq!(g.core.op(), 40);
    g.link_up();
    g.until(200, true, "relocate 40 -> 64 placed");
    assert!(g.has_line("link on 40 is outside radio.use: relocating to 64"));
    assert!(g.has_line("relocate 40 -> 64 placed"));
    assert_eq!(g.pump_rcf().unwrap().hop_ch, 64);
    g.core.note_video(64);
    g.tick(true);
    g.verify_window();
    assert!(g.has_hop("verify_pass"));
    assert_eq!(g.core.op(), 64);
    g.tick(true);
    assert_eq!(
        g.count_lines("outside radio.use"),
        1,
        "steered once, not again once inside"
    );
}

/// A GS restart finds the drone outside radio.use while the boot pick is
/// still open: the link leaves for a usable member at once, not after the
/// scout matures (Quest 2026-11-03: 15-20 s on an unchosen channel).
#[test]
fn a_link_found_outside_radio_use_leaves_before_the_pick_matures() {
    let mut g = Rig::with_use(2, false, 0, &[64, 112]);
    g.tick(false);
    assert!(g.core.pick_open());
    g.link_up();
    g.until(300, true, "relocate 40 -> ");
    assert!(g.has_line("boot pick wants"), "{:?}", g.rec().lines);
    assert!(!g.has_line("pick frozen on 40"));
    let to = g.pump_rcf().unwrap().hop_ch;
    assert!(to == 64 || to == 112, "relocated to {to}");
    g.core.note_video(to);
    g.tick(true);
    g.verify_window();
    assert!(g.has_hop("verify_pass"));
    assert_eq!(g.core.op(), to);
}

/// The operator's channel choice on a live link: one member moves the
/// link there by the relocate hop (no relink), all of them leaves it be,
/// a pinned config refuses.
#[test]
fn set_usable_moves_a_live_link_by_hop() {
    let mut g = Rig::auto(2);
    g.cards[1].s().retune_ok = false;
    g.until(4000, false, "pick frozen on");
    g.link_up();
    g.tick(true);
    assert_eq!(g.core.op(), 40);
    assert!(g.core.set_usable(&[]).is_err());
    assert!(g.core.set_usable(&[36]).is_err());
    g.core.set_usable(&[112]).unwrap();
    g.until(200, true, "relocate 40 -> 112 placed");
    assert!(g.has_line("relocate 40 -> 112 placed"));
    assert_eq!(g.pump_rcf().unwrap().hop_ch, 112);
    g.core.note_video(112);
    g.tick(true);
    g.verify_window();
    assert!(g.has_hop("verify_pass"));
    assert_eq!(g.core.op(), 112);
    g.core.set_usable(&[40, 64, 112, 144]).unwrap();
    for _ in 0..300 {
        g.tick(true);
    }
    assert_eq!(g.core.op(), 112, "all members: the link stays where it is");
    assert_eq!(g.count_lines("channel: relocate "), 1);

    let mut p = Rig::new(2, true, 40);
    assert!(p.core.set_usable(&[64]).is_err());
}

/// The operator scan on a live one-card link: every channel read over
/// every pass, the card back on op after each step, the link not moved.
#[test]
fn op_scan_reads_every_channel_and_leaves_the_link_alone() {
    let mut g = Rig::auto(1);
    g.freeze_auto();
    assert!(g.core.op_scan_json().is_none());
    g.core.begin_op_scan(&[]);
    for _ in 0..2000 {
        g.vrx.on_video(g.ms());
        g.tick(true);
        assert_eq!(g.ch(0), 40, "the card is back on op between steps");
        if g.core
            .op_scan_json()
            .unwrap()
            .contains("\"state\":\"done\"")
        {
            break;
        }
    }
    let j = g.core.op_scan_json().unwrap();
    assert!(j.contains("\"state\":\"done\""), "{j}");
    for ch in [40, 64, 112, 144] {
        let n = format!("\"ch\":{ch},\"foreign\"");
        assert!(j.contains(&n), "{ch} missing: {j}");
    }
    assert!(!j.contains("\"n\":0"), "a channel was never read: {j}");
    assert_eq!(g.core.op(), 40);
    assert!(g.has_line("operator scan of [40, 64, 112, 144]"));
}

#[test]
fn without_radio_use_a_link_on_any_member_stays() {
    let mut g = Rig::auto(2);
    g.cards[1].s().retune_ok = false;
    g.until(4000, false, "pick frozen on");
    g.link_up();
    for _ in 0..300 {
        g.tick(true);
    }
    assert!(!g.has_line("outside radio.use"));
    assert_eq!(g.core.op(), 40);
}

/// Auto, frozen, linked, interfered: an order to a ranked candidate.
fn order_rig(n: usize) -> Rig {
    let mut g = Rig::auto(n);
    g.freeze_auto();
    if n >= 2 {
        g.link_up();
    }
    for _ in 0..5 {
        g.tick(true);
    }
    g.busy_op();
    g.interfere_until_order();
    g
}

#[test]
fn hop_restore_without_a_reference_keeps_the_rung() {
    let mut g = order_rig(2);
    g.vrx.restore_rung(2, 0.0);
    let mcs = g.vrx.cur_op().mcs;
    g.vrx.restore_rung(0, 0.0);
    assert_ne!(g.vrx.cur_op().mcs, mcs);
    g.vrx.restore_rung(2, 0.0);
    g.vrx.restore_rung(-1, 0.0);
    assert_eq!(g.vrx.cur_op().mcs, mcs);
    assert_eq!(g.vrx.ctl().last_event().reason.as_str(), "hop_restore");
}

#[test]
fn two_card_order_rcf_carries_hop_and_plan_leads_then_follows() {
    let mut g = order_rig(2);
    let r = g.pump_rcf().unwrap();
    assert!(r.hop_ch != 0 && r.hop_ch != 40);
    assert_eq!(r.hop_epoch, 1);
    assert!(g.has_move(MoveReason::HopLead));
    assert_eq!(g.vrx.ctl().last_event().reason.as_str(), "hop_restore");
    g.core.note_video(r.hop_ch);
    g.tick(true);
    assert!(g.has_hop("lead_confirm"));
    assert!(g.has_move(MoveReason::HopFollow));
    assert_eq!(g.core.op(), r.hop_ch);
    g.verify_window();
    assert!(g.has_hop("verify_pass"));
    assert_eq!(g.core.snapshot(0).hop.hops, 1);
}

/// After Confirm the cached verdict (measured on the old channel) is
/// re-fed until the next window: the controller stays in Verifying.
#[test]
fn stale_pre_hop_verdict_does_not_break_verify() {
    let mut g = order_rig(2);
    let r = g.pump_rcf().unwrap();
    g.core.note_video(r.hop_ch);
    g.tick(true);
    assert!(g.has_hop("lead_confirm"));
    g.tick(true);
    assert_eq!(g.core.snapshot(0).hop.state, "verifying");
    assert!(!g.has_hop("withdraw"));
}

#[test]
fn one_card_order_rides_repeats_then_retunes() {
    let mut g = order_rig(1);
    assert_eq!(g.ch(0), 40); // the radio stays until one_card_repeats RCFs
                             // The order's own control pass sends one RCF (interfere() ticks only).
    g.pump_rcf();
    let mut sent = 0;
    let mut r = None;
    for _ in 0..10 {
        if g.has_hop("one_card_retune") {
            break;
        }
        r = g.pump_rcf();
        sent += 1;
    }
    assert!(g.has_hop("one_card_retune"));
    assert_eq!(sent, g.cfg.hop.one_card_repeats);
    let r = r.unwrap();
    assert_eq!(g.ch(0), r.hop_ch);
    assert!(g.has_move(MoveReason::HopOneCard));
    for e in g.rec().hops.iter().filter(|e| e.kind == "one_card_retune") {
        assert!((200.0..=600.0).contains(&e.elapsed_ms), "{}", e.elapsed_ms);
    }
    g.core.note_video(r.hop_ch);
    g.tick(true);
    assert!(g.has_hop("lead_confirm"));
    assert!(g.has_move(MoveReason::HopFollow));
    g.verify_window();
    assert!(g.has_hop("verify_pass"));
}

#[test]
fn withdraw_on_no_video_restores_cards() {
    let mut g = order_rig(2);
    g.pump_rcf().unwrap();
    for _ in 0..g.cfg.hop.confirm_extend_ms / 10 + 20 {
        g.vrx.on_video(g.ms());
        g.tick(true);
    }
    assert!(g.has_hop("withdraw"));
    assert!(g.has_move(MoveReason::HopWithdraw));
    assert_eq!(g.ch(1), 40);
    assert_eq!(g.core.op(), 40);
}

#[test]
fn lead_card_dies_mid_order_withdraws_to_op() {
    let mut g = order_rig(2);
    g.cards[1].s().is_ready = false;
    g.tick(true);
    assert_eq!(g.core.snapshot(0).hop.state, "ordered"); // latched lead
    for _ in 0..g.cfg.hop.confirm_extend_ms / 10 + 20 {
        g.vrx.on_video(g.ms());
        g.tick(true);
    }
    assert!(g.has_hop("withdraw"));
    assert_eq!(g.core.op(), 40);
    g.cards[1].s().is_ready = true;
    g.tick(true);
    assert_eq!(g.ch(1), 40);
}

#[test]
fn session_loss_mid_order_withdraws() {
    let mut g = order_rig(2);
    g.tick(false); // hop_active falling edge
    assert!(g.has_hop("session_lost"));
    assert!(!g.core.hopping());
}

/// A move edge that fires while a hop is in flight is held and acted on
/// exactly once, after the hop resolves (mabur pairs this with the
/// calibration hold; the hop hold is the part ported).
#[test]
fn move_edge_held_through_relocation() {
    let mut g = Rig::new(2, true, 40);
    g.session_opened_on(64, 64);
    g.link_up();
    assert_eq!(g.core.op(), 64);
    g.until(200, true, "relocate 64 -> 40 placed");
    assert!(g.has_hop("relocate"));
    // A repeat DISC_ACK refreshes agreed to a non-member 60, then the edge.
    let mut rep = g.ack(60);
    rep.chip_caps = rc::CAP_FRAME_WIRE;
    rep.seq = 2;
    g.vrx.on_rc_frame(&rc::pack_disc_ack(&rep), g.ms());
    assert_eq!(g.vrx.agreed_channel(), 60);
    assert_eq!(g.vrx.link_state(), VrxState::Session);
    // Video still on 64 (last_video_ch starts on the start channel, which
    // is the target here): the confirm waits.
    g.core.note_video(64);
    g.vrx.test_set_move_edge();
    g.tick(true);
    assert!(g.core.hopping());
    let line = "drone acked 60, not in our set; ignored";
    assert!(!g.has_line(line)); // held while hopping
    assert_eq!(g.pump_rcf().unwrap().hop_ch, 40);
    g.core.note_video(40);
    g.tick(true);
    assert!(g.has_hop("lead_confirm"));
    g.verify_window();
    assert!(g.has_hop("verify_pass"));
    assert_eq!(g.count_lines(line), 1);
    assert_eq!(g.core.op(), 40);
}

#[test]
fn relocate_lands_on_verify_pass() {
    let mut g = Rig::new(2, true, 40);
    g.session_opened_on(64, 64);
    g.link_up();
    g.until(200, true, "relocate 64 -> 40 placed");
    assert!(g.has_line("relocate 64 -> 40 placed"));
    assert_eq!(g.pump_rcf().unwrap().hop_ch, 40);
    g.core.note_video(40);
    g.tick(true);
    assert!(g.has_hop("lead_confirm"));
    g.verify_window();
    assert!(g.has_hop("verify_pass"));
    assert_eq!(g.core.op(), 40);
    assert_ne!(g.core.snapshot(0).scan_state, "moving");
}

/// The auto boot hop: op's pair measured to min_rounds before the drone
/// appears on 64, the link forms before maturity; at maturity the pick
/// wants another pair: WantPick, relocate, confirm, verify_pass,
/// "pick frozen on <pick> (relocated)".
#[test]
fn auto_want_pick_relocates_linked_drone_to_the_pick() {
    let mut g = Rig::auto(2);
    g.cards[1].s().nhm_busy_pct_on[64] = 100; // 64's pair is blocked
    let visits = |g: &Rig, ch: u8| g.rec().dwells.iter().filter(|d| d.1.ch == ch).count();
    let mr = g.cfg.radio.scan.min_rounds as usize;
    for _ in 0..3000 {
        if visits(&g, 64) >= mr {
            break;
        }
        g.tick(false);
    }
    assert!(visits(&g, 64) >= mr);
    assert!(visits(&g, 144) < mr);
    assert!(g.core.pick_open());
    g.session_opened_on(64, 64);
    g.link_up();
    assert_eq!(g.core.op(), 64);
    g.until(6000, true, "boot pick wants");
    assert!(g.has_line(", link on 64: relocating"));
    g.until(200, true, "relocate 64 -> ");
    assert!(g.has_line(" placed"));
    assert_eq!(g.core.snapshot(0).scan_state, "moving");
    let target = g.core.hop_target();
    assert!(target != 0 && target != 64);
    assert_eq!(g.pump_rcf().unwrap().hop_ch, target);
    g.core.note_video(target);
    g.tick(true);
    assert!(g.has_hop("lead_confirm"));
    g.verify_window();
    assert!(g.has_hop("verify_pass"));
    assert_eq!(g.core.op(), target);
    assert!(g.has_line("(relocated)"));
    let s = g.core.snapshot(0);
    assert_eq!(s.scan_state, "frozen");
    assert_eq!(s.scan_pick, Some(target));
    assert_eq!(g.stored.lock().unwrap().last(), Some(&target));
}

/// The move edge sets link_edge_seen, which BootPick reads next tick as
/// "one-card linked".
#[test]
fn one_card_link_edge_freezes_one_card_linked() {
    let mut g = Rig::auto(1);
    g.cards[0].s().nhm_busy_pct_on[40] = 100; // 40 blocked, the rest unread
    g.until(800, false, "one-card prelude ranking picks");
    assert!(g.has_line("one-card prelude ranking picks"));
    g.vrx.test_set_move_edge();
    g.tick(true);
    g.tick(true);
    assert!(g.has_line("pick frozen on"));
    assert!(g.has_line("(one-card linked)"));
    let s = g.core.snapshot(0);
    assert_eq!(s.scan_state, "frozen");
    assert_eq!(s.scan_pick, Some(g.core.op()));
}

#[test]
fn inflight_dwell_feeds_ranker_and_dwell_stats() {
    let mut g = Rig::auto(2);
    g.freeze_auto();
    g.link_up();
    for _ in 0..10 {
        g.tick(true);
    }
    g.core.run_inflight_step();
    g.tick(true); // drain
    assert!(!g.core.dwell_busy());
    assert_eq!(g.core.snapshot(0).dwell[1].unwrap().visits, 1);
    assert_eq!(g.rec().dwells.last().unwrap().0, 1);
    assert_eq!(g.ch(1), 40); // back on op
}

#[test]
fn inflight_step_skips_when_not_in_session_or_one_card_or_pinned() {
    let mut g = Rig::new(1, true, 40);
    g.link_up();
    for _ in 0..10 {
        g.tick(true);
    }
    g.core.run_inflight_step();
    g.tick(true);
    assert!(g.core.snapshot(0).dwell[0].is_none()); // one card
    let mut h = Rig::new(2, true, 40);
    for _ in 0..10 {
        h.tick(false);
    }
    h.core.run_inflight_step();
    h.tick(false);
    assert!(h.core.snapshot(0).dwell[1].is_none()); // no session
    let mut p = Rig::new(2, true, 40);
    p.link_up();
    for _ in 0..10 {
        p.tick(true);
    }
    let sweep = p.rec().dwells.len();
    p.core.run_inflight_step();
    p.tick(true);
    assert_eq!(p.rec().dwells.len(), sweep); // pinned
    assert!(p.core.snapshot(0).dwell[1].is_none());
}

#[test]
fn tx_frozen_while_hopping() {
    let mut g = order_rig(2);
    let out = g.tick_tx(true, 0);
    assert!(out.tx_frozen);
    assert!(g.core.tx_frozen());
}

#[test]
fn snapshot_channel_is_the_given_tx_card() {
    let g = Rig::new(2, true, 40);
    g.cards[1].s().ch = 112;
    assert_eq!(g.core.snapshot(0).channel, 40);
    assert_eq!(g.core.snapshot(1).channel, 112);
}

/// A record the core takes a tick late still reads own_air over the
/// sampled span.
#[test]
fn sampled_record_consumed_next_tick_uses_sample_span() {
    let mut g = Rig::new(1, true, 40);
    g.link_up();
    for _ in 0..2 * (g.cfg.hop.window_ms / 10) + 2 {
        g.tick(true);
    }
    g.core.run_sampler_step(); // a record at T1
    g.clk.advance(10);
    g.tick_tx(true, 0); // consumed at T1 + 10
    for _ in 0..13 {
        g.tick(true); // T1 + 140
    }
    {
        let mut s = g.cards[0].s();
        s.fr.own_air_us += 70000; // 50 % of 140 ms
        s.fr.own += 10;
    }
    g.core.run_sampler_step();
    g.clk.advance(30);
    g.cards[0].s().fr.own_air_us += 5000; // after the sample: must not count
    g.tick_tx(true, 0);
    let e = g.core.snapshot(0).energy[0].unwrap();
    let pct = e.own_air_pct.unwrap();
    assert!(pct > 49.0 && pct < 51.0, "{pct}");
}

/// Real threads, real clock: auto + linked with max_ms at its floor; the
/// boot scout measures ~1 s, the pick freezes in place, the in-flight
/// thread dwells on the non-TX card; shutdown() joins all of them.
#[test]
fn shutdown_joins_threads_and_is_idempotent() {
    let mut cfg = bundle();
    cfg.radio.channels = vec![40, 64];
    cfg.radio.scan.max_ms = 1000;
    let clk = FakeClock::default();
    let a = FakeCard::new(clk.clone(), 40);
    let b = FakeCard::new(clk.clone(), 40);
    a.s().width_mhz = 40;
    let t0 = Instant::now();
    let now_ms: MsClock = Arc::new(move || t0.elapsed().as_millis() as u64);
    let now_us: UsClock = Arc::new(move || t0.elapsed().as_micros() as u64);
    let mut vrx = VrxController::new(VrxCfg::from_config(&cfg, 40, 0));
    let agg = Aggregator::new(&cfg.uep_layers(), 32, 2, 0);
    let sink = Arc::new(Mutex::new(Rec::default()));
    let mut core = ChannelCore::new(
        ChannelCoreCfg {
            radio: cfg.radio.clone(),
            width_mhz: 40,
            hop: cfg.hop.clone(),
            key: cfg.link.key,
            start_ch: 40,
            ..Default::default()
        },
        vec![a.clone() as Arc<dyn LinkCard>, b.clone()],
        Box::new(sink),
        Box::new(|_| true),
        now_ms.clone(),
        now_us,
        Arc::new(|ms| std::thread::sleep(Duration::from_millis(ms as u64))),
    );
    let mut dwelt = false;
    for _ in 0..500 {
        clk.set(now_ms());
        let inp = ChannelTickIn {
            now_ms: now_ms() as f64,
            in_session: true,
            cal_running: false,
            tx_card: 0,
            agg: Some(&agg),
        };
        core.tick(&inp, &mut vrx);
        dwelt = core.snapshot(0).dwell[1].is_some();
        if dwelt {
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(dwelt);
    let visits = core.snapshot(0).dwell[1].unwrap().visits;
    let t = Instant::now();
    core.shutdown();
    core.shutdown();
    assert!(t.elapsed() < Duration::from_secs(2));
    std::thread::sleep(Duration::from_millis(400));
    let inp = ChannelTickIn {
        now_ms: now_ms() as f64,
        in_session: true,
        cal_running: false,
        tx_card: 0,
        agg: Some(&agg),
    };
    assert!(!core.tick(&inp, &mut vrx).dwell_busy);
    assert_eq!(core.snapshot(0).dwell[1].unwrap().visits, visits);
}

// ---- calibration channel pick (mabur test_channel_core.cpp, the cal_* cases) ----

impl Rig {
    fn tick_cal(&mut self, session: bool, cal: bool) {
        let inp = ChannelTickIn {
            now_ms: self.ms(),
            in_session: session,
            cal_running: cal,
            tx_card: 0,
            agg: Some(&self.agg),
        };
        self.core.tick(&inp, &mut self.vrx);
        self.clk.advance(10);
    }
    /// One linked tick with the TX card (card 0) on op hearing its own
    /// video and `op_foreign` foreign frames: own frames keep the verdict
    /// out of starved, foreign frames are what op's live reading scores.
    fn live_tick(&mut self, op_foreign: u64) {
        {
            let mut s = self.cards[0].s();
            s.fr.own += 5;
            s.fr.foreign += op_foreign;
        }
        self.vrx.on_video(self.ms());
        self.tick(true);
    }
    fn linked_auto(&mut self, ticks: usize, op_foreign: u64) {
        self.freeze_auto();
        self.link_up();
        for _ in 0..ticks {
            self.live_tick(op_foreign);
        }
    }
    /// Starts a round; it is running once the scout card is owned.
    fn start_round(&mut self) {
        let now = self.ms();
        self.core.begin_cal_measure(now);
        for _ in 0..5 {
            if self.has_line("calibration measure round on card") {
                break;
            }
            self.live_tick(0);
        }
        assert!(self.has_line("calibration measure round on card 1"));
        self.live_tick(0);
        assert!(self.core.scout_owns_card(1));
    }
    fn cal_move(&mut self, ch: u8) {
        assert!(self.core.order_cal_relocate(ch));
        for _ in 0..20 {
            if self.core.hopping() {
                break;
            }
            self.live_tick(0);
        }
        assert!(self.core.hopping());
        let r = self.pump_rcf().expect("rcf");
        assert_eq!(r.hop_ch, ch);
        self.core.note_video(ch);
        for _ in 0..5 {
            if self.core.cal_relocate_settled().is_some() {
                break;
            }
            self.tick(true);
        }
        assert_eq!(self.core.cal_relocate_settled(), Some(true));
        for _ in 0..3 {
            self.live_tick(0);
        }
        assert_eq!(self.core.op(), ch);
    }
}

/// Auto, two cards, linked, the boot pick long closed: one round dwells
/// on every pair outside op's exactly once, reads op from the verdict
/// window, emits one K record and names the best member: op, 64 and 144
/// read 25 % busy, 112 reads 5 %.
#[test]
fn cal_measure_round_ranks_the_set_once() {
    let mut g = Rig::auto(2);
    g.linked_auto(30, 200);
    g.cal_air();
    assert!(g.core.cal_can_measure());
    assert_eq!(g.core.op(), 40);
    let (picks0, dwells0) = {
        let r = g.rec();
        (r.picks.len(), r.dwells.len())
    };
    let now = g.ms();
    g.core.begin_cal_measure(now);
    assert_eq!(g.core.cal_measure_best(), None);
    for _ in 0..50 {
        if g.core.cal_measure_best().is_some() {
            break;
        }
        g.live_tick(200);
    }
    assert_eq!(g.core.cal_measure_best(), Some(112));
    assert_eq!(g.rec().picks.len(), picks0 + 1);
    assert_eq!(*g.rec().picks.last().unwrap(), Some(112));
    let mut chs: Vec<u8> = g.rec().dwells[dwells0..]
        .iter()
        .map(|(card, d)| {
            assert_eq!(*card, 1);
            d.ch
        })
        .collect();
    chs.sort();
    assert_eq!(chs, vec![64, 112, 144]); // one dwell per pair; op's (40) never dwelt on
    assert_eq!(g.core.op(), 40); // measuring moves nothing
    for _ in 0..3 {
        g.live_tick(200);
    }
    assert!(!g.core.scout_owns_card(1)); // parked again ...
    assert!(g.ch(1) == 40 && g.cards[1].s().width_mhz == 40); // ... on op at the link width
    let dwells1 = g.rec().dwells.len();
    for _ in 0..20 {
        g.live_tick(200);
    }
    assert_eq!(g.rec().dwells.len(), dwells1); // one round, not a loop
    assert_eq!(g.rec().picks.len(), picks0 + 1);
    assert_eq!(g.core.cal_measure_best(), Some(112));
}

/// Over a quiet op a candidate must beat op by pick_margin: op stays.
#[test]
fn cal_measure_round_keeps_a_clean_op() {
    let mut g = Rig::auto(2);
    g.linked_auto(30, 0);
    g.cards[1].s().foreign_per_ms_on[60] = 1;
    g.cards[1].s().foreign_per_ms_on[64] = 1;
    let now = g.ms();
    g.core.begin_cal_measure(now);
    for _ in 0..50 {
        if g.core.cal_measure_best().is_some() {
            break;
        }
        g.live_tick(0);
    }
    assert_eq!(g.core.cal_measure_best(), Some(40));
}

/// The boot pick's relocate tagged cal: H relocate with reason "cal", the
/// confirm lands it (no verify window), the store follows op.
#[test]
fn cal_relocate_orders_a_move_and_settles_on_confirm() {
    let mut g = Rig::auto(2);
    g.linked_auto(5, 0);
    assert_eq!(g.core.cal_relocate_settled(), None);
    assert!(!g.core.order_cal_relocate(40)); // already there
    assert!(!g.core.order_cal_relocate(36)); // not a member
    assert!(g.core.order_cal_relocate(112));
    assert!(!g.core.order_cal_relocate(144)); // one relocate at a time
    for _ in 0..20 {
        if g.has_line("relocate 40 -> 112 placed") {
            break;
        }
        g.live_tick(0);
    }
    assert!(g.has_line("maburgs channel: relocate 40 -> 112 placed (cal)"));
    let h = g
        .rec()
        .hops
        .iter()
        .find(|e| e.kind == "relocate")
        .cloned()
        .unwrap();
    assert_eq!(h.target, 112);
    assert_eq!(h.reason, "cal");
    assert_eq!(g.core.cal_relocate_settled(), None);
    assert!(!g.core.order_cal_relocate(144)); // a hop in flight
    let r = g.pump_rcf().expect("rcf");
    assert_eq!(r.hop_ch, 112);
    g.core.note_video(112);
    g.tick(true);
    assert!(g.has_hop("lead_confirm"));
    for _ in 0..3 {
        if g.core.cal_relocate_settled().is_some() {
            break;
        }
        g.tick(true);
    }
    assert_eq!(g.core.cal_relocate_settled(), Some(true));
    assert!(g.has_hop("verify_pass"));
    assert_eq!(g.core.snapshot(0).hop.state, "idle");
    assert_eq!(g.core.op(), 112);
    assert_eq!(g.stored.lock().unwrap().last(), Some(&112));
    for e in &g.rec().hops {
        if e.kind != "relocate" {
            assert!(e.reason.is_empty(), "{}: {}", e.kind, e.reason);
        }
    }
}

/// The drone never shows up on the target: withdraw at confirm_ms, settled
/// false, the link stays on op, no retry behind the run's back.
#[test]
fn cal_relocate_settles_false_when_the_drone_does_not_follow() {
    let mut g = Rig::auto(2);
    g.linked_auto(5, 0);
    assert!(g.core.order_cal_relocate(112));
    for _ in 0..20 {
        if g.has_line("relocate 40 -> 112 placed") {
            break;
        }
        g.live_tick(0);
    }
    assert!(g.has_line("relocate 40 -> 112 placed (cal)"));
    for _ in 0..g.cfg.hop.confirm_ms / 10 + 20 {
        if g.core.cal_relocate_settled().is_some() {
            break;
        }
        g.live_tick(0);
    }
    assert_eq!(g.core.cal_relocate_settled(), Some(false));
    assert!(g.has_hop("withdraw"));
    assert_eq!(g.core.op(), 40);
    for _ in 0..20 {
        g.live_tick(0);
    }
    assert!(g.has_line("relocation to 112 did not land; staying on 40"));
    assert!(g.ch(0) == 40 && g.ch(1) == 40);
    assert_eq!(g.count_lines("relocate 40 -> 112 placed"), 1);
}

/// Pinned and one-card GSs cannot measure: op is the answer at once.
#[test]
fn pinned_core_has_no_measure() {
    let mut g = Rig::new(2, true, 64);
    g.link_up();
    for _ in 0..10 {
        g.live_tick(0);
    }
    assert!(!g.core.cal_can_measure() && g.core.pinned());
    let (dwells0, picks0) = {
        let r = g.rec();
        (r.dwells.len(), r.picks.len())
    };
    let now = g.ms();
    g.core.begin_cal_measure(now);
    assert_eq!(g.core.cal_measure_best(), Some(64));
    for _ in 0..20 {
        g.live_tick(0);
    }
    assert_eq!(g.rec().dwells.len(), dwells0);
    assert_eq!(g.rec().picks.len(), picks0);
    assert!(!g.core.scout_owns_card(1));

    let mut h = Rig::auto(1);
    assert!(!h.core.cal_can_measure() && !h.core.pinned());
    let now = h.ms();
    h.core.begin_cal_measure(now);
    assert_eq!(h.core.cal_measure_best(), Some(h.core.op()));
}

#[test]
fn cal_cancel_releases_the_card_and_withdraws_an_unplaced_relocate() {
    let mut g = Rig::auto(2);
    g.linked_auto(10, 0);
    g.start_round();
    g.core.cancel_cal_measure();
    for _ in 0..3 {
        g.live_tick(0);
    }
    assert!(!g.core.scout_owns_card(1));
    assert!(g.ch(1) == 40 && g.cards[1].s().width_mhz == 40);
    assert_eq!(g.core.cal_measure_best(), None);
    let (dwells, picks) = {
        let r = g.rec();
        (r.dwells.len(), r.picks.len())
    };
    for _ in 0..20 {
        g.live_tick(0);
    }
    assert_eq!(g.rec().dwells.len(), dwells);
    assert_eq!(g.rec().picks.len(), picks);
    // A relocate accepted but not yet placed is withdrawn with it.
    assert!(g.core.order_cal_relocate(112));
    g.core.cancel_cal_measure();
    for _ in 0..20 {
        g.live_tick(0);
    }
    assert!(!g.has_line("relocate 40 -> 112 placed"));
    assert!(!g.has_hop("relocate"));
    assert_eq!(g.core.op(), 40);
    assert_eq!(g.core.cal_relocate_settled(), None);
}

#[test]
fn cal_waiting_round_is_dropped_once_a_run_calibrates() {
    let mut g = Rig::auto(2);
    g.linked_auto(10, 0);
    let now = g.ms();
    g.core.begin_cal_measure(now);
    g.tick_cal(true, true);
    assert!(g.has_line("calibration measure round dropped"));
    let dwells = g.rec().dwells.len();
    for _ in 0..30 {
        g.live_tick(0);
    }
    assert!(!g.has_line("calibration measure round on card"));
    assert!(!g.core.scout_owns_card(1));
    assert_eq!(g.rec().dwells.len(), dwells);
    assert_eq!(g.core.cal_measure_best(), None);
}

#[test]
fn cal_round_never_starts_without_a_running_scout() {
    let mut g = Rig::auto(2);
    g.cards[1].s().is_ready = false; // the scout card never comes up
    g.linked_auto(10, 0);
    let now = g.ms();
    g.core.begin_cal_measure(now);
    for _ in 0..50 {
        g.live_tick(0);
    }
    assert!(!g.has_line("calibration measure round on card"));
    assert!(!g.core.scout_owns_card(1));
    assert_eq!(g.core.cal_measure_best(), None);
    g.core.cancel_cal_measure();
}

#[test]
fn cal_running_round_ends_at_its_bound_with_op() {
    let mut g = Rig::auto(2);
    g.linked_auto(10, 200);
    g.start_round();
    assert_eq!(g.core.cal_measure_best(), None);
    g.clk.advance(10_000); // the scout made no progress for 10 s
    g.live_tick(200);
    assert_eq!(g.core.cal_measure_best(), Some(40));
    assert!(g.has_line("calibration measure round overran"));
    for _ in 0..3 {
        g.live_tick(200);
    }
    assert!(!g.core.scout_owns_card(1));
    assert!(g.ch(1) == 40 && g.cards[1].s().width_mhz == 40);
}

#[test]
fn cal_relocate_refused_at_the_hop_cap() {
    let mut g = Rig::auto(2);
    g.linked_auto(5, 0);
    assert_eq!(g.cfg.hop.max_hops_per_min, 4);
    g.cal_move(112);
    g.cal_move(144);
    g.cal_move(64);
    g.cal_move(40);
    assert!(!g.core.order_cal_relocate(112));
    assert!(g.has_line("calibration relocate to 112 refused: hop cap"));
    assert_eq!(g.core.op(), 40);
}

#[test]
fn cal_relocate_never_placed_settles_false() {
    let mut g = Rig::auto(2);
    g.linked_auto(5, 0);
    assert!(g.core.order_cal_relocate(112));
    g.tick_cal(true, true); // no longer hop-active before the order leaves
    assert_eq!(g.core.cal_relocate_settled(), Some(false));
    assert!(g.has_line("calibration relocate to 112 never placed"));
    for _ in 0..20 {
        g.live_tick(0);
    }
    assert!(!g.has_hop("relocate"));
    assert_eq!(g.core.op(), 40);
}

#[test]
fn cal_relocate_session_lost_in_flight_settles_false() {
    let mut g = Rig::auto(2);
    g.linked_auto(5, 0);
    assert!(g.core.order_cal_relocate(112));
    for _ in 0..20 {
        if g.has_line("relocate 40 -> 112 placed") {
            break;
        }
        g.live_tick(0);
    }
    assert!(g.has_line("relocate 40 -> 112 placed (cal)"));
    g.tick(false); // session lost before the confirm
    assert_eq!(g.core.cal_relocate_settled(), Some(false));
    assert!(g.has_hop("session_lost"));
    assert_eq!(g.core.op(), 40);
}

/// A calibration run on an auto two-card core picks through the real
/// ChannelCore: the round names 112 over a busy op, the relocate lands on
/// the drone's confirm, and the first SweepCmd is due on 112.
#[test]
fn a_calibration_run_picks_its_channel_through_the_core() {
    use crate::cal::log::CalLog;
    use crate::cal::session::{CalSession, CalSessionCfg, State};
    use pal_wire::cal::{CalMsg, Kind, SweepSpec};

    let mut g = Rig::auto(2);
    g.linked_auto(30, 200);
    g.cal_air();
    let mut s = CalSession::new(CalSessionCfg::default(), CalLog::none());
    s.set_peer(true, true, rc::TagCtx::default());
    let sweeps = vec![SweepSpec {
        kind: Kind::Power,
        bw: 20,
    }];
    let now = g.clk.ms();
    s.start(1, sweeps, now, Some(&mut g.core)).unwrap();
    assert_eq!(s.state(), State::Picking);
    for _ in 0..100 {
        if s.snapshot().pick_note.starts_with("relocating") {
            break;
        }
        g.live_tick(200);
        s.step_pick(g.clk.ms(), &mut g.core);
    }
    assert_eq!(s.snapshot().pick_note, "relocating 40 -> 112");
    for _ in 0..20 {
        if g.core.hopping() {
            break;
        }
        g.live_tick(0);
        s.step_pick(g.clk.ms(), &mut g.core);
    }
    let r = g.pump_rcf().expect("rcf");
    assert_eq!(r.hop_ch, 112);
    g.core.note_video(112);
    let mut first = None;
    for _ in 0..10 {
        g.tick(true);
        s.step_pick(g.clk.ms(), &mut g.core);
        if let Some(m) = s.due_mav(g.clk.ms()) {
            first = Some(m);
            break;
        }
    }
    assert!(matches!(first, Some(CalMsg::Sweep(_))));
    assert_eq!(s.state(), State::AwaitAck);
    assert_eq!(g.core.op(), 112);
    let snap = s.snapshot();
    assert_eq!(
        (snap.channel, snap.pick_note.as_str()),
        (112, "auto: moved to 112")
    );
}

// ---- calibration's width hold ----

fn called_set_width(c: &FakeCard) -> bool {
    c.calls().iter().any(|l| l.starts_with("set_width"))
}

/// A 20 MHz link calibrating 40: the cards go to 40 for the sweep and come
/// back to the link's width after, and the link's own width never moves.
#[test]
fn set_cal_width_holds_the_cards_at_40_and_gives_them_back() {
    let mut g = Rig::build(2, true, 40, &[], 20);
    for _ in 0..10 {
        g.tick(true);
    }
    assert_eq!(g.core.width_mhz(), 20);
    for c in &g.cards {
        c.s().calls.clear();
    }
    g.core.set_cal_width(40);
    assert_eq!(g.core.width_mhz(), 20);
    assert!(g.cards.iter().all(|c| c.s().width_mhz == 40));
    assert!(g.has_line("maburgs link: cards 20 -> 40 MHz (calibration)"));
    // Held through the ticks: the per-tick resync aims at the hold.
    for _ in 0..10 {
        g.tick(true);
    }
    assert!(g.cards.iter().all(|c| c.s().width_mhz == 40));
    g.core.set_cal_width(0);
    assert!(g.cards.iter().all(|c| c.s().width_mhz == 20));
    assert!(g.has_line("maburgs link: cards 40 -> 20 MHz (calibration)"));
    for c in &g.cards {
        c.s().calls.clear();
    }
    g.core.set_cal_width(0); // no change: nothing touched
    assert!(!g.cards.iter().any(|c| called_set_width(c)));
}

/// A hold the cards refused leaves a retry time behind; giving the cards back
/// drops it (mabur's `set_cal_width` clears `width_retry_ms_`), so no try
/// from the old hold fires under the next one.
#[test]
fn set_cal_width_drops_the_retry_of_a_refused_hold() {
    let mut g = Rig::build(2, true, 40, &[], 20);
    for _ in 0..10 {
        g.tick(true);
    }
    for c in &g.cards {
        c.s().fail_set_width = true;
    }
    g.core.set_cal_width(40);
    assert!(g.cards.iter().all(|c| c.s().width_mhz == 20));
    assert!(g.core.width_retry_ms.iter().all(|&t| t != 0.0));
    g.core.set_cal_width(0);
    assert!(g.core.width_retry_ms.iter().all(|&t| t == 0.0));
}

/// A link already at 40: the hold changes nothing on the cards.
#[test]
fn set_cal_width_on_a_40_mhz_link_touches_no_card() {
    let mut g = Rig::new(2, true, 40);
    for _ in 0..10 {
        g.tick(true);
    }
    for c in &g.cards {
        c.s().calls.clear();
    }
    g.core.set_cal_width(40);
    assert!(!g.cards.iter().any(|c| called_set_width(c)));
    g.core.set_cal_width(0);
    assert!(!g.cards.iter().any(|c| called_set_width(c)));
}

// ---- scan.members and the observe lease (mabur test_channel_core.cpp, #122) ----

impl Rig {
    /// Live ticks for `ms` of rig time, renewing with observe(3000) every
    /// 1000 ms the way a picker does while it stays open.
    fn lease_ticks(&mut self, ms: u64, renew: bool) {
        let end = self.clk.ms() + ms;
        let mut last = self.clk.ms();
        while self.clk.ms() < end {
            if renew && self.clk.ms() - last >= 1000 {
                self.core.observe(3000);
                last = self.clk.ms();
            }
            self.live_tick(0);
        }
    }
    /// Dwells on `card` per member since record `from`.
    fn card_dwell_counts(&self, card: usize, from: usize, chs: &[u8]) -> Vec<usize> {
        let r = self.rec();
        chs.iter()
            .map(|&ch| {
                r.dwells[from..]
                    .iter()
                    .filter(|(c, d)| *c == card && d.ch == ch)
                    .count()
            })
            .collect()
    }
    fn dwells(&self) -> usize {
        self.rec().dwells.len()
    }
    fn member(&self, ch: u8) -> StatsScanMemberIn {
        self.core
            .snapshot(0)
            .members
            .into_iter()
            .find(|m| m.ch == ch)
            .expect("a member")
    }
    fn linked_frozen(&mut self, ticks: usize) {
        self.freeze_auto();
        self.link_up();
        for _ in 0..ticks {
            self.live_tick(0);
        }
    }
}

fn near(v: Option<f64>, want: f64) -> bool {
    v.is_some_and(|v| (v - want).abs() < 1.0)
}

#[test]
fn members_export_every_member_in_config_order() {
    let g = Rig::auto(2);
    let m = g.core.snapshot(0).members;
    assert_eq!(m.len(), 4);
    assert_eq!((m[0].ch, m[3].ch), (40, 144));
    assert_eq!((m[3].pair_lo, m[3].pair_hi), (Some(140), Some(144)));
    assert!(m[1].busy_pct.is_none() && m[1].src.is_empty() && m[1].age_ms.is_none());
    assert!(m[0].is_op && !m[0].is_pin);
}

#[test]
fn members_have_no_pair_at_width_20() {
    let g = Rig::build(2, false, 0, &[], 20);
    for m in g.core.snapshot(0).members {
        assert!(m.pair_lo.is_none() && m.pair_hi.is_none());
    }
}

/// A boot-scout dwell with an NHM reading lands in the table; newest wins.
#[test]
fn members_take_the_newest_dwell_reading() {
    let mut g = Rig::auto(2);
    g.cards[1].s().nhm_busy_pct_on[112] = 40;
    let count_112 = |g: &Rig| {
        g.rec()
            .dwells
            .iter()
            .filter(|(c, d)| *c == 1 && d.ch == 112 && d.busy_valid)
            .count()
    };
    for _ in 0..40 {
        if count_112(&g) > 0 {
            break;
        }
        g.tick(false);
    }
    assert!(count_112(&g) > 0);
    let m = g.member(112);
    assert_eq!(m.src, "scan");
    assert_eq!(m.busy_pct, Some(40.0));
    assert!(m.blocked); // 40 >= radio.scan.busy.blocked_pct (30)
    g.cards[1].s().nhm_busy_pct_on[112] = 5;
    let had = count_112(&g);
    for _ in 0..40 {
        if count_112(&g) > had {
            break;
        }
        g.tick(false);
    }
    assert!(count_112(&g) > had);
    let m = g.member(112);
    assert_eq!(m.busy_pct, Some(5.0));
    assert!(!m.blocked);
}

/// Linked: op's figure is the window's foreign busy, the best card's.
#[test]
fn members_op_reads_the_verdict_window_min_across_cards() {
    let mut g = Rig::auto(2);
    g.freeze_auto();
    g.link_up();
    g.cards[0].s().nhm_busy_pct_on[40] = 25;
    g.cards[1].s().nhm_busy_pct_on[40] = 60;
    // mabur's live_tick decodes a body each tick: a window with no seq
    // walk gives no busy reading.
    for _ in 0..2 * (g.cfg.hop.window_ms / 10) + 30 {
        g.decode_one();
        g.live_tick(0);
    }
    let m = g.member(40);
    assert_eq!(m.src, "op");
    assert!(near(m.busy_pct, 25.0), "{:?}", m.busy_pct);
}

/// A retune that fails never lands a reading.
#[test]
fn members_skip_a_failed_retune_dwell() {
    let mut g = Rig::auto(2);
    g.linked_frozen(10);
    g.cards[1].s().nhm_busy_pct_on[112] = 40;
    g.cards[1].s().retune_ok = false;
    assert_eq!(g.core.observe(3000), LeaseState::TwoCard);
    g.lease_ticks(1000, true);
    assert!(g.dwells() > 0);
    let m = g.member(112);
    assert!(m.busy_pct.is_none() && m.src.is_empty());
}

/// Auto, two cards, linked, pick closed: rounds run back to back, every
/// member but op at the link width, and nothing moves.
#[test]
fn lease_two_card_rounds_run_back_to_back_auto() {
    let mut g = Rig::auto(2);
    g.linked_frozen(10);
    let d0 = g.dwells();
    assert_eq!(g.core.observe(3000), LeaseState::TwoCard);
    assert!(g.core.observing());
    g.lease_ticks(2000, true);
    let n = g.card_dwell_counts(1, d0, &[40, 64, 112, 144]);
    assert_eq!(n[0], 0, "op never dwelt on");
    assert!(n[1] >= 2 && n[2] >= 2 && n[3] >= 2, "{n:?}");
    for (c, d) in &g.rec().dwells[d0..] {
        assert_eq!(*c, 1);
        assert_eq!(d.width_mhz, 40);
        assert!(d.observe_ms > 0);
    }
    assert!(!g.has_hop("relocate") && !g.has_hop("order"));
    assert_eq!(g.core.op(), 40);
    assert!(g.has_line("maburgs channel: observe lease 3000 (two_card)"));
}

/// Pinned: the lease freezes the scout and turns measuring on for its
/// rounds, never unpins, and hands the card back once it lapses.
#[test]
fn lease_two_card_rounds_run_pinned() {
    let mut g = Rig::new(2, true, 40);
    g.link_up();
    for _ in 0..10 {
        g.live_tick(0);
    }
    let d0 = g.dwells();
    assert_eq!(g.core.observe(3000), LeaseState::TwoCard);
    g.lease_ticks(2000, true);
    let n = g.card_dwell_counts(1, d0, &[40, 64, 112, 144]);
    assert_eq!(n[0], 0);
    assert!(n[1] >= 1 && n[2] >= 1 && n[3] >= 1, "{n:?}");
    assert!(g.core.pinned());
    assert_eq!(g.core.op(), 40);
    g.lease_ticks(4000, false);
    assert!(!g.core.observing());
    assert!(!g.core.scout_owns_card(1));
    assert_eq!(g.ch(1), 40);
    // A link loss later: the pinned search dwells on the members, no observe.
    let d1 = g.dwells();
    for _ in 0..1200 {
        if g.dwells() >= d1 + 4 {
            break;
        }
        g.tick(false);
    }
    assert!(g.dwells() >= d1 + 4);
    for (_, d) in &g.rec().dwells[d1..] {
        assert_eq!(d.observe_ms, 0);
    }
}

#[test]
fn lease_lapses_without_renewal() {
    let mut g = Rig::auto(2);
    g.linked_frozen(10);
    assert_eq!(g.core.observe(3000), LeaseState::TwoCard);
    g.lease_ticks(3500, false);
    assert!(!g.core.observing());
    for _ in 0..20 {
        if !g.core.scout_owns_card(1) {
            break;
        }
        g.live_tick(0);
    }
    assert!(!g.core.scout_owns_card(1));
    let d = g.dwells();
    g.lease_ticks(2000, false);
    assert_eq!(g.dwells(), d);
    assert_eq!(g.count_lines("observe lease lapsed"), 1);
}

/// A lease that runs out mid-round lets that round finish.
#[test]
fn lease_lapse_mid_round_lets_the_round_finish() {
    let mut g = Rig::auto(2);
    g.linked_frozen(10);
    let d0 = g.dwells();
    assert_eq!(g.core.observe(500), LeaseState::TwoCard);
    g.live_tick(0);
    assert!(g.core.scout_owns_card(1));
    for _ in 0..20 {
        if !g.core.scout_owns_card(1) {
            break;
        }
        g.live_tick(0);
    }
    assert!(!g.core.observing());
    assert!(!g.core.scout_owns_card(1));
    let n = g.card_dwell_counts(1, d0, &[40, 64, 112, 144]);
    assert_eq!(n, vec![0, 1, 1, 1]);
}

#[test]
fn lease_observe_zero_releases_on_the_next_tick() {
    let mut g = Rig::auto(2);
    g.linked_frozen(10);
    assert_eq!(g.core.observe(3000), LeaseState::TwoCard);
    for _ in 0..3 {
        g.live_tick(0);
    }
    assert!(g.core.scout_owns_card(1));
    assert_eq!(g.core.observe(0), LeaseState::None);
    assert!(!g.core.observing());
    g.live_tick(0);
    assert!(!g.core.scout_owns_card(1));
    let d = g.dwells();
    for _ in 0..50 {
        g.live_tick(0);
    }
    assert_eq!(g.dwells(), d);
}

#[test]
fn lease_clamps_to_500_and_5000() {
    let mut g = Rig::auto(2);
    g.linked_frozen(5);
    g.core.observe(100);
    g.clk.advance(499);
    assert!(g.core.observing());
    g.clk.advance(2);
    assert!(!g.core.observing());
    g.core.observe(60000);
    g.clk.advance(4999);
    assert!(g.core.observing());
    g.clk.advance(2);
    assert!(!g.core.observing());
}

/// A round starting on the tick BootPick hands out a relocate would hide
/// it from the controller: none starts while one is due or flying.
#[test]
fn lease_round_never_starts_with_a_relocate_due() {
    let mut g = Rig::new(2, true, 40);
    g.session_opened_on(64, 64);
    g.link_up();
    g.live_tick(0);
    assert_eq!(g.core.observe(3000), LeaseState::TwoCard);
    for _ in 0..200 {
        if g.has_line("relocate 64 -> 40 placed") {
            break;
        }
        g.live_tick(0);
        assert!(!g.core.scout_owns_card(1));
    }
    assert!(g.has_line("relocate 64 -> 40 placed"));
    assert!(!g.core.scout_owns_card(1));
}

#[test]
fn lease_none_while_calibration_owns_the_scout() {
    let mut g = Rig::auto(2);
    g.linked_frozen(10);
    g.tick_cal(true, true);
    assert_eq!(g.core.lease_state(), LeaseState::None);
    assert_eq!(g.core.observe(3000), LeaseState::None);
    g.core.observe(0);
    for _ in 0..5 {
        g.live_tick(0);
    }
    // A calibration pick arriving while a lease round runs ends the lease.
    assert_eq!(g.core.observe(3000), LeaseState::TwoCard);
    for _ in 0..3 {
        g.live_tick(0);
    }
    assert!(g.core.scout_owns_card(1));
    let now = g.ms();
    g.core.begin_cal_measure(now);
    assert!(!g.core.observing());
    assert_eq!(g.core.lease_state(), LeaseState::None);
    for _ in 0..50 {
        if g.core.cal_measure_best().is_some() {
            break;
        }
        g.core.observe(3000); // the picker still renewing: no effect
        g.live_tick(0);
    }
    assert!(g.has_line("calibration measure round on card 1"));
    assert!(g.core.cal_measure_best().is_some());
    g.core.cancel_cal_measure();
}

#[test]
fn lease_none_while_boot_pick_open() {
    let mut g = Rig::auto(2);
    g.tick(false);
    assert!(g.core.pick_open());
    assert_eq!(g.core.observe(3000), LeaseState::None);
    let d = g.dwells();
    for _ in 0..5 {
        g.tick(false);
    }
    assert!(g.core.pick_open());
    assert!(g.core.observing());
    assert!(!g.has_line("observe lease 3000"));
    assert!(g.dwells() > d, "the boot scout measures anyway");
}

#[test]
fn lease_state_follows_the_roster() {
    let mut g = Rig::auto(2);
    g.freeze_auto(); // unlinked, pick frozen, scout searching
    assert_eq!(g.core.lease_state(), LeaseState::Search);
    g.link_up();
    g.live_tick(0);
    assert_eq!(g.core.lease_state(), LeaseState::TwoCard);
    let mut g = Rig::auto(1);
    g.freeze_auto(); // links too
    g.live_tick(0);
    assert_eq!(g.core.lease_state(), LeaseState::OneCard);
    assert_eq!(LeaseState::None.as_str(), "none");
    assert_eq!(LeaseState::TwoCard.as_str(), "two_card");
    assert_eq!(LeaseState::OneCard.as_str(), "one_card");
    assert_eq!(LeaseState::Search.as_str(), "search");
}

/// Pinned: a calibration run starts without a measure round, so its first
/// tick cancels the open lease round, and none starts while it runs.
#[test]
fn lease_round_cancelled_when_a_pinned_calibration_run_starts() {
    let mut g = Rig::new(2, true, 40);
    g.link_up();
    for _ in 0..10 {
        g.live_tick(0);
    }
    assert_eq!(g.core.observe(3000), LeaseState::TwoCard);
    for _ in 0..2 {
        g.live_tick(0);
    }
    assert!(g.core.scout_owns_card(1));
    let d1 = g.dwells();
    g.tick_cal(true, true);
    assert!(!g.core.observing());
    g.tick_cal(true, true);
    assert!(!g.core.scout_owns_card(1));
    assert_eq!(g.ch(1), 40);
    assert!(g.dwells() <= d1 + 1);
    let d = g.dwells();
    for i in 0..100 {
        if i % 50 == 0 {
            assert_eq!(g.core.observe(3000), LeaseState::None);
        }
        g.tick_cal(true, true);
        assert!(!g.core.scout_owns_card(1));
    }
    assert_eq!(g.dwells(), d);
}

#[test]
fn lease_round_released_when_scout_card_dies() {
    let mut g = Rig::auto(2);
    g.linked_frozen(10);
    assert_eq!(g.core.observe(3000), LeaseState::TwoCard);
    for _ in 0..3 {
        g.live_tick(0);
    }
    assert!(g.core.scout_owns_card(1));
    g.cards[1].s().is_alive = false;
    g.core.on_card_died(1);
    g.live_tick(0);
    assert!(!g.core.scout_owns_card(1));
    let d = g.dwells();
    g.lease_ticks(2000, true);
    assert_eq!(g.dwells(), d, "no round on a dead card");
    assert!(!g.core.scout_owns_card(1));
}

fn one_card_lease_bursts_three_times_in_2500_ms(g: &mut Rig) {
    for _ in 0..10 {
        g.live_tick(0);
    }
    let d0 = g.dwells();
    assert_eq!(g.core.observe(3000), LeaseState::OneCard);
    g.lease_ticks(2500, true);
    let n = g.card_dwell_counts(0, d0, &[40, 64, 112, 144]);
    assert_eq!(n, vec![0, 3, 3, 3]);
    assert_eq!(g.ch(0), 40);
    assert_eq!(g.core.op(), 40);
}

#[test]
fn lease_one_card_bursts_once_per_period_auto_and_pinned() {
    let mut g = Rig::auto(1);
    g.freeze_auto();
    one_card_lease_bursts_three_times_in_2500_ms(&mut g);
    let mut g = Rig::new(1, true, 40);
    g.link_up();
    one_card_lease_bursts_three_times_in_2500_ms(&mut g);
    assert!(g.core.pinned());
}

/// The burst pacing belongs to one lease: a lease taken again inside one
/// period bursts at once.
#[test]
fn lease_one_card_reopened_within_a_period_bursts_at_once() {
    let mut g = Rig::auto(1);
    g.freeze_auto();
    for _ in 0..10 {
        g.live_tick(0);
    }
    let d0 = g.dwells();
    assert_eq!(g.core.observe(500), LeaseState::OneCard);
    let t0 = g.clk.ms();
    g.live_tick(0);
    assert!(g.dwells() > d0);
    for _ in 0..100 {
        if g.has_line("observe lease lapsed") {
            break;
        }
        g.live_tick(0);
    }
    assert!(g.has_line("maburgs channel: observe lease lapsed"));
    assert!(g.clk.ms() - t0 < g.cfg.hop.lease_sweep_period_ms as u64);
    let d1 = g.dwells();
    assert_eq!(g.core.observe(3000), LeaseState::OneCard);
    g.live_tick(0);
    assert!(g.dwells() > d1);
}

/// A lease burst's visits never reach HopRanker, while its dwells land in
/// scan.members.
#[test]
fn lease_one_card_feeds_no_hop_ranker_visit() {
    let mut g = Rig::auto(1);
    g.freeze_auto();
    {
        let mut s = g.cards[0].s();
        s.nhm_busy_pct_on[64] = 8;
        s.nhm_busy_pct_on[112] = 40;
        s.nhm_busy_pct_on[144] = 2;
    }
    for _ in 0..10 {
        g.live_tick(0);
    }
    let before = g.core.hop_ranker().ranking(g.ms());
    let d0 = g.dwells();
    assert_eq!(g.core.observe(3000), LeaseState::OneCard);
    g.live_tick(0);
    let n = g.card_dwell_counts(0, d0, &[40, 64, 112, 144]);
    assert_eq!(n, vec![0, 1, 1, 1]);
    let after = g.core.hop_ranker().ranking(g.ms());
    assert_eq!(after.len(), before.len());
    for (a, b) in after.iter().zip(&before) {
        assert_eq!((a.ch, a.visits, a.ranked), (b.ch, b.visits, b.ranked));
    }
    let m = g.core.snapshot(0).members;
    assert!(m[0].is_op && m[0].src != "scan");
    assert!(m[1].src == "scan" && near(m[1].busy_pct, 8.0) && !m[1].blocked);
    assert!(m[2].src == "scan" && near(m[2].busy_pct, 40.0) && m[2].blocked);
    assert!(m[3].src == "scan" && near(m[3].busy_pct, 2.0));
}

/// The sampler stamps a pass before it takes the card's lock, so a pass
/// started mid-burst reads its counters after the burst: the masked
/// record re-seeds from the burst's end, and the next window's own airtime
/// is not spread over the burst. Half the air is ours here: 50 %.
#[test]
fn lease_one_card_record_stamped_mid_burst_reseeds_from_the_burst_end() {
    let mut g = Rig::auto(1);
    g.freeze_auto();
    let own_tick = |g: &mut Rig| {
        g.cards[0].s().fr.own_air_us += 5000; // 5 of every 10 ms
        g.live_tick(0);
    };
    for _ in 0..30 {
        own_tick(&mut g);
    }
    assert_eq!(g.core.observe(3000), LeaseState::OneCard);
    let d0 = g.dwells();
    let t_burst = g.clk.ms();
    own_tick(&mut g); // the burst runs inside this tick
    assert_eq!(g.dwells(), d0 + 3);
    let end = g.clk.ms() - 10;
    assert!(end >= t_burst + 30);
    let r = g.clk.ms();
    g.core.run_sampler_step_stamped(t_burst + 1);
    while g.clk.ms() <= r + 160 {
        own_tick(&mut g);
    }
    let e = g.core.snapshot(0).energy[0].unwrap();
    let own = e.own_air_pct.unwrap();
    assert!((own - 50.0).abs() < 0.5, "{own}");
}

#[test]
fn lease_one_card_at_width_20_bursts_without_a_width_switch() {
    let mut g = Rig::build(1, false, 0, &[], 20);
    g.freeze_auto();
    for _ in 0..10 {
        g.live_tick(0);
    }
    g.cards[0].s().calls.clear();
    let d0 = g.dwells();
    assert_eq!(g.core.observe(3000), LeaseState::OneCard);
    g.live_tick(0);
    let n = g.card_dwell_counts(0, d0, &[40, 64, 112, 144]);
    assert_eq!(n, vec![0, 1, 1, 1]);
    for (_, d) in &g.rec().dwells[d0..] {
        assert_eq!(d.width_mhz, 20);
    }
    let s = g.cards[0].s();
    assert!(!s.calls.iter().any(|c| c.starts_with("set_width")));
    assert_eq!((s.width_mhz, s.ch), (20, 40));
}

/// Unlinked and frozen, the scout searches: a held lease turns its search
/// dwells into observes, filling scan.members with no link at all; with no
/// lease every member stays unmeasured.
#[test]
fn lease_search_observes_while_unlinked_and_frozen() {
    let jam = |g: &Rig| {
        for c in &g.cards {
            let mut s = c.s();
            s.nhm_busy_pct_on[64] = 8;
            s.nhm_busy_pct_on[112] = 40;
            s.nhm_busy_pct_on[144] = 2;
        }
    };
    let mut g = Rig::new(2, true, 40);
    jam(&g);
    assert!(!g.core.pick_open());
    assert_eq!(g.core.lease_state(), LeaseState::Search);
    for _ in 0..400 {
        g.tick(false);
    }
    for m in g.core.snapshot(0).members {
        if !m.is_op {
            assert!(m.busy_pct.is_none(), "{m:?}");
        }
    }
    let mut g = Rig::new(2, true, 40);
    jam(&g);
    assert_eq!(g.core.observe(3000), LeaseState::Search);
    assert!(g.has_line("maburgs channel: observe lease 3000 (search)"));
    let mut last = g.clk.ms();
    for _ in 0..400 {
        if g.clk.ms() - last >= 2000 {
            g.core.observe(3000);
            last = g.clk.ms();
        }
        g.tick(false);
    }
    let m = g.core.snapshot(0).members;
    assert!(m[1].src == "scan" && near(m[1].busy_pct, 8.0) && !m[1].blocked);
    assert!(m[2].src == "scan" && near(m[2].busy_pct, 40.0) && m[2].blocked);
    assert!(m[3].src == "scan" && near(m[3].busy_pct, 2.0));
}

#[test]
fn observe_verb_replies() {
    let mut g = Rig::auto(2);
    g.linked_frozen(5);
    assert_eq!(g.core.observe_verb("3000"), "ok observe two_card");
    assert!(g.core.observing());
    assert_eq!(g.core.observe_verb("0"), "ok observe none");
    assert!(!g.core.observing());
    for bad in ["x", "", "-5", "3000 1"] {
        assert_eq!(g.core.observe_verb(bad), "err usage: observe <ms>");
    }
    assert_eq!(
        g.core.observe_verb("99999999999999999999"),
        "ok observe two_card"
    );
    g.clk.advance(5001);
    assert!(!g.core.observing());
}
