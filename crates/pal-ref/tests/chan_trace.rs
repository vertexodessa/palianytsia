//! ChannelCore trace parity: mabur's C++ core (pal_ref::chan_trace) and the
//! Rust port (pal_core::chan::core) run the same script, unthreaded, each
//! over its own scripted FakeCard(s) and FakeClock. After every step both
//! sides' sink records, card calls and vrx frames must be equal; the first
//! difference fails with the lines around it.
//!
//! The record shapes here must match `cpp/chan_trace.cpp` character for
//! character.

use std::path::Path;
use std::sync::{Arc, Mutex};

use pal_core::aggregator::Aggregator;
use pal_core::chan::core::{
    ChannelCore, ChannelCoreCfg, ChannelSink, ChannelSnapshot, ChannelTickIn,
};
use pal_core::chan::fake::{FakeCard, FakeClock};
use pal_core::chan::hop::HopEvent;
use pal_core::chan::plan::MoveEvent;
use pal_core::chan::radio::LinkCard;
use pal_core::chan::ranker::RankEntry;
use pal_core::chan::scout::ScoutDwell;
use pal_core::chan::verdict::{VerdictCardIn, VerdictLinkIn, VerdictOut};
use pal_core::config::{load_config, Config};
use pal_core::ladder::LinkHealth;
use pal_core::vrx::{VrxCfg, VrxController, VrxState};
use pal_ref::chan_trace::{CppTrace, TraceCfg};
use pal_wire::rc::{self, DiscAck};

const CONFIG: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../third_party/mabur/gs/bundle/maburgs.default.toml"
);
const NONCE: u32 = 0x5a17_c0de;

fn num(x: f64) -> String {
    if x.is_nan() {
        "nan".into()
    } else {
        // Readable, then the exact bits: equal lines mean bit-equal f64s.
        format!("{x:.3}~{:x}", x.to_bits())
    }
}
fn b(x: bool) -> u8 {
    x as u8
}
fn opt<T: std::fmt::Display>(x: Option<T>) -> String {
    x.map_or("-".into(), |v| v.to_string())
}

struct TSink(Arc<Mutex<Vec<String>>>);

impl TSink {
    fn emit(&self, s: String) {
        self.0.lock().unwrap().push(s);
    }
}

impl ChannelSink for TSink {
    fn dwell(&mut self, t: f64, card: usize, d: &ScoutDwell) {
        self.emit(format!(
            "D t={} card={card} seq={} ch={} w={} off={} round={} ts={} te={} rt={} st={} ob={} \
             vfa={} fa={} cca={} vigi={} igi={} vnhm={} fl={} own={} fr={} flv={} fld={} ins={} \
             to={} rd={} bk={} bv={} bp={}",
            num(t),
            d.seq,
            d.ch,
            d.width_mhz,
            d.offset,
            d.round,
            d.t_start_ms,
            d.t_end_ms,
            d.retune_us,
            d.settle_ms,
            d.observe_ms,
            b(d.valid_fa),
            d.fa_ofdm,
            d.cca_ofdm,
            b(d.valid_igi),
            d.igi,
            b(d.valid_nhm),
            d.flags,
            d.dvr_frames,
            d.frames,
            b(d.floor_valid),
            d.floor_dbm,
            b(d.in_session),
            d.to_us,
            d.read_us,
            d.back_us,
            b(d.busy_valid),
            num(d.busy_pct)
        ));
    }
    fn pick(&mut self, t: f64, p: Option<u8>, rounds: u64, all: &[RankEntry], mr: i32) {
        let mut s = format!(
            "K t={} pick={} rounds={rounds} mr={mr} all=",
            num(t),
            opt(p)
        );
        for e in all {
            s += &format!(
                "{}:{}:{}:{}:{}:{}:{};",
                e.ch,
                e.worst_busy,
                e.visits,
                b(e.floor_valid),
                e.floor_dbm,
                b(e.busy_valid),
                num(e.worst_busy_pct)
            );
        }
        self.emit(s);
    }
    fn moved(&mut self, e: &MoveEvent) {
        let card = e.card.map_or(-1, |c| c as i64);
        self.emit(format!(
            "M t={} card={card} {}->{} {}",
            num(e.t_ms),
            e.from,
            e.to,
            e.reason.as_str()
        ));
    }
    fn verdict(&mut self, t: f64, o: &VerdictOut, cards: &[VerdictCardIn], l: &VerdictLinkIn) {
        let mut s = format!(
            "V t={} v={} ev={} rr={} trig={} rref={} dr={} ts={} tw={} frz={} first={} \
             link={}:{}:{}:{} cards=",
            num(t),
            o.v.as_str(),
            o.evidence,
            o.ref_rung,
            b(o.trigger),
            num(o.ref_rssi_dbm),
            num(o.d_rssi_db),
            num(o.t_start_ms),
            num(o.t_ms),
            b(o.ref_frozen),
            b(o.first_interfered),
            num(l.pre_fec_loss),
            l.recovered,
            b(l.starved),
            l.au_count
        );
        for c in cards {
            s += &format!(
                "{}:{}:{}:{}:{}:{}:{}:{}:{}:{}:{};",
                b(c.valid),
                c.foreign,
                c.crc_fail,
                c.fa,
                c.cca,
                num(c.rssi_dbm),
                num(c.snr_db),
                b(c.snr_valid),
                b(c.busy_valid),
                num(c.nhm_busy_pct),
                num(c.own_air_pct)
            );
        }
        self.emit(s);
    }
    fn hop(&mut self, e: &HopEvent) {
        self.emit(format!(
            "H t={} {} epoch={} target={} score={} el={} reason={}",
            num(e.t_ms),
            e.kind,
            e.epoch,
            e.target,
            e.score,
            num(e.elapsed_ms),
            e.reason
        ));
    }
    fn log(&mut self, line: &str) {
        self.emit(format!("L {line}"));
    }
}

fn snapshot_line(s: &ChannelSnapshot) -> String {
    let h = &s.hop;
    let mut l = format!(
        "P ch={} state={} rounds={} pick={} v={} ev={} rr={} epoch={} hs={} target={} hops={} \
         holds={} last={} drop={} gated={} energy=",
        s.channel,
        s.scan_state,
        s.scan_rounds,
        opt(s.scan_pick),
        h.verdict,
        h.evidence,
        opt(h.ref_rung),
        h.epoch,
        h.state,
        opt(h.target),
        h.hops,
        h.holds,
        opt(h.last_ms),
        h.sampler_dropped,
        s.scout_gated_sends
    );
    for e in &s.energy {
        match e {
            None => l += "-;",
            Some(e) => {
                l += &format!(
                    "{}:{}:{}:{}:{}:{};",
                    e.cca,
                    e.fa,
                    e.own,
                    e.foreign,
                    e.busy_pct.map_or("-".into(), num),
                    e.own_air_pct.map_or("-".into(), num)
                )
            }
        }
    }
    l += " dwell=";
    for d in &s.dwell {
        match d {
            None => l += "-;",
            Some(d) => l += &format!("{}:{}:{};", d.visits, d.score, d.cost_us),
        }
    }
    l
}

/// Both cores, each on its own fakes and clock, stepped in lockstep.
struct Duo {
    cfg: Config,
    clk_r: FakeClock,
    clk_c: FakeClock,
    cr: Vec<Arc<FakeCard>>,
    cc: Vec<Arc<FakeCard>>,
    core: ChannelCore,
    vrx: VrxController,
    agg: Aggregator,
    lines_r: Arc<Mutex<Vec<String>>>,
    cpp: CppTrace,
    tr: Vec<String>,
    tc: Vec<String>,
}

fn fakes(clk: &FakeClock, n: usize, start: u8, width: u8) -> Vec<Arc<FakeCard>> {
    (0..n)
        .map(|i| {
            let c = FakeCard::new(clk.clone(), start);
            // The boot scout card opens at 20, every other at radio.width.
            c.s().width_mhz = if i == n - 1 { 20 } else { width };
            c
        })
        .collect()
}

fn dyn_cards(v: &[Arc<FakeCard>]) -> Vec<Arc<dyn LinkCard>> {
    v.iter().map(|c| c.clone() as Arc<dyn LinkCard>).collect()
}

impl Duo {
    fn with(n: usize, pin: Option<u8>, start: u8, channels: &[u8], max_ms: Option<i32>) -> Self {
        let mut cfg = load_config(Path::new(CONFIG), None).unwrap();
        cfg.radio.channels = channels.to_vec();
        cfg.radio.width = 40;
        cfg.radio.pin = pin;
        if let Some(m) = max_ms {
            cfg.radio.scan.max_ms = m;
        }
        let (clk_r, clk_c) = (FakeClock::default(), FakeClock::default());
        let cr = fakes(&clk_r, n, start, cfg.radio.width);
        let cc = fakes(&clk_c, n, start, cfg.radio.width);
        let lines_r = Arc::new(Mutex::new(Vec::new()));
        let (st, k1, k2, k3) = (lines_r.clone(), clk_r.clone(), clk_r.clone(), clk_r.clone());
        let core = ChannelCore::new(
            ChannelCoreCfg {
                radio: cfg.radio.clone(),
                hop: cfg.hop.clone(),
                key: cfg.link.key,
                start_ch: start,
                threaded: false,
                store_name: "the-store".into(),
                ..Default::default()
            },
            dyn_cards(&cr),
            Box::new(TSink(lines_r.clone())),
            Box::new(move |ch| {
                st.lock().unwrap().push(format!("S {ch}"));
                true
            }),
            Arc::new(move || k1.ms()),
            Arc::new(move || k2.ms() * 1000),
            Arc::new(move |ms| k3.advance(ms as u64)),
        );
        let (k1, k2, k3) = (clk_c.clone(), clk_c.clone(), clk_c.clone());
        let cpp = CppTrace::new(
            &TraceCfg {
                config_path: CONFIG,
                channels,
                width: 40,
                pin,
                max_ms,
                start_ch: start,
                rz_nonce: NONCE,
            },
            dyn_cards(&cc),
            Arc::new(move || k1.ms()),
            Arc::new(move || k2.ms() * 1000),
            Arc::new(move |ms| k3.advance(ms as u64)),
        )
        .unwrap();
        Duo {
            vrx: VrxController::new(VrxCfg::from_config(&cfg, start, NONCE)),
            agg: Aggregator::new(&cfg.uep_layers(), 32, n, 0),
            cfg,
            clk_r,
            clk_c,
            cr,
            cc,
            core,
            lines_r,
            cpp,
            tr: Vec::new(),
            tc: Vec::new(),
        }
    }
    fn new(n: usize, pin: Option<u8>, start: u8) -> Self {
        Self::with(n, pin, start, &[40, 64, 112, 144], None)
    }

    /// Drains both sides into their traces and fails on the first
    /// difference.
    fn sync(&mut self, step: &str) {
        let from = self.tr.len().min(self.tc.len());
        self.tr.append(&mut self.lines_r.lock().unwrap());
        self.tc.append(&mut self.cpp.take_lines());
        for (i, c) in self.cr.iter().enumerate() {
            for call in std::mem::take(&mut c.s().calls) {
                self.tr.push(format!("C{i} {call}"));
            }
        }
        for (i, c) in self.cc.iter().enumerate() {
            for call in std::mem::take(&mut c.s().calls) {
                self.tc.push(format!("C{i} {call}"));
            }
        }
        assert_eq!(
            self.clk_r.ms(),
            self.clk_c.ms(),
            "clocks apart after {step}"
        );
        let n = self.tr.len().max(self.tc.len());
        for i in from..n {
            let (r, c) = (self.tr.get(i), self.tc.get(i));
            if r != c {
                let lo = i.saturating_sub(12);
                let ctx = |t: &[String]| t[lo..i.min(t.len())].join("\n  ");
                panic!(
                    "trace differs at line {i} after {step} (t={} ms)\n\
                     common before:\n  {}\nrust: {:?}\nc++:  {:?}\n\
                     rust next: {:?}\nc++ next:  {:?}",
                    self.clk_r.ms(),
                    ctx(&self.tr),
                    r,
                    c,
                    self.tr.get(i + 1..(i + 4).min(self.tr.len())),
                    self.tc.get(i + 1..(i + 4).min(self.tc.len())),
                );
            }
        }
    }

    fn ms(&self) -> f64 {
        self.clk_r.ms() as f64
    }
    fn advance(&mut self, ms: u64) {
        self.clk_r.advance(ms);
        self.clk_c.advance(ms);
    }
    /// Both fakes, same change.
    fn cards(&self, f: impl Fn(usize, &mut pal_core::chan::fake::FakeState)) {
        for (i, c) in self.cr.iter().enumerate() {
            f(i, &mut c.s());
        }
        for (i, c) in self.cc.iter().enumerate() {
            f(i, &mut c.s());
        }
    }

    fn tick_tx(&mut self, session: bool, tx: usize) -> (bool, bool) {
        let now = self.ms();
        let inp = ChannelTickIn {
            now_ms: now,
            in_session: session,
            tx_card: tx,
            agg: Some(&self.agg),
        };
        let o = self.core.tick(&inp, &mut self.vrx);
        let c = self.cpp.tick(now, session, tx);
        self.sync("tick");
        assert_eq!((o.tx_frozen, o.dwell_busy), c, "tick out at {now}");
        c
    }
    fn tick(&mut self, session: bool) {
        self.tick_tx(session, 0);
        self.advance(10);
    }
    fn snapshot(&mut self, tx: usize) {
        let l = snapshot_line(&self.core.snapshot(tx));
        self.lines_r.lock().unwrap().push(l);
        self.cpp.snapshot(tx);
        self.sync("snapshot");
    }
    fn vrx_rc(&mut self, wire: &[u8]) {
        let now = self.ms();
        self.vrx.on_rc_frame(wire, now);
        self.cpp.vrx_rc(now, wire);
        assert_eq!(
            self.vrx.link_state() == VrxState::Session,
            self.cpp.vrx_session()
        );
        assert_eq!(self.vrx.agreed_channel(), self.cpp.vrx_agreed());
    }
    fn vrx_video(&mut self) {
        let now = self.ms();
        self.vrx.on_video(now);
        self.cpp.vrx_video(now);
    }
    fn vrx_step(&mut self) -> Option<(Vec<u8>, bool)> {
        let now = self.ms();
        let h = LinkHealth {
            sample_valid: true,
            ..Default::default()
        };
        let r = self.vrx.step(now, &h).map(|o| (o.frame, o.is_disc));
        let c = self.cpp.vrx_step(now);
        assert_eq!(r, c, "vrx frame at {now}");
        assert_eq!(self.vrx.proposal(), self.cpp.vrx_proposal());
        r
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
    fn session_opened_on(&mut self, rx_ch: u8, agreed: u8) {
        let wire = rc::pack_disc_ack(&self.ack(agreed));
        let now = self.ms();
        self.core.on_rc_body(rx_ch);
        self.cpp.on_rc_body(rx_ch);
        self.core
            .on_session_opened(&rc::parse_disc_ack(&wire).unwrap(), now, &mut self.vrx);
        self.cpp.on_session_opened(&wire, now);
        self.sync("on_session_opened");
        assert_eq!(self.core.op(), self.cpp.op());
    }
    fn link_up(&mut self) {
        let mut ack = self.ack(self.core.op());
        ack.chip_caps = rc::CAP_FRAME_WIRE;
        self.vrx_rc(&rc::pack_disc_ack(&ack));
        for _ in 0..200 {
            if self.vrx.link_state() == VrxState::Session {
                break;
            }
            self.advance(10);
            self.vrx_video();
            self.vrx_step();
        }
        assert_eq!(self.vrx.link_state(), VrxState::Session);
    }
    fn note_video(&mut self, ch: u8) {
        self.core.note_video(ch);
        self.cpp.note_video(ch);
    }
    fn move_edge(&mut self) {
        self.vrx.test_set_move_edge();
        self.cpp.vrx_move_edge();
    }
    fn pump_rcf(&mut self) -> Option<rc::Rcf> {
        for _ in 0..20 {
            self.advance(self.cfg.link.feedback_ms as u64);
            self.vrx_video();
            let out = self.vrx_step();
            self.tick_tx(true, 0);
            if let Some((f, false)) = out {
                self.core.note_sent(true, true);
                self.cpp.note_sent(true, true);
                return rc::parse_rcf(&f);
            }
        }
        None
    }
    fn interfere(&mut self, windows: usize) {
        for _ in 0..windows {
            self.cards(|_, s| s.fr.foreign += 40);
            for _ in 0..self.cfg.hop.window_ms / 10 + 1 {
                self.vrx_video();
                self.tick_tx(true, 0);
                self.advance(10);
            }
        }
    }
    fn freeze_auto(&mut self) {
        let n = self.cr.len();
        if n >= 2 {
            self.cards(|i, s| s.retune_ok = i != n - 1);
            for _ in 0..4000 {
                if !self.core.pick_open() {
                    break;
                }
                self.tick(false);
            }
            self.cards(|_, s| s.retune_ok = true);
        } else {
            self.link_up();
            for _ in 0..100 {
                if !self.core.pick_open() {
                    break;
                }
                self.tick(true);
            }
        }
        assert!(!self.core.pick_open() && !self.cpp.pick_open());
    }
    fn verify_window(&mut self) {
        for _ in 0..self.cfg.hop.verify_ms / 10 + 30 {
            self.vrx_video();
            self.tick(true);
        }
    }
    fn has(&self, needle: &str) -> bool {
        self.tr.iter().any(|l| l.contains(needle))
    }
    fn until(&mut self, n: usize, session: bool, needle: &str) {
        for i in 0..n {
            if self.has(needle) {
                break;
            }
            self.tick(session);
            if i % 100 == 0 {
                self.snapshot(0);
            }
        }
        assert!(self.has(needle), "never saw {needle:?}");
    }
    fn send_path(&mut self) {
        for card in 0..self.cr.len() {
            assert_eq!(self.core.may_send(card), self.cpp.may_send(card));
            assert_eq!(self.core.disc_targets(card), self.cpp.disc_targets(card));
        }
    }
}

#[test]
fn auto_two_card_commits_at_maturity() {
    let mut d = Duo::new(2, None, 40);
    d.cards(|i, s| {
        if i == 1 {
            s.cca_per_ms_on[40] = 5;
            s.cca_per_ms_on[36] = 5;
        }
    });
    d.send_path();
    d.until(3000, false, "pick frozen on");
    d.send_path();
    for _ in 0..50 {
        d.tick(false);
    }
    d.snapshot(0);
    d.snapshot(1);
}

#[test]
fn one_card_prelude_then_link_edge() {
    let mut d = Duo::new(1, None, 40);
    d.cards(|_, s| s.cca_per_ms_on[40] = 5);
    d.send_path();
    d.until(800, false, "one-card prelude ranking picks");
    d.send_path();
    d.move_edge();
    d.tick(true);
    d.tick(true);
    d.snapshot(0);
}

#[test]
fn max_ms_freezes_in_place() {
    let mut d = Duo::new(2, None, 40);
    d.cards(|i, s| s.retune_ok = i != 1);
    d.until(4000, false, "(max_ms)");
    d.snapshot(0);
}

#[test]
fn scout_card_death_and_reopen() {
    let mut d = Duo::new(2, None, 40);
    d.tick(false);
    d.cards(|i, s| s.is_alive = i != 1);
    d.core.on_card_died(1);
    d.cpp.on_card_died(1);
    d.sync("on_card_died");
    for _ in 0..20 {
        d.tick(false);
    }
    d.cards(|i, s| {
        if i == 1 {
            s.is_alive = true;
            s.width_mhz = 20;
        }
    });
    d.core.on_card_reopened(1);
    d.cpp.on_card_reopened(1);
    for _ in 0..50 {
        d.tick(false);
    }
    d.snapshot(0);
}

#[test]
fn pinned_relocation_confirm_and_dirty_pin() {
    let mut d = Duo::new(2, Some(40), 40);
    d.session_opened_on(64, 64);
    d.link_up();
    d.until(200, true, "relocate 64 -> 40 placed");
    assert_eq!(d.pump_rcf().unwrap().hop_ch, 40);
    d.note_video(40);
    d.tick(true);
    d.cards(|_, s| s.cca_per_ms_on[40] = 50);
    d.interfere(8);
    d.snapshot(0);
    assert!(d.has("verify_pass"));
}

fn order_duo(n: usize) -> Duo {
    let mut d = Duo::new(n, None, 40);
    d.freeze_auto();
    if n >= 2 {
        d.link_up();
    }
    for _ in 0..5 {
        d.tick(true);
    }
    d.cards(|_, s| s.cca_per_ms_on[40] = 50);
    d.interfere(4);
    assert!(d.has(" order epoch"));
    d
}

#[test]
fn two_card_order_confirm_verify() {
    let mut d = order_duo(2);
    let r = d.pump_rcf().unwrap();
    d.note_video(r.hop_ch);
    d.tick(true);
    d.verify_window();
    d.snapshot(0);
    assert!(d.has("verify_pass"));
    // The in-flight scout on the spare card, driven step by step.
    for _ in 0..6 {
        d.core.run_inflight_step();
        d.cpp.run_inflight_step();
        d.sync("run_inflight_step");
        for _ in 0..40 {
            d.vrx_video();
            d.tick(true);
        }
    }
    d.snapshot(0);
}

#[test]
fn two_card_order_withdraws_without_video() {
    let mut d = order_duo(2);
    d.pump_rcf().unwrap();
    for _ in 0..d.cfg.hop.confirm_extend_ms / 10 + 20 {
        d.vrx_video();
        d.tick(true);
    }
    d.snapshot(0);
    assert!(d.has(" withdraw epoch"));
}

#[test]
fn two_card_session_loss_mid_order() {
    let mut d = order_duo(2);
    d.tick(false);
    for _ in 0..30 {
        d.tick(false);
    }
    d.snapshot(0);
}

#[test]
fn one_card_order_rides_repeats() {
    let mut d = order_duo(1);
    for _ in 0..12 {
        d.pump_rcf();
    }
    let r = d.pump_rcf().unwrap();
    d.note_video(r.hop_ch);
    d.tick(true);
    d.verify_window();
    d.snapshot(0);
}

#[test]
fn auto_boot_hop_relocates_to_the_pick() {
    let mut d = Duo::new(2, None, 40);
    d.cards(|i, s| {
        if i == 1 {
            s.cca_per_ms_on[60] = 5;
            s.cca_per_ms_on[64] = 5;
        }
    });
    let visits = |d: &Duo, ch: u8| {
        let p = format!(" ch={ch} ");
        d.tr.iter()
            .filter(|l| l.starts_with("D ") && l.contains(&p))
            .count()
    };
    for _ in 0..3000 {
        if visits(&d, 64) >= d.cfg.radio.scan.min_rounds as usize {
            break;
        }
        d.tick(false);
    }
    d.session_opened_on(64, 64);
    d.link_up();
    d.until(6000, true, "boot pick wants");
    d.until(200, true, "relocate 64 -> ");
    let target = d.core.hop_target();
    assert_eq!(d.pump_rcf().unwrap().hop_ch, target);
    d.note_video(target);
    d.tick(true);
    d.verify_window();
    d.snapshot(0);
    assert!(d.has("(relocated)"));
}

#[test]
fn sampled_record_span() {
    let mut d = Duo::new(1, Some(40), 40);
    d.link_up();
    for _ in 0..2 * (d.cfg.hop.window_ms / 10) + 2 {
        d.tick(true);
    }
    d.core.run_sampler_step();
    d.cpp.run_sampler_step();
    d.advance(10);
    d.tick_tx(true, 0);
    for _ in 0..13 {
        d.tick(true);
    }
    d.cards(|_, s| {
        s.fr.own_air_us += 70000;
        s.fr.own += 10;
    });
    d.core.run_sampler_step();
    d.cpp.run_sampler_step();
    d.advance(30);
    d.cards(|_, s| s.fr.own_air_us += 5000);
    d.tick_tx(true, 0);
    d.snapshot(0);
}

#[test]
fn disc_for_card_matches() {
    let d = Duo::new(2, None, 40);
    let frame = rc::pack_disc(
        &rc::Disc {
            vrx_nonce: 7,
            op_channel: 40,
            op_width: 40,
            seq: 1,
            ..Default::default()
        },
        &d.cfg.link.key,
    );
    for ch in [64u8, 60, 40, 144] {
        d.cards(|i, s| {
            if i == 1 {
                s.ch = ch
            }
        });
        assert_eq!(
            d.core.disc_for_card(&frame, 1),
            d.cpp.disc_for_card(&frame, 1)
        );
    }
}

impl Drop for Duo {
    fn drop(&mut self) {
        // --nocapture shows how much each scenario compared, by record kind.
        let kinds: String = ["D ", "K ", "M ", "V ", "H ", "L ", "S ", "P ", "C"]
            .iter()
            .map(|k| {
                format!(
                    " {}{}",
                    k.trim(),
                    self.tr.iter().filter(|l| l.starts_with(k)).count()
                )
            })
            .collect();
        eprintln!("trace: {} lines,{kinds}", self.tr.len());
    }
}

/// xorshift64*: a seeded script without a dependency.
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_f491_4f6c_dd1d)
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
}

/// Random scripts: link up/down, interference and clean-ups per channel,
/// video on the hop target or not, RCF pumps, in-flight steps, move edges,
/// card deaths and reopens, found-elsewhere sessions. Each seed is one run.
#[test]
fn random_scripts() {
    let set = [40u8, 64, 112, 144];
    for seed in 1..=200u64 {
        let mut r = Rng(seed.wrapping_mul(0x9e37_79b9_7f4a_7c15) | 1);
        let n = 1 + r.below(2) as usize;
        let pin = (r.below(4) == 0).then(|| set[r.below(4) as usize]);
        let start = pin.unwrap_or(set[r.below(4) as usize]);
        let max_ms = (r.below(2) == 0).then_some(3000);
        let mut d = Duo::with(n, pin, start, &set, max_ms);
        let mut session = false;
        for step in 0..1500u64 {
            match r.below(100) {
                0..=54 => {
                    if session {
                        d.vrx_video();
                    }
                    d.tick(session);
                }
                55..=59 => {
                    let ch = set[r.below(4) as usize];
                    let v = [0u32, 5, 50][r.below(3) as usize];
                    let card = r.below(n as u64) as usize;
                    d.cards(|i, s| {
                        if i == card {
                            s.cca_per_ms_on[ch as usize] = v;
                            s.cca_per_ms_on[ch as usize - 4] = v;
                        }
                    });
                }
                60..=64 => d.cards(|_, s| s.fr.foreign += 40),
                65..=66 => d.cards(|_, s| s.fr.own += 5),
                67..=69 => {
                    if !session {
                        if r.below(3) == 0 {
                            let x = set[r.below(4) as usize];
                            d.session_opened_on(x, x);
                        }
                        d.link_up();
                        session = true;
                    }
                }
                70 => session = false,
                71..=75 => {
                    if session {
                        d.pump_rcf();
                    }
                }
                76..=78 => {
                    let t = d.core.hop_target();
                    let ch = if r.below(2) == 0 && t != 0 {
                        t
                    } else {
                        d.core.op()
                    };
                    d.note_video(ch);
                }
                79..=81 => {
                    d.core.run_inflight_step();
                    d.cpp.run_inflight_step();
                    d.sync("run_inflight_step");
                }
                82 => d.move_edge(),
                83 if n == 2 => {
                    let card = r.below(2) as usize;
                    d.cards(|i, s| {
                        if i == card {
                            s.is_alive = false;
                            s.is_ready = false;
                        }
                    });
                    d.core.on_card_died(card);
                    d.cpp.on_card_died(card);
                    d.sync("on_card_died");
                }
                84 => {
                    for card in 0..n {
                        if !d.cr[card].s().is_alive {
                            d.cards(|i, s| {
                                if i == card {
                                    s.is_alive = true;
                                    s.is_ready = true;
                                }
                            });
                            d.core.on_card_reopened(card);
                            d.cpp.on_card_reopened(card);
                        }
                    }
                }
                85..=86 => d.cards(|i, s| s.retune_ok = i != n - 1 || r_ok(step)),
                87..=89 => {
                    d.core.note_au_end();
                    d.cpp.note_au_end();
                }
                90..=91 => d.snapshot(r.below(n as u64) as usize),
                _ => d.send_path(),
            }
        }
        d.snapshot(0);
    }
}

/// Deterministic per step, the same for both sides.
fn r_ok(step: u64) -> bool {
    !step.is_multiple_of(7)
}
