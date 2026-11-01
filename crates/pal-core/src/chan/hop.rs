//! mabur `hop_controller.{h,cpp}` and `hop_blank.h`: turns a persistent
//! `HopVerdict` trigger into a hop order, confirms it off lead-card video,
//! verifies it holds, withdraws one that never confirms, backs off targets
//! that fail, and rate-limits it all. Pure; the clock is the caller's.
//! No home channel: with nothing ranked and no escape it holds.

use std::collections::{BTreeMap, VecDeque};

use super::verdict::{Verdict, VerdictOut, EV_BLOCKED};
use crate::config::HopCfg;

/// LadderController's post-transition settle-blank, shared by both hop
/// call sites.
pub const HOP_SETTLE_BLANK_MS: f64 = 150.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum HopState {
    #[default]
    Idle,
    Ordered,
    Verifying,
    Hold,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct HopTick {
    pub now_ms: f64,
    pub verdict: VerdictOut,
    pub best: Option<u8>,
    pub best_score: u32,
    /// The lead card (or the only card) received a video AU on hop_ch.
    pub video_on_target: bool,
    /// One card: RCFs sent carrying this epoch.
    pub rcf_sent_since_order: i32,
    pub cur_op: u8,
    pub n_cards: usize,
    /// The non-TX card; None = one card.
    pub lead_card: Option<usize>,
    /// Move the link to `ChannelPlan::want()`: the channel left is not
    /// fled; exempt from cooldown, counted against the hop cap.
    pub relocate: bool,
    /// With relocate: the target is the only legal place (a pin), so the
    /// confirm lands it with no verify window.
    pub no_verify: bool,
    /// The best unblocked candidate that has not failed a verify, used only
    /// with no `best` while the verdict carries `EV_BLOCKED`.
    pub escape: Option<u8>,
    pub escape_score: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum HopActionKind {
    #[default]
    None,
    Order,
    OneCardRetune,
    Confirm,
    Withdraw,
    Hold,
    /// The verify window closed clean: the caller's cue for
    /// `HopVerdict::reset()`.
    VerifyPass,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HopAction {
    pub kind: HopActionKind,
    pub target: u8,
    pub epoch: u8,
    pub restore_rung: i32,
    pub lead_card: Option<usize>,
}

impl Default for HopAction {
    fn default() -> Self {
        Self {
            kind: HopActionKind::None,
            target: 0,
            epoch: 0,
            restore_rung: -1,
            lead_card: None,
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct HopEvent {
    pub t_ms: f64,
    pub kind: &'static str,
    pub epoch: u8,
    pub target: u8,
    pub score: u32,
    pub elapsed_ms: f64,
    /// Set by the caller ("cal" on a calibration relocate), never here.
    pub reason: String,
}

/// Why a channel is backed off: the trigger left it, it failed (verify,
/// withdraw, lost session), or an order to it went undelivered (withdrawn
/// after a confirm extension).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum BackoffWhy {
    Fled,
    Failed,
    Undelivered,
}

#[derive(Clone, Copy, Debug)]
struct Backoff {
    until_ms: f64,
    k: i32,
    why: BackoffWhy,
}

#[derive(Clone, Debug)]
pub struct HopController {
    cfg: HopCfg,
    state: HopState,
    hop_ch: u8,
    epoch: u8,
    order_ms: f64,
    verify_start: f64,
    last_confirm_ms: f64,
    hops: u32,
    holds: u32,
    one_card_retuned: bool,
    confirm_extended: bool,
    no_verify: bool,
    hold_start_ms: f64,
    backoff: BTreeMap<u8, Backoff>,
    hop_times: VecDeque<f64>,
    events: Vec<HopEvent>,
}

impl HopController {
    pub fn new(cfg: HopCfg) -> Self {
        Self {
            cfg,
            state: HopState::Idle,
            hop_ch: 0,
            epoch: 0,
            order_ms: 0.0,
            verify_start: 0.0,
            last_confirm_ms: -1e18,
            hops: 0,
            holds: 0,
            one_card_retuned: false,
            confirm_extended: false,
            no_verify: false,
            hold_start_ms: 0.0,
            backoff: BTreeMap::new(),
            hop_times: VecDeque::new(),
            events: Vec::new(),
        }
    }

    pub fn tick(&mut self, k: &HopTick) -> HopAction {
        let mut out = HopAction::default();
        match self.state {
            HopState::Idle | HopState::Hold => self.idle_tick(k, &mut out),
            HopState::Ordered => self.ordered_tick(k, &mut out),
            HopState::Verifying => self.verifying_tick(k, &mut out),
        }
        out
    }

    /// The session's falling edge. An unconfirmed order is withdrawn like a
    /// confirm_ms timeout (ChannelPlan ignores link loss while hopping, so
    /// it would otherwise stay parked forever); a confirmed one only drops
    /// its stale verify.
    pub fn on_session_lost(&mut self, now_ms: f64, cur_op: u8) -> HopAction {
        let mut out = HopAction::default();
        match self.state {
            HopState::Ordered => {
                let failed = self.hop_ch;
                self.back_off(failed, now_ms, BackoffWhy::Failed);
                self.epoch = self.epoch.wrapping_add(1);
                self.hop_ch = cur_op;
                self.state = HopState::Idle;
                out.kind = HopActionKind::Withdraw;
                out.target = cur_op;
                out.epoch = self.epoch;
                self.log(now_ms, "session_lost", failed, 0, now_ms - self.order_ms);
            }
            HopState::Verifying => {
                self.state = HopState::Idle;
                self.log(
                    now_ms,
                    "session_lost",
                    self.hop_ch,
                    0,
                    now_ms - self.verify_start,
                );
            }
            _ => {}
        }
        out
    }

    fn cap_full(&mut self, now: f64) -> bool {
        self.prune_hop_times(now);
        self.hop_times.len() as i64 >= self.cfg.max_hops_per_min as i64
    }

    fn idle_tick(&mut self, k: &HopTick, out: &mut HopAction) {
        if !k.verdict.trigger {
            // The reason to hold is gone; without this a hold is terminal.
            self.leave_hold(k.now_ms, k.cur_op);
            return;
        }
        if !k.relocate && k.now_ms - self.last_confirm_ms < self.cfg.cooldown_ms as f64 {
            return;
        }
        if self.cap_full(k.now_ms) {
            self.enter_hold(k.now_ms, "hold_cap", k.cur_op, 0.0, out);
            return;
        }
        if let Some(best) = k.best {
            self.leave_hold(k.now_ms, k.cur_op);
            if !k.relocate {
                self.flee(k.cur_op, k.now_ms);
            }
            let kind = if k.relocate { "relocate" } else { "order" };
            self.order(
                best,
                k.verdict.ref_rung,
                k.lead_card,
                k.best_score,
                k.now_ms,
                kind,
                out,
            );
            self.no_verify = k.relocate && k.no_verify;
            return;
        }
        // Nothing ranked but the op is blocked: escape rather than hold on
        // a jammed channel.
        if let Some(esc) = self.escape_allowed(k) {
            self.leave_hold(k.now_ms, k.cur_op);
            self.flee(k.cur_op, k.now_ms);
            self.order(
                esc,
                k.verdict.ref_rung,
                k.lead_card,
                k.escape_score,
                k.now_ms,
                "escape",
                out,
            );
            return;
        }
        self.enter_hold(k.now_ms, "hold_exhausted", k.cur_op, 0.0, out);
    }

    fn ordered_tick(&mut self, k: &HopTick, out: &mut HopAction) {
        let elapsed = k.now_ms - self.order_ms;
        if k.video_on_target {
            self.state = HopState::Verifying;
            self.verify_start = k.now_ms;
            *out = self.action(HopActionKind::Confirm);
            self.log(k.now_ms, "lead_confirm", self.hop_ch, 0, elapsed);
            return;
        }
        if k.n_cards == 1
            && !self.one_card_retuned
            && k.rcf_sent_since_order >= self.cfg.one_card_repeats
        {
            self.one_card_retuned = true;
            *out = self.action(HopActionKind::OneCardRetune);
            self.log(k.now_ms, "one_card_retune", self.hop_ch, 0, elapsed);
            return;
        }
        if elapsed >= self.cfg.confirm_ms as f64 {
            // The confirm extension: a blocked op at confirm_ms probably
            // jams the uplink too, so the order keeps riding RCFs until
            // confirm_extend_ms; once engaged it holds whatever later
            // windows say.
            let engage = self.confirm_extended || k.verdict.evidence & EV_BLOCKED != 0;
            if self.cfg.confirm_extend_ms > self.cfg.confirm_ms
                && engage
                && elapsed < self.cfg.confirm_extend_ms as f64
            {
                if !self.confirm_extended {
                    self.confirm_extended = true;
                    self.log(k.now_ms, "confirm_extend", self.hop_ch, 0, elapsed);
                }
                return;
            }
            self.withdraw(k.cur_op, k.now_ms, self.confirm_extended, out);
        }
    }

    fn verify_pass(&mut self, now: f64, out: &mut HopAction) {
        let landed = self.hop_ch;
        self.backoff.remove(&landed);
        self.state = HopState::Idle;
        self.last_confirm_ms = now;
        self.hops += 1;
        *out = self.action(HopActionKind::VerifyPass);
        self.log(now, "verify_pass", landed, 0, now - self.verify_start);
    }

    fn verifying_tick(&mut self, k: &HopTick, out: &mut HopAction) {
        if self.no_verify {
            self.verify_pass(k.now_ms, out);
            return;
        }
        // A window that started before confirm + settle describes the
        // channel left (or the landing's own repair debris).
        let after_landing = k.verdict.t_start_ms >= self.verify_start + HOP_SETTLE_BLANK_MS;
        if k.verdict.v == Verdict::Interfered && after_landing {
            let failed = self.hop_ch;
            let since = k.now_ms - self.verify_start;
            self.back_off(failed, k.now_ms, BackoffWhy::Failed);
            let next = k.best.filter(|&b| !self.is_backed_off(b, k.now_ms));
            // Retries skip the persist delay and cooldown, not the hop cap.
            if let Some(next) = next {
                if self.cap_full(k.now_ms) {
                    self.enter_hold(k.now_ms, "hold_cap", failed, since, out);
                } else {
                    self.order(
                        next,
                        k.verdict.ref_rung,
                        k.lead_card,
                        k.best_score,
                        k.now_ms,
                        "verify_fail",
                        out,
                    );
                }
                return;
            }
            if let Some(esc) = self.escape_allowed(k).filter(|&e| e != failed) {
                if self.cap_full(k.now_ms) {
                    self.enter_hold(k.now_ms, "hold_cap", failed, since, out);
                } else {
                    self.order(
                        esc,
                        k.verdict.ref_rung,
                        k.lead_card,
                        k.escape_score,
                        k.now_ms,
                        "escape",
                        out,
                    );
                }
                return;
            }
            self.enter_hold(k.now_ms, "verify_fail", failed, since, out);
            return;
        }
        if k.now_ms - self.verify_start >= self.cfg.verify_ms as f64 {
            self.verify_pass(k.now_ms, out);
        }
    }

    fn action(&self, kind: HopActionKind) -> HopAction {
        HopAction {
            kind,
            target: self.hop_ch,
            epoch: self.epoch,
            ..Default::default()
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn order(
        &mut self,
        target: u8,
        restore_rung: i32,
        lead_card: Option<usize>,
        score: u32,
        now: f64,
        kind: &'static str,
        out: &mut HopAction,
    ) {
        self.epoch = self.epoch.wrapping_add(1);
        self.hop_ch = target;
        self.state = HopState::Ordered;
        self.order_ms = now;
        self.one_card_retuned = false;
        self.confirm_extended = false;
        self.no_verify = false;
        self.hop_times.push_back(now);
        *out = HopAction {
            restore_rung,
            lead_card,
            ..self.action(HopActionKind::Order)
        };
        self.log(now, kind, target, score, 0.0);
    }

    fn withdraw(&mut self, restore_to: u8, now: f64, extended: bool, out: &mut HopAction) {
        let failed = self.hop_ch;
        let why = if extended {
            BackoffWhy::Undelivered
        } else {
            BackoffWhy::Failed
        };
        self.back_off(failed, now, why);
        self.epoch = self.epoch.wrapping_add(1);
        self.hop_ch = restore_to;
        self.state = HopState::Idle;
        *out = self.action(HopActionKind::Withdraw);
        let kind = if extended {
            "withdraw_undelivered"
        } else {
            "withdraw"
        };
        self.log(now, kind, failed, 0, now - self.order_ms);
    }

    /// A hold is a state: only the transition into it logs and counts.
    fn enter_hold(
        &mut self,
        now: f64,
        why: &'static str,
        target: u8,
        elapsed_ms: f64,
        out: &mut HopAction,
    ) {
        out.kind = HopActionKind::Hold;
        if self.state == HopState::Hold {
            return;
        }
        self.state = HopState::Hold;
        self.hold_start_ms = now;
        self.holds += 1;
        self.log(now, why, target, 0, elapsed_ms);
    }

    fn leave_hold(&mut self, now: f64, cur_op: u8) {
        if self.state != HopState::Hold {
            return;
        }
        self.state = HopState::Idle;
        self.log(now, "hold_end", cur_op, 0, now - self.hold_start_ms);
    }

    /// The channel a trigger flees is as bad as a failed target: without
    /// the back-off a verify-fail retry went straight back into the jam.
    fn flee(&mut self, ch: u8, now: f64) {
        self.back_off(ch, now, BackoffWhy::Fled);
    }

    /// The escape target when one is offered, the op reads blocked and the
    /// target has not failed a verify.
    fn escape_allowed(&self, k: &HopTick) -> Option<u8> {
        let esc = k.escape?;
        if k.verdict.evidence & EV_BLOCKED == 0 {
            return None;
        }
        let failed = self
            .backoff
            .get(&esc)
            .is_some_and(|b| b.until_ms > k.now_ms && b.why == BackoffWhy::Failed);
        (!failed).then_some(esc)
    }

    /// Doubles per repeat, capped at 5 min; a later back-off overwrites the
    /// reason.
    fn back_off(&mut self, ch: u8, now: f64, why: BackoffWhy) {
        let k = self.backoff.get(&ch).map_or(1, |b| b.k + 1);
        let dur = (self.cfg.backoff_ms as f64 * 2f64.powi(k - 1)).min(300000.0);
        self.backoff.insert(
            ch,
            Backoff {
                until_ms: now + dur,
                k,
                why,
            },
        );
    }

    fn is_backed_off(&self, ch: u8, now: f64) -> bool {
        self.backoff.get(&ch).is_some_and(|b| b.until_ms > now)
    }

    /// The trailing 60 s of orders already fill max_hops_per_min.
    pub fn hop_cap_reached(&self, now_ms: f64) -> bool {
        self.hop_times
            .iter()
            .filter(|&&t| now_ms - t <= 60000.0)
            .count() as i64
            >= self.cfg.max_hops_per_min as i64
    }

    fn prune_hop_times(&mut self, now: f64) {
        while self.hop_times.front().is_some_and(|&t| now - t > 60000.0) {
            self.hop_times.pop_front();
        }
    }

    fn log(&mut self, now: f64, kind: &'static str, target: u8, score: u32, elapsed_ms: f64) {
        self.events.push(HopEvent {
            t_ms: now,
            kind,
            epoch: self.epoch,
            target,
            score,
            elapsed_ms,
            reason: String::new(),
        });
    }

    /// What every RCF carries: the standing target, 0 until the first order.
    pub fn hop_ch(&self) -> u8 {
        self.hop_ch
    }
    pub fn epoch(&self) -> u8 {
        self.epoch
    }
    pub fn state(&self) -> HopState {
        self.state
    }
    pub fn backed_off(&self, now_ms: f64) -> Vec<u8> {
        self.backoff
            .iter()
            .filter(|(_, b)| b.until_ms > now_ms)
            .map(|(&c, _)| c)
            .collect()
    }
    /// Only the channels backed off for failing: the escape's skip list.
    pub fn backed_off_failed(&self, now_ms: f64) -> Vec<u8> {
        self.backoff
            .iter()
            .filter(|(_, b)| b.until_ms > now_ms && b.why == BackoffWhy::Failed)
            .map(|(&c, _)| c)
            .collect()
    }
    pub fn take_events(&mut self) -> Vec<HopEvent> {
        std::mem::take(&mut self.events)
    }
    pub fn hops(&self) -> u32 {
        self.hops
    }
    /// Hold episodes entered, not ticks spent holding.
    pub fn holds(&self) -> u32 {
        self.holds
    }
}

/// How far ahead to blank the ladder's rung store: from the first
/// interfered window of a frozen episode (once per episode, which bounds
/// it) for confirm_ms plus the settle. Never in pinned mode (`reactive`
/// false), never for fades or unknown windows.
pub fn hop_store_blank_until(vo: &VerdictOut, reactive: bool, confirm_ms: i32) -> Option<f64> {
    (reactive && vo.first_interfered).then_some(vo.t_ms + confirm_ms as f64 + HOP_SETTLE_BLANK_MS)
}

/// When the verdict's own loss window must be blanked: at Confirm plus the
/// settle, so the verify judges loss measured on the channel it verifies.
pub fn hop_verdict_loss_blank_until(act: &HopAction, now_ms: f64) -> Option<f64> {
    (act.kind == HopActionKind::Confirm).then_some(now_ms + HOP_SETTLE_BLANK_MS)
}

#[cfg(test)]
mod tests {
    use super::super::plan::{ChannelPlan, ChannelPlanCfg};
    use super::super::verdict::{HopVerdict, VerdictCardIn, VerdictLinkIn, EV_IMPAIRED, EV_RAISED};
    use super::HopActionKind as K;
    use super::*;
    use crate::config::BusyCfg;
    use crate::link_health::S1LossWindow;

    fn cfg() -> HopCfg {
        HopCfg::default()
    }
    fn interfered() -> VerdictOut {
        VerdictOut {
            v: Verdict::Interfered,
            trigger: true,
            ref_rung: 5,
            ..Default::default()
        }
    }
    fn healthy() -> VerdictOut {
        VerdictOut::default()
    }
    fn blocked_here() -> VerdictOut {
        VerdictOut {
            evidence: EV_IMPAIRED | EV_BLOCKED,
            ..interfered()
        }
    }
    fn raised_here() -> VerdictOut {
        VerdictOut {
            evidence: EV_IMPAIRED | EV_RAISED,
            ..interfered()
        }
    }
    /// A verdict with no span is treated as fresh: measured over the window
    /// ending at this tick.
    fn tk(
        t: f64,
        mut v: VerdictOut,
        best: Option<u8>,
        cur: u8,
        video: bool,
        lead: Option<usize>,
    ) -> HopTick {
        if v.t_ms == 0.0 {
            v.t_start_ms = t;
            v.t_ms = t;
        }
        HopTick {
            now_ms: t,
            verdict: v,
            best,
            cur_op: cur,
            video_on_target: video,
            lead_card: lead,
            n_cards: 2,
            ..Default::default()
        }
    }
    fn t(t: f64, v: VerdictOut, best: Option<u8>, cur: u8) -> HopTick {
        tk(t, v, best, cur, false, Some(1))
    }
    fn tv(t: f64, v: VerdictOut, best: Option<u8>, cur: u8) -> HopTick {
        tk(t, v, best, cur, true, Some(1))
    }
    fn measured(v: VerdictOut, s: f64, e: f64) -> VerdictOut {
        VerdictOut {
            t_start_ms: s,
            t_ms: e,
            ..v
        }
    }
    fn backed(h: &HopController, ch: u8, now: f64) -> bool {
        h.backed_off(now).contains(&ch)
    }
    fn kinds(ev: &[HopEvent]) -> Vec<&'static str> {
        ev.iter().map(|e| e.kind).collect()
    }
    fn hc() -> HopController {
        HopController::new(cfg())
    }

    #[test]
    fn trigger_orders_best_and_video_confirms_then_verify_passes() {
        let mut h = hc();
        let a = h.tick(&t(1000.0, interfered(), Some(149), 136));
        assert!(
            a.kind == K::Order
                && a.target == 149
                && a.epoch == 1
                && a.restore_rung == 5
                && a.lead_card == Some(1)
        );
        assert!(h.hop_ch() == 149 && h.state() == HopState::Ordered);
        let a = h.tick(&tv(1080.0, interfered(), Some(149), 136));
        assert!(a.kind == K::Confirm && h.state() == HopState::Verifying);
        let mut x = 1100.0;
        while x < 2100.0 {
            assert_eq!(h.tick(&t(x, healthy(), Some(120), 149)).kind, K::None);
            x += 150.0;
        }
        assert_eq!(
            h.tick(&t(2150.0, healthy(), Some(120), 149)).kind,
            K::VerifyPass
        );
        assert_eq!(h.tick(&t(2300.0, healthy(), Some(120), 149)).kind, K::None);
        assert!(h.state() == HopState::Idle && h.hops() == 1);
        let ev = kinds(&h.take_events());
        assert!(
            ev.len() >= 3
                && ev[0] == "order"
                && ev[1] == "lead_confirm"
                && *ev.last().unwrap() == "verify_pass"
        );
    }

    #[test]
    fn no_video_withdraws_and_backs_off_target() {
        let mut h = hc();
        h.tick(&t(1000.0, interfered(), Some(149), 136));
        let a = h.tick(&t(1600.0, interfered(), Some(149), 136));
        assert!(a.kind == K::Withdraw && a.target == 136 && a.epoch == 2);
        assert!(h.hop_ch() == 136 && h.state() == HopState::Idle);
        assert_eq!(h.backed_off(1601.0), vec![136, 149]);
        assert!(h.backed_off(1600.0 + 30000.0 + 1.0).is_empty());
    }

    #[test]
    fn verify_fail_hops_again_immediately_to_next_best() {
        let mut h = hc();
        h.tick(&t(1000.0, interfered(), Some(149), 136));
        h.tick(&tv(1080.0, interfered(), Some(149), 136));
        let a = h.tick(&t(1300.0, interfered(), Some(165), 149));
        assert!(a.kind == K::Order && a.target == 165 && a.epoch == 2 && a.restore_rung == 5);
        assert_eq!(h.backed_off(1301.0), vec![136, 149]);
    }

    #[test]
    fn exhausted_holds_without_a_home_fallback() {
        let mut h = hc();
        let a = h.tick(&t(1000.0, interfered(), None, 149));
        assert!(a.kind == K::Hold && h.state() == HopState::Hold && h.holds() == 1);
        assert_eq!(h.hop_ch(), 0);
    }

    #[test]
    fn cooldown_and_per_minute_cap() {
        let mut h = hc();
        let mut x = 0.0;
        for _ in 0..4 {
            x += 3000.0;
            assert_eq!(h.tick(&t(x, interfered(), Some(149), 136)).kind, K::Order);
            x += 50.0;
            h.tick(&tv(x, interfered(), Some(149), 136));
            x += 1100.0;
            h.tick(&t(x, healthy(), Some(120), 149));
            x += 100.0;
            h.tick(&t(x, healthy(), Some(120), 149));
        }
        x += 500.0;
        assert_eq!(h.tick(&t(x, interfered(), Some(149), 136)).kind, K::None);
        x += 2500.0;
        assert_eq!(h.tick(&t(x, interfered(), Some(149), 136)).kind, K::Hold);
    }

    #[test]
    fn one_card_retunes_after_repeats() {
        let mut h = hc();
        let mut k = tk(1000.0, interfered(), Some(149), 136, false, None);
        k.n_cards = 1;
        assert_eq!(h.tick(&k).kind, K::Order);
        k.now_ms = 1200.0;
        k.rcf_sent_since_order = 4;
        assert_eq!(h.tick(&k).kind, K::None);
        k.now_ms = 1250.0;
        k.rcf_sent_since_order = 5;
        let a = h.tick(&k);
        assert!(a.kind == K::OneCardRetune && a.target == 149);
        k.now_ms = 1300.0;
        assert_eq!(h.tick(&k).kind, K::None);
        k.now_ms = 1350.0;
        k.video_on_target = true;
        assert_eq!(h.tick(&k).kind, K::Confirm);
    }

    #[test]
    fn ordered_state_never_reorders_without_confirm_or_withdraw() {
        let mut h = hc();
        assert_eq!(
            h.tick(&t(1000.0, interfered(), Some(149), 136)).kind,
            K::Order
        );
        assert_eq!(
            h.tick(&t(1100.0, interfered(), Some(149), 136)).kind,
            K::None
        );
        assert_eq!(h.state(), HopState::Ordered);
    }

    /// Four orders inside a minute, each landing and failing its verify
    /// past the settle; returns the time of the last confirm.
    fn four_retries(h: &mut HopController) -> f64 {
        const PAST: f64 = 160.0;
        let mut x = 1000.0;
        assert_eq!(h.tick(&t(x, interfered(), Some(149), 136)).kind, K::Order);
        x += 50.0;
        assert_eq!(
            h.tick(&tv(x, interfered(), Some(149), 136)).kind,
            K::Confirm
        );
        for next in [165, 40, 44] {
            x += PAST;
            assert_eq!(h.tick(&t(x, interfered(), Some(next), 149)).kind, K::Order);
            x += 10.0;
            assert_eq!(
                h.tick(&tv(x, interfered(), Some(next), 149)).kind,
                K::Confirm
            );
        }
        x + PAST
    }

    #[test]
    fn verify_fail_retries_count_against_rate_cap() {
        let mut h = hc();
        let x = four_retries(&mut h);
        assert_eq!(h.tick(&t(x, interfered(), Some(48), 149)).kind, K::Hold);
        assert!(h.state() == HopState::Hold && h.holds() == 1);
    }

    #[test]
    fn stale_pre_hop_interfered_does_not_break_the_verify_window() {
        let mut h = hc();
        let pre = measured(interfered(), 850.0, 1000.0);
        assert_eq!(h.tick(&t(1000.0, pre, Some(149), 136)).kind, K::Order);
        assert_eq!(h.tick(&tv(1080.0, pre, Some(149), 136)).kind, K::Confirm);
        let mut x = 1090.0;
        while x < 1230.0 {
            assert_eq!(h.tick(&t(x, pre, Some(165), 149)).kind, K::None);
            assert_eq!(h.state(), HopState::Verifying);
            x += 10.0;
        }
        assert!(!backed(&h, 149, 1230.0));
        let a = h.tick(&t(
            1400.0,
            measured(interfered(), 1250.0, 1400.0),
            Some(165),
            149,
        ));
        assert!(a.kind == K::Order && a.target == 165);
        assert!(backed(&h, 149, 1401.0));
    }

    #[test]
    fn verify_window_rejects_a_window_that_straddles_the_confirm() {
        let mut h = hc();
        h.tick(&t(
            1000.0,
            measured(interfered(), 850.0, 1000.0),
            Some(149),
            136,
        ));
        h.tick(&tv(
            1080.0,
            measured(interfered(), 850.0, 1000.0),
            Some(149),
            136,
        ));
        let a = h.tick(&t(
            1160.0,
            measured(interfered(), 1000.0, 1150.0),
            Some(165),
            149,
        ));
        assert_eq!(a.kind, K::None);
        assert_eq!(h.state(), HopState::Verifying);
        assert!(!backed(&h, 149, 1161.0));
    }

    #[test]
    fn hold_is_one_episode_not_one_event_per_tick() {
        let mut h = hc();
        let mut x = 1000.0;
        while x < 2000.0 {
            assert_eq!(h.tick(&t(x, interfered(), None, 136)).kind, K::Hold);
            x += 10.0;
        }
        assert!(h.state() == HopState::Hold && h.holds() == 1);
        assert_eq!(kinds(&h.take_events()), vec!["hold_exhausted"]);
    }

    #[test]
    fn hold_ends_when_the_trigger_clears_and_says_how_long() {
        let mut h = hc();
        h.tick(&t(1000.0, interfered(), None, 136));
        h.take_events();
        assert_eq!(h.tick(&t(1500.0, healthy(), None, 136)).kind, K::None);
        assert_eq!(h.state(), HopState::Idle);
        let ev = h.take_events();
        assert!(ev.len() == 1 && ev[0].kind == "hold_end" && ev[0].elapsed_ms == 500.0);
        h.tick(&t(1600.0, healthy(), None, 136));
        assert!(h.take_events().is_empty());
    }

    #[test]
    fn re_entering_a_hold_after_it_ended_counts_a_second_episode() {
        let mut h = hc();
        h.tick(&t(1000.0, interfered(), None, 136));
        h.tick(&t(1500.0, healthy(), None, 136));
        h.tick(&t(2000.0, interfered(), None, 136));
        assert!(h.state() == HopState::Hold && h.holds() == 2);
    }

    #[test]
    fn session_loss_while_ordered_withdraws_the_hop() {
        let mut h = hc();
        assert_eq!(
            h.tick(&t(1000.0, interfered(), Some(40), 128)).kind,
            K::Order
        );
        let a = h.on_session_lost(1050.0, 128);
        assert!(a.kind == K::Withdraw && a.target == 128 && a.epoch == 2);
        assert!(h.state() == HopState::Idle && h.hop_ch() == 128);
        assert!(backed(&h, 40, 1051.0));
        let ev = h.take_events();
        let last = ev.last().unwrap();
        assert!(last.kind == "session_lost" && last.target == 40);
    }

    #[test]
    fn session_loss_when_idle_does_nothing() {
        let mut h = hc();
        assert_eq!(h.on_session_lost(1000.0, 136).kind, K::None);
        assert!(h.state() == HopState::Idle && h.take_events().is_empty());
    }

    #[test]
    fn session_loss_while_verifying_ends_the_verify() {
        let mut h = hc();
        h.tick(&t(1000.0, interfered(), Some(149), 136));
        assert_eq!(
            h.tick(&tv(1080.0, interfered(), Some(149), 136)).kind,
            K::Confirm
        );
        assert_eq!(h.on_session_lost(1200.0, 149).kind, K::None);
        assert_eq!(h.state(), HopState::Idle);
    }

    #[test]
    fn session_loss_mid_hop_withdraws_and_releases_the_plan() {
        let mut plan = ChannelPlan::new(ChannelPlanCfg {
            start: 136,
            channels: vec![136, 149, 161],
            n_cards: 2,
            search_after_ms: 5000,
        });
        plan.on_ack(0.0, 128, 128);
        plan.tick(100.0, true);
        let mut h = hc();
        let a = h.tick(&tk(1000.0, interfered(), Some(40), 128, false, Some(0)));
        assert_eq!(a.kind, K::Order);
        plan.hop_order(1000.0, a.target, a.lead_card);
        let a = h.on_session_lost(1100.0, plan.op());
        assert_eq!(a.kind, K::Withdraw);
        plan.hop_withdraw(1100.0);
        assert!(!plan.hopping() && plan.op() == 136);
        plan.tick(1100.0, false);
        plan.tick(6100.0, false);
        assert!(plan.release_scout());
    }

    #[test]
    fn trigger_backs_off_the_channel_it_flees() {
        let mut h = hc();
        assert_eq!(
            h.tick(&t(1000.0, interfered(), Some(128), 144)).kind,
            K::Order
        );
        assert!(backed(&h, 144, 1001.0));
    }

    #[test]
    fn verify_fail_never_retries_the_channel_it_fled() {
        let mut h = hc();
        h.tick(&t(1000.0, interfered(), Some(128), 144));
        h.tick(&tv(1062.0, interfered(), Some(128), 144));
        assert_eq!(
            h.tick(&t(1300.0, interfered(), Some(144), 128)).kind,
            K::Hold
        );
    }

    #[test]
    fn verify_ignores_a_window_that_starts_inside_the_landing_settle() {
        let mut h = hc();
        h.tick(&t(
            1000.0,
            measured(interfered(), 850.0, 1000.0),
            Some(149),
            136,
        ));
        h.tick(&tv(
            1080.0,
            measured(interfered(), 850.0, 1000.0),
            Some(149),
            136,
        ));
        let a = h.tick(&t(
            1260.0,
            measured(interfered(), 1107.0, 1257.0),
            Some(165),
            149,
        ));
        assert_eq!(a.kind, K::None);
        assert_eq!(h.state(), HopState::Verifying);
        let a = h.tick(&t(
            1410.0,
            measured(interfered(), 1257.0, 1407.0),
            Some(165),
            149,
        ));
        assert!(a.kind == K::Order && a.target == 165);
    }

    #[test]
    fn escape_from_blocked_op_into_fled_channel() {
        let mut h = hc();
        let mut k = t(1000.0, blocked_here(), None, 136);
        k.escape = Some(112);
        k.escape_score = 7;
        let a = h.tick(&k);
        assert!(a.kind == K::Order && a.target == 112 && a.restore_rung == 5);
        assert_eq!(h.state(), HopState::Ordered);
        let ev = h.take_events();
        assert!(ev.len() == 1 && ev[0].kind == "escape" && ev[0].target == 112 && ev[0].score == 7);
        assert!(backed(&h, 136, 1001.0));
        assert!(!h.backed_off_failed(1001.0).contains(&136));
        assert_eq!(
            h.tick(&tv(1080.0, blocked_here(), None, 136)).kind,
            K::Confirm
        );
        assert_eq!(h.state(), HopState::Verifying);
    }

    #[test]
    fn no_escape_when_op_not_blocked() {
        let mut h = hc();
        let mut k = t(1000.0, raised_here(), None, 136);
        k.escape = Some(112);
        assert_eq!(h.tick(&k).kind, K::Hold);
        assert_eq!(kinds(&h.take_events()), vec!["hold_exhausted"]);
    }

    #[test]
    fn backed_off_failed_lists_only_verify_failed_channels() {
        let mut h = hc();
        h.tick(&t(1000.0, interfered(), Some(128), 144));
        h.tick(&tv(1062.0, interfered(), Some(128), 144));
        let a = h.tick(&t(1300.0, interfered(), Some(40), 128));
        assert!(a.kind == K::Order && a.target == 40);
        assert_eq!(h.backed_off_failed(1301.0), vec![128]);
        assert_eq!(h.backed_off(1301.0), vec![128, 144]);
        h.tick(&t(1900.0, interfered(), Some(40), 128));
        assert!(h.backed_off_failed(1901.0).contains(&40));
        assert!(!h.backed_off_failed(1901.0).contains(&144));
        assert!(h.backed_off_failed(1300.0 + 60000.0 + 1000.0).is_empty());
    }

    #[test]
    fn back_off_reason_is_overwritten_by_a_later_failure() {
        let mut g = hc();
        g.tick(&t(1000.0, interfered(), Some(128), 144));
        g.tick(&tv(1062.0, interfered(), Some(128), 144));
        g.tick(&t(2200.0, healthy(), None, 128));
        assert!(!g.backed_off_failed(2201.0).contains(&144));
        g.tick(&t(40000.0, interfered(), Some(144), 128));
        g.tick(&tv(40062.0, interfered(), Some(144), 128));
        g.tick(&t(40300.0, interfered(), None, 144));
        assert!(g.backed_off_failed(40301.0).contains(&144));
        assert!(backed(&g, 144, 40300.0 + 59000.0));
    }

    #[test]
    fn escape_respects_hop_cap() {
        let mut h = hc();
        let x = four_retries(&mut h);
        h.take_events();
        let mut k = t(x, blocked_here(), None, 44);
        k.escape = Some(112);
        assert_eq!(h.tick(&k).kind, K::Hold);
        assert_eq!(kinds(&h.take_events()), vec!["hold_cap"]);
    }

    #[test]
    fn escape_after_verify_fail() {
        let mut h = hc();
        h.tick(&t(1000.0, interfered(), Some(149), 136));
        h.tick(&tv(1080.0, interfered(), Some(149), 136));
        h.take_events();
        let mut k = t(1400.0, measured(blocked_here(), 1250.0, 1400.0), None, 149);
        k.escape = Some(112);
        k.escape_score = 3;
        let a = h.tick(&k);
        assert!(a.kind == K::Order && a.target == 112);
        let ev = h.take_events();
        assert!(ev.len() == 1 && ev[0].kind == "escape" && ev[0].target == 112);
        assert!(h.backed_off_failed(1401.0).contains(&149));
        let mut g = hc();
        g.tick(&t(1000.0, interfered(), Some(149), 136));
        g.tick(&tv(1080.0, interfered(), Some(149), 136));
        let mut m = t(1400.0, measured(raised_here(), 1250.0, 1400.0), None, 149);
        m.escape = Some(112);
        assert_eq!(g.tick(&m).kind, K::Hold);
    }

    fn evcount(ev: &[HopEvent], k: &str) -> usize {
        ev.iter().filter(|e| e.kind == k).count()
    }

    #[test]
    fn confirm_extends_while_op_blocked() {
        let mut h = hc();
        assert_eq!(
            h.tick(&t(1000.0, blocked_here(), Some(112), 144)).kind,
            K::Order
        );
        h.take_events();
        for x in [1500.0, 2000.0, 3900.0] {
            assert_eq!(h.tick(&t(x, blocked_here(), Some(112), 144)).kind, K::None);
        }
        assert!(h.state() == HopState::Ordered && h.hop_ch() == 112);
        let ev = h.take_events();
        assert!(
            ev.len() == 1
                && ev[0].kind == "confirm_extend"
                && ev[0].target == 112
                && ev[0].elapsed_ms == 500.0
        );
    }

    #[test]
    fn extension_expires_as_undelivered() {
        let mut h = hc();
        h.tick(&t(1000.0, blocked_here(), Some(112), 144));
        h.tick(&t(1500.0, blocked_here(), Some(112), 144));
        h.take_events();
        let a = h.tick(&t(4000.0, blocked_here(), Some(112), 144));
        assert!(a.kind == K::Withdraw && a.target == 144);
        assert!(h.state() == HopState::Idle && h.hop_ch() == 144);
        let ev = h.take_events();
        assert!(
            ev.len() == 1
                && ev[0].kind == "withdraw_undelivered"
                && ev[0].target == 112
                && ev[0].elapsed_ms == 3000.0
        );
        assert!(!h.backed_off_failed(4001.0).contains(&112));
        assert!(backed(&h, 112, 4001.0));
    }

    #[test]
    fn unblocked_op_withdraws_at_confirm_ms() {
        let mut h = hc();
        h.tick(&t(1000.0, raised_here(), Some(112), 144));
        h.take_events();
        assert_eq!(
            h.tick(&t(1500.0, raised_here(), Some(112), 144)).kind,
            K::Withdraw
        );
        assert_eq!(kinds(&h.take_events()), vec!["withdraw"]);
        assert!(h.backed_off_failed(1501.0).contains(&112));
        let mut g = HopController::new(HopCfg {
            confirm_extend_ms: 0,
            ..cfg()
        });
        g.tick(&t(1000.0, blocked_here(), Some(112), 144));
        assert_eq!(
            g.tick(&t(1500.0, blocked_here(), Some(112), 144)).kind,
            K::Withdraw
        );
        assert!(g.backed_off_failed(1501.0).contains(&112));
        assert_eq!(evcount(&g.take_events(), "confirm_extend"), 0);
    }

    #[test]
    fn extension_holds_after_op_unblocks() {
        let mut h = hc();
        h.tick(&t(1000.0, blocked_here(), Some(112), 144));
        h.tick(&t(1500.0, blocked_here(), Some(112), 144));
        h.take_events();
        assert_eq!(
            h.tick(&t(1800.0, raised_here(), Some(112), 144)).kind,
            K::None
        );
        assert_eq!(h.tick(&t(2500.0, healthy(), Some(112), 144)).kind, K::None);
        assert!(h.state() == HopState::Ordered && h.hop_ch() == 112);
        assert!(h.take_events().is_empty());
        let a = h.tick(&tv(2600.0, healthy(), Some(112), 144));
        assert!(a.kind == K::Confirm && h.state() == HopState::Verifying);
        let mut g = hc();
        g.tick(&t(1000.0, blocked_here(), Some(112), 144));
        g.tick(&t(1500.0, blocked_here(), Some(112), 144));
        assert_eq!(g.tick(&t(3900.0, healthy(), Some(112), 144)).kind, K::None);
        g.take_events();
        assert_eq!(
            g.tick(&t(4000.0, healthy(), Some(112), 144)).kind,
            K::Withdraw
        );
        assert_eq!(kinds(&g.take_events()), vec!["withdraw_undelivered"]);
    }

    #[test]
    fn confirm_extend_not_above_confirm_ms_never_extends() {
        let mut h = HopController::new(HopCfg {
            confirm_extend_ms: 400,
            ..cfg()
        });
        h.tick(&t(1000.0, blocked_here(), Some(112), 144));
        assert_eq!(
            h.tick(&t(1500.0, blocked_here(), Some(112), 144)).kind,
            K::Withdraw
        );
        let ev = h.take_events();
        assert!(evcount(&ev, "confirm_extend") == 0 && evcount(&ev, "withdraw") == 1);
        assert!(h.backed_off_failed(1501.0).contains(&112));
    }

    /// Order 112 off a blocked 144, let it expire as undelivered.
    fn undelivered_112() -> HopController {
        let mut h = hc();
        h.tick(&t(1000.0, blocked_here(), Some(112), 144));
        h.tick(&t(1500.0, blocked_here(), Some(112), 144));
        assert_eq!(
            h.tick(&t(4000.0, blocked_here(), Some(112), 144)).kind,
            K::Withdraw
        );
        h
    }

    #[test]
    fn escape_may_target_undelivered_channel() {
        let mut h = undelivered_112();
        h.take_events();
        let mut k = t(6100.0, blocked_here(), None, 144);
        k.escape = Some(112);
        k.escape_score = 4;
        let a = h.tick(&k);
        assert!(a.kind == K::Order && a.target == 112);
        assert_eq!(kinds(&h.take_events()), vec!["escape"]);
    }

    #[test]
    fn confirm_during_extension_proceeds_to_verifying() {
        let mut h = hc();
        h.tick(&t(1000.0, blocked_here(), Some(112), 144));
        h.tick(&t(1500.0, blocked_here(), Some(112), 144));
        let a = h.tick(&tv(2700.0, blocked_here(), Some(112), 144));
        assert!(a.kind == K::Confirm && a.target == 112);
        assert_eq!(h.state(), HopState::Verifying);
        let ev = h.take_events();
        let last = ev.last().unwrap();
        assert!(last.kind == "lead_confirm" && last.elapsed_ms == 1700.0);
    }

    #[test]
    fn verify_pass_clears_undelivered_backoff() {
        let mut h = undelivered_112();
        let mut k = t(6100.0, blocked_here(), None, 144);
        k.escape = Some(112);
        assert_eq!(h.tick(&k).kind, K::Order);
        assert_eq!(
            h.tick(&tv(6150.0, blocked_here(), None, 144)).kind,
            K::Confirm
        );
        assert_eq!(h.tick(&t(7200.0, healthy(), None, 112)).kind, K::VerifyPass);
        assert!(!backed(&h, 112, 7201.0));
    }

    fn reloc(x: f64, best: Option<u8>, cur: u8) -> HopTick {
        HopTick {
            relocate: true,
            ..t(x, interfered(), best, cur)
        }
    }

    #[test]
    fn relocate_does_not_flee_the_current_channel() {
        let mut h = hc();
        let a = h.tick(&reloc(1000.0, Some(149), 136));
        assert!(a.kind == K::Order && a.target == 149);
        assert!(!backed(&h, 136, 1001.0));
        assert_eq!(kinds(&h.take_events()), vec!["relocate"]);
        h.tick(&tv(1080.0, interfered(), Some(149), 136));
        let mut x = 1100.0;
        while x < 2300.0 {
            h.tick(&t(x, healthy(), Some(120), 149));
            x += 150.0;
        }
        assert!(h.state() == HopState::Idle && h.hops() == 1);
    }

    #[test]
    fn relocate_verify_fail_retries_the_callers_best_then_holds() {
        let mut h = hc();
        h.tick(&reloc(1000.0, Some(149), 136));
        h.tick(&tv(1080.0, interfered(), Some(149), 136));
        let a = h.tick(&t(
            1400.0,
            measured(interfered(), 1250.0, 1400.0),
            Some(165),
            149,
        ));
        assert!(a.kind == K::Order && a.target == 165);
        assert_eq!(h.take_events().last().unwrap().kind, "verify_fail");
        h.tick(&tv(1480.0, interfered(), Some(165), 149));
        let a = h.tick(&t(
            1800.0,
            measured(interfered(), 1650.0, 1800.0),
            None,
            165,
        ));
        assert_eq!(a.kind, K::Hold);
    }

    #[test]
    fn relocate_ignores_cooldown() {
        let mut h = hc();
        let a = h.tick(&reloc(1000.0, Some(149), 136));
        assert!(a.kind == K::Order && a.target == 149 && h.hop_ch() == 149);
        assert_eq!(h.take_events().last().unwrap().kind, "relocate");
        assert_eq!(
            h.tick(&tv(1080.0, interfered(), None, 136)).kind,
            K::Confirm
        );
        assert_eq!(h.tick(&t(2300.0, healthy(), None, 149)).kind, K::VerifyPass);
        assert_eq!(h.take_events().last().unwrap().kind, "verify_pass");
        assert_eq!(h.tick(&reloc(2400.0, Some(136), 149)).kind, K::Order);
        assert_eq!(h.on_session_lost(2500.0, 149).kind, K::Withdraw);
    }

    #[test]
    fn pinned_relocation_lands_on_confirm_without_a_verify_window() {
        let mut h = hc();
        let k = HopTick {
            no_verify: true,
            ..reloc(1000.0, Some(40), 64)
        };
        assert_eq!(h.tick(&k).kind, K::Order);
        assert_eq!(h.tick(&tv(1170.0, interfered(), None, 64)).kind, K::Confirm);
        let a = h.tick(&t(1400.0, measured(interfered(), 1330.0, 1400.0), None, 40));
        assert!(a.kind == K::VerifyPass && a.target == 40);
        assert_eq!(h.state(), HopState::Idle);
        assert!(h.backed_off(1401.0).is_empty());
        assert!(h.holds() == 0 && h.hops() == 1);
        assert_eq!(
            kinds(&h.take_events()),
            vec!["relocate", "lead_confirm", "verify_pass"]
        );
        h.tick(&t(5000.0, interfered(), Some(149), 40));
        h.tick(&tv(5080.0, interfered(), None, 40));
        let a = h.tick(&t(
            5400.0,
            measured(interfered(), 5330.0, 5400.0),
            None,
            149,
        ));
        assert_eq!(a.kind, K::Hold);
    }

    #[test]
    fn relocate_counts_against_the_hop_cap() {
        let mut h = HopController::new(HopCfg {
            max_hops_per_min: 1,
            ..cfg()
        });
        assert_eq!(h.tick(&reloc(1000.0, Some(149), 136)).kind, K::Order);
        assert_eq!(
            h.tick(&t(1600.0, interfered(), None, 136)).kind,
            K::Withdraw
        );
        assert_eq!(h.tick(&reloc(1700.0, Some(149), 136)).kind, K::Hold);
    }

    // ---- hop_blank ----

    fn card(rssi: f64, snr: f64, foreign_ps: f64, fa_ps: f64) -> VerdictCardIn {
        let n = |ps: f64| (ps * 0.15 + 0.5) as u32;
        VerdictCardIn {
            valid: true,
            rssi_dbm: rssi,
            snr_db: snr,
            foreign: n(foreign_ps),
            fa: n(fa_ps),
            cca: n(fa_ps),
            ..Default::default()
        }
    }
    fn clean2() -> [VerdictCardIn; 2] {
        [card(-61.0, 30.0, 0.0, 4.0); 2]
    }
    fn link(loss: f64, rec: u32) -> VerdictLinkIn {
        VerdictLinkIn {
            pre_fec_loss: loss,
            recovered: rec,
            ..Default::default()
        }
    }
    fn hv() -> HopVerdict {
        HopVerdict::new(cfg(), BusyCfg::default(), 2)
    }
    fn warm(v: &mut HopVerdict) -> f64 {
        let mut x = 0.0;
        for _ in 0..40 {
            v.window(x, &clean2(), &link(0.0, 20), 5);
            x += 150.0;
        }
        x
    }
    fn jam(v: &mut HopVerdict, x: f64) -> VerdictOut {
        v.window(x, &[card(-55.0, 33.0, 237.0, 2.0); 2], &link(0.06, 80), 5)
    }
    fn fade(v: &mut HopVerdict, x: f64) -> VerdictOut {
        v.window(
            x,
            &[card(-90.0, 8.0, 0.0, 0.0), card(-86.0, 10.0, 0.0, 10.0)],
            &link(0.05, 90),
            5,
        )
    }
    fn calm(v: &mut HopVerdict, x: f64) -> VerdictOut {
        v.window(x, &clean2(), &link(0.0, 20), 5)
    }

    #[test]
    fn no_blank_while_the_link_is_clean() {
        let mut v = hv();
        let x = warm(&mut v);
        let o = calm(&mut v, x);
        assert_eq!(o.v, Verdict::Healthy);
        assert!(hop_store_blank_until(&o, true, 500).is_none());
    }

    #[test]
    fn blank_opens_at_the_first_interfered_window() {
        let mut v = hv();
        let x = warm(&mut v);
        let o = jam(&mut v, x);
        assert_eq!(o.v, Verdict::Interfered);
        assert_eq!(
            hop_store_blank_until(&o, true, 500),
            Some(x + 500.0 + HOP_SETTLE_BLANK_MS)
        );
    }

    #[test]
    fn a_fade_never_blanks_the_store_even_though_it_freezes_the_references() {
        let mut v = hv();
        let x = warm(&mut v);
        let o = fade(&mut v, x);
        assert!(o.v == Verdict::Fade && o.evidence & EV_IMPAIRED != 0 && o.ref_frozen);
        assert!(hop_store_blank_until(&o, true, 500).is_none());
        for i in 1..20 {
            assert!(
                hop_store_blank_until(&fade(&mut v, x + 150.0 * i as f64), true, 500).is_none()
            );
        }
    }

    #[test]
    fn an_unknown_window_never_blanks_the_store() {
        let mut v = hv();
        let x = warm(&mut v);
        let o = v.window(x, &clean2(), &link(0.08, 100), 5);
        assert!(o.v == Verdict::Unknown && o.evidence & EV_IMPAIRED != 0 && o.ref_frozen);
        assert!(hop_store_blank_until(&o, true, 500).is_none());
    }

    #[test]
    fn pinned_never_blanks() {
        let mut v = hv();
        let x = warm(&mut v);
        let o = jam(&mut v, x);
        assert_eq!(o.v, Verdict::Interfered);
        assert!(hop_store_blank_until(&o, false, 500).is_none());
    }

    #[test]
    fn a_sustained_jam_arms_the_blank_exactly_once() {
        let mut v = hv();
        let x = warm(&mut v);
        let armed = (0..40)
            .filter(|&i| {
                hop_store_blank_until(&jam(&mut v, x + 150.0 * i as f64), true, 500).is_some()
            })
            .count();
        assert_eq!(armed, 1);
    }

    #[test]
    fn interfered_healthy_alternation_inside_one_episode_does_not_rearm() {
        let mut v = hv();
        let mut x = warm(&mut v);
        let mut armed = 0;
        for _ in 0..20 {
            if hop_store_blank_until(&jam(&mut v, x), true, 500).is_some() {
                armed += 1;
            }
            x += 150.0;
            calm(&mut v, x);
            x += 150.0;
        }
        assert_eq!(armed, 1);
    }

    #[test]
    fn rearms_after_three_healthy_windows() {
        let mut v = hv();
        let x = warm(&mut v);
        assert!(hop_store_blank_until(&jam(&mut v, x), true, 500).is_some());
        for i in 1..=3 {
            calm(&mut v, x + 150.0 * i as f64);
        }
        assert!(hop_store_blank_until(&jam(&mut v, x + 600.0), true, 500).is_some());
    }

    #[test]
    fn rearms_after_reset_ends_a_verify_window() {
        let mut v = hv();
        let x = warm(&mut v);
        assert!(hop_store_blank_until(&jam(&mut v, x), true, 500).is_some());
        assert!(hop_store_blank_until(&jam(&mut v, x + 150.0), true, 500).is_none());
        v.reset();
        assert!(hop_store_blank_until(&jam(&mut v, x + 300.0), true, 500).is_some());
    }

    #[test]
    fn span_follows_the_configured_confirm_ms() {
        let mut v = hv();
        let x = warm(&mut v);
        assert_eq!(
            hop_store_blank_until(&jam(&mut v, x), true, 2000),
            Some(x + 2000.0 + HOP_SETTLE_BLANK_MS)
        );
    }

    #[test]
    fn verify_loss_window_blanks_when_the_hop_lands() {
        let confirm = HopAction {
            kind: K::Confirm,
            ..Default::default()
        };
        assert_eq!(
            hop_verdict_loss_blank_until(&confirm, 1000.0),
            Some(1000.0 + HOP_SETTLE_BLANK_MS)
        );
    }

    #[test]
    fn verify_loss_window_untouched_by_every_other_action() {
        for kind in [
            K::None,
            K::Order,
            K::OneCardRetune,
            K::Withdraw,
            K::Hold,
            K::VerifyPass,
        ] {
            let a = HopAction {
                kind,
                ..Default::default()
            };
            assert!(hop_verdict_loss_blank_until(&a, 1000.0).is_none());
        }
    }

    #[test]
    fn carried_over_loss_does_not_reach_the_new_channels_verify() {
        let mut w = S1LossWindow::default();
        let (mut exp, mut arr, mut x) = (0u64, 0u64, 0.0);
        while x < 1000.0 {
            exp += 10;
            arr += 5;
            w.add(exp, arr, x);
            x += 10.0;
        }
        let s = w.sample(x);
        assert!(s.valid && s.loss > 0.4);
        let confirm = HopAction {
            kind: K::Confirm,
            ..Default::default()
        };
        if let Some(b) = hop_verdict_loss_blank_until(&confirm, x) {
            w.blank_until(b);
        }
        while x < 1400.0 {
            exp += 10;
            arr += 10;
            w.add(exp, arr, x);
            x += 10.0;
        }
        let s = w.sample(x);
        assert!(!s.valid || s.loss < 0.001);
    }
}
