//! mabur `boot_pick.{h,cpp}`: the boot pick (open from start in auto mode
//! until it freezes, exactly once) and the relocation that moves the link
//! to `ChannelPlan::want()` on every link-up where it is elsewhere: one
//! relocate order once the scout owns no card, no hop is in flight and the
//! controller is Idle/Hold. VerifyPass: done; anything else accepts the
//! channel the link is on until the next link-up. Calibration is out of
//! scope, so its inputs and the cal re-arm are not ported. Pure.

use super::hop::HopActionKind;

/// What the boot pick needs from the core loop this tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BootPickIn {
    pub now_ms: f64,
    pub since_start_ms: u64,
    /// radio.scan.max_ms
    pub max_ms: i32,
    pub in_session: bool,
    /// A calibration run counts as linked.
    pub cal_running: bool,
    pub one_card: bool,
    pub scout_mature: bool,
    pub scout_op_ranked: bool,
    pub scout_prelude_done: bool,
    /// The scout holds its card.
    pub scout_owns: bool,
    pub proposal: u8,
    pub op: u8,
    pub want: u8,
    /// In session, no hop in flight, op != want.
    pub relocate_due: bool,
    pub plan_hopping: bool,
    pub hop_active: bool,
    pub hop_idle_or_hold: bool,
    /// The drone's move edge was acted on (one card: linked).
    pub link_edge: bool,
    pub scout_card_died: bool,
}

impl Default for BootPickIn {
    fn default() -> Self {
        Self {
            now_ms: 0.0,
            since_start_ms: 0,
            max_ms: 30000,
            in_session: false,
            cal_running: false,
            one_card: false,
            scout_mature: false,
            scout_op_ranked: false,
            scout_prelude_done: false,
            scout_owns: false,
            proposal: 0,
            op: 0,
            want: 0,
            relocate_due: false,
            plan_hopping: false,
            hop_active: false,
            hop_idle_or_hold: true,
            link_edge: false,
            scout_card_died: false,
        }
    }
}

/// One decision per tick:
/// - `Commit(ch)`: plan.commit(ch); the pick is closed.
/// - `AckPrelude(ch)`: one card: plan.commit(ch) when ch != 0, then
///   scout.ack_prelude(plan.op()). The pick stays open.
/// - `WantPick(ch)`: linked at maturity on another pair: plan.set_want(ch)
///   and scout.freeze(); the relocation then moves the link (the boot hop).
/// - `Relocate(ch)`: place one relocate order toward ch (= want), then
///   report the controller's answer through `note_hop_action`.
/// - `Freeze { reason, accept_op }`: the pick is closed; accept_op:
///   plan.set_want(op) first.
/// - `AcceptOp`: plan.set_want(op) (a relocation failed after the pick had
///   closed).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum BootPickOut {
    #[default]
    None,
    Commit(u8),
    AckPrelude(u8),
    WantPick(u8),
    Relocate(u8),
    Freeze {
        reason: &'static str,
        accept_op: bool,
    },
    AcceptOp,
}

#[derive(Clone, Debug)]
pub struct BootPick {
    open: bool,
    one_card_committed: bool,
    /// One card: a link edge was seen (latched).
    one_card_linked: bool,
    pick_wanted: bool,
    was_session: bool,
    /// A Relocate was handed out since this link-up.
    tried: bool,
    /// The controller took it (Order).
    relocating: bool,
    /// The controller refused it on the spot (Hold: the hop cap).
    resolve_now: bool,
    /// VerifyPass seen for the in-flight relocation.
    landed: bool,
}

impl BootPick {
    pub fn new(pick_open: bool) -> Self {
        Self {
            open: pick_open,
            one_card_committed: false,
            one_card_linked: false,
            pick_wanted: false,
            was_session: false,
            tried: false,
            relocating: false,
            resolve_now: false,
            landed: false,
        }
    }

    fn freeze(&mut self, reason: &'static str, accept_op: bool) -> BootPickOut {
        self.open = false;
        self.pick_wanted = false;
        BootPickOut::Freeze { reason, accept_op }
    }

    pub fn tick(&mut self, k: &BootPickIn) -> BootPickOut {
        if k.in_session && !self.was_session {
            self.tried = false;
        }
        self.was_session = k.in_session;
        if k.one_card && k.link_edge {
            self.one_card_linked = true;
        }
        let linked = k.in_session || k.cal_running;

        if (self.relocating || self.resolve_now) && k.hop_idle_or_hold {
            let ok = self.relocating && self.landed;
            self.relocating = false;
            self.resolve_now = false;
            self.landed = false;
            if self.open {
                return if ok {
                    self.freeze("relocated", false)
                } else {
                    self.freeze("relocate failed, staying", true)
                };
            }
            self.pick_wanted = false;
            if !ok {
                return BootPickOut::AcceptOp;
            }
        }

        if self.open {
            if k.scout_card_died {
                return self.freeze("scout card died", false);
            }
            if k.one_card && k.scout_prelude_done && !self.one_card_committed {
                self.one_card_committed = true;
                let ch = if !linked && k.proposal != k.op {
                    k.proposal
                } else {
                    0
                };
                return BootPickOut::AckPrelude(ch);
            }
            if self.one_card_linked {
                return self.freeze("one-card linked", false);
            }
            if k.since_start_ms >= k.max_ms.max(0) as u64 {
                if !self.relocating {
                    let accept = !self.pick_wanted;
                    return self.freeze("max_ms", accept);
                }
            } else if self.pick_wanted && !k.in_session && !self.relocating {
                return self.freeze("link lost before relocate", false);
            } else if !k.one_card && !self.pick_wanted && k.scout_mature {
                if !k.scout_op_ranked {
                    return self.freeze("op unmeasured", true);
                }
                if !linked {
                    self.open = false;
                    return BootPickOut::Commit(k.proposal);
                }
                if k.proposal == k.op {
                    return self.freeze("in place", true);
                }
                if k.cal_running {
                    return self.freeze("calibration running", true);
                }
                self.pick_wanted = true;
                self.tried = false;
                return BootPickOut::WantPick(k.proposal);
            }
        }

        if k.in_session
            && k.relocate_due
            && !self.tried
            && !self.relocating
            && !self.resolve_now
            && k.hop_active
            && !k.scout_owns
            && !k.plan_hopping
            && k.hop_idle_or_hold
        {
            self.tried = true;
            return BootPickOut::Relocate(k.want);
        }
        BootPickOut::None
    }

    /// Every HopAction the controller returned this tick (its own and the
    /// session-lost one); `relocate_tick`: the tick a Relocate was handed
    /// to the controller.
    pub fn note_hop_action(&mut self, kind: HopActionKind, relocate_tick: bool) {
        if relocate_tick {
            if kind == HopActionKind::Order {
                self.relocating = true;
                self.landed = false;
            } else {
                self.resolve_now = true;
            }
            return;
        }
        if !self.relocating {
            return;
        }
        match kind {
            HopActionKind::VerifyPass => self.landed = true,
            // A retry is a fresh attempt; only a VerifyPass counts.
            HopActionKind::Order
            | HopActionKind::Confirm
            | HopActionKind::Withdraw
            | HopActionKind::Hold => self.landed = false,
            HopActionKind::OneCardRetune | HopActionKind::None => {}
        }
    }

    pub fn open(&self) -> bool {
        self.open
    }
    /// A relocate order is in flight: the caller keeps HopTick best/escape
    /// empty and places no reactive order.
    pub fn relocating(&self) -> bool {
        self.relocating
    }
    /// A calibration run's relocate: the next due relocation may go out
    /// again this link-up.
    pub fn rearm_relocation(&mut self) {
        self.tried = false;
    }
    /// A relocation is due and untried this link-up: no reactive order may
    /// take the controller first.
    pub fn relocation_pending(&self, k: &BootPickIn) -> bool {
        k.in_session && k.relocate_due && !self.tried && !self.relocating
    }
    /// WantPick issued, relocation not resolved.
    pub fn pick_wanted(&self) -> bool {
        self.pick_wanted
    }
}

#[cfg(test)]
mod tests {
    use super::HopActionKind as H;
    use super::*;

    fn base() -> BootPickIn {
        BootPickIn {
            now_ms: 1000.0,
            since_start_ms: 1000,
            max_ms: 30000,
            op: 40,
            want: 40,
            proposal: 40,
            scout_owns: true,
            hop_idle_or_hold: true,
            ..Default::default()
        }
    }
    fn linked(k: BootPickIn) -> BootPickIn {
        BootPickIn {
            in_session: true,
            hop_active: true,
            ..k
        }
    }
    fn frozen(o: BootPickOut, why: &str) -> bool {
        matches!(o, BootPickOut::Freeze { reason, .. } if reason == why)
    }
    fn accepts(o: BootPickOut) -> bool {
        matches!(
            o,
            BootPickOut::Freeze {
                accept_op: true,
                ..
            }
        )
    }
    fn mature(k: BootPickIn, proposal: u8) -> BootPickIn {
        BootPickIn {
            scout_mature: true,
            scout_op_ranked: true,
            proposal,
            ..k
        }
    }

    #[test]
    fn nothing_before_maturity() {
        let mut b = BootPick::new(true);
        assert_eq!(b.tick(&base()), BootPickOut::None);
        assert_eq!(b.tick(&linked(base())), BootPickOut::None);
        assert!(b.open());
    }

    #[test]
    fn unlinked_maturity_commits_and_freezes_once() {
        let mut b = BootPick::new(true);
        let k = mature(base(), 144);
        assert_eq!(b.tick(&k), BootPickOut::Commit(144));
        assert!(!b.open());
        for _ in 0..3 {
            assert_eq!(b.tick(&k), BootPickOut::None);
        }
    }

    #[test]
    fn linked_maturity_on_op_freezes_in_place_and_accepts_op() {
        let mut b = BootPick::new(true);
        let k = BootPickIn {
            op: 64,
            ..mature(linked(base()), 64)
        };
        let o = b.tick(&k);
        assert!(frozen(o, "in place") && accepts(o));
    }

    #[test]
    fn linked_maturity_with_op_unmeasured_freezes_in_place() {
        let mut b = BootPick::new(true);
        let k = BootPickIn {
            scout_op_ranked: false,
            ..mature(linked(base()), 144)
        };
        let o = b.tick(&k);
        assert!(frozen(o, "op unmeasured") && accepts(o));
    }

    #[test]
    fn linked_maturity_on_another_pair_relocates_and_freezes_relocated() {
        let mut b = BootPick::new(true);
        let mut k = mature(linked(base()), 144);
        assert_eq!(b.tick(&k), BootPickOut::WantPick(144));
        assert!(b.open() && b.pick_wanted());
        k.want = 144;
        k.relocate_due = true;
        assert_eq!(b.tick(&k), BootPickOut::None);
        k.scout_owns = false;
        assert_eq!(b.tick(&k), BootPickOut::Relocate(144));
        b.note_hop_action(H::Order, true);
        assert!(b.relocating());
        k.hop_idle_or_hold = false;
        k.relocate_due = false;
        k.plan_hopping = true;
        assert_eq!(b.tick(&k), BootPickOut::None);
        b.note_hop_action(H::Confirm, false);
        k.plan_hopping = false;
        k.op = 144;
        assert_eq!(b.tick(&k), BootPickOut::None);
        b.note_hop_action(H::VerifyPass, false);
        k.hop_idle_or_hold = true;
        let o = b.tick(&k);
        assert!(frozen(o, "relocated") && !accepts(o));
        assert!(!b.open() && !b.relocating());
    }

    fn placed() -> (BootPick, BootPickIn) {
        let mut b = BootPick::new(true);
        let mut k = mature(linked(base()), 144);
        b.tick(&k);
        k.want = 144;
        k.relocate_due = true;
        k.scout_owns = false;
        b.tick(&k);
        b.note_hop_action(H::Order, true);
        k.hop_idle_or_hold = false;
        b.tick(&k);
        (b, k)
    }

    #[test]
    fn relocate_withdrawn_accepts_the_channel_the_link_is_on() {
        let (mut b, mut k) = placed();
        b.note_hop_action(H::Withdraw, false);
        k.hop_idle_or_hold = true;
        let o = b.tick(&k);
        assert!(frozen(o, "relocate failed, staying") && accepts(o));
    }

    #[test]
    fn relocate_verify_fail_hold_accepts_op() {
        let (mut b, mut k) = placed();
        b.note_hop_action(H::Confirm, false);
        b.note_hop_action(H::Hold, false);
        k.hop_idle_or_hold = true;
        assert!(frozen(b.tick(&k), "relocate failed, staying"));
    }

    #[test]
    fn relocate_cut_short_counts_as_failed() {
        let (mut b, mut k) = placed();
        b.note_hop_action(H::Confirm, false);
        k.hop_idle_or_hold = true;
        k.in_session = false;
        assert!(frozen(b.tick(&k), "relocate failed, staying"));
    }

    #[test]
    fn relocate_refused_by_the_hop_cap_fails_next_tick() {
        let mut b = BootPick::new(true);
        let mut k = mature(linked(base()), 144);
        b.tick(&k);
        k.want = 144;
        k.relocate_due = true;
        k.scout_owns = false;
        assert_eq!(b.tick(&k), BootPickOut::Relocate(144));
        b.note_hop_action(H::Hold, true);
        assert!(!b.relocating());
        assert!(frozen(b.tick(&k), "relocate failed, staying"));
    }

    #[test]
    fn link_lost_before_the_relocation_is_placed_keeps_the_pick() {
        let mut b = BootPick::new(true);
        let mut k = mature(linked(base()), 144);
        b.tick(&k);
        k.in_session = false;
        k.hop_active = false;
        k.want = 144;
        let o = b.tick(&k);
        assert!(frozen(o, "link lost before relocate") && !accepts(o));
    }

    #[test]
    fn max_ms_freezes_unmeasured_but_waits_for_a_placed_relocation() {
        let mut b = BootPick::new(true);
        let o = b.tick(&BootPickIn {
            since_start_ms: 30000,
            ..base()
        });
        assert!(frozen(o, "max_ms") && accepts(o));
        let (mut c, mut k) = placed();
        k.since_start_ms = 30000;
        assert_eq!(c.tick(&k), BootPickOut::None);
        c.note_hop_action(H::VerifyPass, false);
        k.hop_idle_or_hold = true;
        assert!(frozen(c.tick(&k), "relocated"));
    }

    #[test]
    fn scout_card_died_freezes() {
        let mut b = BootPick::new(true);
        let o = b.tick(&BootPickIn {
            scout_card_died: true,
            ..base()
        });
        assert!(frozen(o, "scout card died") && !accepts(o));
    }

    #[test]
    fn one_card_prelude_commits_once_only_when_unlinked() {
        let mut b = BootPick::new(true);
        let mut k = BootPickIn {
            one_card: true,
            scout_prelude_done: true,
            proposal: 112,
            ..base()
        };
        assert_eq!(b.tick(&k), BootPickOut::AckPrelude(112));
        assert_eq!(b.tick(&k), BootPickOut::None);
        assert!(b.open());
        let mut c = BootPick::new(true);
        assert_eq!(c.tick(&linked(k)), BootPickOut::AckPrelude(0));
        let mut d = BootPick::new(true);
        k.proposal = 40;
        assert_eq!(d.tick(&k), BootPickOut::AckPrelude(0));
    }

    #[test]
    fn one_card_link_edge_freezes_then_relocates() {
        let mut b = BootPick::new(true);
        let mut k = BootPickIn {
            one_card: true,
            scout_prelude_done: true,
            ..linked(base())
        };
        b.tick(&k);
        k.op = 112;
        k.want = 40;
        k.relocate_due = true;
        assert_eq!(b.tick(&k), BootPickOut::None);
        k.link_edge = true;
        assert!(frozen(b.tick(&k), "one-card linked"));
        k.link_edge = false;
        k.scout_owns = false;
        assert_eq!(b.tick(&k), BootPickOut::Relocate(40));
    }

    fn off_pin() -> BootPickIn {
        BootPickIn {
            scout_owns: false,
            op: 112,
            want: 40,
            relocate_due: true,
            ..linked(base())
        }
    }

    #[test]
    fn pinned_found_off_pin_relocates_to_the_pin() {
        let mut b = BootPick::new(false);
        let mut k = off_pin();
        assert_eq!(b.tick(&k), BootPickOut::Relocate(40));
        b.note_hop_action(H::Order, true);
        k.hop_idle_or_hold = false;
        b.tick(&k);
        b.note_hop_action(H::Withdraw, false);
        k.hop_idle_or_hold = true;
        assert_eq!(b.tick(&k), BootPickOut::AcceptOp);
        let mut c = BootPick::new(false);
        c.tick(&k);
        c.note_hop_action(H::Order, true);
        c.note_hop_action(H::VerifyPass, false);
        assert_eq!(c.tick(&k), BootPickOut::None);
    }

    #[test]
    fn relocation_is_tried_once_per_link_up() {
        let mut b = BootPick::new(false);
        let mut k = off_pin();
        assert_eq!(b.tick(&k), BootPickOut::Relocate(40));
        b.note_hop_action(H::Hold, true);
        assert_eq!(b.tick(&k), BootPickOut::AcceptOp);
        assert_eq!(b.tick(&k), BootPickOut::None);
        k.in_session = false;
        k.hop_active = false;
        b.tick(&k);
        k.in_session = true;
        k.hop_active = true;
        assert_eq!(b.tick(&k), BootPickOut::Relocate(40));
    }

    #[test]
    fn relocation_waits_for_the_cards_and_the_controller() {
        let mut k = BootPickIn {
            op: 112,
            want: 40,
            relocate_due: true,
            ..linked(base())
        };
        let mut b = BootPick::new(false);
        assert_eq!(b.tick(&k), BootPickOut::None);
        k.scout_owns = false;
        k.plan_hopping = true;
        assert_eq!(b.tick(&k), BootPickOut::None);
        k.plan_hopping = false;
        k.hop_idle_or_hold = false;
        assert_eq!(b.tick(&k), BootPickOut::None);
        k.hop_idle_or_hold = true;
        k.hop_active = false;
        assert_eq!(b.tick(&k), BootPickOut::None);
        k.hop_active = true;
        assert!(b.relocation_pending(&k));
        assert_eq!(b.tick(&k), BootPickOut::Relocate(40));
        assert!(!b.relocation_pending(&k));
    }
}
