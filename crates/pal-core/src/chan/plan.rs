//! mabur `channel_plan.{h,cpp}`: where the link lives (GS side). Pure: the
//! caller passes the clock and the session state. `op` is where the link
//! (and every non-scouting card) sits; `want` is where it should live (the
//! pin, the start channel, later the committed pick). A linked link moves
//! only by a hop order: one card leads onto the target, the other keeps the
//! link on `op` until confirm (video on the target) or withdraw.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MoveReason {
    Commit,
    AckOverride,
    HopLead,
    HopFollow,
    HopWithdraw,
    HopOneCard,
    LinkFound,
}

impl MoveReason {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Commit => "commit",
            Self::AckOverride => "ack_override",
            Self::HopLead => "hop_lead",
            Self::HopFollow => "hop_follow",
            Self::HopWithdraw => "hop_withdraw",
            Self::HopOneCard => "hop_one_card",
            Self::LinkFound => "link_found",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MoveEvent {
    pub t_ms: f64,
    /// None = every card.
    pub card: Option<usize>,
    pub from: u8,
    pub to: u8,
    pub reason: MoveReason,
}

#[derive(Clone, Debug)]
pub struct ChannelPlanCfg {
    pub start: u8,
    pub channels: Vec<u8>,
    pub n_cards: usize,
    pub search_after_ms: i32,
}

pub fn channel_set_member(set: &[u8], ch: u8) -> bool {
    ch != 0 && set.contains(&ch)
}

#[derive(Clone, Debug)]
pub struct ChannelPlan {
    cfg: ChannelPlanCfg,
    op: u8,
    want: u8,
    in_session: bool,
    ever_linked: bool,
    have_lost_since: bool,
    lost_since_ms: f64,
    now_ms: f64,
    events: Vec<MoveEvent>,
    hopping: bool,
    hop_target: u8,
    /// None = the one-card hop (the only card goes).
    hop_lead: Option<usize>,
    hop_from: u8,
    hop_start_ms: f64,
}

impl ChannelPlan {
    pub fn new(cfg: ChannelPlanCfg) -> Self {
        let start = cfg.start;
        Self {
            cfg,
            op: start,
            want: start,
            in_session: false,
            ever_linked: false,
            have_lost_since: false,
            lost_since_ms: 0.0,
            now_ms: 0.0,
            events: Vec::new(),
            hopping: false,
            hop_target: 0,
            hop_lead: None,
            hop_from: 0,
            hop_start_ms: 0.0,
        }
    }

    pub fn member(&self, ch: u8) -> bool {
        channel_set_member(&self.cfg.channels, ch)
    }

    pub fn tick(&mut self, now_ms: f64, in_session: bool) {
        self.now_ms = now_ms;
        self.in_session = in_session;
        if in_session {
            self.ever_linked = true;
            self.have_lost_since = false;
            return;
        }
        if self.hopping {
            return;
        }
        if !self.have_lost_since {
            self.have_lost_since = true;
            self.lost_since_ms = now_ms;
        }
    }

    /// The spare card may leave `op` to scout: not in session, no hop in
    /// flight, and either never linked or lost for `search_after_ms`.
    pub fn release_scout(&self) -> bool {
        if self.in_session || self.hopping {
            return false;
        }
        if !self.ever_linked {
            return true;
        }
        self.have_lost_since && self.now_ms - self.lost_since_ms >= self.cfg.search_after_ms as f64
    }

    /// `agreed` == op or not a member: no-op. Otherwise op follows it:
    /// Commit if the drone took the proposal, AckOverride if not.
    pub fn on_ack(&mut self, now_ms: f64, agreed: u8, proposed: u8) {
        self.now_ms = now_ms;
        if agreed == self.op || !self.member(agreed) {
            return;
        }
        let from = self.op;
        self.op = agreed;
        let reason = if agreed == proposed {
            MoveReason::Commit
        } else {
            MoveReason::AckOverride
        };
        self.push(None, from, agreed, reason);
    }

    /// The GS's own pick: sets want; without a session also op.
    pub fn commit(&mut self, now_ms: f64, to: u8) {
        self.now_ms = now_ms;
        if !self.member(to) {
            return;
        }
        self.want = to;
        if self.in_session || to == self.op {
            return;
        }
        self.push(None, self.op, to, MoveReason::Commit);
        self.op = to;
    }

    pub fn set_want(&mut self, now_ms: f64, ch: u8) {
        self.now_ms = now_ms;
        if self.member(ch) {
            self.want = ch;
        }
    }

    /// The ack that opened the session was heard on `x`: the link forms there.
    pub fn link_found(&mut self, now_ms: f64, x: u8) {
        self.now_ms = now_ms;
        if self.hopping || x == self.op || !self.member(x) {
            return;
        }
        self.push(None, self.op, x, MoveReason::LinkFound);
        self.op = x;
    }

    pub fn relocate_due(&self) -> bool {
        self.in_session && !self.hopping && self.op != self.want
    }
    pub fn want(&self) -> u8 {
        self.want
    }
    pub fn op(&self) -> u8 {
        self.op
    }

    pub fn desired(&self, card: usize) -> u8 {
        if self.hopping {
            return match self.hop_lead {
                None => self.hop_target,
                Some(l) if l == card => self.hop_target,
                _ => self.op,
            };
        }
        self.op
    }

    pub fn take_events(&mut self) -> Vec<MoveEvent> {
        std::mem::take(&mut self.events)
    }

    pub fn hop_order(&mut self, now_ms: f64, target: u8, lead: Option<usize>) {
        self.now_ms = now_ms;
        if self.hopping {
            self.push(
                self.hop_lead,
                self.hop_target,
                self.op,
                MoveReason::HopWithdraw,
            );
        } else {
            self.hop_start_ms = now_ms;
        }
        self.hopping = true;
        self.hop_target = target;
        self.hop_lead = lead;
        self.hop_from = self.op;
        let reason = if lead.is_none() {
            MoveReason::HopOneCard
        } else {
            MoveReason::HopLead
        };
        self.push(lead, self.op, target, reason);
    }

    /// No-op with no hop in flight (a one-card order defers hop_order).
    pub fn hop_confirmed(&mut self, now_ms: f64) {
        self.now_ms = now_ms;
        if !self.hopping {
            return;
        }
        self.op = self.hop_target;
        self.want = self.op;
        self.hopping = false;
        if self.have_lost_since {
            self.lost_since_ms += now_ms - self.hop_start_ms;
        }
        self.push(None, self.hop_from, self.op, MoveReason::HopFollow);
    }

    pub fn hop_withdraw(&mut self, now_ms: f64) {
        self.now_ms = now_ms;
        if !self.hopping {
            return;
        }
        self.hopping = false;
        if self.have_lost_since {
            self.lost_since_ms += now_ms - self.hop_start_ms;
        }
        self.push(
            self.hop_lead,
            self.hop_target,
            self.op,
            MoveReason::HopWithdraw,
        );
    }

    /// A video body on `rx_ch` keeps the session alive only where the link
    /// lives: op, or the hop target in flight. 0 = mid-retune or replay.
    pub fn is_link_video(&self, rx_ch: u8) -> bool {
        rx_ch == 0 || rx_ch == self.op || (self.hopping && rx_ch == self.hop_target)
    }
    pub fn hopping(&self) -> bool {
        self.hopping
    }
    pub fn hop_target(&self) -> u8 {
        self.hop_target
    }
    pub fn hop_lead(&self) -> Option<usize> {
        self.hop_lead
    }

    fn push(&mut self, card: Option<usize>, from: u8, to: u8, reason: MoveReason) {
        self.events.push(MoveEvent {
            t_ms: self.now_ms,
            card,
            from,
            to,
            reason,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use MoveReason::*;

    fn c(n: usize, start: u8) -> ChannelPlanCfg {
        ChannelPlanCfg {
            start,
            channels: vec![136, 149, 161],
            n_cards: n,
            search_after_ms: 5000,
        }
    }
    fn p(n: usize) -> ChannelPlan {
        ChannelPlan::new(c(n, 136))
    }
    fn reasons(ev: &[MoveEvent]) -> Vec<MoveReason> {
        ev.iter().map(|e| e.reason).collect()
    }

    #[test]
    fn starts_on_start_channel_and_releases_the_scout_before_any_link() {
        let mut p = p(2);
        assert_eq!(p.op(), 136);
        assert!(p.desired(0) == 136 && p.desired(1) == 136);
        p.tick(0.0, false);
        assert!(p.release_scout());
        assert!(p.take_events().is_empty());
    }

    #[test]
    fn commit_moves_op_and_logs() {
        let mut p = p(2);
        p.commit(1000.0, 149);
        assert!(p.op() == 149 && p.desired(0) == 149);
        let ev = p.take_events();
        assert_eq!(ev.len(), 1);
        assert!(
            ev[0].reason == Commit && ev[0].from == 136 && ev[0].to == 149 && ev[0].card.is_none()
        );
        p.commit(1100.0, 149);
        assert!(p.take_events().is_empty());
    }

    #[test]
    fn ack_agreeing_with_op_changes_nothing() {
        let mut p = p(2);
        p.on_ack(1000.0, 136, 136);
        assert_eq!(p.op(), 136);
        assert!(p.take_events().is_empty());
    }

    #[test]
    fn ack_override_to_a_member_is_followed_non_member_ignored() {
        let mut p = p(2);
        p.on_ack(1000.0, 149, 136);
        assert_eq!(p.op(), 149);
        let ev = p.take_events();
        assert_eq!(ev.len(), 1);
        assert!(ev[0].reason == AckOverride && ev[0].from == 136 && ev[0].to == 149);
        p.on_ack(1100.0, 112, 149);
        assert_eq!(p.op(), 149);
        assert!(p.take_events().is_empty());
        assert!(!p.member(112) && p.member(161));
    }

    #[test]
    fn loss_holds_op_for_search_after_ms_then_releases_the_scout() {
        let mut p = p(2);
        p.tick(100.0, true);
        assert!(!p.release_scout());
        p.tick(1100.0, false);
        p.tick(6000.0, false);
        assert!(!p.release_scout());
        assert!(p.desired(0) == 136 && p.desired(1) == 136);
        p.tick(6100.0, false);
        assert!(p.release_scout());
        p.tick(6200.0, true);
        assert!(!p.release_scout());
        assert!(p.take_events().is_empty());
    }

    #[test]
    fn hop_window_does_not_count_toward_the_search_timer() {
        let mut p = p(2);
        p.tick(100.0, true);
        p.tick(1100.0, false);
        p.hop_order(1200.0, 149, Some(1));
        assert!(!p.release_scout());
        p.hop_withdraw(4200.0);
        p.tick(6500.0, false);
        assert!(!p.release_scout());
        p.tick(9200.0, false);
        assert!(p.release_scout());
    }

    #[test]
    fn two_card_hop_lead_then_follow_moves_op() {
        let mut p = p(2);
        p.tick(0.0, true);
        p.hop_order(1000.0, 149, Some(1));
        assert!(p.hopping() && p.desired(1) == 149 && p.desired(0) == 136);
        assert!(p.is_link_video(149) && p.is_link_video(136) && !p.is_link_video(161));
        p.hop_confirmed(1300.0);
        assert!(!p.hopping() && p.op() == 149 && p.desired(0) == 149);
        assert_eq!(reasons(&p.take_events()), vec![HopLead, HopFollow]);
    }

    #[test]
    fn one_card_hop_moves_the_only_card_and_withdraw_restores() {
        let mut p = p(1);
        p.tick(0.0, true);
        p.hop_order(1000.0, 149, None);
        assert_eq!(p.desired(0), 149);
        p.hop_withdraw(1600.0);
        assert!(p.op() == 136 && p.desired(0) == 136);
        assert_eq!(reasons(&p.take_events()), vec![HopOneCard, HopWithdraw]);
    }

    #[test]
    fn as_str_covers_every_reason() {
        let all = [
            Commit,
            AckOverride,
            HopLead,
            HopFollow,
            HopWithdraw,
            HopOneCard,
            LinkFound,
        ];
        let s: Vec<&str> = all.iter().map(|r| r.as_str()).collect();
        assert_eq!(
            s,
            [
                "commit",
                "ack_override",
                "hop_lead",
                "hop_follow",
                "hop_withdraw",
                "hop_one_card",
                "link_found"
            ]
        );
    }

    #[test]
    fn hop_order_while_hopping_emits_withdraw_for_abandoned_lead() {
        let mut p = p(2);
        p.tick(0.0, true);
        p.hop_order(1000.0, 149, Some(1));
        p.hop_order(2000.0, 165, Some(0));
        assert!(p.hopping() && p.hop_target() == 165 && p.hop_lead() == Some(0));
        let ev = p.take_events();
        assert_eq!(ev.len(), 3);
        assert!(
            ev[0].reason == HopLead
                && ev[0].card == Some(1)
                && ev[0].from == 136
                && ev[0].to == 149
        );
        assert!(
            ev[1].reason == HopWithdraw
                && ev[1].card == Some(1)
                && ev[1].from == 149
                && ev[1].to == 136
        );
        assert!(
            ev[2].reason == HopLead
                && ev[2].card == Some(0)
                && ev[2].from == 136
                && ev[2].to == 165
        );
    }

    #[test]
    fn withdraw_with_no_hop_in_flight_leaves_the_search_timer_alone() {
        let mut p = p(1);
        p.tick(0.0, true);
        p.tick(30000.0, false);
        p.hop_withdraw(30100.0);
        assert!(!p.hopping());
        assert!(p.take_events().is_empty());
        p.tick(34999.0, false);
        assert!(!p.release_scout());
        p.tick(35000.0, false);
        assert!(p.release_scout());
    }

    #[test]
    fn confirm_with_no_hop_in_flight_is_a_no_op() {
        let mut p = p(1);
        p.tick(0.0, true);
        p.tick(30000.0, false);
        p.hop_confirmed(30100.0);
        assert!(p.op() == 136 && p.desired(0) == 136);
        assert!(p.take_events().is_empty());
        p.tick(35000.0, false);
        assert!(p.release_scout());
    }

    #[test]
    fn want_starts_on_the_start_channel() {
        let mut p = ChannelPlan::new(c(2, 149));
        assert!(p.want() == 149 && p.op() == 149);
        p.tick(0.0, true);
        assert!(!p.relocate_due());
    }

    #[test]
    fn link_found_moves_op_for_every_card_and_logs() {
        let mut p = p(2);
        p.tick(0.0, false);
        p.link_found(10.0, 149);
        assert!(p.op() == 149 && p.desired(0) == 149 && p.desired(1) == 149);
        assert_eq!(p.want(), 136);
        let ev = p.take_events();
        assert_eq!(ev.len(), 1);
        assert!(
            ev[0].reason == LinkFound
                && ev[0].card.is_none()
                && ev[0].from == 136
                && ev[0].to == 149
        );
        p.link_found(20.0, 149);
        p.link_found(20.0, 112);
        assert!(p.op() == 149 && p.take_events().is_empty());
    }

    #[test]
    fn relocate_due_truth_table() {
        let mut p = p(2);
        p.tick(0.0, false);
        p.link_found(10.0, 149);
        assert!(!p.relocate_due());
        p.tick(20.0, true);
        assert!(p.relocate_due());
        p.hop_order(30.0, 136, Some(1));
        assert!(!p.relocate_due());
        p.hop_confirmed(200.0);
        assert!(p.op() == 136 && !p.relocate_due());
    }

    #[test]
    fn commit_without_session_moves_op_and_want() {
        let mut p = p(2);
        p.tick(0.0, false);
        p.commit(100.0, 149);
        assert!(p.op() == 149 && p.want() == 149);
        assert_eq!(reasons(&p.take_events()), vec![Commit]);
    }

    #[test]
    fn commit_in_session_only_sets_want() {
        let mut p = p(2);
        p.tick(0.0, true);
        p.commit(100.0, 149);
        assert!(p.op() == 136 && p.want() == 149 && p.desired(0) == 136);
        assert!(p.take_events().is_empty());
        assert!(p.relocate_due());
        p.commit(110.0, 112);
        assert_eq!(p.want(), 149);
    }

    #[test]
    fn set_want_accepts_members_only() {
        let mut p = p(2);
        p.tick(0.0, true);
        p.link_found(5.0, 161);
        p.set_want(10.0, 161);
        assert!(p.want() == 161 && !p.relocate_due());
        p.set_want(20.0, 112);
        assert_eq!(p.want(), 161);
    }

    #[test]
    fn hop_confirm_moves_want_with_op() {
        let mut p = p(2);
        p.tick(0.0, true);
        p.hop_order(100.0, 161, Some(1));
        p.hop_confirmed(300.0);
        assert!(p.op() == 161 && p.want() == 161 && !p.relocate_due());
    }
}
