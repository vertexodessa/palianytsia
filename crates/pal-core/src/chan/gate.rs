//! mabur `hop_burst_gate.h`: the core loop's small hop gates, pure so each
//! has a test.

use super::hop::HopState;

/// The in-session hop block (verdict window, burst, controller tick) runs
/// only in SESSION: during the boot rendezvous the loss window reads 80-90 %
/// and the boot scout's dwells read as raised, i.e. interfered. A
/// calibration run deactivates it too: its sweeps silence the link on
/// purpose, and a hop would retune off the channel being measured.
pub fn hop_active(in_session: bool, cal_running: bool) -> bool {
    in_session && !cal_running
}

/// Whether a due calibration SweepCmd may go out this tick. `dwell_busy`:
/// the in-flight scout idles while a run is going, but checks that only
/// once per dwell period, so a dwell begun just before the start is still
/// off-channel for up to dwell_ms and that card would miss the first
/// row's opening cells. Holding costs at most one dwell of the 3 s ack
/// timeout; an unacked SweepCmd is repeated every 200 ms anyway.
pub fn cal_cmd_clear(radio_silent: bool, dwell_busy: bool) -> bool {
    !radio_silent && !dwell_busy
}

/// Keep the TX card while the in-flight scout has it off on a dwell, or
/// while a hop is in flight: switching onto the lead card moved the RCFs
/// carrying the order to a channel the drone was not on yet.
pub fn tx_selection_frozen(dwell_busy: bool, hopping: bool) -> bool {
    dwell_busy || hopping
}

/// Card i feeds the op verdict only when ready, not mid-dwell and on the
/// op channel (a lead card parked on the target would clear `blocked`
/// through the min-across-cards rule).
pub fn verdict_card_usable(ready: bool, mid_dwell: bool, card_ch: u8, op_ch: u8) -> bool {
    ready && !mid_dwell && card_ch == op_ch
}

/// The freshness burst: no hop in flight (Idle or Hold), a live trigger,
/// at most one per dwell period (the caller's last_burst_ms starts at
/// -1e18 so the first is not delayed).
pub fn hop_burst_due(
    state: HopState,
    trigger: bool,
    now_ms: f64,
    last_burst_ms: f64,
    dwell_period_ms: i32,
) -> bool {
    matches!(state, HopState::Idle | HopState::Hold)
        && trigger
        && now_ms - last_burst_ms >= dwell_period_ms as f64
}

/// mabur `CalMoveEdgeHold`: holds the link-pairing move edge while `hold`
/// (a hop in flight; in mabur also a calibration run) and delivers it once
/// after.
#[derive(Clone, Copy, Debug, Default)]
pub struct MoveEdgeHold {
    held: bool,
}

impl MoveEdgeHold {
    pub fn take(&mut self, edge: bool, hold: bool) -> bool {
        self.held |= edge;
        if hold {
            return false;
        }
        std::mem::take(&mut self.held)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn idle_and_hold_with_trigger_and_no_prior_burst_are_due() {
        assert!(hop_burst_due(HopState::Idle, true, 0.0, -1e18, 333));
        assert!(hop_burst_due(HopState::Hold, true, 0.0, -1e18, 333));
    }

    #[test]
    fn ordered_and_verifying_are_never_due() {
        assert!(!hop_burst_due(HopState::Ordered, true, 0.0, -1e18, 333));
        assert!(!hop_burst_due(HopState::Verifying, true, 0.0, -1e18, 333));
    }

    #[test]
    fn no_trigger_is_never_due_even_when_state_and_timing_allow() {
        assert!(!hop_burst_due(HopState::Idle, false, 100000.0, -1e18, 333));
        assert!(!hop_burst_due(HopState::Hold, false, 100000.0, -1e18, 333));
    }

    #[test]
    fn rate_limit_enforced_across_successive_calls() {
        assert!(!hop_burst_due(HopState::Hold, true, 1010.0, 1000.0, 333));
        assert!(!hop_burst_due(HopState::Hold, true, 1332.0, 1000.0, 333));
        assert!(hop_burst_due(HopState::Hold, true, 1333.0, 1000.0, 333));
        assert!(hop_burst_due(HopState::Hold, true, 5000.0, 1000.0, 333));
    }

    #[test]
    fn first_burst_not_delayed_by_initial_last_burst_ms() {
        assert!(hop_burst_due(HopState::Idle, true, 0.0, -1e18, 333));
        assert!(hop_burst_due(HopState::Idle, true, 10.0, -1e18, 333));
    }

    #[test]
    fn hop_block_is_inactive_outside_session() {
        assert!(hop_active(true, false));
        assert!(!hop_active(false, false));
        assert!(!hop_active(true, true));
        assert!(cal_cmd_clear(false, false));
        assert!(!cal_cmd_clear(true, false));
        assert!(!cal_cmd_clear(false, true));
    }

    #[test]
    fn tx_selection_is_frozen_during_a_dwell_or_an_in_flight_hop() {
        assert!(!tx_selection_frozen(false, false));
        assert!(tx_selection_frozen(true, false));
        assert!(tx_selection_frozen(false, true));
        assert!(tx_selection_frozen(true, true));
    }

    #[test]
    fn verdict_card_usable_only_on_the_op_channel() {
        assert!(verdict_card_usable(true, false, 144, 144));
        assert!(!verdict_card_usable(true, false, 112, 144));
        assert!(!verdict_card_usable(true, true, 144, 144));
        assert!(!verdict_card_usable(false, false, 144, 144));
    }

    #[test]
    fn move_edge_hold_defers_the_edge_until_the_hold_ends() {
        let mut h = MoveEdgeHold::default();
        assert!(h.take(true, false));
        assert!(!h.take(false, false));
        assert!(!h.take(true, true));
        assert!(!h.take(false, true));
        assert!(h.take(false, false));
        assert!(!h.take(false, false));
    }
}
