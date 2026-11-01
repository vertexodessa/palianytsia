//! A calibration run's view of the channel machinery (maburgs
//! `cal_picker.h`). Before its first sweep the run moves the pair onto the
//! quietest member of the channel set: one fresh measure round, then (when
//! the best member is not op) one relocate, then the wait for the confirm.
//! `CalSession` talks only to this trait so its tests run on a fake;
//! `ChannelCore` is the real one.
use crate::chan::core::ChannelCore;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// Two or more cards, channel = "auto": a round can be measured.
    Auto,
    /// The operator pinned the channel.
    Pinned,
    /// No spare card to measure with.
    OneCard,
}

pub trait CalPicker {
    fn mode(&self) -> Mode;
    fn op(&self) -> u8;
    /// One measure round over the set. Called before the run sets
    /// cal_running: a running calibration freezes every channel change,
    /// the scout included.
    fn begin_measure(&mut self, now_ms: u64);
    /// The round's best member; None while the round runs.
    fn measured_best(&mut self) -> Option<u8>;
    /// Move the pair to `ch` (the boot pick's relocate, reason cal). false:
    /// refused (not a member, already there, a hop or relocate in flight,
    /// the hop cap, the link down).
    fn order_relocate(&mut self, ch: u8) -> bool;
    /// Some(true): the drone followed; Some(false): it did not (withdrawn,
    /// session lost, never placed); None: pending, or nothing ordered.
    fn relocate_settled(&mut self) -> Option<bool>;
    /// The run is done with the pick: a pending or running round is dropped
    /// and the scout card released; a relocate not yet placed is withdrawn,
    /// one already placed is left to the hop machinery.
    fn cancel(&mut self);
}

impl CalPicker for ChannelCore {
    fn mode(&self) -> Mode {
        if self.pinned() {
            Mode::Pinned
        } else if self.cal_can_measure() {
            Mode::Auto
        } else {
            Mode::OneCard
        }
    }
    fn op(&self) -> u8 {
        ChannelCore::op(self)
    }
    fn begin_measure(&mut self, now_ms: u64) {
        self.begin_cal_measure(now_ms as f64);
    }
    fn measured_best(&mut self) -> Option<u8> {
        self.cal_measure_best()
    }
    fn order_relocate(&mut self, ch: u8) -> bool {
        self.order_cal_relocate(ch)
    }
    fn relocate_settled(&mut self) -> Option<bool> {
        self.cal_relocate_settled()
    }
    fn cancel(&mut self) {
        self.cancel_cal_measure();
    }
}

/// A GS with no channel core (a pinned config): it never measures, and op
/// is the configured channel.
#[derive(Clone, Copy, Debug)]
pub struct FixedPicker {
    pub mode: Mode,
    pub op: u8,
}

impl CalPicker for FixedPicker {
    fn mode(&self) -> Mode {
        self.mode
    }
    fn op(&self) -> u8 {
        self.op
    }
    fn begin_measure(&mut self, _now_ms: u64) {}
    fn measured_best(&mut self) -> Option<u8> {
        Some(self.op)
    }
    fn order_relocate(&mut self, _ch: u8) -> bool {
        false
    }
    fn relocate_settled(&mut self) -> Option<bool> {
        None
    }
    fn cancel(&mut self) {}
}
