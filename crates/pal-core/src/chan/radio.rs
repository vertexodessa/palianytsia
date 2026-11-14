//! The card as the channel search sees it: mabur `scout_radio.h`,
//! `link_card.h` (USB part), `nhm_busy.h`, `nhm_window.h`, `width_resync.h`
//! and `snr_units.h`.

/// Raw SNR/EVM units to dB.
pub const SNR_RAW_TO_DB: f64 = 0.5;
pub const EVM_RAW_TO_DB: f64 = 0.5;

/// devourer `nf::kNhmAbsThDbm`: the NHM bucket edges, dBm.
pub const NHM_ABS_TH_DBM: [i8; 11] = [-104, -101, -98, -95, -92, -89, -86, -83, -80, -75, -70];

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ScoutEnergy {
    pub fa_valid: bool,
    pub cca_ofdm: u32,
    pub fa_ofdm: u32,
    pub igi_valid: bool,
    pub igi: u8,
    pub nhm_valid: bool,
    pub floor_valid: bool,
    pub floor_dbm: i8,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ScoutFrames {
    pub own: u64,
    pub foreign: u64,
    pub own_air_us: u64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NhmBusyRead {
    pub valid: bool,
    pub buckets: [u8; 12],
    pub duration: u16,
    pub period: u16,
}

/// mabur `ClmBusyRead`: one armed busy-airtime CLM window (devourer
/// `GetChannelBusy` after `ArmChannelBusy`), the share of 4 us ticks the
/// baseband held CCA busy, receive-side only. The fallback for a card whose
/// NHM window is not ported (Jaguar1: the RTL8812AU). `own_tx` = the card
/// transmitted inside it, which reads low.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ClmBusyRead {
    pub valid: bool,
    pub pct: u8,
    pub window_us: u32,
    pub own_tx: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CardCaps {
    pub valid: bool,
    pub chip: String,
    pub gen: String,
    pub tx_chains: i32,
    pub rx_chains: i32,
    pub bw_mask: u8,
    pub tune5g_lo: u16,
    pub tune5g_hi: u16,
    pub fast_retune: bool,
    pub fa_ok: bool,
    pub igi_ok: bool,
    pub nhm_ok: bool,
    pub floor_ok: bool,
    pub snr_ok: bool,
}

impl Default for CardCaps {
    fn default() -> Self {
        Self {
            valid: false,
            chip: String::new(),
            gen: String::new(),
            tx_chains: 0,
            rx_chains: 0,
            bw_mask: 0,
            tune5g_lo: 0,
            tune5g_hi: 0,
            fast_retune: false,
            fa_ok: false,
            igi_ok: false,
            nhm_ok: false,
            floor_ok: false,
            snr_ok: true,
        }
    }
}

/// The scout's view of a card (mabur `ScoutRadio`). Implementations are
/// shared between the core and the scout, hence `&self` and interior locking.
pub trait ScoutRadio: Send + Sync {
    /// Retune to `ch` at the card's current width. The scout never changes
    /// a card's width: only the core does (a live width flip).
    fn retune(&self, ch: u8) -> bool;
    fn read_energy(&self, with_nhm: bool) -> ScoutEnergy;
    fn read_energy_scout(&self) -> ScoutEnergy;
    fn frames(&self) -> ScoutFrames;
    fn arm_nhm_busy(&self, _period_4us: u16) -> bool {
        false
    }
    fn read_nhm_busy(&self) -> NhmBusyRead {
        NhmBusyRead::default()
    }
    /// Busy-airtime CLM window, for a card that refuses `arm_nhm_busy`: the
    /// window actually armed in us (0 = unsupported). Read only after an arm
    /// that returned nonzero: an unarmed read on devourer takes a 2 ms NHM
    /// sample and drains the FA/CCA delta the dwell is measuring.
    fn arm_clm_busy(&self, _window_us: u32) -> u32 {
        0
    }
    fn read_clm_busy(&self) -> ClmBusyRead {
        ClmBusyRead::default()
    }
}

/// A USB link card (mabur `LinkCard` minus the relay calls, which
/// Palianytsia has no hardware for).
pub trait LinkCard: ScoutRadio {
    fn ready(&self) -> bool;
    fn alive(&self) -> bool;
    fn caps(&self) -> CardCaps;
    fn rx_frames(&self) -> u64;
    fn foreign(&self) -> u64;
    fn send_control(&self, body: &[u8]) -> bool;
    fn tx_frames(&self) -> u64;
    fn tx_fail(&self) -> u64;
    fn set_width(&self, ch: u8, width_mhz: u8) -> bool;
    fn channel(&self) -> u8;
    fn width(&self) -> u8;
    /// -1 = unknown.
    fn tuned_central(&self) -> i32;
    fn can_scout(&self) -> bool;
}

pub fn busy_dbm_is_edge(dbm: i32) -> bool {
    NHM_ABS_TH_DBM.iter().any(|&e| e as i32 == dbm)
}

/// The share of NHM samples at or above `busy_dbm` (a bucket edge), %. None
/// for an invalid or empty window ("ready but all zero" is no reading).
pub fn nhm_busy_pct(r: &NhmBusyRead, busy_dbm: i32) -> Option<f64> {
    if !r.valid {
        return None;
    }
    let first = NHM_ABS_TH_DBM.iter().position(|&e| e as i32 == busy_dbm)? + 1;
    let all: u32 = r.buckets.iter().map(|&b| b as u32).sum();
    let above: u32 = r.buckets[first..].iter().map(|&b| b as u32).sum();
    if all == 0 {
        return None;
    }
    Some(100.0 * above as f64 / all as f64)
}

pub fn nhm_period_4us(ms: i32) -> u16 {
    (ms as i64 * 250).clamp(1, 65535) as u16
}

/// mabur `BusySensor`: which sensor one observation's busy window was armed
/// on, the NHM share above busy_dbm where the card has it (Jaguar3), else
/// CLM busy airtime over the same span (Jaguar1). Without the fallback an
/// RTL8812AU GS read no busy at all: every pick kept the channel it started
/// on and no hop fired.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum BusySensor {
    #[default]
    None,
    Nhm,
    Clm,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BusyArm {
    pub sensor: BusySensor,
    /// The NHM period asked for.
    pub period_4us: u16,
    /// The CLM window devourer armed.
    pub clm_us: u32,
}

pub fn arm_busy<R: ScoutRadio + ?Sized>(r: &R, period_4us: u16) -> BusyArm {
    let mut a = BusyArm {
        period_4us,
        ..Default::default()
    };
    if r.arm_nhm_busy(period_4us) {
        a.sensor = BusySensor::Nhm;
    } else {
        a.clm_us = r.arm_clm_busy(u32::from(period_4us) * 4);
        if a.clm_us != 0 {
            a.sensor = BusySensor::Clm;
        }
    }
    a
}

/// A CLM read is this window's only if it spans the window armed.
pub fn clm_busy_pct(c: &ClmBusyRead, a: &BusyArm) -> Option<f64> {
    (c.valid && c.window_us == a.clm_us).then_some(f64::from(c.pct))
}

/// The busy % of the window `a` armed, read at the observation's end, and
/// whether the card sent inside a CLM window (which reads low; the flight
/// log carries it, mabur has no field for it).
pub fn read_busy_tx<R: ScoutRadio + ?Sized>(
    r: &R,
    a: &BusyArm,
    busy_dbm: i32,
) -> (Option<f64>, bool) {
    match a.sensor {
        BusySensor::Nhm => {
            let nb = r.read_nhm_busy();
            if !nb.valid || nb.period != a.period_4us {
                return (None, false);
            }
            (nhm_busy_pct(&nb, busy_dbm), false)
        }
        BusySensor::Clm => {
            let c = r.read_clm_busy();
            (clm_busy_pct(&c, a), c.own_tx)
        }
        BusySensor::None => (None, false),
    }
}

/// mabur `read_busy`.
pub fn read_busy<R: ScoutRadio + ?Sized>(r: &R, a: &BusyArm, busy_dbm: i32) -> Option<f64> {
    read_busy_tx(r, a, busy_dbm).0
}

/// Whether an NHM read is the window this scout armed: same channel, same
/// period, no dwell or retune since. A card without the NHM window arms CLM
/// instead (`armed_clm`): the same pins hold, and its read must span the
/// window devourer armed.
#[derive(Clone, Copy, Debug, Default)]
pub struct NhmWindowTracker {
    armed: bool,
    ch: u8,
    period: u16,
    clm_us: u32,
    dwell_gen: u32,
    tune_gen: u32,
}

impl NhmWindowTracker {
    pub fn armed(&mut self, ch: u8, period: u16, dwell_gen: u32, tune_gen: u32) {
        *self = Self {
            armed: true,
            ch,
            period,
            clm_us: 0,
            dwell_gen,
            tune_gen,
        };
    }
    pub fn armed_clm(&mut self, ch: u8, clm_us: u32, dwell_gen: u32, tune_gen: u32) {
        *self = Self {
            armed: true,
            ch,
            period: 0,
            clm_us,
            dwell_gen,
            tune_gen,
        };
    }
    pub fn invalidate(&mut self) {
        self.armed = false;
        self.clm_us = 0;
    }
    /// The window in flight is a CLM one: read it with `read_clm_busy`, and
    /// only then.
    pub fn clm(&self) -> bool {
        self.armed && self.clm_us != 0
    }
    fn pinned(&self, ch_now: u8, dwell_gen_now: u32, tune_gen_now: u32) -> bool {
        ch_now == self.ch && dwell_gen_now == self.dwell_gen && tune_gen_now == self.tune_gen
    }
    pub fn usable(
        &self,
        r: &NhmBusyRead,
        ch_now: u8,
        dwell_gen_now: u32,
        tune_gen_now: u32,
    ) -> bool {
        self.armed
            && self.clm_us == 0
            && r.valid
            && r.period == self.period
            && self.pinned(ch_now, dwell_gen_now, tune_gen_now)
    }
    pub fn usable_clm(
        &self,
        r: &ClmBusyRead,
        ch_now: u8,
        dwell_gen_now: u32,
        tune_gen_now: u32,
    ) -> bool {
        self.clm()
            && r.valid
            && r.window_us == self.clm_us
            && self.pinned(ch_now, dwell_gen_now, tune_gen_now)
    }
}

/// mabur `width_resync.h`: the fix-up is closed while a scan holds the
/// scout card (the boot pick open, or a calibration round). No scout at
/// all is open from the start; a scout that only searches keeps its own
/// card (`WidthCard::scouting`) and every other card is fixed.
pub fn width_resync_open(has_scout: bool, pick_open: bool, round_open: bool) -> bool {
    !has_scout || (!pick_open && !round_open)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WidthCard {
    pub ready: bool,
    pub width: u8,
    /// A fix-up already ran on this bring-up.
    pub tried: bool,
    /// The in-flight scout has it off on a dwell.
    pub busy: bool,
    /// The boot scout has it (searching): fixed once the search lets go.
    pub scouting: bool,
}

impl Default for WidthCard {
    fn default() -> Self {
        Self {
            ready: false,
            width: 20,
            tried: false,
            busy: false,
            scouting: false,
        }
    }
}

/// How long a card waits after a failed `set_width` before the next try: a
/// transient USB error must not leave it at the wrong width, and a lasting
/// one must not cost a full channel set every tick.
pub const WIDTH_RETRY_MS: f64 = 1000.0;

/// Once per bring-up and width (tried resets on reopen, when the boot scout
/// lets go of it, when the width flips, or `WIDTH_RETRY_MS` after a failed
/// `set_width`): a failed `set_width` is logged and paced.
pub fn needs_width_fix(c: &WidthCard, want_mhz: u8) -> bool {
    c.ready && !c.tried && !c.busy && !c.scouting && c.width != want_mhz
}

#[cfg(test)]
mod tests {
    use super::*;

    fn r(b: &[u8]) -> NhmBusyRead {
        let mut x = NhmBusyRead {
            valid: true,
            ..Default::default()
        };
        x.buckets[..b.len()].copy_from_slice(b);
        x
    }

    #[test]
    fn edges_are_the_absolute_table() {
        assert!(busy_dbm_is_edge(-83) && busy_dbm_is_edge(-104) && busy_dbm_is_edge(-70));
        assert!(!busy_dbm_is_edge(-82) && !busy_dbm_is_edge(-60));
    }

    #[test]
    fn busy_is_the_share_at_or_above_the_edge() {
        let x = r(&[100, 0, 0, 0, 0, 0, 0, 0, 50, 50, 25, 30]);
        assert!((nhm_busy_pct(&x, -83).unwrap() - 100.0 * 155.0 / 255.0).abs() < 1e-9);
        assert!((nhm_busy_pct(&x, -70).unwrap() - 100.0 * 30.0 / 255.0).abs() < 1e-9);
    }

    #[test]
    fn invalid_or_empty_window_is_no_reading() {
        assert!(nhm_busy_pct(&NhmBusyRead::default(), -83).is_none());
        assert!(nhm_busy_pct(&r(&[0; 12]), -83).is_none());
    }

    #[test]
    fn period_clamps_to_16_bits() {
        assert_eq!(nhm_period_4us(140), 35000);
        assert_eq!(nhm_period_4us(5), 1250);
        assert_eq!(nhm_period_4us(1990), 65535);
        assert_eq!(nhm_period_4us(0), 1);
    }

    fn rd(period: u16, valid: bool) -> NhmBusyRead {
        let mut x = NhmBusyRead {
            valid,
            period,
            ..Default::default()
        };
        x.buckets[0] = 255;
        x
    }
    fn armed() -> NhmWindowTracker {
        let mut t = NhmWindowTracker::default();
        t.armed(136, 35000, 7, 3);
        t
    }

    #[test]
    fn nhm_window_usability() {
        assert!(armed().usable(&rd(35000, true), 136, 7, 3));
        assert!(!armed().usable(&rd(1250, true), 136, 7, 3)); // someone else's arm
        assert!(!armed().usable(&rd(35000, true), 144, 7, 3)); // channel changed
        assert!(!NhmWindowTracker::default().usable(&rd(35000, true), 136, 7, 3));
        assert!(!armed().usable(&rd(35000, false), 136, 7, 3));
        let mut t = armed();
        t.invalidate();
        assert!(!t.usable(&rd(35000, true), 136, 7, 3));
        let mut c = NhmWindowTracker::default();
        c.armed(136, nhm_period_4us(1990), 7, 3);
        assert!(c.usable(&rd(65535, true), 136, 7, 3)); // clamped period still matches
        assert!(!armed().usable(&rd(35000, true), 136, 8, 3)); // a dwell since
        assert!(!armed().usable(&rd(35000, true), 136, 7, 4)); // a retune since
    }

    #[test]
    fn width_resync_gate() {
        assert!(width_resync_open(false, true, true), "no scout: open");
        assert!(!width_resync_open(true, true, false), "boot pick open");
        assert!(!width_resync_open(true, false, true), "calibration round");
        assert!(width_resync_open(true, false, false));
    }

    #[test]
    fn card_needs_fix_only_when_ready_idle_untried_and_off_width() {
        let c = WidthCard {
            ready: true,
            width: 20,
            tried: false,
            busy: false,
            scouting: false,
        };
        assert!(needs_width_fix(&c, 40));
        assert!(!needs_width_fix(&c, 20));
        assert!(!needs_width_fix(&WidthCard { ready: false, ..c }, 40));
        assert!(!needs_width_fix(&WidthCard { tried: true, ..c }, 40));
        assert!(!needs_width_fix(&WidthCard { busy: true, ..c }, 40));
        assert!(!needs_width_fix(
            &WidthCard {
                scouting: true,
                ..c
            },
            40
        ));
        assert!(!needs_width_fix(&WidthCard { width: 40, ..c }, 40));
    }
}
