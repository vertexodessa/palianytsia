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
    fn retune(&self, ch: u8) -> bool;
    fn retune_width(&self, ch: u8, _width_mhz: u8) -> bool {
        self.retune(ch)
    }
    fn read_energy(&self, with_nhm: bool) -> ScoutEnergy;
    fn read_energy_scout(&self) -> ScoutEnergy;
    fn frames(&self) -> ScoutFrames;
    fn arm_nhm_busy(&self, _period_4us: u16) -> bool {
        false
    }
    fn read_nhm_busy(&self) -> NhmBusyRead {
        NhmBusyRead::default()
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

/// Whether an NHM read is the window this scout armed: same channel, same
/// period, no dwell or retune since.
#[derive(Clone, Copy, Debug, Default)]
pub struct NhmWindowTracker {
    armed: bool,
    ch: u8,
    period: u16,
    dwell_gen: u32,
    tune_gen: u32,
}

impl NhmWindowTracker {
    pub fn armed(&mut self, ch: u8, period: u16, dwell_gen: u32, tune_gen: u32) {
        *self = Self {
            armed: true,
            ch,
            period,
            dwell_gen,
            tune_gen,
        };
    }
    pub fn invalidate(&mut self) {
        self.armed = false;
    }
    pub fn usable(
        &self,
        r: &NhmBusyRead,
        ch_now: u8,
        dwell_gen_now: u32,
        tune_gen_now: u32,
    ) -> bool {
        self.armed
            && r.valid
            && r.period == self.period
            && ch_now == self.ch
            && dwell_gen_now == self.dwell_gen
            && tune_gen_now == self.tune_gen
    }
}

/// Width resync may run once the scout thread is joined and, if there is a
/// scout, it has frozen.
pub fn width_resync_open(scout_joined: bool, has_scout: bool, scout_frozen: bool) -> bool {
    scout_joined && (!has_scout || scout_frozen)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WidthCard {
    pub ready: bool,
    pub width: u8,
    /// A fix-up already ran on this bring-up.
    pub tried: bool,
    /// The in-flight scout has it off on a dwell.
    pub busy: bool,
}

impl Default for WidthCard {
    fn default() -> Self {
        Self {
            ready: false,
            width: 20,
            tried: false,
            busy: false,
        }
    }
}

pub fn needs_width_fix(c: &WidthCard, want_mhz: u8) -> bool {
    c.ready && !c.tried && !c.busy && c.width != want_mhz
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
        assert!(!width_resync_open(false, true, false));
        assert!(!width_resync_open(false, true, true));
        assert!(!width_resync_open(true, true, false));
        assert!(width_resync_open(true, true, true));
        assert!(width_resync_open(true, false, false));
    }

    #[test]
    fn card_needs_fix_only_when_ready_idle_untried_and_off_width() {
        let c = WidthCard {
            ready: true,
            width: 20,
            tried: false,
            busy: false,
        };
        assert!(needs_width_fix(&c, 40));
        assert!(!needs_width_fix(&c, 20));
        assert!(!needs_width_fix(&WidthCard { ready: false, ..c }, 40));
        assert!(!needs_width_fix(&WidthCard { tried: true, ..c }, 40));
        assert!(!needs_width_fix(&WidthCard { busy: true, ..c }, 40));
        assert!(!needs_width_fix(&WidthCard { width: 40, ..c }, 40));
    }
}
