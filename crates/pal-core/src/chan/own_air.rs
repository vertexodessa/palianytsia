//! Our own video's airtime, rebuilt on the GS from the frames a card
//! decoded (mabur `own_air.h`): payload at the HT-1SS PHY rate plus one
//! HT-mixed preamble per PPDU. devourer marks a PPDU's first subframe with
//! the PHY status and only that subframe carries bw/stbc, so the
//! accumulator latches them there and times the later subframes at the
//! same width.

/// L-STF/LTF/SIG + HT-SIG/STF/LTF1.
pub const HT_PREAMBLE_US: u32 = 40;
/// STBC 1SS -> 2 space-time streams: one extra HT-LTF.
pub const STBC_EXTRA_US: u32 = 4;

pub fn ht_rate_mbps(mcs: u8, bw_mhz: u8, sgi: bool) -> f64 {
    const K20: [f64; 8] = [6.5, 13.0, 19.5, 26.0, 39.0, 52.0, 58.5, 65.0];
    if mcs > 7 {
        return 0.0;
    }
    // 40 MHz: 108/52 data subcarriers.
    let mut r = K20[mcs as usize] * if bw_mhz == 40 { 2.076923076923077 } else { 1.0 };
    if sgi {
        r = r * 10.0 / 9.0;
    }
    r
}

pub fn mpdu_air_us(bytes: usize, mcs: u8, bw_mhz: u8, sgi: bool) -> f64 {
    let r = ht_rate_mbps(mcs, bw_mhz, sgi);
    if r > 0.0 {
        bytes as f64 * 8.0 / r
    } else {
        0.0
    }
}

pub fn ppdu_overhead_us(stbc: bool) -> u32 {
    HT_PREAMBLE_US + if stbc { STBC_EXTRA_US } else { 0 }
}

/// The HT MCS of an RX descriptor rate (devourer's MGN codes: 0x0C..0x13
/// and 0x80..0x87), 255 for anything else (legacy, VHT).
pub fn ht_mcs_of_rate(r: u16) -> u8 {
    match r {
        0x0C..=0x13 => (r - 0x0C) as u8,
        0x80..=0x87 => (r - 0x80) as u8,
        _ => 255,
    }
}

#[derive(Clone, Copy, Debug)]
pub struct OwnAirAcc {
    bw: u8,
    stbc: bool,
    sgi: bool,
    acc_us: f64,
}

impl Default for OwnAirAcc {
    fn default() -> Self {
        Self {
            bw: 20,
            stbc: false,
            sgi: false,
            acc_us: 0.0,
        }
    }
}

impl OwnAirAcc {
    pub fn on_frame(
        &mut self,
        bytes: usize,
        mcs: u8,
        physt: bool,
        bw_mhz: u8,
        stbc: bool,
        sgi: bool,
    ) {
        if mcs > 7 {
            return;
        }
        if physt {
            self.bw = bw_mhz;
            self.stbc = stbc;
            self.sgi = sgi;
        }
        self.acc_us += mpdu_air_us(bytes, mcs, self.bw, self.sgi)
            + if physt {
                ppdu_overhead_us(self.stbc) as f64
            } else {
                0.0
            };
    }
    pub fn total_us(&self) -> u64 {
        (self.acc_us + 0.5) as u64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn near(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-6
    }

    #[test]
    fn ht_rates_match_the_standard_table() {
        assert!(near(ht_rate_mbps(0, 20, false), 6.5));
        assert!(near(ht_rate_mbps(7, 20, false), 65.0));
        assert!(near(ht_rate_mbps(3, 40, false), 54.0));
        assert!(near(ht_rate_mbps(4, 40, false), 81.0));
        assert!(near(ht_rate_mbps(7, 20, true), 65.0 * 10.0 / 9.0));
        assert_eq!(ht_rate_mbps(8, 20, false), 0.0);
    }

    #[test]
    fn single_frames_pay_a_preamble_each() {
        let mut a = OwnAirAcc::default();
        // 650 B at MCS0/20 = 800 us payload + 40 us preamble, twice.
        a.on_frame(650, 0, true, 20, false, false);
        a.on_frame(650, 0, true, 20, false, false);
        assert_eq!(a.total_us(), 2 * (800 + 40));
    }

    #[test]
    fn aggregate_pays_one_preamble_and_later_subframes_inherit_width() {
        let mut a = OwnAirAcc::default();
        // 40 MHz MCS3 (54 Mb/s), STBC: 1350 B = 200 us. Later subframes
        // carry bw=20/stbc=0 (no PHY status) and are timed at 40 MHz.
        a.on_frame(1350, 3, true, 40, true, false);
        a.on_frame(1350, 3, false, 20, false, false);
        a.on_frame(1350, 3, false, 20, false, false);
        assert_eq!(a.total_us(), 3 * 200 + 40 + 4);
    }

    #[test]
    fn unknown_rate_frames_add_nothing() {
        let mut a = OwnAirAcc::default();
        a.on_frame(1000, 255, true, 20, false, false);
        assert_eq!(a.total_us(), 0);
    }

    #[test]
    fn descriptor_rates_map_to_ht_mcs() {
        assert_eq!(ht_mcs_of_rate(0x0C), 0);
        assert_eq!(ht_mcs_of_rate(0x13), 7);
        assert_eq!(ht_mcs_of_rate(0x87), 7);
        assert_eq!(ht_mcs_of_rate(0x0B), 255);
        assert_eq!(ht_mcs_of_rate(0x14), 255);
    }
}
