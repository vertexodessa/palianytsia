//! The RCF profile byte: bits 3:0 MCS, bits 5:4 width code (20/40/80 MHz),
//! bit 6 VHT. As `common/src/profile.cpp`.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PhyMode {
    Ht,
    Vht,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Profile {
    pub mode: PhyMode,
    pub mcs: u8,
    pub bw_mhz: u8,
}

fn bw_to_code(bw: u8) -> u8 {
    match bw {
        40 => 1,
        80 => 2,
        // The C++ maps every unknown width to the 20 MHz code, silently.
        _ => 0,
    }
}

fn code_to_bw(code: u8) -> u8 {
    match code {
        1 => 40,
        2 => 80,
        _ => 20,
    }
}

pub fn encode(p: Profile) -> u8 {
    let mut b = (p.mcs & 0x0F) | (bw_to_code(p.bw_mhz) << 4);
    if p.mode == PhyMode::Vht {
        b |= 0x40;
    }
    b
}

/// MCS is clamped to 7 (HT) or 8 (VHT), as the drone does.
pub fn decode(b: u8) -> Profile {
    let mode = if b & 0x40 != 0 {
        PhyMode::Vht
    } else {
        PhyMode::Ht
    };
    Profile {
        mode,
        mcs: layer_mcs(mode, b & 0x0F),
        bw_mhz: code_to_bw((b >> 4) & 0x3),
    }
}

const VHT20_LGI: [f64; 9] = [6.5, 13.0, 19.5, 26.0, 39.0, 52.0, 58.5, 65.0, 78.0];
const VHT40_LGI: [f64; 9] = [13.5, 27.0, 40.5, 54.0, 81.0, 108.0, 121.5, 135.0, 162.0];
const VHT80_LGI: [f64; 9] = [29.3, 58.5, 87.8, 117.0, 175.5, 234.0, 263.3, 292.5, 351.0];

/// On-air PHY rate in Mbps, long GI (`phy_rate_mbps` of a `ladder_from`
/// spec, which never sets SGI).
pub fn phy_rate_mbps(mode: PhyMode, mcs: u8, bw_mhz: u8) -> f64 {
    let t = match bw_mhz {
        40 => &VHT40_LGI,
        80 if mode == PhyMode::Vht => &VHT80_LGI,
        _ => &VHT20_LGI,
    };
    t[usize::from(layer_mcs(mode, mcs))]
}

/// `ladder_from`'s MCS for both layers: clamped to 7 (HT) / 8 (VHT).
pub fn layer_mcs(mode: PhyMode, mcs: u8) -> u8 {
    mcs.min(if mode == PhyMode::Vht { 8 } else { 7 })
}
