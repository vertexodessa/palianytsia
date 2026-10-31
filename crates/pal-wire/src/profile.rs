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
    let top = if mode == PhyMode::Vht { 8 } else { 7 };
    Profile {
        mode,
        mcs: (b & 0x0F).min(top),
        bw_mhz: code_to_bw((b >> 4) & 0x3),
    }
}
