//! The 5 GHz HT40 pair grid (mabur common/include/mabur/ht40.h): which
//! channels can carry a 40 MHz link and where the pair's other half is.
//! Here, not in pal-core, so the UI can offer only the channels a width
//! allows.

/// 0 = no standard HT40 pair; 1 = secondary above; 2 = below.
pub fn ht40_offset(ch: u8) -> u8 {
    if (36..=144).contains(&ch) && (ch - 36).is_multiple_of(4) {
        return if ((ch - 36) / 4).is_multiple_of(2) {
            1
        } else {
            2
        };
    }
    if (149..=161).contains(&ch) && (ch - 149).is_multiple_of(4) {
        return if ((ch - 149) / 4).is_multiple_of(2) {
            1
        } else {
            2
        };
    }
    0
}

pub fn ht40_pair_other(ch: u8) -> u8 {
    match ht40_offset(ch) {
        1 => ch + 4,
        2 => ch - 4,
        _ => 0,
    }
}
