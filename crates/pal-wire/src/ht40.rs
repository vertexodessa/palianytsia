//! The 5 GHz HT40 pair grid (mabur common/include/mabur/ht40.h): which
//! channels can carry a 40 MHz link and where the pair's other half is.
//! Here, not in pal-core, so the UI can offer only the channels a width
//! allows.

use alloc::vec::Vec;

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

/// The 20 MHz channels of the 5 GHz grid the link can sit on (UNII-1, 2,
/// 2e, 3). mabur's drone follows the GS to any of them (mabur
/// `channel_5g`, fork commit 44069eb4).
pub const GRID_5G: [u8; 25] = [
    36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144,
    149, 153, 157, 161, 165,
];

/// The grid channels a link at `bw` MHz can use: at 40 those with a pair.
pub fn grid_5g(bw: u8) -> Vec<u8> {
    GRID_5G
        .iter()
        .copied()
        .filter(|&c| bw != 40 || ht40_offset(c) != 0)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn grid_at_40_drops_only_165() {
        assert_eq!(grid_5g(20).len(), 25);
        let g40 = grid_5g(40);
        assert_eq!(g40.len(), 24);
        assert!(!g40.contains(&165));
        assert_eq!(ht40_offset(36), 1);
        assert_eq!(ht40_offset(40), 2);
        assert_eq!(ht40_pair_other(161), 157);
    }
}
