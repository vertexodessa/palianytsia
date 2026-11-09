//! dot11 RX parse, as maburgs `fill_rx_body` (`gs/src/dot11.cpp`). Header
//! only: the caller copies `frame[body_off..]` (FCS included, as maburgs
//! keeps it) and adds its own clock and card id.

/// mabur's canonical SA, on both directions of the link.
pub const SA: [u8; 6] = [0x57, 0x42, 0x75, 0x05, 0xd6, 0x00];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// Too short for the header plus one body byte.
    Short,
    /// CRC-good with a foreign SA: never reaches the core.
    Foreign,
    Body(Parsed),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Parsed {
    pub body_off: usize,
    pub mac_seq: u16,
    /// HT MCS, 255 = unknown.
    pub mcs: u8,
}

/// 26 for QoS-Data (fc 0x88, the drone video wire), else 24; 0 when the
/// frame cannot hold the header plus one byte.
pub fn body_offset(d: &[u8]) -> usize {
    let off = if d.first() == Some(&0x88) { 26 } else { 24 };
    if d.len() > off {
        off
    } else {
        0
    }
}

pub fn sa_canonical(d: &[u8]) -> bool {
    d.len() >= 16 && d[10..16] == SA
}

/// RX rate code to HT MCS: Jaguar DESC_RATE 0x0C..0x13 or Kestrel 0x80..0x87.
pub fn mcs_from_rate(dr: u16) -> u8 {
    match dr {
        0x0C..=0x13 => (dr - 0x0C) as u8,
        0x80..=0x87 => (dr - 0x80) as u8,
        _ => 255,
    }
}

/// CRC-failed frames pass the SA check: a corrupt SA proves nothing and
/// their intact SBI blocks are salvageable.
pub fn parse(d: &[u8], crc_err: bool, data_rate: u16) -> Verdict {
    let body_off = body_offset(d);
    if body_off == 0 {
        return Verdict::Short;
    }
    if !crc_err && !sa_canonical(d) {
        return Verdict::Foreign;
    }
    Verdict::Body(Parsed {
        body_off,
        mac_seq: u16::from_le_bytes([d[22], d[23]]) >> 4,
        mcs: mcs_from_rate(data_rate),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(fc: u8, sa: [u8; 6], seq: u16) -> [u8; 64] {
        let mut f = [0u8; 64];
        f[0] = fc;
        f[10..16].copy_from_slice(&sa);
        let sc = (seq << 4).to_le_bytes();
        f[22] = sc[0];
        f[23] = sc[1];
        f
    }

    #[test]
    fn qos_data_offset_and_seq() {
        let f = frame(0x88, SA, 0xABC);
        assert_eq!(
            parse(&f, false, 0x0F),
            Verdict::Body(Parsed {
                body_off: 26,
                mac_seq: 0xABC,
                mcs: 3
            })
        );
    }

    #[test]
    fn probe_req_offset_kestrel_rate() {
        let f = frame(0x40, SA, 7);
        assert_eq!(
            parse(&f, false, 0x85),
            Verdict::Body(Parsed {
                body_off: 24,
                mac_seq: 7,
                mcs: 5
            })
        );
    }

    #[test]
    fn foreign_unless_crc_bad() {
        let f = frame(0x88, [1, 2, 3, 4, 5, 6], 1);
        assert_eq!(parse(&f, false, 0x0C), Verdict::Foreign);
        assert!(matches!(parse(&f, true, 0x0C), Verdict::Body(_)));
    }

    #[test]
    fn short_frames() {
        let f = frame(0x88, SA, 1);
        assert_eq!(parse(&f[..26], false, 0), Verdict::Short);
        assert!(matches!(parse(&f[..27], false, 0), Verdict::Body(_)));
        assert_eq!(parse(&f[..24], false, 0), Verdict::Short);
        assert_eq!(parse(&[], false, 0), Verdict::Short);
    }

    #[test]
    fn unknown_rates() {
        assert_eq!(mcs_from_rate(0x0B), 255);
        assert_eq!(mcs_from_rate(0x14), 255);
        assert_eq!(mcs_from_rate(0x88), 255);
        assert_eq!(mcs_from_rate(0x13), 7);
    }
}
