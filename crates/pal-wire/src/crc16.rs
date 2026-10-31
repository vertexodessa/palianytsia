//! CRC-16/CCITT-FALSE (poly 0x1021, MSB first), as `common/src/crc16.cpp`.
//! Guards RC frames and every SBI block.

const TABLE: [u16; 256] = make_table();

const fn make_table() -> [u16; 256] {
    let mut t = [0u16; 256];
    let mut i = 0;
    while i < 256 {
        let mut crc = (i as u16) << 8;
        let mut b = 0;
        while b < 8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
            b += 1;
        }
        t[i] = crc;
        i += 1;
    }
    t
}

/// The C++ default `init` is 0xFFFF.
pub const INIT: u16 = 0xFFFF;

pub fn crc16_ccitt_with(data: &[u8], init: u16) -> u16 {
    data.iter().fold(init, |crc, &b| {
        (crc << 8) ^ TABLE[usize::from((crc >> 8) as u8 ^ b)]
    })
}

pub fn crc16_ccitt(data: &[u8]) -> u16 {
    crc16_ccitt_with(data, INIT)
}
