//! SipHash-2-4, 128-bit key, 64-bit output (mabur `siphash.cpp`): the MAC on
//! every GS->drone RC frame.

#[inline]
fn round(v: &mut [u64; 4]) {
    v[0] = v[0].wrapping_add(v[1]);
    v[1] = v[1].rotate_left(13);
    v[1] ^= v[0];
    v[0] = v[0].rotate_left(32);
    v[2] = v[2].wrapping_add(v[3]);
    v[3] = v[3].rotate_left(16);
    v[3] ^= v[2];
    v[0] = v[0].wrapping_add(v[3]);
    v[3] = v[3].rotate_left(21);
    v[3] ^= v[0];
    v[2] = v[2].wrapping_add(v[1]);
    v[1] = v[1].rotate_left(17);
    v[1] ^= v[2];
    v[2] = v[2].rotate_left(32);
}

pub fn siphash24(key: &[u8; 16], msg: &[u8]) -> u64 {
    let k0 = u64::from_le_bytes(key[0..8].try_into().unwrap());
    let k1 = u64::from_le_bytes(key[8..16].try_into().unwrap());
    let mut v = [
        0x736f_6d65_7073_6575 ^ k0,
        0x646f_7261_6e64_6f6d ^ k1,
        0x6c79_6765_6e65_7261 ^ k0,
        0x7465_6462_7974_6573 ^ k1,
    ];
    let mut chunks = msg.chunks_exact(8);
    for c in &mut chunks {
        let m = u64::from_le_bytes(c.try_into().unwrap());
        v[3] ^= m;
        round(&mut v);
        round(&mut v);
        v[0] ^= m;
    }
    let mut b = (msg.len() as u64) << 56;
    for (i, &x) in chunks.remainder().iter().enumerate() {
        b |= u64::from(x) << (8 * i);
    }
    v[3] ^= b;
    round(&mut v);
    round(&mut v);
    v[0] ^= b;
    v[2] ^= 0xff;
    for _ in 0..4 {
        round(&mut v);
    }
    v[0] ^ v[1] ^ v[2] ^ v[3]
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn reference_vectors() {
        let mut k = [0u8; 16];
        for (i, b) in k.iter_mut().enumerate() {
            *b = i as u8;
        }
        let msg: [u8; 15] = core::array::from_fn(|i| i as u8);
        assert_eq!(siphash24(&k, &msg[..0]), 0x726f_db47_dd0e_0e31);
        assert_eq!(siphash24(&k, &msg[..1]), 0x74f8_39c5_93dc_67fd);
        assert_eq!(siphash24(&k, &msg[..15]), 0xa129_ca61_49be_45e5);
    }
}
