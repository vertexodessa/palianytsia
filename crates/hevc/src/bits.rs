//! RBSP escaping and MSB-first bit I/O (mabur `hevc_bits.cpp`).

pub fn unescape(p: &[u8]) -> Vec<u8> {
    let mut o = Vec::with_capacity(p.len());
    let mut zeros = 0;
    for &b in p {
        if zeros >= 2 && b == 3 {
            zeros = 0;
            continue;
        }
        o.push(b);
        zeros = if b == 0 { zeros + 1 } else { 0 };
    }
    o
}

pub fn escape(p: &[u8]) -> Vec<u8> {
    let mut o = Vec::with_capacity(p.len() + p.len() / 64 + 4);
    let mut zeros = 0;
    for &b in p {
        if zeros >= 2 && b <= 3 {
            o.push(3);
            zeros = 0;
        }
        o.push(b);
        zeros = if b == 0 { zeros + 1 } else { 0 };
    }
    o
}

/// Smallest c with (1 << c) >= x; 0 for x <= 1.
pub fn ceil_log2(x: u32) -> u32 {
    let mut c = 0;
    while (1u64 << c) < u64::from(x) {
        c += 1;
    }
    c
}

pub struct BitReader<'a> {
    d: &'a [u8],
    pos: usize,
    bad: bool,
}

impl<'a> BitReader<'a> {
    pub fn new(d: &'a [u8]) -> Self {
        Self {
            d,
            pos: 0,
            bad: false,
        }
    }
    pub fn u(&mut self, bits: u32) -> u32 {
        let mut v = 0u32;
        for _ in 0..bits {
            let byte = self.pos >> 3;
            if byte >= self.d.len() {
                self.bad = true;
                return 0;
            }
            v = (v << 1) | u32::from((self.d[byte] >> (7 - (self.pos & 7))) & 1);
            self.pos += 1;
        }
        v
    }
    pub fn flag(&mut self) -> bool {
        self.u(1) != 0
    }
    pub fn ue(&mut self) -> u32 {
        let mut lz = 0;
        while self.u(1) == 0 {
            lz += 1;
            if self.bad || lz > 31 {
                self.bad = true;
                return 0;
            }
        }
        ((1u64 << lz) - 1) as u32 + self.u(lz)
    }
    pub fn se(&mut self) -> i32 {
        let k = self.ue();
        if k & 1 != 0 {
            k.div_ceil(2) as i32
        } else {
            -((k / 2) as i32)
        }
    }
    pub fn pos(&self) -> usize {
        self.pos
    }
    pub fn bad(&self) -> bool {
        self.bad
    }
}

#[derive(Default)]
pub struct BitWriter {
    buf: Vec<u8>,
    bits: usize,
}

impl BitWriter {
    pub fn put(&mut self, bit: bool) {
        if self.bits & 7 == 0 {
            self.buf.push(0);
        }
        if bit {
            *self.buf.last_mut().unwrap() |= 0x80u8 >> (self.bits & 7);
        }
        self.bits += 1;
    }
    pub fn u(&mut self, v: u32, bits: u32) {
        for i in (0..bits).rev() {
            self.put((v >> i) & 1 != 0);
        }
    }
    /// Appends src bits [from_bit, to_bit).
    pub fn copy(&mut self, src: &[u8], from_bit: usize, to_bit: usize) {
        for b in from_bit..to_bit {
            self.put((src[b >> 3] >> (7 - (b & 7))) & 1 != 0);
        }
    }
    pub fn align_zero(&mut self) {
        while self.bits & 7 != 0 {
            self.put(false);
        }
    }
    pub fn bits(&self) -> usize {
        self.bits
    }
    pub fn bytes(&self) -> &[u8] {
        &self.buf
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn ue_se_roundtrip() {
        // ue(v) of 0, 1, 2, 7 and 255: 1 010 011 0001000 00000000100000000.
        let bytes = [0b1010_0110, 0b0010_0000, 0b0000_0010, 0];
        let mut r = BitReader::new(&bytes);
        for v in [0u32, 1, 2, 7, 255] {
            assert_eq!(r.ue(), v);
        }
        assert!(!r.bad());
    }
    #[test]
    fn escape_roundtrip() {
        let raw = [0u8, 0, 1, 0, 0, 0, 0, 0, 3, 5];
        let e = escape(&raw);
        assert_eq!(e, [0, 0, 3, 1, 0, 0, 3, 0, 0, 3, 0, 3, 5]);
        assert_eq!(unescape(&e), raw);
    }
}
