//! GF(2^8) with primitive polynomial 0x11D (mabur `common/src/gf256.cpp`,
//! zfec/wfb-ng compatible). Tables are built at compile time.

const POLY: u16 = 0x11D;

struct Tables {
    exp: [u8; 512],
    log: [u8; 256],
}

const fn build() -> Tables {
    let mut exp = [0u8; 512];
    let mut log = [0u8; 256];
    let mut x: u16 = 1;
    let mut i = 0;
    while i < 255 {
        exp[i] = x as u8;
        log[x as usize] = i as u8;
        x <<= 1;
        if x & 0x100 != 0 {
            x ^= POLY;
        }
        i += 1;
    }
    while i < 512 {
        exp[i] = exp[i - 255];
        i += 1;
    }
    Tables { exp, log }
}

static T: Tables = build();

/// Nibble product tables for the SIMD shuffle kernel: lo[c][x] = c*x,
/// hi[c][x] = c*(x<<4).
#[allow(dead_code)]
struct Nibbles {
    lo: [[u8; 16]; 256],
    hi: [[u8; 16]; 256],
}

#[allow(dead_code)]
const fn build_nibbles() -> Nibbles {
    let t = build();
    let mut lo = [[0u8; 16]; 256];
    let mut hi = [[0u8; 16]; 256];
    let mut c = 1;
    while c < 256 {
        let mut x = 1;
        while x < 16 {
            lo[c][x] = t.exp[t.log[c] as usize + t.log[x] as usize];
            hi[c][x] = t.exp[t.log[c] as usize + t.log[x << 4] as usize];
            x += 1;
        }
        c += 1;
    }
    Nibbles { lo, hi }
}

#[allow(dead_code)]
static N: Nibbles = build_nibbles();

#[inline]
pub fn mul(a: u8, b: u8) -> u8 {
    if a == 0 || b == 0 {
        return 0;
    }
    T.exp[T.log[a as usize] as usize + T.log[b as usize] as usize]
}

/// inv(0) = 0; repair coefficients are never 0.
#[inline]
pub fn inv(a: u8) -> u8 {
    if a == 0 {
        0
    } else {
        T.exp[255 - T.log[a as usize] as usize]
    }
}

fn lincomb_scalar(acc: &mut [u8], sym: &[u8], coeff: u8) {
    let lc = T.log[coeff as usize] as usize;
    for (a, &s) in acc.iter_mut().zip(sym) {
        if s != 0 {
            *a ^= T.exp[lc + T.log[s as usize] as usize];
        }
    }
}

/// acc[i] ^= coeff * sym[i] over min(acc.len(), sym.len()) bytes.
pub fn lincomb(acc: &mut [u8], sym: &[u8], coeff: u8) {
    if coeff == 0 {
        return;
    }
    let len = acc.len().min(sym.len());
    #[cfg(target_arch = "aarch64")]
    let done = {
        // SAFETY: NEON is mandatory on aarch64; every access stays below len.
        unsafe { neon::lincomb(acc.as_mut_ptr(), sym.as_ptr(), coeff, len) }
    };
    #[cfg(not(target_arch = "aarch64"))]
    let done = 0;
    lincomb_scalar(&mut acc[done..len], &sym[done..len], coeff);
}

/// out = XOR_k coeffs[k] * rows[k]; overwrites out (len = out.len()).
pub fn lincomb_rows(out: &mut [u8], rows: &[&[u8]], coeffs: &[u8]) {
    out.fill(0);
    for (r, &c) in rows.iter().zip(coeffs) {
        lincomb(out, r, c);
    }
}

/// Which kernel this build uses, for start-up logs.
pub fn backend() -> &'static str {
    if cfg!(target_arch = "aarch64") {
        "neon-vqtbl"
    } else {
        "scalar"
    }
}

#[cfg(target_arch = "aarch64")]
mod neon {
    use core::arch::aarch64::*;

    /// Returns the number of bytes handled (a multiple of 16).
    pub unsafe fn lincomb(acc: *mut u8, sym: *const u8, coeff: u8, len: usize) -> usize {
        let tlo = vld1q_u8(super::N.lo[coeff as usize].as_ptr());
        let thi = vld1q_u8(super::N.hi[coeff as usize].as_ptr());
        let mask = vdupq_n_u8(0x0F);
        let mut i = 0;
        while i + 16 <= len {
            let s = vld1q_u8(sym.add(i));
            let p = veorq_u8(
                vqtbl1q_u8(tlo, vandq_u8(s, mask)),
                vqtbl1q_u8(thi, vshrq_n_u8(s, 4)),
            );
            vst1q_u8(acc.add(i), veorq_u8(vld1q_u8(acc.add(i)), p));
            i += 16;
        }
        i
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inv_roundtrip() {
        for a in 1..=255u8 {
            assert_eq!(mul(a, inv(a)), 1);
        }
        assert_eq!(inv(0), 0);
    }

    #[test]
    fn nibble_tables_agree_with_mul() {
        for c in 0..=255u8 {
            for x in 0..16u8 {
                assert_eq!(N.lo[c as usize][x as usize], mul(c, x));
                assert_eq!(N.hi[c as usize][x as usize], mul(c, x << 4));
            }
        }
    }

    #[test]
    fn lincomb_matches_bytewise_mul() {
        let mut seed = 7u32;
        let mut rnd = || {
            seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12345);
            (seed >> 16) as u8
        };
        for len in [1usize, 15, 16, 17, 64, 333] {
            let sym: Vec<u8> = (0..len).map(|_| rnd()).collect();
            let mut acc: Vec<u8> = (0..len).map(|_| rnd()).collect();
            let c = rnd() | 1;
            let want: Vec<u8> = acc.iter().zip(&sym).map(|(a, s)| a ^ mul(c, *s)).collect();
            lincomb(&mut acc, &sym, c);
            assert_eq!(acc, want);
        }
    }
}
