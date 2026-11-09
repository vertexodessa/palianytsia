//! Annex-B start-code walk (mabur `hevc_params.cpp`).

/// (pos, code_len) of the first 00 00 01 / 00 00 00 01 at or after `from`.
pub fn find_start_code(au: &[u8], from: usize) -> Option<(usize, usize)> {
    let n = au.len();
    let mut j = from;
    while j + 3 <= n {
        if au[j] == 0 && au[j + 1] == 0 {
            if au[j + 2] == 1 {
                return Some((j, 3));
            }
            if au[j + 2] == 0 && j + 4 <= n && au[j + 3] == 1 {
                return Some((j, 4));
            }
        }
        j += 1;
    }
    None
}

/// The type field of a NAL header's first byte.
pub fn nal_type(first: u8) -> u8 {
    (first >> 1) & 0x3F
}

/// One NAL unit: `off..off+len` of the AU, from the 2-byte NAL header.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NalView {
    pub off: usize,
    pub len: usize,
    pub ty: u8,
}

/// Splits an Annex-B AU into NAL views; empty NALs are skipped.
pub fn split_nals(au: &[u8]) -> Vec<NalView> {
    let mut out = Vec::new();
    let Some((pos, cl)) = find_start_code(au, 0) else {
        return out;
    };
    let mut begin = pos + cl;
    loop {
        let next = find_start_code(au, begin);
        let end = next.map_or(au.len(), |(p, _)| p);
        if end > begin {
            out.push(NalView {
                off: begin,
                len: end - begin,
                ty: nal_type(au[begin]),
            });
        }
        match next {
            Some((p, l)) => begin = p + l,
            None => break,
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn splits_mixed_codes() {
        let au = [
            0, 0, 0, 1, 0x40, 1, 9, 0, 0, 1, 0x42, 1, 0, 0, 1, 0, 0, 1, 0x26, 1,
        ];
        let v = split_nals(&au);
        assert_eq!(v.len(), 3);
        assert_eq!((v[0].ty, v[1].ty, v[2].ty), (32, 33, 19));
        assert_eq!(v[0].len, 3);
    }
}
