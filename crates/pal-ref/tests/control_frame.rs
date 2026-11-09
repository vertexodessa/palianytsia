//! The Rust GS control frame (pal-wire `dot11::control_frame`) against mabur's
//! C++ `build_control_frame`, byte for byte behind mabur's radiotap: the on-air
//! header, 12-bit sequence included.
use pal_ref as _;
use pal_wire::dot11::{control_frame, next_control_seq};

extern "C" {
    fn pal_mabur_control_frame(
        seq: u16,
        body: *const u8,
        len: usize,
        out: *mut u8,
        cap: usize,
    ) -> usize;
}

/// mabur's frame with its radiotap removed, as the radiotap's own length says.
fn mabur_dot11(seq: u16, body: &[u8]) -> Vec<u8> {
    let mut out = vec![0u8; 256 + body.len()];
    let n = unsafe {
        pal_mabur_control_frame(seq, body.as_ptr(), body.len(), out.as_mut_ptr(), out.len())
    };
    assert_ne!(n, 0, "oracle buffer too small");
    out.truncate(n);
    assert_eq!(out[0], 0, "radiotap version");
    let rt = usize::from(u16::from_le_bytes([out[2], out[3]]));
    assert!(rt >= 8 && rt < n, "radiotap length {rt} of {n}");
    out.split_off(rt)
}

#[test]
fn every_sequence_value_matches() {
    let body = [0x5Au8, 0, 0xFF, 7];
    for seq in 0..=u16::MAX {
        assert_eq!(
            control_frame(seq, &body),
            mabur_dot11(seq, &body),
            "seq {seq}"
        );
    }
}

#[test]
fn every_body_length_matches() {
    for len in (0..64).chain([255, 256, 1500]) {
        let body: Vec<u8> = (0..len).map(|i| (i * 7 + 3) as u8).collect();
        assert_eq!(
            control_frame(0xABC, &body),
            mabur_dot11(0xABC, &body),
            "len {len}"
        );
    }
}

/// The counter as a card runs it, through more than one wrap: the frame must
/// match mabur's for the unwrapped count, whose bits above 12 it drops.
#[test]
fn counter_walk_matches_across_the_wrap() {
    let body = [1u8, 2, 3];
    let mut seq = 0;
    for count in 0..10_000u16 {
        assert_eq!(
            control_frame(seq, &body),
            mabur_dot11(count, &body),
            "count {count}"
        );
        seq = next_control_seq(seq);
    }
}
