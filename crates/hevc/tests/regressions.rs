//! Inputs the property tests (`tests/prop.rs`) once failed on, kept as
//! exact bytes. Each must parse to a result, never panic.
use hevc::ps::parse_sps;

/// An SPS whose min-CB size field decodes past 31: the min-CB mask shifted
/// a u32 by more than its width.
#[test]
fn sps_min_cb_shift_past_u32() {
    let nal = [
        0, 0, 82, 0, 0, 0, 0, 4, 0, 0, 4, 0, 0, 4, 0, 192, 0, 0, 0, 4, 0, 0, 4, 0, 0, 4, 0, 0, 4,
        212, 173, 192, 8, 0, 158, 117, 0, 8, 0, 8, 0, 34, 32, 129, 18,
    ];
    assert!(parse_sps(&nal).is_err());
}

/// 1080p SPS fields with the three size fields at the top of the ue()
/// range: each sum wraps (as the C++ unsigned one does) instead of
/// overflowing, and the range checks judge the wrapped value.
#[test]
fn sps_size_field_sums_wrap() {
    let valid = [
        66, 1, 1, 0, 0, 3, 0, 0, 3, 0, 0, 3, 0, 0, 3, 0, 0, 3, 0, 0, 160, 3, 192, 128, 16, 229,
        141, 121, 60, 168, 64,
    ];
    let s = parse_sps(&valid).unwrap();
    assert_eq!(
        (s.width, s.height, s.log2_ctb, s.log2_max_poc_lsb),
        (1920, 1080, 6, 16)
    );
    // log2_min_luma_coding_block_size_minus3 = 2^32 - 2: min CB wraps to 1.
    let min_cb = [
        66, 1, 1, 0, 0, 3, 0, 0, 3, 0, 0, 3, 0, 0, 3, 0, 0, 3, 0, 0, 160, 3, 192, 128, 16, 229,
        141, 112, 0, 0, 3, 0, 31, 255, 255, 255, 255, 42, 16,
    ];
    assert!(parse_sps(&min_cb).is_err());
    // log2_diff_max_min_luma_coding_block_size = 2^32 - 2: CTB wraps to 1.
    let ctb = [
        66, 1, 1, 0, 0, 3, 0, 0, 3, 0, 0, 3, 0, 0, 3, 0, 0, 3, 0, 0, 160, 3, 192, 128, 16, 229,
        141, 120, 0, 0, 3, 0, 15, 255, 255, 255, 255, 42, 16,
    ];
    assert!(parse_sps(&ctb).is_err());
    // log2_max_pic_order_cnt_lsb_minus4 = 2^32 - 2: wraps to 2, accepted.
    let poc = [
        66, 1, 1, 0, 0, 3, 0, 0, 3, 0, 0, 3, 0, 0, 3, 0, 0, 3, 0, 0, 160, 3, 192, 128, 16, 229,
        128, 0, 0, 3, 0, 255, 255, 255, 255, 121, 60, 168, 64,
    ];
    assert_eq!(parse_sps(&poc).unwrap().log2_max_poc_lsb, 2);
}
