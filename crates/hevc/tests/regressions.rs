//! Inputs the property tests (`tests/prop.rs`) once failed on, kept as
//! exact bytes. Each must parse to a result, never panic.
use hevc::nal::split_nals;
use hevc::ps::{parse_pps, parse_sps, Pps, Sps};
use hevc::slice::parse_slice_header;

/// mabur's 1080p fixture SPS and PPS, the parameter sets slices parse with.
fn fixture_ps() -> (Sps, Pps) {
    let p = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../third_party/mabur/tests/fixtures/slices4_1080p.h265");
    let b = std::fs::read(p).expect("fixture (git submodule update --init?)");
    let nals = split_nals(&b);
    let get = |ty: u8| {
        let v = nals.iter().find(|v| v.ty == ty).unwrap();
        b[v.off..v.off + v.len].to_vec()
    };
    (parse_sps(&get(33)).unwrap(), parse_pps(&get(34)).unwrap())
}

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

/// A PPS whose init_qp_minus26 is i32::MAX: 26 + it overflows an i32.
#[test]
fn pps_init_qp_past_i32() {
    let nal = [68, 1, 192, 96, 0, 0, 3, 0, 63, 255, 255, 255, 134, 0, 72];
    let p = parse_pps(&nal).unwrap();
    assert_eq!(p.init_qp, 26i32.wrapping_add(i32::MAX));
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

/// An IDR slice header whose slice_qp_delta decodes to 2^31 - 4: added to
/// the PPS init_qp it overflows an i32.
#[test]
fn slice_qp_delta_past_i32() {
    let (sps, pps) = fixture_ps();
    let nal = [38, 1, 32, 21, 32, 0, 0, 3, 0, 63, 255, 255, 254, 32];
    let _ = parse_slice_header(&nal, &sps, &pps);
}

/// A P slice under an SPS with four long-term pictures: num_long_term_sps
/// 2 and num_long_term_pics 2^32 - 2. Their sum overflows a u32 before the
/// "at most 32" check (the C++ sum wraps and passes it).
#[test]
fn slice_long_term_count_sum_past_u32() {
    let (mut sps, pps) = fixture_ps();
    sps.long_term = true;
    sps.num_lt_sps = 4;
    sps.lt_used_sps = vec![1, 0, 1, 0];
    let nal = [2, 1, 208, 0, 4, 96, 0, 0, 3, 0, 63, 255, 255, 255, 224];
    let _ = parse_slice_header(&nal, &sps, &pps);
}
