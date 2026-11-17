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

/// A minimal SPS (one sub-layer, 4:2:0, 8x8 min CB, 64x64 CTB) behind a
/// type-33 NAL header, as mabur's `test_hevc_ps.cpp` builds it.
fn sps_nal(width: u32, height: u32) -> Vec<u8> {
    use hevc::bits::{escape, BitWriter};
    fn ue(w: &mut BitWriter, v: u32) {
        let n = u64::from(v) + 1;
        let len = 64 - n.leading_zeros();
        w.u(0, len - 1);
        for i in (0..len).rev() {
            w.put((n >> i) & 1 != 0);
        }
    }
    let mut w = BitWriter::default();
    w.u(0, 4); // vps id
    w.u(0, 3); // max_sub_layers_minus1
    w.u(1, 1); // temporal id nesting
    for _ in 0..3 {
        w.u(0, 32); // profile_tier_level, general only
    }
    ue(&mut w, 0); // sps id
    ue(&mut w, 1); // chroma_format_idc
    ue(&mut w, width);
    ue(&mut w, height);
    w.u(0, 1); // no conformance window
    for v in [0, 0, 4] {
        ue(&mut w, v); // bit depths, log2_max_poc_lsb_minus4
    }
    w.u(1, 1); // sub-layer ordering info
    for _ in 0..3 {
        ue(&mut w, 0);
    }
    ue(&mut w, 0); // min CB 8
    ue(&mut w, 3); // CTB 64
    for _ in 0..4 {
        ue(&mut w, 0); // transform sizes and depths
    }
    w.u(0b0010, 4); // scaling list, amp, sao, pcm
    ue(&mut w, 0); // no st_rps
    w.u(0b01, 2); // no long-term, tmvp
    w.put(true);
    w.align_zero();
    let mut nal = vec![0x42, 0x01];
    nal.extend(escape(w.bytes()));
    nal
}

/// mabur `sps_of_a_picture_no_level_allows_is_unsupported`: the SPS is
/// whatever the stream delivered and its users size buffers by it, so a
/// picture no HEVC level allows (level 6.2: 16 888 px a side, 35 651 584
/// luma samples) is unsupported at the parse.
#[test]
fn sps_of_a_picture_no_level_allows_is_unsupported() {
    use hevc::ps::PsStatus;
    let ok = parse_sps(&sps_nal(1920, 1080)).expect("1080p parses");
    assert_eq!((ok.width, ok.height, ok.log2_ctb), (1920, 1080, 6));
    assert!(parse_sps(&sps_nal(8192, 4320)).is_ok());
    assert_eq!(
        parse_sps(&sps_nal(16896, 64)).err(),
        Some(PsStatus::Unsupported)
    );
    assert_eq!(
        parse_sps(&sps_nal(8192, 8192)).err(),
        Some(PsStatus::Unsupported)
    );
    assert_eq!(
        parse_sps(&sps_nal(1 << 31, 1080)).err(),
        Some(PsStatus::Unsupported)
    );
}
