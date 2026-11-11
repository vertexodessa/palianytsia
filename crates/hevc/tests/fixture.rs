//! mabur's 4-slice 1080p fixture (tests/fixtures/slices4_1080p.h265), the
//! same checks as test_hevc_ps / test_hevc_slice / test_hevc_skip_slice.
use hevc::bits::BitWriter;
use hevc::nal::split_nals;
use hevc::ps::{parse_pps, parse_sps, ParamTracker};
use hevc::skip_slice::{make_skip_slice, skip_fill_steps, FillBin};
use hevc::slice::{parse_slice_header, write_slice_header, HeaderEdit};
use std::path::PathBuf;

fn fixture() -> Vec<u8> {
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../third_party/mabur/tests/fixtures/slices4_1080p.h265");
    std::fs::read(p).expect("fixture (git submodule update --init?)")
}

/// Slice NALs (from the 2-byte header), grouped per picture by first_slice flag.
fn pictures(b: &[u8]) -> Vec<Vec<&[u8]>> {
    let mut out: Vec<Vec<&[u8]>> = Vec::new();
    for v in split_nals(b) {
        if v.ty >= 32 {
            continue;
        }
        let nal = &b[v.off..v.off + v.len];
        if nal[2] & 0x80 != 0 {
            out.push(Vec::new());
        }
        out.last_mut().unwrap().push(nal);
    }
    out
}

#[test]
fn sps_pps_of_the_bench_encoder() {
    let b = fixture();
    let nals = split_nals(&b);
    let s = nals.iter().find(|v| v.ty == 33).unwrap();
    let sps = parse_sps(&b[s.off..s.off + s.len]).unwrap();
    assert_eq!(
        (sps.width, sps.height, sps.chroma_format_idc),
        (1920, 1080, 1)
    );
    assert_eq!((sps.log2_ctb, sps.log2_min_cb), (6, 3));
    assert_eq!(
        (sps.pic_w_ctbs(), sps.pic_h_ctbs(), sps.addr_bits()),
        (30, 17, 9)
    );
    assert!(sps.sao && sps.tmvp && !sps.long_term);
    assert_eq!((sps.log2_max_poc_lsb, sps.num_st_rps), (16, 3));
    let p = nals.iter().find(|v| v.ty == 34).unwrap();
    let pps = parse_pps(&b[p.off..p.off + p.len]).unwrap();
    assert!(pps.dependent_slices && pps.lists_modification && pps.loop_filter_across_slices);
    assert!(!pps.cabac_init_present && !pps.deblocking_override_enabled);
    assert_eq!(pps.init_qp, 26);
}

#[test]
fn every_header_parses_and_rewrites_bit_exact() {
    let b = fixture();
    let mut t = ParamTracker::default();
    t.feed(&b);
    assert!(t.usable());
    let pics = pictures(&b);
    assert_eq!(pics.len(), 7);
    for (a, pic) in pics.iter().enumerate() {
        if pic.len() == 1 {
            continue; // the refresh-start picture is one slice
        }
        assert_eq!(pic.len(), 4);
        for (k, nal) in pic.iter().enumerate() {
            let h = parse_slice_header(nal, t.sps(), t.pps()).unwrap();
            assert_eq!(h.first, k == 0);
            assert_eq!(h.address, 150 * k as u32);
            assert_eq!(h.slice_type, if a == 0 { 2 } else { 1 });
            if a != 0 {
                assert_eq!(h.max_num_merge_cand, 2);
            }
            assert!(h.sao_luma && h.sao_chroma);
            let mut w = BitWriter::default();
            write_slice_header(
                &mut w,
                &h,
                &HeaderEdit {
                    first: h.first,
                    address: h.address,
                    clear_sao: false,
                },
                t.sps(),
                t.pps(),
            );
            assert_eq!(w.bits(), h.data_byte * 8);
            assert_eq!(&h.rbsp[..h.data_byte], w.bytes());
        }
    }
}

#[test]
fn fill_steps_and_slices() {
    let b = fixture();
    let mut t = ParamTracker::default();
    t.feed(&b);
    let s = skip_fill_steps(t.sps(), 2, 30, 60);
    assert_eq!(s.len(), 120);
    assert_eq!((s[0].kind, s[0].ctx_inc), (FillBin::Split, 0));
    assert_eq!((s[5].kind, s[5].ctx_inc), (FillBin::Skip, 1));
    assert_eq!(skip_fill_steps(t.sps(), 2, 30, 90)[124 + 1].ctx_inc, 2);
    let bot = skip_fill_steps(t.sps(), 2, 480, 481);
    let n = |k| bot.iter().filter(|x| x.kind == k).count();
    assert_eq!(
        (n(FillBin::Split), n(FillBin::Skip), n(FillBin::MergeIdx)),
        (6, 14, 14)
    );
    let pics = pictures(&b);
    let tmpl = parse_slice_header(pics[5][1], t.sps(), t.pps()).unwrap();
    for (f, e) in [(0u32, 150u32), (150, 300), (450, 510)] {
        let fill = make_skip_slice(t.sps(), t.pps(), &tmpl, f, e).unwrap();
        assert_eq!(&fill[..4], &[0, 0, 0, 1]);
        let h = parse_slice_header(&fill[4..], t.sps(), t.pps()).unwrap();
        assert_eq!(
            (h.first, h.address, h.sao_luma, h.sao_chroma),
            (f == 0, f, false, false)
        );
    }
    let idr = parse_slice_header(pics[0][0], t.sps(), t.pps()).unwrap();
    assert!(make_skip_slice(t.sps(), t.pps(), &idr, 0, 150).is_none());
}

/// The fill's work grid is sized by the SPS, and the SPS is whatever the
/// stream delivered: a picture no HEVC level allows is refused before
/// anything is allocated by it (1 048 576 x 1 044 480 would ask for three
/// vectors of 17 GB).
#[test]
fn a_picture_no_level_allows_is_refused() {
    let b = fixture();
    let mut t = ParamTracker::default();
    t.feed(&b);
    let pics = pictures(&b);
    let tmpl = parse_slice_header(pics[5][1], t.sps(), t.pps()).unwrap();
    let mut forged = t.sps().clone();
    forged.width = 1 << 20;
    forged.height = 64 * 255 * 64;
    assert!(make_skip_slice(&forged, t.pps(), &tmpl, 0, 1).is_none());
    // One side over the limit is enough.
    let mut wide = t.sps().clone();
    wide.width = 16_896;
    wide.height = 64;
    assert!(make_skip_slice(&wide, t.pps(), &tmpl, 0, 1).is_none());
    // The largest common picture, 8192 x 4320, still gets its fill.
    let mut big = t.sps().clone();
    big.width = 8192;
    big.height = 4320;
    assert!(make_skip_slice(&big, t.pps(), &tmpl, 0, 1).is_some());
}
