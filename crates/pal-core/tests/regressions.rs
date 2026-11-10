//! Inputs that break an invariant of the per-packet path, kept as exact
//! bytes.
use hevc::nal::split_nals;
use hevc::ps::{parse_pps, parse_sps};
use pal_core::slice_asm::{ChunkMap, SliceAssembler, SliceFallback};
use pal_wire::frame_wire::HDR_LEN;

/// A truncated row-sliced AU under an SPS whose width is 2^31 (a multiple
/// of the min CB, so the SPS parses): 128 rows per slice times 2^25 CTBs a
/// row is 2^32, which overflows the u32 slice span; the C++ sum wraps to
/// 0, and a zero span then divides the slice address (a crash in both a
/// release build and mabur). Salvage must refuse the geometry instead.
#[test]
fn slice_span_of_a_huge_picture() {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../third_party/mabur/tests/fixtures/slices4_1080p.h265");
    let b = std::fs::read(p).unwrap();
    let nals = split_nals(&b);
    let nal = |ty: u8| nals.iter().find(|v| v.ty == ty).unwrap();
    let (s, p) = (nal(33), nal(34));
    let mut sps = parse_sps(&b[s.off..s.off + s.len]).unwrap();
    let pps = parse_pps(&b[p.off..p.off + p.len]).unwrap();
    sps.width = 1 << 31;
    // Chunk 0: FrameHdr, the first slice whole, the next slice's start;
    // chunk 1 of 3 is lost.
    let slices: Vec<_> = nals.iter().filter(|v| v.ty < 32).take(2).collect();
    let mut c0 = vec![0u8; HDR_LEN];
    c0.extend_from_slice(&[0, 0, 1]);
    c0.extend_from_slice(&b[slices[0].off..slices[0].off + slices[0].len]);
    c0.extend_from_slice(&[0, 0, 1]);
    c0.extend_from_slice(&b[slices[1].off..slices[1].off + 8]);
    let mut chunks = ChunkMap::new();
    chunks.insert(0, c0);
    let mut sa = SliceAssembler::new(&sps, &pps, 128, 3, HDR_LEN);
    assert_eq!(sa.slices(), 1);
    sa.finish(&chunks, &mut |_| {}, &mut |_| {});
    assert_eq!(sa.result().fallback, SliceFallback::Geometry);
}
