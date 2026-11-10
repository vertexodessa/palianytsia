//! Inputs that break an invariant of the per-packet path, kept as exact
//! bytes. The ignored ones are open findings: the C++ core has the same
//! arithmetic, so a fix changes ported logic and goes through mabur first.
use hevc::nal::split_nals;
use hevc::ps::{parse_pps, parse_sps};
use pal_core::frame_stream::{AuLat, FragArrival, FrameOut, FrameStream, FrameStreamCfg};
use pal_core::slice_asm::{ChunkMap, SliceAssembler, SliceFallback};
use pal_wire::frame_wire::{FrameHdr, HDR_LEN};

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

#[derive(Default)]
struct Out {
    open: bool,
    nested: u32,
    ends: u32,
}

impl FrameOut for Out {
    fn begin(&mut self, _: &FrameHdr, _: u8, _: u8) {
        self.nested += u32::from(self.open);
        self.open = true;
    }
    fn data(&mut self, _: &[u8]) {}
    fn end(&mut self, _: bool, _: &AuLat) {
        self.open = false;
        self.ends += 1;
    }
}

/// One fragment `<fseq, idx 0, count>`, FrameHdr, 4 AU bytes.
fn frame(fseq: u16, count: u16, frame_id: u16) -> Vec<u8> {
    let mut p = vec![0u8; 6];
    p[..2].copy_from_slice(&fseq.to_le_bytes());
    p[4..6].copy_from_slice(&count.to_le_bytes());
    let h = FrameHdr {
        frame_id,
        ..FrameHdr::default()
    };
    p.extend_from_slice(&h.pack());
    p.extend_from_slice(&[0, 0, 1, 0x26]);
    p
}

/// Frame ids 0, 32768, 0, 65535: each step a delta of -32768 or -1, so the
/// unwrapped id walks from 0x10000 to below zero and lands on u64::MAX; the
/// emit cursor then overflows. A release build wraps instead, counts about
/// u64::MAX dropped frames and parks the cursor at the top, so every later
/// frame is late until the stall reset. Any transmitter that can put four
/// FEC-valid frames on the air reaches it; no key is involved.
#[test]
#[ignore = "open: a frame id walk below zero overflows the emit cursor (same in mabur)"]
fn frame_id_walk_below_zero() {
    let mut fs = FrameStream::new(FrameStreamCfg::default());
    let mut out = Out::default();
    for (i, id) in [0u16, 32768, 0, 65535].into_iter().enumerate() {
        fs.push_fragment(
            0,
            &frame(i as u16, 1, id),
            10,
            &FragArrival::default(),
            &mut out,
        );
    }
    assert!(fs.stats().dropped < 8, "{:?}", fs.stats());
}

/// Two slots whose headers name one frame id: the second begins while the
/// first is open, so the core's AU sink drops the open AU's bytes and the
/// first slot's later data and end land on an empty AU. No panic; mabur's
/// FrameStream has the same shape. Reached only by FEC-valid symbols that
/// repeat a frame id on another fseq or layer (forged, or a corrupt header
/// the per-block CRC let through).
#[test]
#[ignore = "open: slots with one frame id nest begin and end (same in mabur)"]
fn two_slots_one_frame_id_nest() {
    let mut fs = FrameStream::new(FrameStreamCfg::default());
    let mut out = Out::default();
    fs.push_fragment(1, &frame(9, 2, 5), 10, &FragArrival::default(), &mut out);
    fs.push_fragment(0, &frame(3, 2, 5), 10, &FragArrival::default(), &mut out);
    fs.poll(1000, &mut out);
    assert_eq!(out.nested, 0);
}
