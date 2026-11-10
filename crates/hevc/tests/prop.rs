//! Property tests of the H.265 parsers the salvage path runs on received
//! AUs: arbitrary bytes and bent copies of mabur's 1080p fixture NALs never
//! panic, the bit I/O and escaping round-trip, a header the parser accepts
//! rewrites to the same bits, and a skip fill parses back at the address it
//! was made for.
//!
//! Run in the test profile, where overflow checks are on: a wrapping
//! subtraction on hostile input fails here although release builds would
//! carry on with a wrapped value. Deterministic (fixed seed, no failure
//! files); `PAL_PROP_SCALE=N` multiplies the case counts and
//! `PAL_PROP_SEED=S` picks another seed for a long local run.
use hevc::bits::{escape, unescape, BitReader, BitWriter};
use hevc::nal::{find_start_code, nal_type, split_nals};
use hevc::ps::{parse_pps, parse_sps, ParamTracker, Pps, Sps};
use hevc::skip_slice::{make_skip_slice, skip_fill_steps};
use hevc::slice::{parse_slice_header, write_slice_header, HeaderEdit};
use proptest::prelude::*;
use proptest::test_runner::{Config, RngAlgorithm, RngSeed};
use std::path::PathBuf;
use std::sync::OnceLock;

fn cfg(cases: u32) -> Config {
    let env = |k: &str| std::env::var(k).ok().and_then(|v| v.parse::<u64>().ok());
    let scale = env("PAL_PROP_SCALE").unwrap_or(1).max(1);
    Config {
        cases: (u64::from(cases) * scale).min(u64::from(u32::MAX)) as u32,
        failure_persistence: None,
        rng_algorithm: RngAlgorithm::ChaCha,
        rng_seed: RngSeed::Fixed(env("PAL_PROP_SEED").unwrap_or(0x9a1)),
        ..Config::default()
    }
}

#[derive(Clone, Debug)]
enum Edit {
    Flip(usize, u8),
    Set(usize, u8),
    Truncate(usize),
    Extend(Vec<u8>),
    Insert(usize, u8),
    Remove(usize),
}

fn edits() -> impl Strategy<Value = Vec<Edit>> {
    let e = prop_oneof![
        4 => (any::<usize>(), 0u8..8).prop_map(|(p, b)| Edit::Flip(p, b)),
        2 => (any::<usize>(), any::<u8>()).prop_map(|(p, v)| Edit::Set(p, v)),
        1 => any::<usize>().prop_map(Edit::Truncate),
        1 => proptest::collection::vec(any::<u8>(), 1..16).prop_map(Edit::Extend),
        1 => (any::<usize>(), any::<u8>()).prop_map(|(p, v)| Edit::Insert(p, v)),
        1 => any::<usize>().prop_map(Edit::Remove),
    ];
    proptest::collection::vec(e, 1..5)
}

/// Edits land from `keep` on: the NAL header stays, so the bent NAL still
/// reaches the parser it was taken for.
fn apply(mut b: Vec<u8>, es: &[Edit], keep: usize) -> Vec<u8> {
    for e in es {
        let n = b.len();
        if n <= keep {
            b.extend_from_slice(&[0xA5]);
            continue;
        }
        let at = |p: &usize| keep + p % (n - keep);
        match e {
            Edit::Flip(p, bit) => {
                let i = at(p);
                b[i] ^= 1 << bit
            }
            Edit::Set(p, v) => {
                let i = at(p);
                b[i] = *v
            }
            Edit::Truncate(p) => b.truncate(keep + p % (n - keep + 1)),
            Edit::Extend(x) => b.extend_from_slice(x),
            Edit::Insert(p, v) => b.insert(keep + p % (n - keep + 1), *v),
            Edit::Remove(p) => {
                b.remove(at(p));
            }
        }
    }
    b
}

struct Fixture {
    sps_nal: Vec<u8>,
    pps_nal: Vec<u8>,
    sps: Sps,
    pps: Pps,
    slices: Vec<Vec<u8>>,
    whole: Vec<u8>,
}

fn fixture() -> &'static Fixture {
    static F: OnceLock<Fixture> = OnceLock::new();
    F.get_or_init(|| {
        let p = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../third_party/mabur/tests/fixtures/slices4_1080p.h265");
        let b = std::fs::read(p).expect("fixture (git submodule update --init?)");
        let nals = split_nals(&b);
        let get = |ty: u8| {
            let v = nals.iter().find(|v| v.ty == ty).unwrap();
            b[v.off..v.off + v.len].to_vec()
        };
        let (sps_nal, pps_nal) = (get(33), get(34));
        let slices = nals
            .iter()
            .filter(|v| v.ty < 32)
            .map(|v| b[v.off..v.off + v.len].to_vec())
            .collect();
        Fixture {
            sps: parse_sps(&sps_nal).unwrap(),
            pps: parse_pps(&pps_nal).unwrap(),
            sps_nal,
            pps_nal,
            slices,
            whole: b,
        }
    })
}

/// Skip fills allocate per min-CB of the picture; a parsed SPS this large
/// is not a picture this GS can decode (see the size regression test).
fn sane(sps: &Sps) -> bool {
    u64::from(sps.width) * u64::from(sps.height) <= 8192 * 4320
}

/// ue(v) (9.2): len zeros, then v + 1 in len + 1 bits.
fn put_ue(w: &mut BitWriter, v: u32) {
    let x = u64::from(v) + 1;
    let len = 63 - x.leading_zeros();
    for _ in 0..len {
        w.put(false);
    }
    for i in (0..=len).rev() {
        w.put((x >> i) & 1 != 0);
    }
}

fn reader_walk(b: &[u8]) {
    let mut r = BitReader::new(b);
    for _ in 0..64 {
        let _ = r.ue();
        let _ = r.se();
        let _ = r.u(7);
        if r.bad() {
            break;
        }
    }
}

proptest! {
    #![proptest_config(cfg(512))]

    #[test]
    fn nal_walk_any_bytes(b in proptest::collection::vec(prop_oneof![3 => Just(0u8), 1 => Just(1u8), 2 => any::<u8>()], 0..600), from: usize) {
        let vs = split_nals(&b);
        for v in &vs {
            prop_assert!(v.len >= 1 && v.off + v.len <= b.len());
            prop_assert_eq!(v.ty, nal_type(b[v.off]));
        }
        if let Some((p, l)) = find_start_code(&b, from % (b.len() + 1)) {
            prop_assert!(p + l <= b.len());
        }
        let mut t = ParamTracker::default();
        t.feed(&b);
        let _ = (t.usable(), t.unsupported());
    }

    #[test]
    fn escape_round_trip(b in proptest::collection::vec(prop_oneof![Just(0u8), Just(3u8), any::<u8>()], 0..400)) {
        let e = escape(&b);
        prop_assert_eq!(unescape(&e), b);
        // No start-code emulation survives escaping.
        prop_assert!(find_start_code(&e, 0).is_none());
        reader_walk(&e);
    }

    #[test]
    fn bit_io_round_trip(vals in proptest::collection::vec((0u32..u32::MAX, any::<i32>(), any::<u32>(), 0u32..=32), 0..40)) {
        let mut w = BitWriter::default();
        for &(ue, se, u, n) in &vals {
            put_ue(&mut w, ue);
            // se(v) is ue of 2v-1 (v > 0) or -2v (v <= 0); the reader's
            // range is |v| < 2^31.
            let se = se.clamp(-i32::MAX, i32::MAX);
            let k = if se > 0 { 2 * se as u32 - 1 } else { 2 * se.unsigned_abs() };
            put_ue(&mut w, k);
            let u = if n == 32 { u } else { u & ((1u32 << n) - 1) };
            w.u(u, n);
        }
        let bits = w.bits();
        let mut r = BitReader::new(w.bytes());
        for &(ue, se, u, n) in &vals {
            prop_assert_eq!(r.ue(), ue);
            prop_assert_eq!(r.se(), se.clamp(-i32::MAX, i32::MAX));
            let u = if n == 32 { u } else { u & ((1u32 << n) - 1) };
            prop_assert_eq!(r.u(n), u);
        }
        prop_assert!(!r.bad());
        prop_assert_eq!(r.pos(), bits);
    }

    #[test]
    fn ps_any_bytes(b in proptest::collection::vec(any::<u8>(), 0..200)) {
        reader_walk(&b);
        if let Ok(s) = parse_sps(&b) {
            let _ = (s.pic_w_ctbs(), s.pic_h_ctbs(), s.addr_bits());
        }
        let _ = parse_pps(&b);
        let f = fixture();
        let _ = parse_slice_header(&b, &f.sps, &f.pps);
    }

    #[test]
    fn ps_mutated(es in edits(), which: bool) {
        let f = fixture();
        let base = if which { &f.sps_nal } else { &f.pps_nal };
        let bent = apply(base.clone(), &es, 2);
        if which {
            if let Ok(s) = parse_sps(&bent) {
                prop_assert!((4..=6).contains(&s.log2_ctb));
                if sane(&s) {
                    let _ = (s.pic_w_ctbs(), s.pic_h_ctbs(), s.addr_bits());
                    for nal in f.slices.iter().take(8) {
                        let _ = parse_slice_header(nal, &s, &f.pps);
                    }
                }
            }
        } else if let Ok(p) = parse_pps(&bent) {
            for nal in f.slices.iter().take(8) {
                let _ = parse_slice_header(nal, &f.sps, &p);
            }
        }
    }

    #[test]
    fn slice_header_mutated(i: usize, es in edits()) {
        let f = fixture();
        let base = &f.slices[i % f.slices.len()];
        // Headers are short: edit only the first bytes after the NAL header
        // most of the time, where the parse actually reads.
        let head = base.len().min(24);
        let mut bent = apply(base[..head].to_vec(), &es, 2);
        bent.extend_from_slice(&base[head..]);
        if let Ok(h) = parse_slice_header(&bent, &f.sps, &f.pps) {
            // An unedited rewrite reproduces the parsed bits, then the stop bit.
            let mut w = BitWriter::default();
            let e = HeaderEdit { first: h.first, address: h.address, clear_sao: false };
            write_slice_header(&mut w, &h, &e, &f.sps, &f.pps);
            let mut want = BitWriter::default();
            want.copy(&h.rbsp, 0, h.pos_end);
            want.put(true);
            want.align_zero();
            prop_assert_eq!(w.bytes(), want.bytes());
            prop_assert!(h.data_byte <= h.rbsp.len());
            let total = f.sps.pic_w_ctbs() * f.sps.pic_h_ctbs();
            let _ = make_skip_slice(&f.sps, &f.pps, &h, 0, total);
        }
    }

    #[test]
    fn skip_fill_parses_back(i: usize, a: u32, b: u32, clear_sao: bool, addr: u32) {
        let f = fixture();
        let total = f.sps.pic_w_ctbs() * f.sps.pic_h_ctbs();
        let h = parse_slice_header(&f.slices[i % f.slices.len()], &f.sps, &f.pps).unwrap();
        let (first, end) = (a % total, b % (total + 1));
        let _ = skip_fill_steps(&f.sps, h.max_num_merge_cand, first, end.max(first));
        let mut w = BitWriter::default();
        let e = HeaderEdit { first: addr % total == 0, address: addr % total, clear_sao };
        write_slice_header(&mut w, &h, &e, &f.sps, &f.pps);
        let Some(nal) = make_skip_slice(&f.sps, &f.pps, &h, first, end) else {
            prop_assert!(h.slice_type == 2 || first >= end);
            return Ok(());
        };
        let v = split_nals(&nal);
        prop_assert_eq!(v.len(), 1);
        let back = parse_slice_header(&nal[v[0].off..], &f.sps, &f.pps).unwrap();
        prop_assert_eq!(back.first, first == 0);
        if first != 0 {
            prop_assert_eq!(back.address, first);
        }
        prop_assert_eq!(back.slice_type, h.slice_type);
    }

    #[test]
    fn tracker_on_mutated_stream(es in edits(), from: usize, len in 0usize..4000) {
        let f = fixture();
        let a = from % f.whole.len();
        let piece = f.whole[a..(a + len).min(f.whole.len())].to_vec();
        let bent = apply(piece, &es, 0);
        let mut t = ParamTracker::default();
        t.feed(&bent);
        if t.usable() && sane(t.sps()) {
            for v in split_nals(&bent).iter().filter(|v| v.ty < 32) {
                let _ = parse_slice_header(&bent[v.off..v.off + v.len], t.sps(), t.pps());
            }
        }
    }
}

/// One syntax element of a synthetic parameter set or slice header.
#[derive(Clone, Debug)]
enum Tok {
    Bit(bool),
    Ue(u32),
    Se(i32),
}

/// Exp-Golomb values at the edges a header field can overflow on.
fn wide_ue() -> impl Strategy<Value = u32> {
    prop_oneof![
        6 => 0u32..16,
        1 => 0u32..u32::MAX,
        1 => (i32::MAX as u32 - 40)..(i32::MAX as u32 + 40),
        1 => (u32::MAX - 40)..u32::MAX,
    ]
}

fn wide_se() -> impl Strategy<Value = i32> {
    prop_oneof![
        6 => -16i32..16,
        1 => any::<i32>(),
        1 => (i32::MAX - 40)..=i32::MAX,
        1 => -i32::MAX..(-i32::MAX + 40),
    ]
}

fn toks() -> impl Strategy<Value = Vec<Tok>> {
    proptest::collection::vec(
        prop_oneof![
            4 => any::<bool>().prop_map(Tok::Bit),
            3 => wide_ue().prop_map(Tok::Ue),
            1 => wide_se().prop_map(Tok::Se),
        ],
        0..120,
    )
}

/// NAL header, then the tokens as an RBSP (stop bit, alignment), escaped.
fn synth(nal_hdr: [u8; 2], prefix: &[(u32, u32)], ts: &[Tok]) -> Vec<u8> {
    let mut w = BitWriter::default();
    for &(v, n) in prefix {
        w.u(v, n);
    }
    for t in ts {
        match *t {
            Tok::Bit(b) => w.put(b),
            Tok::Ue(v) => put_ue(&mut w, v),
            Tok::Se(v) => {
                let k = if v > 0 {
                    2 * v as u32 - 1
                } else {
                    2 * v.unsigned_abs()
                };
                put_ue(&mut w, k)
            }
        }
    }
    w.put(true);
    w.align_zero();
    let mut nal = nal_hdr.to_vec();
    nal.extend(escape(w.bytes()));
    nal
}

proptest! {
    #![proptest_config(cfg(1024))]

    /// Parameter sets and slice headers built from Exp-Golomb fields at
    /// their extremes (the values a random byte string almost never
    /// spells): no panic, and what parses stays usable.
    #[test]
    fn synthetic_headers(
        max_sub in 0u32..8,
        ptl in any::<[u32; 3]>(),
        sps_t in toks(),
        pps_t in toks(),
        sl_t in toks(),
        idr: bool,
    ) {
        let f = fixture();
        let sps_nal = synth([0x42, 0x01], &[(0, 4), (max_sub, 3), (1, 1), (ptl[0], 32), (ptl[1], 32), (ptl[2], 32)], &sps_t);
        let sps = parse_sps(&sps_nal);
        let pps = parse_pps(&synth([0x44, 0x01], &[], &pps_t));
        let sl_hdr = if idr { [0x26, 0x01] } else { [0x02, 0x01] };
        let sl = synth(sl_hdr, &[], &sl_t);
        let _ = parse_slice_header(&sl, &f.sps, &f.pps);
        // The fixture SPS with long-term pictures on: the slice header's
        // long-term loop is reachable only through one.
        let mut lt = f.sps.clone();
        lt.long_term = true;
        lt.num_lt_sps = max_sub * 4;
        lt.lt_used_sps = (0..lt.num_lt_sps).map(|i| (ptl[0] >> (i % 32)) as u8 & 1).collect();
        let _ = parse_slice_header(&sl, &lt, &f.pps);
        if let Ok(p) = &pps {
            let _ = parse_slice_header(&sl, &f.sps, p);
            for nal in f.slices.iter().take(4) {
                let _ = parse_slice_header(nal, &f.sps, p);
            }
        }
        if let Ok(s) = &sps {
            prop_assert!((4..=6).contains(&s.log2_ctb));
            if sane(s) {
                let _ = (s.pic_w_ctbs(), s.pic_h_ctbs(), s.addr_bits());
                let _ = parse_slice_header(&sl, s, &f.pps);
                if let Ok(p) = &pps {
                    let _ = parse_slice_header(&sl, s, p);
                }
            }
        }
    }
}
