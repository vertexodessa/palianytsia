//! Property tests of the per-packet path: the Rust core (through `GsCore`,
//! as the replay gates drive it) fed a synthetic drone stream, built here
//! from mabur's 4-slice 1080p fixture the way maburd builds it (FrameHdr,
//! FRAG fragments, sliding-window FEC with repairs, SBI bodies), through a
//! channel that drops, duplicates, reorders, truncates and flips bits; plus
//! a hostile mix (RC frames of every type bent and re-CRC'd, calibration
//! and probe bodies, foreign cards, garbage). FrameStream is also driven
//! alone with arbitrary fragments, where its output protocol is visible.
//!
//! Run in the test profile, where overflow checks are on: a wrapping
//! subtraction on hostile input fails here although release builds would
//! carry on with a wrapped value. Deterministic (fixed seed, no failure
//! files); `PAL_PROP_SCALE=N` multiplies the case counts and
//! `PAL_PROP_SEED=S` picks another seed for a long local run.
use hevc::nal::split_nals;
use hevc::ps::parse_sps;
use hevc::slice::parse_slice_header;
use pal_core::config::{load_config, Config};
use pal_core::frame_stream::{AuLat, FragArrival, FrameOut, FrameStream, FrameStreamCfg};
use pal_core::rust_core::RustCore;
use pal_core::{Au, CoreIo, CoreOpts, GsCore, Mode, RxBody};
use pal_wire::crc16::crc16_ccitt;
use pal_wire::frame_wire::{FrameHdr, FLAG_DISCONT, FLAG_IDR, FRAG_HDR_LEN};
use pal_wire::rc::{self, TagCtx};
use pal_wire::{cal, probe, sbi, sw};
use proptest::prelude::*;
use proptest::test_runner::{Config as PropConfig, RngAlgorithm, RngSeed};
use std::collections::HashMap;
use std::path::Path;
use std::sync::OnceLock;

fn cfg(cases: u32) -> PropConfig {
    let env = |k: &str| std::env::var(k).ok().and_then(|v| v.parse::<u64>().ok());
    let scale = env("PAL_PROP_SCALE").unwrap_or(1).max(1);
    PropConfig {
        cases: (u64::from(cases) * scale).min(u64::from(u32::MAX)) as u32,
        failure_persistence: None,
        rng_algorithm: RngAlgorithm::ChaCha,
        rng_seed: RngSeed::Fixed(env("PAL_PROP_SEED").unwrap_or(0x9a1)),
        // A failing core run is long; shrinking it far is not worth the time.
        max_shrink_iters: 256,
        ..PropConfig::default()
    }
}

// ---- the drone ----

struct Gf {
    exp: [u8; 512],
    log: [u8; 256],
}

/// GF(2^8) over 0x11D, independent of the crate's tables.
fn gf() -> &'static Gf {
    static G: OnceLock<Gf> = OnceLock::new();
    G.get_or_init(|| {
        let mut g = Gf {
            exp: [0; 512],
            log: [0; 256],
        };
        let mut x = 1u16;
        for i in 0..255 {
            g.exp[i] = x as u8;
            g.log[x as usize] = i as u8;
            x <<= 1;
            if x & 0x100 != 0 {
                x ^= 0x11D;
            }
        }
        for i in 255..512 {
            g.exp[i] = g.exp[i - 255];
        }
        g
    })
}

fn gmul(a: u8, b: u8) -> u8 {
    if a == 0 || b == 0 {
        return 0;
    }
    let g = gf();
    g.exp[usize::from(g.log[usize::from(a)]) + usize::from(g.log[usize::from(b)])]
}

struct Fixture {
    /// (AU, IDR) in decode order: the IDR, then the P cycle.
    aus: Vec<(Vec<u8>, bool)>,
    slice_rows: u8,
}

/// mabur's fixture cut into AUs as `tools/slices-frames.py` cuts it.
fn fixture() -> &'static Fixture {
    static F: OnceLock<Fixture> = OnceLock::new();
    F.get_or_init(|| {
        let p = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../third_party/mabur/tests/fixtures/slices4_1080p.h265");
        let d = std::fs::read(p).expect("fixture (git submodule update --init?)");
        let nals = split_nals(&d);
        let mut cuts = Vec::new();
        let mut prev_ps = false;
        for v in &nals {
            // Back to the start code (3 or 4 bytes).
            let s = if v.off >= 4 && d[v.off - 4] == 0 {
                v.off - 4
            } else {
                v.off - 3
            };
            if v.ty >= 32 {
                if !prev_ps {
                    cuts.push(s);
                }
                prev_ps = true;
            } else {
                if d[v.off + 2] & 0x80 != 0 && !prev_ps {
                    cuts.push(s);
                }
                prev_ps = false;
            }
        }
        cuts.push(d.len());
        let aus: Vec<(Vec<u8>, bool)> = cuts
            .windows(2)
            .map(|w| {
                let au = d[w[0]..w[1]].to_vec();
                let idr = split_nals(&au).iter().any(|v| v.ty == 19 || v.ty == 20);
                (au, idr)
            })
            .collect();
        // The rows per slice the drone would stamp: the second slice's
        // address over the picture width in CTBs.
        let sps_v = nals.iter().find(|v| v.ty == 33).unwrap();
        let pps_v = nals.iter().find(|v| v.ty == 34).unwrap();
        let sps = parse_sps(&d[sps_v.off..sps_v.off + sps_v.len]).unwrap();
        let pps = hevc::ps::parse_pps(&d[pps_v.off..pps_v.off + pps_v.len]).unwrap();
        let second = nals
            .iter()
            .filter(|v| v.ty < 32)
            .map(|v| parse_slice_header(&d[v.off..v.off + v.len], &sps, &pps).unwrap())
            .find(|h| !h.first)
            .unwrap();
        Fixture {
            aus,
            slice_rows: (second.address / sps.pic_w_ctbs()) as u8,
        }
    })
}

fn config() -> &'static Config {
    static C: OnceLock<Config> = OnceLock::new();
    C.get_or_init(|| {
        load_config(
            &Path::new(env!("CARGO_MANIFEST_DIR")).join("../../configs/gs.toml"),
            None,
        )
        .unwrap()
    })
}

/// One UEP layer of maburd: fragments, sources, repairs, SBI bodies.
struct Layer {
    ss: usize,
    seq: u32,
    hist: Vec<Vec<u8>>,
    fseq: u16,
    packer: sbi::Packer,
    repair_every: usize,
    repair_win: usize,
    key: u32,
}

impl Layer {
    fn new(sid: u8, ss: usize, bpb: usize, seq0: u32, every: usize, win: usize) -> Self {
        Self {
            ss,
            seq: seq0,
            hist: Vec::new(),
            fseq: 0,
            packer: sbi::Packer::new(sw::HEADER_LEN + ss, bpb, sid),
            repair_every: every.max(1),
            repair_win: win.max(1),
            key: 0x5eed,
        }
    }

    fn env(&self, h: &sw::Header, payload: &[u8]) -> Vec<u8> {
        let mut w = [0u8; sw::HEADER_LEN];
        h.pack(&mut w);
        let mut e = w.to_vec();
        e.extend_from_slice(payload);
        e
    }

    fn push_env(&mut self, e: Vec<u8>, bodies: &mut Vec<Vec<u8>>) {
        bodies.extend(self.packer.add(&e));
    }

    fn source(&mut self, pkt: &[u8], bodies: &mut Vec<Vec<u8>>) {
        let mut sym = Vec::with_capacity(self.ss);
        sym.extend_from_slice(&(pkt.len() as u16).to_le_bytes());
        sym.extend_from_slice(pkt);
        sym.resize(self.ss, 0);
        let h = sw::Header {
            repair: false,
            symbol_size: self.ss as u16,
            seq: self.seq,
            window_len: 0,
            repair_key: 0,
        };
        let e = self.env(&h, &sym);
        self.push_env(e, bodies);
        self.hist.push(sym);
        if self.hist.len() > 255 {
            self.hist.remove(0);
        }
        self.seq = self.seq.wrapping_add(1);
        if self.seq.is_multiple_of(self.repair_every as u32) {
            self.repair(bodies);
        }
    }

    fn repair(&mut self, bodies: &mut Vec<Vec<u8>>) {
        let wl = self.repair_win.min(self.hist.len()).min(255);
        if wl == 0 {
            return;
        }
        self.key = self.key.wrapping_mul(0x9E37_79B9).wrapping_add(1);
        let mut c = vec![0u8; wl];
        sw::repair_coeffs(self.key, &mut c);
        let first = self.hist.len() - wl;
        let mut pay = vec![0u8; self.ss];
        for (i, &ci) in c.iter().enumerate() {
            for (a, &s) in pay.iter_mut().zip(&self.hist[first + i]) {
                *a ^= gmul(ci, s);
            }
        }
        let h = sw::Header {
            repair: true,
            symbol_size: self.ss as u16,
            seq: self.seq.wrapping_sub(wl as u32),
            window_len: wl as u8,
            repair_key: self.key,
        };
        let e = self.env(&h, &pay);
        self.push_env(e, bodies);
    }

    /// One frame: FrameHdr + AU cut into fixed fragments, then the frame
    /// tail's repairs and a flushed body (maburd closes a body per frame).
    fn frame(&mut self, hdr: &FrameHdr, au: &[u8], bodies: &mut Vec<Vec<u8>>) {
        let mut payload = hdr.pack().to_vec();
        payload.extend_from_slice(au);
        let chunk = self.ss - 2 - FRAG_HDR_LEN;
        let count = payload.len().div_ceil(chunk) as u16;
        for (i, c) in payload.chunks(chunk).enumerate() {
            let mut pkt = Vec::with_capacity(FRAG_HDR_LEN + c.len());
            pkt.extend_from_slice(&self.fseq.to_le_bytes());
            pkt.extend_from_slice(&(i as u16).to_le_bytes());
            pkt.extend_from_slice(&count.to_le_bytes());
            pkt.extend_from_slice(c);
            self.source(&pkt, bodies);
        }
        self.repair(bodies);
        bodies.extend(self.packer.flush());
        self.fseq = self.fseq.wrapping_add(1);
    }
}

#[derive(Clone, Debug)]
struct Stream {
    frames: usize,
    seq0: u32,
    frame_id0: u16,
    bpb: usize,
    every: usize,
    win: usize,
    sliced: bool,
    /// Frame index that re-bases with FLAG_DISCONT (a maburd restart).
    discont_at: Option<usize>,
}

fn stream() -> impl Strategy<Value = Stream> {
    (
        8usize..48,
        prop_oneof![Just(0u32), Just(u32::MAX - 30), any::<u32>()],
        prop_oneof![Just(0u16), Just(u16::MAX - 5), any::<u16>()],
        1usize..5,
        2usize..12,
        4usize..64,
        any::<bool>(),
        proptest::option::weighted(0.15, 1usize..48),
    )
        .prop_map(
            |(frames, seq0, frame_id0, bpb, every, win, sliced, discont_at)| Stream {
                frames,
                seq0,
                frame_id0,
                bpb,
                every,
                win,
                sliced,
                discont_at,
            },
        )
}

/// What each frame was: pts -> (frame index, AU).
type Truth = HashMap<u32, (usize, Vec<u8>)>;

/// Bodies on the air, and the truth behind them.
fn drone(s: &Stream) -> (Vec<Vec<u8>>, Truth) {
    let f = fixture();
    let ss = config().fec.symbol_size[0];
    let mut l = Layer::new(0, ss, s.bpb, s.seq0, s.every, s.win);
    let mut bodies = Vec::new();
    let mut truth = HashMap::new();
    let mut id = s.frame_id0;
    for i in 0..s.frames {
        // The IDR, then the P cycle (which re-sends the parameter sets).
        let (au, idr) = if i == 0 {
            &f.aus[0]
        } else {
            &f.aus[1 + (i - 1) % (f.aus.len() - 1)]
        };
        let mut flags = if *idr { FLAG_IDR } else { 0 };
        if s.discont_at == Some(i) {
            flags |= FLAG_DISCONT;
            id = id.wrapping_add(1000);
        }
        let pts = 1_000_000 + i as u32 * 16_667;
        let hdr = FrameHdr {
            frame_id: id,
            flags,
            slice_rows: if s.sliced { f.slice_rows } else { 0 },
            pts_us: pts,
        };
        l.frame(&hdr, au, &mut bodies);
        truth.insert(pts, (i, au.clone()));
        id = id.wrapping_add(1);
    }
    (bodies, truth)
}

// ---- the channel ----

#[derive(Clone, Debug)]
enum Hit {
    Clean,
    Drop,
    Dup,
    Swap,
    /// Cut at a fraction of the body, FCS bad.
    Truncate(u16),
    /// Up to three bit flips (CRC16 sees every such error), FCS bad.
    Flip(Vec<(usize, u8)>),
}

fn hit() -> impl Strategy<Value = Hit> {
    prop_oneof![
        70 => Just(Hit::Clean),
        10 => Just(Hit::Drop),
        3 => Just(Hit::Dup),
        5 => Just(Hit::Swap),
        4 => any::<u16>().prop_map(Hit::Truncate),
        8 => proptest::collection::vec((any::<usize>(), 0u8..8), 1..=3).prop_map(Hit::Flip),
    ]
}

fn rx(body: Vec<u8>, us: u64, crc_ok: bool) -> RxBody {
    RxBody {
        card_id: 0,
        mono_us: us,
        rssi: [70, 68],
        snr: [30, 28],
        evm: [-40, -38],
        phy_valid: true,
        crc_ok,
        mac_seq: (us / 300) as u16 & 0xFFF,
        tsfl: us as u32,
        mcs: 3,
        rx_channel: 144,
        body,
    }
}

fn channel(bodies: Vec<Vec<u8>>, hits: &[Hit]) -> Vec<(Vec<u8>, bool)> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < bodies.len() {
        let h = hits
            .get(i % hits.len().max(1))
            .cloned()
            .unwrap_or(Hit::Clean);
        let b = bodies[i].clone();
        match h {
            Hit::Clean => out.push((b, true)),
            Hit::Drop => {}
            Hit::Dup => {
                out.push((b.clone(), true));
                out.push((b, true));
            }
            Hit::Swap if i + 1 < bodies.len() => {
                out.push((bodies[i + 1].clone(), true));
                out.push((b, true));
                i += 1;
            }
            Hit::Swap => out.push((b, true)),
            Hit::Truncate(f) => {
                let n = b.len() * usize::from(f) / 65536;
                out.push((b[..n].to_vec(), false));
            }
            Hit::Flip(fs) => {
                let mut b = b;
                for (p, bit) in fs {
                    let n = b.len();
                    b[p % n] ^= 1 << bit;
                }
                out.push((b, false));
            }
        }
        i += 1;
    }
    out
}

#[derive(Default)]
struct Io {
    aus: Vec<Au>,
    sends: usize,
    mav: usize,
}

impl CoreIo for Io {
    fn on_au(&mut self, au: Au) {
        self.aus.push(au);
    }
    fn send(&mut self, rc_body: &[u8]) {
        assert!(!rc_body.is_empty());
        self.sends += 1;
    }
    fn on_mav(&mut self, _pkt: &[u8]) {
        self.mav += 1;
    }
}

fn core(mode: Mode, adaptive_gap: bool) -> RustCore {
    RustCore::new(
        config(),
        CoreOpts {
            mode,
            adaptive_gap,
            rz_nonce: pal_core::RZ_NONCE,
            channel: 0,
        },
    )
    .unwrap()
}

/// The replay loop: one body, one tick on its stamp; a final tick past the
/// gap timeout.
fn replay(c: &mut RustCore, air: &[(Vec<u8>, bool)], io: &mut Io, t0: u64, step: u64) -> u64 {
    let mut t = t0;
    for (b, ok) in air {
        t += step;
        let m = rx(b.clone(), t, *ok);
        c.on_rx(&m, io);
        c.tick(t, io);
    }
    t += 1_000_000;
    c.tick(t, io);
    t
}

/// What every AU stream out of the core must satisfy against the truth:
/// each AU is a frame the drone sent, at most once, frames of one producer
/// run (before or after the FLAG_DISCONT re-base) come out in send order,
/// and a complete AU is byte-identical to the frame. Across the re-base
/// the order is the header arrival order, by design.
fn check_aus(
    aus: &[Au],
    truth: &HashMap<u32, (usize, Vec<u8>)>,
    discont_at: Option<usize>,
) -> Result<(), TestCaseError> {
    // The re-basing frame itself lands wherever its header arrives.
    let run = |i: usize| match discont_at {
        Some(d) if i == d => 1,
        Some(d) if i > d => 2,
        _ => 0,
    };
    let mut last: [Option<usize>; 3] = [None, None, None];
    let mut seen = std::collections::HashSet::new();
    for a in aus {
        let Some((i, want)) = truth.get(&a.pts_us) else {
            return Err(TestCaseError::fail(format!(
                "AU with unknown pts {}",
                a.pts_us
            )));
        };
        prop_assert!(seen.insert(*i), "frame {} out twice", i);
        let r = run(*i);
        if let Some(l) = last[r] {
            prop_assert!(*i > l, "frame {} out after frame {}", i, l);
        }
        last[r] = Some(*i);
        prop_assert_eq!(a.sid, 0);
        if a.complete {
            prop_assert!(&a.data == want, "complete AU of frame {} differs", i);
            prop_assert!(!a.salvaged);
        }
    }
    Ok(())
}

proptest! {
    #![proptest_config(cfg(24))]

    /// A clean air delivers every frame complete, in order, bit-exact.
    #[test]
    fn clean_air_delivers_every_frame(s in stream(), gs: bool) {
        let (bodies, truth) = drone(&s);
        let air: Vec<_> = bodies.into_iter().map(|b| (b, true)).collect();
        let mode = if gs { Mode::Gs } else { Mode::Spotter };
        let mut c = core(mode, true);
        if gs {
            c.inject_disc_ack_for_replay(1_000_000);
            c.tick(1_000_000, &mut Io::default());
        }
        let mut io = Io::default();
        replay(&mut c, &air, &mut io, 1_000_000, 300);
        check_aus(&io.aus, &truth, s.discont_at)?;
        prop_assert_eq!(io.aus.len(), s.frames);
        prop_assert!(io.aus.iter().all(|a| a.complete));
    }

    /// Drops, duplicates, swaps, truncation and bit flips at realistic
    /// rates: no panic, and the AU contract above still holds.
    #[test]
    fn impaired_air_keeps_the_au_contract(
        s in stream(),
        hits in proptest::collection::vec(hit(), 1..200),
        gs: bool,
        adaptive: bool,
        step in 100u64..3000,
    ) {
        let (bodies, truth) = drone(&s);
        let air = channel(bodies, &hits);
        let mode = if gs { Mode::Gs } else { Mode::Spotter };
        let mut c = core(mode, adaptive);
        if gs {
            c.inject_disc_ack_for_replay(1_000_000);
            c.tick(1_000_000, &mut Io::default());
        }
        let mut io = Io::default();
        replay(&mut c, &air, &mut io, 1_000_000, step);
        check_aus(&io.aus, &truth, s.discont_at)?;
        prop_assert!(io.aus.len() <= s.frames);
        let _ = c.stats_json();
        let _ = c.link_op();
    }
}

// ---- hostile mix ----

#[derive(Clone, Debug)]
enum Extra {
    /// An RC frame of every type, bent; `fix` rewrites its CRC.
    Rc(u8, Vec<(usize, u8)>, usize, bool),
    Cal(u8, u8, i8, u16),
    Probe(u32, u8, u8),
    Garbage(Vec<u8>),
    /// A video body re-sent with its SBI stream byte changed.
    Restream(usize, u8),
    ForeignCard(usize, u8),
    /// A body with FCS good although bits flipped (a hostile transmitter
    /// that recomputes the per-block CRCs).
    Forged(usize, Vec<(usize, u8)>),
    Clock(i64),
}

fn extra() -> impl Strategy<Value = Extra> {
    let flips = proptest::collection::vec((any::<usize>(), any::<u8>()), 0..4);
    prop_oneof![
        6 => (0u8..8, flips.clone(), any::<usize>(), any::<bool>())
            .prop_map(|(t, f, cut, fix)| Extra::Rc(t, f, cut, fix)),
        2 => (1u8..=3, 0u8..9, any::<i8>(), 12u16..1500)
            .prop_map(|(k, r, i, l)| Extra::Cal(k, r, i, l)),
        2 => (any::<u32>(), any::<u8>(), 1u8..40).prop_map(|(s, p, n)| Extra::Probe(s, p, n)),
        2 => proptest::collection::vec(any::<u8>(), 0..400).prop_map(Extra::Garbage),
        2 => (any::<usize>(), any::<u8>()).prop_map(|(i, s)| Extra::Restream(i, s)),
        1 => (any::<usize>(), any::<u8>()).prop_map(|(i, c)| Extra::ForeignCard(i, c)),
        2 => (any::<usize>(), proptest::collection::vec((any::<usize>(), 0u8..8), 1..8))
            .prop_map(|(i, f)| Extra::Forged(i, f)),
        1 => (-5_000i64..3_000_000).prop_map(Extra::Clock),
    ]
}

fn recrc(mut b: Vec<u8>) -> Vec<u8> {
    if b.len() >= 2 {
        let n = b.len() - 2;
        let c = crc16_ccitt(&b[..n]);
        b[n..].copy_from_slice(&c.to_le_bytes());
    }
    b
}

/// Rewrites every whole SBI sub-block CRC: the body passes all checks.
fn forge(mut b: Vec<u8>, bp: usize) -> Vec<u8> {
    let stride = 2 + bp;
    let mut o = sbi::HDR_LEN;
    while o + stride <= b.len() {
        let c = crc16_ccitt(&b[o + 2..o + stride]);
        b[o..o + 2].copy_from_slice(&c.to_le_bytes());
        o += stride;
    }
    b
}

fn rc_frame(t: u8, n: u32) -> Vec<u8> {
    let key = config().link.key;
    let ctx = TagCtx {
        vrx_nonce: pal_core::RZ_NONCE,
        vtx_nonce: 1,
        seq32: n,
    };
    match t {
        0 => rc::pack_telem(&rc::Telem {
            tlm_seq: n as u16,
            state: 2,
            flags: 0x08,
            rcf_seq_echo: n as u16,
            ..rc::Telem::default()
        }),
        1 => rc::pack_disc_ack(&rc::DiscAck {
            vrx_nonce: pal_core::RZ_NONCE,
            vtx_nonce: 1 + (n & 1),
            chip_caps: rc::CAP_FRAME_WIRE | rc::CAP_TELEMETRY | rc::CAP_MAVLINK,
            agreed_channel: 144,
            seq: n as u16,
            ..rc::DiscAck::default()
        }),
        2 => {
            let mut x = rc::Nack {
                counter: n,
                n: 2,
                ..Default::default()
            };
            x.e[0].first_seq = n;
            x.e[1].first_seq = n.wrapping_add(40);
            rc::pack_nack(&x, &key, &ctx)
        }
        3 => {
            // The drone's tunnel: a param value and a calibration ack.
            let mut payload = pal_wire::mavlink::pack_v2(
                pal_wire::mavlink::MSG_PARAM_EXT_VALUE,
                1,
                1,
                n as u8,
                &[n as u8; 149],
            );
            payload.extend(cal::encode(
                &cal::CalMsg::Ack(cal::SweepAck::default()),
                1,
                1,
                n as u8,
            ));
            payload.truncate(rc::MAV_MAX_PAYLOAD);
            rc::pack_mav(
                &rc::Mav {
                    counter: n,
                    payload,
                },
                false,
                &key,
                &ctx,
            )
        }
        4 => rc::pack_rcf(&rc::Rcf::default(), &key, &ctx),
        5 => rc::pack_disc(&rc::Disc::default(), &key),
        6 => {
            let mut b = rc::pack_telem(&rc::Telem::default());
            b[2] = rc::RC_VERSION.wrapping_add(1);
            b
        }
        _ => rc::pack_mav(
            &rc::Mav {
                counter: n,
                payload: vec![0xFD; 30],
            },
            true,
            &key,
            &ctx,
        ),
    }
}

proptest! {
    #![proptest_config(cfg(24))]

    /// Video plus every other body type, bent and forged, interleaved on a
    /// jittery clock, in both modes: no panic.
    #[test]
    fn hostile_mix_never_panics(
        s in stream(),
        hits in proptest::collection::vec(hit(), 1..100),
        extras in proptest::collection::vec((any::<usize>(), extra()), 0..120),
        gs: bool,
        adaptive: bool,
    ) {
        let (bodies, _) = drone(&s);
        let video = bodies.clone();
        let mut air: Vec<(Vec<u8>, bool, u8, i64)> =
            channel(bodies, &hits).into_iter().map(|(b, ok)| (b, ok, 0, 0)).collect();
        let bp = sw::HEADER_LEN + config().fec.symbol_size[0];
        for (k, (at, e)) in extras.into_iter().enumerate() {
            let at = at % (air.len() + 1);
            let pick = |i: usize| video[i % video.len()].clone();
            let item = match e {
                Extra::Rc(t, flips, cut, fix) => {
                    let mut b = rc_frame(t, k as u32);
                    for (p, v) in flips {
                        let n = b.len();
                        b[p % n] ^= v;
                    }
                    if cut % 4 == 0 {
                        b.truncate(cut % (b.len() + 1));
                    }
                    (if fix { recrc(b) } else { b }, true, 0, 0)
                }
                Extra::Cal(kind, rate, idx, len) => (
                    cal::build_payload(
                        cal::Kind::from_u8(kind).unwrap(),
                        rate,
                        idx,
                        20,
                        0,
                        k as u32,
                        usize::from(len),
                    ),
                    rate % 2 == 0,
                    0,
                    0,
                ),
                Extra::Probe(seq, prof, n) => {
                    let pbp = sw::HEADER_LEN + config().fec.symbol_size[1];
                    let mut blk = vec![0u8; pbp.max(probe::HDR_LEN)];
                    blk[..2].copy_from_slice(&probe::MAGIC.to_le_bytes());
                    blk[2..6].copy_from_slice(&seq.to_le_bytes());
                    blk[6] = prof;
                    let mut p = sbi::Packer::new(blk.len(), usize::from(n), sbi::PROBE_STREAM_ID);
                    let mut b = Vec::new();
                    for _ in 0..n {
                        b.extend(p.add(&blk));
                    }
                    (b.into_iter().next().unwrap_or_default(), true, 0, 0)
                }
                Extra::Garbage(g) => (g, k % 2 == 0, 0, 0),
                Extra::Restream(i, sid) => {
                    let mut b = pick(i);
                    b[3] = sid;
                    (b, k % 3 == 0, 0, 0)
                }
                Extra::ForeignCard(i, card) => (pick(i), true, card.max(1), 0),
                Extra::Forged(i, flips) => {
                    let mut b = pick(i);
                    for (p, bit) in flips {
                        let n = b.len();
                        b[p % n] ^= 1 << bit;
                    }
                    (forge(b, bp), true, 0, 0)
                }
                Extra::Clock(dt) => (Vec::new(), false, 0xFF, dt),
            };
            air.insert(at, item);
        }
        let mode = if gs { Mode::Gs } else { Mode::Spotter };
        let mut c = core(mode, adaptive);
        let mut io = Io::default();
        if gs {
            c.inject_disc_ack_for_replay(1_000_000);
        }
        let mut t = 1_000_000u64;
        for (b, ok, card, dt) in air {
            if card == 0xFF {
                // A clock event: a tick far ahead, or a body stamped behind.
                t = t.saturating_add_signed(dt);
                c.tick(t, &mut io);
                continue;
            }
            t += 400;
            let mut m = rx(b, t, ok);
            m.card_id = card;
            c.on_rx(&m, &mut io);
            c.tick(t, &mut io);
        }
        c.tick(t + 5_000_000, &mut io);
        prop_assert!(io.aus.len() <= 4 * s.frames + 64);
        let _ = c.stats_json();
        let _ = c.params_json(t);
    }
}

// ---- FrameStream alone ----

/// Records the FrameOut protocol: begin, data, end strictly alternating.
#[derive(Default)]
struct Proto {
    open: bool,
    begins: u64,
    ends: u64,
    bytes: u64,
    err: Option<String>,
}

impl FrameOut for Proto {
    fn begin(&mut self, _hdr: &FrameHdr, _sid: u8, _nslices: u8) {
        if self.open {
            self.err.get_or_insert("begin inside an open AU".into());
        }
        self.open = true;
        self.begins += 1;
    }
    fn data(&mut self, b: &[u8]) {
        if !self.open {
            self.err.get_or_insert("data outside an AU".into());
        }
        self.bytes += b.len() as u64;
    }
    fn end(&mut self, _complete: bool, _lat: &AuLat) {
        if !self.open {
            self.err.get_or_insert("end without begin".into());
        }
        self.open = false;
        self.ends += 1;
    }
}

#[derive(Clone, Debug)]
enum FsOp {
    Frag {
        sid: u8,
        fseq: u16,
        idx: u16,
        count: u16,
        hdr: Option<(u16, u8, u8)>,
        len: usize,
    },
    Raw(u8, Vec<u8>),
    Poll(u64),
    Reset,
    Gap(u8, u64, u32),
}

fn fs_op() -> impl Strategy<Value = FsOp> {
    let small = prop_oneof![4 => 0u16..8, 1 => any::<u16>()];
    prop_oneof![
        12 => (0u8..3, small.clone(), small.clone(), small, proptest::option::of((
            prop_oneof![4 => 0u16..16, 1 => any::<u16>()],
            prop_oneof![Just(0u8), Just(FLAG_IDR), Just(FLAG_DISCONT), any::<u8>()],
            prop_oneof![Just(0u8), 1u8..6, any::<u8>()],
        )), 0usize..400)
            .prop_map(|(sid, fseq, idx, count, hdr, len)| FsOp::Frag { sid, fseq, idx, count, hdr, len }),
        1 => (0u8..3, proptest::collection::vec(any::<u8>(), 0..40)).prop_map(|(s, b)| FsOp::Raw(s, b)),
        3 => (0u64..200).prop_map(FsOp::Poll),
        1 => Just(FsOp::Reset),
        1 => (0u8..3, 0u64..300, 0u32..5000).prop_map(|(s, g, u)| FsOp::Gap(s, g, u)),
    ]
}

/// Feeds `ops` to a fresh FrameStream. `distinct_ids`: every (sid, fseq)
/// slot carries its own frame id, as maburd's do (frame_id is global across
/// the layers); otherwise ids are anything, AU bytes read as a header
/// included. Returns the output protocol record.
fn run_frame_stream(ops: Vec<FsOp>, au: &[u8], distinct_ids: bool) -> Result<Proto, TestCaseError> {
    let mut fs = FrameStream::new(FrameStreamCfg::default());
    let mut out = Proto::default();
    let mut now = 1000u64;
    let mut last = *fs.stats();
    for op in ops {
        match op {
            FsOp::Frag {
                sid,
                fseq,
                idx,
                count,
                hdr,
                len,
            } => {
                let (fseq, hdr) = if distinct_ids {
                    let fseq = fseq % 16;
                    let id = u16::from(sid) * 16 + fseq;
                    let hdr = hdr.or((idx == 0).then_some((0, 0, 0)));
                    (fseq, hdr.map(|(_, f, r)| (id, f, r)))
                } else {
                    (fseq, hdr)
                };
                let mut p = Vec::new();
                p.extend_from_slice(&fseq.to_le_bytes());
                p.extend_from_slice(&idx.to_le_bytes());
                p.extend_from_slice(&count.to_le_bytes());
                if let Some((id, flags, rows)) = hdr {
                    let h = FrameHdr {
                        frame_id: id,
                        flags,
                        slice_rows: rows,
                        pts_us: 7,
                    };
                    p.extend_from_slice(&h.pack());
                }
                let from = (usize::from(idx) * 300) % au.len();
                p.extend_from_slice(&au[from..(from + len).min(au.len())]);
                let arr = FragArrival {
                    body_mono_us: now * 1000,
                    sw_seq: Some(u32::from(idx)),
                    ..FragArrival::default()
                };
                fs.push_fragment(sid, &p, now, &arr, &mut out);
            }
            FsOp::Raw(_, _) if distinct_ids => {}
            FsOp::Raw(sid, b) => fs.push_fragment(sid, &b, now, &FragArrival::default(), &mut out),
            FsOp::Poll(dt) => {
                now += dt;
                fs.poll(now, &mut out);
            }
            FsOp::Reset => fs.reset(&mut out),
            FsOp::Gap(sid, g, us) => {
                fs.set_gap_timeout(usize::from(sid), g);
                fs.set_gap_frame_span(usize::from(sid), us, 15, 100);
            }
        }
        let st = *fs.stats();
        prop_assert!(st.clean >= last.clean && st.truncated >= last.truncated);
        prop_assert!(st.dropped >= last.dropped && st.bad_frags >= last.bad_frags);
        last = st;
        for s in 0..3 {
            let _ = fs.tail_view(s);
        }
    }
    now += 10_000;
    fs.poll(now, &mut out);
    // reset() closes begun AUs without counting them.
    let st = fs.stats();
    if distinct_ids {
        prop_assert!(st.clean + st.truncated <= out.ends);
    }
    Ok(out)
}

proptest! {
    #![proptest_config(cfg(256))]

    /// Fragments of maburd-shaped slots (one frame id per slot) in any
    /// order, counts, indices and timing, frame ids jumping and walking
    /// back, fixture AUs with salvage on: begin, data and end strictly
    /// alternate, the counters never step back.
    #[test]
    fn frame_stream_protocol_holds(ops in proptest::collection::vec(fs_op(), 1..300), seed_au: usize) {
        let f = fixture();
        let out = run_frame_stream(ops, &f.aus[seed_au % f.aus.len()].0, true)?;
        if let Some(e) = out.err {
            return Err(TestCaseError::fail(e));
        }
    }

    /// Anything at all, two slots claiming one frame id included: no panic,
    /// and begin, data and end still alternate.
    #[test]
    fn frame_stream_any_fragments(ops in proptest::collection::vec(fs_op(), 1..300), seed_au: usize) {
        let f = fixture();
        let out = run_frame_stream(ops, &f.aus[seed_au % f.aus.len()].0, false)?;
        if let Some(e) = out.err {
            return Err(TestCaseError::fail(e));
        }
    }
}

/// A body kept with a bad FCS has its sub-blocks salvaged by their own CRC,
/// and the SBI header that names the stream has none. One bit error in the
/// stream byte would move every CRC-good block of the body to stream 1,
/// whose decoder and FrameStream would emit an AU the drone never sent
/// there; the aggregator routes an FCS-bad body only to a stream the card
/// heard FCS-good recently. The flip lands on every fifth body.
#[test]
fn stream_byte_flip_on_a_bad_fcs_body() {
    let s = Stream {
        frames: 32,
        seq0: 0,
        frame_id0: 0,
        bpb: 4,
        every: 3,
        win: 6,
        sliced: false,
        discont_at: None,
    };
    let (bodies, _) = drone(&s);
    let air: Vec<_> = bodies
        .into_iter()
        .enumerate()
        .map(|(i, mut b)| {
            if i % 5 == 4 {
                b[3] ^= 1;
                (b, false)
            } else {
                (b, true)
            }
        })
        .collect();
    let mut c = core(Mode::Spotter, true);
    let mut io = Io::default();
    replay(&mut c, &air, &mut io, 1_000_000, 300);
    let on_1 = io.aus.iter().filter(|a| a.sid != 0).count();
    assert_eq!(
        on_1,
        0,
        "{} of {} AUs came out on another stream",
        on_1,
        io.aus.len()
    );
}
