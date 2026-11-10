//! Property tests of the sliding-window decoder: an honest drone stream
//! (sources plus repairs over the trailing window) through a channel that
//! drops, duplicates and reorders, and a hostile one with arbitrary headers,
//! windows and keys. The encoder here is independent of the crate's GF code
//! (shift-and-add over 0x11D), so a recovered packet that matches its source
//! checks the decoder's arithmetic as well as its bookkeeping.
//!
//! Run in the test profile, where overflow checks are on: a wrapping
//! subtraction on hostile input fails here although release builds would
//! carry on with a wrapped value. Deterministic (fixed seed, no failure
//! files); `PAL_PROP_SCALE=N` multiplies the case counts and
//! `PAL_PROP_SEED=S` picks another seed for a long local run.
use pal_fec::{Boundary, SourceState, SwConfig, SwDecoder};
use pal_wire::sw;
use proptest::prelude::*;
use proptest::test_runner::{Config, RngAlgorithm, RngSeed};
use std::collections::{HashMap, HashSet};

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

fn gmul(mut a: u8, mut b: u8) -> u8 {
    let mut p = 0u8;
    while b != 0 {
        if b & 1 != 0 {
            p ^= a;
        }
        let hi = a & 0x80 != 0;
        a <<= 1;
        if hi {
            a ^= 0x1D;
        }
        b >>= 1;
    }
    p
}

fn header(h: &sw::Header) -> Vec<u8> {
    let mut w = [0u8; sw::HEADER_LEN];
    h.pack(&mut w);
    w.to_vec()
}

/// The drone side: sources numbered from `start` (wrapping), each one or two
/// length-prefixed packets; repairs over the last `wl` sources.
struct Drone {
    ss: usize,
    start: u32,
    syms: Vec<Vec<u8>>,
    pkts: Vec<Vec<Vec<u8>>>,
}

impl Drone {
    fn source(&mut self, lens: (usize, usize), fill: u8) -> Vec<u8> {
        let i = self.syms.len();
        let seq = self.start.wrapping_add(i as u32);
        let mut sym = Vec::with_capacity(self.ss);
        let mut pkts = Vec::new();
        for (k, l) in [lens.0, lens.1].into_iter().enumerate() {
            let room = self.ss - sym.len();
            if l == 0 || room < 3 {
                break;
            }
            let l = l.min(room - 2);
            let mut p: Vec<u8> = (0..l)
                .map(|j| fill.wrapping_add((j * 7 + k * 13) as u8))
                .collect();
            for (d, s) in p.iter_mut().zip(seq.to_le_bytes()) {
                *d = s;
            }
            sym.extend_from_slice(&(l as u16).to_le_bytes());
            sym.extend_from_slice(&p);
            pkts.push(p);
        }
        sym.resize(self.ss, 0);
        let h = sw::Header {
            repair: false,
            symbol_size: self.ss as u16,
            seq,
            window_len: 0,
            repair_key: 0,
        };
        let mut env = header(&h);
        env.extend_from_slice(&sym);
        self.syms.push(sym);
        self.pkts.push(pkts);
        env
    }

    fn repair(&self, wl: usize, key: u32) -> Option<Vec<u8>> {
        let n = self.syms.len();
        let wl = wl.min(n).min(sw::MAX_WINDOW);
        if wl == 0 {
            return None;
        }
        let ws = n - wl;
        let mut c = vec![0u8; wl];
        sw::repair_coeffs(key, &mut c);
        let mut pay = vec![0u8; self.ss];
        for (i, &ci) in c.iter().enumerate() {
            for (a, &s) in pay.iter_mut().zip(&self.syms[ws + i]) {
                *a ^= gmul(ci, s);
            }
        }
        let h = sw::Header {
            repair: true,
            symbol_size: self.ss as u16,
            seq: self.start.wrapping_add(ws as u32),
            window_len: wl as u8,
            repair_key: key,
        };
        let mut env = header(&h);
        env.extend_from_slice(&pay);
        Some(env)
    }
}

/// One source: packet lengths, fill, then the repairs after it.
type Item = ((usize, usize), u8, Vec<(usize, u32)>);

#[derive(Clone, Debug)]
struct Plan {
    ss: usize,
    start: u32,
    horizon: u32,
    items: Vec<Item>,
    /// Channel: per envelope, (drop, dup, swap with the next).
    chan: Vec<(bool, bool, bool)>,
}

fn plan() -> impl Strategy<Value = Plan> {
    let starts = prop_oneof![
        Just(0u32),
        Just(u32::MAX - 40),
        Just(i32::MAX as u32 - 20),
        any::<u32>()
    ];
    (16usize..160, starts, prop_oneof![Just(0u32), 8u32..600]).prop_flat_map(
        |(ss, start, horizon)| {
            let item = (
                (0usize..200, 0usize..80),
                any::<u8>(),
                proptest::collection::vec((1usize..64, any::<u32>()), 0..3),
            );
            (
                Just(ss),
                Just(start),
                Just(horizon),
                proptest::collection::vec(item, 1..300),
                proptest::collection::vec(
                    (
                        proptest::bool::weighted(0.2),
                        proptest::bool::weighted(0.05),
                        proptest::bool::weighted(0.1),
                    ),
                    0..900,
                ),
            )
                .prop_map(|(ss, start, horizon, items, chan)| Plan {
                    ss,
                    start,
                    horizon,
                    items,
                    chan,
                })
        },
    )
}

fn decoder(ss: usize, horizon: u32) -> SwDecoder {
    SwDecoder::new(
        SwConfig {
            symbol_size: ss,
            window: 128,
            overhead: 0.5,
        },
        horizon,
        0,
    )
}

fn horizon_of(h: u32) -> usize {
    if h == 0 {
        4 * 128
    } else {
        h as usize
    }
}

proptest! {
    #![proptest_config(cfg(128))]

    /// Every packet out is the packet the drone put in that seq, no seq
    /// comes out twice, and the repair rows stay inside the horizon.
    #[test]
    fn honest_stream_recovers_only_the_truth(p in plan()) {
        let mut d = Drone { ss: p.ss, start: p.start, syms: Vec::new(), pkts: Vec::new() };
        let mut air = Vec::new();
        for (lens, fill, reps) in &p.items {
            air.push(d.source(*lens, *fill));
            for &(wl, key) in reps {
                air.extend(d.repair(wl, key));
            }
        }
        let mut sent = Vec::new();
        let mut i = 0;
        while i < air.len() {
            let (drop, dup, swap) = p.chan.get(i).copied().unwrap_or_default();
            if swap && i + 1 < air.len() {
                sent.push(air[i + 1].clone());
                if !drop {
                    sent.push(air[i].clone());
                }
                i += 2;
                continue;
            }
            if !drop {
                sent.push(air[i].clone());
                if dup {
                    sent.push(air[i].clone());
                }
            }
            i += 1;
        }
        let by_seq: HashMap<u32, usize> = (0..d.syms.len())
            .map(|i| (p.start.wrapping_add(i as u32), i))
            .collect();
        let mut dec = decoder(p.ss, p.horizon);
        let mut out_seqs = HashSet::new();
        let mut packets = 0u64;
        for (t, env) in sent.iter().enumerate() {
            let pkts = dec.add_symbol(env, t as u64, Boundary::None, true, false);
            let seqs = dec.last_out_seqs().to_vec();
            prop_assert_eq!(seqs.len(), pkts.len());
            let mut prev = None;
            for (pkt, &s) in pkts.iter().zip(&seqs) {
                let src = by_seq[&s];
                prop_assert!(d.pkts[src].contains(pkt), "seq {} packet differs from its source", s);
                if prev != Some(s) {
                    prop_assert!(out_seqs.insert(s), "seq {} delivered twice", s);
                }
                prev = Some(s);
            }
            packets += pkts.len() as u64;
            prop_assert_eq!(dec.stats().packets_out, packets);
            prop_assert!(dec.rows_in_flight() <= horizon_of(p.horizon) + 1);
            prop_assert_eq!(dec.stats().resets, 0);
        }
        // Nothing lost on the way: every source with a packet came out.
        if p.chan.iter().take(air.len()).all(|c| !c.0) {
            for (i, pk) in d.pkts.iter().enumerate() {
                if !pk.is_empty() {
                    prop_assert!(out_seqs.contains(&p.start.wrapping_add(i as u32)));
                }
            }
        }
        let _ = dec.missing_sources(256);
        let _ = dec.take_episodes();
    }
}

#[derive(Clone, Debug)]
enum Op {
    /// A well-formed header with arbitrary fields and payload bytes.
    Env {
        repair: bool,
        seq: u32,
        wl: u8,
        key: u32,
        fill: u8,
        size_ok: bool,
    },
    Raw(Vec<u8>),
    Transition,
    Close,
    Expire(u64),
    Query(u32, u32),
}

fn op() -> impl Strategy<Value = Op> {
    // Seqs mostly near each other, some anywhere (an encoder restart), some
    // across the u32 wrap, and a few at the reset span, where one envelope
    // makes the decoder walk a million missing seqs (slow, so rare here).
    let near = prop_oneof![
        60 => 0u32..600,
        10 => any::<u32>(),
        10 => (u32::MAX - 300)..=u32::MAX,
        1 => (1u32 << 20) - 50..(1u32 << 20) + 50,
    ];
    prop_oneof![
        12 => (any::<bool>(), near, any::<u8>(), any::<u32>(), any::<u8>(), proptest::bool::weighted(0.95))
            .prop_map(|(repair, seq, wl, key, fill, size_ok)| Op::Env { repair, seq, wl, key, fill, size_ok }),
        1 => proptest::collection::vec(any::<u8>(), 0..120).prop_map(Op::Raw),
        1 => Just(Op::Transition),
        1 => Just(Op::Close),
        1 => (0u64..3000).prop_map(Op::Expire),
        1 => (any::<u32>(), 0u32..1000).prop_map(|(a, b)| Op::Query(a, b)),
    ]
}

proptest! {
    #![proptest_config(cfg(128))]

    /// Arbitrary seqs, windows, keys, boundaries and clocks: no panic, and
    /// the bookkeeping stays inside its bounds.
    #[test]
    fn hostile_stream_never_panics(
        ss in 8usize..64,
        horizon in prop_oneof![Just(0u32), 1u32..700],
        ops in proptest::collection::vec(
            (op(), any::<bool>(), any::<bool>(), 0u8..3, 0u64..50),
            1..400,
        ),
    ) {
        let mut dec = decoder(ss, horizon);
        let mut now = 1000u64;
        for (o, clean, retx, b, dt) in ops {
            // Card stamps can step back a little; the clock otherwise grows.
            now = (now + dt).saturating_sub(if dt == 0 { 3 } else { 0 });
            let b = [Boundary::None, Boundary::Pre, Boundary::Post][usize::from(b)];
            match o {
                Op::Env { repair, seq, wl, key, fill, size_ok } => {
                    let h = sw::Header {
                        repair,
                        symbol_size: if size_ok { ss as u16 } else { ss as u16 + 1 },
                        seq,
                        window_len: if repair { wl.max(1) } else { 0 },
                        repair_key: if repair { key } else { 0 },
                    };
                    let mut env = header(&h);
                    env.extend((0..ss).map(|i| fill.wrapping_mul(i as u8 | 1)));
                    let pkts = dec.add_symbol(&env, now, b, clean, retx);
                    prop_assert_eq!(dec.last_out_seqs().len(), pkts.len());
                    prop_assert!(pkts.iter().all(|p| !p.is_empty() && p.len() + 2 <= ss));
                }
                Op::Raw(r) => {
                    let _ = dec.add_symbol(&r, now, b, clean, retx);
                }
                Op::Transition => dec.mark_transition(),
                Op::Close => dec.close_boundary(),
                Op::Expire(ms) => {
                    let _ = dec.expire_rows_older_than(ms, now);
                }
                Op::Query(s, lb) => {
                    let st = dec.source_state(s);
                    prop_assert!(matches!(
                        st,
                        SourceState::Unknown | SourceState::Direct | SourceState::Recovered
                            | SourceState::Retx | SourceState::BelowFloor
                    ));
                    prop_assert!(dec.missing_sources(lb).len() <= lb as usize);
                }
            }
            prop_assert!(dec.rows_in_flight() <= horizon_of(horizon) + 1);
            let _ = (dec.repair_window(), dec.repair_window_recent(), dec.newest_seq());
        }
        prop_assert!(dec.take_episodes().len() <= pal_fec::decoder::MAX_QUEUED_EPISODES);
    }
}
