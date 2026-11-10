//! FEC bench: the GF(256) kernel and the sliding-window decoder under body
//! loss. Self-timed with `Instant` and no files, so the same binary runs on
//! the host and from `/data/local/tmp` on an Android device:
//!
//!     cargo run --release -p pal-fec --example fec_bench [-- --quick]
//!     tools/android-build.sh build --release -p pal-fec --example fec_bench
//!
//! Geometry follows the shipped drone bundle: 332-byte symbols, 4 blocks per
//! body, base overhead 0.5, repair window 64 (frames widen theirs up to 128,
//! so 128 is measured too). Loss drops whole bodies, as a lost air frame does.
//! The stream is a simplified encoder: one repair per two sources over the
//! newest window, no frame-tail hold.

use pal_fec::gf256;
use pal_fec::{Boundary, SwConfig, SwDecoder};
use pal_wire::sw;
use std::hint::black_box;
use std::time::Instant;

const SYMBOL: usize = 332;
const BLOCKS_PER_BODY: usize = 4;
const OVERHEAD: f64 = 0.5;
const SEQ_HORIZON: u32 = 512;
const ARRIVAL_GUARD: u32 = 192;

struct XorShift(u64);

impl XorShift {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn fill(&mut self, b: &mut [u8]) {
        b.iter_mut().for_each(|x| *x = self.next() as u8);
    }
    fn chance(&mut self, p: f64) -> bool {
        ((self.next() >> 11) as f64 / (1u64 << 53) as f64) < p
    }
}

fn median(v: &mut [f64]) -> f64 {
    v.sort_by(|a, b| a.total_cmp(b));
    v[v.len() / 2]
}

fn pct(sorted: &[u64], p: f64) -> u64 {
    let i = ((sorted.len() as f64 - 1.0) * p).round() as usize;
    sorted[i]
}

/// Symbols per second for `lincomb` at one symbol size: one call per symbol,
/// cycling over a window's worth of rows so the data is not all in L1.
fn bench_lincomb(size: usize, runs: usize, calls: usize) {
    let mut rng = XorShift(0x9E37_79B9_7F4A_7C15);
    let rows = 64;
    let mut src = vec![0u8; rows * size];
    rng.fill(&mut src);
    let mut coeffs = [0u8; 64];
    rng.fill(&mut coeffs);
    coeffs.iter_mut().for_each(|c| *c |= 1);
    let mut acc = vec![0u8; size];
    let mut rates = Vec::with_capacity(runs);
    for _ in 0..runs {
        let t = Instant::now();
        for i in 0..calls {
            let r = i % rows;
            gf256::lincomb(&mut acc, &src[r * size..(r + 1) * size], coeffs[r]);
        }
        black_box(&acc);
        rates.push(calls as f64 / t.elapsed().as_secs_f64());
    }
    let sps = median(&mut rates);
    println!(
        "lincomb size={size} symbols_per_s={:.0} MB_per_s={:.1}",
        sps,
        sps * size as f64 / 1e6
    );
}

/// One repair over 64 rows of 332 bytes: what a window-64 repair costs the
/// encoder side, and the upper bound of one row reduction in the decoder.
fn bench_lincomb_rows(runs: usize, calls: usize) {
    let mut rng = XorShift(0xD1B5_4A32_D192_ED03);
    let n = 64;
    let mut src = vec![0u8; n * SYMBOL];
    rng.fill(&mut src);
    let rows: Vec<&[u8]> = src.chunks(SYMBOL).collect();
    let mut coeffs = vec![0u8; n];
    sw::repair_coeffs(1, &mut coeffs);
    let mut out = vec![0u8; SYMBOL];
    let mut per_call = Vec::with_capacity(runs);
    for _ in 0..runs {
        let t = Instant::now();
        for _ in 0..calls {
            gf256::lincomb_rows(&mut out, black_box(&rows), &coeffs);
        }
        black_box(&out);
        per_call.push(t.elapsed().as_secs_f64() * 1e6 / calls as f64);
    }
    println!(
        "lincomb_rows rows={n} size={SYMBOL} us_per_repair={:.2}",
        median(&mut per_call)
    );
}

/// The envelopes of `n_src` sources and their repairs, grouped into bodies
/// in emission order.
fn make_stream(n_src: usize, window: usize, seed: u64) -> Vec<Vec<Vec<u8>>> {
    let mut rng = XorShift(seed);
    let mut sources: Vec<Vec<u8>> = Vec::with_capacity(n_src);
    let mut envs: Vec<Vec<u8>> = Vec::new();
    let mut credit = 0.0;
    let mut key = 1u32;
    let mut coeffs = [0u8; sw::MAX_WINDOW];
    for seq in 0..n_src {
        // One app packet filling the symbol: u16 length, then random bytes
        // (zero bytes are a shortcut in the scalar kernel).
        let mut sym = vec![0u8; SYMBOL];
        let ln = SYMBOL - 2;
        sym[..2].copy_from_slice(&(ln as u16).to_le_bytes());
        rng.fill(&mut sym[2..]);
        let mut env = vec![0u8; sw::HEADER_LEN + SYMBOL];
        let h = sw::Header {
            repair: false,
            symbol_size: SYMBOL as u16,
            seq: seq as u32,
            window_len: 0,
            repair_key: 0,
        };
        h.pack((&mut env[..sw::HEADER_LEN]).try_into().unwrap());
        env[sw::HEADER_LEN..].copy_from_slice(&sym);
        envs.push(env);
        sources.push(sym);

        credit += OVERHEAD;
        while credit >= 1.0 {
            credit -= 1.0;
            let wl = window.min(sources.len());
            let ws = sources.len() - wl;
            sw::repair_coeffs(key, &mut coeffs[..wl]);
            let rows: Vec<&[u8]> = sources[ws..].iter().map(|s| s.as_slice()).collect();
            let mut env = vec![0u8; sw::HEADER_LEN + SYMBOL];
            let h = sw::Header {
                repair: true,
                symbol_size: SYMBOL as u16,
                seq: ws as u32,
                window_len: wl as u8,
                repair_key: key,
            };
            h.pack((&mut env[..sw::HEADER_LEN]).try_into().unwrap());
            gf256::lincomb_rows(&mut env[sw::HEADER_LEN..], &rows, &coeffs[..wl]);
            envs.push(env);
            key = key.wrapping_add(1);
        }
    }
    envs.chunks(BLOCKS_PER_BODY).map(|c| c.to_vec()).collect()
}

struct DecodeRun {
    body_ns: Vec<u64>,
    total_s: f64,
    symbols_in: usize,
    delivered: u64,
    recovered: u64,
    bodies_in: usize,
}

fn decode_once(bodies: &[Vec<Vec<u8>>], window: usize, loss: f64, seed: u64) -> DecodeRun {
    let mut rng = XorShift(seed);
    let keep: Vec<bool> = bodies.iter().map(|_| !rng.chance(loss)).collect();
    let cfg = SwConfig {
        symbol_size: SYMBOL,
        window,
        overhead: OVERHEAD,
    };
    let mut dec = SwDecoder::new(cfg, SEQ_HORIZON, ARRIVAL_GUARD);
    let mut body_ns = Vec::with_capacity(bodies.len());
    let mut symbols_in = 0;
    let start = Instant::now();
    for (i, (body, &k)) in bodies.iter().zip(&keep).enumerate() {
        if !k {
            continue;
        }
        let t = Instant::now();
        symbols_in += body.len();
        for env in body {
            let out = dec.add_symbol(env, i as u64, Boundary::None, true, false);
            black_box(out);
        }
        body_ns.push(t.elapsed().as_nanos() as u64);
    }
    let total_s = start.elapsed().as_secs_f64();
    let st = dec.stats();
    DecodeRun {
        bodies_in: body_ns.len(),
        body_ns,
        total_s,
        symbols_in,
        delivered: st.syms_delivered,
        recovered: st.syms_recovered,
    }
}

fn bench_decoder(window: usize, loss: f64, n_src: usize, runs: usize) {
    let bodies = make_stream(n_src, window, 0x2545_F491_4F6C_DD1D);
    let (mut p50, mut p99, mut p999, mut max, mut sps) = (vec![], vec![], vec![], vec![], vec![]);
    let mut last = None;
    for r in 0..runs {
        let mut run = decode_once(&bodies, window, loss, 0x5DEE_CE66_D000 + r as u64);
        run.body_ns.sort_unstable();
        p50.push(pct(&run.body_ns, 0.5) as f64 / 1e3);
        p99.push(pct(&run.body_ns, 0.99) as f64 / 1e3);
        p999.push(pct(&run.body_ns, 0.999) as f64 / 1e3);
        max.push(*run.body_ns.last().unwrap() as f64 / 1e3);
        sps.push(run.symbols_in as f64 / run.total_s);
        last = Some(run);
    }
    let run = last.unwrap();
    println!(
        "decode window={window} loss={:.0}% bodies={} kept={} sources={n_src} direct={} recovered={} lost={} \
         us_per_body p50={:.2} p99={:.2} p99.9={:.2} max={:.1} symbols_per_s={:.0}",
        loss * 100.0,
        bodies.len(),
        run.bodies_in,
        run.delivered,
        run.recovered,
        n_src as u64 - run.delivered - run.recovered,
        median(&mut p50),
        median(&mut p99),
        median(&mut p999),
        median(&mut max),
        median(&mut sps),
    );
}

fn main() {
    let quick = std::env::args().any(|a| a == "--quick");
    let (runs, calls, n_src) = if quick {
        (5, 20_000, 4_000)
    } else {
        (15, 200_000, 40_000)
    };
    println!(
        "fec_bench backend={} runs={runs} (each figure is the median over runs)",
        gf256::backend()
    );
    // 332: the video symbol ([fec] symbol_size). 1312: the flight-controller
    // OSD stream's symbol ([msp] symbol_size).
    for size in [SYMBOL, 1312] {
        bench_lincomb(size, runs, calls);
    }
    bench_lincomb_rows(runs, calls / 64);
    for window in [64, 128] {
        for loss in [0.0, 0.1, 0.2, 0.3] {
            bench_decoder(window, loss, n_src, runs);
        }
    }
}
