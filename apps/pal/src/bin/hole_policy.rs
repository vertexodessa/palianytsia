//! What reaches the glass when an AU arrives with a hole (pal-pzj.11 step 1).
//!
//! Runs the Rust core over a capture twice, loss-free and with `--drop-pct`
//! of the bodies dropped (the same splitmix64 draw as `pal replay`), and
//! decodes the lossy AUs three ways, each against the loss-free decode:
//!   prefix  every AU as the core emits it (today);
//!   drop    an incomplete AU is not fed;
//!   skip    an incomplete non-IRAP AU becomes a whole-frame skip picture
//!           (every CTU skip, merge_idx 0), or is dropped when a donor rule
//!           fails.
//! "Incomplete" means `!complete && !salvaged` (the AU salvage could not
//! rebuild); `--salvaged-too` applies drop and skip to salvaged AUs as well.
//! A slot whose AU is not fed, or yields no frame, shows the last frame
//! again, as the glass does. Slots are the loss-free run's AUs, matched by
//! pts_us.
//!
//!   cargo run --release -p pal --bin hole_policy -- <capture.bin> -c configs/gs.toml \
//!       --mode gs --drop-pct 10 [--seed 0] [--salvaged-too] [--out DIR] [--probe]
//!
//! `--out DIR` writes each policy's fed stream as DIR/<policy>.h265 (for a
//! conformance decode with `ffmpeg -err_detect explode`).
use decode::{AvDecoder, Planes};
use flightlog::capture::{self, Record};
use hevc::nal::split_nals;
use hevc::skip_pic::{Params, SkipMaker};
use pal_core::rust_core::RustCore;
use pal_core::{Au, CoreIo, CoreOpts, GsCore, Mode, RZ_NONCE};
use std::collections::HashMap;
use std::io::{BufReader, Write};
use std::path::PathBuf;

struct Opts {
    capture: PathBuf,
    config: PathBuf,
    mode: Mode,
    drop_pct: u32,
    seed: u64,
    salvaged_too: bool,
    out: Option<PathBuf>,
    probe: bool,
    skip_every: Option<usize>,
}

fn die(msg: &str) -> ! {
    eprintln!("hole_policy: {msg}");
    std::process::exit(2)
}

fn opts() -> Opts {
    let mut a = std::env::args().skip(1);
    let (mut capture, mut config) = (None, None);
    let mut o = Opts {
        capture: PathBuf::new(),
        config: PathBuf::new(),
        mode: Mode::Gs,
        drop_pct: 0,
        seed: 0,
        salvaged_too: false,
        out: None,
        probe: false,
        skip_every: None,
    };
    while let Some(s) = a.next() {
        let mut val = || {
            a.next()
                .unwrap_or_else(|| die(&format!("{s} needs a value")))
        };
        match s.as_str() {
            "-c" => config = Some(PathBuf::from(val())),
            "--mode" => {
                o.mode = match val().as_str() {
                    "gs" => Mode::Gs,
                    "spotter" => Mode::Spotter,
                    m => die(&format!("--mode gs|spotter, got {m}")),
                }
            }
            "--drop-pct" => o.drop_pct = val().parse().unwrap_or_else(|_| die("--drop-pct")),
            "--seed" => o.seed = val().parse().unwrap_or_else(|_| die("--seed")),
            "--out" => o.out = Some(PathBuf::from(val())),
            "--salvaged-too" => o.salvaged_too = true,
            "--probe" => o.probe = true,
            "--skip-every" => {
                o.skip_every = Some(val().parse().unwrap_or_else(|_| die("--skip-every")))
            }
            f if f.starts_with('-') => die(&format!("unknown flag {f}")),
            _ => capture = Some(PathBuf::from(s)),
        }
    }
    o.capture = capture.unwrap_or_else(|| die("need <capture.bin>"));
    o.config = config.unwrap_or_else(|| die("-c <gs.toml> is required"));
    o
}

#[derive(Default)]
struct Collect(Vec<Au>);

impl CoreIo for Collect {
    fn on_au(&mut self, au: Au) {
        self.0.push(au);
    }
    fn send(&mut self, _rc_body: &[u8]) {}
}

/// The core's AUs for the capture with `drop_pct` % of bodies dropped: the
/// draw `pal replay --drop-pct` makes, so the runs match it.
fn run_core(o: &Opts, drop_pct: u32) -> Vec<Au> {
    let cfg =
        pal_core::config::load_config(&o.config, None).unwrap_or_else(|e| die(&e.to_string()));
    let mut core = RustCore::new(
        &cfg,
        CoreOpts {
            mode: o.mode,
            adaptive_gap: false,
            rz_nonce: RZ_NONCE,
            channel: 0,
        },
    )
    .unwrap_or_else(|e| die(&e));
    let f = std::fs::File::open(&o.capture).unwrap_or_else(|e| die(&e.to_string()));
    let r = capture::Reader::new(BufReader::with_capacity(1 << 20, f))
        .unwrap_or_else(|e| die(&e.to_string()));
    let mut io = Collect::default();
    let mut rng = o.seed.wrapping_add(pal_wire::sw::SPLITMIX_GAMMA);
    for rec in r {
        let rec = match rec {
            Ok(rec) => rec,
            Err(e) if capture::is_torn_tail(&e) => break,
            Err(e) => die(&format!("capture: {e}")),
        };
        match rec {
            Record::Body(b) => {
                if drop_pct > 0 && (pal_wire::sw::splitmix64(&mut rng) % 100) < u64::from(drop_pct)
                {
                    continue;
                }
                core.on_rx(&b, &mut io);
            }
            Record::Tick(t) => core.tick(t, &mut io),
        }
    }
    io.0
}

fn is_irap(au: &Au) -> bool {
    (16..=23).contains(&au.vcl_nal) || au.flags & 1 != 0
}

fn probe(aus: &[Au]) {
    let mut ps = Params::default();
    for (i, a) in aus.iter().enumerate() {
        ps.feed(&a.data, a.complete);
        let types: Vec<u8> = split_nals(&a.data).iter().map(|v| v.ty).collect();
        println!(
            "{i} pts={} sid={} flags={:#x} vcl={} complete={} salvaged={} skipped_before={} len={} nals={types:?} hdr={:?}",
            a.pts_us,
            a.sid,
            a.flags,
            a.vcl_nal,
            a.complete,
            a.salvaged,
            a.skipped_before,
            a.data.len(),
            ps.hdr(&a.data)
        );
    }
    if ps.pt.usable() {
        println!(
            "sps={:?}\npps={:?}\nrps_sets={:?}",
            ps.pt.sps(),
            ps.pt.pps(),
            ps.sets
        );
    }
}

/// Y PSNR in dB (capped at 100 for identical planes) and the MSE.
fn psnr(a: &[u8], b: &[u8]) -> (f64, f64) {
    let se: u64 = a
        .iter()
        .zip(b)
        .map(|(x, y)| {
            let d = i32::from(*x) - i32::from(*y);
            (d * d) as u64
        })
        .sum();
    let mse = se as f64 / a.len() as f64;
    (db(mse), mse)
}

fn db(mse: f64) -> f64 {
    if mse == 0.0 {
        100.0
    } else {
        (10.0 * (255.0 * 255.0 / mse).log10()).min(100.0)
    }
}

const POLICIES: [&str; 6] = [
    "prefix",
    "drop",
    "skip",
    "skip+missing",
    "prefix+fill",
    "skip+fill",
];

/// The skip picture for `au`, counting it or its fallback under `tag`.
fn try_skip(
    (id, sk): (&str, &mut SkipMaker),
    au: &Au,
    frame_us: u32,
    fallback: &mut HashMap<String, u32>,
    tag: &str,
) -> Option<Vec<u8>> {
    let layer = if is_irap(au) {
        "irap"
    } else if au.sid == 0 {
        "base"
    } else {
        "enh"
    };
    let r = if is_irap(au) {
        Err("irap")
    } else {
        sk.make(&au.data, au.pts_us, frame_us)
    };
    let key = match r {
        Ok(_) => format!("{id}/{tag}/made/{layer}"),
        Err(why) => format!("{id}/{tag}/{why}/{layer}"),
    };
    *fallback.entry(key).or_default() += 1;
    r.ok()
}

struct Policy {
    dec: AvDecoder,
    shown: Option<Vec<u8>>,
    /// Per slot; NaN before the reference shows a frame.
    psnr: Vec<f64>,
    mse: Vec<f64>,
    dump: Option<std::fs::File>,
    fed: u64,
}

fn percentile(v: &[f64], p: f64) -> f64 {
    let mut s = v.to_vec();
    s.sort_by(f64::total_cmp);
    if s.is_empty() {
        return f64::NAN;
    }
    s[((s.len() - 1) as f64 * p).round() as usize]
}

/// Damage events: one opens at a slot whose input differs from the
/// loss-free run's and closes at the first slot (that one included) whose
/// PSNR is `thr` or above; damage inside an open event extends it. Returns
/// each closed event's length in ms of pts and the count still open at
/// the end of the capture.
fn heal(psnr: &[f64], damaged: &[bool], pts: &[u32], thr: f64) -> (Vec<f64>, u32) {
    let mut out = Vec::new();
    let mut open: Option<usize> = None;
    for (i, (&p, &d)) in psnr.iter().zip(damaged).enumerate() {
        if p.is_nan() {
            continue;
        }
        let good = p >= thr;
        match open {
            None if d && good => out.push(0.0),
            None if d => open = Some(i),
            Some(s) if good => {
                out.push(f64::from(pts[i].wrapping_sub(pts[s])) / 1000.0);
                open = None;
            }
            _ => {}
        }
    }
    (out, u32::from(open.is_some()))
}

/// For each slot in `holes`: ms of pts until the first slot (that one
/// included) whose PSNR is `thr` or above; None when that never comes.
fn heal_from(psnr: &[f64], holes: &[usize], pts: &[u32], thr: f64) -> (Vec<f64>, u32) {
    let mut out = Vec::new();
    let mut never = 0;
    for &s in holes {
        match (s..psnr.len()).find(|&j| psnr[j] >= thr) {
            Some(j) => out.push(f64::from(pts[j].wrapping_sub(pts[s])) / 1000.0),
            None => never += 1,
        }
    }
    (out, never)
}

/// The loss-free stream with every `every`-th complete base P picture
/// replaced by a skip picture, written to OUT/conformance.h265: the skip
/// syntax checked by a decoder with no loss around it.
fn conformance(o: &Opts, reference: &[Au], every: usize) {
    let d = o
        .out
        .as_ref()
        .unwrap_or_else(|| die("--skip-every needs --out DIR"));
    std::fs::create_dir_all(d).unwrap_or_else(|e| die(&e.to_string()));
    let path = d.join("conformance.h265");
    let mut w = std::io::BufWriter::new(
        std::fs::File::create(&path).unwrap_or_else(|e| die(&e.to_string())),
    );
    let frame_us = 1;
    let mut sk = SkipMaker::default();
    let (mut base, mut made) = (0usize, 0u32);
    let mut why: HashMap<&'static str, u32> = HashMap::new();
    for a in reference {
        let mut out = a.data.clone();
        let mut skip = false;
        if a.complete && a.sid == 0 && !is_irap(a) {
            base += 1;
            if base % every == 0 {
                // Its own header gives the POC; the rest is the donor's.
                match sk.make(&a.data, a.pts_us, frame_us) {
                    Ok(nal) => {
                        out = nal;
                        skip = true;
                        made += 1;
                    }
                    Err(e) => *why.entry(e).or_default() += 1,
                }
            }
        }
        w.write_all(&out).unwrap();
        sk.fed(&out, a.complete && !skip, a.complete || skip, a.pts_us);
    }
    println!(
        "conformance {} skips={made} not_made={why:?} aus={}",
        path.display(),
        reference.len()
    );
}

fn main() {
    let o = opts();
    if std::env::var_os("PAL_FILL_REF").is_some() {
        // Where the loss-free run itself lost a base AU on the live air:
        // skips SkipFill makes there, and the slots to the next IRAP.
        let reference = run_core(&o, 0);
        let mut f = runtime::skip_fill::SkipFill::default();
        for (i, r) in reference.iter().enumerate() {
            for (pts, _) in f.feed(r).before {
                let next = reference[i..]
                    .iter()
                    .position(is_irap)
                    .unwrap_or(reference.len() - i);
                println!("ref_fill slot={i} pts={pts} slots_to_next_irap={next}");
            }
        }
        println!("ref_fill made={} refused={}", f.made, f.refused);
        return;
    }
    let reference = run_core(&o, 0);
    if o.probe {
        let aus = if o.drop_pct > 0 {
            run_core(&o, o.drop_pct)
        } else {
            reference
        };
        probe(&aus);
        return;
    }
    if let Some(every) = o.skip_every {
        conformance(&o, &reference, every);
        return;
    }
    let lossy = run_core(&o, o.drop_pct);
    let slot_of: HashMap<u32, usize> = reference
        .iter()
        .enumerate()
        .map(|(i, a)| (a.pts_us, i))
        .collect();
    let mut at: Vec<Option<&Au>> = vec![None; reference.len()];
    let mut stray = 0;
    for a in &lossy {
        match slot_of.get(&a.pts_us) {
            Some(&i) if at[i].is_none() => at[i] = Some(a),
            _ => stray += 1,
        }
    }
    let pts: Vec<u32> = reference.iter().map(|a| a.pts_us).collect();
    let frame_us = {
        let mut d: Vec<u32> = pts
            .windows(2)
            .map(|w| w[1].wrapping_sub(w[0]))
            .filter(|d| *d > 0 && *d < 1_000_000)
            .collect();
        d.sort_unstable();
        d.get(d.len() / 2).copied().unwrap_or(16_667).max(1)
    };
    if let Some(d) = &o.out {
        std::fs::create_dir_all(d).unwrap_or_else(|e| die(&e.to_string()));
    }
    let mut refdec = AvDecoder::open(0).unwrap_or_else(|e| die(&e));
    let mut pol: Vec<Policy> = POLICIES
        .iter()
        .map(|name| Policy {
            dec: AvDecoder::open(0).unwrap_or_else(|e| die(&e)),
            shown: None,
            psnr: Vec::new(),
            mse: Vec::new(),
            dump: o.out.as_ref().map(|d| {
                std::fs::File::create(d.join(format!("{name}.h265")))
                    .unwrap_or_else(|e| die(&e.to_string()))
            }),
            fed: 0,
        })
        .collect();
    let mut skippers = [SkipMaker::default(), SkipMaker::default()];
    let mut fill = runtime::skip_fill::SkipFill::default();
    let mut fill5 = runtime::skip_fill::SkipFill::new(true);
    let mut fallback: HashMap<String, u32> = HashMap::new();
    let mut n = HashMap::<&str, u32>::new();
    let mut ref_shown: Option<Vec<u8>> = None;
    let mut planes = Planes::default();
    let mut damaged_slot = vec![false; reference.len()];
    let mut holes = Vec::new();
    for (i, r) in reference.iter().enumerate() {
        if refdec.decode(&r.data) > 0 && refdec.planes(&mut planes) {
            ref_shown = Some(planes.y.clone());
        }
        let a = at[i];
        let hole = a.is_some_and(|a| !a.complete && (o.salvaged_too || !a.salvaged));
        match a {
            None => *n.entry("missing").or_default() += 1,
            Some(a) if !a.complete => {
                *n.entry("incomplete").or_default() += 1;
                if a.salvaged {
                    *n.entry("salvaged").or_default() += 1;
                }
                if hole {
                    *n.entry("policy_aus").or_default() += 1;
                    let layer = if is_irap(a) {
                        "irap"
                    } else if a.sid == 0 {
                        "base"
                    } else {
                        "enh"
                    };
                    *n.entry(layer).or_default() += 1;
                }
            }
            _ => {}
        }
        if hole {
            holes.push(i);
        }
        damaged_slot[i] = a.is_none_or(|a| !a.complete || a.data != r.data);
        // A whole AU the core never emitted, as the skip+missing policy sees
        // it: the loss-free run's pts and layer (the GS would infer them
        // from the frame count and the POC parity), no bytes.
        let ghost = Au {
            data: Vec::new(),
            ..r.clone()
        };
        let feeds: [Option<Vec<u8>>; 6] = match a {
            None if is_irap(r) => [None, None, None, None, None, None],
            None => [
                None,
                None,
                None,
                try_skip(
                    ("skip+missing", &mut skippers[1]),
                    &ghost,
                    frame_us,
                    &mut fallback,
                    "missing",
                ),
                None,
                None,
            ],
            Some(a) if !hole => [
                Some(a.data.clone()),
                Some(a.data.clone()),
                Some(a.data.clone()),
                Some(a.data.clone()),
                Some(a.data.clone()),
                Some(a.data.clone()),
            ],
            Some(a) => [
                Some(a.data.clone()),
                None,
                try_skip(
                    ("skip", &mut skippers[0]),
                    a,
                    frame_us,
                    &mut fallback,
                    "hole",
                ),
                try_skip(
                    ("skip+missing", &mut skippers[1]),
                    a,
                    frame_us,
                    &mut fallback,
                    "hole",
                ),
                Some(a.data.clone()),
                Some(a.data.clone()),
            ],
        };
        // prefix+fill: what the runtime does (runtime::skip_fill), from the
        // lossy AUs alone: skip pictures for base AUs it infers were lost
        // go in right before the AU that shows the gap.
        let mut feeds = feeds;
        if let Some(a) = a {
            let fd5 = fill5.feed(a);
            let p5 = &mut pol[5];
            for (_, sp) in fd5.before {
                p5.fed += 1;
                if let Some(w) = p5.dump.as_mut() {
                    w.write_all(&sp).unwrap();
                }
                if p5.dec.decode(&sp) > 0 && p5.dec.planes(&mut planes) {
                    p5.shown = Some(planes.y.clone());
                }
            }
            if let Some(x) = fd5.instead {
                feeds[5] = Some(x);
            }
            let p = &mut pol[4];
            for (_, sp) in fill.feed(a).before {
                p.fed += 1;
                if let Some(w) = p.dump.as_mut() {
                    w.write_all(&sp).unwrap();
                }
                if p.dec.decode(&sp) > 0 && p.dec.planes(&mut planes) {
                    p.shown = Some(planes.y.clone());
                }
            }
        }
        for (k, p) in pol.iter_mut().enumerate() {
            if let Some(f) = &feeds[k] {
                p.fed += 1;
                if let Some(w) = p.dump.as_mut() {
                    w.write_all(f).unwrap();
                }
                if p.dec.decode(f) > 0 && p.dec.planes(&mut planes) {
                    p.shown = Some(planes.y.clone());
                }
                if k == 2 || k == 3 {
                    let complete = a.is_some_and(|a| a.complete);
                    // Fed whole, salvaged (its header is the original's) or
                    // a skip picture: a donor candidate.
                    let whole = feeds[1].is_none() || a.is_some_and(|a| a.complete || a.salvaged);
                    skippers[k - 2].fed(f, complete, whole, r.pts_us);
                }
            }
            match (&ref_shown, &p.shown) {
                (None, _) => {
                    p.psnr.push(f64::NAN);
                    p.mse.push(f64::NAN);
                }
                (Some(rs), Some(ps)) if rs.len() == ps.len() => {
                    let (d, mse) = psnr(rs, ps);
                    p.psnr.push(d);
                    p.mse.push(mse);
                }
                // Nothing on the glass yet, or another size: a dark slot,
                // counted apart (it is the same in every policy).
                _ => {
                    p.psnr.push(f64::NEG_INFINITY);
                    p.mse.push(f64::NAN);
                }
            }
        }
    }
    let g = |k: &str| n.get(k).copied().unwrap_or(0);
    println!(
        "capture={} drop_pct={} seed={} salvaged_too={} ref_aus={} ref_incomplete={} lossy_aus={} stray={stray} missing={} incomplete={} salvaged={} policy_aus={} (irap {} base {} enh {}) literal_rule_fail={} frame_us={frame_us}",
        o.capture.display(),
        o.drop_pct,
        o.seed,
        o.salvaged_too,
        reference.len(),
        reference.iter().filter(|a| !a.complete).count(),
        lossy.len(),
        g("missing"),
        g("incomplete"),
        g("salvaged"),
        g("policy_aus"),
        g("irap"),
        g("base"),
        g("enh"),
        skippers[0].literal_fail,
    );
    let mut fb: Vec<_> = fallback.into_iter().collect();
    fb.sort();
    println!("skip_fallback {fb:?}");
    println!("prefix+fill made={} refused={}", fill.made, fill.refused);
    println!(
        "skip+fill made={} refused={} replaced={}",
        fill5.made, fill5.refused, fill5.replaced
    );
    let differ: Vec<usize> = (0..reference.len())
        .filter(|&i| {
            let a = pol[0].psnr[i];
            pol.iter().any(|p| p.psnr[i].to_bits() != a.to_bits())
        })
        .collect();
    println!(
        "slots={} differ_slots={} policy_holes={}",
        reference.len(),
        differ.len(),
        holes.len()
    );
    if let Some(d) = &o.out {
        // One row per slot: what arrived and each policy's PSNR.
        let mut w = std::io::BufWriter::new(
            std::fs::File::create(d.join("slots.tsv")).unwrap_or_else(|e| die(&e.to_string())),
        );
        writeln!(
            w,
            "slot\tpts\tsid\tirap\tarrived\tprefix\tdrop\tskip\tskip_missing"
        )
        .unwrap();
        for (i, r) in reference.iter().enumerate() {
            let arrived = match at[i] {
                None => "missing",
                Some(a) if a.complete => "complete",
                Some(a) if a.salvaged => "salvaged",
                Some(_) => "truncated",
            };
            writeln!(
                w,
                "{i}\t{}\t{}\t{}\t{arrived}\t{:.2}\t{:.2}\t{:.2}\t{:.2}",
                r.pts_us,
                r.sid,
                u8::from(is_irap(r)),
                pol[0].psnr[i],
                pol[1].psnr[i],
                pol[2].psnr[i],
                pol[3].psnr[i]
            )
            .unwrap();
        }
    }
    let max = |h: &[f64]| h.iter().copied().fold(f64::NAN, f64::max);
    for (k, p) in pol.iter().enumerate() {
        let v: Vec<f64> = p.psnr.iter().copied().filter(|x| x.is_finite()).collect();
        let dark = p.psnr.iter().filter(|x| **x == f64::NEG_INFINITY).count();
        let m: Vec<f64> = p.mse.iter().copied().filter(|x| !x.is_nan()).collect();
        let mean = v.iter().sum::<f64>() / v.len().max(1) as f64;
        let mean_mse = m.iter().sum::<f64>() / m.len().max(1) as f64;
        let below30 = v.iter().filter(|x| **x < 30.0).count();
        let dv: Vec<f64> = differ
            .iter()
            .map(|&i| p.psnr[i])
            .filter(|x| x.is_finite())
            .collect();
        let dmean = dv.iter().sum::<f64>() / dv.len().max(1) as f64;
        let dbelow30 = dv.iter().filter(|x| **x < 30.0).count();
        let (h40, open40) = heal(&p.psnr, &damaged_slot, &pts, 40.0);
        let (hh40, never40) = heal_from(&p.psnr, &holes, &pts, 40.0);
        let (hhx, neverx) = heal_from(&p.psnr, &holes, &pts, 100.0);
        let (f, e) = p.dec.counters();
        println!(
            "policy={} fed={} frames={f} dec_errors={e} shown_slots={} dark={dark} mean_psnr={mean:.2} psnr_of_mean_mse={:.2} p10={:.2} below30={below30} | differ: mean={dmean:.2} p10={:.2} below30={dbelow30} | hole_heal40_ms(n={} median={:.0} max={:.0} never={never40}) hole_heal_exact_ms(n={} median={:.0} max={:.0} never={neverx}) | any_damage_heal40_ms(events={} median={:.0} max={:.0} open={open40})",
            POLICIES[k],
            p.fed,
            v.len(),
            db(mean_mse),
            percentile(&v, 0.10),
            percentile(&dv, 0.10),
            hh40.len(),
            percentile(&hh40, 0.5),
            max(&hh40),
            hhx.len(),
            percentile(&hhx, 0.5),
            max(&hhx),
            h40.len(),
            percentile(&h40, 0.5),
            max(&h40),
        );
    }
}
