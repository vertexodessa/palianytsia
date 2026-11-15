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
use hevc::bits::{ceil_log2, unescape, BitReader};
use hevc::nal::split_nals;
use hevc::ps::{ParamTracker, Pps, Sps};
use hevc::skip_slice::make_skip_slice;
use hevc::slice::parse_slice_header;
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

/// A short-term RPS: the delta POCs it keeps (negative first), used or not.
type RpsSet = Vec<i32>;

/// st_ref_pic_set(idx) with the deltas derived (H.265 7.4.8). `sets` are
/// the SPS's sets before `idx`; `in_slice` for the one a slice header
/// codes (idx == num_short_term_ref_pic_sets). Some((negative, positive))
/// unless the syntax is bad.
fn read_st_rps(
    r: &mut BitReader,
    idx: usize,
    in_slice: bool,
    sets: &[(Vec<i32>, Vec<i32>)],
) -> Option<(Vec<i32>, Vec<i32>)> {
    let inter = idx != 0 && r.flag();
    if inter {
        let delta_idx = if in_slice { r.ue() as usize + 1 } else { 1 };
        let rf = idx.checked_sub(delta_idx)?;
        let (rneg, rpos) = sets.get(rf)?;
        let sign = r.u(1);
        let delta_rps = (1 - 2 * sign as i32) * (r.ue() as i32 + 1);
        let n = rneg.len() + rpos.len();
        let mut use_delta = vec![true; n + 1];
        for u in use_delta.iter_mut() {
            if !r.flag() {
                *u = r.flag();
            }
        }
        let (mut neg, mut pos) = (Vec::new(), Vec::new());
        for j in (0..rpos.len()).rev() {
            let d = rpos[j] + delta_rps;
            if d < 0 && use_delta[rneg.len() + j] {
                neg.push(d);
            }
        }
        if delta_rps < 0 && use_delta[n] {
            neg.push(delta_rps);
        }
        for (j, &rn) in rneg.iter().enumerate() {
            let d = rn + delta_rps;
            if d < 0 && use_delta[j] {
                neg.push(d);
            }
        }
        for j in (0..rneg.len()).rev() {
            let d = rneg[j] + delta_rps;
            if d > 0 && use_delta[j] {
                pos.push(d);
            }
        }
        if delta_rps > 0 && use_delta[n] {
            pos.push(delta_rps);
        }
        for (j, &rp) in rpos.iter().enumerate() {
            let d = rp + delta_rps;
            if d > 0 && use_delta[rneg.len() + j] {
                pos.push(d);
            }
        }
        (!r.bad()).then_some((neg, pos))
    } else {
        let nneg = r.ue();
        let npos = r.ue();
        if nneg > 16 || npos > 16 {
            return None;
        }
        let (mut neg, mut pos) = (Vec::new(), Vec::new());
        let mut poc = 0i32;
        for _ in 0..nneg {
            poc -= r.ue() as i32 + 1;
            r.u(1);
            neg.push(poc);
        }
        poc = 0;
        for _ in 0..npos {
            poc += r.ue() as i32 + 1;
            r.u(1);
            pos.push(poc);
        }
        (!r.bad()).then_some((neg, pos))
    }
}

/// The SPS's short-term RPS sets with their deltas (hevc::ps keeps only
/// the counts). `nal` from its 2-byte header, escaped.
fn sps_rps_sets(nal: &[u8]) -> Option<Vec<(Vec<i32>, Vec<i32>)>> {
    let rb = unescape(nal.get(2..)?);
    let mut r = BitReader::new(&rb);
    r.u(4);
    let max_sub = r.u(3) as usize;
    r.u(1);
    // profile_tier_level(1, max_sub_layers_minus1): 88 bits + level
    r.u(32);
    r.u(32);
    r.u(32);
    let mut flags = vec![(0, 0); max_sub];
    for f in flags.iter_mut() {
        *f = (r.u(1), r.u(1));
    }
    if max_sub > 0 {
        for _ in max_sub..8 {
            r.u(2);
        }
    }
    for (pp, lp) in flags {
        if pp != 0 {
            r.u(32);
            r.u(32);
            r.u(24);
        }
        if lp != 0 {
            r.u(8);
        }
    }
    r.ue(); // sps_id
    let chroma = r.ue();
    if chroma == 3 {
        r.u(1);
    }
    r.ue();
    r.ue();
    if r.flag() {
        for _ in 0..4 {
            r.ue();
        }
    }
    r.ue();
    r.ue();
    r.ue(); // log2_max_poc_lsb_minus4
    let sub_info = r.flag();
    let start = if sub_info { 0 } else { max_sub };
    for _ in start..=max_sub {
        r.ue();
        r.ue();
        r.ue();
    }
    for _ in 0..6 {
        r.ue();
    }
    if r.flag() && r.flag() {
        return None; // scaling list data: not walked
    }
    r.u(1); // amp
    r.u(1); // sao
    if r.flag() {
        r.u(4);
        r.u(4);
        r.ue();
        r.ue();
        r.u(1);
    }
    let n = r.ue() as usize;
    if n > 64 {
        return None;
    }
    let mut sets = Vec::with_capacity(n);
    for i in 0..n {
        let s = read_st_rps(&mut r, i, false, &sets)?;
        sets.push(s);
    }
    (!r.bad()).then_some(sets)
}

/// The part of a first slice's header the skip rules look at.
#[derive(Clone, Debug)]
struct Hdr {
    nal_type: u8,
    slice_type: u32,
    poc_lsb: u32,
    /// Bit position of slice_pic_order_cnt_lsb in the RBSP.
    poc_pos: usize,
    /// Kept delta POCs of the short-term RPS; None for an IDR.
    rps: Option<RpsSet>,
    /// How the RPS was coded: Some(idx) for an SPS set, None explicit.
    rps_idx: Option<u32>,
}

/// First-slice header fields; `nal` from its 2-byte header, escaped.
fn parse_hdr(nal: &[u8], sps: &Sps, pps: &Pps, sets: &[(Vec<i32>, Vec<i32>)]) -> Option<Hdr> {
    if nal.len() < 4 {
        return None;
    }
    let nal_type = hevc::nal::nal_type(nal[0]);
    let rb = unescape(&nal[2..]);
    let mut r = BitReader::new(&rb);
    let first = r.flag();
    if (16..=23).contains(&nal_type) {
        r.u(1);
    }
    if r.ue() != pps.pps_id {
        return None;
    }
    if !first {
        if pps.dependent_slices && r.flag() {
            return None;
        }
        r.u(sps.addr_bits());
    }
    r.u(pps.extra_bits);
    let slice_type = r.ue();
    if pps.output_flag {
        r.u(1);
    }
    let mut h = Hdr {
        nal_type,
        slice_type,
        poc_lsb: 0,
        poc_pos: r.pos(),
        rps: None,
        rps_idx: None,
    };
    if nal_type != 19 && nal_type != 20 {
        h.poc_lsb = r.u(sps.log2_max_poc_lsb);
        let (neg, pos) = if !r.flag() {
            read_st_rps(&mut r, sets.len(), true, sets)?
        } else {
            let mut idx = 0;
            if sps.num_st_rps > 1 {
                idx = r.u(ceil_log2(sps.num_st_rps));
            }
            h.rps_idx = Some(idx);
            sets.get(idx as usize)?.clone()
        };
        if sps.long_term {
            return None;
        }
        h.rps = Some(neg.into_iter().chain(pos).collect());
    }
    (!r.bad()).then_some(h)
}

/// The first VCL NAL of an AU (escaped, from its NAL header).
fn first_vcl(au: &[u8]) -> Option<&[u8]> {
    split_nals(au)
        .into_iter()
        .find(|v| v.ty < 32)
        .map(|v| &au[v.off..v.off + v.len])
}

fn is_irap(au: &Au) -> bool {
    (16..=23).contains(&au.vcl_nal) || au.flags & 1 != 0
}

/// Parameter sets and RPS sets of the complete AUs seen so far.
#[derive(Default)]
struct Params {
    pt: ParamTracker,
    sets: Vec<(Vec<i32>, Vec<i32>)>,
}

impl Params {
    /// Every whole NAL of `au`: all of a complete AU, the NALs before the
    /// last start code of an incomplete one (a truncated IDR still brings
    /// its VPS/SPS/PPS).
    fn feed(&mut self, au: &[u8], complete: bool) {
        let au = if complete {
            au
        } else {
            match split_nals(au).last() {
                Some(v) => &au[..v.off.saturating_sub(3)],
                None => return,
            }
        };
        self.pt.feed(au);
        for v in split_nals(au) {
            if v.ty == 33 {
                if let Some(s) = sps_rps_sets(&au[v.off..v.off + v.len]) {
                    self.sets = s;
                }
            }
        }
    }
    fn hdr(&self, au: &[u8]) -> Option<Hdr> {
        if !self.pt.usable() {
            return None;
        }
        parse_hdr(first_vcl(au)?, self.pt.sps(), self.pt.pps(), &self.sets)
    }
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

/// A P picture the skip decoder fed, as a donor candidate.
#[derive(Clone)]
struct Pic {
    hdr: Hdr,
    nal: Vec<u8>,
    pts: u32,
}

/// Builds the whole-frame skip picture for a damaged AU, or says why not.
/// Donor rules (waybeam-link): the donor is the newest reference P
/// picture fed (TRAIL_R, never TRAIL_N); it and the reference P picture
/// before it carry the same short-term RPS; the skip copies the donor's
/// slice header (so its L0 count, collocated_ref_idx and TMVP flag) with
/// only the POC changed, and every picture the copied RPS names must be in
/// the DPB (the donor itself or one the donor's RPS kept).
struct SkipMaker {
    ps: Params,
    /// The last two reference P pictures fed, oldest first.
    refs: Vec<Pic>,
    /// The last two P pictures fed of any kind (the rule read literally).
    any: Vec<Hdr>,
    /// "skip" or "skip+missing": the policy it serves, for the counts.
    id: &'static str,
    /// Damaged AUs for which the literal rule (last two P pictures of any
    /// kind share the RPS) failed.
    literal_fail: u32,
}

impl SkipMaker {
    fn new(id: &'static str) -> Self {
        Self {
            id,
            ps: Params::default(),
            refs: Vec::new(),
            any: Vec::new(),
            literal_fail: 0,
        }
    }

    /// An AU that went to this decoder: `whole` when complete or a skip
    /// picture (a skip TRAIL_R picture is a reference like any other).
    fn fed(&mut self, data: &[u8], complete: bool, whole: bool, pts: u32) {
        self.ps.feed(data, complete);
        let Some(h) = self.ps.hdr(data) else { return };
        if (16..=23).contains(&h.nal_type) {
            self.refs.clear();
            self.any.clear();
            return;
        }
        if h.slice_type == 2 || !whole {
            return;
        }
        self.any.push(h.clone());
        if self.any.len() > 2 {
            self.any.remove(0);
        }
        if h.nal_type % 2 == 1 {
            self.refs.push(Pic {
                hdr: h,
                nal: first_vcl(data).unwrap_or_default().to_vec(),
                pts,
            });
            if self.refs.len() > 2 {
                self.refs.remove(0);
            }
        }
    }

    fn make(&mut self, damaged: &Au, frame_us: u32) -> Result<Vec<u8>, &'static str> {
        if self.any.len() == 2 && self.any[0].rps != self.any[1].rps {
            self.literal_fail += 1;
        }
        if !self.ps.pt.usable() {
            return Err("no_params");
        }
        let (sps, pps) = (self.ps.pt.sps().clone(), self.ps.pt.pps().clone());
        if self.refs.len() < 2 {
            return Err("no_two_ref_p");
        }
        let (prev, donor) = (&self.refs[0], &self.refs[1]);
        if prev.hdr.rps != donor.hdr.rps {
            return Err("rps_differs");
        }
        if donor.hdr.nal_type != 1 {
            return Err("donor_not_trail_r");
        }
        let deltas = donor.hdr.rps.clone().ok_or("donor_no_rps")?;
        let max = 1u32 << sps.log2_max_poc_lsb;
        // The damaged picture's own POC when its first slice header arrived,
        // else the donor's plus the frame steps its pts says passed.
        let poc = match self.ps.hdr(&damaged.data) {
            Some(h) => h.poc_lsb,
            None => {
                let steps =
                    (damaged.pts_us.wrapping_sub(donor.pts) + frame_us / 2) / frame_us.max(1);
                (donor.hdr.poc_lsb + steps) % max
            }
        };
        let dpoc = ((poc + max - donor.hdr.poc_lsb) % max) as i32;
        if dpoc == 0 || dpoc > (max / 2) as i32 {
            return Err("poc_not_after_donor");
        }
        for d in &deltas {
            let want = dpoc + d;
            if want != 0 && !deltas.contains(&want) {
                return Err("ref_not_in_dpb");
            }
        }
        let mut tmpl =
            parse_slice_header(&donor.nal, &sps, &pps).map_err(|_| "donor_unparsable")?;
        let w = sps.log2_max_poc_lsb as usize;
        for b in 0..w {
            let p = donor.hdr.poc_pos + b;
            let m = 0x80u8 >> (p % 8);
            if (poc >> (w - 1 - b)) & 1 != 0 {
                tmpl.rbsp[p / 8] |= m;
            } else {
                tmpl.rbsp[p / 8] &= !m;
            }
        }
        let total = sps.pic_w_ctbs() * sps.pic_h_ctbs();
        make_skip_slice(&sps, &pps, &tmpl, 0, total).ok_or("make_skip_slice")
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

const POLICIES: [&str; 4] = ["prefix", "drop", "skip", "skip+missing"];

/// The skip picture for `au`, counting it or its fallback under `tag`.
fn try_skip(
    sk: &mut SkipMaker,
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
        sk.make(au, frame_us)
    };
    let key = match r {
        Ok(_) => format!("{}/{tag}/made/{layer}", sk.id),
        Err(why) => format!("{}/{tag}/{why}/{layer}", sk.id),
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
    let mut sk = SkipMaker::new("conformance");
    let (mut base, mut made) = (0usize, 0u32);
    let mut why: HashMap<&'static str, u32> = HashMap::new();
    for a in reference {
        let mut out = a.data.clone();
        let mut skip = false;
        if a.complete && a.sid == 0 && !is_irap(a) {
            base += 1;
            if base % every == 0 {
                // Its own header gives the POC; the rest is the donor's.
                match sk.make(a, frame_us) {
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
    let mut skippers = [SkipMaker::new("skip"), SkipMaker::new("skip+missing")];
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
        let feeds: [Option<Vec<u8>>; 4] = match a {
            None if is_irap(r) => [None, None, None, None],
            None => [
                None,
                None,
                None,
                try_skip(&mut skippers[1], &ghost, frame_us, &mut fallback, "missing"),
            ],
            Some(a) if !hole => [
                Some(a.data.clone()),
                Some(a.data.clone()),
                Some(a.data.clone()),
                Some(a.data.clone()),
            ],
            Some(a) => [
                Some(a.data.clone()),
                None,
                try_skip(&mut skippers[0], a, frame_us, &mut fallback, "hole"),
                try_skip(&mut skippers[1], a, frame_us, &mut fallback, "hole"),
            ],
        };
        for (k, p) in pol.iter_mut().enumerate() {
            if let Some(f) = &feeds[k] {
                p.fed += 1;
                if let Some(w) = p.dump.as_mut() {
                    w.write_all(f).unwrap();
                }
                if p.dec.decode(f) > 0 && p.dec.planes(&mut planes) {
                    p.shown = Some(planes.y.clone());
                }
                if k >= 2 {
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
    // Slots where the policies show different pictures: the only ones
    // that tell them apart.
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
