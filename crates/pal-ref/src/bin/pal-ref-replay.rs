//! The `webgs replay` loop, driven from Rust through `GsCore`. Writes the
//! same LP AU file (u32 len | u8 sid | u8 flags|0x04 complete | u32 pts_us |
//! Annex-B, little-endian) so `cmp` against `webgs replay` checks the
//! wrapper end to end.
//!
//! usage: pal-ref-replay <frames.bin> <out.aus> -c <gs.toml> [--mode gs|spotter]
//!        [--drop-pct N] [--seed N] [--fixed-gap] [--fake-ack] [--core ref|rust]
//!        [--sends <out.sends>]
//!
//! `--core rust` runs pal-core's RustCore instead, for the port's parity
//! gate against the same frames. `--sends` writes every transmitted RC
//! body as u32 len | bytes.
use pal_core::rust_core::{RustCore, RustOpts};
use pal_core::{Au, CoreIo, GsCore, Mode};
use pal_ref::{FrameFile, RefCore, RefOpts};
use std::io::Write;
use std::path::{Path, PathBuf};

struct Sink {
    out: std::io::BufWriter<std::fs::File>,
    aus: u64,
    sends: u64,
    sends_out: Option<std::io::BufWriter<std::fs::File>>,
}

impl CoreIo for Sink {
    fn on_au(&mut self, a: Au) {
        let flags = a.flags | if a.complete { 0x04 } else { 0 };
        self.out
            .write_all(&(a.data.len() as u32).to_le_bytes())
            .unwrap();
        self.out.write_all(&[a.sid, flags]).unwrap();
        self.out.write_all(&a.pts_us.to_le_bytes()).unwrap();
        self.out.write_all(&a.data).unwrap();
        self.aus += 1;
    }
    fn send(&mut self, rc_body: &[u8]) {
        self.sends += 1;
        if let Some(w) = self.sends_out.as_mut() {
            w.write_all(&(rc_body.len() as u32).to_le_bytes()).unwrap();
            w.write_all(rc_body).unwrap();
        }
    }
}

impl Sink {
    fn create(out: &Path, sends: Option<&Path>) -> Self {
        let file = |p: &Path| std::io::BufWriter::new(std::fs::File::create(p).unwrap());
        Sink {
            out: file(out),
            aus: 0,
            sends: 0,
            sends_out: sends.map(file),
        }
    }
}

fn main() {
    let mut args = std::env::args().skip(1);
    let mut pos = Vec::new();
    let (mut config, mut mode, mut drop_pct, mut seed) = (None, Mode::Spotter, 0, 1u32);
    let (mut fixed_gap, mut fake_ack, mut rust) = (false, false, false);
    let mut sends: Option<PathBuf> = None;
    while let Some(a) = args.next() {
        match a.as_str() {
            "-c" => config = args.next().map(PathBuf::from),
            "--mode" => {
                mode = match args.next().as_deref() {
                    Some("gs") => Mode::Gs,
                    Some("spotter") => Mode::Spotter,
                    m => panic!("--mode gs|spotter, got {m:?}"),
                }
            }
            "--drop-pct" => drop_pct = args.next().unwrap().parse().unwrap(),
            "--seed" => seed = args.next().unwrap().parse().unwrap(),
            "--fixed-gap" => fixed_gap = true,
            "--fake-ack" => fake_ack = true,
            "--sends" => sends = args.next().map(PathBuf::from),
            "--core" => rust = args.next().as_deref() == Some("rust"),
            _ => pos.push(PathBuf::from(a)),
        }
    }
    let [frames, out] = <[PathBuf; 2]>::try_from(pos).expect("need <frames.bin> <out.aus>");
    let config = config.expect("-c <gs.toml> is required");

    let src = || FrameFile::open(&frames, 1, drop_pct, seed).unwrap();
    let sink = || Sink::create(&out, sends.as_deref());
    let adaptive_gap = !fixed_gap;
    if rust {
        let cfg = pal_core::config::load_config(&config, None).unwrap_or_else(|e| panic!("{e}"));
        let mut core = RustCore::new(
            &cfg,
            RustOpts {
                mode,
                adaptive_gap,
                rz_nonce: 1,
                channel: 0,
            },
        )
        .unwrap_or_else(|e| panic!("{e}"));
        let gap = cfg.video.frame_gap_timeout_ms;
        let ack = |c: &mut RustCore, t, _: &mut Sink| c.inject_disc_ack_for_replay(t);
        drive(&mut core, src(), sink(), fake_ack, ack, |_| gap);
    } else {
        let mut core = RefCore::new(
            &config,
            RefOpts {
                mode,
                adaptive_gap,
                rz_nonce: 1,
                channel: 0,
            },
        )
        .unwrap_or_else(|e| panic!("pal_ref_new: {e}"));
        let ack = |c: &mut RefCore, t, s: &mut Sink| c.inject_disc_ack_for_replay(t, s);
        let gap = |c: &RefCore| u64::from(c.frame_gap_timeout_ms());
        drive(&mut core, src(), sink(), fake_ack, ack, gap);
    }
}

/// The replay loop m0-parity compares: one body, both cores.
fn drive<C: GsCore>(
    core: &mut C,
    mut src: FrameFile,
    mut sink: Sink,
    fake_ack: bool,
    ack: impl Fn(&mut C, u64, &mut Sink),
    gap_ms: impl Fn(&C) -> u64,
) {
    let mut last_us = 0u64;
    let mut first = true;
    for body in src.by_ref() {
        if first && fake_ack {
            ack(core, body.mono_us, &mut sink);
        }
        first = false;
        core.on_rx(&body, &mut sink);
        core.tick(body.mono_us, &mut sink);
        last_us = body.mono_us;
    }
    // The dry-run's final poll on the ms clock: last_ms + gap + 1.
    core.tick((last_us / 1000 + gap_ms(core) + 1) * 1000, &mut sink);
    sink.out.flush().unwrap();
    if let Some(w) = sink.sends_out.as_mut() {
        w.flush().unwrap();
    }
    eprintln!(
        "pal-ref replay: frames={} dropped={} aus_out={} sends={} {}",
        src.frames_read(),
        src.dropped(),
        sink.aus,
        sink.sends,
        core.stats_json()
    );
}
