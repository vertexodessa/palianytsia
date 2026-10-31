//! The `webgs replay` loop, driven from Rust through `GsCore`. Writes the
//! same LP AU file (u32 len | u8 sid | u8 flags|0x04 complete | u32 pts_us |
//! Annex-B, little-endian) so `cmp` against `webgs replay` checks the
//! wrapper end to end.
//!
//! usage: pal-ref-replay <frames.bin> <out.aus> -c <gs.toml> [--mode gs|spotter]
//!        [--drop-pct N] [--seed N] [--fixed-gap] [--fake-ack]
use pal_core::{Au, CoreIo, GsCore, Mode};
use pal_ref::{FrameFile, RefCore, RefOpts};
use std::io::Write;
use std::path::PathBuf;

struct Sink {
    out: std::io::BufWriter<std::fs::File>,
    aus: u64,
    sends: u64,
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
    fn send(&mut self, _rc_body: &[u8]) {
        self.sends += 1;
    }
}

fn main() {
    let mut args = std::env::args().skip(1);
    let mut pos = Vec::new();
    let (mut config, mut mode, mut drop_pct, mut seed) = (None, Mode::Spotter, 0, 1u32);
    let (mut fixed_gap, mut fake_ack) = (false, false);
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
            _ => pos.push(PathBuf::from(a)),
        }
    }
    let [frames, out] = <[PathBuf; 2]>::try_from(pos).expect("need <frames.bin> <out.aus>");
    let config = config.expect("-c <gs.toml> is required");

    let mut core = RefCore::new(
        &config,
        RefOpts {
            mode,
            adaptive_gap: !fixed_gap,
            rz_nonce: 1,
            channel: 0,
        },
    )
    .unwrap_or_else(|e| panic!("pal_ref_new: {e}"));
    let mut src = FrameFile::open(&frames, 1, drop_pct, seed).unwrap();
    let mut sink = Sink {
        out: std::io::BufWriter::new(std::fs::File::create(&out).unwrap()),
        aus: 0,
        sends: 0,
    };

    let mut last_us = 0u64;
    let mut first = true;
    for body in src.by_ref() {
        if first && fake_ack {
            core.inject_disc_ack_for_replay(body.mono_us, &mut sink);
        }
        first = false;
        core.on_rx(&body, &mut sink);
        core.tick(body.mono_us, &mut sink);
        last_us = body.mono_us;
    }
    // The dry-run's final poll on the ms clock: last_ms + gap + 1.
    let gap = u64::from(core.frame_gap_timeout_ms());
    core.tick((last_us / 1000 + gap + 1) * 1000, &mut sink);
    sink.out.flush().unwrap();
    eprintln!(
        "pal-ref replay: frames={} dropped={} aus_out={} sends={} {}",
        src.frames_read(),
        src.dropped(),
        sink.aus,
        sink.sends,
        core.stats_json()
    );
}
