//! Decoder cost on a `pal replay --aus` file, doing what pal-desktop's
//! decode thread does per AU: decode, and RGBA of every frame that comes
//! out. Run under `/usr/bin/time` for CPU; prints frames and wall time.
//!
//!   cargo run --release -p decode --example decbench -- in.aus [hwaccel] [threads]
//!   DECBENCH_NO_RGBA=1: decode only (no download, no colour conversion).
//!   DECBENCH_PLANES=1: packed YUV planes instead of RGBA (pal-desktop's
//!   GPU-conversion path: download and copy, no swscale).
use decode::{AvDecoder, Decoder, Planes};
use std::time::Instant;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let path = a.get(1).expect("decbench in.aus [hwaccel] [threads]");
    let hw = a.get(2).map_or("", String::as_str);
    let threads: i32 = a.get(3).map_or(0, |t| t.parse().expect("threads"));
    let buf = std::fs::read(path).expect("read");
    let mut aus = Vec::new();
    let mut p = 0;
    while p + 10 <= buf.len() {
        let n = u32::from_le_bytes(buf[p..p + 4].try_into().unwrap()) as usize;
        p += 10; // len u32, sid u8, flags u8, pts u32
        aus.push(&buf[p..p + n]);
        p += n;
    }
    let mut d = AvDecoder::open_hw(threads, if hw == "sw" { "" } else { hw })
        .unwrap_or_else(|e| panic!("open {hw}: {e}"));
    let mut rgba = Vec::new();
    let convert = std::env::var_os("DECBENCH_NO_RGBA").is_none();
    let planes = std::env::var_os("DECBENCH_PLANES").is_some();
    let mut p = Planes::default();
    let t0 = Instant::now();
    for au in &aus {
        if d.decode(au) > 0 && convert {
            if planes {
                assert!(d.planes(&mut p), "no planes");
            } else {
                d.rgba(&mut rgba);
            }
        }
    }
    let wall = t0.elapsed().as_secs_f64() * 1e3;
    let (f, e) = d.counters();
    println!(
        "hw={} aus={} frames={f} hw_frames={} errors={e} wall_ms={wall:.0} wall_ms_per_frame={:.2}",
        if hw.is_empty() { "sw" } else { hw },
        aus.len(),
        d.hw_frames(),
        wall / f.max(1) as f64
    );
}
