use super::*;
use std::sync::Mutex;

#[derive(Clone, Default)]
struct Mem(Arc<Mutex<Vec<u8>>>, Arc<AtomicBool>);

impl Sink for Mem {
    fn write(&mut self, p: &[u8]) -> bool {
        if self.1.load(Ordering::Relaxed) {
            return false;
        }
        self.0.lock().unwrap().extend_from_slice(p);
        true
    }
    fn flush(&mut self) -> bool {
        !self.1.load(Ordering::Relaxed)
    }
}

fn nal(ty: u8, body: &[u8]) -> Vec<u8> {
    let mut v = vec![0, 0, 0, 1, ty << 1, 1];
    v.extend_from_slice(body);
    v
}

/// SPS NALs (header included) the bench quad's encoder sent on 2026-11-14:
/// 1920x1080 (coded 1920x1088 with a conformance window) and 2944x1656.
/// Escaped as on the wire: the zero compatibility and constraint flags grow
/// four emulation-prevention bytes inside the profile_tier_level.
const SPS_1080: &str =
    "42010101600000030000030000030000030096a003c08010e58dae493222fe16700800001f48000aff5040";
const SPS_1656: &str =
    "42010101600000030000030000030000030096a00170200679636b924c88bf859c02000007d20001d53810";

fn sps(hex: &str) -> Vec<u8> {
    let mut v = vec![0, 0, 0, 1];
    v.extend(
        (0..hex.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap()),
    );
    v
}

fn sps_hd() -> Vec<u8> {
    sps(SPS_1080)
}

/// IDR with VPS/SPS/PPS, like the encoder's keyframe.
fn idr(sps: &[u8]) -> Vec<u8> {
    let mut v = nal(32, &[0x0c, 0x01, 0xff, 0xff]);
    v.extend_from_slice(sps);
    v.extend(nal(34, &[0xc1, 0x72, 0xb4, 0x62, 0x40]));
    v.extend(nal(19, &[0xaf, 0x00, 0x11, 0x22]));
    v
}

fn p_frame(i: u8) -> Vec<u8> {
    nal(1, &[0x80 | (i & 0x3f), 0x10, i])
}

/// Top-level (type, size) boxes; panics on a size that runs past the end.
fn boxes(f: &[u8]) -> Vec<(String, usize, usize)> {
    let mut out = Vec::new();
    let mut at = 0;
    while at + 8 <= f.len() {
        let size = u32::from_be_bytes(f[at..at + 4].try_into().unwrap()) as usize;
        let ty = String::from_utf8_lossy(&f[at + 4..at + 8]).to_string();
        assert!(
            size >= 8 && at + size <= f.len(),
            "box {ty} at {at} size {size}"
        );
        out.push((ty, at, size));
        at += size;
    }
    assert_eq!(at, f.len());
    out
}

fn find(f: &[u8], tag: &[u8; 4]) -> Vec<usize> {
    f.windows(4)
        .enumerate()
        .filter(|(_, w)| w == tag)
        .map(|(i, _)| i)
        .collect()
}

fn u32_at(f: &[u8], i: usize) -> u32 {
    u32::from_be_bytes(f[i..i + 4].try_into().unwrap())
}

fn record(aus: &[(Vec<u8>, u32, bool)]) -> (Vec<u8>, Recorder) {
    let mem = Mem::default();
    let m2 = mem.clone();
    let mut r = Recorder::new(1000);
    r.start(Box::new(move || {
        Ok((
            Box::new(m2.clone()) as Box<dyn Sink>,
            PathBuf::from("t.mp4"),
        ))
    }));
    for (au, pts, c) in aus {
        r.feed(au, *pts, *c);
    }
    r.stop();
    let f = mem.0.lock().unwrap().clone();
    (f, r)
}

#[test]
fn sps_size_and_hvcc_profile_come_from_the_unescaped_rbsp() {
    let mut p = HevcParams::default();
    assert!(p.feed(&idr(&sps_hd())));
    assert_eq!(p.sps_dimensions(), Some((1920, 1080)));
    let h = p.hvcc().unwrap();
    // general profile space/tier/idc 0x01, compat 0x60000000, constraint
    // flags all zero, level 0x96 (5.0): no 0x03 left in them.
    assert_eq!(&h[1..13], &[0x01, 0x60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x96]);
    assert_eq!(h[22], 0x03, "three arrays");
}

#[test]
fn a_file_begins_at_the_first_irap_and_skips_incomplete_aus() {
    let mut aus = vec![(p_frame(0), 0, true)];
    aus.push((idr(&sps_hd()), 16_667, true));
    for i in 1..5u32 {
        aus.push((p_frame(i as u8), 16_667 * (i + 1), i != 3));
    }
    let (f, r) = record(&aus);
    let top: Vec<_> = boxes(&f).into_iter().map(|b| b.0).collect();
    assert_eq!(top, ["ftyp", "moov", "moof", "mdat"]);
    let s = &r.sealed[0];
    assert_eq!((s.samples, s.ok), (4, true), "IDR + 3 complete P frames");
    assert_eq!(s.bytes, f.len() as u64);
    // trun: 4 samples, the first a sync sample, the rest not.
    let trun = find(&f, b"trun")[0];
    assert_eq!(u32_at(&f, trun + 8), 4);
    assert_eq!(f[trun + 8 + 8 + 8], 0x02);
    assert_eq!(f[trun + 8 + 8 + 12 + 8], 0x01);
    // tkhd width/height from the SPS.
    let tkhd = find(&f, b"tkhd")[0];
    assert_eq!(u32_at(&f, tkhd + 4 + 76) >> 16, 1920);
    assert_eq!(u32_at(&f, tkhd + 4 + 80) >> 16, 1080);
}

#[test]
fn fragments_cut_at_each_key_and_after_a_second_and_tile_the_timeline() {
    let mut aus = Vec::new();
    let mut pts = 4_294_000_000u32; // wraps inside the recording
    for i in 0..200u32 {
        let au = if i % 90 == 0 {
            idr(&sps_hd())
        } else {
            p_frame(i as u8)
        };
        aus.push((au, pts, true));
        pts = pts.wrapping_add(11_111);
    }
    let (f, r) = record(&aus);
    let top: Vec<_> = boxes(&f).into_iter().map(|b| b.0).collect();
    // keys at 0, 90, 180; 90 samples * 11.1 ms < 1 s, so cuts only at keys.
    assert_eq!(top.iter().filter(|t| *t == "moof").count(), 3);
    assert_eq!(r.sealed[0].samples, 200);
    let tfdt: Vec<u64> = find(&f, b"tfdt")
        .iter()
        .map(|&i| u64::from_be_bytes(f[i + 8..i + 16].try_into().unwrap()))
        .collect();
    assert_eq!(tfdt, [0, 90 * 11_111, 180 * 11_111]);
}

#[test]
fn a_long_gop_is_cut_every_second() {
    let mut aus = vec![(idr(&sps_hd()), 0, true)];
    for i in 1..300u32 {
        aus.push((p_frame(i as u8), i * 10_000, true));
    }
    let (f, _) = record(&aus);
    assert_eq!(find(&f, b"moof").len(), 3, "0-1 s, 1-2 s, 2-3 s");
}

#[test]
fn a_file_cut_short_still_walks_to_its_last_whole_fragment() {
    let mut aus = Vec::new();
    for i in 0..400u32 {
        let au = if i % 90 == 0 {
            idr(&sps_hd())
        } else {
            p_frame(i as u8)
        };
        aus.push((au, i * 11_111, true));
    }
    let (f, _) = record(&aus);
    let whole = boxes(&f);
    let moofs = whole.iter().filter(|b| b.0 == "moof").count();
    assert_eq!(moofs, 5);
    // Cut at every byte of the last fragment (a pulled battery): the boxes
    // before it are intact, and a reader that stops at the first box running
    // past the end sees all the earlier fragments.
    let last = whole[whole.len() - 2].1;
    for cut in last..f.len() {
        let g = &f[..cut];
        let (mut at, mut whole_frags) = (0, 0);
        while at + 8 <= g.len() {
            let size = u32_at(g, at) as usize;
            if at + size > g.len() {
                break;
            }
            whole_frags += (&g[at + 4..at + 8] == b"mdat") as usize;
            at += size;
        }
        assert_eq!(whole_frags, moofs - 1, "cut at {cut}");
    }
}

#[test]
fn a_size_change_at_an_irap_seals_the_file_and_opens_the_next() {
    let sps_big = sps(SPS_1656);
    let mut p = HevcParams::default();
    p.feed(&idr(&sps_big));
    assert_eq!(p.sps_dimensions(), Some((2944, 1656)));
    let files = Arc::new(Mutex::new(Vec::<Mem>::new()));
    let fs = files.clone();
    let mut r = Recorder::new(1000);
    r.start(Box::new(move || {
        let m = Mem::default();
        let mut l = fs.lock().unwrap();
        l.push(m.clone());
        Ok((
            Box::new(m) as Box<dyn Sink>,
            PathBuf::from(format!("{}.mp4", l.len())),
        ))
    }));
    r.feed(&idr(&sps_hd()), 0, true);
    r.feed(&p_frame(1), 10_000, true);
    r.feed(&idr(&sps_hd()), 20_000, true); // same size: no roll
    r.feed(&idr(&sps_big), 30_000, true);
    r.feed(&p_frame(2), 40_000, true);
    r.stop();
    let sealed = r.take_sealed();
    assert_eq!(sealed.len(), 2);
    assert_eq!((sealed[0].samples, sealed[1].samples), (3, 2));
    assert_eq!(sealed[1].path, PathBuf::from("2.mp4"));
}

#[test]
fn a_failed_write_stops_the_recording_and_keeps_its_error() {
    let mem = Mem::default();
    let m2 = mem.clone();
    let mut r = Recorder::new(1000);
    r.start(Box::new(move || {
        Ok((
            Box::new(m2.clone()) as Box<dyn Sink>,
            PathBuf::from("t.mp4"),
        ))
    }));
    r.feed(&idr(&sps_hd()), 0, true);
    assert_eq!(r.state(), State::Recording);
    mem.1.store(true, Ordering::Relaxed);
    r.feed(&idr(&sps_hd()), 20_000, true); // cuts: the first fragment's write fails
    assert_eq!(r.state(), State::Error);
    assert_eq!(r.error(), Some("write failed"));
    r.feed(&idr(&sps_hd()), 40_000, true);
    assert_eq!(r.state(), State::Error, "no restart by itself");
}

#[test]
fn an_unopenable_file_is_an_error_not_a_panic() {
    let mut r = Recorder::new(1000);
    r.start(Box::new(|| Err(std::io::Error::other("no card"))));
    r.feed(&idr(&sps_hd()), 0, true);
    assert_eq!((r.state(), r.error()), (State::Error, Some("no card")));
}

#[test]
fn the_thread_sink_writes_what_the_muxer_flushed() {
    let mem = Mem::default();
    let mut t = ThreadSink::new(Box::new(mem.clone())).unwrap();
    assert!(t.write(b"abc"));
    assert!(t.write(b"def"));
    assert!(t.flush());
    drop(t);
    for _ in 0..200 {
        if mem.0.lock().unwrap().len() == 6 {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(5));
    }
    assert_eq!(&*mem.0.lock().unwrap(), b"abcdef");
}
