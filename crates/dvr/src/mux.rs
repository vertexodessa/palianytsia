//! Fragmented MP4 writer (mabur `dvr_mux.cpp`, same boxes byte for byte).
//!
//! One hvc1 track, timescale 1 000 000 (the capture stamp's microseconds),
//! no B-frames, so decode order is presentation order. A fragment is cut at
//! every key AU and after `fragment_ms` of sample time, and is written whole
//! (moof, mdat header, samples, one flush): a crash or a pulled battery loses
//! at most the open fragment. Never reads a clock.

use crate::params::annexb_to_length_prefixed;

/// Where the muxer's bytes go.
pub trait Sink: Send {
    /// All of `p`, or false.
    fn write(&mut self, p: &[u8]) -> bool;
    /// End of a fragment.
    fn flush(&mut self) -> bool;
}

/// A buffered file.
pub struct FileSink(std::io::BufWriter<std::fs::File>);

impl FileSink {
    pub fn create(path: &std::path::Path) -> std::io::Result<Self> {
        let f = std::fs::File::create(path)?;
        Ok(Self(std::io::BufWriter::with_capacity(1 << 20, f)))
    }
}

impl Sink for FileSink {
    fn write(&mut self, p: &[u8]) -> bool {
        use std::io::Write;
        self.0.write_all(p).is_ok()
    }
    fn flush(&mut self) -> bool {
        use std::io::Write;
        self.0.flush().is_ok()
    }
}

#[derive(Default)]
struct Boxes {
    buf: Vec<u8>,
}

impl Boxes {
    fn begin(&mut self, tag: &[u8; 4]) -> usize {
        let at = self.buf.len();
        self.u32(0);
        self.buf.extend_from_slice(tag);
        at
    }
    fn end(&mut self, at: usize) {
        let size = (self.buf.len() - at) as u32;
        self.buf[at..at + 4].copy_from_slice(&size.to_be_bytes());
    }
    fn u8(&mut self, v: u8) {
        self.buf.push(v);
    }
    fn u16(&mut self, v: u16) {
        self.buf.extend_from_slice(&v.to_be_bytes());
    }
    fn u32(&mut self, v: u32) {
        self.buf.extend_from_slice(&v.to_be_bytes());
    }
    fn u64(&mut self, v: u64) {
        self.buf.extend_from_slice(&v.to_be_bytes());
    }
    fn bytes(&mut self, p: &[u8]) {
        self.buf.extend_from_slice(p);
    }
    fn matrix(&mut self) {
        for v in [0x0001_0000, 0, 0, 0, 0x0001_0000, 0, 0, 0, 0x4000_0000] {
            self.u32(v);
        }
    }
    /// A box holding only zero u32 fields.
    fn zeros(&mut self, tag: &[u8; 4], n: usize) {
        let at = self.begin(tag);
        for _ in 0..n {
            self.u32(0);
        }
        self.end(at);
    }
}

struct Sample {
    data: Vec<u8>,
    pts: u64,
    key: bool,
}

/// The muxer for one file.
pub struct Mux {
    sink: Box<dyn Sink>,
    fragment_us: u64,
    samples: u64,
    fragments: u64,
    bytes: u64,
    ok: bool,
    last_raw: Option<u32>,
    last_pts: u64,
    pending: Vec<Sample>,
    fragment_start: u64,
    /// The last measured sample interval: the duration of a fragment's last
    /// sample when nothing follows it (never 0; players derive the rate from
    /// it). Seeded with 60 fps.
    last_dur: u32,
}

impl Mux {
    /// Writes the init segment. `hvcc` from `HevcParams::hvcc`.
    pub fn open(
        sink: Box<dyn Sink>,
        hvcc: &[u8],
        width: u32,
        height: u32,
        fragment_ms: u32,
    ) -> Self {
        let mut m = Self {
            sink,
            fragment_us: fragment_ms as u64 * 1000,
            samples: 0,
            fragments: 0,
            bytes: 0,
            ok: true,
            last_raw: None,
            last_pts: 0,
            pending: Vec::new(),
            fragment_start: 0,
            last_dur: 16_667,
        };
        let b = init_segment(hvcc, width, height);
        m.put(&b);
        if m.ok && !m.sink.flush() {
            m.ok = false;
        }
        m
    }

    fn put(&mut self, p: &[u8]) {
        if !self.ok {
            return;
        }
        if self.sink.write(p) {
            self.bytes += p.len() as u64;
        } else {
            self.ok = false;
        }
    }

    /// The recording's time base starts at 0 with its first sample; the u32
    /// capture stamp is unwrapped by forward deltas.
    fn unwrap(&mut self, raw: u32) -> u64 {
        match self.last_raw {
            None => {
                self.last_pts = 0;
            }
            Some(prev) => self.last_pts += raw.wrapping_sub(prev) as u64,
        }
        self.last_raw = Some(raw);
        self.last_pts
    }

    /// One Annex B AU; `key` = IRAP.
    pub fn write_sample(&mut self, au: &[u8], pts_us: u32, key: bool) {
        let pts = self.unwrap(pts_us);
        if !self.pending.is_empty() && (key || pts - self.fragment_start >= self.fragment_us) {
            self.flush_fragment(Some(pts));
        }
        if self.pending.is_empty() {
            self.fragment_start = pts;
        }
        self.pending.push(Sample {
            data: annexb_to_length_prefixed(au),
            pts,
            key,
        });
        self.samples += 1;
    }

    fn flush_fragment(&mut self, next_pts: Option<u64>) {
        if self.pending.is_empty() {
            return;
        }
        if !self.ok {
            self.pending.clear();
            return;
        }
        self.fragments += 1;
        let mut b = Boxes::default();
        let moof = b.begin(b"moof");
        let at = b.begin(b"mfhd");
        b.u32(0);
        b.u32(self.fragments as u32);
        b.end(at);
        let traf = b.begin(b"traf");
        let at = b.begin(b"tfhd");
        b.u32(0x0002_0000); // default-base-is-moof
        b.u32(1);
        b.end(at);
        let at = b.begin(b"tfdt");
        b.u32(0x0100_0000); // version 1
        b.u64(self.fragment_start);
        b.end(at);
        let at = b.begin(b"trun");
        b.u32(0x0000_0701); // data offset, duration, size, flags
        b.u32(self.pending.len() as u32);
        let off_pos = b.buf.len();
        b.u32(0);
        for i in 0..self.pending.len() {
            let next = self.pending.get(i + 1).map(|s| s.pts).or(next_pts);
            let cur = self.pending[i].pts;
            let dur = match next {
                Some(n) if n > cur => {
                    let d = (n - cur).min(u32::MAX as u64) as u32;
                    self.last_dur = d;
                    d
                }
                _ => self.last_dur,
            };
            b.u32(dur);
            b.u32(self.pending[i].data.len() as u32);
            if self.pending[i].key {
                b.bytes(&[0x02, 0, 0, 0]);
            } else {
                b.bytes(&[0x01, 0x01, 0, 0]);
            }
        }
        b.end(at);
        b.end(traf);
        b.end(moof);
        let data_off = b.buf.len() as u32 + 8;
        b.buf[off_pos..off_pos + 4].copy_from_slice(&data_off.to_be_bytes());
        let mdat: u64 = 8 + self
            .pending
            .iter()
            .map(|s| s.data.len() as u64)
            .sum::<u64>();
        b.u32(mdat as u32);
        b.bytes(b"mdat");
        let pending = std::mem::take(&mut self.pending);
        self.put(&b.buf);
        for s in &pending {
            self.put(&s.data);
        }
        if self.ok && !self.sink.flush() {
            self.ok = false;
        }
    }

    /// Writes the open fragment.
    pub fn close(&mut self) {
        self.flush_fragment(None);
    }

    pub fn ok(&self) -> bool {
        self.ok
    }
    pub fn bytes(&self) -> u64 {
        self.bytes
    }
    pub fn samples(&self) -> u64 {
        self.samples
    }
    pub fn fragments(&self) -> u64 {
        self.fragments
    }
}

fn init_segment(hvcc: &[u8], width: u32, height: u32) -> Vec<u8> {
    let mut b = Boxes::default();
    let at = b.begin(b"ftyp");
    b.bytes(b"iso5");
    b.u32(0x0000_0200);
    b.bytes(b"iso5iso6mp41");
    b.end(at);

    let moov = b.begin(b"moov");
    let at = b.begin(b"mvhd");
    b.u32(0);
    b.u32(0);
    b.u32(0);
    b.u32(1_000_000);
    b.u32(0);
    b.u32(0x0001_0000);
    b.u16(0x0100);
    b.u16(0);
    b.u32(0);
    b.u32(0);
    b.matrix();
    for _ in 0..6 {
        b.u32(0);
    }
    b.u32(2);
    b.end(at);

    let trak = b.begin(b"trak");
    let at = b.begin(b"tkhd");
    b.u32(0x0000_0003);
    b.u32(0);
    b.u32(0);
    b.u32(1);
    b.u32(0);
    b.u32(0);
    b.u32(0);
    b.u32(0);
    b.u16(0);
    b.u16(0);
    b.u16(0);
    b.u16(0);
    b.matrix();
    b.u32(width << 16);
    b.u32(height << 16);
    b.end(at);

    let mdia = b.begin(b"mdia");
    let at = b.begin(b"mdhd");
    b.u32(0);
    b.u32(0);
    b.u32(0);
    b.u32(1_000_000);
    b.u32(0);
    b.u16(0x55C4); // 'und'
    b.u16(0);
    b.end(at);
    let at = b.begin(b"hdlr");
    b.u32(0);
    b.u32(0);
    b.bytes(b"vide");
    b.u32(0);
    b.u32(0);
    b.u32(0);
    b.bytes(b"mabur DVR\0");
    b.end(at);

    let minf = b.begin(b"minf");
    let at = b.begin(b"vmhd");
    b.u32(1);
    b.u16(0);
    b.u16(0);
    b.u16(0);
    b.u16(0);
    b.end(at);
    let dinf = b.begin(b"dinf");
    let dref = b.begin(b"dref");
    b.u32(0);
    b.u32(1);
    let url = b.begin(b"url ");
    b.u32(1);
    b.end(url);
    b.end(dref);
    b.end(dinf);

    let stbl = b.begin(b"stbl");
    let stsd = b.begin(b"stsd");
    b.u32(0);
    b.u32(1);
    let hvc1 = b.begin(b"hvc1");
    b.u32(0);
    b.u16(0);
    b.u16(1);
    b.u16(0);
    b.u16(0);
    b.u32(0);
    b.u32(0);
    b.u32(0);
    b.u16(width as u16);
    b.u16(height as u16);
    b.u32(0x0048_0000);
    b.u32(0x0048_0000);
    b.u32(0);
    b.u16(1);
    for _ in 0..32 {
        b.u8(0);
    }
    b.u16(24);
    b.u16(0xFFFF);
    let at = b.begin(b"hvcC");
    b.bytes(hvcc);
    b.end(at);
    b.end(hvc1);
    b.end(stsd);
    b.zeros(b"stts", 2);
    b.zeros(b"stsc", 2);
    b.zeros(b"stsz", 3);
    b.zeros(b"stco", 2);
    b.end(stbl);
    b.end(minf);
    b.end(mdia);
    b.end(trak);

    let mvex = b.begin(b"mvex");
    let at = b.begin(b"trex");
    b.u32(0);
    b.u32(1);
    b.u32(1);
    b.u32(0);
    b.u32(0);
    b.u32(0);
    b.end(at);
    b.end(mvex);
    b.end(moov);
    b.buf
}
