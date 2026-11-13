//! The maintenance lane (mabur `common/include/mabur/maint_wire.h`): a shell on
//! the drone over the air. Not an RC frame: its own magic and envelope version,
//! so a GS reaches a drone at another RC_VERSION. The envelope only moves bytes;
//! the commands are the drone shell's. A GS keeps every envelope version a drone
//! in the field may run.
//!
//! `u16 magic "MT" | u8 version | u8 kind | u32 session | u32 seq | u32 ack |
//! u8 flags | i16 status | u16 len | u8[len] data | u16 crc`, little-endian,
//! CRC16-CCITT over every byte before it.

use alloc::collections::VecDeque;
use alloc::vec::Vec;

use crate::crc16::crc16_ccitt;

pub const MAGIC: u16 = 0x544D;
pub const VERSION: u8 = 1;
pub const HEAD_LEN: usize = 21;
pub const MAX_DATA: usize = 1200;

/// GS -> drone: run `data` with /bin/sh -c (empty = interactive shell).
pub const K_OPEN: u8 = 1;
/// Both ways: stream bytes at `seq`, `ack` = next offset expected.
pub const K_DATA: u8 = 2;
/// GS -> drone: end the session. Drone -> GS: no such session; `status` why.
pub const K_CLOSE: u8 = 3;
/// GS -> drone: video off for data\[0..2\] (u16) ms, 0 = on now. Drone -> GS: the lease left.
pub const K_VIDEO: u8 = 4;

pub const FLAG_FIN: u8 = 0x01;
/// On every frame the drone sends, on none the GS sends: a two-card GS hears
/// its own frames and must not take them for the drone's.
pub const FLAG_FROM_DRONE: u8 = 0x80;

pub const STATUS_CLOSED: i16 = 0;
pub const STATUS_ARMED: i16 = -1;
pub const STATUS_SPAWN_FAILED: i16 = -2;
pub const STATUS_UNKNOWN: i16 = -3;
pub const STATUS_UNSUPPORTED: i16 = -4;
pub const STATUS_IDLE: i16 = -5;

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Frame {
    pub kind: u8,
    pub session: u32,
    pub seq: u32,
    pub ack: u32,
    pub flags: u8,
    pub status: i16,
    pub data: Vec<u8>,
}

/// Caller keeps `data.len() <= MAX_DATA`.
pub fn pack(f: &Frame) -> Vec<u8> {
    let mut b = Vec::with_capacity(HEAD_LEN + f.data.len() + 2);
    b.extend_from_slice(&MAGIC.to_le_bytes());
    b.push(VERSION);
    b.push(f.kind);
    b.extend_from_slice(&f.session.to_le_bytes());
    b.extend_from_slice(&f.seq.to_le_bytes());
    b.extend_from_slice(&f.ack.to_le_bytes());
    b.push(f.flags);
    b.extend_from_slice(&f.status.to_le_bytes());
    b.extend_from_slice(&(f.data.len() as u16).to_le_bytes());
    b.extend_from_slice(&f.data);
    let c = crc16_ccitt(&b);
    b.extend_from_slice(&c.to_le_bytes());
    b
}

/// Magic and version only, no CRC: routes a body on the RX path.
pub fn is_maint(b: &[u8]) -> bool {
    b.len() >= 3 && u16::from_le_bytes([b[0], b[1]]) == MAGIC && b[2] == VERSION
}

/// Any kind; None for a short buffer, another magic or version, len >
/// MAX_DATA or a bad CRC. Bytes past the CRC (the 802.11 FCS) are ignored.
pub fn parse(b: &[u8]) -> Option<Frame> {
    if b.len() < HEAD_LEN + 2 || !is_maint(b) {
        return None;
    }
    let g16 = |i: usize| u16::from_le_bytes([b[i], b[i + 1]]);
    let g32 = |i: usize| u32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
    let n = usize::from(g16(19));
    if n > MAX_DATA || b.len() < HEAD_LEN + n + 2 {
        return None;
    }
    if crc16_ccitt(&b[..HEAD_LEN + n]) != g16(HEAD_LEN + n) {
        return None;
    }
    Some(Frame {
        kind: b[3],
        session: g32(4),
        seq: g32(8),
        ack: g32(12),
        flags: b[16],
        status: g16(17) as i16,
        data: b[HEAD_LEN..HEAD_LEN + n].to_vec(),
    })
}

#[derive(Clone, Copy, Debug)]
pub struct StreamCfg {
    pub max_frame: usize,
    pub window: usize,
    pub send_cap: usize,
    pub recv_cap: usize,
    pub rto_ms: u64,
    pub ack_delay_ms: u64,
    pub max_frames_per_poll: usize,
}

impl Default for StreamCfg {
    fn default() -> Self {
        Self {
            max_frame: MAX_DATA,
            window: 16 * MAX_DATA,
            send_cap: 64 * 1024,
            recv_cap: 64 * 1024,
            rto_ms: 300,
            ack_delay_ms: 5,
            max_frames_per_poll: 8,
        }
    }
}

/// One session's byte streams, the same machine as mabur's `maint::Stream`:
/// go-back-N out (resend from the oldest unacked byte after `rto_ms` without
/// progress, or on the second duplicate pure ack), in-order delivery in. FIN
/// takes one offset after the last byte.
#[derive(Debug)]
pub struct Stream {
    session: u32,
    cfg: StreamCfg,
    buf: VecDeque<u8>,
    snd_una: u32,
    snd_nxt: u32,
    snd_max: u32,
    fin: bool,
    fin_off: u32,
    fin_status: i16,
    progress_ms: u64,
    dup_acks: u32,
    resends: u64,
    rx: Vec<u8>,
    rcv_nxt: u32,
    peer_fin: bool,
    peer_status: i16,
    ack_due: bool,
    ack_at: u64,
}

impl Stream {
    pub fn new(session: u32, cfg: StreamCfg) -> Self {
        Self {
            session,
            cfg,
            buf: VecDeque::new(),
            snd_una: 0,
            snd_nxt: 0,
            snd_max: 0,
            fin: false,
            fin_off: 0,
            fin_status: 0,
            progress_ms: 0,
            dup_acks: 0,
            resends: 0,
            rx: Vec::new(),
            rcv_nxt: 0,
            peer_fin: false,
            peer_status: 0,
            ack_due: false,
            ack_at: 0,
        }
    }
    pub fn session(&self) -> u32 {
        self.session
    }
    fn end_off(&self) -> u32 {
        self.snd_una + self.buf.len() as u32
    }
    pub fn send_room(&self) -> usize {
        if self.fin {
            0
        } else {
            self.cfg.send_cap.saturating_sub(self.buf.len())
        }
    }
    /// Takes as much as fits; returns the count.
    pub fn write(&mut self, p: &[u8]) -> usize {
        let take = p.len().min(self.send_room());
        self.buf.extend(&p[..take]);
        take
    }
    pub fn finish(&mut self, status: i16) {
        if self.fin {
            return;
        }
        self.fin = true;
        self.fin_off = self.end_off();
        self.fin_status = status;
    }
    pub fn finishing(&self) -> bool {
        self.fin
    }
    pub fn out_done(&self) -> bool {
        self.fin && self.snd_una == self.fin_off + 1
    }
    pub fn unacked(&self) -> usize {
        self.buf.len()
    }
    pub fn read(&mut self, out: &mut Vec<u8>) -> usize {
        let n = self.rx.len();
        out.append(&mut self.rx);
        n
    }
    pub fn readable(&self) -> usize {
        self.rx.len()
    }
    pub fn peer_fin(&self) -> bool {
        self.peer_fin
    }
    pub fn peer_status(&self) -> i16 {
        self.peer_status
    }
    pub fn resends(&self) -> u64 {
        self.resends
    }
    pub fn ack_now(&mut self) {
        self.ack_due = true;
        self.ack_at = 0;
    }

    fn make_data(&self, seq: u32, len: usize, fin: bool) -> Frame {
        let from = (seq - self.snd_una) as usize;
        Frame {
            kind: K_DATA,
            session: self.session,
            seq,
            ack: self.rcv_nxt,
            flags: if fin { FLAG_FIN } else { 0 },
            status: if fin { self.fin_status } else { 0 },
            data: self.buf.range(from..from + len).copied().collect(),
        }
    }

    pub fn on_frame(&mut self, f: &Frame, now_ms: u64) {
        if f.kind != K_DATA || f.session != self.session {
            return;
        }
        let pure_ack = f.data.is_empty() && f.flags & FLAG_FIN == 0;
        if f.ack > self.snd_una && f.ack <= self.snd_max {
            let data_acked = (f.ack.min(self.end_off()) - self.snd_una) as usize;
            self.buf.drain(..data_acked);
            self.snd_una = f.ack;
            if self.snd_nxt < self.snd_una {
                self.snd_nxt = self.snd_una;
            }
            self.progress_ms = now_ms;
            self.dup_acks = 0;
        } else if pure_ack && f.ack == self.snd_una && self.snd_max > self.snd_una {
            self.dup_acks += 1;
            if self.dup_acks == 2 {
                self.snd_nxt = self.snd_una;
                self.progress_ms = now_ms;
                self.resends += 1;
            }
        }
        if pure_ack {
            return;
        }
        let end = f.seq + f.data.len() as u32;
        let fin = f.flags & FLAG_FIN != 0;
        let new_bytes = f.seq <= self.rcv_nxt && end > self.rcv_nxt;
        let new_fin = fin && f.seq <= self.rcv_nxt && end == self.rcv_nxt;
        if !self.peer_fin && (new_bytes || new_fin) {
            let skip = (self.rcv_nxt - f.seq) as usize;
            let take = f.data.len() - skip;
            if self.rx.len() + take <= self.cfg.recv_cap {
                self.rx.extend_from_slice(&f.data[skip..]);
                self.rcv_nxt = end;
                if fin {
                    self.rcv_nxt = end + 1;
                    self.peer_fin = true;
                    self.peer_status = f.status;
                }
                if !self.ack_due {
                    self.ack_due = true;
                    self.ack_at = now_ms + self.cfg.ack_delay_ms;
                }
                return;
            }
        }
        self.ack_now();
    }

    pub fn poll(&mut self, now_ms: u64, out: &mut Vec<Frame>) {
        if self.snd_max > self.snd_una && now_ms.saturating_sub(self.progress_ms) >= self.cfg.rto_ms
        {
            self.snd_nxt = self.snd_una;
            self.progress_ms = now_ms;
            self.resends += 1;
        }
        let limit = self.end_off() + u32::from(self.fin);
        let mut sent = 0;
        while sent < self.cfg.max_frames_per_poll
            && self.snd_nxt < limit
            && ((self.snd_nxt - self.snd_una) as usize) < self.cfg.window
        {
            let data_end = self.end_off();
            let len = if self.snd_nxt < data_end {
                self.cfg.max_frame.min((data_end - self.snd_nxt) as usize)
            } else {
                0
            };
            let fin_here = self.fin && self.snd_nxt + len as u32 == self.fin_off;
            if len == 0 && !fin_here {
                break;
            }
            if self.snd_nxt == self.snd_una && self.snd_max == self.snd_una {
                self.progress_ms = now_ms;
            }
            out.push(self.make_data(self.snd_nxt, len, fin_here));
            self.snd_nxt += len as u32 + u32::from(fin_here);
            self.snd_max = self.snd_max.max(self.snd_nxt);
            sent += 1;
        }
        if sent > 0 {
            self.ack_due = false;
        } else if self.ack_due && now_ms >= self.ack_at {
            out.push(Frame {
                kind: K_DATA,
                session: self.session,
                seq: self.snd_nxt,
                ack: self.rcv_nxt,
                ..Frame::default()
            });
            self.ack_due = false;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    /// mabur `tests/test_maint_wire.cpp` golden_frame_packs_and_parses.
    #[test]
    fn golden_frame_matches_mabur() {
        let f = Frame {
            kind: K_DATA,
            session: 0x1122_3344,
            seq: 5,
            ack: 7,
            flags: FLAG_FIN,
            status: -2,
            data: b"ok\n".to_vec(),
        };
        let w = pack(&f);
        let want = [
            0x4D, 0x54, 0x01, 0x02, 0x44, 0x33, 0x22, 0x11, 0x05, 0x00, 0x00, 0x00, 0x07, 0x00,
            0x00, 0x00, 0x01, 0xFE, 0xFF, 0x03, 0x00, b'o', b'k', b'\n', 0x8D, 0x5D,
        ];
        assert_eq!(w, want);
        assert_eq!(parse(&w), Some(f));
        let mut fcs = w.clone();
        fcs.extend_from_slice(&[1, 2, 3, 4]);
        assert!(parse(&fcs).is_some());
        let mut bad = w.clone();
        bad[21] ^= 1;
        assert!(parse(&bad).is_none());
        let mut ver = w;
        ver[2] = 2;
        assert!(!is_maint(&ver) && parse(&ver).is_none());
        assert!(!is_maint(&[0x43, 0x52, 18, 1]));
    }

    fn bytes(n: usize, mut seed: u32) -> Vec<u8> {
        (0..n)
            .map(|_| {
                seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12345);
                (seed >> 16) as u8
            })
            .collect()
    }

    fn carry(from: &mut Stream, to: &mut Stream, now: u64, drop: &mut impl FnMut() -> bool) {
        let mut fs = Vec::new();
        from.poll(now, &mut fs);
        for f in fs {
            let w = pack(&f);
            if !drop() {
                to.on_frame(&parse(&w).unwrap(), now);
            }
        }
    }

    /// Both ends through pack/parse, `drop` deciding per frame.
    fn run(
        a: &mut Stream,
        b: &mut Stream,
        up: &[u8],
        mut drop: impl FnMut() -> bool,
    ) -> (Vec<u8>, u64) {
        let (mut off, mut got, mut now) = (0, Vec::new(), 0u64);
        while now < 60_000 && !(a.out_done() && b.peer_fin()) {
            if off < up.len() {
                off += a.write(&up[off..]);
            }
            if off == up.len() {
                a.finish(3);
            }
            carry(a, b, now, &mut drop);
            carry(b, a, now, &mut drop);
            b.read(&mut got);
            now += 1;
        }
        (got, now)
    }

    #[test]
    fn clean_and_lossy_transfers_deliver_in_order_with_the_status() {
        let data = bytes(300_000, 1);
        let (mut a, mut b) = (
            Stream::new(9, StreamCfg::default()),
            Stream::new(9, StreamCfg::default()),
        );
        let (got, _) = run(&mut a, &mut b, &data, || false);
        assert_eq!(got, data);
        assert_eq!((b.peer_status(), a.resends()), (3, 0));
        let mut s = 7u32;
        let (mut a, mut b) = (
            Stream::new(9, StreamCfg::default()),
            Stream::new(9, StreamCfg::default()),
        );
        let (got, _) = run(&mut a, &mut b, &data[..100_000], || {
            s = s.wrapping_mul(1_103_515_245).wrapping_add(12345);
            (s >> 16) % 100 < 20
        });
        assert_eq!(got, data[..100_000]);
        assert!(b.peer_fin() && a.resends() > 0);
    }

    #[test]
    fn a_lost_frame_is_resent_on_duplicate_acks() {
        let cfg = StreamCfg {
            rto_ms: 10_000,
            ..StreamCfg::default()
        };
        let data = bytes(10 * MAX_DATA, 4);
        let (mut a, mut b) = (Stream::new(9, cfg), Stream::new(9, cfg));
        let mut n = 0;
        let (got, ms) = run(&mut a, &mut b, &data, || {
            n += 1;
            n == 1
        });
        assert_eq!(got, data);
        assert!(ms < 100, "{ms} ms");
        assert_eq!(a.resends(), 1);
    }

    #[test]
    fn fin_alone_and_other_sessions() {
        let (mut a, mut b) = (
            Stream::new(9, StreamCfg::default()),
            Stream::new(9, StreamCfg::default()),
        );
        let (got, _) = run(&mut a, &mut b, &[], || false);
        assert!(got.is_empty() && b.peer_fin() && a.out_done());
        let f = Frame {
            kind: K_DATA,
            session: 10,
            data: vec![1],
            ..Frame::default()
        };
        b.on_frame(&f, 0);
        assert_eq!(b.readable(), 0);
    }

    /// mabur `test_maint_wire.cpp` trace_hash_pins_the_frame_sequence_for_the_rust_port:
    /// the same scenario must give the same frames.
    #[test]
    fn trace_hash_matches_mabur() {
        let (mut a, mut b) = (
            Stream::new(9, StreamCfg::default()),
            Stream::new(9, StreamCfg::default()),
        );
        let (up, down) = (bytes(30_000, 11), bytes(5000, 12));
        let (mut ua, mut ub, mut s, mut h, mut now) = (0, 0, 7u32, 2_166_136_261u32, 0u64);
        let (mut got_a, mut got_b) = (Vec::new(), Vec::new());
        let mut carry = |from: &mut Stream, to: &mut Stream, now: u64| {
            let mut fs = Vec::new();
            from.poll(now, &mut fs);
            for f in fs {
                let w = pack(&f);
                for x in &w {
                    h = (h ^ u32::from(*x)).wrapping_mul(16_777_619);
                }
                s = s.wrapping_mul(1_103_515_245).wrapping_add(12345);
                if (s >> 16) % 100 < 20 {
                    continue;
                }
                to.on_frame(&parse(&w).unwrap(), now);
            }
        };
        while now < 20_000 && !(a.out_done() && b.out_done() && a.peer_fin() && b.peer_fin()) {
            if ua < up.len() {
                ua += a.write(&up[ua..]);
            }
            if ua == up.len() {
                a.finish(0);
            }
            if ub < down.len() {
                ub += b.write(&down[ub..]);
            }
            if ub == down.len() {
                b.finish(42);
            }
            carry(&mut a, &mut b, now);
            carry(&mut b, &mut a, now);
            a.read(&mut got_a);
            b.read(&mut got_b);
            now += 1;
        }
        assert!(got_b == up && got_a == down);
        assert_eq!((now, h), (1510, 0x6aea_47cc), "now={now} hash={h:08x}");
    }
}
