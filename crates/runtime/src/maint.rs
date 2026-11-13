//! The GS end of the maintenance lane (`pal_wire::maint`): shell sessions on the
//! drone and the video-off wish. The core loop feeds it the bodies it hears and
//! sends what `poll` returns; any other thread (the CLI, the updater, an app's
//! menu) drives sessions through the same `Hub`.

use std::collections::HashMap;
use std::sync::Mutex;

use pal_wire::maint::{self, Frame, Stream, StreamCfg};

/// How often an unanswered OPEN goes out again.
const OPEN_RESEND_MS: u64 = 200;
/// The video-off lease asked for, and how often it is renewed.
const VIDEO_LEASE_MS: u16 = 3000;
const VIDEO_RENEW_MS: u64 = 500;
/// A CLOSE (or a video-on) is sent this many times: no answer is waited for.
const REPEATS: u8 = 3;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// OPEN sent, no answer yet (the drone may be restarting).
    Opening,
    Open,
    /// The command ended with this status; output may still be unread.
    Exited(i16),
    /// The drone ended or refused the session (`maint::STATUS_*`).
    Closed(i16),
}

#[derive(Clone, Copy, Debug)]
pub struct Info {
    pub state: State,
    /// Since anything of this session was last heard (or it was opened).
    pub quiet_ms: u64,
    pub resends: u64,
}

struct Sess {
    cmd: Vec<u8>,
    stream: Stream,
    state: State,
    next_open_ms: u64,
    heard_ms: Option<u64>,
    opened_ms: Option<u64>,
}

#[derive(Default)]
struct Inner {
    seed: u32,
    sessions: HashMap<u32, Sess>,
    closes: Vec<(u32, u8)>,
    video_off: bool,
    video_next_ms: u64,
    video_on_left: u8,
    /// The drone's last answer to a video frame: (lease ms, heard at).
    video_echo: Option<(u16, u64)>,
    now_ms: u64,
    frames_in: u64,
    frames_out: u64,
}

#[derive(Default)]
pub struct Hub {
    inner: Mutex<Inner>,
}

impl Hub {
    fn lock(&self) -> std::sync::MutexGuard<'_, Inner> {
        self.inner.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Starts a session running `cmd` (empty: an interactive shell) and
    /// returns its id; the OPEN goes out on the next poll and repeats until
    /// the drone answers.
    pub fn open(&self, cmd: &str) -> u32 {
        let mut g = self.lock();
        if g.seed == 0 {
            let t = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(1, |d| d.subsec_nanos() ^ d.as_secs() as u32);
            g.seed = t | 1;
        }
        let id = loop {
            g.seed = g.seed.wrapping_mul(1_103_515_245).wrapping_add(12345);
            let id = g.seed ^ std::process::id().rotate_left(16);
            if id != 0 && !g.sessions.contains_key(&id) {
                break id;
            }
        };
        g.sessions.insert(
            id,
            Sess {
                cmd: cmd.as_bytes().to_vec(),
                stream: Stream::new(id, StreamCfg::default()),
                state: State::Opening,
                next_open_ms: 0,
                heard_ms: None,
                opened_ms: None,
            },
        );
        id
    }

    /// Takes as much of `p` for the command's stdin as fits; returns the count.
    pub fn write(&self, id: u32, p: &[u8]) -> usize {
        self.lock()
            .sessions
            .get_mut(&id)
            .map_or(0, |s| s.stream.write(p))
    }
    /// The command's stdin ends after what was written.
    pub fn finish(&self, id: u32) {
        if let Some(s) = self.lock().sessions.get_mut(&id) {
            s.stream.finish(0);
        }
    }
    /// Appends the command's output received so far.
    pub fn read(&self, id: u32, out: &mut Vec<u8>) -> usize {
        self.lock()
            .sessions
            .get_mut(&id)
            .map_or(0, |s| s.stream.read(out))
    }
    /// Unacked stdin bytes: 0 once the drone has all of it.
    pub fn unacked(&self, id: u32) -> usize {
        self.lock()
            .sessions
            .get(&id)
            .map_or(0, |s| s.stream.unacked())
    }
    pub fn info(&self, id: u32) -> Option<Info> {
        let g = self.lock();
        let now = g.now_ms;
        g.sessions.get(&id).map(|s| Info {
            state: s.state,
            quiet_ms: now.saturating_sub(s.heard_ms.or(s.opened_ms).unwrap_or(now)),
            resends: s.stream.resends(),
        })
    }
    /// Ends the session: the drone kills the command; nothing more is read.
    pub fn close(&self, id: u32) {
        let mut g = self.lock();
        if g.sessions.remove(&id).is_some() {
            g.closes.push((id, REPEATS));
        }
    }

    /// While true the drone's video stays off (a lease renewed every
    /// VIDEO_RENEW_MS; it lapses on the drone by itself when the GS goes away).
    pub fn set_video_off(&self, off: bool) {
        let mut g = self.lock();
        if g.video_off == off {
            return;
        }
        g.video_off = off;
        g.video_next_ms = 0;
        g.video_on_left = if off { 0 } else { REPEATS };
    }
    /// The drone's last answer: its lease left in ms, and how long ago it was heard.
    pub fn video_echo(&self) -> Option<(u16, u64)> {
        let g = self.lock();
        g.video_echo
            .map(|(lease, at)| (lease, g.now_ms.saturating_sub(at)))
    }
    pub fn counts(&self) -> (u64, u64) {
        let g = self.lock();
        (g.frames_in, g.frames_out)
    }

    /// A body heard on the air; anything not a drone's maintenance frame is ignored.
    pub fn on_body(&self, body: &[u8], now_ms: u64) -> bool {
        if !maint::is_maint(body) {
            return false;
        }
        let Some(f) = maint::parse(body) else {
            return true;
        };
        if f.flags & maint::FLAG_FROM_DRONE == 0 {
            return true; // our own frame, heard by another card
        }
        let mut g = self.lock();
        g.frames_in += 1;
        match f.kind {
            maint::K_VIDEO => {
                let lease = f
                    .data
                    .get(..2)
                    .map_or(0, |b| u16::from_le_bytes([b[0], b[1]]));
                g.video_echo = Some((lease, now_ms));
            }
            maint::K_DATA => {
                if let Some(s) = g.sessions.get_mut(&f.session) {
                    s.heard_ms = Some(now_ms);
                    if s.state == State::Opening {
                        s.state = State::Open;
                    }
                    s.stream.on_frame(&f, now_ms);
                    if s.stream.peer_fin() && s.state == State::Open {
                        s.state = State::Exited(s.stream.peer_status());
                    }
                }
            }
            maint::K_CLOSE => {
                if let Some(s) = g.sessions.get_mut(&f.session) {
                    s.heard_ms = Some(now_ms);
                    if !matches!(s.state, State::Exited(_)) {
                        s.state = State::Closed(f.status);
                    }
                }
            }
            _ => {}
        }
        true
    }

    /// The frames due now, packed.
    pub fn poll(&self, now_ms: u64) -> Vec<Vec<u8>> {
        let mut g = self.lock();
        g.now_ms = now_ms;
        let mut out: Vec<Frame> = Vec::new();
        for (id, s) in g.sessions.iter_mut() {
            s.opened_ms.get_or_insert(now_ms);
            match s.state {
                State::Opening => {
                    if now_ms >= s.next_open_ms {
                        out.push(Frame {
                            kind: maint::K_OPEN,
                            session: *id,
                            data: s.cmd.clone(),
                            ..Frame::default()
                        });
                        s.next_open_ms = now_ms + OPEN_RESEND_MS;
                    }
                }
                State::Open | State::Exited(_) => s.stream.poll(now_ms, &mut out),
                State::Closed(_) => {}
            }
        }
        let mut closes = std::mem::take(&mut g.closes);
        for (id, left) in &mut closes {
            out.push(Frame {
                kind: maint::K_CLOSE,
                session: *id,
                ..Frame::default()
            });
            *left -= 1;
        }
        closes.retain(|(_, left)| *left > 0);
        g.closes = closes;
        let lease = if g.video_off {
            (now_ms >= g.video_next_ms).then_some(VIDEO_LEASE_MS)
        } else if g.video_on_left > 0 {
            g.video_on_left -= 1;
            Some(0)
        } else {
            None
        };
        if let Some(l) = lease {
            out.push(Frame {
                kind: maint::K_VIDEO,
                data: l.to_le_bytes().to_vec(),
                ..Frame::default()
            });
            g.video_next_ms = now_ms + VIDEO_RENEW_MS;
        }
        g.frames_out += out.len() as u64;
        out.iter().map(maint::pack).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A drone end good enough for the hub: answers OPEN, echoes stdin
    /// upper-cased, FINs with 7 at stdin EOF.
    struct Drone {
        s: Option<Stream>,
        video: u16,
    }
    impl Drone {
        fn on(&mut self, w: &[u8], now: u64, out: &mut Vec<Frame>) {
            let f = maint::parse(w).unwrap();
            assert_eq!(f.flags & maint::FLAG_FROM_DRONE, 0);
            match f.kind {
                maint::K_OPEN => {
                    if self.s.as_ref().map(Stream::session) != Some(f.session) {
                        self.s = Some(Stream::new(f.session, StreamCfg::default()));
                    }
                    self.s.as_mut().unwrap().ack_now();
                }
                maint::K_DATA => {
                    let s = self.s.as_mut().unwrap();
                    s.on_frame(&f, now);
                    let mut b = Vec::new();
                    s.read(&mut b);
                    s.write(&b.to_ascii_uppercase());
                    if s.peer_fin() {
                        s.finish(7);
                    }
                }
                maint::K_VIDEO => {
                    self.video = u16::from_le_bytes([f.data[0], f.data[1]]);
                    out.push(Frame {
                        kind: maint::K_VIDEO,
                        data: f.data.clone(),
                        ..Frame::default()
                    });
                }
                maint::K_CLOSE => self.s = None,
                _ => {}
            }
        }
    }

    fn step(hub: &Hub, d: &mut Drone, now: u64) {
        let mut back = Vec::new();
        for w in hub.poll(now) {
            d.on(&w, now, &mut back);
        }
        if let Some(s) = d.s.as_mut() {
            s.poll(now, &mut back);
        }
        for mut f in back {
            f.flags |= maint::FLAG_FROM_DRONE;
            let w = maint::pack(&f);
            assert!(hub.on_body(&w, now));
            // The GS's own frames heard back are ignored.
            let mut own = f.clone();
            own.flags &= !maint::FLAG_FROM_DRONE;
            hub.on_body(&maint::pack(&own), now);
        }
    }

    #[test]
    fn a_session_opens_streams_and_ends_with_the_status() {
        let hub = Hub::default();
        let mut d = Drone { s: None, video: 0 };
        let id = hub.open("tr a-z A-Z");
        assert_eq!(hub.info(id).unwrap().state, State::Opening);
        hub.write(id, b"hello\n");
        hub.finish(id);
        let mut got = Vec::new();
        for now in 0..200 {
            step(&hub, &mut d, now);
            hub.read(id, &mut got);
        }
        assert_eq!(got, b"HELLO\n");
        assert_eq!(hub.info(id).unwrap().state, State::Exited(7));
        assert_eq!(hub.unacked(id), 0);
        hub.close(id);
        for now in 200..205 {
            step(&hub, &mut d, now);
        }
        assert!(d.s.is_none() && hub.info(id).is_none());
    }

    #[test]
    fn the_video_wish_renews_and_turns_back_on() {
        let hub = Hub::default();
        let mut d = Drone { s: None, video: 0 };
        hub.set_video_off(true);
        step(&hub, &mut d, 0);
        assert_eq!(d.video, VIDEO_LEASE_MS);
        assert_eq!(hub.video_echo(), Some((VIDEO_LEASE_MS, 0)));
        d.video = 1;
        step(&hub, &mut d, 100);
        assert_eq!(d.video, 1, "no renewal before VIDEO_RENEW_MS");
        step(&hub, &mut d, 500);
        assert_eq!(d.video, VIDEO_LEASE_MS);
        hub.set_video_off(false);
        step(&hub, &mut d, 501);
        assert_eq!(d.video, 0);
    }

    #[test]
    fn a_close_from_the_drone_ends_the_session_and_other_bodies_pass() {
        let hub = Hub::default();
        let id = hub.open("x");
        hub.poll(0);
        let f = Frame {
            kind: maint::K_CLOSE,
            session: id,
            flags: maint::FLAG_FROM_DRONE,
            status: maint::STATUS_ARMED,
            ..Frame::default()
        };
        assert!(hub.on_body(&maint::pack(&f), 5));
        assert_eq!(
            hub.info(id).unwrap().state,
            State::Closed(maint::STATUS_ARMED)
        );
        assert!(!hub.on_body(&[0x43, 0x52, 18, 4, 0, 0], 6));
    }
}
