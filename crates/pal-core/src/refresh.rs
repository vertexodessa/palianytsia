//! When to ask the drone for an IDR (mabur `gs/src/refresh_requester.h`).
//!
//! The encoder runs with no GOP, so a damaged base picture stays damaged
//! until an IRAP arrives. The request is the RCF's `idr_epoch` byte, which
//! the drone serves once per change; this type only decides when the byte
//! changes. The host feeds it per finished AU and with decoder damage
//! reports, then hands `epoch()` to `GsCore::set_idr_requests`.
//!
//! Arms on: a base AU finished incomplete (reason hole), a base AU rebuilt
//! by slice salvage (a filled band is a stale reference; reason salvaged),
//! whole AUs skipped before this one (reason lost), or an explicit request.
//! Never on enhance damage: nothing references an enhance frame.
//! Clears on a complete, unsalvaged base AU whose first slice NAL is an
//! IRAP (16..21). While armed the byte is bumped again, at least `RETRY_MS`
//! apart and further apart with each try: see `retry_wait_ms`. A request
//! that `STUCK_AFTER` retries have not healed is stuck (`stuck`): the link
//! cannot bring a keyframe in whole, so its retries back off to seconds
//! instead of queueing one keyframe behind the next. The C++ requester
//! keeps the fixed `RETRY_MS`; the parity gate is unaffected because both
//! cores here are driven by this one.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reason {
    Hole,
    Salvaged,
    Lost,
    Start,
    Resync,
    Decoder,
    Dvr,
}

pub const REASONS: usize = 7;

impl Reason {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Hole => "hole",
            Self::Salvaged => "salvaged",
            Self::Lost => "lost",
            Self::Start => "start",
            Self::Resync => "resync",
            Self::Decoder => "decoder",
            Self::Dvr => "dvr",
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Counters {
    /// Epoch bumps that started a request (one per arming).
    pub requests: u64,
    /// Bumps while already armed.
    pub retries: u64,
    /// Clearing IRAPs.
    pub heals: u64,
    /// Arming to clearing IRAP, last heal.
    pub heal_ms_last: u64,
    pub by_reason: [u64; REASONS],
}

/// 'I' per bump (a retry repeats the arming reason, `retries` = its
/// ordinal), 'S' once per request when it turns stuck (after that retry's
/// 'I'), 'H' per clearing IRAP (`retries` = how many it took).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Event {
    pub kind: char,
    pub reason: Reason,
    pub epoch: u8,
    pub t_ms: u64,
    pub heal_ms: u64,
    pub retries: u64,
}

pub const RETRY_MS: u64 = 300;
/// The longest wait between two tries of one request.
pub const RETRY_MAX_MS: u64 = 1000;
/// Tries after which the wait stops doubling.
const RETRY_DOUBLINGS: u64 = 1;
/// Base-layer repairs an IDR carries on top of its bytes, as a share.
const IDR_FEC_OVERHEAD: f64 = 0.5;
/// Retries without a heal after which a request is stuck: five keyframes
/// asked for and none came in whole, where one heals in about 150 ms. On a
/// fast rung the waits before it are 300 and 600 ms, so this is about 2 s.
pub const STUCK_AFTER: u64 = 4;
/// A stuck request's waits: `RETRY_MAX_MS` doubled per retry this many
/// times (2 s, then 4 s), or the keyframe's span when that is longer.
const STUCK_DOUBLINGS: u64 = 2;

#[derive(Debug)]
pub struct RefreshRequester {
    armed: bool,
    reason: Reason,
    epoch: u8,
    armed_since_ms: u64,
    last_bump_ms: u64,
    retries_this: u64,
    /// Armed too soon after the last bump: the request's own bump is owed.
    first_owed: bool,
    /// Bytes of the last clean IRAP; 0 until one is seen.
    irap_bytes: u64,
    /// PHY rate of the current rung; 0 until known.
    link_mbps: f64,
    c: Counters,
    events: Vec<Event>,
}

impl Default for RefreshRequester {
    fn default() -> Self {
        Self {
            armed: false,
            reason: Reason::Hole,
            epoch: 0,
            armed_since_ms: 0,
            last_bump_ms: 0,
            retries_this: 0,
            first_owed: false,
            irap_bytes: 0,
            link_mbps: 0.0,
            c: Counters::default(),
            events: Vec::new(),
        }
    }
}

impl RefreshRequester {
    /// One finished AU. `skipped_before`: whole AUs the stream skipped right
    /// before it (0 when the core does not report it).
    pub fn note_au(
        &mut self,
        sid: u8,
        complete: bool,
        salvaged: bool,
        vcl_nal: u8,
        skipped_before: u32,
        now_ms: u64,
    ) {
        let clean = complete && !salvaged;
        if sid == 0 && clean && (16..=21).contains(&vcl_nal) {
            self.clear(now_ms);
            return;
        }
        if skipped_before > 0 {
            self.arm(Reason::Lost, now_ms);
        }
        if sid != 0 || clean {
            return;
        }
        self.arm(
            if salvaged {
                Reason::Salvaged
            } else {
                Reason::Hole
            },
            now_ms,
        );
    }

    pub fn request(&mut self, r: Reason, now_ms: u64) {
        self.arm(r, now_ms);
    }

    /// The size of the last clean IRAP, the best guess at the next one.
    pub fn set_irap_bytes(&mut self, bytes: u64) {
        self.irap_bytes = bytes;
    }

    /// The PHY rate of the current rung; 0 = unknown.
    pub fn set_link_mbps(&mut self, mbps: f64) {
        self.link_mbps = mbps;
    }

    /// Twice the air time of an IDR at the current rung: the span a request
    /// needs for its IDR to be sent and the P frames behind it to drain. An
    /// IDR requested sooner lands in the queue behind the previous one, and
    /// at MCS0 that is what keeps the queue full and every IDR damaged.
    fn idr_span_ms(&self) -> u64 {
        self.idr_span_raw_ms().min(RETRY_MAX_MS)
    }

    /// `idr_span_ms` without the cap: only a stuck request waits this long.
    fn idr_span_raw_ms(&self) -> u64 {
        if self.link_mbps <= 0.0 {
            return 0;
        }
        let bits = self.irap_bytes as f64 * 8.0 * (1.0 + IDR_FEC_OVERHEAD);
        (2.0 * bits / (self.link_mbps * 1000.0)) as u64
    }

    fn retry_wait_ms(&self) -> u64 {
        if self.retries_this >= STUCK_AFTER {
            let k = (self.retries_this - STUCK_AFTER + 1).min(STUCK_DOUBLINGS);
            return (RETRY_MAX_MS << k).max(self.idr_span_raw_ms());
        }
        let base = RETRY_MS.max(self.idr_span_ms());
        (base << self.retries_this.min(RETRY_DOUBLINGS)).min(RETRY_MAX_MS)
    }

    /// Retry cadence; call once per loop iteration while in a session.
    pub fn poll(&mut self, now_ms: u64) {
        if !self.armed {
            return;
        }
        if self.first_owed {
            if now_ms >= self.last_bump_ms + self.idr_span_ms() {
                self.first_owed = false;
                self.c.requests += 1;
                self.bump(now_ms);
            }
            return;
        }
        if now_ms < self.last_bump_ms + self.retry_wait_ms() {
            return;
        }
        self.c.retries += 1;
        self.retries_this += 1;
        self.bump(now_ms);
        if self.retries_this == STUCK_AFTER {
            self.push(now_ms, 'S', 0);
        }
    }

    /// The retries so far of a request that is stuck (see `STUCK_AFTER`),
    /// None while there is none.
    pub fn stuck(&self) -> Option<u64> {
        (self.armed && self.retries_this >= STUCK_AFTER).then_some(self.retries_this)
    }

    pub fn epoch(&self) -> u8 {
        self.epoch
    }
    pub fn armed(&self) -> bool {
        self.armed
    }
    pub fn counters(&self) -> &Counters {
        &self.c
    }
    pub fn take_events(&mut self) -> Vec<Event> {
        std::mem::take(&mut self.events)
    }

    fn arm(&mut self, r: Reason, now_ms: u64) {
        self.c.by_reason[r as usize] += 1;
        if self.armed {
            return;
        }
        self.armed = true;
        self.reason = r;
        self.armed_since_ms = now_ms;
        self.retries_this = 0;
        // A heal followed by a new hole must not start an IDR on top of the
        // one just asked for either.
        if now_ms < self.last_bump_ms + self.idr_span_ms() {
            self.first_owed = true;
            return;
        }
        self.c.requests += 1;
        self.bump(now_ms);
    }
    fn bump(&mut self, now_ms: u64) {
        self.epoch = self.epoch.wrapping_add(1);
        self.last_bump_ms = now_ms;
        self.push(now_ms, 'I', 0);
    }
    fn push(&mut self, t_ms: u64, kind: char, heal_ms: u64) {
        self.events.push(Event {
            kind,
            reason: self.reason,
            epoch: self.epoch,
            t_ms,
            heal_ms,
            retries: self.retries_this,
        });
    }
    fn clear(&mut self, now_ms: u64) {
        if !self.armed {
            return;
        }
        self.armed = false;
        self.first_owed = false;
        // Stamps come from two clocks (body RX time, tick time), so an IRAP
        // body can carry a stamp older than the arming tick.
        let heal = now_ms.saturating_sub(self.armed_since_ms);
        self.c.heals += 1;
        self.c.heal_ms_last = heal;
        self.push(now_ms, 'H', heal);
    }
}

/// The host side of the request path, shared by `pal live` and pal-desktop:
/// AUs and decoder damage in, `GsCore::set_idr_requests` out. Both cores get
/// identical calls, so the parity gate covers it (`pal replay --idr`).
/// `skipped_before` and `vcl_nal` come with the AU from both cores
/// (webgs::Au carries them since the palianytsia mabur fork), as maburgs
/// reads them off FrameStream.
#[derive(Debug, Default)]
pub struct IdrDriver {
    pub r: RefreshRequester,
    applied: u64,
    dec_err_seen: u64,
    /// AUs seen so far; AU number n is the n-th `on_au` call (1-based).
    aus: u64,
    /// Number of the AU whose IRAP last cleared a request.
    healed_at: u64,
    started: bool,
    in_discont: bool,
}

impl IdrDriver {
    pub fn on_au(&mut self, a: &crate::Au, now_us: u64) {
        self.aus += 1;
        let vcl = a.vcl_nal;
        let irap = a.sid == 0 && a.complete && !a.salvaged && (16..=21).contains(&vcl);
        // The player's "start" (maburgs take_refresh): a decoder that joins
        // mid-stream has no reference until an IRAP, and the encoder runs
        // with no GOP, so ask unless the first AU is that IRAP. The first AU
        // of a DISCONT run (producer restart: new pts/frame_id base, maybe a
        // new resolution) is the same join, as "resync".
        let discont = a.flags & pal_wire::frame_wire::FLAG_DISCONT != 0;
        let new_run = discont && !self.in_discont;
        self.in_discont = discont;
        if !irap && (!self.started || new_run) {
            let r = if self.started {
                Reason::Resync
            } else {
                Reason::Start
            };
            self.r.request(r, now_us / 1000);
        }
        self.started = true;
        if irap {
            self.r.set_irap_bytes(a.data.len() as u64);
        }
        let was_armed = self.r.armed();
        self.r.note_au(
            a.sid,
            a.complete,
            a.salvaged,
            vcl,
            a.skipped_before,
            now_us / 1000,
        );
        if was_armed && !self.r.armed() {
            self.healed_at = self.aus;
        }
    }
    /// AUs seen so far: the number to tag decoder work with.
    pub fn aus(&self) -> u64 {
        self.aus
    }
    /// The decoder's cumulative error count, and the number (`aus()` at
    /// submit time) of the newest AU it had decoded when the count was
    /// read. Any rise is one damage report, unless that AU went in before
    /// the last clearing IRAP: a decoder behind a queue reports the old
    /// chain's errors after the heal, and those need no new IDR.
    pub fn on_decoder_errors(&mut self, total: u64, au: u64, now_us: u64) {
        if total > self.dec_err_seen {
            self.dec_err_seen = total;
            if au >= self.healed_at {
                self.r.request(Reason::Decoder, now_us / 1000);
            }
        }
    }
    /// After each core tick: the retry clock, then the epoch into the core.
    pub fn step(&mut self, now_us: u64, core: &mut dyn crate::GsCore) -> Vec<Event> {
        self.r.poll(now_us / 1000);
        // The cumulative bump count, as WebGs takes it: `idr_req` in the
        // stats stays monotonic and the RCF byte is its low 8 bits (= epoch).
        let c = self.r.counters();
        let total = c.requests + c.retries;
        if total != self.applied {
            self.applied = total;
            core.set_idr_requests(total as u32);
            let (mcs, bw) = core.link_op();
            self.r.set_link_mbps(link_mbps(mcs, bw));
        }
        self.r.take_events()
    }
}

/// The rung's PHY rate; 0 while the core knows no MCS (it reports -1).
fn link_mbps(mcs: i32, bw: i32) -> f64 {
    match (u8::try_from(mcs), u8::try_from(bw)) {
        (Ok(mcs), Ok(bw)) => {
            pal_wire::profile::phy_rate_mbps(pal_wire::profile::PhyMode::Ht, mcs, bw)
        }
        _ => 0.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn retries_space_out_and_cap() {
        let mut r = RefreshRequester::default();
        r.request(Reason::Hole, 1000);
        r.poll(1299);
        assert_eq!(r.epoch(), 1);
        r.poll(1300);
        assert_eq!(r.epoch(), 2);
        r.poll(1899);
        assert_eq!(r.epoch(), 2);
        r.poll(1900);
        assert_eq!(r.epoch(), 3);
        // One doubling only, and a slow link is capped.
        r.poll(2499);
        assert_eq!(r.epoch(), 3);
        r.poll(2500);
        assert_eq!(r.epoch(), 4);
        r.set_irap_bytes(400_000);
        r.set_link_mbps(6.5);
        r.poll(3499);
        assert_eq!(r.epoch(), 4);
        r.poll(3500);
        assert_eq!(r.epoch(), 5);
    }

    #[test]
    fn retry_waits_for_a_big_idr_on_a_slow_rung() {
        let mut r = RefreshRequester::default();
        // 102 KB at MCS0 20 MHz: 2 x 188 ms of air.
        r.set_irap_bytes(102_000);
        r.set_link_mbps(6.5);
        r.request(Reason::Hole, 10_000);
        r.poll(10_375);
        assert_eq!(r.epoch(), 1);
        r.poll(10_376);
        assert_eq!(r.epoch(), 2);
        // The same IDR on a fast rung keeps the RETRY_MS floor.
        let mut f = RefreshRequester::default();
        f.set_irap_bytes(102_000);
        f.set_link_mbps(26.0);
        f.request(Reason::Hole, 10_000);
        f.poll(10_300);
        assert_eq!(f.epoch(), 2);
    }

    #[test]
    fn new_hole_right_after_a_request_waits_for_the_idr_in_flight() {
        let mut r = RefreshRequester::default();
        r.set_irap_bytes(102_000);
        r.set_link_mbps(6.5);
        r.request(Reason::Hole, 10_000);
        r.note_au(0, true, false, 19, 0, 10_100);
        assert!(!r.armed());
        r.note_au(0, false, false, 1, 0, 10_200);
        assert!(r.armed());
        assert_eq!(r.epoch(), 1);
        assert_eq!(r.counters().requests, 1);
        r.poll(10_375);
        assert_eq!(r.epoch(), 1);
        r.poll(10_376);
        assert_eq!(r.epoch(), 2);
        assert_eq!((r.counters().requests, r.counters().retries), (2, 0));
    }

    #[test]
    fn owed_request_heals_without_a_bump() {
        let mut r = RefreshRequester::default();
        r.set_irap_bytes(102_000);
        r.set_link_mbps(6.5);
        r.request(Reason::Hole, 10_000);
        r.note_au(0, true, false, 19, 0, 10_100);
        r.note_au(0, false, false, 1, 0, 10_200);
        r.note_au(0, true, false, 19, 0, 10_300);
        r.poll(11_000);
        assert_eq!(r.epoch(), 1);
        assert_eq!(r.counters().requests, 1);
    }

    #[test]
    fn link_rate_comes_from_the_stats_line() {
        assert_eq!(link_mbps(0, 20), 6.5);
        assert_eq!(link_mbps(3, 40), 54.0);
        assert_eq!(link_mbps(-1, 20), 0.0);
    }

    #[test]
    fn hole_arms_retries_and_irap_heals() {
        let mut r = RefreshRequester::default();
        r.note_au(0, false, false, 1, 0, 100);
        assert!(r.armed());
        assert_eq!(r.epoch(), 1);
        // A second hole while armed only counts.
        r.note_au(0, false, false, 1, 0, 150);
        assert_eq!(r.epoch(), 1);
        r.poll(399);
        assert_eq!(r.epoch(), 1);
        r.poll(400);
        assert_eq!(r.epoch(), 2);
        // TRAIL_R does not clear; enhance damage never arms.
        r.note_au(0, true, false, 1, 0, 420);
        assert!(r.armed());
        r.note_au(0, true, false, 19, 0, 450);
        assert!(!r.armed());
        let c = r.counters();
        assert_eq!(
            (c.requests, c.retries, c.heals, c.heal_ms_last),
            (1, 1, 1, 350)
        );
        assert_eq!(c.by_reason[Reason::Hole as usize], 2);
        let ev = r.take_events();
        assert_eq!(ev.len(), 3);
        assert_eq!((ev[1].kind, ev[1].retries), ('I', 1));
        assert_eq!((ev[2].kind, ev[2].heal_ms, ev[2].retries), ('H', 350, 1));
        r.note_au(1, false, false, 1, 0, 500);
        assert!(!r.armed());
    }

    #[test]
    fn salvaged_irap_does_not_clear_and_skip_before_irap_heals() {
        let mut r = RefreshRequester::default();
        r.note_au(0, false, true, 19, 0, 10);
        assert!(r.armed());
        assert_eq!(r.counters().by_reason[Reason::Salvaged as usize], 1);
        r.note_au(0, true, false, 19, 3, 20);
        assert!(!r.armed());
        assert_eq!(r.counters().by_reason[Reason::Lost as usize], 0);
        r.note_au(0, true, false, 1, 2, 30);
        assert!(r.armed());
        assert_eq!(r.counters().by_reason[Reason::Lost as usize], 1);
    }

    #[test]
    fn irap_stamped_before_arming_heals_in_zero_ms() {
        let mut r = RefreshRequester::default();
        r.request(Reason::Decoder, 1000);
        r.note_au(0, true, false, 19, 0, 990);
        assert!(!r.armed());
        assert_eq!(r.counters().heal_ms_last, 0);
    }

    /// A keyframe that never arrives whole: 30 s of retries. Without the stuck back-off they
    /// come every 600 ms from the second on (50 in 30 s); with it the request goes stuck at the
    /// fourth retry and asks every 4 s after (11 in all), and says so once.
    #[test]
    fn a_request_that_does_not_heal_backs_off_and_says_stuck() {
        let mut r = RefreshRequester::default();
        r.request(Reason::Hole, 0);
        let mut bumps = vec![];
        for t in 0..=30_000 {
            let e = r.epoch();
            r.poll(t);
            if r.epoch() != e {
                bumps.push(t);
            }
        }
        assert_eq!(
            bumps,
            [300, 900, 1500, 2100, 4100, 8100, 12100, 16100, 20100, 24100, 28100]
        );
        assert_eq!(r.stuck(), Some(11));
        let ev = r.take_events();
        let stuck: Vec<_> = ev.iter().filter(|e| e.kind == 'S').collect();
        assert_eq!(stuck.len(), 1);
        assert_eq!((stuck[0].t_ms, stuck[0].retries), (2100, STUCK_AFTER));
        // The heal ends it; the next request starts with the short waits again.
        r.note_au(0, true, false, 19, 0, 30_100);
        assert_eq!(r.stuck(), None);
        r.request(Reason::Hole, 40_000);
        r.poll(40_300);
        assert_eq!(r.counters().retries, 12);
    }

    /// A stuck request waits for the keyframe's whole span, past the 1 s cap.
    #[test]
    fn a_stuck_request_waits_out_a_keyframe_longer_than_its_back_off() {
        let mut r = RefreshRequester::default();
        // 400 KB at MCS0 20 MHz: 1476 ms of span, capped at 1 s while not stuck.
        r.set_irap_bytes(400_000);
        r.set_link_mbps(6.5);
        r.request(Reason::Hole, 10_000);
        for t in [11_000, 12_000, 13_000, 14_000] {
            r.poll(t);
        }
        assert_eq!(r.stuck(), Some(4));
        r.poll(14_000 + 1999);
        assert_eq!(r.stuck(), Some(4));
        r.poll(14_000 + 2000);
        assert_eq!(r.stuck(), Some(5));
        // 2 MB: a 7384 ms span outlasts the 4 s back-off.
        r.set_irap_bytes(2_000_000);
        r.poll(16_000 + 4000);
        assert_eq!(r.stuck(), Some(5));
        r.poll(16_000 + 7384);
        assert_eq!(r.stuck(), Some(6));
    }

    #[test]
    fn epoch_wraps() {
        let mut r = RefreshRequester::default();
        r.request(Reason::Decoder, 0);
        for i in 1..=255u64 {
            r.poll(i * (RETRY_MAX_MS << STUCK_DOUBLINGS));
        }
        assert_eq!(r.epoch(), 0);
        // IdrDriver hands the core the cumulative count, which keeps going.
        let c = r.counters();
        assert_eq!(c.requests + c.retries, 256);
    }

    fn au(sid: u8, complete: bool, flags: u8, nal: u8) -> crate::Au {
        crate::Au {
            data: vec![0, 0, 0, 1, nal << 1, 1, 0xAA],
            pts_us: 0,
            sid,
            flags,
            complete,
            salvaged: false,
            skipped_before: 0,
            vcl_nal: nal,
        }
    }

    #[test]
    fn driver_arms_lost_on_skipped_aus() {
        let mut d = IdrDriver::default();
        d.on_au(&au(0, true, 0, 19), 1_000); // clean IRAP start
        assert!(!d.r.armed());
        // Two whole AUs vanished before this complete P picture.
        d.on_au(
            &crate::Au {
                skipped_before: 2,
                ..au(0, true, 0, 1)
            },
            2_000,
        );
        assert!(d.r.armed());
        assert_eq!(d.r.counters().by_reason[Reason::Lost as usize], 1);
    }

    #[test]
    fn driver_start_resync_on_discont_run() {
        let mut d = IdrDriver::default();
        d.on_au(&au(0, true, 0, 1), 1_000);
        assert!(d.r.armed());
        d.on_au(&au(0, true, 0, 19), 2_000);
        assert!(!d.r.armed());
        // A producer restart that opens on a P picture: resync.
        d.on_au(&au(0, true, 2, 1), 3_000);
        assert!(d.r.armed());
        assert_eq!(d.r.counters().by_reason[Reason::Resync as usize], 1);
        // The rest of the run does not re-arm; one opening on an IRAP asks nothing.
        d.on_au(&au(0, true, 2, 19), 4_000);
        d.on_au(&au(0, true, 2, 1), 5_000);
        d.on_au(&au(0, true, 0, 1), 6_000);
        d.on_au(&au(0, true, 2, 19), 7_000);
        assert!(!d.r.armed());
        assert_eq!(d.r.counters().requests, 2);
    }

    #[test]
    fn driver_ignores_decoder_errors_from_before_the_heal() {
        let mut d = IdrDriver::default();
        d.on_au(&au(0, false, 0, 1), 1_000); // start + hole
        d.on_au(&au(0, true, 0, 1), 2_000);
        d.on_au(&au(0, true, 0, 19), 3_000); // AU 3 heals
        assert!(!d.r.armed());
        // The queued decoder reports errors from AU 2 after the heal.
        d.on_decoder_errors(4, 2, 3_500);
        assert!(!d.r.armed());
        // Damage on the IRAP itself or later is real.
        d.on_decoder_errors(5, 3, 3_600);
        assert!(d.r.armed());
        assert_eq!(d.r.counters().by_reason[Reason::Decoder as usize], 1);
    }
}
