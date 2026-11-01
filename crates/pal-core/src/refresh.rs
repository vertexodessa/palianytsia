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
//! IRAP (16..21). While armed the byte is bumped again every `RETRY_MS`.

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
/// ordinal), 'H' per clearing IRAP (`retries` = how many it took).
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

#[derive(Debug)]
pub struct RefreshRequester {
    armed: bool,
    reason: Reason,
    epoch: u8,
    armed_since_ms: u64,
    last_bump_ms: u64,
    retries_this: u64,
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

    /// Retry cadence; call once per loop iteration while in a session.
    pub fn poll(&mut self, now_ms: u64) {
        if !self.armed || now_ms < self.last_bump_ms + RETRY_MS {
            return;
        }
        self.c.retries += 1;
        self.retries_this += 1;
        self.bump(now_ms);
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
        self.c.requests += 1;
        self.bump(now_ms);
    }
    fn bump(&mut self, now_ms: u64) {
        self.epoch = self.epoch.wrapping_add(1);
        self.last_bump_ms = now_ms;
        self.events.push(Event {
            kind: 'I',
            reason: self.reason,
            epoch: self.epoch,
            t_ms: now_ms,
            heal_ms: 0,
            retries: self.retries_this,
        });
    }
    fn clear(&mut self, now_ms: u64) {
        if !self.armed {
            return;
        }
        self.armed = false;
        let heal = now_ms - self.armed_since_ms;
        self.c.heals += 1;
        self.c.heal_ms_last = heal;
        self.events.push(Event {
            kind: 'H',
            reason: self.reason,
            epoch: self.epoch,
            t_ms: now_ms,
            heal_ms: heal,
            retries: self.retries_this,
        });
    }
}

/// The host side of the request path, shared by `pal live` and pal-desktop:
/// AUs and decoder damage in, `GsCore::set_idr_requests` out. Both cores get
/// identical calls, so the parity gate covers it (`pal replay --idr`).
/// Deviation from maburgs: `skipped_before` is not reported through
/// `GsCore` (webgs::Au has no such field), so whole-AU skips reach this
/// only as decoder damage.
#[derive(Debug, Default)]
pub struct IdrDriver {
    pub r: RefreshRequester,
    applied: u8,
    dec_err_seen: u64,
    started: bool,
}

impl IdrDriver {
    pub fn on_au(&mut self, a: &crate::Au, now_us: u64) {
        let vcl = first_vcl_nal(&a.data);
        // The player's "start" (maburgs take_refresh): a decoder that joins
        // mid-stream has no reference until an IRAP, and the encoder runs
        // with no GOP, so ask unless the first AU is that IRAP.
        if !self.started {
            self.started = true;
            let irap = a.sid == 0 && a.complete && !a.salvaged && (16..=21).contains(&vcl);
            if !irap {
                self.r.request(Reason::Start, now_us / 1000);
            }
        }
        self.r
            .note_au(a.sid, a.complete, a.salvaged, vcl, 0, now_us / 1000);
    }
    /// The decoder's cumulative error count; any rise is one damage report.
    pub fn on_decoder_errors(&mut self, total: u64, now_us: u64) {
        if total > self.dec_err_seen {
            self.dec_err_seen = total;
            self.r.request(Reason::Decoder, now_us / 1000);
        }
    }
    /// After each core tick: the retry clock, then the epoch into the core.
    pub fn step(&mut self, now_us: u64, core: &mut dyn crate::GsCore) -> Vec<Event> {
        self.r.poll(now_us / 1000);
        if self.r.epoch() != self.applied {
            self.applied = self.r.epoch();
            core.set_idr_requests(u32::from(self.applied));
        }
        self.r.take_events()
    }
}

/// NAL type of the first slice (VCL, type < 32) in an Annex-B AU; 0xFF if
/// it carries none. maburgs reads the same off FrameStream (`lat.vcl_nal`).
pub fn first_vcl_nal(au: &[u8]) -> u8 {
    hevc::nal::split_nals(au)
        .iter()
        .find(|v| v.ty < 32)
        .map_or(0xFF, |v| v.ty)
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn epoch_wraps() {
        let mut r = RefreshRequester::default();
        r.request(Reason::Decoder, 0);
        for i in 1..=255u64 {
            r.poll(i * RETRY_MS);
        }
        assert_eq!(r.epoch(), 0);
    }

    #[test]
    fn vcl_nal_skips_parameter_sets() {
        let au = [0, 0, 0, 1, 0x40, 1, 0xAA, 0, 0, 1, 0x26, 1, 0xBB];
        assert_eq!(first_vcl_nal(&au), 19);
        assert_eq!(first_vcl_nal(&[0, 0, 1, 0x40, 1]), 0xFF);
    }
}
