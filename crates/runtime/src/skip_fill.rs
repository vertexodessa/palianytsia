//! A base AU lost whole, filled with a skip picture before the decoder
//! (pal-pzj.39).
//!
//! When no body of a base AU arrives, the decoder meets the next picture
//! with one of its references missing: libavcodec invents a gray picture
//! and everything after it shows at about 13 dB until the next IDR.
//! A whole-frame skip picture in the lost one's place (every CTU skip,
//! `hevc::skip_pic`) keeps the reference chain: on the salvage capture at
//! 5 % loss the frames under 30 dB went from 497 to 1 (hole_policy, with
//! the lost AUs' pts taken from the loss-free run).
//!
//! Here the lost AU is inferred, not known: every base slot (the last
//! base AU's pts plus whole base intervals) that lies before the AU at
//! hand was lost, whatever layer that AU is. Filling only at the next
//! base AU came too late: with two temporal layers the enhance AU after
//! the lost base one refers to it, and libavcodec had already put a gray
//! picture under that POC, so the skip added nothing (salvage 5 %: 497
//! frames under 30 dB with or without it). The POC comes from the pts
//! through the camera
//! frame interval, and `SkipMaker::make` refuses a picture whose
//! references are not all held, so a wrong guess costs a refusal, not a
//! broken picture. Runs in the runtime between the core and the decoder
//! queue: the AU bytes out of the core (ref == rust) do not change.
use hevc::skip_pic::SkipMaker;
use pal_core::Au;

/// Base AUs filled in one gap at most; a longer outage waits for the IDR
/// the IDR driver asks for.
pub const MAX_FILL: u32 = 3;
/// Recent intervals kept for the medians.
const HIST: usize = 16;
/// A pts step longer than this is an outage or a restart, not a frame
/// interval.
const MAX_STEP_US: u32 = 200_000;

/// What to feed the decoder for one AU out of the core.
#[derive(Debug, Default)]
pub struct Feed {
    /// Skip pictures (pts, Annex-B) for base AUs lost whole before it.
    pub before: Vec<(u32, Vec<u8>)>,
    /// A skip picture to feed in its place (an incomplete one, `holes`).
    pub instead: Option<Vec<u8>>,
}

#[derive(Default)]
pub struct SkipFill {
    /// Also stand a skip picture in for an AU that arrived incomplete and
    /// unsalvaged (its prefix when none can be made).
    pub holes: bool,
    sk: SkipMaker,
    last_pts: Option<u32>,
    last_base: Option<u32>,
    /// Intervals between consecutive AUs of any layer (the camera's).
    frame_d: Vec<u32>,
    /// Intervals between consecutive base AUs.
    base_d: Vec<u32>,
    /// Skip pictures made, and lost base AUs for which none could be.
    pub made: u64,
    pub refused: u64,
    /// Incomplete AUs a skip picture stood in for.
    pub replaced: u64,
}

fn median(v: &[u32]) -> Option<u32> {
    let mut s = v.to_vec();
    s.sort_unstable();
    s.get(s.len() / 2).copied()
}

fn push(v: &mut Vec<u32>, d: u32) {
    if (1..=MAX_STEP_US).contains(&d) {
        v.push(d);
        if v.len() > HIST {
            v.remove(0);
        }
    }
}

impl SkipFill {
    /// `holes`: also stand a skip picture in for an incomplete AU.
    pub fn new(holes: bool) -> Self {
        Self {
            holes,
            ..Self::default()
        }
    }

    /// What to feed for `a`: a skip picture per base AU lost whole since
    /// the last base AU, then `a` itself or (`holes`) a skip picture in
    /// place of an incomplete one; notes what was fed.
    pub fn feed(&mut self, a: &Au) -> Feed {
        let irap = (16..=23).contains(&a.vcl_nal) || a.flags & 0x01 != 0;
        let discont = a.flags & 0x02 != 0;
        let mut out = Feed::default();
        // Only after a loss the core saw (a gap in the drone's frame ids):
        // a pts gap alone may be a frame the drone never sent, and a skip
        // picture there would push a real reference out of the DPB.
        if !irap && !discont && a.skipped_before > 0 {
            out.before = self.fill(a.pts_us);
        }
        if discont {
            *self = Self {
                holes: self.holes,
                made: self.made,
                refused: self.refused,
                replaced: self.replaced,
                ..Self::default()
            };
        }
        if let Some(l) = self.last_pts {
            push(&mut self.frame_d, a.pts_us.wrapping_sub(l));
        }
        self.last_pts = Some(a.pts_us);
        if a.sid == 0 {
            if let Some(l) = self.last_base {
                push(&mut self.base_d, a.pts_us.wrapping_sub(l));
            }
            self.last_base = Some(a.pts_us);
        }
        if self.holes && !irap && !a.complete && !a.salvaged {
            let frame = median(&self.frame_d).unwrap_or(1);
            if let Ok(p) = self.sk.make(&a.data, a.pts_us, frame) {
                self.sk.fed(&p, false, true, a.pts_us);
                self.replaced += 1;
                out.instead = Some(p);
                return out;
            }
        }
        self.sk
            .fed(&a.data, a.complete, a.complete || a.salvaged, a.pts_us);
        out
    }

    fn fill(&mut self, pts: u32) -> Vec<(u32, Vec<u8>)> {
        let mut out = Vec::new();
        let (Some(last), Some(step), Some(frame)) =
            (self.last_base, median(&self.base_d), median(&self.frame_d))
        else {
            return out;
        };
        let gap = pts.wrapping_sub(last);
        if gap > MAX_STEP_US * (MAX_FILL + 1) {
            return out;
        }
        // Base slots wholly before this AU, a quarter interval of pts
        // jitter allowed: 0 for the next base AU on time or an enhance AU
        // between, 1 for the AU after one lost base AU.
        let tol = step / 4;
        let lost = gap.saturating_sub(tol) / step;
        if lost == 0 || lost > MAX_FILL {
            return out;
        }
        for j in 1..=lost {
            let at = last.wrapping_add(j * step);
            match self.sk.make(&[], at, frame) {
                Ok(p) => {
                    self.sk.fed(&p, false, true, at);
                    self.last_base = Some(at);
                    self.made += 1;
                    out.push((at, p));
                }
                Err(_) => {
                    // A later skip would name the one not made; the slots
                    // are given up, so the next AU does not count them again.
                    self.refused += u64::from(lost - j + 1);
                    self.last_base = Some(last.wrapping_add(lost * step));
                    break;
                }
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two layers at 30 fps: base on even frames, enhance on odd.
    fn au(frame: u32, flags: u8) -> Au {
        lost_au(frame, flags, 1)
    }

    fn lost_au(frame: u32, flags: u8, skipped_before: u32) -> Au {
        Au {
            skipped_before,
            pts_us: 1_000_000 + frame * 33_333,
            sid: (frame % 2) as u8,
            flags,
            vcl_nal: u8::from(frame.is_multiple_of(2)),
            complete: true,
            ..Au::default()
        }
    }

    /// Lost base slots are counted on the first AU past them, whatever its
    /// layer; these AUs carry no parameter sets, so each one is refused,
    /// which is what makes the count visible.
    #[test]
    fn a_lost_base_slot_is_found_on_the_next_au_of_either_layer() {
        let mut f = SkipFill::default();
        for fr in 0..12 {
            assert!(f.feed(&lost_au(fr, 0, 0)).before.is_empty());
        }
        assert_eq!(f.refused, 0, "no gap, nothing to fill");
        // Base frame 12 lost: enhance 13 shows it.
        f.feed(&au(13, 0));
        assert_eq!(f.refused, 1);
        f.feed(&au(14, 0));
        assert_eq!(f.refused, 1, "counted once");
        // Base 16 and enhance 17 lost: base 18 shows one lost base slot.
        f.feed(&au(15, 0));
        f.feed(&au(18, 0));
        assert_eq!(f.refused, 2);
        // A keyframe after a gap starts afresh: nothing to fill before it.
        f.feed(&au(19, 0));
        f.feed(&au(22, 0x01));
        assert_eq!(f.refused, 2);
        // A pts gap with no loss behind it (the drone sent no frame there)
        // is left alone.
        f.feed(&lost_au(24, 0, 0));
        f.feed(&lost_au(27, 0, 0));
        assert_eq!(f.refused, 2);
        // An outage longer than MAX_FILL base slots waits for the IDR.
        f.feed(&lost_au(28, 0, 0));
        f.feed(&lost_au(29, 0, 0));
        f.feed(&au(29 + 2 * (MAX_FILL + 2), 0));
        assert_eq!(f.refused, 2);
    }
}
