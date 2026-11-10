//! When the Android decode loop gives up on its MediaCodec and builds a new
//! one (`crates/pal-android` `decode_loop`). Pure: counts and times in, a
//! verdict out, so the host tests it.
//!
//! A codec in its error state fails every call from then on and never
//! recovers by itself; each AU fed to it would raise the damage count and
//! ask for another keyframe that cannot help. Two signs mark it:
//! - `FAIL_RUN` AUs in a row the codec refused (`Fed::Failed`) or took no
//!   input buffer for (`Fed::NoInput`: a codec that stopped consuming);
//! - an IRAP went in, `NO_OUTPUT_AUS` more AUs went in after it, and
//!   `NO_OUTPUT_MS` later still no frame came out. A working decoder puts out
//!   a frame for an IRAP whatever came before it, so neither the start (no
//!   IRAP yet) nor a loss waiting for its keyframe (P frames only) can fire
//!   this; and after a rebuild it needs a new IRAP that again yields nothing.
//!
//! While a run of refused AUs lasts, the damage count is held back from the
//! IDR driver (`Verdict::Keep { report: false }`): the run either breaks, and
//! its damage goes out as one rise, or it ends in a rebuild, which reports one
//! rise of its own. A rebuild that fails again before the new codec has put
//! out frames for `HEALTHY_MS` waits `BACKOFF_MS` (1 s, 2 s, then 4 s).

/// Refused AUs in a row that mark a dead codec. A codec in its error
/// state refuses every AU, so this only has to outlast a short burst (a
/// keyframe that keeps the buffers busy for a few frame times).
pub const FAIL_RUN: u32 = 8;
/// An IRAP with no frame out this long after it marks a dead codec.
pub const NO_OUTPUT_MS: u64 = 500;
/// AUs after that IRAP before the rule applies: a decoder that holds a frame
/// until more input comes has had its input.
pub const NO_OUTPUT_AUS: u32 = 3;
/// A codec that has put out frames this long ends a failure streak.
pub const HEALTHY_MS: u64 = 2000;
/// Wait before the second, third, and any later rebuild in a streak.
pub const BACKOFF_MS: [u64; 3] = [1000, 2000, 4000];

/// What became of one AU handed to the codec.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fed {
    /// In an input buffer.
    Queued,
    /// No input buffer in time.
    NoInput,
    /// Larger than the input buffer.
    Oversize,
    /// The codec returned an error.
    Failed,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cause {
    InputErrors,
    NoInput,
    NoOutput,
    OpenFailed,
}

impl Cause {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::InputErrors => "input_errors",
            Self::NoInput => "no_input",
            Self::NoOutput => "no_output",
            Self::OpenFailed => "open_failed",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// Keep the codec. `report`: its damage count may go to the IDR driver.
    Keep { report: bool },
    /// Drop the codec now and build the next one at `rebuild_at_ms`.
    /// `streak`: failures in a row, this one included.
    Rebuild {
        cause: Cause,
        rebuild_at_ms: u64,
        streak: u32,
    },
}

#[derive(Debug, Default)]
pub struct Watchdog {
    /// AUs in a row the codec refused or took no buffer for.
    run: u32,
    /// The run holds a refused one.
    run_failed: bool,
    /// The codec's output count at the last call.
    out_seen: u64,
    /// When this codec first put out a frame.
    first_out_ms: Option<u64>,
    /// (fed at, AUs queued after it) of the oldest IRAP with no frame out
    /// since.
    irap: Option<(u64, u32)>,
    /// Rebuilds in a row with no healthy codec between them.
    streak: u32,
}

impl Watchdog {
    /// One AU went to the codec. `irap`: its first slice is an IRAP; `out`:
    /// frames the codec has put out since it started (shown or dropped).
    pub fn fed(&mut self, now_ms: u64, fed: Fed, irap: bool, out: u64) -> Verdict {
        if out > self.out_seen {
            self.out_seen = out;
            self.irap = None;
            self.first_out_ms.get_or_insert(now_ms);
        }
        if self.first_out_ms.is_some_and(|t| now_ms >= t + HEALTHY_MS) {
            self.streak = 0;
        }
        if matches!(fed, Fed::Failed | Fed::NoInput) {
            self.run += 1;
            self.run_failed |= fed == Fed::Failed;
        } else {
            self.run = 0;
            self.run_failed = false;
        }
        if fed == Fed::Queued {
            if let Some((_, n)) = self.irap.as_mut() {
                *n += 1;
            } else if irap {
                self.irap = Some((now_ms, 0));
            }
        }
        if self.run >= FAIL_RUN {
            let cause = if self.run_failed {
                Cause::InputErrors
            } else {
                Cause::NoInput
            };
            return self.fail(cause, now_ms);
        }
        if self
            .irap
            .is_some_and(|(t, n)| n >= NO_OUTPUT_AUS && now_ms >= t + NO_OUTPUT_MS)
        {
            return self.fail(Cause::NoOutput, now_ms);
        }
        Verdict::Keep {
            report: self.run == 0,
        }
    }

    /// Building or starting the next codec failed.
    pub fn open_failed(&mut self, now_ms: u64) -> Verdict {
        self.fail(Cause::OpenFailed, now_ms)
    }

    fn fail(&mut self, cause: Cause, now_ms: u64) -> Verdict {
        self.streak += 1;
        let wait = match self.streak {
            1 => 0,
            s => BACKOFF_MS[(s as usize - 2).min(BACKOFF_MS.len() - 1)],
        };
        // The next codec starts from zero.
        *self = Self {
            streak: self.streak,
            ..Self::default()
        };
        Verdict::Rebuild {
            cause,
            rebuild_at_ms: now_ms + wait,
            streak: self.streak,
        }
    }
}

/// True when the AU's first slice NAL is an IRAP (types 16..21), the AU a
/// decoder can start from.
pub fn starts_with_irap(au: &[u8]) -> bool {
    let mut from = 0;
    while let Some((p, l)) = hevc::nal::find_start_code(au, from) {
        let h = p + l;
        let Some(&b) = au.get(h) else {
            return false;
        };
        let ty = hevc::nal::nal_type(b);
        if ty < 32 {
            return (16..=21).contains(&ty);
        }
        from = h;
    }
    false
}

/// At most one log line per `period_ms`; the lines held back in between are
/// counted for the next one.
#[derive(Debug)]
pub struct LogEvery {
    period_ms: u64,
    last_ms: Option<u64>,
    held: u64,
}

impl LogEvery {
    pub const fn new(period_ms: u64) -> Self {
        Self {
            period_ms,
            last_ms: None,
            held: 0,
        }
    }

    /// `Some(lines held back since the last one)` when a line may go out.
    pub fn hit(&mut self, now_ms: u64) -> Option<u64> {
        if self.last_ms.is_some_and(|t| now_ms < t + self.period_ms) {
            self.held += 1;
            return None;
        }
        self.last_ms = Some(now_ms);
        Some(std::mem::take(&mut self.held))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Feeds `n` AUs 10 ms apart from `t`, all with outcome `fed` and no
    /// output change; returns the last verdict and the time after.
    fn feed(w: &mut Watchdog, t: u64, n: u32, fed: Fed, out: u64) -> (Verdict, u64) {
        let mut v = Verdict::Keep { report: true };
        let mut t = t;
        for _ in 0..n {
            v = w.fed(t, fed, false, out);
            t += 10;
        }
        (v, t)
    }

    #[test]
    fn a_run_of_refused_aus_rebuilds_at_once_and_holds_the_damage() {
        let mut w = Watchdog::default();
        let (v, t) = feed(&mut w, 0, 10, Fed::Queued, 1);
        assert_eq!(v, Verdict::Keep { report: true });
        for i in 1..FAIL_RUN {
            assert_eq!(
                w.fed(t + i as u64, Fed::Failed, false, 1),
                Verdict::Keep { report: false }
            );
        }
        assert_eq!(
            w.fed(t + 100, Fed::Failed, false, 1),
            Verdict::Rebuild {
                cause: Cause::InputErrors,
                rebuild_at_ms: t + 100,
                streak: 1
            }
        );
    }

    #[test]
    fn a_short_burst_keeps_the_codec_and_reports_once_it_breaks() {
        let mut w = Watchdog::default();
        let (v, t) = feed(&mut w, 0, FAIL_RUN - 1, Fed::Failed, 0);
        assert_eq!(v, Verdict::Keep { report: false });
        assert_eq!(
            w.fed(t, Fed::Queued, false, 0),
            Verdict::Keep { report: true }
        );
        // The run starts over: another FAIL_RUN - 1 still keeps it.
        let (v, _) = feed(&mut w, t + 10, FAIL_RUN - 1, Fed::Failed, 0);
        assert_eq!(v, Verdict::Keep { report: false });
    }

    #[test]
    fn oversize_aus_and_a_missing_buffer_now_and_then_keep_the_codec() {
        let mut w = Watchdog::default();
        for i in 0..100 {
            assert_eq!(
                w.fed(i * 10, Fed::Oversize, false, 0),
                Verdict::Keep { report: true }
            );
        }
        // `debug.pal.dec_starve 5`: every 5th AU misses its buffer; its
        // damage goes out with the next AU the codec takes.
        for i in 0..100 {
            let fed = if i % 5 == 4 {
                Fed::NoInput
            } else {
                Fed::Queued
            };
            let v = w.fed(1000 + i * 10, fed, false, 0);
            assert_eq!(
                v,
                Verdict::Keep {
                    report: fed == Fed::Queued
                }
            );
        }
    }

    #[test]
    fn a_codec_that_stops_taking_input_is_rebuilt() {
        let mut w = Watchdog::default();
        let (v, t) = feed(&mut w, 0, FAIL_RUN, Fed::NoInput, 0);
        assert_eq!(
            v,
            Verdict::Rebuild {
                cause: Cause::NoInput,
                rebuild_at_ms: t - 10,
                streak: 1
            }
        );
        // A mixed run is the codec's error.
        let (_, t) = feed(&mut w, t, FAIL_RUN - 1, Fed::NoInput, 0);
        assert!(matches!(
            w.fed(t, Fed::Failed, false, 0),
            Verdict::Rebuild {
                cause: Cause::InputErrors,
                ..
            }
        ));
    }

    #[test]
    fn no_trigger_before_the_first_irap() {
        // Start-up: P frames with no reference, nothing comes out, for 5 s.
        let mut w = Watchdog::default();
        let (v, _) = feed(&mut w, 0, 500, Fed::Queued, 0);
        assert_eq!(v, Verdict::Keep { report: true });
    }

    #[test]
    fn no_trigger_while_a_loss_waits_for_its_keyframe() {
        let mut w = Watchdog::default();
        let mut t = 0;
        assert_eq!(
            w.fed(t, Fed::Queued, true, 0),
            Verdict::Keep { report: true }
        );
        // The IRAP and its P frames come out ...
        for out in 1..=60 {
            t += 10;
            w.fed(t, Fed::Queued, false, out);
        }
        // ... then the reference is lost and P frames yield nothing for 3 s.
        let (v, t) = feed(&mut w, t + 10, 300, Fed::Queued, 60);
        assert_eq!(v, Verdict::Keep { report: true });
        // The keyframe comes and decodes.
        assert_eq!(
            w.fed(t, Fed::Queued, true, 60),
            Verdict::Keep { report: true }
        );
        assert_eq!(
            w.fed(t + 20, Fed::Queued, false, 61),
            Verdict::Keep { report: true }
        );
        let (v, _) = feed(&mut w, t + 30, 100, Fed::Queued, 61);
        assert_eq!(v, Verdict::Keep { report: true });
    }

    #[test]
    fn an_irap_with_no_frame_out_rebuilds_after_500_ms() {
        let mut w = Watchdog::default();
        w.fed(0, Fed::Queued, true, 0);
        // Enough AUs but not yet 500 ms.
        let (v, _) = feed(&mut w, 10, 40, Fed::Queued, 0);
        assert_eq!(v, Verdict::Keep { report: true });
        // 500 ms but only on the AUs that follow.
        assert_eq!(
            w.fed(NO_OUTPUT_MS, Fed::Queued, false, 0),
            Verdict::Rebuild {
                cause: Cause::NoOutput,
                rebuild_at_ms: NO_OUTPUT_MS,
                streak: 1
            }
        );
    }

    #[test]
    fn the_no_output_rule_needs_aus_after_the_irap() {
        let mut w = Watchdog::default();
        w.fed(0, Fed::Queued, true, 0);
        // The link went quiet right after the IRAP: the first AUs after a
        // long gap do not count as the codec's failure yet.
        for i in 1..NO_OUTPUT_AUS {
            assert_eq!(
                w.fed(2000 + i as u64, Fed::Queued, false, 0),
                Verdict::Keep { report: true }
            );
        }
        // Output for the IRAP once more input came: healthy.
        assert_eq!(
            w.fed(2010, Fed::Queued, false, 1),
            Verdict::Keep { report: true }
        );
    }

    #[test]
    fn a_frame_out_clears_the_irap_clock() {
        let mut w = Watchdog::default();
        w.fed(0, Fed::Queued, true, 0);
        w.fed(20, Fed::Queued, false, 1);
        let (v, _) = feed(&mut w, 30, 200, Fed::Queued, 1);
        assert_eq!(v, Verdict::Keep { report: true });
    }

    #[test]
    fn failures_in_a_row_back_off_1_2_4_4_s() {
        let mut w = Watchdog::default();
        let mut t = 1000;
        let mut waits = Vec::new();
        for _ in 0..5 {
            let (v, t2) = feed(&mut w, t, FAIL_RUN, Fed::Failed, 0);
            let Verdict::Rebuild { rebuild_at_ms, .. } = v else {
                panic!("{v:?}");
            };
            let at = t2 - 10;
            waits.push(rebuild_at_ms - at);
            t = rebuild_at_ms;
        }
        assert_eq!(waits, [0, 1000, 2000, 4000, 4000]);
    }

    #[test]
    fn an_open_failure_counts_in_the_streak() {
        let mut w = Watchdog::default();
        let (v, t) = feed(&mut w, 0, FAIL_RUN, Fed::Failed, 0);
        assert!(matches!(v, Verdict::Rebuild { streak: 1, .. }));
        assert_eq!(
            w.open_failed(t),
            Verdict::Rebuild {
                cause: Cause::OpenFailed,
                rebuild_at_ms: t + 1000,
                streak: 2
            }
        );
    }

    #[test]
    fn a_healthy_codec_ends_the_streak() {
        let mut w = Watchdog::default();
        let (v, t) = feed(&mut w, 0, FAIL_RUN, Fed::Failed, 0);
        assert!(matches!(v, Verdict::Rebuild { streak: 1, .. }));
        // The new codec puts out frames, but not yet for HEALTHY_MS.
        let t = run_healthy(&mut w, t, 10);
        let (v, t) = feed(&mut w, t, FAIL_RUN, Fed::Failed, 10);
        assert!(matches!(v, Verdict::Rebuild { streak: 2, .. }), "{v:?}");
        // The next one puts out frames for HEALTHY_MS, then fails.
        let t2 = run_healthy(&mut w, t, HEALTHY_MS / 10 + 1);
        let (v, t3) = feed(&mut w, t2, FAIL_RUN, Fed::Failed, HEALTHY_MS / 10 + 1);
        assert_eq!(
            v,
            Verdict::Rebuild {
                cause: Cause::InputErrors,
                rebuild_at_ms: t3 - 10,
                streak: 1
            }
        );
    }

    /// `n` AUs 10 ms apart from `t`, an IRAP first, each with a frame out;
    /// returns the time after.
    fn run_healthy(w: &mut Watchdog, t: u64, n: u64) -> u64 {
        for i in 0..n {
            let v = w.fed(t + i * 10, Fed::Queued, i == 0, i);
            assert_eq!(v, Verdict::Keep { report: true });
        }
        t + n * 10
    }

    #[test]
    fn a_codec_with_no_output_never_ends_the_streak() {
        let mut w = Watchdog::default();
        feed(&mut w, 0, FAIL_RUN, Fed::Failed, 0);
        // 10 s of queued AUs, no IRAP, nothing out: no reset.
        let (_, t) = feed(&mut w, 100, 1000, Fed::Queued, 0);
        let (v, _) = feed(&mut w, t, FAIL_RUN, Fed::Failed, 0);
        assert!(matches!(v, Verdict::Rebuild { streak: 2, .. }), "{v:?}");
    }

    #[test]
    fn irap_found_after_parameter_sets() {
        let idr = [
            0, 0, 0, 1, 0x40, 1, 0xAA, 0, 0, 1, 0x42, 1, 0xBB, 0, 0, 1, 0x44, 1, 0, 0, 1, 0x26, 1,
            0xCC,
        ];
        assert!(starts_with_irap(&idr));
        let cra = [0, 0, 1, 21 << 1, 1, 0x80];
        assert!(starts_with_irap(&cra));
        // An SEI then a TRAIL_R slice.
        let p = [0, 0, 0, 1, 39 << 1, 1, 5, 0, 0, 1, 1 << 1, 1, 0x80];
        assert!(!starts_with_irap(&p));
        assert!(!starts_with_irap(&[]));
        assert!(!starts_with_irap(&[0, 0, 1]));
        assert!(!starts_with_irap(&[1, 2, 3, 4]));
    }

    #[test]
    fn log_every_holds_lines_and_counts_them() {
        let mut l = LogEvery::new(1000);
        assert_eq!(l.hit(0), Some(0));
        for t in 1..=199 {
            assert_eq!(l.hit(t * 5), None);
        }
        assert_eq!(l.hit(1000), Some(199));
        assert_eq!(l.hit(1001), None);
        assert_eq!(l.hit(5000), Some(1));
    }
}
