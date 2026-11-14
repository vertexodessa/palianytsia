//! When the GS DVR records (spec F12): from the FC's arm to a tail after its
//! disarm, or the whole run for `pal replay --dvr`. The files and the muxing
//! are the `dvr` crate's; this decides start and stop, names the files, and
//! keeps the volume from filling up.

use std::path::{Path, PathBuf};

use dvr::{Recorder, Sealed, State};

/// Seconds of video kept after a disarm: the landing and the crash.
pub const TAIL_US: u64 = 3_000_000;
/// No recording starts with less room than this, and one stops below
/// `MIN_FREE / 2`: the flight log shares the volume.
pub const MIN_FREE: u64 = 1 << 30;
const SPACE_CHECK_US: u64 = 5_000_000;

pub enum When {
    /// Armed (FC HEARTBEAT) to `TAIL_US` after a disarm. A HEARTBEAT that
    /// goes stale (link loss) changes nothing.
    OnArm { dir: PathBuf, stem: String },
    /// From the first AU to the end, into one file (replay, CLI).
    Always(PathBuf),
}

/// What happened in one `on_au`, for the flight log and the IDR request.
#[derive(Default)]
pub struct Step {
    /// A recording was armed: ask the drone for an IDR (the GDR encoder sends
    /// one only every few seconds, and the file waits for it).
    pub started: Option<PathBuf>,
    pub sealed: Vec<Sealed>,
    /// Why a recording could not start or stopped early.
    pub error: Option<String>,
}

pub struct Dvr {
    when: When,
    rec: Recorder,
    /// Files made in this session (the next file's number).
    n: u32,
    disarm_at: Option<u64>,
    /// An arm whose recording failed: no retry until the next arm.
    failed_arm: bool,
    space_at: u64,
    free: fn(&Path) -> Option<u64>,
}

impl Dvr {
    pub fn new(when: When) -> Self {
        Self::with_free(when, crate::free_bytes)
    }

    fn with_free(when: When, free: fn(&Path) -> Option<u64>) -> Self {
        Self {
            when,
            rec: Recorder::default(),
            n: 0,
            disarm_at: None,
            failed_arm: false,
            space_at: 0,
            free,
        }
    }

    fn dir(&self) -> &Path {
        match &self.when {
            When::OnArm { dir, .. } => dir,
            When::Always(p) => p.parent().unwrap_or(Path::new(".")),
        }
    }

    fn wants(&mut self, armed: Option<bool>, now_us: u64) -> bool {
        match &self.when {
            When::Always(_) => true,
            When::OnArm { .. } => match armed {
                Some(true) => {
                    self.disarm_at = None;
                    true
                }
                Some(false) => {
                    self.failed_arm = false;
                    if self.rec.state() == State::Off {
                        return false;
                    }
                    let t = *self.disarm_at.get_or_insert(now_us);
                    now_us.saturating_sub(t) < TAIL_US
                }
                None => matches!(self.rec.state(), State::WaitSync | State::Recording),
            },
        }
    }

    fn start(&mut self, step: &mut Step) {
        let dir = self.dir().to_path_buf();
        if let Err(e) = std::fs::create_dir_all(&dir) {
            self.failed_arm = true;
            step.error = Some(format!("{}: {e}", dir.display()));
            return;
        }
        if let Some(free) = (self.free)(&dir).filter(|&f| f < MIN_FREE) {
            self.failed_arm = true;
            step.error = Some(format!(
                "{} MB free, {} MB needed",
                free >> 20,
                MIN_FREE >> 20
            ));
            return;
        }
        self.n += 1;
        let path = match &self.when {
            When::OnArm { dir, stem } => dir.join(format!("{stem}-{}.mp4", self.n)),
            When::Always(p) => p.clone(),
        };
        let mut next = Some(path.clone());
        let base = path.clone();
        let mut roll = 0;
        let threaded = matches!(self.when, When::OnArm { .. });
        self.rec.start(Box::new(move || {
            // A size change mid-recording opens `<name>.2.mp4` and so on.
            let p = next.take().unwrap_or_else(|| {
                roll += 1;
                base.with_extension(format!("{}.mp4", roll + 1))
            });
            let f = Box::new(dvr::FileSink::create(&p)?);
            // In flight the writes go to a thread of their own; a run that
            // ends with the process (replay) writes in place, so nothing is
            // still queued when it exits.
            let s: Box<dyn dvr::Sink> = if threaded {
                Box::new(dvr::ThreadSink::new(f)?)
            } else {
                f
            };
            Ok((s, p))
        }));
        self.disarm_at = None;
        step.started = Some(path);
    }

    /// Every AU the core emits, with the FC's arm state.
    pub fn on_au(&mut self, a: &pal_core::Au, armed: Option<bool>, now_us: u64) -> Step {
        let mut step = Step::default();
        let want = self.wants(armed, now_us);
        let st = self.rec.state();
        if want && st == State::Off && !self.failed_arm {
            self.start(&mut step);
        } else if !want && st != State::Off {
            self.rec.stop();
            self.disarm_at = None;
        }
        if matches!(self.rec.state(), State::WaitSync | State::Recording) {
            if now_us >= self.space_at {
                self.space_at = now_us + SPACE_CHECK_US;
                if let Some(free) = (self.free)(self.dir()).filter(|&f| f < MIN_FREE / 2) {
                    self.rec.stop();
                    self.failed_arm = true;
                    step.error = Some(format!("stopped: {} MB free", free >> 20));
                }
            }
            self.rec.feed(&a.data, a.pts_us, a.complete);
            if self.rec.state() == State::Error {
                self.failed_arm = true;
                step.error = self.rec.error().map(str::to_string);
            }
        }
        step.sealed = self.rec.take_sealed();
        step
    }

    /// Seals the open file (the link stops).
    pub fn stop(&mut self) -> Vec<Sealed> {
        self.rec.stop();
        self.rec.take_sealed()
    }

    /// The stats line's `dvr` field: state, the open file, MB written, the
    /// last error. None while nothing was ever recorded or tried.
    pub fn status(&self) -> Option<serde_json::Value> {
        let st = match self.rec.state() {
            State::Off if self.n == 0 && self.rec.error().is_none() && !self.failed_arm => {
                return None
            }
            State::Off => "off",
            State::WaitSync => "wait",
            State::Recording => "rec",
            State::Error => "error",
        };
        let file = self
            .rec
            .path()
            .and_then(|p| p.file_name())
            .map(|f| f.to_string_lossy().into_owned());
        Some(serde_json::json!({
            "state": st,
            "file": file,
            "mb": (self.rec.bytes() as f64 / 1e5).round() / 10.0,
            "files": self.n,
            "error": self.rec.error(),
        }))
    }
}

impl Drop for Dvr {
    fn drop(&mut self) {
        self.rec.stop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn au(data: Vec<u8>, pts: u32) -> pal_core::Au {
        pal_core::Au {
            data,
            pts_us: pts,
            complete: true,
            ..Default::default()
        }
    }

    fn nal(ty: u8, body: &[u8]) -> Vec<u8> {
        let mut v = vec![0, 0, 0, 1, ty << 1, 1];
        v.extend_from_slice(body);
        v
    }

    fn idr() -> Vec<u8> {
        let sps = "42010101600000030000030000030000030096a003c08010e58dae493222fe16700800001f48000aff5040";
        let mut v = nal(32, &[0x0c, 0x01, 0xff]);
        v.extend([0, 0, 0, 1]);
        v.extend(
            (0..sps.len())
                .step_by(2)
                .map(|i| u8::from_str_radix(&sps[i..i + 2], 16).unwrap()),
        );
        v.extend(nal(34, &[0xc1, 0x72]));
        v.extend(nal(19, &[0xaf, 0x01]));
        v
    }

    fn tmp() -> PathBuf {
        let d = std::env::temp_dir().join(format!(
            "pal-dvr-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    fn plenty(_: &Path) -> Option<u64> {
        Some(10 << 30)
    }
    fn full(_: &Path) -> Option<u64> {
        Some(100 << 20)
    }

    #[test]
    fn records_from_arm_to_the_tail_after_disarm_and_asks_for_an_idr_once() {
        let dir = tmp();
        let mut d = Dvr::with_free(
            When::OnArm {
                dir: dir.clone(),
                stem: "fl-1_x".into(),
            },
            plenty,
        );
        let p = nal(1, &[0x80, 0x01]);
        assert!(d.on_au(&au(p.clone(), 0), Some(false), 0).started.is_none());
        assert!(d.status().is_none(), "nothing recorded: no chip");
        let s = d.on_au(&au(p.clone(), 10_000), Some(true), 10_000);
        assert_eq!(s.started, Some(dir.join("fl-1_x-1.mp4")));
        assert_eq!(d.status().unwrap()["state"], "wait");
        d.on_au(&au(idr(), 20_000), Some(true), 20_000);
        assert_eq!(d.status().unwrap()["state"], "rec");
        // Link loss: no HEARTBEAT, the recording goes on.
        assert!(d
            .on_au(&au(p.clone(), 30_000), None, 30_000)
            .started
            .is_none());
        assert_eq!(d.status().unwrap()["state"], "rec");
        // Disarm: three more seconds, then sealed.
        d.on_au(&au(p.clone(), 40_000), Some(false), 1_000_000);
        assert_eq!(d.status().unwrap()["state"], "rec");
        let s = d.on_au(&au(p.clone(), 50_000), Some(false), 1_000_000 + TAIL_US);
        assert_eq!(s.sealed.len(), 1);
        // IDR and the two P frames inside the tail; the one at its end is not.
        assert_eq!(s.sealed[0].samples, 3);
        assert_eq!(d.status().unwrap()["state"], "off");
        // Re-arm: the next file.
        let s = d.on_au(&au(p, 60_000), Some(true), 5_000_000);
        assert_eq!(s.started, Some(dir.join("fl-1_x-2.mp4")));
        d.stop();
        // The writer thread finishes the file after the seal; wait for it.
        let f = dir.join("fl-1_x-1.mp4");
        let mut len = 0;
        for _ in 0..400 {
            len = std::fs::metadata(&f).map_or(0, |m| m.len());
            if len > 0 {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        assert!(len > 0);
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn a_full_volume_refuses_once_per_arm() {
        let dir = tmp();
        let mut d = Dvr::with_free(
            When::OnArm {
                dir: dir.clone(),
                stem: "s".into(),
            },
            full,
        );
        let s = d.on_au(&au(idr(), 0), Some(true), 0);
        assert!(s.started.is_none());
        assert!(s.error.unwrap().contains("MB free"));
        assert!(
            d.on_au(&au(idr(), 1), Some(true), 1).error.is_none(),
            "told once"
        );
        assert_eq!(d.status().unwrap()["state"], "off");
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn always_records_the_whole_run_without_an_fc() {
        let dir = tmp();
        let path = dir.join("out.mp4");
        let mut d = Dvr::with_free(When::Always(path.clone()), plenty);
        assert_eq!(d.on_au(&au(idr(), 0), None, 0).started, Some(path.clone()));
        d.on_au(&au(nal(1, &[0x80, 1]), 16_000), Some(false), 16_000);
        let sealed = d.stop();
        assert_eq!(sealed[0].samples, 2);
        let _ = std::fs::remove_dir_all(dir);
    }
}
