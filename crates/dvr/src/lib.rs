//! GS DVR (spec F12): the received HEVC AUs into a fragmented MP4, one sample
//! per AU, starting on an IRAP. A port of mabur's `DvrMux`, `RawDvr` and
//! `HevcParams` (`common/src/{dvr_mux,raw_dvr,hevc_params}.cpp`); the boxes
//! are the same bytes, so a file plays wherever mabur's GS DVR files play.
//! The deferred roll of mabur's web GS (OPFS) is left out.

pub mod mux;
pub mod params;

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, SyncSender, TrySendError};
use std::sync::Arc;

pub use mux::{FileSink, Mux, Sink};
pub use params::{au_has_param_sets, au_is_irap, HevcParams};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    Off,
    /// Started; the file opens on the next IRAP with all parameter sets known.
    WaitSync,
    Recording,
    /// Stopped by an open or write failure; `Recorder::error` says which.
    Error,
}

/// Opens the next file: a sink and the path it writes, for the report.
pub type Opener = Box<dyn FnMut() -> std::io::Result<(Box<dyn Sink>, PathBuf)> + Send>;

/// A file the recorder has closed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Sealed {
    pub path: PathBuf,
    pub bytes: u64,
    pub samples: u64,
    pub ok: bool,
}

/// mabur's `RawDvr` rules: start, wait for a sync point, write complete AUs,
/// seal; a new file when the coded size changes at an IRAP.
pub struct Recorder {
    params: HevcParams,
    mux: Option<Mux>,
    path: Option<PathBuf>,
    open: Option<Opener>,
    state: State,
    error: Option<String>,
    /// The SPS size the open file was made for; None when it did not parse.
    size: Option<(u32, u32)>,
    fragment_ms: u32,
    sealed: Vec<Sealed>,
}

impl Default for Recorder {
    fn default() -> Self {
        Self::new(1000)
    }
}

impl Recorder {
    pub fn new(fragment_ms: u32) -> Self {
        Self {
            params: HevcParams::default(),
            mux: None,
            path: None,
            open: None,
            state: State::Off,
            error: None,
            size: None,
            fragment_ms,
            sealed: Vec::new(),
        }
    }

    /// Arms a recording; `open` is called on the first sync point and again
    /// for each size change. Seals an open file first.
    pub fn start(&mut self, open: Opener) {
        self.stop();
        self.open = Some(open);
        self.error = None;
        self.state = State::WaitSync;
    }

    pub fn state(&self) -> State {
        self.state
    }
    pub fn error(&self) -> Option<&str> {
        self.error.as_deref()
    }
    /// The open file.
    pub fn path(&self) -> Option<&std::path::Path> {
        self.mux.as_ref().and(self.path.as_deref())
    }
    pub fn bytes(&self) -> u64 {
        self.mux.as_ref().map_or(0, Mux::bytes)
    }
    pub fn samples(&self) -> u64 {
        self.mux.as_ref().map_or(0, Mux::samples)
    }
    /// Files closed since the last call.
    pub fn take_sealed(&mut self) -> Vec<Sealed> {
        std::mem::take(&mut self.sealed)
    }

    fn seal(&mut self) {
        if let Some(mut m) = self.mux.take() {
            m.close();
            self.sealed.push(Sealed {
                path: self.path.take().unwrap_or_default(),
                bytes: m.bytes(),
                samples: m.samples(),
                ok: m.ok(),
            });
        }
    }

    fn fail(&mut self, e: String) {
        self.seal();
        self.open = None;
        self.error = Some(e);
        self.state = State::Error;
    }

    /// Opens a file for (w, h); false after a failure (state Error).
    fn open_file(&mut self, w: u32, h: u32) -> bool {
        let Some(hvcc) = self.params.hvcc() else {
            self.fail("SPS too short for an hvcC".into());
            return false;
        };
        let Some(open) = self.open.as_mut() else {
            return false;
        };
        match open() {
            Ok((sink, path)) => {
                let m = Mux::open(sink, &hvcc, w, h, self.fragment_ms);
                self.path = Some(path);
                if !m.ok() {
                    self.mux = Some(m);
                    self.fail("write failed".into());
                    return false;
                }
                self.mux = Some(m);
                true
            }
            Err(e) => {
                self.fail(e.to_string());
                false
            }
        }
    }

    /// Every AU in arrival order; ignored unless started. Incomplete AUs are
    /// skipped (the muxer cuts at the next key).
    pub fn feed(&mut self, au: &[u8], pts_us: u32, complete: bool) {
        if !matches!(self.state, State::WaitSync | State::Recording) || !complete {
            return;
        }
        if au_has_param_sets(au) {
            self.params.feed(au);
        }
        // Sync is a real IRAP: the GDR refresh start also carries VPS/SPS/PPS
        // but references earlier frames, and a file opened on it is refused
        // by Apple's decoder.
        let key = au_is_irap(au);
        if self.state == State::WaitSync {
            if !key || !self.params.complete() {
                return;
            }
            let sps = self.params.sps_dimensions();
            let (w, h) = sps.unwrap_or((1920, 1080));
            if !self.open_file(w, h) {
                return;
            }
            self.size = sps;
            self.state = State::Recording;
        } else if key {
            // One size per file header: a camera mode change at an IRAP
            // seals this file and opens the next. Rewritten parameter sets at
            // the same size are not a roll.
            if let Some(s) = self.params.sps_dimensions() {
                match self.size {
                    None => self.size = Some(s),
                    Some(old) if old != s => {
                        self.seal();
                        if !self.open_file(s.0, s.1) {
                            return;
                        }
                        self.size = Some(s);
                    }
                    _ => {}
                }
            }
        }
        let Some(m) = self.mux.as_mut() else { return };
        m.write_sample(au, pts_us, key);
        if !m.ok() {
            self.fail("write failed".into());
        }
    }

    /// Seals an open file; Off afterwards. `error` stays readable.
    pub fn stop(&mut self) {
        self.seal();
        self.open = None;
        self.state = State::Off;
    }
}

/// A sink that hands each finished fragment to a writer thread, so a slow
/// card or flash never stalls the caller. A full queue (16 fragments, about
/// 16 s of video) is a failure like a full disk: the file keeps what was
/// written before.
pub struct ThreadSink {
    buf: Vec<u8>,
    tx: Option<SyncSender<Vec<u8>>>,
    failed: Arc<AtomicBool>,
}

impl ThreadSink {
    pub fn new(mut inner: Box<dyn Sink>) -> std::io::Result<Self> {
        let (tx, rx) = mpsc::sync_channel::<Vec<u8>>(16);
        let failed = Arc::new(AtomicBool::new(false));
        let f = failed.clone();
        std::thread::Builder::new()
            .name("pal-dvr".into())
            .spawn(move || {
                for chunk in rx {
                    if f.load(Ordering::Relaxed) {
                        continue;
                    }
                    if !inner.write(&chunk) || !inner.flush() {
                        f.store(true, Ordering::Relaxed);
                    }
                }
            })?;
        Ok(Self {
            buf: Vec::new(),
            tx: Some(tx),
            failed,
        })
    }
}

impl Sink for ThreadSink {
    fn write(&mut self, p: &[u8]) -> bool {
        if self.failed.load(Ordering::Relaxed) {
            return false;
        }
        self.buf.extend_from_slice(p);
        true
    }
    fn flush(&mut self) -> bool {
        if self.failed.load(Ordering::Relaxed) {
            return false;
        }
        let Some(tx) = &self.tx else { return false };
        match tx.try_send(std::mem::take(&mut self.buf)) {
            Ok(()) => true,
            Err(TrySendError::Full(_)) | Err(TrySendError::Disconnected(_)) => {
                self.failed.store(true, Ordering::Relaxed);
                false
            }
        }
    }
}

impl Drop for ThreadSink {
    /// Closing the queue ends the thread once it has written what is queued;
    /// the caller does not wait for it.
    fn drop(&mut self) {
        self.tx = None;
    }
}

#[cfg(test)]
mod tests;
