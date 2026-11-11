//! Android backend (spec F8): NDK MediaCodec H.265, one whole AU per input
//! buffer, low-latency keys, output rendered straight into the Surface's
//! ANativeWindow (no copy, no GL pass). An output thread releases each frame
//! as it comes out; a frame older than `STALE_US` since its AU was queued is
//! dropped instead of shown. No flush on loss: the next IDR heals.
//! An AU that never reached the codec (no input buffer in time, or larger
//! than the buffer) breaks the reference chain like a decode error does:
//! `damage()` counts decode errors, missing buffers and oversize AUs for the
//! IDR driver.
//!
//! The input buffers are sized by `MAX_INPUT` and the output by the largest
//! camera mode, not by what the codec derives from the size hint: a keyframe
//! that outgrew a derived buffer would be dropped, ask for another keyframe
//! of the same size, and loop until the scene got simpler.
//!
//! The codec is the platform's pick for video/hevc unless the system
//! property `debug.pal.decoder` names one (the Quest also lists
//! `c2.qti.hevc.decoder.low_latency`); every `LAT_WINDOW` frames the log
//! gets the queue-to-output time per frame, so two codecs compare on one
//! headset by `setprop` and a relaunch. `debug.pal.dec_starve N` drops
//! every Nth AU the way a missing input buffer does (a bench switch for the
//! heal path; unset in flight). `debug.pal.dec_break N` stops the codec
//! behind this wrapper once per process, after N AUs: every call fails from
//! then on, as in the codec's error state (a bench switch for the decode
//! loop's rebuild; unset in flight).
//!
//! A codec in its error state is never revived here: `decode` says what
//! became of each AU (`watchdog::Fed`) and the caller drops the decoder and
//! builds a new one (`watchdog::Watchdog`). Warnings from a failing codec
//! are limited to one a second.
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use ndk::media::media_codec::{
    DequeuedInputBufferResult as In, DequeuedOutputBufferInfoResult as Out, MediaCodec,
    MediaCodecDirection,
};
use ndk::media::media_format::MediaFormat;
use ndk::native_window::NativeWindow;

use crate::watchdog::{Fed, LogEvery};

/// PixelPilot's stale cap: a frame this late would only add latency.
const STALE_US: u64 = 150_000;
/// Frames per latency line in the log (~5 s at 60 fps).
const LAT_WINDOW: usize = 300;
/// Ages kept for `take_ages`: several seconds at 120 fps.
const AGES_CAP: usize = 1024;
const DECODER_PROP: &std::ffi::CStr = c"debug.pal.decoder";
const STARVE_PROP: &std::ffi::CStr = c"debug.pal.dec_starve";
const BREAK_PROP: &std::ffi::CStr = c"debug.pal.dec_break";
/// `debug.pal.dec_break` has fired: once per process, so the rebuilt codec
/// is left alone.
static BROKE: AtomicBool = AtomicBool::new(false);
/// One warning line per this many ms from a failing codec (a dead one fails
/// every call: 200 a second on the output thread alone).
const WARN_EVERY_MS: u64 = 1000;
/// Input buffer bytes asked for: the median keyframe measured at 2944x1656
/// with the shipped `VENC_MIN_IQP` is 239 KB, and the largest was not
/// recorded, so this leaves eight times that.
const MAX_INPUT: i32 = 2 << 20;
/// The largest mode in the drone's driver list (mabur
/// `drone/venc/camera_modes.c` `k4K`, 3840x2160 at up to 33 fps; venc.size
/// may use all of it when a recording needs that mode): a mode switch then
/// stays inside the buffers the codec already has.
const MAX_W: i32 = 3840;
const MAX_H: i32 = 2160;

/// A media format key as libmediandk exports it.
fn key(k: *const std::ffi::c_char) -> &'static str {
    // SAFETY: the AMEDIAFORMAT_KEY_* constants are static NUL-terminated
    // ASCII in libmediandk.
    unsafe { std::ffi::CStr::from_ptr(k) }
        .to_str()
        .unwrap_or("")
}

extern "C" {
    // bionic; PROP_VALUE_MAX is 92 bytes with the NUL.
    fn __system_property_get(name: *const std::ffi::c_char, value: *mut std::ffi::c_char) -> i32;
}

/// A system property's value, if set.
pub fn prop(name: &std::ffi::CStr) -> Option<String> {
    let mut v = [0 as std::ffi::c_char; 92];
    // SAFETY: a NUL-terminated name and a PROP_VALUE_MAX buffer.
    let n = unsafe { __system_property_get(name.as_ptr(), v.as_mut_ptr()) };
    if n <= 0 {
        return None;
    }
    // SAFETY: bionic NUL-terminates within PROP_VALUE_MAX.
    let s = unsafe { std::ffi::CStr::from_ptr(v.as_ptr()) };
    Some(s.to_string_lossy().into_owned())
}

/// The named codec when the property names one that opens, else the
/// platform's pick for the type.
fn open_codec() -> Result<MediaCodec, String> {
    if let Some(name) = prop(DECODER_PROP) {
        match MediaCodec::from_codec_name(&name) {
            Some(c) => return Ok(c),
            None => log::warn!("mediacodec: {name} (debug.pal.decoder) did not open"),
        }
    }
    MediaCodec::from_decoder_type("video/hevc").ok_or_else(|| "no video/hevc decoder".into())
}

struct Shared {
    codec: MediaCodec,
    name: String,
    stop: AtomicBool,
    frames: AtomicU64,
    stale: AtomicU64,
    errors: AtomicU64,
    /// AUs dropped because no input buffer freed up within 20 ms (a
    /// decoder that stopped consuming).
    no_input: AtomicU64,
    /// AUs dropped for being larger than the input buffer: a buffer size
    /// problem, not a codec fault, so kept apart from `errors`.
    oversize: AtomicU64,
    /// (pts, queued at) of AUs in flight, oldest first.
    queued: Mutex<VecDeque<(u64, Instant)>>,
    /// Queue-to-output ages (µs) since the last `take_ages`, for the
    /// flight log's decoder record; capped so an unread one stays small.
    ages: Mutex<Vec<u64>>,
    /// The output surface, to log the buffer size the codec gave it.
    win: NativeWindow,
    /// Clock of the warning limits.
    t0: Instant,
}

impl Shared {
    fn ms(&self) -> u64 {
        self.t0.elapsed().as_millis() as u64
    }
}

// SAFETY: AMediaCodec allows one thread queueing input while another
// dequeues output (the NDK docs' async-free two-thread pattern, PixelPilot
// VideoDecoder.cpp does the same); everything else here is atomics or locked.
unsafe impl Send for Shared {}
unsafe impl Sync for Shared {}

pub struct MediaCodecDecoder {
    sh: Arc<Shared>,
    out: Option<JoinHandle<()>>,
    pts: u64,
    /// `debug.pal.dec_starve`: every this-many-th AU is dropped as if no
    /// input buffer had come; 0 = off.
    starve_every: u64,
    fed: u64,
    /// The first input buffer's length is in the log.
    buf_logged: bool,
    /// `debug.pal.dec_break`: the AU count at which the codec is stopped
    /// behind the wrapper; 0 = off.
    break_after: u64,
    in_warn: LogEvery,
}

/// The decoder format: the stream's type, the size hint, the low-latency
/// keys and, with `bounds`, the buffer sizes above.
fn configure(
    codec: &MediaCodec,
    win: &NativeWindow,
    w: i32,
    h: i32,
    bounds: bool,
) -> Result<(), String> {
    let mut fmt = MediaFormat::new();
    fmt.set_str("mime", "video/hevc");
    fmt.set_i32("width", w);
    fmt.set_i32("height", h);
    fmt.set_i32("low-latency", 1);
    fmt.set_i32("vendor.qti-ext-dec-low-latency.enable", 1);
    fmt.set_i32("vendor.qti-ext-dec-picture-order.enable", 1);
    if bounds {
        // SAFETY: reads of libmediandk's constant key pointers.
        let (k_in, k_w, k_h) = unsafe {
            (
                ndk_sys::AMEDIAFORMAT_KEY_MAX_INPUT_SIZE,
                ndk_sys::AMEDIAFORMAT_KEY_MAX_WIDTH,
                ndk_sys::AMEDIAFORMAT_KEY_MAX_HEIGHT,
            )
        };
        fmt.set_i32(key(k_in), MAX_INPUT);
        fmt.set_i32(key(k_w), MAX_W);
        fmt.set_i32(key(k_h), MAX_H);
    }
    codec
        .configure(&fmt, Some(win), MediaCodecDirection::Decoder)
        .map_err(|e| format!("configure: {e:?}"))
}

impl MediaCodecDecoder {
    /// `w`/`h` are a hint for the first buffers; the stream's SPS rules.
    pub fn new(win: NativeWindow, w: i32, h: i32) -> Result<Self, String> {
        let mut codec = open_codec()?;
        let name = codec.name().unwrap_or_else(|_| "?".into());
        log::info!("mediacodec: {name}");
        if let Err(e) = configure(&codec, &win, w, h, true) {
            // A codec that refuses the bounds still decodes with the sizes it
            // derives itself; a fresh instance, since a failed configure may
            // leave this one in its error state.
            log::warn!("mediacodec: configure with max sizes: {e}; again without");
            codec = open_codec()?;
            configure(&codec, &win, w, h, false)?;
        }
        codec.start().map_err(|e| format!("start: {e:?}"))?;
        log::info!("mediacodec: input format {}", codec.input_format());
        let sh = Arc::new(Shared {
            codec,
            name,
            stop: AtomicBool::new(false),
            frames: AtomicU64::new(0),
            stale: AtomicU64::new(0),
            errors: AtomicU64::new(0),
            no_input: AtomicU64::new(0),
            oversize: AtomicU64::new(0),
            queued: Mutex::new(VecDeque::new()),
            ages: Mutex::new(Vec::new()),
            win,
            t0: Instant::now(),
        });
        let s2 = sh.clone();
        let out = std::thread::Builder::new()
            .name("pal-mc-out".into())
            .spawn(move || output_loop(&s2))
            .map_err(|e| e.to_string())?;
        let starve_every = prop(STARVE_PROP).and_then(|v| v.parse().ok()).unwrap_or(0);
        if starve_every > 0 {
            log::warn!("mediacodec: dropping every {starve_every}th AU (debug.pal.dec_starve)");
        }
        let break_after = match prop(BREAK_PROP).and_then(|v| v.parse().ok()) {
            Some(n) if n > 0 && !BROKE.load(Ordering::Relaxed) => {
                log::warn!("mediacodec: stopping the codec after {n} AUs (debug.pal.dec_break)");
                n
            }
            _ => 0,
        };
        Ok(Self {
            sh,
            out: Some(out),
            pts: 0,
            starve_every,
            fed: 0,
            buf_logged: false,
            break_after,
            in_warn: LogEvery::new(WARN_EVERY_MS),
        })
    }

    /// AUs dropped for want of an input buffer since open.
    pub fn no_input(&self) -> u64 {
        self.sh.no_input.load(Ordering::Relaxed)
    }

    /// AUs dropped for being larger than the input buffer since open.
    pub fn oversize(&self) -> u64 {
        self.sh.oversize.load(Ordering::Relaxed)
    }

    /// Events since open that left the reference chain broken: decode
    /// errors and AUs that never reached the codec. The picture stays
    /// damaged from each until an IRAP, and with no periodic one in the
    /// stream (`venc.gop_s = 0`) that IRAP has to be asked for.
    pub fn damage(&self) -> u64 {
        self.sh.errors.load(Ordering::Relaxed)
            + self.sh.no_input.load(Ordering::Relaxed)
            + self.sh.oversize.load(Ordering::Relaxed)
    }

    /// Frames the codec has put out since open, shown or dropped as stale.
    pub fn output(&self) -> u64 {
        self.sh.frames.load(Ordering::Relaxed) + self.sh.stale.load(Ordering::Relaxed)
    }

    /// Frames dropped by the stale cap since open.
    pub fn stale(&self) -> u64 {
        self.sh.stale.load(Ordering::Relaxed)
    }

    /// Queue-to-output ages (µs) of the frames out since the last call.
    pub fn take_ages(&self) -> Vec<u64> {
        std::mem::take(&mut *self.sh.ages.lock().unwrap_or_else(|p| p.into_inner()))
    }
}

/// Queue-to-output microseconds of the frames in one window.
fn log_latency(name: &str, ages: &mut Vec<u64>) {
    ages.sort_unstable();
    let at = |q: usize| ages[(ages.len() - 1) * q / 100];
    let mean = ages.iter().sum::<u64>() / ages.len() as u64;
    log::info!(
        "mediacodec {name}: queue->out us n={} mean={mean} p50={} p90={} p99={} max={}",
        ages.len(),
        at(50),
        at(90),
        at(99),
        at(100)
    );
    ages.clear();
}

fn output_loop(sh: &Shared) {
    let mut ages = Vec::with_capacity(LAT_WINDOW);
    let mut warn = LogEvery::new(WARN_EVERY_MS);
    while !sh.stop.load(Ordering::Relaxed) {
        match sh.codec.dequeue_output_buffer(Duration::from_millis(10)) {
            Ok(Out::Buffer(ob)) => {
                let pts = ob.info().presentation_time_us() as u64;
                let age = {
                    let mut q = sh.queued.lock().unwrap_or_else(|p| p.into_inner());
                    // Drop entries for AUs that produced no frame.
                    while q.front().is_some_and(|(p, _)| *p < pts) {
                        q.pop_front();
                    }
                    q.pop_front()
                        .filter(|(p, _)| *p == pts)
                        .map(|(_, t)| t.elapsed().as_micros() as u64)
                };
                if let Some(a) = age {
                    {
                        let mut w = sh.ages.lock().unwrap_or_else(|p| p.into_inner());
                        if w.len() < AGES_CAP {
                            w.push(a);
                        }
                    }
                    ages.push(a);
                    if ages.len() == LAT_WINDOW {
                        log_latency(&sh.name, &mut ages);
                    }
                }
                let show = age.is_none_or(|a| a <= STALE_US);
                if sh.codec.release_output_buffer(ob, show).is_err() {
                    sh.errors.fetch_add(1, Ordering::Relaxed);
                } else if show {
                    sh.frames.fetch_add(1, Ordering::Relaxed);
                } else {
                    sh.stale.fetch_add(1, Ordering::Relaxed);
                }
            }
            // The codec sizes the surface's buffers to the stream, whatever
            // size the surface was created with: this line says which
            // resolution reaches the compositor.
            Ok(Out::OutputFormatChanged) => log::info!(
                "mediacodec: output format {}; surface buffers {}x{}",
                sh.codec.output_format(),
                sh.win.width(),
                sh.win.height()
            ),
            Ok(_) => {}
            Err(e) => {
                sh.errors.fetch_add(1, Ordering::Relaxed);
                if let Some(held) = warn.hit(sh.ms()) {
                    log::warn!("mediacodec: dequeue_output {e:?} ({held} more not logged)");
                }
                std::thread::sleep(Duration::from_millis(5));
            }
        }
    }
}

impl MediaCodecDecoder {
    /// Queues the AU (waits up to 20 ms for an input buffer). Frames come
    /// out on the output thread.
    pub fn decode(&mut self, au: &[u8]) -> Fed {
        let sh = &self.sh;
        self.fed += 1;
        if self.starve_every > 0 && self.fed.is_multiple_of(self.starve_every) {
            sh.no_input.fetch_add(1, Ordering::Relaxed);
            return Fed::NoInput;
        }
        if self.fed == self.break_after && !BROKE.swap(true, Ordering::Relaxed) {
            log::warn!("mediacodec: codec stopped behind the decoder (debug.pal.dec_break)");
            let _ = sh.codec.stop();
        }
        match sh.codec.dequeue_input_buffer(Duration::from_millis(20)) {
            Ok(In::Buffer(mut b)) => {
                let dst = b.buffer_mut();
                if !self.buf_logged {
                    self.buf_logged = true;
                    log::info!(
                        "mediacodec {}: input buffer {} bytes (asked {MAX_INPUT})",
                        sh.name,
                        dst.len()
                    );
                }
                if au.len() > dst.len() {
                    // A truncated AU decodes to garbage; skip it and let the
                    // IDR driver see the damage count move.
                    let n = sh.oversize.fetch_add(1, Ordering::Relaxed) + 1;
                    log::warn!(
                        "mediacodec: AU of {} bytes over the {} byte input buffer ({n} so far)",
                        au.len(),
                        dst.len()
                    );
                    let _ = sh.codec.queue_input_buffer(b, 0, 0, self.pts, 0);
                    return Fed::Oversize;
                }
                for (d, s) in dst.iter_mut().zip(au) {
                    d.write(*s);
                }
                // Monotonic synthetic pts: the decoder only needs ordering.
                self.pts += 1000;
                sh.queued
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push_back((self.pts, Instant::now()));
                match sh.codec.queue_input_buffer(b, 0, au.len(), self.pts, 0) {
                    Ok(()) => Fed::Queued,
                    Err(e) => {
                        sh.errors.fetch_add(1, Ordering::Relaxed);
                        if let Some(held) = self.in_warn.hit(sh.ms()) {
                            log::warn!("mediacodec: queue_input {e:?} ({held} more not logged)");
                        }
                        Fed::Failed
                    }
                }
            }
            Ok(_) => {
                sh.no_input.fetch_add(1, Ordering::Relaxed);
                Fed::NoInput
            }
            Err(e) => {
                sh.errors.fetch_add(1, Ordering::Relaxed);
                if let Some(held) = self.in_warn.hit(sh.ms()) {
                    log::warn!("mediacodec: dequeue_input {e:?} ({held} more not logged)");
                }
                Fed::Failed
            }
        }
    }

    /// (frames out, decode errors) since open.
    pub fn counters(&self) -> (u64, u64) {
        (
            self.sh.frames.load(Ordering::Relaxed),
            self.sh.errors.load(Ordering::Relaxed),
        )
    }
}

impl Drop for MediaCodecDecoder {
    fn drop(&mut self) {
        self.sh.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.out.take() {
            let _ = t.join();
        }
        let _ = self.sh.codec.stop();
        // The joined output thread held the only other `sh`: the codec is
        // deleted, and lets go of the window, as this returns, so the next
        // decoder can configure on the same window (one producer at a time).
    }
}
