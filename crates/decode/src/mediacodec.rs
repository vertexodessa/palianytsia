//! Android backend (spec F8): NDK MediaCodec H.265, one whole AU per input
//! buffer, low-latency keys, output rendered straight into the Surface's
//! ANativeWindow (no copy, no GL pass). An output thread releases each frame
//! as it comes out; a frame older than `STALE_US` since its AU was queued is
//! dropped instead of shown. No flush on loss: the next IDR heals.
//!
//! The codec is the platform's pick for video/hevc unless the system
//! property `debug.pal.decoder` names one (the Quest also lists
//! `c2.qti.hevc.decoder.low_latency`); every `LAT_WINDOW` frames the log
//! gets the queue-to-output time per frame, so two codecs compare on one
//! headset by `setprop` and a relaunch.
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

/// PixelPilot's stale cap: a frame this late would only add latency.
const STALE_US: u64 = 150_000;
/// Frames per latency line in the log (~5 s at 60 fps).
const LAT_WINDOW: usize = 300;
const DECODER_PROP: &std::ffi::CStr = c"debug.pal.decoder";

extern "C" {
    // bionic; PROP_VALUE_MAX is 92 bytes with the NUL.
    fn __system_property_get(name: *const std::ffi::c_char, value: *mut std::ffi::c_char) -> i32;
}

/// `debug.pal.decoder`, if set.
fn decoder_prop() -> Option<String> {
    let mut v = [0 as std::ffi::c_char; 92];
    // SAFETY: a NUL-terminated name and a PROP_VALUE_MAX buffer.
    let n = unsafe { __system_property_get(DECODER_PROP.as_ptr(), v.as_mut_ptr()) };
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
    if let Some(name) = decoder_prop() {
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
    /// (pts, queued at) of AUs in flight, oldest first.
    queued: Mutex<VecDeque<(u64, Instant)>>,
    /// The output surface, to log the buffer size the codec gave it.
    win: NativeWindow,
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
}

impl MediaCodecDecoder {
    /// `w`/`h` are a hint for the first buffers; the stream's SPS rules.
    pub fn new(win: NativeWindow, w: i32, h: i32) -> Result<Self, String> {
        let codec = open_codec()?;
        let name = codec.name().unwrap_or_else(|_| "?".into());
        log::info!("mediacodec: {name}");
        let mut fmt = MediaFormat::new();
        fmt.set_str("mime", "video/hevc");
        fmt.set_i32("width", w);
        fmt.set_i32("height", h);
        fmt.set_i32("low-latency", 1);
        fmt.set_i32("vendor.qti-ext-dec-low-latency.enable", 1);
        fmt.set_i32("vendor.qti-ext-dec-picture-order.enable", 1);
        codec
            .configure(&fmt, Some(&win), MediaCodecDirection::Decoder)
            .map_err(|e| format!("configure: {e:?}"))?;
        codec.start().map_err(|e| format!("start: {e:?}"))?;
        let sh = Arc::new(Shared {
            codec,
            name,
            stop: AtomicBool::new(false),
            frames: AtomicU64::new(0),
            stale: AtomicU64::new(0),
            errors: AtomicU64::new(0),
            no_input: AtomicU64::new(0),
            queued: Mutex::new(VecDeque::new()),
            win,
        });
        let s2 = sh.clone();
        let out = std::thread::Builder::new()
            .name("pal-mc-out".into())
            .spawn(move || output_loop(&s2))
            .map_err(|e| e.to_string())?;
        Ok(Self {
            sh,
            out: Some(out),
            pts: 0,
        })
    }

    /// AUs dropped for want of an input buffer since open.
    pub fn no_input(&self) -> u64 {
        self.sh.no_input.load(Ordering::Relaxed)
    }

    /// Frames dropped by the stale cap since open.
    pub fn stale(&self) -> u64 {
        self.sh.stale.load(Ordering::Relaxed)
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
                log::warn!("mediacodec: dequeue_output {e:?}");
                std::thread::sleep(Duration::from_millis(5));
            }
        }
    }
}

impl MediaCodecDecoder {
    /// Queues the AU (waits up to 20 ms for an input buffer). Frames come
    /// out on the output thread.
    pub fn decode(&mut self, au: &[u8]) {
        let sh = &self.sh;
        match sh.codec.dequeue_input_buffer(Duration::from_millis(20)) {
            Ok(In::Buffer(mut b)) => {
                let dst = b.buffer_mut();
                if au.len() > dst.len() {
                    // A truncated AU decodes to garbage; skip it and let the
                    // IDR driver see the decoder error count move.
                    sh.errors.fetch_add(1, Ordering::Relaxed);
                    let _ = sh.codec.queue_input_buffer(b, 0, 0, self.pts, 0);
                    return;
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
                if sh
                    .codec
                    .queue_input_buffer(b, 0, au.len(), self.pts, 0)
                    .is_err()
                {
                    sh.errors.fetch_add(1, Ordering::Relaxed);
                }
            }
            Ok(_) => {
                sh.no_input.fetch_add(1, Ordering::Relaxed);
            }
            Err(e) => {
                log::warn!("mediacodec: dequeue_input {e:?}");
                sh.errors.fetch_add(1, Ordering::Relaxed);
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
    }
}
