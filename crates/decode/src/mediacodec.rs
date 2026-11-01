//! Android backend (spec F8): NDK MediaCodec H.265, one whole AU per input
//! buffer, low-latency keys, output rendered straight into the Surface's
//! ANativeWindow (no copy, no GL pass). An output thread releases each frame
//! as it comes out; a frame older than `STALE_US` since its AU was queued is
//! dropped instead of shown. No flush on loss: the next IDR heals.
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

use crate::Decoder;

/// PixelPilot's stale cap: a frame this late would only add latency.
const STALE_US: u64 = 150_000;

struct Shared {
    codec: MediaCodec,
    stop: AtomicBool,
    frames: AtomicU64,
    stale: AtomicU64,
    errors: AtomicU64,
    /// AUs dropped because no input buffer freed up within 20 ms (a
    /// decoder that stopped consuming).
    no_input: AtomicU64,
    /// (pts, queued at) of AUs in flight, oldest first.
    queued: Mutex<VecDeque<(u64, Instant)>>,
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
    _win: NativeWindow,
}

impl MediaCodecDecoder {
    /// `w`/`h` are a hint for the first buffers; the stream's SPS rules.
    pub fn new(win: NativeWindow, w: i32, h: i32) -> Result<Self, String> {
        let codec = MediaCodec::from_decoder_type("video/hevc").ok_or("no video/hevc decoder")?;
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
            stop: AtomicBool::new(false),
            frames: AtomicU64::new(0),
            stale: AtomicU64::new(0),
            errors: AtomicU64::new(0),
            no_input: AtomicU64::new(0),
            queued: Mutex::new(VecDeque::new()),
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
            _win: win,
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

fn output_loop(sh: &Shared) {
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
                let show = age.is_none_or(|a| a <= STALE_US);
                if sh.codec.release_output_buffer(ob, show).is_err() {
                    sh.errors.fetch_add(1, Ordering::Relaxed);
                } else if show {
                    sh.frames.fetch_add(1, Ordering::Relaxed);
                } else {
                    sh.stale.fetch_add(1, Ordering::Relaxed);
                }
            }
            Ok(Out::OutputFormatChanged) => log::info!("mediacodec: output format changed"),
            Ok(_) => {}
            Err(e) => {
                sh.errors.fetch_add(1, Ordering::Relaxed);
                log::warn!("mediacodec: dequeue_output {e:?}");
                std::thread::sleep(Duration::from_millis(5));
            }
        }
    }
}

impl Decoder for MediaCodecDecoder {
    /// Queues the AU (waits up to 20 ms for an input buffer). Frames come
    /// out on the output thread, so this always returns 0.
    fn decode(&mut self, au: &[u8]) -> usize {
        let sh = &self.sh;
        match sh.codec.dequeue_input_buffer(Duration::from_millis(20)) {
            Ok(In::Buffer(mut b)) => {
                let dst = b.buffer_mut();
                if au.len() > dst.len() {
                    // A truncated AU decodes to garbage; skip it and let the
                    // IDR driver see the decoder error count move.
                    sh.errors.fetch_add(1, Ordering::Relaxed);
                    let _ = sh.codec.queue_input_buffer(b, 0, 0, self.pts, 0);
                    return 0;
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
        0
    }

    /// Frames go to the Surface, never to memory.
    fn rgba(&mut self, _out: &mut Vec<u8>) -> Option<(usize, usize)> {
        None
    }

    fn counters(&self) -> (u64, u64) {
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
