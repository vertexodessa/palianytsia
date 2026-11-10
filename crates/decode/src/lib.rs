//! Video decode (spec 5.3 `decode`). Linux: libavcodec HEVC through a C
//! shim (`cpp/dec.c`, module `av`), low delay, slice threads, one AU in,
//! newest frame out, optionally through a hwaccel. Android: NDK MediaCodec
//! rendering straight into a Surface (module `mediacodec`).
#[cfg(not(target_os = "android"))]
mod av;
#[cfg(not(target_os = "android"))]
pub use av::AvDecoder;
#[cfg(target_os = "android")]
pub mod mediacodec;
pub mod watchdog;

/// One frame as tightly packed YUV planes, for a GPU colour conversion.
#[derive(Default, Clone)]
pub struct Planes {
    pub w: usize,
    pub h: usize,
    /// NV12 (`u` holds interleaved UV, `v` empty) or I420.
    pub nv12: bool,
    /// BT.709 matrix (else BT.601), full range (else limited).
    pub bt709: bool,
    pub full_range: bool,
    pub y: Vec<u8>,
    pub u: Vec<u8>,
    pub v: Vec<u8>,
}
