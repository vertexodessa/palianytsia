//! libavcodec backend (Linux).
use std::ffi::{c_char, c_int, CStr, CString};

use crate::Planes;

fn copy_plane(dst: &mut Vec<u8>, src: *const u8, stride: c_int, row_bytes: usize, rows: usize) {
    dst.resize(row_bytes * rows, 0);
    for r in 0..rows {
        // SAFETY: libavcodec's plane holds `rows` rows of `stride` bytes.
        let line = unsafe {
            std::slice::from_raw_parts(src.offset(r as isize * stride as isize), row_bytes)
        };
        dst[r * row_bytes..(r + 1) * row_bytes].copy_from_slice(line);
    }
}

#[repr(C)]
struct RawDec {
    _p: [u8; 0],
}

extern "C" {
    fn pal_dec_new_hw(
        threads: c_int,
        hw: *const c_char,
        err: *mut c_char,
        cap: usize,
    ) -> *mut RawDec;
    fn pal_dec_hw_frames(d: *const RawDec) -> c_int;
    fn pal_dec_planes(
        d: *mut RawDec,
        w: *mut c_int,
        h: *mut c_int,
        fmt: *mut c_int,
        cs: *mut c_int,
        full: *mut c_int,
        data: *mut [*const u8; 3],
        linesize: *mut [c_int; 3],
    ) -> c_int;
    fn pal_dec_free(d: *mut RawDec);
    fn pal_dec_send(d: *mut RawDec, au: *const u8, n: usize) -> c_int;
    fn pal_dec_size(d: *const RawDec, w: *mut c_int, h: *mut c_int) -> c_int;
    fn pal_dec_rgba(d: *mut RawDec, out: *mut u8, cap: usize) -> c_int;
    fn pal_dec_counters(d: *const RawDec, frames: *mut c_int, errors: *mut c_int);
}

pub struct AvDecoder {
    raw: *mut RawDec,
}

// The shim owns plain libavcodec state; it is used from one thread at a time.
unsafe impl Send for AvDecoder {}

impl AvDecoder {
    /// `threads`: slice threads, 0 = libavcodec's choice.
    pub fn open(threads: i32) -> Result<Self, String> {
        Self::open_hw(threads, "")
    }
    /// `hw`: an av_hwdevice type ("vaapi", "vdpau", "cuda", ...), "" for
    /// software. Errs when that device cannot be opened; a stream the
    /// hwaccel refuses later decodes in software (see `hw_frames`).
    pub fn open_hw(threads: i32, hw: &str) -> Result<Self, String> {
        let hw = CString::new(hw).map_err(|_| "hwaccel name has a NUL".to_string())?;
        let mut err = [0 as c_char; 256];
        let raw = unsafe { pal_dec_new_hw(threads, hw.as_ptr(), err.as_mut_ptr(), err.len()) };
        if raw.is_null() {
            let msg = unsafe { CStr::from_ptr(err.as_ptr()) };
            return Err(msg.to_string_lossy().into_owned());
        }
        Ok(Self { raw })
    }
    /// The newest frame's planes, tightly packed into `out` (buffers reused);
    /// false when there is no frame or it is neither I420 nor NV12.
    pub fn planes(&mut self, out: &mut Planes) -> bool {
        let (mut w, mut h, mut fmt, mut cs, mut full) = (0, 0, 0, 0, 0);
        let mut data = [std::ptr::null(); 3];
        let mut ls = [0; 3];
        let r = unsafe {
            pal_dec_planes(
                self.raw, &mut w, &mut h, &mut fmt, &mut cs, &mut full, &mut data, &mut ls,
            )
        };
        if r != 0 || w <= 0 || h <= 0 {
            return false;
        }
        let (w, h) = (w as usize, h as usize);
        let (cw, ch) = (w.div_ceil(2), h.div_ceil(2));
        out.w = w;
        out.h = h;
        out.nv12 = fmt == 1;
        out.bt709 = cs != 0;
        out.full_range = full != 0;
        copy_plane(&mut out.y, data[0], ls[0], w, h);
        if out.nv12 {
            copy_plane(&mut out.u, data[1], ls[1], cw * 2, ch);
            out.v.clear();
        } else {
            copy_plane(&mut out.u, data[1], ls[1], cw, ch);
            copy_plane(&mut out.v, data[2], ls[2], cw, ch);
        }
        true
    }
    /// Frames that came out as hardware surfaces.
    pub fn hw_frames(&self) -> u64 {
        unsafe { pal_dec_hw_frames(self.raw) as u64 }
    }
    /// `open_hw(hw)` if it opens, else software with a note on stderr.
    /// "auto" tries vdpau, vaapi, cuda in that order. vdpau first: on NVIDIA
    /// the VA-API path is nvidia-vaapi-driver, which hung (default and egl
    /// backends) or crashed (direct) on this HEVC stream after a few frames,
    /// while vdpau decodes at 0.4 ms per 1080p frame (cuda 0.8 ms). Intel
    /// and AMD hosts usually lack vdpau and land on vaapi.
    pub fn open_or_sw(threads: i32, hw: &str) -> Result<Self, String> {
        if hw.is_empty() {
            return Self::open(threads);
        }
        if hw == "auto" {
            for h in ["vdpau", "vaapi", "cuda"] {
                if let Ok(d) = Self::open_hw(threads, h) {
                    eprintln!("decode: hwaccel {h}");
                    return Ok(d);
                }
            }
            eprintln!("decode: no hwaccel opened; software decode");
            return Self::open(threads);
        }
        Self::open_hw(threads, hw).or_else(|e| {
            eprintln!("decode: {hw}: {e}; software decode");
            Self::open(threads)
        })
    }
}

impl Drop for AvDecoder {
    fn drop(&mut self) {
        unsafe { pal_dec_free(self.raw) }
    }
}

impl AvDecoder {
    /// One Annex-B access unit. Returns how many frames came out (0 or 1
    /// with low delay); errors are counted, never fatal.
    pub fn decode(&mut self, au: &[u8]) -> usize {
        let r = unsafe { pal_dec_send(self.raw, au.as_ptr(), au.len()) };
        r.max(0) as usize
    }

    /// The newest frame as packed RGBA, or None before the first frame.
    pub fn rgba(&mut self, out: &mut Vec<u8>) -> Option<(usize, usize)> {
        let (mut w, mut h) = (0, 0);
        if unsafe { pal_dec_size(self.raw, &mut w, &mut h) } == 0 {
            return None;
        }
        let (w, h) = (w as usize, h as usize);
        out.resize(w * h * 4, 0);
        (unsafe { pal_dec_rgba(self.raw, out.as_mut_ptr(), out.len()) } == 0).then_some((w, h))
    }

    /// (frames out, decode errors) since open.
    pub fn counters(&self) -> (u64, u64) {
        let (mut f, mut e) = (0, 0);
        unsafe { pal_dec_counters(self.raw, &mut f, &mut e) };
        (f as u64, e as u64)
    }
}
