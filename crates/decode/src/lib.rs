//! Video decode (spec 5.3 `decode`). Linux backend: libavcodec HEVC through
//! a C shim (`cpp/dec.c`), low delay, slice threads, one AU in, newest frame
//! out. The Android MediaCodec backend lands with the phone app.
use std::ffi::{c_char, c_int, CStr};

#[repr(C)]
struct RawDec {
    _p: [u8; 0],
}

extern "C" {
    fn pal_dec_new(threads: c_int, err: *mut c_char, cap: usize) -> *mut RawDec;
    fn pal_dec_free(d: *mut RawDec);
    fn pal_dec_send(d: *mut RawDec, au: *const u8, n: usize) -> c_int;
    fn pal_dec_size(d: *const RawDec, w: *mut c_int, h: *mut c_int) -> c_int;
    fn pal_dec_rgba(d: *mut RawDec, out: *mut u8, cap: usize) -> c_int;
    fn pal_dec_counters(d: *const RawDec, frames: *mut c_int, errors: *mut c_int);
}

pub trait Decoder: Send {
    /// One Annex-B access unit. Returns how many frames came out (0 or 1
    /// with low delay); errors are counted, never fatal.
    fn decode(&mut self, au: &[u8]) -> usize;
    /// The newest frame as packed RGBA, or None before the first frame.
    fn rgba(&mut self, out: &mut Vec<u8>) -> Option<(usize, usize)>;
    /// (frames out, decode errors) since open.
    fn counters(&self) -> (u64, u64);
}

pub struct AvDecoder {
    raw: *mut RawDec,
}

// The shim owns plain libavcodec state; it is used from one thread at a time.
unsafe impl Send for AvDecoder {}

impl AvDecoder {
    /// `threads`: slice threads, 0 = libavcodec's choice.
    pub fn open(threads: i32) -> Result<Self, String> {
        let mut err = [0 as c_char; 256];
        let raw = unsafe { pal_dec_new(threads, err.as_mut_ptr(), err.len()) };
        if raw.is_null() {
            let msg = unsafe { CStr::from_ptr(err.as_ptr()) };
            return Err(msg.to_string_lossy().into_owned());
        }
        Ok(Self { raw })
    }
}

impl Drop for AvDecoder {
    fn drop(&mut self) {
        unsafe { pal_dec_free(self.raw) }
    }
}

impl Decoder for AvDecoder {
    fn decode(&mut self, au: &[u8]) -> usize {
        let r = unsafe { pal_dec_send(self.raw, au.as_ptr(), au.len()) };
        r.max(0) as usize
    }

    fn rgba(&mut self, out: &mut Vec<u8>) -> Option<(usize, usize)> {
        let (mut w, mut h) = (0, 0);
        if unsafe { pal_dec_size(self.raw, &mut w, &mut h) } == 0 {
            return None;
        }
        let (w, h) = (w as usize, h as usize);
        out.resize(w * h * 4, 0);
        (unsafe { pal_dec_rgba(self.raw, out.as_mut_ptr(), out.len()) } == 0).then_some((w, h))
    }

    fn counters(&self) -> (u64, u64) {
        let (mut f, mut e) = (0, 0);
        unsafe { pal_dec_counters(self.raw, &mut f, &mut e) };
        (f as u64, e as u64)
    }
}
