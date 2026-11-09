//! The unsafe shapes every C++ boundary in this crate repeats.

use std::ffi::{c_char, CStr};

/// Runs a callback body that C++ called; a panic must never unwind into C++.
pub(crate) fn guard<R>(f: impl FnOnce() -> R) -> R {
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(f))
        .unwrap_or_else(|_| std::process::abort())
}

/// C++ passes a null pointer with an empty buffer, which `from_raw_parts` forbids.
///
/// # Safety
/// `len` is 0, or `p` points at `len` readable bytes that live for `'a`.
pub(crate) unsafe fn slice<'a>(p: *const u8, len: usize) -> &'a [u8] {
    if len == 0 {
        &[]
    } else {
        std::slice::from_raw_parts(p, len)
    }
}

/// # Safety
/// `p` points at a NUL-terminated string.
pub(crate) unsafe fn cstr_lossy(p: *const c_char) -> String {
    CStr::from_ptr(p).to_string_lossy().into_owned()
}

/// The message a C++ call wrote into its error buffer.
pub(crate) fn err_text(err: &[c_char]) -> String {
    // SAFETY: the shims NUL-terminate within the capacity they were given.
    unsafe { cstr_lossy(err.as_ptr()) }
}

/// A shim that returns the text's length and copies it only when given a
/// buffer: asked twice, first for the size. `fill` takes (buf, cap).
pub(crate) fn read_json(fill: impl Fn(*mut c_char, usize) -> usize) -> String {
    let n = fill(std::ptr::null_mut(), 0);
    let mut buf = vec![0 as c_char; n + 1];
    fill(buf.as_mut_ptr(), buf.len());
    // SAFETY: the second call NUL-terminated buf (n bytes fit in n + 1).
    unsafe { cstr_lossy(buf.as_ptr()) }
}
