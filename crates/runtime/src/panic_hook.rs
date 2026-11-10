//! The process's panic hook. Release builds abort on a panic, and on
//! Android the default hook's stderr goes nowhere: the tombstone shows
//! abort(), not the message or where it came from. This hook logs both
//! (logcat on Android), appends them with the build id and the session's
//! name to `panic.txt` in the flight-log root, then hands over to the hook
//! it replaced. The next session reports that file once, as `panic_prev`
//! (`flightlog::session`).
//!
//! Inside the hook nothing takes a lock the panicking thread may hold, and
//! nothing allocates by the message's size: the record is formatted into a
//! fixed stack buffer (cut at RECORD_MAX) and written with one append.
use std::fmt;
use std::fs::OpenOptions;
use std::io::Write as _;
use std::panic::PanicHookInfo;
use std::path::{Path, PathBuf};
use std::sync::{Once, OnceLock};
use std::time::{SystemTime, UNIX_EPOCH};

use flightlog::session::{last_opened, PANIC_FILE};

/// One record's bytes; a longer message is cut.
const RECORD_MAX: usize = 4096;

/// panic.txt, set by the first install that names a log root.
static PANIC_PATH: OnceLock<PathBuf> = OnceLock::new();

struct StackBuf {
    b: [u8; RECORD_MAX],
    n: usize,
}

impl fmt::Write for StackBuf {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let mut take = s.len().min(RECORD_MAX - self.n);
        while !s.is_char_boundary(take) {
            take -= 1;
        }
        self.b[self.n..self.n + take].copy_from_slice(&s.as_bytes()[..take]);
        self.n += take;
        Ok(())
    }
}

impl StackBuf {
    fn as_str(&self) -> &str {
        // Only whole chars go in (write_str cuts on a boundary).
        std::str::from_utf8(&self.b[..self.n]).unwrap_or("")
    }
}

/// Installs the hook once per process; later calls only fill in the log
/// root if no earlier one named it. `log_dir`: the flight-log root, None
/// when there is no flight log (the hook still logs).
pub fn install_panic_hook(log_dir: Option<&Path>) {
    static ONCE: Once = Once::new();
    if let Some(d) = log_dir.filter(|d| !d.as_os_str().is_empty()) {
        let _ = PANIC_PATH.set(d.join(PANIC_FILE));
    }
    ONCE.call_once(|| {
        let prev = std::panic::take_hook();
        std::panic::set_hook(Box::new(move |info| {
            record(info);
            prev(info);
        }));
    });
}

fn record(info: &PanicHookInfo<'_>) {
    let mut buf = StackBuf {
        b: [0; RECORD_MAX],
        n: 0,
    };
    let t = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    let thread = std::thread::current();
    let _ = fmt::write(
        &mut buf,
        format_args!(
            "unix_s={t} build={} session={} thread={}\n{info}\n",
            pal_build::ID,
            last_opened().unwrap_or("none"),
            thread.name().unwrap_or("?"),
        ),
    );
    log::error!("{}", buf.as_str());
    if let Some(p) = PANIC_PATH.get() {
        if let Ok(mut f) = OpenOptions::new().create(true).append(true).open(p) {
            // One write of the whole record: two threads panicking at once
            // append whole records, not interleaved lines.
            let _ = f.write_all(&buf.b[..buf.n]);
        }
    }
    #[cfg(not(target_os = "android"))]
    {
        // No logger on the desktop apps: the build id goes next to the
        // default hook's message on stderr.
        let _ = writeln!(
            std::io::stderr(),
            "panic in build {}, session {}",
            pal_build::ID,
            last_opened().unwrap_or("none")
        );
    }
}

/// Bench switch: `adb shell setprop debug.pal.panic_test 1` before a start
/// makes a thread panic 5 s after the first start (unset or "0": nothing),
/// so the hook's logcat line and panic.txt can be checked on a device.
#[cfg(target_os = "android")]
pub fn arm_panic_test() {
    static ONCE: Once = Once::new();
    ONCE.call_once(spawn_panic_test);
}

#[cfg(target_os = "android")]
fn spawn_panic_test() {
    const PROP: &std::ffi::CStr = c"debug.pal.panic_test";
    extern "C" {
        // bionic; PROP_VALUE_MAX is 92 bytes with the NUL.
        fn __system_property_get(
            name: *const std::ffi::c_char,
            value: *mut std::ffi::c_char,
        ) -> i32;
    }
    let _ = std::thread::Builder::new()
        .name("pal-panic-test".into())
        .spawn(|| {
            std::thread::sleep(std::time::Duration::from_secs(5));
            let mut v = [0 as std::ffi::c_char; 92];
            // SAFETY: a NUL-terminated name and a PROP_VALUE_MAX buffer.
            let n = unsafe { __system_property_get(PROP.as_ptr(), v.as_mut_ptr()) };
            if n > 0 && !(n == 1 && v[0] == b'0' as std::ffi::c_char) {
                panic!("debug.pal.panic_test: forced panic");
            }
        });
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fmt::Write as _;

    #[test]
    fn a_long_message_is_cut_on_a_char_boundary() {
        let mut b = StackBuf {
            b: [0; RECORD_MAX],
            n: 0,
        };
        let _ = b.write_str(&"x".repeat(RECORD_MAX - 1));
        let _ = b.write_str("é and more");
        assert_eq!(b.n, RECORD_MAX - 1);
        assert!(b.as_str().ends_with('x'));
    }
}
