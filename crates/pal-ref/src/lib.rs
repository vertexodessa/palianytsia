//! mabur's C++ `webgs::WebGs` behind [`pal_core::GsCore`]: the oracle the
//! Rust core is diffed against. Test-only, never shipped (spec decision 3).
//!
//! Also exposes mabur's `FrameFileSource` (maburd `--dry-run --out` frame
//! files) as the M0 replay input, until live captures exist.

use pal_core::{Au, CoreIo, GsCore, Mode, RxBody};
use std::ffi::{c_char, c_int, c_void, CStr, CString};
use std::path::Path;

#[repr(C)]
struct PalRxBody {
    card_id: u8,
    mono_us: u64,
    rssi: [u8; 2],
    snr: [i8; 2],
    evm: [i8; 2],
    phy_valid: u8,
    crc_ok: u8,
    mac_seq: u16,
    tsfl: u32,
    mcs: u8,
    rx_channel: u8,
    body: *const u8,
    body_len: usize,
}

type AuCb = extern "C" fn(*mut c_void, u8, u8, u8, u8, u32, *const u8, usize);
type SendCb = extern "C" fn(*mut c_void, *const u8, usize);

#[repr(C)]
struct PalRefRaw {
    _p: [u8; 0],
}
#[repr(C)]
struct PalFfsRaw {
    _p: [u8; 0],
}

extern "C" {
    fn pal_ref_new(
        config_path: *const c_char,
        mode: c_int,
        adaptive_gap: c_int,
        rz_nonce: u32,
        channel: u8,
        user: *mut c_void,
        on_au: AuCb,
        send: SendCb,
        err: *mut c_char,
        err_cap: usize,
    ) -> *mut PalRefRaw;
    fn pal_ref_free(r: *mut PalRefRaw);
    fn pal_ref_on_rx(r: *mut PalRefRaw, b: *const PalRxBody) -> c_int;
    fn pal_ref_tick(r: *mut PalRefRaw, now_us: u64) -> c_int;
    fn pal_ref_inject_disc_ack_for_replay(r: *mut PalRefRaw, now_us: u64) -> c_int;
    fn pal_ref_frame_gap_timeout_ms(r: *const PalRefRaw) -> u32;
    fn pal_ref_mav_take(r: *mut PalRefRaw, out: *mut u8, cap: usize) -> usize;
    fn pal_ref_mav_push(r: *mut PalRefRaw, p: *const u8, n: usize);
    fn pal_ref_set_idr_requests(r: *mut PalRefRaw, n: u32);
    fn pal_ref_stats_json(r: *const PalRefRaw, buf: *mut c_char, cap: usize) -> usize;

    fn pal_ffs_open(
        path: *const c_char,
        cards: c_int,
        drop_pct: c_int,
        seed: u32,
    ) -> *mut PalFfsRaw;
    fn pal_ffs_free(f: *mut PalFfsRaw);
    fn pal_ffs_next(f: *mut PalFfsRaw, out: *mut PalRxBody) -> c_int;
    fn pal_ffs_frames_read(f: *const PalFfsRaw) -> u64;
    fn pal_ffs_dropped(f: *const PalFfsRaw) -> u64;
}

/// The `CoreIo` of the call in progress. Set for the duration of one
/// `on_rx`/`tick` only; the C++ calls back synchronously from inside it.
struct Ctx {
    io: Option<*mut (dyn CoreIo + 'static)>,
}

extern "C" fn on_au_tramp(
    user: *mut c_void,
    sid: u8,
    flags: u8,
    complete: u8,
    salvaged: u8,
    pts_us: u32,
    data: *const u8,
    len: usize,
) {
    let r = std::panic::catch_unwind(|| {
        // SAFETY: `user` is the boxed Ctx owned by the RefCore making this
        // call; `io` points at the caller's &mut dyn CoreIo for this call.
        let ctx = unsafe { &mut *(user as *mut Ctx) };
        let io = unsafe { &mut *ctx.io.expect("C++ called back outside on_rx/tick") };
        let bytes = if len == 0 {
            &[][..]
        } else {
            unsafe { std::slice::from_raw_parts(data, len) }
        };
        io.on_au(Au {
            data: bytes.to_vec(),
            pts_us,
            sid,
            flags,
            complete: complete != 0,
            salvaged: salvaged != 0,
        });
    });
    if r.is_err() {
        std::process::abort(); // never unwind into C++
    }
}

extern "C" fn send_tramp(user: *mut c_void, data: *const u8, len: usize) {
    let r = std::panic::catch_unwind(|| {
        // SAFETY: as in on_au_tramp.
        let ctx = unsafe { &mut *(user as *mut Ctx) };
        let io = unsafe { &mut *ctx.io.expect("C++ called back outside on_rx/tick") };
        let bytes = if len == 0 {
            &[][..]
        } else {
            unsafe { std::slice::from_raw_parts(data, len) }
        };
        io.send(bytes);
    });
    if r.is_err() {
        std::process::abort();
    }
}

pub struct RefOpts {
    pub mode: Mode,
    /// false = fixed `video.frame_gap_timeout_ms` (parity with maburgs --dry-run).
    pub adaptive_gap: bool,
    /// 0 = random; non-zero pins the DISC nonce and RCF tags.
    pub rz_nonce: u32,
    /// Channel the radio is tuned to; 0 = first `radio.channels` member.
    pub channel: u8,
}

pub struct RefCore {
    raw: *mut PalRefRaw,
    ctx: Box<Ctx>,
}

impl RefCore {
    pub fn new(config: &Path, opts: RefOpts) -> Result<Self, String> {
        let path =
            CString::new(config.as_os_str().as_encoded_bytes()).map_err(|e| e.to_string())?;
        let mut ctx = Box::new(Ctx { io: None });
        let mut err = vec![0 as c_char; 512];
        // SAFETY: all pointers are valid for the call; ctx outlives raw (Drop order below).
        let raw = unsafe {
            pal_ref_new(
                path.as_ptr(),
                if opts.mode == Mode::Gs { 0 } else { 1 },
                c_int::from(opts.adaptive_gap),
                opts.rz_nonce,
                opts.channel,
                &mut *ctx as *mut Ctx as *mut c_void,
                on_au_tramp,
                send_tramp,
                err.as_mut_ptr(),
                err.len(),
            )
        };
        if raw.is_null() {
            let msg = unsafe { CStr::from_ptr(err.as_ptr()) }
                .to_string_lossy()
                .into_owned();
            return Err(msg);
        }
        Ok(Self { raw, ctx })
    }

    pub fn frame_gap_timeout_ms(&self) -> u32 {
        unsafe { pal_ref_frame_gap_timeout_ms(self.raw) }
    }

    /// Replay only: the drone's DISC_ACK, so Gs mode opens a session.
    pub fn inject_disc_ack_for_replay(&mut self, now_us: u64, io: &mut dyn CoreIo) {
        self.with_io(io, |raw| unsafe {
            pal_ref_inject_disc_ack_for_replay(raw, now_us)
        });
    }

    fn with_io(&mut self, io: &mut dyn CoreIo, f: impl FnOnce(*mut PalRefRaw) -> c_int) {
        let p: *mut (dyn CoreIo + '_) = io;
        // SAFETY: the erased lifetime never escapes; io is cleared right after f.
        self.ctx.io = Some(unsafe {
            std::mem::transmute::<*mut (dyn CoreIo + '_), *mut (dyn CoreIo + 'static)>(p)
        });
        let rc = f(self.raw);
        self.ctx.io = None;
        assert_eq!(rc, 0, "the C++ core threw (see stderr)");
    }
}

impl GsCore for RefCore {
    fn on_rx(&mut self, b: &RxBody, io: &mut dyn CoreIo) {
        let raw_body = PalRxBody {
            card_id: b.card_id,
            mono_us: b.mono_us,
            rssi: b.rssi,
            snr: b.snr,
            evm: b.evm,
            phy_valid: u8::from(b.phy_valid),
            crc_ok: u8::from(b.crc_ok),
            mac_seq: b.mac_seq,
            tsfl: b.tsfl,
            mcs: b.mcs,
            rx_channel: b.rx_channel,
            body: b.body.as_ptr(),
            body_len: b.body.len(),
        };
        self.with_io(io, |raw| unsafe { pal_ref_on_rx(raw, &raw_body) });
        // WebGs queues inbound MAVLink for the page; hand it on at once.
        let mut buf = [0u8; 300];
        loop {
            let n = unsafe { pal_ref_mav_take(self.raw, buf.as_mut_ptr(), buf.len()) };
            if n == 0 {
                break;
            }
            io.on_mav(&buf[..n]);
        }
    }

    fn tick(&mut self, now_us: u64, io: &mut dyn CoreIo) {
        self.with_io(io, |raw| unsafe { pal_ref_tick(raw, now_us) });
    }

    fn mav_push_outbound(&mut self, bytes: &[u8]) {
        unsafe { pal_ref_mav_push(self.raw, bytes.as_ptr(), bytes.len()) };
    }

    fn set_idr_requests(&mut self, n: u32) {
        unsafe { pal_ref_set_idr_requests(self.raw, n) };
    }

    fn stats_json(&self) -> String {
        let n = unsafe { pal_ref_stats_json(self.raw, std::ptr::null_mut(), 0) };
        let mut buf = vec![0 as c_char; n + 1];
        unsafe { pal_ref_stats_json(self.raw, buf.as_mut_ptr(), buf.len()) };
        unsafe { CStr::from_ptr(buf.as_ptr()) }
            .to_string_lossy()
            .into_owned()
    }
}

impl Drop for RefCore {
    fn drop(&mut self) {
        // The core's threads (none without cards) join here, before ctx goes.
        unsafe { pal_ref_free(self.raw) };
    }
}

/// maburd `--dry-run --out` frame file as `cards` virtual cards with
/// deterministic per-card loss (mabur's `FrameFileSource`).
pub struct FrameFile {
    raw: *mut PalFfsRaw,
}

impl FrameFile {
    pub fn open(path: &Path, cards: i32, drop_pct: i32, seed: u32) -> Result<Self, String> {
        let p = CString::new(path.as_os_str().as_encoded_bytes()).map_err(|e| e.to_string())?;
        let raw = unsafe { pal_ffs_open(p.as_ptr(), cards, drop_pct, seed) };
        if raw.is_null() {
            return Err(format!("cannot read frame file {}", path.display()));
        }
        Ok(Self { raw })
    }

    pub fn frames_read(&self) -> u64 {
        unsafe { pal_ffs_frames_read(self.raw) }
    }

    pub fn dropped(&self) -> u64 {
        unsafe { pal_ffs_dropped(self.raw) }
    }
}

impl Iterator for FrameFile {
    type Item = RxBody;
    fn next(&mut self) -> Option<RxBody> {
        let mut b = std::mem::MaybeUninit::<PalRxBody>::zeroed();
        if unsafe { pal_ffs_next(self.raw, b.as_mut_ptr()) } == 0 {
            return None;
        }
        let b = unsafe { b.assume_init() };
        let body = if b.body_len == 0 {
            Vec::new()
        } else {
            unsafe { std::slice::from_raw_parts(b.body, b.body_len) }.to_vec()
        };
        Some(RxBody {
            card_id: b.card_id,
            mono_us: b.mono_us,
            rssi: b.rssi,
            snr: b.snr,
            evm: b.evm,
            phy_valid: b.phy_valid != 0,
            crc_ok: b.crc_ok != 0,
            mac_seq: b.mac_seq,
            tsfl: b.tsfl,
            mcs: b.mcs,
            rx_channel: b.rx_channel,
            body,
        })
    }
}

impl Drop for FrameFile {
    fn drop(&mut self) {
        unsafe { pal_ffs_free(self.raw) };
    }
}
