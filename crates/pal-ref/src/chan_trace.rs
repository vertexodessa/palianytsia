//! mabur's C++ ChannelCore, unthreaded, over Rust cards (`cpp/chan_trace.cpp`):
//! the C++ half of the ChannelCore trace harness (`tests/chan_trace.rs`).
//! Every card call goes back into the given `LinkCard`; every sink record
//! comes back as one text line (`take_lines`).

use pal_core::chan::core::{MsClock, Sleeper, UsClock};
use pal_core::chan::radio::LinkCard;
use std::ffi::{c_char, c_int, c_void, CStr, CString};
use std::sync::{Arc, Mutex};

#[repr(C)]
#[derive(Default)]
struct TEnergy {
    fa_valid: u8,
    igi_valid: u8,
    nhm_valid: u8,
    floor_valid: u8,
    cca_ofdm: u32,
    fa_ofdm: u32,
    igi: u8,
    floor_dbm: i8,
}

#[repr(C)]
struct TNhm {
    valid: u8,
    buckets: [u8; 12],
    duration: u16,
    period: u16,
}

type U = *mut c_void;

#[repr(C)]
struct TCardVt {
    user: U,
    retune: extern "C" fn(U, u8) -> c_int,
    read_energy: extern "C" fn(U, c_int, *mut TEnergy),
    read_energy_scout: extern "C" fn(U, *mut TEnergy),
    frames: extern "C" fn(U, *mut u64),
    arm_nhm_busy: extern "C" fn(U, u16) -> c_int,
    read_nhm_busy: extern "C" fn(U, *mut TNhm),
    ready: extern "C" fn(U) -> c_int,
    alive: extern "C" fn(U) -> c_int,
    snr_ok: extern "C" fn(U) -> c_int,
    rx_frames: extern "C" fn(U) -> u64,
    foreign: extern "C" fn(U) -> u64,
    send_control: extern "C" fn(U, *const u8, usize) -> c_int,
    tx_frames: extern "C" fn(U) -> u64,
    tx_fail: extern "C" fn(U) -> u64,
    set_width: extern "C" fn(U, u8, u8) -> c_int,
    channel: extern "C" fn(U) -> u8,
    width: extern "C" fn(U) -> u8,
    tuned_central: extern "C" fn(U) -> c_int,
    can_scout: extern "C" fn(U) -> c_int,
}

#[repr(C)]
struct TEnv {
    user: U,
    now_ms: extern "C" fn(U) -> u64,
    now_us: extern "C" fn(U) -> u64,
    sleep_ms: extern "C" fn(U, c_int),
    line: extern "C" fn(U, *const c_char, usize),
}

#[repr(C)]
struct TCfg {
    config_path: *const c_char,
    channels: *const u8,
    n_channels: usize,
    width: u8,
    pin: u8,
    max_ms: i32,
    start_ch: u8,
    rz_nonce: u32,
}

#[repr(C)]
struct Raw {
    _p: [u8; 0],
}

extern "C" {
    fn pal_ctrace_new(
        c: *const TCfg,
        cards: *const TCardVt,
        n: usize,
        env: *const TEnv,
        err: *mut c_char,
        cap: usize,
    ) -> *mut Raw;
    fn pal_ctrace_free(t: *mut Raw);
    fn pal_ctrace_tick(
        t: *mut Raw,
        now: f64,
        session: c_int,
        tx: c_int,
        fz: *mut c_int,
        b: *mut c_int,
    );
    fn pal_ctrace_vrx_rc(t: *mut Raw, now: f64, b: *const u8, n: usize);
    fn pal_ctrace_vrx_video(t: *mut Raw, now: f64);
    fn pal_ctrace_vrx_step(
        t: *mut Raw,
        now: f64,
        out: *mut u8,
        cap: usize,
        disc: *mut c_int,
    ) -> usize;
    fn pal_ctrace_vrx_session(t: *const Raw) -> c_int;
    fn pal_ctrace_vrx_agreed(t: *const Raw) -> u8;
    fn pal_ctrace_vrx_proposal(t: *const Raw) -> u8;
    fn pal_ctrace_vrx_move_edge(t: *mut Raw);
    fn pal_ctrace_on_rc_body(t: *mut Raw, ch: u8);
    fn pal_ctrace_on_session_opened(t: *mut Raw, b: *const u8, n: usize, now: f64);
    fn pal_ctrace_note_video(t: *mut Raw, ch: u8);
    fn pal_ctrace_note_sent(t: *mut Raw, ok: c_int, rcf: c_int);
    fn pal_ctrace_note_au_end(t: *mut Raw);
    fn pal_ctrace_junk_body(t: *mut Raw, card: c_int, seq: u16);
    fn pal_ctrace_run_scout_step(t: *mut Raw);
    fn pal_ctrace_run_inflight_step(t: *mut Raw);
    fn pal_ctrace_run_sampler_step(t: *mut Raw);
    fn pal_ctrace_on_card_died(t: *mut Raw, card: c_int);
    fn pal_ctrace_on_card_reopened(t: *mut Raw, card: c_int);
    fn pal_ctrace_may_send(t: *mut Raw, card: c_int) -> c_int;
    fn pal_ctrace_op(t: *const Raw) -> u8;
    fn pal_ctrace_pick_open(t: *const Raw) -> c_int;
    fn pal_ctrace_disc_targets(t: *const Raw, tx: c_int, out: *mut c_int, cap: usize) -> usize;
    fn pal_ctrace_disc_for_card(
        t: *const Raw,
        f: *const u8,
        n: usize,
        card: c_int,
        out: *mut u8,
        cap: usize,
    ) -> usize;
    fn pal_ctrace_snapshot(t: *mut Raw, tx: c_int);
}

fn guard<R>(f: impl FnOnce() -> R) -> R {
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(f))
        .unwrap_or_else(|_| std::process::abort())
}

fn card(u: U) -> &'static dyn LinkCard {
    // SAFETY: `u` points at an Arc<dyn LinkCard> in CppTrace::cards (a Vec
    // never grown after construction, so its elements do not move),
    // which outlives the C++ core.
    unsafe { &**(u as *const Arc<dyn LinkCard>) }
}

extern "C" fn c_retune(u: U, ch: u8) -> c_int {
    guard(|| card(u).retune(ch) as c_int)
}
fn put_energy(e: pal_core::chan::radio::ScoutEnergy, o: *mut TEnergy) {
    // SAFETY: the C++ side passes a valid out pointer.
    unsafe {
        *o = TEnergy {
            fa_valid: e.fa_valid as u8,
            igi_valid: e.igi_valid as u8,
            nhm_valid: e.nhm_valid as u8,
            floor_valid: e.floor_valid as u8,
            cca_ofdm: e.cca_ofdm,
            fa_ofdm: e.fa_ofdm,
            igi: e.igi,
            floor_dbm: e.floor_dbm,
        }
    }
}
extern "C" fn c_read_energy(u: U, nhm: c_int, o: *mut TEnergy) {
    guard(|| put_energy(card(u).read_energy(nhm != 0), o))
}
extern "C" fn c_read_energy_scout(u: U, o: *mut TEnergy) {
    guard(|| put_energy(card(u).read_energy_scout(), o))
}
extern "C" fn c_frames(u: U, o: *mut u64) {
    guard(|| {
        let f = card(u).frames();
        // SAFETY: three u64 out slots.
        unsafe { std::ptr::copy_nonoverlapping([f.own, f.foreign, f.own_air_us].as_ptr(), o, 3) }
    })
}
extern "C" fn c_arm_nhm(u: U, p: u16) -> c_int {
    guard(|| card(u).arm_nhm_busy(p) as c_int)
}
extern "C" fn c_read_nhm(u: U, o: *mut TNhm) {
    guard(|| {
        let r = card(u).read_nhm_busy();
        // SAFETY: valid out pointer.
        unsafe {
            *o = TNhm {
                valid: r.valid as u8,
                buckets: r.buckets,
                duration: r.duration,
                period: r.period,
            }
        }
    })
}
extern "C" fn c_ready(u: U) -> c_int {
    guard(|| card(u).ready() as c_int)
}
extern "C" fn c_alive(u: U) -> c_int {
    guard(|| card(u).alive() as c_int)
}
extern "C" fn c_snr_ok(u: U) -> c_int {
    guard(|| card(u).caps().snr_ok as c_int)
}
extern "C" fn c_rx_frames(u: U) -> u64 {
    guard(|| card(u).rx_frames())
}
extern "C" fn c_foreign(u: U) -> u64 {
    guard(|| card(u).foreign())
}
extern "C" fn c_send(u: U, b: *const u8, n: usize) -> c_int {
    guard(|| {
        let s = if n == 0 {
            &[][..]
        } else {
            // SAFETY: n bytes at b.
            unsafe { std::slice::from_raw_parts(b, n) }
        };
        card(u).send_control(s) as c_int
    })
}
extern "C" fn c_tx_frames(u: U) -> u64 {
    guard(|| card(u).tx_frames())
}
extern "C" fn c_tx_fail(u: U) -> u64 {
    guard(|| card(u).tx_fail())
}
extern "C" fn c_set_width(u: U, ch: u8, w: u8) -> c_int {
    guard(|| card(u).set_width(ch, w) as c_int)
}
extern "C" fn c_channel(u: U) -> u8 {
    guard(|| card(u).channel())
}
extern "C" fn c_width(u: U) -> u8 {
    guard(|| card(u).width())
}
extern "C" fn c_tuned_central(u: U) -> c_int {
    guard(|| card(u).tuned_central())
}
extern "C" fn c_can_scout(u: U) -> c_int {
    guard(|| card(u).can_scout() as c_int)
}

struct Env {
    now_ms: MsClock,
    now_us: UsClock,
    sleep: Sleeper,
    lines: Mutex<Vec<String>>,
}

fn env(u: U) -> &'static Env {
    // SAFETY: `u` is the boxed Env owned by CppTrace.
    unsafe { &*(u as *const Env) }
}
extern "C" fn e_now_ms(u: U) -> u64 {
    guard(|| (env(u).now_ms)())
}
extern "C" fn e_now_us(u: U) -> u64 {
    guard(|| (env(u).now_us)())
}
extern "C" fn e_sleep(u: U, ms: c_int) {
    guard(|| (env(u).sleep)(ms))
}
extern "C" fn e_line(u: U, s: *const c_char, n: usize) {
    guard(|| {
        // SAFETY: n bytes at s.
        let b = unsafe { std::slice::from_raw_parts(s as *const u8, n) };
        env(u)
            .lines
            .lock()
            .unwrap()
            .push(String::from_utf8_lossy(b).into_owned());
    })
}

/// The run's knobs over the config file.
pub struct TraceCfg<'a> {
    pub config_path: &'a str,
    pub channels: &'a [u8],
    pub width: u8,
    /// None = auto.
    pub pin: Option<u8>,
    pub max_ms: Option<i32>,
    pub start_ch: u8,
    pub rz_nonce: u32,
}

pub struct CppTrace {
    raw: *mut Raw,
    _cards: Vec<Arc<dyn LinkCard>>,
    env: Box<Env>,
}

impl CppTrace {
    pub fn new(
        cfg: &TraceCfg,
        cards: Vec<Arc<dyn LinkCard>>,
        now_ms: MsClock,
        now_us: UsClock,
        sleep: Sleeper,
    ) -> Result<Self, String> {
        let vts: Vec<TCardVt> = cards
            .iter()
            .map(|b| TCardVt {
                user: b as *const Arc<dyn LinkCard> as U,
                retune: c_retune,
                read_energy: c_read_energy,
                read_energy_scout: c_read_energy_scout,
                frames: c_frames,
                arm_nhm_busy: c_arm_nhm,
                read_nhm_busy: c_read_nhm,
                ready: c_ready,
                alive: c_alive,
                snr_ok: c_snr_ok,
                rx_frames: c_rx_frames,
                foreign: c_foreign,
                send_control: c_send,
                tx_frames: c_tx_frames,
                tx_fail: c_tx_fail,
                set_width: c_set_width,
                channel: c_channel,
                width: c_width,
                tuned_central: c_tuned_central,
                can_scout: c_can_scout,
            })
            .collect();
        let env = Box::new(Env {
            now_ms,
            now_us,
            sleep,
            lines: Mutex::new(Vec::new()),
        });
        let tenv = TEnv {
            user: &*env as *const Env as U,
            now_ms: e_now_ms,
            now_us: e_now_us,
            sleep_ms: e_sleep,
            line: e_line,
        };
        let path = CString::new(cfg.config_path).map_err(|e| e.to_string())?;
        let tcfg = TCfg {
            config_path: path.as_ptr(),
            channels: cfg.channels.as_ptr(),
            n_channels: cfg.channels.len(),
            width: cfg.width,
            pin: cfg.pin.unwrap_or(0),
            max_ms: cfg.max_ms.unwrap_or(-1),
            start_ch: cfg.start_ch,
            rz_nonce: cfg.rz_nonce,
        };
        let mut err = [0 as c_char; 256];
        // SAFETY: every pointer is valid for the call; the cards and env
        // stay boxed in the returned value.
        let raw = unsafe {
            pal_ctrace_new(
                &tcfg,
                vts.as_ptr(),
                vts.len(),
                &tenv,
                err.as_mut_ptr(),
                err.len(),
            )
        };
        if raw.is_null() {
            // SAFETY: NUL-terminated by snprintf.
            return Err(unsafe { CStr::from_ptr(err.as_ptr()) }
                .to_string_lossy()
                .into_owned());
        }
        Ok(Self {
            raw,
            _cards: cards,
            env,
        })
    }

    pub fn take_lines(&self) -> Vec<String> {
        std::mem::take(&mut *self.env.lines.lock().unwrap())
    }
    /// (tx_frozen, dwell_busy)
    pub fn tick(&mut self, now_ms: f64, session: bool, tx: usize) -> (bool, bool) {
        let (mut f, mut b) = (0, 0);
        // SAFETY: raw is live for self's lifetime (here and below).
        unsafe {
            pal_ctrace_tick(
                self.raw,
                now_ms,
                session as c_int,
                tx as c_int,
                &mut f,
                &mut b,
            )
        };
        (f != 0, b != 0)
    }
    pub fn vrx_rc(&mut self, now_ms: f64, b: &[u8]) {
        unsafe { pal_ctrace_vrx_rc(self.raw, now_ms, b.as_ptr(), b.len()) }
    }
    pub fn vrx_video(&mut self, now_ms: f64) {
        unsafe { pal_ctrace_vrx_video(self.raw, now_ms) }
    }
    /// One healthy vrx.step(): (frame, is_disc).
    pub fn vrx_step(&mut self, now_ms: f64) -> Option<(Vec<u8>, bool)> {
        let mut buf = vec![0u8; 2048];
        let mut disc = 0;
        let n = unsafe {
            pal_ctrace_vrx_step(self.raw, now_ms, buf.as_mut_ptr(), buf.len(), &mut disc)
        };
        (n > 0).then(|| {
            buf.truncate(n);
            (buf, disc != 0)
        })
    }
    pub fn vrx_session(&self) -> bool {
        unsafe { pal_ctrace_vrx_session(self.raw) != 0 }
    }
    pub fn vrx_agreed(&self) -> u8 {
        unsafe { pal_ctrace_vrx_agreed(self.raw) }
    }
    pub fn vrx_proposal(&self) -> u8 {
        unsafe { pal_ctrace_vrx_proposal(self.raw) }
    }
    pub fn vrx_move_edge(&mut self) {
        unsafe { pal_ctrace_vrx_move_edge(self.raw) }
    }
    pub fn on_rc_body(&mut self, ch: u8) {
        unsafe { pal_ctrace_on_rc_body(self.raw, ch) }
    }
    pub fn on_session_opened(&mut self, ack: &[u8], now_ms: f64) {
        unsafe { pal_ctrace_on_session_opened(self.raw, ack.as_ptr(), ack.len(), now_ms) }
    }
    pub fn note_video(&mut self, ch: u8) {
        unsafe { pal_ctrace_note_video(self.raw, ch) }
    }
    pub fn note_sent(&mut self, ok: bool, rcf: bool) {
        unsafe { pal_ctrace_note_sent(self.raw, ok as c_int, rcf as c_int) }
    }
    pub fn note_au_end(&mut self) {
        unsafe { pal_ctrace_note_au_end(self.raw) }
    }
    /// One decoded frame on `card` with mac_seq `seq` (not SBI, not RC).
    pub fn junk_body(&mut self, card: usize, seq: u16) {
        unsafe { pal_ctrace_junk_body(self.raw, card as c_int, seq) }
    }
    pub fn run_scout_step(&mut self) {
        unsafe { pal_ctrace_run_scout_step(self.raw) }
    }
    pub fn run_inflight_step(&mut self) {
        unsafe { pal_ctrace_run_inflight_step(self.raw) }
    }
    pub fn run_sampler_step(&mut self) {
        unsafe { pal_ctrace_run_sampler_step(self.raw) }
    }
    pub fn on_card_died(&mut self, card: usize) {
        unsafe { pal_ctrace_on_card_died(self.raw, card as c_int) }
    }
    pub fn on_card_reopened(&mut self, card: usize) {
        unsafe { pal_ctrace_on_card_reopened(self.raw, card as c_int) }
    }
    pub fn may_send(&mut self, card: usize) -> bool {
        unsafe { pal_ctrace_may_send(self.raw, card as c_int) != 0 }
    }
    pub fn op(&self) -> u8 {
        unsafe { pal_ctrace_op(self.raw) }
    }
    pub fn pick_open(&self) -> bool {
        unsafe { pal_ctrace_pick_open(self.raw) != 0 }
    }
    pub fn disc_targets(&self, tx: usize) -> Vec<usize> {
        let mut out = [0 as c_int; 16];
        let n =
            unsafe { pal_ctrace_disc_targets(self.raw, tx as c_int, out.as_mut_ptr(), out.len()) };
        out[..n.min(16)].iter().map(|&c| c as usize).collect()
    }
    pub fn disc_for_card(&self, frame: &[u8], card: usize) -> Vec<u8> {
        let mut out = vec![0u8; 2048];
        let n = unsafe {
            pal_ctrace_disc_for_card(
                self.raw,
                frame.as_ptr(),
                frame.len(),
                card as c_int,
                out.as_mut_ptr(),
                out.len(),
            )
        };
        out.truncate(n);
        out
    }
    /// The snapshot as one "P ..." line, into the line stream.
    pub fn snapshot(&mut self, tx: usize) {
        unsafe { pal_ctrace_snapshot(self.raw, tx as c_int) }
    }
}

impl Drop for CppTrace {
    fn drop(&mut self) {
        // SAFETY: raw came from pal_ctrace_new and is freed once.
        unsafe { pal_ctrace_free(self.raw) }
    }
}
