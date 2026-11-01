//! devourer behind a small safe API: open one card, receive raw dot11 frames
//! with their RX descriptor fields on devourer's RX thread, send GS control
//! frames. Bring-up mirrors mabur's RadioFrontend (see cpp/dv.cpp).

use std::ffi::{c_char, c_int, c_void, CStr};
use std::sync::Mutex;

#[repr(C)]
struct DvRx {
    frame: *const u8,
    len: usize,
    crc_err: u8,
    physt: u8,
    data_rate: u16,
    rssi: [u8; 2],
    snr: [i8; 2],
    evm: [i8; 2],
    tsfl: u32,
    bw40: u8,
}

#[repr(C)]
struct DvCfg {
    vid: u16,
    pid: u16,
    index: c_int,
    fd: c_int,
    channel: u8,
    width_mhz: u8,
}

#[repr(C)]
struct DvCard {
    _p: [u8; 0],
}

type RxCb = extern "C" fn(*mut c_void, *const DvRx);

extern "C" {
    fn dv_open(cfg: *const DvCfg, err: *mut c_char, cap: usize) -> *mut DvCard;
    fn dv_start_rx(c: *mut DvCard, cb: RxCb, user: *mut c_void) -> c_int;
    fn dv_send_control(c: *mut DvCard, body: *const u8, len: usize) -> c_int;
    fn dv_set_channel(c: *mut DvCard, ch: u8, width_mhz: u8) -> c_int;
    fn dv_chip(c: *const DvCard) -> *const c_char;
    fn dv_close(c: *mut DvCard);
}

/// One received frame, borrowed for the duration of the callback.
pub struct RxFrame<'a> {
    /// dot11 frame as the chip delivered it, FCS included.
    pub frame: &'a [u8],
    pub crc_err: bool,
    pub physt: bool,
    pub data_rate: u16,
    pub rssi: [u8; 2],
    pub snr: [i8; 2],
    pub evm: [i8; 2],
    pub tsfl: u32,
    pub bw40: bool,
}

#[derive(Clone, Debug)]
pub enum Select {
    /// 0bda:{a81a,881a,8812}, the n-th match.
    Scan {
        index: i32,
    },
    Id {
        vid: u16,
        pid: u16,
        index: i32,
    },
    /// An already opened usbfs fd (Android `UsbDeviceConnection`).
    Fd(i32),
}

#[derive(Clone, Debug)]
pub struct Config {
    pub select: Select,
    pub channel: u8,
    pub width_mhz: u8,
}

type Handler = Box<dyn FnMut(&RxFrame) + Send>;

pub struct Card {
    raw: *mut DvCard,
    tx: Mutex<()>,
    // Boxed twice so the C side holds a thin pointer that outlives start_rx.
    handler: Option<Box<Handler>>,
}

// The C side serialises nothing itself; TX goes through `tx`, RX runs on
// devourer's own thread and touches only `handler`.
unsafe impl Send for Card {}
unsafe impl Sync for Card {}

extern "C" fn rx_tramp(user: *mut c_void, rx: *const DvRx) {
    let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        // SAFETY: user is the Box<Handler> owned by the Card, alive until
        // dv_close joins the RX thread; rx is valid for this call.
        let h = unsafe { &mut *(user as *mut Handler) };
        let rx = unsafe { &*rx };
        let frame = if rx.len == 0 {
            &[][..]
        } else {
            unsafe { std::slice::from_raw_parts(rx.frame, rx.len) }
        };
        h(&RxFrame {
            frame,
            crc_err: rx.crc_err != 0,
            physt: rx.physt != 0,
            data_rate: rx.data_rate,
            rssi: rx.rssi,
            snr: rx.snr,
            evm: rx.evm,
            tsfl: rx.tsfl,
            bw40: rx.bw40 != 0,
        });
    }));
    if r.is_err() {
        std::process::abort(); // never unwind into C++
    }
}

impl Card {
    pub fn open(cfg: &Config) -> Result<Card, String> {
        let (vid, pid, index, fd) = match cfg.select {
            Select::Scan { index } => (0, 0, index, -1),
            Select::Id { vid, pid, index } => (vid, pid, index, -1),
            Select::Fd(fd) => (0, 0, 0, fd),
        };
        let c = DvCfg {
            vid,
            pid,
            index,
            fd,
            channel: cfg.channel,
            width_mhz: cfg.width_mhz,
        };
        let mut err = [0 as c_char; 256];
        let raw = unsafe { dv_open(&c, err.as_mut_ptr(), err.len()) };
        if raw.is_null() {
            return Err(unsafe { CStr::from_ptr(err.as_ptr()) }
                .to_string_lossy()
                .into_owned());
        }
        Ok(Card {
            raw,
            tx: Mutex::new(()),
            handler: None,
        })
    }

    pub fn chip(&self) -> Option<String> {
        let p = unsafe { dv_chip(self.raw) };
        (!p.is_null()).then(|| unsafe { CStr::from_ptr(p) }.to_string_lossy().into_owned())
    }

    /// Starts the RX thread. `f` runs on it for every frame; keep it short
    /// and non-blocking (spec 5.4).
    pub fn start_rx(&mut self, f: impl FnMut(&RxFrame) + Send + 'static) -> Result<(), String> {
        if self.handler.is_some() {
            return Err("RX already started".into());
        }
        let mut h: Box<Handler> = Box::new(Box::new(f));
        let user = &mut *h as *mut Handler as *mut c_void;
        if unsafe { dv_start_rx(self.raw, rx_tramp, user) } != 0 {
            return Err("dv_start_rx failed".into());
        }
        self.handler = Some(h);
        Ok(())
    }

    /// Moves the card to `ch` / `width_mhz` (FastRetune when only the
    /// channel changes). Shares the TX lock, so it never overlaps a send;
    /// never call it from the RX handler.
    pub fn set_channel(&self, ch: u8, width_mhz: u8) -> Result<(), String> {
        let _g = self.tx.lock().unwrap_or_else(|e| e.into_inner());
        match unsafe { dv_set_channel(self.raw, ch, width_mhz) } {
            0 => Ok(()),
            -2 => Err(format!("channel {ch} has no 40 MHz pair")),
            r => Err(format!("set_channel({ch}, {width_mhz}) failed ({r})")),
        }
    }

    /// GS uplink control frame around an RC body. Thread-safe.
    pub fn send_control(&self, body: &[u8]) -> bool {
        let _g = self.tx.lock().unwrap_or_else(|e| e.into_inner());
        unsafe { dv_send_control(self.raw, body.as_ptr(), body.len()) == 0 }
    }
}

impl Drop for Card {
    fn drop(&mut self) {
        // Joins the RX thread before the handler box is freed.
        unsafe { dv_close(self.raw) };
        self.handler.take();
    }
}
