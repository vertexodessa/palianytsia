//! devourer behind a small safe API: open one card, receive raw dot11 frames
//! with their RX descriptor fields on devourer's RX thread, send GS control
//! frames. Bring-up mirrors mabur's RadioFrontend (see cpp/dv.cpp).

use std::ffi::{c_char, c_int, c_void, CStr};
use std::sync::{Mutex, MutexGuard};

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
    stbc: u8,
    sgi: u8,
}

#[repr(C)]
struct DvCfg {
    index: c_int,
    fd: c_int,
    channel: u8,
    width_mhz: u8,
}

#[repr(C)]
struct DvCard {
    _p: [u8; 0],
}

#[repr(C)]
#[derive(Default)]
struct DvEnergy {
    valid_fa: u8,
    valid_igi: u8,
    valid_nhm: u8,
    valid_floor: u8,
    cca_ofdm: u32,
    fa_ofdm: u32,
    igi: u8,
    floor_dbm: i8,
}

#[repr(C)]
#[derive(Default)]
struct DvNhm {
    valid: u8,
    buckets: [u8; 12],
    duration: u16,
    period: u16,
}

#[repr(C)]
struct DvCaps {
    supported: u8,
    chip: [c_char; 32],
    gen: [c_char; 16],
    tx_chains: c_int,
    rx_chains: c_int,
    bw_mask: u8,
    tune5g_lo: u16,
    tune5g_hi: u16,
    fast_retune: u8,
}

type RxCb = extern "C" fn(*mut c_void, *const DvRx);

extern "C" {
    fn dv_open(cfg: *const DvCfg, err: *mut c_char, cap: usize) -> *mut DvCard;
    fn dv_start_rx(c: *mut DvCard, cb: RxCb, user: *mut c_void) -> c_int;
    fn dv_send_control(c: *mut DvCard, body: *const u8, len: usize) -> c_int;
    fn dv_chip(c: *const DvCard) -> *const c_char;
    fn dv_radio(c: *mut DvCard) -> *mut c_void;
    fn dv_close(c: *mut DvCard);
    fn dv_get_caps(c: *mut DvCard, out: *mut DvCaps) -> c_int;
    fn dv_rx_energy(c: *mut DvCard, with_nhm: c_int, out: *mut DvEnergy) -> c_int;
    fn dv_rx_energy_scout(c: *mut DvCard, out: *mut DvEnergy) -> c_int;
    fn dv_fast_retune(c: *mut DvCard, ch: u8) -> c_int;
    fn dv_set_monitor(c: *mut DvCard, ch: u8, width_mhz: u8) -> c_int;
    fn dv_tuned_central(c: *mut DvCard) -> c_int;
    fn dv_arm_nhm_busy(c: *mut DvCard, period_4us: u16) -> c_int;
    fn dv_read_nhm_busy(c: *mut DvCard, out: *mut DvNhm) -> c_int;
}

/// devourer `RxEnergy`: the counters a channel search reads.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Energy {
    pub valid_fa: bool,
    pub valid_igi: bool,
    pub valid_nhm: bool,
    pub valid_floor: bool,
    pub cca_ofdm: u32,
    pub fa_ofdm: u32,
    pub igi: u8,
    pub floor_dbm: i8,
}

impl From<DvEnergy> for Energy {
    fn from(e: DvEnergy) -> Self {
        Self {
            valid_fa: e.valid_fa != 0,
            valid_igi: e.valid_igi != 0,
            valid_nhm: e.valid_nhm != 0,
            valid_floor: e.valid_floor != 0,
            cca_ofdm: e.cca_ofdm,
            fa_ofdm: e.fa_ofdm,
            igi: e.igi,
            floor_dbm: e.floor_dbm,
        }
    }
}

/// devourer `NhmBusy`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NhmBusy {
    pub valid: bool,
    pub buckets: [u8; 12],
    pub duration: u16,
    pub period: u16,
}

/// devourer `AdapterCaps`, the part a channel search uses.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Caps {
    pub supported: bool,
    pub chip: String,
    pub gen: String,
    pub tx_chains: i32,
    pub rx_chains: i32,
    pub bw_mask: u8,
    /// 0 = unknown.
    pub tune5g_lo: u16,
    pub tune5g_hi: u16,
    pub fast_retune: bool,
}

fn c_str(b: &[c_char]) -> String {
    let n = b.iter().position(|&c| c == 0).unwrap_or(b.len());
    // c_char is i8 on x86_64 and u8 on aarch64: the cast is a no-op there.
    #[allow(clippy::unnecessary_cast)]
    let bytes: Vec<u8> = b[..n].iter().map(|&c| c as u8).collect();
    String::from_utf8_lossy(&bytes).into_owned()
}

/// The card's control plane, held exclusively: sends, retunes and register
/// reads never overlap (devourer's control calls are single-thread). A
/// caller that must keep its own state in step with the card (the channel
/// it is on) updates it while holding this.
pub struct Ctl<'a> {
    raw: *mut DvCard,
    _g: MutexGuard<'a, ()>,
}

impl Ctl<'_> {
    pub fn caps(&self) -> Caps {
        let mut o = DvCaps {
            supported: 0,
            chip: [0; 32],
            gen: [0; 16],
            tx_chains: 0,
            rx_chains: 0,
            bw_mask: 0,
            tune5g_lo: 0,
            tune5g_hi: 0,
            fast_retune: 0,
        };
        if unsafe { dv_get_caps(self.raw, &mut o) } != 0 {
            return Caps::default();
        }
        Caps {
            supported: o.supported != 0,
            chip: c_str(&o.chip),
            gen: c_str(&o.gen),
            tx_chains: o.tx_chains,
            rx_chains: o.rx_chains,
            bw_mask: o.bw_mask,
            tune5g_lo: o.tune5g_lo,
            tune5g_hi: o.tune5g_hi,
            fast_retune: o.fast_retune != 0,
        }
    }

    /// GetRxEnergy; all-invalid on a failed read.
    pub fn rx_energy(&self, with_nhm: bool) -> Energy {
        let mut o = DvEnergy::default();
        if unsafe { dv_rx_energy(self.raw, c_int::from(with_nhm), &mut o) } != 0 {
            return Energy::default();
        }
        o.into()
    }

    /// GetRxEnergyScout: fa/cca since the previous read.
    pub fn rx_energy_scout(&self) -> Energy {
        let mut o = DvEnergy::default();
        if unsafe { dv_rx_energy_scout(self.raw, &mut o) } != 0 {
            return Energy::default();
        }
        o.into()
    }

    /// FastRetune: the RF switch only, width kept.
    pub fn fast_retune(&self, ch: u8) -> bool {
        unsafe { dv_fast_retune(self.raw, ch) == 0 }
    }

    /// The full SetMonitorChannel.
    pub fn set_monitor(&self, ch: u8, width_mhz: u8) -> Result<(), String> {
        match unsafe { dv_set_monitor(self.raw, ch, width_mhz) } {
            0 => Ok(()),
            -2 => Err(format!("channel {ch} has no 40 MHz pair")),
            r => Err(format!("set_monitor({ch}, {width_mhz}) failed ({r})")),
        }
    }

    /// -1 = unknown.
    pub fn tuned_central(&self) -> i32 {
        unsafe { dv_tuned_central(self.raw) }
    }

    pub fn arm_nhm_busy(&self, period_4us: u16) -> bool {
        unsafe { dv_arm_nhm_busy(self.raw, period_4us) != 0 }
    }

    pub fn read_nhm_busy(&self) -> NhmBusy {
        let mut o = DvNhm::default();
        if unsafe { dv_read_nhm_busy(self.raw, &mut o) } != 0 {
            return NhmBusy::default();
        }
        NhmBusy {
            valid: o.valid != 0,
            buckets: o.buckets,
            duration: o.duration,
            period: o.period,
        }
    }

    /// GS uplink control frame around an RC body.
    pub fn send_control(&self, body: &[u8]) -> bool {
        unsafe { dv_send_control(self.raw, body.as_ptr(), body.len()) == 0 }
    }
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
    pub stbc: bool,
    pub sgi: bool,
}

#[derive(Clone, Debug)]
pub enum Select {
    /// 0bda:{a81a,881a,8812}, the n-th match.
    Scan { index: i32 },
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
    /// The control-plane lock (`ctl`): sends, retunes, register reads.
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
            stbc: rx.stbc != 0,
            sgi: rx.sgi != 0,
        });
    }));
    if r.is_err() {
        std::process::abort(); // never unwind into C++
    }
}

impl Card {
    pub fn open(cfg: &Config) -> Result<Card, String> {
        let (index, fd) = match cfg.select {
            Select::Scan { index } => (index, -1),
            Select::Fd(fd) => (0, fd),
        };
        let c = DvCfg {
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

    /// devourer's IRtlRadio behind this card, for a C++ control plane in
    /// the same binary (pal-ref's LinkCard). Valid while the Card lives.
    pub fn radio_ptr(&self) -> *mut c_void {
        unsafe { dv_radio(self.raw) }
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

    /// The control plane, exclusively until the guard drops. Never take it
    /// from the RX handler: a control call can wait on a USB transfer.
    pub fn ctl(&self) -> Ctl<'_> {
        Ctl {
            raw: self.raw,
            _g: self.tx.lock().unwrap_or_else(|e| e.into_inner()),
        }
    }

    /// GS uplink control frame around an RC body. Thread-safe.
    pub fn send_control(&self, body: &[u8]) -> bool {
        self.ctl().send_control(body)
    }
}

impl Drop for Card {
    fn drop(&mut self) {
        // Joins the RX thread before the handler box is freed.
        unsafe { dv_close(self.raw) };
        self.handler.take();
    }
}
