//! Test-only fake USB card (mabur `tests/fake_link_card.h`, USB part) and
//! fake clock. Every device call is recorded; the busy rate per channel is
//! programmable; one NHM read can be made to block until released.

use std::sync::atomic::{AtomicU64, Ordering::SeqCst};
use std::sync::{Arc, Condvar, Mutex, MutexGuard};

use super::radio::{CardCaps, LinkCard, NhmBusyRead, ScoutEnergy, ScoutFrames, ScoutRadio};

#[derive(Clone, Default)]
pub struct FakeClock(pub Arc<AtomicU64>);

impl FakeClock {
    pub fn ms(&self) -> u64 {
        self.0.load(SeqCst)
    }
    pub fn set(&self, ms: u64) {
        self.0.store(ms, SeqCst)
    }
    pub fn advance(&self, ms: u64) {
        self.0.fetch_add(ms, SeqCst);
    }
}

pub struct FakeState {
    pub ch: u8,
    pub width_mhz: u8,
    pub is_ready: bool,
    pub is_alive: bool,
    pub scout_ok: bool,
    pub retune_ok: bool,
    pub calls: Vec<String>,
    pub sent: Vec<Vec<u8>>,
    pub cca_per_ms_on: Vec<u32>,
    pub foreign_per_ms_on: Vec<u32>,
    pub fr: ScoutFrames,
    pub rx: u64,
    pub foreign_n: u64,
    pub tx: u64,
    pub txf: u64,
    pub last_read: u64,
    /// Returned once by read_energy_scout's fa_ofdm.
    pub fa_next: u32,
    pub nhm_arm_ok: bool,
    pub nhm_read: NhmBusyRead,
    // the blocking read
    pub block_next_read: bool,
    pub in_read: bool,
    pub release: bool,
    /// The first device call of a pass (ready()) happened.
    pub touched: bool,
}

pub struct FakeCard {
    pub st: Mutex<FakeState>,
    pub cv: Condvar,
    pub clk: FakeClock,
}

impl FakeCard {
    pub fn new(clk: FakeClock, ch: u8) -> Arc<Self> {
        Arc::new(Self {
            st: Mutex::new(FakeState {
                ch,
                width_mhz: 20,
                is_ready: true,
                is_alive: true,
                scout_ok: true,
                retune_ok: true,
                calls: Vec::new(),
                sent: Vec::new(),
                cca_per_ms_on: vec![0; 256],
                foreign_per_ms_on: vec![0; 256],
                fr: ScoutFrames::default(),
                rx: 0,
                foreign_n: 0,
                tx: 0,
                txf: 0,
                last_read: 0,
                fa_next: 0,
                nhm_arm_ok: false,
                nhm_read: NhmBusyRead::default(),
                block_next_read: false,
                in_read: false,
                release: false,
                touched: false,
            }),
            cv: Condvar::new(),
            clk,
        })
    }
    pub fn s(&self) -> MutexGuard<'_, FakeState> {
        self.st.lock().unwrap()
    }
    pub fn calls(&self) -> Vec<String> {
        self.s().calls.clone()
    }
    fn call(&self, c: String) {
        self.s().calls.push(c);
    }
}

impl ScoutRadio for FakeCard {
    fn retune(&self, c: u8) -> bool {
        let mut s = self.s();
        s.calls.push(format!("retune {c}"));
        if !s.retune_ok {
            return false;
        }
        s.ch = c;
        true
    }
    fn retune_width(&self, c: u8, w: u8) -> bool {
        let mut s = self.s();
        s.calls.push(format!("retune_width {c}/{w}"));
        s.ch = c;
        s.width_mhz = w;
        true
    }
    fn read_energy(&self, with_nhm: bool) -> ScoutEnergy {
        let now = self.clk.ms();
        let mut s = self.s();
        let dt = now - s.last_read;
        let ch = s.ch as usize;
        let mut e = ScoutEnergy {
            fa_valid: true,
            cca_ofdm: (dt as u32).wrapping_mul(s.cca_per_ms_on[ch]),
            ..Default::default()
        };
        s.fr.foreign += dt * s.foreign_per_ms_on[ch] as u64;
        s.last_read = now;
        if with_nhm {
            e.floor_valid = true;
            e.floor_dbm = -95;
        }
        e
    }
    fn read_energy_scout(&self) -> ScoutEnergy {
        let mut s = self.s();
        ScoutEnergy {
            fa_valid: true,
            fa_ofdm: std::mem::take(&mut s.fa_next),
            ..Default::default()
        }
    }
    fn frames(&self) -> ScoutFrames {
        self.s().fr
    }
    fn arm_nhm_busy(&self, period_4us: u16) -> bool {
        let mut s = self.s();
        s.calls.push(format!("arm_nhm {period_4us}"));
        s.nhm_arm_ok
    }
    fn read_nhm_busy(&self) -> NhmBusyRead {
        let mut s = self.s();
        s.calls.push("read_nhm".into());
        if s.block_next_read {
            s.block_next_read = false;
            s.in_read = true;
            self.cv.notify_all();
            s = self.cv.wait_while(s, |s| !s.release).unwrap();
            s.in_read = false;
        }
        s.nhm_read
    }
}

impl LinkCard for FakeCard {
    fn ready(&self) -> bool {
        let mut s = self.s();
        s.touched = true;
        self.cv.notify_all();
        s.is_ready
    }
    fn alive(&self) -> bool {
        self.s().is_alive
    }
    fn caps(&self) -> CardCaps {
        CardCaps {
            valid: true,
            chip: "RTL8822E".into(),
            gen: "jaguar3".into(),
            tx_chains: 2,
            rx_chains: 2,
            fast_retune: true,
            fa_ok: true,
            igi_ok: true,
            nhm_ok: true,
            snr_ok: true,
            ..Default::default()
        }
    }
    fn rx_frames(&self) -> u64 {
        self.s().rx
    }
    fn foreign(&self) -> u64 {
        self.s().foreign_n
    }
    fn send_control(&self, body: &[u8]) -> bool {
        let mut s = self.s();
        s.sent.push(body.to_vec());
        s.tx += 1;
        true
    }
    fn tx_frames(&self) -> u64 {
        self.s().tx
    }
    fn tx_fail(&self) -> u64 {
        self.s().txf
    }
    fn set_width(&self, c: u8, w: u8) -> bool {
        self.call(format!("set_width {c}/{w}"));
        let mut s = self.s();
        s.ch = c;
        s.width_mhz = w;
        true
    }
    fn channel(&self) -> u8 {
        self.s().ch
    }
    fn width(&self) -> u8 {
        self.s().width_mhz
    }
    fn tuned_central(&self) -> i32 {
        -1
    }
    fn can_scout(&self) -> bool {
        self.s().scout_ok
    }
}
