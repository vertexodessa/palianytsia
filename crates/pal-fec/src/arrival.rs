//! Pre-FEC loss booked at arrival time (mabur `arrival_tracker.h`), in the
//! decoder's virtual-seq space: `expected` is seq advance past a settle line
//! `guard` seqs behind the newest, `arrived` is what was heard. Counters are
//! monotonic; a seq heard after it settled counts `late`.

pub const DEFAULT_GUARD: u32 = 32;
const HEARD: u8 = 1;
const CLEAN: u8 = 2;

#[derive(Clone, Debug)]
pub struct ArrivalTracker {
    guard: u32,
    bits: Vec<u8>,
    mask: u64,
    anchored: bool,
    newest: u64,
    settle: u64,
    pub expected: u64,
    pub arrived: u64,
    pub expected_stale: u64,
    pub arrived_stale: u64,
    pub late: u64,
    pub salvage_only: u64,
}

impl ArrivalTracker {
    pub fn new(guard: u32) -> Self {
        Self::with_ring(guard, 1024)
    }

    pub fn with_ring(guard: u32, ring_bits: usize) -> Self {
        assert!(ring_bits.is_power_of_two());
        Self {
            guard,
            bits: vec![0; ring_bits],
            mask: ring_bits as u64 - 1,
            anchored: false,
            newest: 0,
            settle: 0,
            expected: 0,
            arrived: 0,
            expected_stale: 0,
            arrived_stale: 0,
            late: 0,
            salvage_only: 0,
        }
    }

    pub fn anchored(&self) -> bool {
        self.anchored
    }

    pub fn on_source(&mut self, v: u64, stale_end: u64, clean: bool) {
        if !self.anchored {
            self.anchor(v);
        }
        if v < self.settle {
            self.late += 1;
            return;
        }
        let ring = self.mask + 1;
        if v >= self.settle.wrapping_add(ring) {
            self.settle_to(v - ring + 1, stale_end);
        }
        self.bits[(v & self.mask) as usize] |= if clean { HEARD | CLEAN } else { HEARD };
        if v > self.newest {
            self.newest = v;
        }
    }

    pub fn advance(&mut self, newest: u64, stale_end: u64) {
        if !self.anchored {
            self.anchor(newest);
        }
        if newest > self.newest {
            self.newest = newest;
        }
        if self.newest < u64::from(self.guard) {
            return;
        }
        self.settle_to(self.newest - u64::from(self.guard) + 1, stale_end);
    }

    pub fn reset(&mut self, anchor_v: u64) {
        self.bits.fill(0);
        self.anchored = false;
        self.anchor(anchor_v);
    }

    fn anchor(&mut self, v: u64) {
        self.anchored = true;
        self.newest = v;
        self.settle = v;
    }

    fn settle_to(&mut self, target: u64, stale_end: u64) {
        while self.settle < target {
            let i = (self.settle & self.mask) as usize;
            let heard = self.bits[i] & HEARD != 0;
            let clean = self.bits[i] & CLEAN != 0;
            self.bits[i] = 0;
            self.expected += 1;
            if heard {
                self.arrived += 1;
            }
            if heard && !clean {
                self.salvage_only += 1;
            }
            if self.settle < stale_end {
                self.expected_stale += 1;
                if heard {
                    self.arrived_stale += 1;
                }
            }
            self.settle += 1;
        }
    }
}
