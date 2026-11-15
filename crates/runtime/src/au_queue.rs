//! The AU queue from the core loop to the decoder thread, with its depth in
//! view of both ends.
//!
//! When the decoder falls behind, the queue fills. The drone's bundle runs
//! two temporal layers (`ref_base = 1`, `ref_enhance = 1`): enhance AUs
//! (sid 1) reference only the base, so skipping one costs one frame and no
//! keyframe. From `skip_enh_at` queued AUs on, enhance AUs are skipped; a
//! base AU is dropped only when the queue is full, and that drop breaks the
//! reference chain (the caller asks for a keyframe).

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, SyncSender, TrySendError};
use std::sync::Arc;
use std::time::Duration;

/// An AU on its way to the decoder: its number (`IdrDriver::aus`), the
/// `lat_us()` it was queued at, its bytes.
pub type QueuedAu = (u64, u64, Vec<u8>);

/// What `AuTx::offer` did with an AU.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Offer {
    Queued,
    /// An enhance AU left out because the decoder is behind: no keyframe
    /// needed.
    SkippedEnh,
    /// The queue was full: the AU is lost and so is the reference chain.
    Dropped,
    /// The decoder end is gone.
    Closed,
}

/// The decision alone, for the depth `len` of a queue that skips enhance
/// AUs from `skip_enh_at` on.
pub fn admit(len: usize, skip_enh_at: usize, sid: u8) -> bool {
    sid == 0 || len < skip_enh_at
}

struct Depth {
    len: AtomicUsize,
    /// Highest `len` since the decoder end last took it.
    high: AtomicUsize,
}

pub struct AuTx {
    tx: SyncSender<QueuedAu>,
    depth: Arc<Depth>,
    skip_enh_at: usize,
}

pub struct AuRx {
    rx: Receiver<QueuedAu>,
    depth: Arc<Depth>,
}

/// A queue of `cap` AUs that skips enhance AUs once half of it is taken.
pub fn au_queue(cap: usize) -> (AuTx, AuRx) {
    let (tx, rx) = mpsc::sync_channel(cap);
    let depth = Arc::new(Depth {
        len: AtomicUsize::new(0),
        high: AtomicUsize::new(0),
    });
    (
        AuTx {
            tx,
            depth: depth.clone(),
            skip_enh_at: (cap / 2).max(1),
        },
        AuRx { rx, depth },
    )
}

impl AuTx {
    pub fn offer(&self, au: QueuedAu, sid: u8) -> Offer {
        let d = &self.depth;
        if !admit(d.len.load(Ordering::Acquire), self.skip_enh_at, sid) {
            return Offer::SkippedEnh;
        }
        // Counted before the send so the receiver's decrement never runs
        // ahead of it (the count would wrap below zero).
        let len = d.len.fetch_add(1, Ordering::AcqRel) + 1;
        match self.tx.try_send(au) {
            Ok(()) => {
                d.high.fetch_max(len, Ordering::Relaxed);
                Offer::Queued
            }
            Err(e) => {
                d.len.fetch_sub(1, Ordering::AcqRel);
                match e {
                    TrySendError::Full(_) => Offer::Dropped,
                    TrySendError::Disconnected(_) => Offer::Closed,
                }
            }
        }
    }
}

impl AuRx {
    fn took(&self, x: QueuedAu) -> QueuedAu {
        self.depth.len.fetch_sub(1, Ordering::AcqRel);
        x
    }

    pub fn recv(&self) -> Option<QueuedAu> {
        self.rx.recv().ok().map(|x| self.took(x))
    }

    pub fn try_recv(&self) -> Option<QueuedAu> {
        self.rx.try_recv().ok().map(|x| self.took(x))
    }

    pub fn recv_timeout(&self, d: Duration) -> Result<QueuedAu, RecvTimeoutError> {
        self.rx.recv_timeout(d).map(|x| self.took(x))
    }

    /// The deepest the queue was since the last call (what is queued now
    /// starts the next window).
    pub fn take_high(&self) -> usize {
        let now = self.depth.len.load(Ordering::Acquire);
        self.depth.high.swap(now, Ordering::Relaxed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn au(n: u64) -> QueuedAu {
        (n, 0, vec![n as u8])
    }

    #[test]
    fn a_behind_decoder_loses_enhance_aus_first_and_base_aus_only_when_full() {
        let (tx, rx) = au_queue(8);
        // Four queued: half full, enhance still goes in below that.
        for n in 0..3 {
            assert_eq!(tx.offer(au(n), (n % 2) as u8), Offer::Queued);
        }
        assert_eq!(tx.offer(au(3), 1), Offer::Queued);
        assert_eq!(tx.offer(au(4), 1), Offer::SkippedEnh);
        for n in 5..9 {
            assert_eq!(tx.offer(au(n), 0), Offer::Queued);
        }
        assert_eq!(tx.offer(au(9), 0), Offer::Dropped);
        assert_eq!(rx.take_high(), 8);
        // Drained to three: enhance AUs are taken again.
        for n in [0, 1, 2, 3, 5] {
            assert_eq!(rx.try_recv().map(|x| x.0), Some(n));
        }
        assert_eq!(tx.offer(au(10), 1), Offer::Queued);
        // A window starts at what was queued when the last one was taken.
        assert_eq!(rx.take_high(), 8);
        assert_eq!(rx.take_high(), 4);
        drop(rx);
        assert_eq!(tx.offer(au(11), 0), Offer::Closed);
    }
}
