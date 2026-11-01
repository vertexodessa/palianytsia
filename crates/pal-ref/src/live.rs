//! A live devourer-sys card handed to the C++ core (`runtime::Attach`):
//! mabur's whole GS, channel core included, drives and transmits through it.
use std::path::PathBuf;
use std::sync::Arc;

use crate::{CardRx, CardSend, RefCard, RefCore};

/// The card as the core's LinkCard, seen from the RX thread.
struct RefTap(Arc<RefCard>);

impl runtime::RxTap for RefTap {
    fn on_rx(&self, f: &devourer_sys::RxFrame, v: &pal_wire::dot11::Verdict) -> Option<u8> {
        let a = CardRx {
            crc_err: u8::from(f.crc_err),
            physt: u8::from(f.physt),
            data_rate: f.data_rate,
            rssi: f.rssi,
            snr: f.snr,
            evm: f.evm,
            tsfl: f.tsfl,
            bw40: u8::from(f.bw40),
            stbc: u8::from(f.stbc),
            sgi: u8::from(f.sgi),
        };
        let v = match v {
            pal_wire::dot11::Verdict::Body(_) => 0,
            pal_wire::dot11::Verdict::Short => 1,
            pal_wire::dot11::Verdict::Foreign => 2,
        };
        self.0.on_rx(v, f.frame.len(), &a)
    }
    fn channel(&self) -> u8 {
        self.0.channel()
    }
}

/// Builds the C++ core on the card once it is open at `ch` / `width`.
pub fn channel_core(config: PathBuf, ch: u8, width: u8) -> runtime::Attach {
    Box::new(move |cards| {
        // mabur's WebGs drives one card; any further ones stay untapped.
        let card = cards.first().ok_or("no card")?;
        // Weak: the card's RX handler holds the tap, so a strong ref would
        // keep the card open forever.
        let tx = Arc::downgrade(card);
        let send: CardSend =
            Box::new(move |body| tx.upgrade().is_some_and(|c| c.send_control(body)));
        // SAFETY: the card outlives the tap and the core (run_live drops
        // the core first; the tap goes with the card's RX handler).
        let rc = Arc::new(unsafe { RefCard::new(card.radio_ptr(), ch, width, send)? });
        let core = RefCore::new_with_card(&config, rc.clone())?;
        Ok(runtime::Attached {
            core: Box::new(core),
            taps: vec![Arc::new(RefTap(rc))],
            clock: crate::now_us,
        })
    })
}
