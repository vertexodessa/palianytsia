//! `bodies.bin`: every `RxBody` as the core saw it plus every `tick` time,
//! in order, so a replay feeds a core exactly what it got live (spec 7, 9).
//!
//! Layout, little-endian:
//!   magic line `palcap 1\n`
//!   records: tag u8
//!     1 body: card_id u8, mono_us u64, rssi [u8;2], snr [i8;2], evm [i8;2],
//!             flags u8 (bit0 phy_valid, bit1 crc_ok), mac_seq u16, tsfl u32,
//!             mcs u8, rx_channel u8, len u32, body [u8; len]
//!     2 tick: now_us u64
use pal_core::RxBody;
use std::io::{self, BufRead, Write};

pub const MAGIC: &[u8] = b"palcap 1\n";
const TAG_BODY: u8 = 1;
const TAG_TICK: u8 = 2;

#[derive(Clone, Debug)]
pub enum Record {
    Body(RxBody),
    Tick(u64),
}

pub struct Writer<W: Write> {
    w: W,
}

impl<W: Write> Writer<W> {
    pub fn new(mut w: W) -> io::Result<Self> {
        w.write_all(MAGIC)?;
        Ok(Self { w })
    }

    pub fn body(&mut self, b: &RxBody) -> io::Result<()> {
        let mut h = [0u8; 31];
        h[0] = TAG_BODY;
        h[1] = b.card_id;
        h[2..10].copy_from_slice(&b.mono_us.to_le_bytes());
        h[10..12].copy_from_slice(&b.rssi);
        h[12] = b.snr[0] as u8;
        h[13] = b.snr[1] as u8;
        h[14] = b.evm[0] as u8;
        h[15] = b.evm[1] as u8;
        h[16] = u8::from(b.phy_valid) | (u8::from(b.crc_ok) << 1);
        h[17..19].copy_from_slice(&b.mac_seq.to_le_bytes());
        h[19..23].copy_from_slice(&b.tsfl.to_le_bytes());
        h[23] = b.mcs;
        h[24] = b.rx_channel;
        h[25..29].copy_from_slice(&(b.body.len() as u32).to_le_bytes());
        self.w.write_all(&h[..29])?;
        self.w.write_all(&b.body)
    }

    pub fn tick(&mut self, now_us: u64) -> io::Result<()> {
        self.w.write_all(&[TAG_TICK])?;
        self.w.write_all(&now_us.to_le_bytes())
    }

    pub fn flush(&mut self) -> io::Result<()> {
        self.w.flush()
    }

    pub fn into_inner(self) -> W {
        self.w
    }
}

pub struct Reader<R: BufRead> {
    r: R,
}

impl<R: BufRead> Reader<R> {
    pub fn new(mut r: R) -> io::Result<Self> {
        let mut m = [0u8; MAGIC.len()];
        r.read_exact(&mut m)?;
        if m != MAGIC {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "not a palcap 1 file",
            ));
        }
        Ok(Self { r })
    }

    /// None at a clean end of file; a truncated last record is an error.
    pub fn next_record(&mut self) -> io::Result<Option<Record>> {
        let mut tag = [0u8; 1];
        if self.r.read(&mut tag)? == 0 {
            return Ok(None);
        }
        match tag[0] {
            TAG_TICK => {
                let mut t = [0u8; 8];
                self.r.read_exact(&mut t)?;
                Ok(Some(Record::Tick(u64::from_le_bytes(t))))
            }
            TAG_BODY => {
                let mut h = [0u8; 28];
                self.r.read_exact(&mut h)?;
                let len = u32::from_le_bytes(h[24..28].try_into().unwrap()) as usize;
                let mut body = vec![0u8; len];
                self.r.read_exact(&mut body)?;
                Ok(Some(Record::Body(RxBody {
                    card_id: h[0],
                    mono_us: u64::from_le_bytes(h[1..9].try_into().unwrap()),
                    rssi: [h[9], h[10]],
                    snr: [h[11] as i8, h[12] as i8],
                    evm: [h[13] as i8, h[14] as i8],
                    phy_valid: h[15] & 1 != 0,
                    crc_ok: h[15] & 2 != 0,
                    mac_seq: u16::from_le_bytes([h[16], h[17]]),
                    tsfl: u32::from_le_bytes(h[18..22].try_into().unwrap()),
                    mcs: h[22],
                    rx_channel: h[23],
                    body,
                })))
            }
            t => Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("bad record tag {t}"),
            )),
        }
    }
}

impl<R: BufRead> Iterator for Reader<R> {
    type Item = io::Result<Record>;
    fn next(&mut self) -> Option<Self::Item> {
        self.next_record().transpose()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip() {
        let b = RxBody {
            card_id: 2,
            mono_us: 0x0102_0304_0506_0708,
            rssi: [80, 81],
            snr: [-3, 27],
            evm: [-40, 0],
            phy_valid: false,
            crc_ok: true,
            mac_seq: 0xFFF,
            tsfl: 0xDEAD_BEEF,
            mcs: 255,
            rx_channel: 144,
            body: vec![1, 2, 3, 0, 255],
        };
        let mut w = Writer::new(Vec::new()).unwrap();
        w.tick(5).unwrap();
        w.body(&b).unwrap();
        w.body(&RxBody::default()).unwrap();
        w.tick(u64::MAX).unwrap();
        let bytes = w.into_inner();
        let recs: Vec<_> = Reader::new(&bytes[..])
            .unwrap()
            .map(|r| r.unwrap())
            .collect();
        assert_eq!(recs.len(), 4);
        assert!(matches!(recs[0], Record::Tick(5)));
        let Record::Body(ref got) = recs[1] else {
            panic!()
        };
        assert_eq!(format!("{got:?}"), format!("{b:?}"));
        let Record::Body(ref e) = recs[2] else {
            panic!()
        };
        assert!(e.body.is_empty() && !e.crc_ok);
        assert!(matches!(recs[3], Record::Tick(u64::MAX)));
    }

    #[test]
    fn truncated_is_error() {
        let mut w = Writer::new(Vec::new()).unwrap();
        w.body(&RxBody {
            body: vec![9; 10],
            ..Default::default()
        })
        .unwrap();
        let bytes = w.into_inner();
        let mut r = Reader::new(&bytes[..bytes.len() - 1]).unwrap();
        assert!(r.next_record().is_err());
        assert!(Reader::new(&b"palcap 2\n"[..]).is_err());
    }
}
