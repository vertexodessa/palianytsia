//! Property tests of the `palcap 1` reader: arbitrary and bent files never
//! panic and always end, a written capture reads back field for field, and
//! a capture cut anywhere reads every whole record and then reports a torn
//! tail, never a damaged file.
//!
//! Run in the test profile, where overflow checks are on. Deterministic
//! (fixed seed, no failure files); `PAL_PROP_SCALE=N` multiplies the case
//! counts and `PAL_PROP_SEED=S` picks another seed for a long local run.
use flightlog::capture::{is_torn_tail, Reader, Record, Writer, MAGIC};
use pal_core::RxBody;
use proptest::prelude::*;
use proptest::test_runner::{Config, RngAlgorithm, RngSeed};

fn cfg(cases: u32) -> Config {
    let env = |k: &str| std::env::var(k).ok().and_then(|v| v.parse::<u64>().ok());
    let scale = env("PAL_PROP_SCALE").unwrap_or(1).max(1);
    Config {
        cases: (u64::from(cases) * scale).min(u64::from(u32::MAX)) as u32,
        failure_persistence: None,
        rng_algorithm: RngAlgorithm::ChaCha,
        rng_seed: RngSeed::Fixed(env("PAL_PROP_SEED").unwrap_or(0x9a1)),
        ..Config::default()
    }
}

fn body() -> impl Strategy<Value = RxBody> {
    (
        any::<(u8, u64, [u8; 2], [i8; 2], [i8; 2])>(),
        any::<(bool, bool, u16, u32, u8, u8)>(),
        proptest::collection::vec(any::<u8>(), 0..300),
    )
        .prop_map(
            |(
                (card_id, mono_us, rssi, snr, evm),
                (phy_valid, crc_ok, mac_seq, tsfl, mcs, ch),
                b,
            )| {
                RxBody {
                    card_id,
                    mono_us,
                    rssi,
                    snr,
                    evm,
                    phy_valid,
                    crc_ok,
                    mac_seq,
                    tsfl,
                    mcs,
                    rx_channel: ch,
                    body: b,
                }
            },
        )
}

#[derive(Clone, Debug)]
enum Rec {
    Body(RxBody),
    Tick(u64),
}

fn recs() -> impl Strategy<Value = Vec<Rec>> {
    proptest::collection::vec(
        prop_oneof![
            3 => body().prop_map(Rec::Body),
            1 => any::<u64>().prop_map(Rec::Tick),
        ],
        0..12,
    )
}

/// The file, and the end offset of each record.
fn write(rs: &[Rec]) -> (Vec<u8>, Vec<usize>) {
    let mut w = Writer::new(Vec::new()).unwrap();
    let mut ends = Vec::new();
    for r in rs {
        match r {
            Rec::Body(b) => w.body(b).unwrap(),
            Rec::Tick(t) => w.tick(*t).unwrap(),
        }
        ends.push(w.written() as usize);
    }
    let bytes = w.into_inner();
    assert_eq!(bytes.len(), ends.last().copied().unwrap_or(MAGIC.len()));
    (bytes, ends)
}

fn same(got: &Record, want: &Rec) -> bool {
    match (got, want) {
        (Record::Body(a), Rec::Body(b)) => format!("{a:?}") == format!("{b:?}"),
        (Record::Tick(a), Rec::Tick(b)) => a == b,
        _ => false,
    }
}

/// Every item the reader yields, bounded: a reader that stops consuming
/// would spin here instead of ending.
fn drain(bytes: &[u8]) -> Option<Vec<std::io::Result<Record>>> {
    let r = Reader::new(bytes).ok()?;
    let mut out = Vec::new();
    for item in r {
        out.push(item);
        assert!(out.len() <= bytes.len() + 1, "reader does not advance");
    }
    Some(out)
}

proptest! {
    #![proptest_config(cfg(512))]

    #[test]
    fn any_bytes_end_without_panic(tail in proptest::collection::vec(any::<u8>(), 0..600), magic: bool) {
        let mut b = if magic { MAGIC.to_vec() } else { Vec::new() };
        b.extend_from_slice(&tail);
        let _ = drain(&b);
    }

    #[test]
    fn written_reads_back(rs in recs()) {
        let (bytes, _) = write(&rs);
        let got = drain(&bytes).unwrap();
        prop_assert_eq!(got.len(), rs.len());
        for (g, w) in got.iter().zip(&rs) {
            prop_assert!(same(g.as_ref().unwrap(), w));
        }
    }

    #[test]
    fn cut_anywhere_is_a_torn_tail(rs in recs(), cut: usize) {
        let (bytes, ends) = write(&rs);
        let cut = MAGIC.len() + cut % (bytes.len() - MAGIC.len() + 1);
        let got = drain(&bytes[..cut]).unwrap();
        let whole = ends.iter().filter(|&&e| e <= cut).count();
        prop_assert!(got.len() >= whole);
        for (g, w) in got.iter().zip(&rs).take(whole) {
            prop_assert!(same(g.as_ref().unwrap(), w));
        }
        if let Some(Err(e)) = got.get(whole) {
            prop_assert!(is_torn_tail(e), "{}", e);
        }
    }

    #[test]
    fn bent_files_end_without_panic(
        rs in recs(),
        edits in proptest::collection::vec((any::<usize>(), any::<u8>()), 1..8),
    ) {
        let (mut bytes, _) = write(&rs);
        let n = bytes.len();
        for (p, v) in edits {
            bytes[p % n] ^= v;
        }
        let _ = drain(&bytes);
    }
}
