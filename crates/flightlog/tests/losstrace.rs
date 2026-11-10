//! tools/losstrace.py against a synthetic capture this crate writes, so the
//! script and the capture format cannot drift apart unnoticed. Skipped (not
//! failed) on a host without python3.

use flightlog::capture::Writer;
use pal_core::RxBody;
use std::path::Path;
use std::process::Command;

fn body(card: u8, mono_us: u64, seq: u16) -> RxBody {
    RxBody {
        card_id: card,
        mono_us,
        // -30 dBm on both chains.
        rssi: [80, 80],
        snr: [30, 30],
        evm: [0, 0],
        phy_valid: true,
        crc_ok: true,
        mac_seq: seq,
        tsfl: 0,
        mcs: 3,
        rx_channel: 144,
        body: vec![0u8; 16],
    }
}

#[test]
fn losstrace_counts_the_union_loss_across_a_wrap_and_a_silence() {
    if Command::new("python3").arg("--version").output().is_err() {
        eprintln!("python3 not found: skipped");
        return;
    }
    let dir = std::env::temp_dir().join(format!("pal-losstrace-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).expect("dir");
    let cap = dir.join("bodies.bin");
    let mut w = Writer::new(std::fs::File::create(&cap).expect("create")).expect("writer");
    let mut t = 1_000_000u64;
    // Segment 1: 296 frames across the 12-bit wrap (4000..4095, 0..199),
    // 1 ms apart. Card 0 misses seq 10, card 1 misses 10 and 20..=22: the
    // union lost one frame.
    let seg1 = (4000u16..4096).chain(0..200);
    for s in seg1 {
        if s != 10 {
            w.body(&body(0, t, s)).expect("body");
        }
        if s != 10 && !(20..=22).contains(&s) {
            let mut b = body(1, t + 100, s);
            // A frame with no valid PHY status: received, no RSSI.
            if s == 30 {
                b.phy_valid = false;
                b.rssi = [0, 0];
            }
            w.body(&b).expect("body");
        }
        t += 1000;
    }
    // A weak foreign frame with a failed FCS: counted apart, not as loss.
    let mut junk = body(1, t, 1234);
    junk.crc_ok = false;
    junk.rssi = [15, 15];
    junk.mcs = 255;
    w.body(&junk).expect("body");
    // One second of silence, then the drone restarted elsewhere in its
    // count: 100 frames the unwrap cannot relate to the first segment.
    t += 1_000_000;
    for s in 3000u16..3100 {
        w.body(&body(0, t, s)).expect("body");
        w.body(&body(1, t + 100, s)).expect("body");
        t += 1000;
    }
    w.flush().expect("flush");
    drop(w);

    let script = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tools/losstrace.py");
    let json = dir.join("trace.json");
    let out = Command::new("python3")
        .arg(&script)
        .arg(&cap)
        .arg("--json")
        .arg(&json)
        .output()
        .expect("run losstrace.py");
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let v: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&json).expect("json")).expect("parse");
    assert_eq!(v["outages"], 1, "{v}");
    // The silence hides how many frames the drone sent: no count, and the
    // time it lasted.
    let o = &v["outage_list"][0];
    assert!(o[1].is_null(), "{o}");
    assert!(o[2].as_i64().expect("dt") >= 1_000_000, "{o}");
    assert_eq!(v["frames_sent"], 396, "{v}");
    let union = v["loss_pct_union"].as_f64().expect("union");
    assert!((union - 100.0 / 396.0).abs() < 1e-9, "{union}");
    assert_eq!(v["fcs_failed"]["1"], 1);
    // Only the drone's -30 dBm: the phy-invalid frame adds no -110 bucket.
    for card in ["0", "1"] {
        let keys: Vec<&String> = v["loss_pct_by_rssi_bucket_card"][card]
            .as_object()
            .expect("buckets")
            .keys()
            .collect();
        assert_eq!(keys, ["-30"], "card {card}");
    }
    let _ = std::fs::remove_dir_all(&dir);
}
