//! mabur's golden vectors, read in place from the pinned submodule.
use pal_wire::{crc16, profile};
use serde_json::Value;
use std::path::PathBuf;

fn vectors(name: &str) -> Value {
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../third_party/mabur/tests/vectors")
        .join(format!("{name}.json"));
    let text = std::fs::read_to_string(&p)
        .unwrap_or_else(|e| panic!("{}: {e} (git submodule update --init?)", p.display()));
    serde_json::from_str(&text).unwrap()
}

fn unhex(s: &str) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
        .collect()
}

#[test]
fn crc16_matches_vectors() {
    let v = vectors("crc16");
    let cases = v["cases"].as_array().unwrap();
    assert!(!cases.is_empty());
    for c in cases {
        let input = unhex(c["in"].as_str().unwrap());
        assert_eq!(
            u64::from(crc16::crc16_ccitt(&input)),
            c["crc"].as_u64().unwrap(),
            "in={}",
            c["in"]
        );
    }
}

#[test]
fn profile_matches_vectors() {
    let v = vectors("profile");
    let rows = v["profiles"].as_array().unwrap();
    assert!(!rows.is_empty());
    for r in rows {
        let p = profile::Profile {
            mode: match r["mode"].as_str().unwrap() {
                "ht" => profile::PhyMode::Ht,
                "vht" => profile::PhyMode::Vht,
                m => panic!("mode {m}"),
            },
            mcs: r["mcs"].as_u64().unwrap() as u8,
            bw_mhz: r["bw"].as_u64().unwrap() as u8,
        };
        let byte = r["byte"].as_u64().unwrap() as u8;
        assert_eq!(profile::encode(p), byte, "{r}");
        assert_eq!(profile::decode(byte), p, "{r}");
    }
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

// Goldens copied from mabur tests/test_rc.cpp (pinned to the same fields as
// tests/vectors/rc.json).
const RCF_GOLDEN: [&str; 3] = [
    "43521201000700243232ff000000000000d8922d902b70677e29a3",
    "4352120100ffff006464ff00000300c800e0390d6a99b162aa7e89",
    "43521201002a00086432060000025adbff124b63f32743e99651e5",
];

#[test]
fn rc_rcf_disc_ack_match_goldens() {
    use pal_wire::rc::*;
    let v = vectors("rc");
    let rcf = v["rcf"].as_array().unwrap();
    assert_eq!(rcf.len(), RCF_GOLDEN.len());
    for (c, g) in rcf.iter().zip(RCF_GOLDEN) {
        let f = &c["fields"];
        let u = |k: &str| f[k].as_i64().unwrap();
        let r = Rcf {
            seq: u("seq") as u16,
            profile: u("profile") as u8,
            fec_overhead_base: f["fec_overhead_base"].as_f64().unwrap(),
            fec_overhead_enh: f["fec_overhead_enh"].as_f64().unwrap(),
            probe_profile: f
                .get("probe_profile")
                .map_or(NO_PROBE_PROFILE, |x| x.as_u64().unwrap() as u8),
            hop_ch: u("hop_ch") as u8,
            hop_epoch: u("hop_epoch") as u8,
            rec: u("rec") as u8,
            idr_epoch: u("idr_epoch") as u8,
            snr_off: u("snr_off") as i16,
        };
        let w = pack_rcf(&r, &DEFAULT_LINK_KEY, &TagCtx::default());
        assert_eq!(hex(&w), g);
        assert_eq!(parse_rcf(&unhex(g)), Some(r));
        assert_eq!(frame_type(&w), Some(T_RCF));
        assert!(verify_control(&w, &DEFAULT_LINK_KEY, &TagCtx::default()));
    }
    let d = &v["disc"][0]["fields"];
    let disc = Disc {
        vrx_nonce: d["vrx_nonce"].as_u64().unwrap() as u32,
        op_channel: d["op_channel"].as_u64().unwrap() as u8,
        op_width: d["op_width"].as_u64().unwrap() as u8,
        table_ver: d["table_ver"].as_u64().unwrap() as u8,
        init_profile: d["init_profile"].as_u64().unwrap() as u8,
        cap_bits: d["cap_bits"].as_u64().unwrap() as u16,
        seq: d["seq"].as_u64().unwrap() as u16,
    };
    let w = pack_disc(&disc, &DEFAULT_LINK_KEY);
    assert_eq!(
        hex(&w),
        "43521202040100feca95140100000002005c1f5bb3bda794272ab8"
    );
    assert_eq!(parse_disc(&w), Some(disc));
    let a = &v["disc_ack"][0]["fields"];
    let ack = DiscAck {
        vrx_nonce: a["vrx_nonce"].as_u64().unwrap() as u32,
        vtx_nonce: a["vtx_nonce"].as_u64().unwrap() as u32,
        chip_caps: a["chip_caps"].as_u64().unwrap() as u16,
        agreed_channel: a["agreed_channel"].as_u64().unwrap() as u8,
        agreed_width: a["agreed_width"].as_u64().unwrap() as u8,
        flags: a["flags"].as_u64().unwrap() as u8,
        seq: a["seq"].as_u64().unwrap() as u16,
    };
    let w = pack_disc_ack(&ack);
    assert_eq!(hex(&w), "43521203040100feca0200efbe03009514000100b80a");
    assert_eq!(parse_disc_ack(&w), Some(ack));
}

#[test]
fn rc_telem_golden() {
    use pal_wire::rc::*;
    let g = "43521204090201022d0034127766554433221100a0860100282307000000d2040200\
             333415163d48000d000e000f00160000000000000305";
    let t = parse_telem(&unhex(g)).expect("golden parses");
    assert_eq!((t.flags, t.rcf_age_ms, t.rcf_seq_echo), (0x09, 45, 0x1234));
    assert_eq!(t.pts_at_build, 0x0011_2233_4455_6677);
    assert_eq!(
        (t.cmd_kbps, t.txq_drops, t.txq_wait_max_ms, t.usb_fail),
        (9000, 7, 1234, 2)
    );
    assert_eq!(
        (t.up_rssi, t.up_snr, t.soc_temp_c, t.cpu_busy_x100),
        ([51, 52], [21, 22], 61, 72)
    );
    assert_eq!(
        (t.rx_own, t.rx_foreign, t.rx_crcfail, t.rec_status),
        (13, 14, 15, 0x16)
    );
    assert_eq!(hex(&pack_telem(&t)), g);
    let mut flip = unhex(g);
    flip[20] ^= 0xFF;
    assert!(parse_telem(&flip).is_none());

    // The same Telem from a drone one RC version up or down is that version, and nothing else
    // is: this version's Telem, another version's RCF, a short body.
    assert_eq!(foreign_rc_version(&unhex(g)), None);
    for v in [RC_VERSION - 1, RC_VERSION + 1] {
        let mut other = unhex(g);
        other[2] = v;
        assert_eq!(foreign_rc_version(&other), Some(v));
        assert!(parse_telem(&other).is_none());
        other[3] = T_RCF;
        assert_eq!(foreign_rc_version(&other), None);
        assert_eq!(foreign_rc_version(&other[..3]), None);
    }
}

#[test]
fn rc_nack_mav_roundtrip() {
    use pal_wire::rc::*;
    let mut n = Nack {
        counter: 9,
        sid: 0,
        flags: NACK_FLAG_REPEAT,
        n: 2,
        ..Default::default()
    };
    n.e[0] = NackEntry {
        first_seq: 100,
        bitmap: 0b1011,
    };
    n.e[1] = NackEntry {
        first_seq: 200,
        bitmap: 0,
    };
    let ctx = TagCtx {
        vrx_nonce: 1,
        vtx_nonce: 2,
        seq32: 9,
    };
    let w = pack_nack(&n, &DEFAULT_LINK_KEY, &ctx);
    let p = parse_nack(&w).unwrap();
    assert_eq!((p.counter, p.n, p.e[1].bitmap), (9, 2, 1));
    assert!(verify_control(&w, &DEFAULT_LINK_KEY, &ctx));
    assert!(!verify_control(&w, &DEFAULT_LINK_KEY, &TagCtx::default()));
    let m = Mav {
        counter: 5,
        payload: vec![0xFD, 1, 2, 3],
    };
    for tagged in [false, true] {
        let w = pack_mav(&m, tagged, &DEFAULT_LINK_KEY, &ctx);
        assert_eq!(parse_mav(&w, tagged), Some(m.clone()));
        assert!(parse_mav(&w, !tagged).is_none());
    }
}
