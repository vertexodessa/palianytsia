//! mabur's golden vectors, read in place from the pinned submodule, and the
//! MAVLink goldens in `tests/vectors/` (made by pymavlink, see mavlink_gen.py).
use pal_wire::{crc16, mavlink, profile, sbi};
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

fn local_vectors(name: &str) -> Value {
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/vectors")
        .join(format!("{name}.json"));
    let text = std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("{}: {e}", p.display()));
    serde_json::from_str(&text).unwrap()
}

/// Every pymavlink frame parses once, as Ok, on its last byte, with its
/// header fields; an unsigned MAVLink 2 frame packs back to the same bytes;
/// the FC HEARTBEATs give their arm state.
#[test]
fn mavlink_matches_pymavlink_goldens() {
    let v = local_vectors("mavlink");
    let frames = v["frames"].as_array().unwrap();
    assert_eq!(frames.len(), 8);
    let (mut v1, mut signed, mut repacked) = (0, 0, 0);
    for f in frames {
        let name = f["name"].as_str().unwrap();
        let w = unhex(f["hex"].as_str().unwrap());
        let mut p = mavlink::Parser::default();
        let r: Vec<mavlink::Framing> = w.iter().map(|&b| p.feed(b)).collect();
        let oks: Vec<usize> = (0..r.len())
            .filter(|&i| r[i] == mavlink::Framing::Ok)
            .collect();
        assert_eq!(
            oks,
            vec![w.len() - 1],
            "{name}: Ok exactly on the last byte"
        );
        assert!(p.is_idle(), "{name}");
        let m = p.message();
        assert_eq!(u64::from(m.msgid), f["msgid"].as_u64().unwrap(), "{name}");
        assert_eq!(u64::from(m.sysid), f["sysid"].as_u64().unwrap(), "{name}");
        assert_eq!(u64::from(m.compid), f["compid"].as_u64().unwrap(), "{name}");
        assert_eq!(m.v1, f["v1"].as_bool().unwrap(), "{name}");
        v1 += usize::from(m.v1);
        signed += usize::from(f["signed"].as_bool().unwrap());
        if f["repack"].as_bool().unwrap() {
            let (_, _, max) = mavlink::entry(m.msgid).unwrap();
            let again = mavlink::pack_v2(
                m.msgid,
                m.sysid,
                m.compid,
                m.seq,
                &m.payload[..usize::from(max)],
            );
            assert_eq!(
                hex(&again),
                hex(&w),
                "{name}: pack_v2 of the parsed payload"
            );
            repacked += 1;
        }
        let armed = mavlink::fc_heartbeat_armed(&w);
        let want = match name {
            "heartbeat_fc_armed" | "heartbeat_v1_armed" | "heartbeat_signed_armed" => Some(true),
            "heartbeat_fc_disarmed" => Some(false),
            _ => None,
        };
        assert_eq!(armed, want, "{name}");
    }
    assert_eq!((v1, signed, repacked), (1, 1, 6));
}

/// A corrupted signature byte does not fail the frame (no signing context:
/// the check passes, as mavlink_signature_check); a corrupted CRC byte of the
/// signed frame does.
#[test]
fn mavlink_signed_golden_signature_unchecked_crc_checked() {
    let v = local_vectors("mavlink");
    let f = v["frames"]
        .as_array()
        .unwrap()
        .iter()
        .find(|f| f["signed"].as_bool().unwrap())
        .unwrap();
    let w = unhex(f["hex"].as_str().unwrap());
    let last = |w: &[u8]| {
        let mut p = mavlink::Parser::default();
        w.iter().map(|&b| p.feed(b)).last().unwrap()
    };
    let mut sig = w.clone();
    *sig.last_mut().unwrap() ^= 0xFF;
    assert_eq!(last(&sig), mavlink::Framing::Ok);
    let mut crc = w.clone();
    let n = crc.len();
    crc[n - 13 - 1] ^= 0xFF;
    assert_eq!(last(&crc), mavlink::Framing::BadCrc);
}

fn sbi_case_survivors(c: &Value) -> Vec<String> {
    c["survivors"]
        .as_array()
        .unwrap()
        .iter()
        .map(|s| s.as_str().unwrap().to_string())
        .collect()
}

/// mabur's sbi_unpack.json: survivors, failures and header verdict per body.
#[test]
fn sbi_unpack_matches_vectors() {
    let v = vectors("sbi_unpack");
    let cases = v["cases"].as_array().unwrap();
    assert_eq!(cases.len(), 10);
    for c in cases {
        let name = c["name"].as_str().unwrap();
        let body = unhex(c["body"].as_str().unwrap());
        let bp = c["block_payload"].as_u64().unwrap() as usize;
        let u = sbi::unpack(&body, bp);
        assert_eq!(u.header_ok, c["header_ok"].as_bool().unwrap(), "{name}");
        assert_eq!(u.n_failed as u64, c["n_failed"].as_u64().unwrap(), "{name}");
        let got: Vec<String> = u.survivors.iter().map(|s| hex(s)).collect();
        assert_eq!(got, sbi_case_survivors(c), "{name}");
        if u.header_ok {
            let sid = c["stream_id"].as_u64().unwrap() as u8;
            assert_eq!(sbi::peek_stream_id(&body), Some(sid), "{name}");
        }
    }
}

/// mabur's sbi.json: the Rust Packer makes the same bodies from the same
/// envelopes (full bodies, then the flushed remainder).
#[test]
fn sbi_pack_matches_vectors() {
    let v = vectors("sbi");
    let bp = v["block_payload"].as_u64().unwrap() as usize;
    let per = v["blocks_per_body"].as_u64().unwrap() as usize;
    let sid = v["stream_id"].as_u64().unwrap() as u8;
    let mut p = sbi::Packer::new(bp, per, sid);
    let mut stream = Vec::new();
    for e in v["envelopes"].as_array().unwrap() {
        if let Some(b) = p.add(&unhex(e.as_str().unwrap())) {
            stream.push(hex(&b));
        }
    }
    let want: Vec<String> = v["stream"]
        .as_array()
        .unwrap()
        .iter()
        .map(|s| s.as_str().unwrap().to_string())
        .collect();
    assert_eq!(stream, want);
    let flush: Vec<String> = p.flush().iter().map(|b| hex(b)).collect();
    let want: Vec<String> = v["flush"]
        .as_array()
        .unwrap()
        .iter()
        .map(|s| s.as_str().unwrap().to_string())
        .collect();
    assert_eq!(flush, want);
}

/// The ver-2 header fields by their byte position in mabur's sbi.cpp
/// (magic, ver, stream id, block_payload, n_blocks, q_ms, enc_us, air_ms;
/// u16 little-endian), written by hand so swapped offsets in sbi.rs fail.
#[test]
fn sbi_header_timing_fields_by_position() {
    let env = [0x5Au8; 8];
    let crc = crc16::crc16_ccitt(&env).to_le_bytes();
    let mut body = vec![
        0xB0, 0xF5, // magic
        2,    // ver
        0x82, // stream 2, retransmit mark
        8, 0, // block_payload
        1, // n_blocks
        0x34, 0x12, // q_ms
        0x78, 0x56, // enc_us
        0xBC, 0x9A, // air_ms
    ];
    body.extend_from_slice(&crc);
    body.extend_from_slice(&env);
    let u = sbi::unpack(&body, 8);
    assert!(u.header_ok && u.retx);
    assert_eq!((u.q_ms, u.enc_us, u.air_ms), (0x1234, 0x5678, 0x9ABC));
    assert_eq!(u.survivors, vec![&env[..]]);
    assert_eq!(sbi::peek_stream_id(&body), Some(2));
}
