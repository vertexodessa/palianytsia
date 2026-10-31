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
