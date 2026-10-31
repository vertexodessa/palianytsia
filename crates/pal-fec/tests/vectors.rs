//! mabur `tests/vectors/sw.json` decode scenarios (same check as
//! `test_sw_decoder.cpp` vectors_decode_scenarios).
use pal_fec::{Boundary, SwConfig, SwDecoder};
use serde_json::Value;
use std::path::PathBuf;

fn unhex(s: &str) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
        .collect()
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

#[test]
fn sw_decode_scenarios() {
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../third_party/mabur/tests/vectors/sw.json");
    let v: Value = serde_json::from_str(&std::fs::read_to_string(p).unwrap()).unwrap();
    let mut checked = 0;
    for c in v["cases"].as_array().unwrap() {
        let envs: Vec<Vec<u8>> = c["stream"]
            .as_array()
            .unwrap()
            .iter()
            .chain(c["flush"].as_array().unwrap())
            .map(|s| unhex(s.as_str().unwrap()))
            .collect();
        let cfg = SwConfig {
            symbol_size: c["symbol_size"].as_u64().unwrap() as usize,
            window: c["window"].as_u64().unwrap() as usize,
            overhead: c["overhead"].as_f64().unwrap(),
        };
        for d in c["decode"].as_array().unwrap() {
            let drop: Vec<usize> = d["drop"]
                .as_array()
                .unwrap()
                .iter()
                .map(|x| x.as_u64().unwrap() as usize)
                .collect();
            let mut dec = SwDecoder::new(cfg, 0, 0);
            let mut got = Vec::new();
            for (i, e) in envs.iter().enumerate() {
                if drop.contains(&i) {
                    continue;
                }
                for p in dec.add_symbol(e, 1000, Boundary::None, true, false) {
                    got.push(hex(&p));
                }
            }
            got.sort();
            let want: Vec<String> = d["recovered_sorted"]
                .as_array()
                .unwrap()
                .iter()
                .map(|s| s.as_str().unwrap().to_string())
                .collect();
            assert_eq!(got, want, "drop={drop:?}");
            checked += 1;
        }
    }
    assert!(checked > 3);
}
