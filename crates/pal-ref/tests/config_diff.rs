//! pal-core's config parser against mabur's `maburgs::load_config`: the
//! same TOML text goes to both, and the verdict (loaded or refused), the
//! error text and, when both load it, every field of the loaded config
//! (`config::dump` and `pal_ref_config_check`, doubles by their bits) must
//! be equal.
//!
//! Cases: the bundled configs and an empty file, then for every key of the
//! Rust parser's own key table (`config::KEY_TABLE`): the key deleted, at
//! and one past each end of its range (for a float also the next
//! representable value past each end), and of a wrong type; per section an
//! unknown key, the section deleted and the section as a scalar; then the
//! special keys (channel set, pin, relays, symbol size, ladder, stats
//! destinations, pairing key) and the outputs of the link-pick helpers.
//!
//! Coverage of the key set: the C++ side reports every key it left at its
//! default (`load_config`'s `defaulted` list); two documents with every
//! table present but empty make it name every key it knows. That set must
//! equal the Rust key table (less the Rust-only keys below), so a key a pin
//! adds to maburgs fails here until the Rust parser and the range table
//! below know it. Coverage of the fields: both dumps destructure every
//! struct whole (a C++ structured binding, a Rust pattern without `..`), so
//! a new field stops the build until it is dumped, and a dump line one side
//! has and the other lacks fails here unless it is listed in ONE_SIDED.
//!
//! Every difference that is not a bug is listed in ALLOWED with its reason;
//! an unlisted difference fails, and so does a listed one that no longer
//! differs.

use std::collections::{BTreeMap, BTreeSet};
use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_int};
use std::time::Instant;

use pal_core::config::{self, LinkPick, KEY_TABLE};
use pal_ref as _;
use toml::{Table, Value};

#[repr(C)]
struct CheckOut {
    ok: c_int,
    text: *mut c_char,
    defaulted: *mut c_char,
}

extern "C" {
    fn pal_ref_config_check(toml_text: *const c_char, out: *mut CheckOut);
    fn pal_ref_config_check_free(out: *mut CheckOut);
}

const REPO: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../..");

/// Keys the Rust parser accepts and maburgs does not.
const RUST_ONLY_KEYS: &[(&str, &str)] = &[(
    "radio.use",
    "pal: the members the link may sit on (the operator's channel pick); maburgs has no such key",
)];

/// Dump lines only one side prints.
const ONE_SIDED: &[(&str, &str)] = &[
    (
        "radio.use",
        "pal-only field behind the pal-only key radio.use",
    ),
    (
        "derived.uep_layers[0].fec.overhead",
        "C++ only: decode-inert placeholder 0.5 (SwDecoder never reads it); the Rust decode layer has no such field",
    ),
    (
        "derived.uep_layers[1].fec.overhead",
        "C++ only: as layer 0",
    ),
    (
        "derived.uep_layers[0].blocks_per_body",
        "C++ only: constant 4, unused on decode; the Rust decode layer has no such field",
    ),
    (
        "derived.uep_layers[1].blocks_per_body",
        "C++ only: as layer 0",
    ),
];

/// Keys load_config never reports as defaulted, with why: they cannot show
/// up in the defaulted list, so the key-set check takes them from here.
const NOT_REPORTED: &[(&str, &str)] = &[
    (
        "link.key",
        "read by presence only: an optional overlay key, not a tunable",
    ),
    (
        "link.ladder[].bw",
        "required on every rung: absent is an error, never a default",
    ),
    (
        "stats.out[].port",
        "required on every destination: absent is an error, never a default",
    ),
    (
        "stats.out",
        "absent = the legacy stats.host/stats.port pair, which is reported instead",
    ),
];

const SCALAR_SECTION: &str =
    "maburgs reads a scalar where a table is due as an empty table (toml.cpp: \
     contains() on a non-table is false, check_keys finds no keys) and loads the defaults; \
     pal refuses it as \"not a table\". Kept: the stricter verdict is the safe side";
const PAL_GRID: &str = "pal: radio.channels may list up to the whole 5 GHz grid (25, maburgs 8) \
     and, at 40 MHz, members on both sides of the pair grid, so the GS can search every \
     channel the drone may have been moved to (mabur fork 44069eb4)";

/// Cases whose verdicts or texts differ on purpose. Each id names one case.
const ALLOWED: &[(&str, &str)] = &[
    ("radio = 5", SCALAR_SECTION),
    ("radio.scan = 5", SCALAR_SECTION),
    ("radio.cards = [5]", SCALAR_SECTION),
    ("hop = 5", SCALAR_SECTION),
    ("hop.verdict = 5", SCALAR_SECTION),
    ("fec = 5", SCALAR_SECTION),
    ("link = 5", SCALAR_SECTION),
    ("link.ladder = [5]", SCALAR_SECTION),
    ("link.probe = 5", SCALAR_SECTION),
    ("link.fade = 5", SCALAR_SECTION),
    ("link.nack = 5", SCALAR_SECTION),
    ("link.rung_stats = 5", SCALAR_SECTION),
    ("video = 5", SCALAR_SECTION),
    ("msp = 5", SCALAR_SECTION),
    ("msp.out = 5", SCALAR_SECTION),
    ("stats = 5", SCALAR_SECTION),
    ("au_ring = 5", SCALAR_SECTION),
    ("debug_log = 5", SCALAR_SECTION),
    ("mavlink = 5", SCALAR_SECTION),
    (
        "radio.channels = []",
        "pal: the set-size message names pal's limit (1 to 25, maburgs 1 to 8); same key, same refusal",
    ),
    ("radio.channels = 9 members", PAL_GRID),
    ("radio.channels = both sides of the pair grid", PAL_GRID),
    ("radio.use = [64]", "pal-only key radio.use"),
    ("radio.use = [36]", "pal-only key radio.use"),
    ("radio.use = \"x\"", "pal-only key radio.use"),
    ("link pick: 20 MHz, all channels", PAL_GRID),
    ("link pick: 20 MHz, [161]", PAL_GRID),
    ("link pick: 40 MHz, [36, 40, 165]", PAL_GRID),
    ("with_channels [64, 112]", "pal-only key radio.use"),
    ("with_search_grid 40 MHz", PAL_GRID),
];

#[derive(Clone, Debug, PartialEq)]
enum Out {
    Loaded(BTreeMap<String, String>),
    Refused(String),
}

fn lines(text: &str) -> BTreeMap<String, String> {
    let mut m = BTreeMap::new();
    for l in text.lines() {
        let (k, v) = l.split_once('=').unwrap_or((l, ""));
        assert!(
            m.insert(k.to_string(), v.to_string()).is_none(),
            "dump line {k} twice"
        );
    }
    m
}

fn cpp(text: &str) -> (Out, Vec<String>) {
    let c = CString::new(text).unwrap();
    let mut o = CheckOut {
        ok: 0,
        text: std::ptr::null_mut(),
        defaulted: std::ptr::null_mut(),
    };
    // SAFETY: c is NUL-terminated; the shim fills o with malloc'd strings,
    // freed below after they are copied.
    unsafe {
        pal_ref_config_check(c.as_ptr(), &mut o);
        let t = CStr::from_ptr(o.text).to_string_lossy().into_owned();
        let d = CStr::from_ptr(o.defaulted).to_string_lossy().into_owned();
        let ok = o.ok != 0;
        pal_ref_config_check_free(&mut o);
        let out = if ok {
            Out::Loaded(lines(&t))
        } else {
            Out::Refused(t)
        };
        (out, d.lines().map(str::to_string).collect())
    }
}

fn rust(text: &str) -> Out {
    match config::parse_config_str(text) {
        Ok(c) => Out::Loaded(lines(&config::dump(&c))),
        Err(e) => Out::Refused(e.0),
    }
}

fn drop_one_sided(m: &mut BTreeMap<String, String>) {
    for (k, _) in ONE_SIDED {
        m.remove(*k);
    }
}

/// The difference between the two outcomes, None = equal.
fn compare(r: &Out, c: &Out) -> Option<String> {
    match (r, c) {
        (Out::Loaded(a), Out::Loaded(b)) => {
            let (mut a, mut b) = (a.clone(), b.clone());
            drop_one_sided(&mut a);
            drop_one_sided(&mut b);
            let keys: BTreeSet<&String> = a.keys().chain(b.keys()).collect();
            let mut d = String::new();
            for k in keys {
                let (x, y) = (a.get(k), b.get(k));
                if x != y {
                    d += &format!(
                        "\n    {k}: rust {} | c++ {}",
                        show(x.map(String::as_str)),
                        show(y.map(String::as_str))
                    );
                }
            }
            (!d.is_empty()).then(|| format!("both load, fields differ:{d}"))
        }
        (Out::Refused(a), Out::Refused(b)) => {
            (a != b).then(|| format!("both refuse:\n    rust {a}\n    c++  {b}"))
        }
        (Out::Loaded(_), Out::Refused(b)) => Some(format!("rust loads, c++ refuses: {b}")),
        (Out::Refused(a), Out::Loaded(_)) => Some(format!("rust refuses: {a}; c++ loads")),
    }
}

fn show(v: Option<&str>) -> String {
    match v {
        None => "(absent)".into(),
        Some(s) => match s.strip_prefix("f64:") {
            Some(h) => format!(
                "{} ({s})",
                f64::from_bits(u64::from_str_radix(h, 16).unwrap())
            ),
            None => s.to_string(),
        },
    }
}

fn read(rel: &str) -> String {
    std::fs::read_to_string(format!("{REPO}/{rel}")).unwrap()
}

fn base() -> Table {
    read("configs/gs.toml").parse().unwrap()
}

/// How a key is probed: the ranges are mabur gs/src/config.cpp's, and the
/// test checks them against the C++ (a bound one past which it does not say
/// "out of range" fails), so a transcription slip cannot hide a probe.
#[derive(Clone, Copy, Debug)]
enum Kind {
    Int(i64, i64),
    Num(f64, f64),
    Bool,
    Str,
    /// Probed by hand-written cases (special_cases).
    Special,
}

fn kind(path: &str) -> Option<Kind> {
    use Kind::*;
    const MS: (i64, i64) = (0, 600000);
    Some(match path {
        "radio.channels" | "radio.channel" | "radio.use" | "radio.relays" | "radio.cards" => {
            Special
        }
        "radio.tx_card" => Int(-1, 15),
        "radio.cards[].usb_vid" | "radio.cards[].usb_pid" => Int(0, 0xFFFF),
        "radio.cards[].index" => Int(0, 15),
        "radio.scan.dwell_ms" => Int(50, 10000),
        "radio.scan.settle_ms" => Int(0, 1000),
        "radio.scan.min_rounds" => Int(1, 100),
        "radio.scan.search_ms" => Int(40, 2000),
        "radio.scan.op_window_ms" => Int(40, 10000),
        "radio.scan.search_after_ms" => Int(0, 600000),
        "radio.scan.pick_margin_pct" => Int(0, 100),
        "radio.scan.one_card_ms" => Int(0, 60000),
        "radio.scan.max_ms" => Int(1000, 600000),
        "radio.scan.busy_dbm" => Int(-104, -70),
        "radio.scan.blocked_pct" => Num(1.0, 100.0),
        "hop.window_ms" => Int(50, 2000),
        "hop.persist" => Int(1, 3),
        "hop.dwell_observe_ms" => Int(1, 250),
        "hop.dwell_period_ms" => Int(20, 60000),
        "hop.burst_period_ms" => Int(100, 60000),
        "hop.rank_visits" => Int(1, 100),
        "hop.rank_max_age_ms" => Int(1000, 600000),
        "hop.confirm_ms" => Int(100, 5000),
        "hop.confirm_extend_ms" => Int(0, 30000),
        "hop.verify_ms" => Int(200, 10000),
        "hop.cooldown_ms" => Int(0, 60000),
        "hop.max_hops_per_min" => Int(1, 60),
        "hop.backoff_ms" => Int(1000, 600000),
        "hop.one_card_repeats" => Int(1, 50),
        "hop.relay_burst_period_ms" => Int(500, 60000),
        "hop.verdict.loss_pct" => Num(0.1, 100.0),
        "hop.verdict.recovered_x" => Num(1.0, 100.0),
        "hop.verdict.recovered_min" => Int(0, 1000),
        "hop.verdict.weak_rssi_dbm" => Int(-110, -20),
        "hop.verdict.weak_snr_db" => Int(0, 40),
        "hop.verdict.occupied_windows" => Int(1, 100),
        "hop.verdict.detect_pct" => Num(1.0, 100.0),
        "hop.verdict.detect_windows" => Int(2, 100),
        "hop.verdict.detect_impaired" => Int(1, 100),
        "fec.symbol_size" => Special,
        "fec.seq_horizon" => Int(16, 65536),
        "link.feedback_ms" => Int(20, 5000),
        "link.beacon_keepalive_ms" => Int(100, 60000),
        "link.rcf_slot_hold_ms" => Int(0, 1000),
        "link.arrival_guard_syms" => Int(16, 512),
        "link.static_mcs" => Int(-1, 7),
        "link.static_overhead_base" | "link.static_overhead_enh" => Num(0.1, 2.0),
        "link.static_bw" => Int(20, 40),
        "link.ladder" | "link.key" => Special,
        "link.max_mcs" => Int(0, 7),
        "link.down_util" | "link.up_util" => Num(0.0, 1.0),
        "link.confirm_ms"
        | "link.clean_ms"
        | "link.probation_ms"
        | "link.penalty_base_ms"
        | "link.penalty_max_ms"
        | "link.hold_after_down_ms"
        | "link.min_between_changes_ms"
        | "link.feedback_timeout_ms"
        | "link.starved_confirm_ms" => Int(MS.0, MS.1),
        "link.s3_demote" => Bool,
        "link.s3_down_util" => Num(0.01, 2.0),
        "link.s3_min_syms" => Int(1, 100000),
        "link.s3_settle_ms" => Int(0, 5000),
        "link.key_file" => Str,
        "link.ladder[].mcs" => Int(0, 7),
        "link.ladder[].bw" => Int(20, 40),
        "link.ladder[].overhead_base" | "link.ladder[].overhead_enh" => Num(0.1, 2.0),
        "link.probe.enable" => Bool,
        "link.probe.rung_offset" => Int(1, 7),
        "link.probe.clean_bodies" => Int(10, 100000),
        "link.probe.max_util" => Num(0.01, 2.0),
        "link.probe.min_syms" => Int(4, 100000),
        "link.probe.silence_ms" => Int(100, 10000),
        "link.probe.pin_mcs" => Int(-1, 7),
        "link.fade.cascade" | "link.fade.predict" => Bool,
        "link.fade.hold_ms" => Int(0, 60000),
        "link.fade.confirm_ms" => Int(20, 1000),
        "link.fade.rssi_db" | "link.fade.snr_db" => Num(0.5, 40.0),
        "link.fade.trigger_ms" => Int(50, 5000),
        "link.fade.min_rung" => Int(0, 15),
        "link.nack.enable" => Bool,
        "link.nack.lookback" => Int(8, 4096),
        "link.nack.repeat_ms" => Int(1, 1000),
        "link.nack.max_tries" => Int(0, 16),
        "link.nack.min_lead_ms" => Int(1, 100),
        "link.rung_stats.half_life_samples" => Int(10, 100000),
        "video.frame_gap_timeout_ms" => Int(10, 1000),
        "video.frame_gap_timeout_max_ms" => Int(0, 400),
        "video.frame_lookahead" => Int(2, 64),
        "msp.enable" => Bool,
        "msp.symbol_size" => Int(16, 2048),
        "msp.window" => Int(2, 255),
        "msp.out.host" => Str,
        "msp.out.port" => Int(1, 65535),
        "stats.enable" => Bool,
        "stats.interval_ms" => Int(100, 10000),
        "stats.host" => Str,
        "stats.port" => Int(1, 65535),
        "stats.out" => Special,
        "stats.out[].host" => Str,
        "stats.out[].port" => Int(1, 65535),
        "au_ring.enable" => Bool,
        "au_ring.path" | "au_ring.socket" => Str,
        "au_ring.slot_kb" => Int(64, 4096),
        "au_ring.slot_count" => Int(4, 256),
        "debug_log.enable" => Bool,
        "debug_log.dir" => Str,
        "debug_log.ctl_period_ms" => Int(50, 60000),
        "debug_log.rung_period_s" => Int(1, 600),
        "mavlink.enable" => Bool,
        "mavlink.bind" => Str,
        "mavlink.port" => Int(1, 65535),
        _ => return None,
    })
}

fn join(sec: &str, key: &str) -> String {
    if sec.is_empty() {
        key.to_string()
    } else {
        format!("{sec}.{key}")
    }
}

fn is_table_section(path: &str) -> bool {
    KEY_TABLE.iter().any(|(s, _)| *s == path)
}

/// The leaf keys of KEY_TABLE as (section, key): every accepted key that is
/// not itself a table section. An array of tables (`radio.cards`) is a leaf
/// here, and its element section (`radio.cards[]`) has leaves of its own.
fn leaves() -> Vec<(&'static str, &'static str)> {
    let mut out = Vec::new();
    for (sec, keys) in KEY_TABLE {
        for k in *keys {
            if !is_table_section(&join(sec, k)) {
                out.push((*sec, *k));
            }
        }
    }
    out
}

/// The section's table in `doc`, created when absent. An array-of-tables
/// section is its first element, created with a one-card list for cards.
fn table_at<'a>(doc: &'a mut Table, sec: &str) -> &'a mut Table {
    if sec.is_empty() {
        return doc;
    }
    if let Some(arr) = sec.strip_suffix("[]") {
        let (parent, key) = arr.rsplit_once('.').unwrap();
        let t = table_at(doc, parent);
        if !matches!(t.get(key), Some(Value::Array(a)) if !a.is_empty()) {
            let mut card = Table::new();
            card.insert("usb_vid".into(), Value::Integer(0x0bda));
            card.insert("usb_pid".into(), Value::Integer(0x881a));
            card.insert("index".into(), Value::Integer(0));
            t.insert(key.into(), Value::Array(vec![Value::Table(card)]));
        }
        let Some(Value::Array(a)) = t.get_mut(key) else {
            unreachable!()
        };
        return a[0].as_table_mut().unwrap();
    }
    let (parent, key) = match sec.rsplit_once('.') {
        Some((p, k)) => (p, k),
        None => ("", sec),
    };
    table_at(doc, parent)
        .entry(key)
        .or_insert_with(|| Value::Table(Table::new()))
        .as_table_mut()
        .unwrap()
}

/// The base prepared for probing `sec`: legacy stats.host/stats.port need
/// the destination list gone (the two forms refuse to mix).
fn base_for(sec: &str, key: &str) -> Table {
    let mut doc = base();
    if sec == "stats" && (key == "host" || key == "port") {
        table_at(&mut doc, "stats").remove("out");
    }
    doc
}

fn text(doc: &Table) -> String {
    toml::to_string(doc).unwrap()
}

struct Case {
    id: String,
    text: String,
}

fn set_case(sec: &str, key: &str, id_val: &str, v: Value) -> Case {
    let mut doc = base_for(sec, key);
    table_at(&mut doc, sec).insert(key.into(), v);
    Case {
        id: format!("{} = {id_val}", join(sec, key)),
        text: text(&doc),
    }
}

fn edit_case(id: &str, f: impl FnOnce(&mut Table)) -> Case {
    let mut doc = base();
    f(&mut doc);
    Case {
        id: id.to_string(),
        text: text(&doc),
    }
}

fn ints(v: &[i64]) -> Value {
    Value::Array(v.iter().map(|&x| Value::Integer(x)).collect())
}

fn strs(v: &[&str]) -> Value {
    Value::Array(v.iter().map(|&x| Value::String(x.into())).collect())
}

/// Key files for the key_file cases; removed when the test ends.
fn key_dir() -> std::path::PathBuf {
    std::env::temp_dir().join(format!("pal-config-diff-{}", std::process::id()))
}

fn special_cases() -> Vec<Case> {
    let mut v = Vec::new();
    // radio.channels: the pin (144) has to stay a member, or "auto".
    let chans = |id: &str, chs: Value, auto: bool| {
        edit_case(id, |d| {
            let r = table_at(d, "radio");
            r.insert("channels".into(), chs);
            if auto {
                r.insert("channel".into(), Value::String("auto".into()));
            }
        })
    };
    v.push(chans("radio.channels = []", ints(&[]), true));
    v.push(chans("radio.channels = [0]", ints(&[0]), true));
    v.push(chans("radio.channels = [178]", ints(&[178]), true));
    v.push(chans(
        "radio.channels = [1.5]",
        Value::Array(vec![Value::Float(1.5)]),
        true,
    ));
    v.push(chans("radio.channels = [\"x\"]", strs(&["x"]), true));
    v.push(chans("radio.channels = 40", Value::Integer(40), true));
    v.push(chans("radio.channels = [144]", ints(&[144]), false));
    v.push(chans("radio.channels = [40, 40]", ints(&[40, 40]), true));
    v.push(chans(
        "radio.channels = [40, 165] at 40 MHz",
        ints(&[40, 165]),
        true,
    ));
    v.push(chans(
        "radio.channels = 8 members",
        ints(&[36, 44, 52, 60, 100, 108, 116, 124]),
        true,
    ));
    v.push(chans(
        "radio.channels = 9 members",
        ints(&[36, 44, 52, 60, 100, 108, 116, 124, 132]),
        true,
    ));
    v.push(chans(
        "radio.channels = both sides of the pair grid",
        ints(&[40, 44]),
        true,
    ));
    v.push(chans(
        "radio.channels = [64, 112] keeps pin 144",
        ints(&[64, 112]),
        false,
    ));
    // radio.channel
    for (id, val) in [
        ("\"auto\"", Value::String("auto".into())),
        ("\"x\"", Value::String("x".into())),
        ("0", Value::Integer(0)),
        ("178", Value::Integer(178)),
        ("296", Value::Integer(296)),
        ("36", Value::Integer(36)),
        ("64", Value::Integer(64)),
        ("true", Value::Boolean(true)),
        ("64.0", Value::Float(64.0)),
    ] {
        v.push(set_case("radio", "channel", id, val));
    }
    // radio.use (pal only)
    v.push(set_case("radio", "use", "[64]", ints(&[64])));
    v.push(set_case("radio", "use", "[36]", ints(&[36])));
    v.push(set_case("radio", "use", "\"x\"", Value::String("x".into())));
    // radio.relays
    for (id, val) in [
        ("[\"10.0.0.1:8310\"]", strs(&["10.0.0.1:8310"])),
        (
            "[\"10.0.0.1:8310\", \"10.0.0.2:8310\"]",
            strs(&["10.0.0.1:8310", "10.0.0.2:8310"]),
        ),
        ("[\"host:1\"]", strs(&["host:1"])),
        ("[\"01.2.3.4:1\"]", strs(&["01.2.3.4:1"])),
        ("[\"1.2.3:1\"]", strs(&["1.2.3:1"])),
        ("[\"1.2.3.4\"]", strs(&["1.2.3.4"])),
        ("[\"1.2.3.4:\"]", strs(&["1.2.3.4:"])),
        ("[\":1\"]", strs(&[":1"])),
        ("[\"1.2.3.4:0\"]", strs(&["1.2.3.4:0"])),
        ("[\"1.2.3.4:65536\"]", strs(&["1.2.3.4:65536"])),
        ("[\"1.2.3.4:123456\"]", strs(&["1.2.3.4:123456"])),
        ("[\"1.2.3.4:x\"]", strs(&["1.2.3.4:x"])),
        ("[\"1.2.3.4:+1\"]", strs(&["1.2.3.4:+1"])),
        (
            "[\"1.2.3.4:1\", \"1.2.3.4:1\"]",
            strs(&["1.2.3.4:1", "1.2.3.4:1"]),
        ),
        ("[1]", ints(&[1])),
        ("\"x\"", Value::String("x".into())),
    ] {
        v.push(set_case("radio", "relays", id, val));
    }
    // radio.cards and tx_card against the explicit list.
    v.push(set_case("radio", "cards", "[]", Value::Array(vec![])));
    v.push(set_case("radio", "cards", "5", Value::Integer(5)));
    v.push(edit_case("radio.cards = one card, tx_card = 0", |d| {
        table_at(d, "radio.cards[]");
        table_at(d, "radio").insert("tx_card".into(), Value::Integer(0));
    }));
    v.push(edit_case("radio.cards = one card, tx_card = 1", |d| {
        table_at(d, "radio.cards[]");
        table_at(d, "radio").insert("tx_card".into(), Value::Integer(1));
    }));
    v.push(edit_case(
        "radio.cards = one card + one relay, tx_card = 1",
        |d| {
            table_at(d, "radio.cards[]");
            let r = table_at(d, "radio");
            r.insert("tx_card".into(), Value::Integer(1));
            r.insert("relays".into(), strs(&["10.0.0.1:8310"]));
        },
    ));
    v.push(edit_case("radio.cards = two cards", |d| {
        let c = table_at(d, "radio.cards[]").clone();
        let mut c2 = c.clone();
        c2.insert("index".into(), Value::Integer(1));
        table_at(d, "radio").insert(
            "cards".into(),
            Value::Array(vec![Value::Table(c), Value::Table(c2)]),
        );
    }));
    // radio.scan.busy_dbm: an NHM bucket edge, not just in range.
    for x in [-103, -86, -82, -75, -71] {
        v.push(set_case(
            "radio.scan",
            "busy_dbm",
            &x.to_string(),
            Value::Integer(x),
        ));
    }
    // Cross-key: detect_pct vs blocked_pct, detect_impaired vs windows.
    v.push(edit_case(
        "hop.verdict.detect_pct = 30.0 = blocked_pct",
        |d| {
            table_at(d, "hop.verdict").insert("detect_pct".into(), Value::Float(30.0));
        },
    ));
    v.push(edit_case(
        "hop.verdict.detect_impaired = detect_windows",
        |d| {
            let t = table_at(d, "hop.verdict");
            t.insert("detect_windows".into(), Value::Integer(7));
            t.insert("detect_impaired".into(), Value::Integer(7));
        },
    ));
    v.push(edit_case("no [hop], radio.scan.blocked_pct = 5", |d| {
        d.remove("hop");
        table_at(d, "radio.scan").insert("blocked_pct".into(), Value::Integer(5));
    }));
    // fec.symbol_size
    for (id, val) in [
        ("64", Value::Integer(64)),
        ("[64, 332]", ints(&[64, 332])),
        ("[64]", ints(&[64])),
        ("[64, 64, 64]", ints(&[64, 64, 64])),
        (
            "[1.5, 2.5]",
            Value::Array(vec![Value::Float(1.5), Value::Float(2.5)]),
        ),
        ("[\"a\", \"b\"]", strs(&["a", "b"])),
        ("\"x\"", Value::String("x".into())),
        ("true", Value::Boolean(true)),
        ("31", Value::Integer(31)),
        ("32", Value::Integer(32)),
        ("1500", Value::Integer(1500)),
        ("1501", Value::Integer(1501)),
        ("[32, 1501]", ints(&[32, 1501])),
        ("[31, 1500]", ints(&[31, 1500])),
        ("-1", Value::Integer(-1)),
    ] {
        v.push(set_case("fec", "symbol_size", id, val));
    }
    // Cross-key: nack lookback vs seq_horizon.
    v.push(edit_case("link.nack.lookback = fec.seq_horizon", |d| {
        table_at(d, "link.nack").insert("lookback".into(), Value::Integer(512));
    }));
    v.push(edit_case("link.nack.lookback = fec.seq_horizon - 1", |d| {
        table_at(d, "link.nack").insert("lookback".into(), Value::Integer(511));
    }));
    v.push(edit_case("no [link], fec.seq_horizon = 200", |d| {
        d.remove("link");
        table_at(d, "fec").insert("seq_horizon".into(), Value::Integer(200));
    }));
    // link.static_bw and a rung's bw: 20 or 40 only.
    v.push(set_case("link", "static_bw", "30", Value::Integer(30)));
    v.push(set_case("link.ladder[]", "bw", "30", Value::Integer(30)));
    v.push(edit_case("link.static_mcs = 3, static_bw = 40", |d| {
        let l = table_at(d, "link");
        l.insert("static_mcs".into(), Value::Integer(3));
        l.insert("static_bw".into(), Value::Integer(40));
    }));
    v.push(edit_case(
        "link.static_mcs = 3, static_bw = 20 (rx width 20)",
        |d| {
            let l = table_at(d, "link");
            l.insert("static_mcs".into(), Value::Integer(3));
            l.insert("static_bw".into(), Value::Integer(20));
        },
    ));
    // link.up_util against down_util.
    v.push(edit_case("link.up_util = down_util", |d| {
        table_at(d, "link").insert("up_util".into(), Value::Float(0.35));
    }));
    // Sentinels: probe.max_util and s3_down_util follow down_util when absent.
    v.push(edit_case(
        "link.probe.max_util and s3_down_util absent",
        |d| {
            table_at(d, "link.probe").remove("max_util");
            let l = table_at(d, "link");
            l.remove("s3_down_util");
            l.insert("down_util".into(), Value::Float(0.45));
        },
    ));
    // link.ladder
    v.push(set_case("link", "ladder", "[]", Value::Array(vec![])));
    v.push(set_case("link", "ladder", "5", Value::Integer(5)));
    let rung = |mcs: i64, bw: i64, ob: f64, oe: f64| {
        let mut t = Table::new();
        t.insert("mcs".into(), Value::Integer(mcs));
        t.insert("bw".into(), Value::Integer(bw));
        t.insert("overhead_base".into(), Value::Float(ob));
        t.insert("overhead_enh".into(), Value::Float(oe));
        Value::Table(t)
    };
    v.push(set_case(
        "link",
        "ladder",
        "8 rungs",
        Value::Array((0..8).map(|m| rung(m, 20, 0.5, 0.25)).collect()),
    ));
    v.push(set_case(
        "link",
        "ladder",
        "9 rungs",
        Value::Array((0..9).map(|m| rung(m % 8, 20, 0.5, 0.25)).collect()),
    ));
    v.push(set_case(
        "link",
        "ladder",
        "mixed 20 and 40 rungs",
        Value::Array(vec![rung(0, 20, 1.0, 0.5), rung(3, 40, 0.5, 0.5)]),
    ));
    v.push(edit_case("link.max_mcs = 0 over rungs from mcs 1", |d| {
        let l = table_at(d, "link");
        l.insert(
            "ladder".into(),
            Value::Array(vec![rung(1, 40, 1.0, 0.5), rung(2, 40, 1.0, 0.5)]),
        );
        l.insert("max_mcs".into(), Value::Integer(0));
    }));
    v.push(edit_case("link.max_mcs = 2 filters rungs 3 and 4", |d| {
        table_at(d, "link").insert("max_mcs".into(), Value::Integer(2));
    }));
    // stats.out
    v.push(set_case("stats", "out", "[]", Value::Array(vec![])));
    v.push(set_case("stats", "out", "5", Value::Integer(5)));
    v.push(edit_case("stats.out with stats.host", |d| {
        table_at(d, "stats").insert("host".into(), Value::String("10.0.0.1".into()));
    }));
    v.push(edit_case("stats.out[0] without port", |d| {
        table_at(d, "stats.out[]").remove("port");
    }));
    v.push(edit_case("stats: neither out nor host/port", |d| {
        table_at(d, "stats").remove("out");
    }));
    // mavlink.bind: a dotted IPv4 address.
    for b in ["10.1.2.3", "0.0.0.0", "01.2.3.4", "1.2.3", "localhost", ""] {
        v.push(set_case(
            "mavlink",
            "bind",
            &format!("{b:?}"),
            Value::String(b.into()),
        ));
    }
    // link.key (inline pairing key) and key_file.
    for (id, k) in [
        ("lower hex", "00112233445566778899aabbccddeeff"),
        ("upper hex", "00112233445566778899AABBCCDDEEFF"),
        ("31 chars", "00112233445566778899aabbccddeef"),
        ("33 chars", "00112233445566778899aabbccddeeff0"),
        ("non-hex", "0011223344556677889zaabbccddeeff"),
        ("plus sign", "+0112233445566778899aabbccddeeff"),
        ("empty", ""),
    ] {
        v.push(set_case("link", "key", id, Value::String(k.into())));
    }
    v.push(set_case("link", "key", "1", Value::Integer(1)));
    v.push(set_case(
        "link",
        "key_file",
        "a missing file",
        Value::String("/nonexistent/pal-config-diff.key".into()),
    ));
    v.push(set_case(
        "link",
        "key_file",
        "a directory",
        Value::String("/".into()),
    ));
    let dir = key_dir();
    std::fs::create_dir_all(&dir).unwrap();
    let key = "00112233445566778899aabbccddeeff";
    for (id, body) in [
        ("one key", format!("# pairing key\n{key}\n")),
        ("one key, CRLF", format!("{key}\r\n")),
        ("one key, vertical tab", format!("\x0b{key}\x0b\n")),
        ("one key, no-break space", format!("\u{a0}{key}\n")),
        ("two keys", format!("{key}\n{key}\n")),
        ("no key", "# none\n\n".to_string()),
        ("bad hex", "zz112233445566778899aabbccddeeff\n".to_string()),
        (
            "plus sign",
            "+0112233445566778899aabbccddeeff\n".to_string(),
        ),
    ] {
        let p = dir.join(id.replace([' ', ','], "_"));
        std::fs::write(&p, &body).unwrap();
        v.push(set_case(
            "link",
            "key_file",
            &format!("file with {id}"),
            Value::String(p.to_str().unwrap().into()),
        ));
    }
    let p = dir.join("not_utf8");
    std::fs::write(&p, b"\xff0112233445566778899aabbccddeeff\n").unwrap();
    v.push(set_case(
        "link",
        "key_file",
        "file with a non-UTF-8 byte",
        Value::String(p.to_str().unwrap().into()),
    ));
    v
}

fn keyed_cases() -> Vec<Case> {
    let mut v = Vec::new();
    for (sec, key) in leaves() {
        let path = join(sec, key);
        let k = kind(&path).unwrap_or_else(|| panic!("{path}: no range in kind(); add it"));
        // Deleted. A required key (a rung's bw) deleted is a refusal on both.
        let mut doc = base_for(sec, key);
        table_at(&mut doc, sec).remove(key);
        v.push(Case {
            id: format!("{path} deleted"),
            text: text(&doc),
        });
        match k {
            Kind::Int(lo, hi) => {
                for x in [lo - 1, lo, hi, hi + 1] {
                    v.push(set_case(sec, key, &x.to_string(), Value::Integer(x)));
                }
                v.push(set_case(sec, key, "\"x\"", Value::String("x".into())));
                v.push(set_case(sec, key, "1.5", Value::Float(1.5)));
                v.push(set_case(sec, key, "true", Value::Boolean(true)));
            }
            Kind::Num(lo, hi) => {
                let mut xs = vec![lo - 1.0, lo, hi, hi.next_up(), hi + 1.0];
                // Below zero the neighbour prints as a subnormal, which the
                // mabur TOML reader cannot take (no exponent form).
                if lo != 0.0 {
                    xs.push(lo.next_down());
                }
                for x in xs {
                    v.push(set_case(sec, key, &format!("{x:?}"), Value::Float(x)));
                }
                let i = hi.floor() as i64;
                v.push(set_case(
                    sec,
                    key,
                    &format!("{i} (integer)"),
                    Value::Integer(i),
                ));
                v.push(set_case(sec, key, "\"x\"", Value::String("x".into())));
                v.push(set_case(sec, key, "true", Value::Boolean(true)));
            }
            Kind::Bool => {
                for b in [true, false] {
                    v.push(set_case(sec, key, &b.to_string(), Value::Boolean(b)));
                }
                v.push(set_case(sec, key, "1", Value::Integer(1)));
                v.push(set_case(sec, key, "\"true\"", Value::String("true".into())));
            }
            Kind::Str => {
                v.push(set_case(
                    sec,
                    key,
                    "\"pal-test\"",
                    Value::String("pal-test".into()),
                ));
                v.push(set_case(sec, key, "1", Value::Integer(1)));
                v.push(set_case(sec, key, "true", Value::Boolean(true)));
            }
            Kind::Special => {}
        }
    }
    // Per section: an unknown key, the section deleted, the section a scalar.
    for (sec, _) in KEY_TABLE {
        let mut doc = base_for(sec, "");
        table_at(&mut doc, sec).insert("pal_bogus".into(), Value::Integer(1));
        v.push(Case {
            id: format!(
                "{} unknown key",
                if sec.is_empty() { "top level" } else { sec }
            ),
            text: text(&doc),
        });
        if sec.is_empty() {
            continue;
        }
        if let Some(arr) = sec.strip_suffix("[]") {
            let mut doc = base();
            table_at(&mut doc, sec);
            let (parent, key) = arr.rsplit_once('.').unwrap();
            table_at(&mut doc, parent).insert(key.into(), ints(&[5]));
            v.push(Case {
                id: format!("{arr} = [5]"),
                text: text(&doc),
            });
            continue;
        }
        let (parent, key) = sec.rsplit_once('.').unwrap_or(("", sec));
        let mut doc = base();
        table_at(&mut doc, parent).remove(key);
        v.push(Case {
            id: format!("{sec} deleted"),
            text: text(&doc),
        });
        let mut doc = base();
        table_at(&mut doc, parent).insert(key.into(), Value::Integer(5));
        v.push(Case {
            id: format!("{sec} = 5"),
            text: text(&doc),
        });
    }
    v
}

fn helper_cases() -> Vec<Case> {
    let gs = read("configs/gs.toml");
    let auto = read("configs/gs-auto.toml");
    let pick = |bw, chs: &[u8]| LinkPick {
        bw,
        channels: chs.to_vec(),
        s3_demote: Some(false),
    };
    let mut v = vec![
        Case {
            id: "with_link_bw 20".into(),
            text: config::with_link_bw(&gs, 20).unwrap(),
        },
        Case {
            id: "with_link_bw 40".into(),
            text: config::with_link_bw(&gs, 40).unwrap(),
        },
        Case {
            id: "with_channel 64".into(),
            text: config::with_channel(&auto, 64).unwrap(),
        },
        Case {
            id: "with_channel auto".into(),
            text: config::with_channel(&gs, 0).unwrap(),
        },
        Case {
            id: "with_channels [64, 112]".into(),
            text: config::with_channels(&gs, &[64, 112]).unwrap(),
        },
        Case {
            id: "with_search_grid 40 MHz".into(),
            text: config::with_search_grid(&auto).unwrap(),
        },
    ];
    for (id, p) in [
        ("link pick: 20 MHz, all channels", pick(20, &[])),
        ("link pick: 20 MHz, [161]", pick(20, &[161])),
        ("link pick: 40 MHz, [36, 40, 165]", pick(40, &[36, 40, 165])),
    ] {
        let (text, _) = config::with_link_pick(&auto, p);
        v.push(Case {
            id: id.into(),
            text,
        });
    }
    v
}

fn bundled_cases() -> Vec<Case> {
    [
        "configs/gs.toml",
        "configs/gs-auto.toml",
        "configs/bench-ch40.toml",
        "third_party/mabur/gs/bundle/maburgs.default.toml",
    ]
    .iter()
    .map(|f| Case {
        id: format!("bundled {f}"),
        text: read(f),
    })
    .chain(std::iter::once(Case {
        id: "empty file".into(),
        text: String::new(),
    }))
    .collect()
}

/// Every table present and empty: load_config reports each key it knows
/// as defaulted. Two documents, since an array of tables present (cards,
/// a rung, a stats destination) hides the array key's own report.
const ALL_EMPTY: [&str; 2] = [
    "[radio]\n[radio.scan]\n[hop]\n[hop.verdict]\n[fec]\n[link]\n[link.probe]\n[link.fade]\n\
     [link.nack]\n[link.rung_stats]\n[video]\n[msp]\n[msp.out]\n[stats]\n[au_ring]\n\
     [debug_log]\n[mavlink]\n",
    "[radio]\n[[radio.cards]]\n[link]\n[[link.ladder]]\nbw = 20\n[stats]\n[[stats.out]]\n\
     port = 8300\n",
];

#[test]
fn cpp_knows_the_rust_key_table() {
    let mut cpp_keys = BTreeSet::new();
    for doc in ALL_EMPTY {
        let (out, defaulted) = cpp(doc);
        assert!(matches!(out, Out::Loaded(_)), "{out:?}");
        for d in defaulted {
            let (k, v) = d.split_once('=').unwrap();
            // A table the document leaves out is one line, not its keys.
            if v != "(section absent)" {
                cpp_keys.insert(k.replace("[0]", "[]"));
            }
        }
    }
    cpp_keys.extend(NOT_REPORTED.iter().map(|(k, _)| k.to_string()));
    let rust_keys: BTreeSet<String> = leaves()
        .into_iter()
        .map(|(s, k)| join(s, k))
        .filter(|k| !RUST_ONLY_KEYS.iter().any(|(r, _)| r == k))
        .collect();
    let only_cpp: Vec<_> = cpp_keys.difference(&rust_keys).collect();
    let only_rust: Vec<_> = rust_keys.difference(&cpp_keys).collect();
    assert!(
        only_cpp.is_empty() && only_rust.is_empty(),
        "key sets differ: maburgs only {only_cpp:?} (add them to pal-core's parser and kind()), \
         pal only {only_rust:?} (remove them, or list them in RUST_ONLY_KEYS with the reason)"
    );
    for (k, _) in RUST_ONLY_KEYS {
        assert!(
            leaves().iter().any(|(s, l)| join(s, l) == *k),
            "{k}: listed as pal-only but not in KEY_TABLE"
        );
    }
}

#[test]
fn rust_parser_matches_load_config() {
    let t0 = Instant::now();
    let mut cases = bundled_cases();
    cases.extend(keyed_cases());
    cases.extend(special_cases());
    cases.extend(helper_cases());
    let mut ids = BTreeSet::new();
    for c in &cases {
        assert!(ids.insert(c.id.clone()), "case id {} twice", c.id);
    }
    let mut failures = Vec::new();
    let mut allowed_hit = BTreeSet::new();
    let mut one_sided_seen = BTreeSet::new();
    let mut both_loaded = 0;
    for c in &cases {
        let r = rust(&c.text);
        let (cc, _) = cpp(&c.text);
        if let (Out::Loaded(a), Out::Loaded(b)) = (&r, &cc) {
            both_loaded += 1;
            for (k, _) in ONE_SIDED {
                if a.contains_key(*k) != b.contains_key(*k) {
                    one_sided_seen.insert(*k);
                }
            }
        }
        let diff = compare(&r, &cc);
        let allowed = ALLOWED.iter().find(|(id, _)| *id == c.id);
        match (diff, allowed) {
            (Some(d), None) => failures.push(format!("{}: {d}", c.id)),
            (Some(_), Some(_)) => {
                allowed_hit.insert(c.id.clone());
            }
            (None, Some(_)) => failures.push(format!(
                "{}: listed in ALLOWED but both sides now agree; drop the entry",
                c.id
            )),
            (None, None) => {}
        }
    }
    let _ = std::fs::remove_dir_all(key_dir());
    for (id, _) in ALLOWED {
        if !ids.contains(*id) {
            failures.push(format!("ALLOWED names {id}, which no case has"));
        }
    }
    for (k, _) in ONE_SIDED {
        if !one_sided_seen.contains(k) {
            failures.push(format!(
                "ONE_SIDED lists {k}, but no loaded case printed it on one side only"
            ));
        }
    }
    // The range table against the C++: one past a bound must be refused as
    // out of range, the bound itself must not be.
    for (sec, key) in leaves() {
        let path = join(sec, key);
        let msg = format!(
            "config: {}: out of range",
            join(
                &sec.replace("cards[]", "cards[0]")
                    .replace("ladder[]", "ladder[0]"),
                key
            )
        );
        let probe = |x: Value| {
            let mut doc = base_for(sec, key);
            table_at(&mut doc, sec).insert(key.into(), x);
            matches!(cpp(&text(&doc)).0, Out::Refused(e) if e == msg)
        };
        let (inside, outside) = match kind(&path) {
            Some(Kind::Int(lo, hi)) => (
                vec![Value::Integer(lo), Value::Integer(hi)],
                vec![Value::Integer(lo - 1), Value::Integer(hi + 1)],
            ),
            Some(Kind::Num(lo, hi)) => (
                vec![Value::Float(lo), Value::Float(hi)],
                vec![Value::Float(lo - 1.0), Value::Float(hi.next_up())],
            ),
            _ => continue,
        };
        for x in inside {
            if probe(x.clone()) {
                failures.push(format!(
                    "kind() range of {path}: maburgs refuses {x} as out of range"
                ));
            }
        }
        for x in outside {
            if !probe(x.clone()) {
                failures.push(format!(
                    "kind() range of {path}: maburgs does not refuse {x} as out of range ({msg})"
                ));
            }
        }
    }
    eprintln!(
        "config_diff: {} keys, {} cases ({} loaded by both), {} allowed differences, {:?}",
        leaves().len(),
        cases.len(),
        both_loaded,
        allowed_hit.len(),
        t0.elapsed()
    );
    assert!(
        failures.is_empty(),
        "{} difference(s) between pal-core's config parser and maburgs::load_config:\n{}",
        failures.len(),
        failures.join("\n")
    );
}
