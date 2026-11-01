//! The GS TOML config, a port of mabur `gs/src/config.cpp` (pin 51cea32f):
//! same sections, keys, defaults, ranges and cross-checks, and the same
//! refusal of unknown keys (a config written for another build fails loudly).
//! Error text keeps mabur's "config: <field>: <why>" shape, without the line.

use pal_wire::rc::{LinkKey, DEFAULT_LINK_KEY};
use std::path::Path;
use toml::{Table, Value};

#[derive(Clone, Debug, PartialEq)]
pub struct CardCfg {
    pub usb_vid: u16,
    pub usb_pid: u16,
    pub index: i32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct BusyCfg {
    pub busy_dbm: i32,
    pub blocked_pct: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ScanCfg {
    pub dwell_ms: i32,
    pub settle_ms: i32,
    pub min_rounds: i32,
    pub search_ms: i32,
    pub op_window_ms: i32,
    pub search_after_ms: i32,
    pub pick_margin: i32,
    pub one_card_ms: i32,
    pub max_ms: i32,
    pub busy: BusyCfg,
}

impl Default for ScanCfg {
    fn default() -> Self {
        Self {
            dwell_ms: 250,
            settle_ms: 30,
            min_rounds: 3,
            search_ms: 100,
            op_window_ms: 300,
            search_after_ms: 5000,
            pick_margin: 20,
            one_card_ms: 5000,
            max_ms: 30000,
            busy: BusyCfg {
                busy_dbm: -83,
                blocked_pct: 50.0,
            },
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct HopVerdictCfg {
    pub loss_pct: f64,
    pub recovered_x: f64,
    pub recovered_min: i32,
    pub weak_rssi_dbm: i32,
    pub weak_snr_db: i32,
    pub fading_drop_db: i32,
    pub foreign_pps: i32,
    pub fa_pps: i32,
    pub starved_frac: f64,
}

impl Default for HopVerdictCfg {
    fn default() -> Self {
        Self {
            loss_pct: 3.0,
            recovered_x: 3.0,
            recovered_min: 8,
            weak_rssi_dbm: -78,
            weak_snr_db: 12,
            fading_drop_db: 6,
            foreign_pps: 50,
            fa_pps: 100,
            starved_frac: 0.25,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct HopCfg {
    pub window_ms: i32,
    pub persist: i32,
    pub dwell_observe_ms: i32,
    pub dwell_period_ms: i32,
    pub rank_visits: i32,
    pub rank_max_age_ms: i32,
    pub confirm_ms: i32,
    pub confirm_extend_ms: i32,
    pub verify_ms: i32,
    pub cooldown_ms: i32,
    pub max_hops_per_min: i32,
    pub backoff_ms: i32,
    pub one_card_repeats: i32,
    pub relay_burst_period_ms: i32,
    pub verdict: HopVerdictCfg,
}

impl Default for HopCfg {
    fn default() -> Self {
        Self {
            window_ms: 150,
            persist: 2,
            dwell_observe_ms: 5,
            dwell_period_ms: 333,
            rank_visits: 5,
            rank_max_age_ms: 10000,
            confirm_ms: 500,
            confirm_extend_ms: 3000,
            verify_ms: 1000,
            cooldown_ms: 2000,
            max_hops_per_min: 4,
            backoff_ms: 30000,
            one_card_repeats: 5,
            relay_burst_period_ms: 1000,
            verdict: HopVerdictCfg::default(),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct RadioCfg {
    pub channels: Vec<u8>,
    /// None = "auto".
    pub pin: Option<u8>,
    pub width: u8,
    pub cards: Vec<CardCfg>,
    pub auto_scan: bool,
    pub tx_card: i32,
    pub relays: Vec<String>,
    pub scan: ScanCfg,
}

impl Default for RadioCfg {
    fn default() -> Self {
        Self {
            channels: vec![40, 64, 112, 144],
            pin: None,
            width: 20,
            cards: Vec::new(),
            auto_scan: true,
            tx_card: -1,
            relays: Vec::new(),
            scan: ScanCfg::default(),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct FecCfg {
    pub symbol_size: [usize; 2],
    pub seq_horizon: u32,
}

impl Default for FecCfg {
    fn default() -> Self {
        Self {
            symbol_size: [64, 64],
            seq_horizon: 512,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rung {
    pub mcs: i32,
    pub overhead_base: f64,
    pub overhead_enh: f64,
    pub bw: i32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FadeCfg {
    pub cascade: bool,
    pub predict: bool,
    pub hold_ms: i32,
    pub confirm_ms: i32,
    pub rssi_db: f64,
    pub snr_db: f64,
    pub trigger_ms: i32,
    pub min_rung: i32,
}

impl Default for FadeCfg {
    fn default() -> Self {
        Self {
            cascade: true,
            predict: true,
            hold_ms: 2500,
            confirm_ms: 100,
            rssi_db: 8.0,
            snr_db: 4.0,
            trigger_ms: 300,
            min_rung: 2,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ProbeCfg {
    pub enable: bool,
    pub rung_offset: i32,
    pub clean_bodies: i32,
    /// Resolved: down_util when the file leaves it out.
    pub max_util: f64,
    pub min_syms: i32,
    pub silence_ms: i32,
    pub pin_mcs: i32,
}

impl Default for ProbeCfg {
    fn default() -> Self {
        Self {
            enable: true,
            rung_offset: 1,
            clean_bodies: 90,
            max_util: -1.0,
            min_syms: 16,
            silence_ms: 500,
            pin_mcs: -1,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct LadderCfg {
    pub ladder: Vec<Rung>,
    pub down_util: f64,
    pub up_util: f64,
    pub confirm_ms: i32,
    pub clean_ms: i32,
    pub probation_ms: i32,
    pub penalty_base_ms: i32,
    pub penalty_max_ms: i32,
    pub hold_after_down_ms: i32,
    pub min_between_changes_ms: i32,
    pub feedback_timeout_ms: i32,
    pub starved_confirm_ms: i32,
    pub probe: ProbeCfg,
    pub s3_demote: bool,
    pub s3_down_util: f64,
    pub s3_min_syms: i32,
    pub s3_settle_ms: i32,
    pub fade: FadeCfg,
    pub rung_stats_half_life_samples: i32,
}

fn rung(mcs: i32, b: f64, e: f64) -> Rung {
    Rung {
        mcs,
        overhead_base: b,
        overhead_enh: e,
        bw: 20,
    }
}

impl Default for LadderCfg {
    fn default() -> Self {
        Self {
            ladder: vec![
                rung(0, 2.0, 2.0),
                rung(2, 1.0, 1.0),
                rung(4, 0.5, 0.5),
                rung(5, 0.5, 0.5),
                rung(6, 0.5, 0.5),
                rung(7, 0.2, 0.2),
            ],
            down_util: 0.6,
            up_util: 0.15,
            confirm_ms: 250,
            clean_ms: 5000,
            probation_ms: 3000,
            penalty_base_ms: 10000,
            penalty_max_ms: 60000,
            hold_after_down_ms: 4000,
            min_between_changes_ms: 150,
            feedback_timeout_ms: 1000,
            starved_confirm_ms: 300,
            probe: ProbeCfg::default(),
            s3_demote: true,
            s3_down_util: -1.0,
            s3_min_syms: 50,
            s3_settle_ms: 300,
            fade: FadeCfg::default(),
            rung_stats_half_life_samples: 600,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct NackCfg {
    pub enable: bool,
    pub lookback: i32,
    pub repeat_ms: i32,
    pub max_tries: i32,
    pub min_lead_ms: i32,
    pub settle_min_ms: i32,
    pub settle_max_ms: i32,
    pub settle_seed_ms: i32,
    pub settle_window_ms: i32,
    pub settle_min_samples: i32,
}

impl Default for NackCfg {
    fn default() -> Self {
        Self {
            enable: false,
            lookback: 256,
            repeat_ms: 16,
            max_tries: 2,
            min_lead_ms: 12,
            settle_min_ms: 4,
            settle_max_ms: 24,
            settle_seed_ms: 12,
            settle_window_ms: 10000,
            settle_min_samples: 50,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct LinkCfg {
    pub feedback_ms: i32,
    pub beacon_keepalive_ms: i32,
    pub rcf_slot_hold_ms: i32,
    pub arrival_guard_syms: u32,
    pub nack: NackCfg,
    pub static_mcs: i32,
    pub static_overhead_base: f64,
    pub static_overhead_enh: f64,
    pub static_bw: i32,
    pub ladder_cfg: LadderCfg,
    pub key_file: String,
    pub key: LinkKey,
    pub key_is_default: bool,
    pub key_source: String,
}

impl Default for LinkCfg {
    fn default() -> Self {
        Self {
            feedback_ms: 100,
            beacon_keepalive_ms: 1000,
            rcf_slot_hold_ms: 30,
            arrival_guard_syms: 192,
            nack: NackCfg::default(),
            static_mcs: -1,
            static_overhead_base: 0.5,
            static_overhead_enh: 0.5,
            static_bw: 20,
            ladder_cfg: LadderCfg::default(),
            key_file: "/etc/mabur.key".into(),
            key: DEFAULT_LINK_KEY,
            key_is_default: true,
            key_source: String::new(),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct VideoCfg {
    pub frame_gap_timeout_ms: u64,
    pub frame_gap_timeout_max_ms: u64,
    pub frame_lookahead: u64,
}

impl Default for VideoCfg {
    fn default() -> Self {
        Self {
            frame_gap_timeout_ms: 50,
            frame_gap_timeout_max_ms: 150,
            frame_lookahead: 8,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct MspCfg {
    pub enable: bool,
    pub out_host: String,
    pub out_port: i32,
    pub symbol_size: usize,
    pub window: usize,
}

impl Default for MspCfg {
    fn default() -> Self {
        Self {
            enable: false,
            out_host: "127.0.0.1".into(),
            out_port: 14560,
            symbol_size: 1312,
            window: 16,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct StatsOut {
    pub host: String,
    pub port: i32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct StatsCfg {
    pub enable: bool,
    pub interval_ms: i32,
    pub out: Vec<StatsOut>,
}

impl Default for StatsCfg {
    fn default() -> Self {
        Self {
            enable: false,
            interval_ms: 500,
            out: vec![StatsOut {
                host: "127.0.0.1".into(),
                port: 8300,
            }],
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct AuRingOutCfg {
    pub enable: bool,
    pub path: String,
    pub socket: String,
    pub slot_kb: i32,
    pub slot_count: i32,
}

impl Default for AuRingOutCfg {
    fn default() -> Self {
        Self {
            enable: false,
            path: "/dev/shm/mabur-au".into(),
            socket: "/run/mabur-au.sock".into(),
            slot_kb: 512,
            slot_count: 16,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct DebugLogCfg {
    pub enable: bool,
    pub dir: String,
    pub ctl_period_ms: i32,
    pub rung_period_s: i32,
}

impl Default for DebugLogCfg {
    fn default() -> Self {
        Self {
            enable: false,
            dir: "/media/dvr/log".into(),
            ctl_period_ms: 1000,
            rung_period_s: 10,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct MavlinkCfg {
    pub enable: bool,
    pub bind: String,
    pub port: i32,
}

impl Default for MavlinkCfg {
    fn default() -> Self {
        Self {
            enable: true,
            bind: "127.0.0.1".into(),
            port: 14550,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Default)]
pub struct Config {
    pub radio: RadioCfg,
    pub fec: FecCfg,
    pub link: LinkCfg,
    pub video: VideoCfg,
    pub msp: MspCfg,
    pub stats: StatsCfg,
    pub au_ring: AuRingOutCfg,
    pub debug_log: DebugLogCfg,
    pub hop: HopCfg,
    pub mavlink: MavlinkCfg,
}

/// One decode layer: what the GS decoder needs of the drone's FEC (window
/// and overhead are not on the decode path; mabur pins them 128 / 0.5).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UepLayerCfg {
    pub symbol_size: usize,
    pub window: usize,
}

impl Config {
    pub fn uep_layers(&self) -> [UepLayerCfg; 2] {
        [0, 1].map(|s| UepLayerCfg {
            symbol_size: self.fec.symbol_size[s],
            window: 128,
        })
    }
}

/// 0 = no standard HT40 pair; 1 = secondary above; 2 = below.
pub fn ht40_offset(ch: u8) -> u8 {
    if (36..=144).contains(&ch) && (ch - 36) % 4 == 0 {
        return if ((ch - 36) / 4) % 2 == 0 { 1 } else { 2 };
    }
    if (149..=161).contains(&ch) && (ch - 149) % 4 == 0 {
        return if ((ch - 149) / 4) % 2 == 0 { 1 } else { 2 };
    }
    0
}

pub fn ht40_pair_other(ch: u8) -> u8 {
    match ht40_offset(ch) {
        1 => ch + 4,
        2 => ch - 4,
        _ => 0,
    }
}

const NHM_EDGES: [i32; 11] = [-104, -101, -98, -95, -92, -89, -86, -83, -80, -75, -70];

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConfigError(pub String);

impl std::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for ConfigError {}

type R<T> = Result<T, ConfigError>;

fn fail<T>(field: &str, why: &str) -> R<T> {
    Err(ConfigError(format!("config: {field}: {why}")))
}

fn join(where_: &str, key: &str) -> String {
    if where_.is_empty() {
        key.to_string()
    } else {
        format!("{where_}.{key}")
    }
}

fn check_keys(t: &Table, where_: &str, allowed: &[&str]) -> R<()> {
    for k in t.keys() {
        if !allowed.contains(&k.as_str()) {
            return fail(&join(where_, k), "unknown key");
        }
    }
    Ok(())
}

fn get_int(t: &Table, key: &str, dflt: i64, lo: i64, hi: i64, w: &str) -> R<i64> {
    match t.get(key) {
        None => Ok(dflt),
        Some(Value::Integer(v)) => {
            if *v < lo || *v > hi {
                fail(&join(w, key), "out of range")
            } else {
                Ok(*v)
            }
        }
        Some(_) => fail(&join(w, key), "not an integer"),
    }
}

fn get_i32(t: &Table, key: &str, dflt: i64, lo: i64, hi: i64, w: &str) -> R<i32> {
    get_int(t, key, dflt, lo, hi, w).map(|v| v as i32)
}

fn get_num(t: &Table, key: &str, dflt: f64, lo: f64, hi: f64, w: &str) -> R<f64> {
    let v = match t.get(key) {
        None => return Ok(dflt),
        Some(Value::Integer(v)) => *v as f64,
        Some(Value::Float(v)) => *v,
        Some(_) => return fail(&join(w, key), "not a number"),
    };
    if v < lo || v > hi {
        fail(&join(w, key), "out of range")
    } else {
        Ok(v)
    }
}

fn get_str(t: &Table, key: &str, dflt: &str, w: &str) -> R<String> {
    match t.get(key) {
        None => Ok(dflt.to_string()),
        Some(Value::String(s)) => Ok(s.clone()),
        Some(_) => fail(&join(w, key), "not a string"),
    }
}

fn get_bool(t: &Table, key: &str, dflt: bool, w: &str) -> R<bool> {
    match t.get(key) {
        None => Ok(dflt),
        Some(Value::Boolean(b)) => Ok(*b),
        Some(_) => fail(&join(w, key), "not a boolean"),
    }
}

fn table<'a>(v: &'a Value, field: &str) -> R<&'a Table> {
    v.as_table()
        .ok_or_else(|| ConfigError(format!("config: {field}: not a table")))
}

fn section<'a>(j: &'a Table, key: &str) -> R<Option<&'a Table>> {
    j.get(key).map(|v| table(v, key)).transpose()
}

fn merge_overlay(base: &mut Table, ov: &Table) {
    for (k, v) in ov {
        match (base.get_mut(k), v) {
            (Some(Value::Table(b)), Value::Table(o)) => merge_overlay(b, o),
            (Some(b), _) => *b = v.clone(),
            (None, _) => {
                base.insert(k.clone(), v.clone());
            }
        }
    }
}

pub fn parse_key_hex(hex: &str) -> Option<LinkKey> {
    if hex.len() != 32 || !hex.is_ascii() {
        return None;
    }
    let mut k = [0u8; 16];
    for (i, b) in k.iter_mut().enumerate() {
        *b = u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).ok()?;
    }
    Some(k)
}

/// mabur `load_key_file`: missing file = the default key; one 32-hex token
/// per file, `#` comments allowed.
pub fn load_key_file(path: &str) -> Result<(LinkKey, bool, String), String> {
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            return Ok((DEFAULT_LINK_KEY, true, "default".into()))
        }
        Err(e) => return Err(format!("{path}: {e}")),
    };
    let toks: Vec<&str> = text
        .lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .collect();
    match toks.as_slice() {
        [] => Err(format!("{path}: no key")),
        [t] => parse_key_hex(t)
            .map(|k| (k, false, path.to_string()))
            .ok_or_else(|| format!("{path}: not 32 hex characters")),
        _ => Err(format!("{path}: more than one key")),
    }
}

fn channel_set_issue(set: &[u8], width: u8, field: &str) -> R<()> {
    if set.is_empty() || set.len() > 8 {
        return fail(field, "must list 1 to 8 channels");
    }
    for (i, &ch) in set.iter().enumerate() {
        if !(1..=177).contains(&ch) {
            return fail(field, &format!("channel {ch} must be in [1,177]"));
        }
        if set[..i].contains(&ch) {
            return fail(field, &format!("duplicate channel {ch}"));
        }
    }
    if width == 40 {
        let off0 = ht40_offset(set[0]);
        for &ch in set {
            let off = ht40_offset(ch);
            if off == 0 {
                return fail(field, &format!("channel {ch} has no 40 MHz pair (docs/bw40.md)"));
            }
            if off != off0 {
                return fail(
                    field,
                    &format!(
                        "channel {ch} is on the other side of the pair grid from channel {}; FastRetune keeps the offset, every member must share one (docs/bw40.md)",
                        set[0]
                    ),
                );
            }
        }
    }
    Ok(())
}

pub fn radio_width_issue(channel: u8, width: i32) -> Option<(String, String)> {
    if width != 20 && width != 40 {
        return Some(("radio.width".into(), "must be 20 or 40 (HT20 / HT40)".into()));
    }
    if width == 40 && ht40_offset(channel) == 0 {
        return Some((
            "radio.width".into(),
            format!("40 MHz needs a standard 5 GHz pair and channel {channel} has none (common/include/mabur/ht40.h)"),
        ));
    }
    None
}

pub fn link_width_issue(link: &LinkCfg, width: i32) -> Option<(String, String)> {
    for (i, r) in link.ladder_cfg.ladder.iter().enumerate() {
        if r.bw == 40 && width != 40 {
            return Some((
                format!("link.ladder[{i}].bw"),
                "40 MHz rung but radio.width is 20: the GS could not receive it".into(),
            ));
        }
    }
    if link.static_bw == 40 && width != 40 {
        return Some(("link.static_bw".into(), "40 MHz pin but radio.width is 20".into()));
    }
    None
}

fn is_ipv4(s: &str) -> bool {
    s.parse::<std::net::Ipv4Addr>().is_ok()
}

pub fn load_config(path: &Path, overlay: Option<&Path>) -> R<Config> {
    let read = |p: &Path| -> R<Table> {
        let text = std::fs::read_to_string(p).map_err(|e| ConfigError(format!("config: {}: {e}", p.display())))?;
        text.parse::<Table>()
            .map_err(|e| ConfigError(format!("config: {}: {e}", p.display())))
    };
    let mut j = read(path)?;
    if let Some(o) = overlay {
        merge_overlay(&mut j, &read(o)?);
    }
    parse_config(&j)
}

pub fn parse_config_str(text: &str) -> R<Config> {
    let j = text
        .parse::<Table>()
        .map_err(|e| ConfigError(format!("config: {e}")))?;
    parse_config(&j)
}

fn parse_config(j: &Table) -> R<Config> {
    check_keys(
        j,
        "",
        &["radio", "fec", "link", "video", "msp", "stats", "au_ring", "debug_log", "hop", "mavlink"],
    )?;
    let mut c = Config::default();
    let mut cards_absent = false;
    if let Some(r) = section(j, "radio")? {
        check_keys(r, "radio", &["channel", "channels", "width", "cards", "tx_card", "scan", "relays"])?;
        c.radio.width = get_int(r, "width", 20, 20, 40, "radio")? as u8;
        if let Some(v) = r.get("channels") {
            let Value::Array(a) = v else {
                return fail("radio.channels", "not an array");
            };
            c.radio.channels.clear();
            for x in a {
                let Value::Integer(ch) = x else {
                    return fail("radio.channels", "not an integer");
                };
                if !(1..=177).contains(ch) {
                    return fail("radio.channels", "must be in [1,177]");
                }
                c.radio.channels.push(*ch as u8);
            }
        }
        channel_set_issue(&c.radio.channels, c.radio.width, "radio.channels")?;
        for &ch in &c.radio.channels {
            if let Some((f, w)) = radio_width_issue(ch, i32::from(c.radio.width)) {
                return fail(&f, &w);
            }
        }
        match r.get("channel") {
            None => {}
            Some(Value::String(s)) => {
                if s != "auto" {
                    return fail("radio.channel", "must be \"auto\" or a member of radio.channels");
                }
                c.radio.pin = None;
            }
            Some(Value::Integer(ch)) => {
                if !(1..=177).contains(ch) {
                    return fail("radio.channel", &format!("{ch} is not a channel in [1,177]"));
                }
                if !c.radio.channels.contains(&(*ch as u8)) {
                    return fail("radio.channel", &format!("{ch} is not a member of radio.channels"));
                }
                c.radio.pin = Some(*ch as u8);
            }
            Some(_) => return fail("radio.channel", "must be \"auto\" or a member of radio.channels"),
        }
        c.radio.tx_card = get_i32(r, "tx_card", -1, -1, 15, "radio")?;
        if let Some(v) = r.get("relays") {
            let Value::Array(a) = v else {
                return fail("radio.relays", "not an array");
            };
            for (i, x) in a.iter().enumerate() {
                let w = format!("radio.relays[{i}]");
                let Value::String(s) = x else {
                    return fail(&w, "not a string");
                };
                let Some(colon) = s.rfind(':').filter(|&c| c > 0 && c + 1 < s.len()) else {
                    return fail(&w, "must be ipv4:port");
                };
                if !is_ipv4(&s[..colon]) {
                    return fail(&w, "host must be a dotted IPv4 address (the UDP transport resolves nothing)");
                }
                let port = &s[colon + 1..];
                if port.len() > 5 || !port.bytes().all(|b| b.is_ascii_digit()) {
                    return fail(&w, "port is not a number");
                }
                let p: u32 = port.parse().unwrap_or(0);
                if !(1..=65535).contains(&p) {
                    return fail(&w, "port must be in [1,65535]");
                }
                if c.radio.relays.contains(s) {
                    return fail(&w, "duplicate relay address");
                }
                c.radio.relays.push(s.clone());
            }
        }
        if let Some(v) = r.get("cards") {
            let a = match v {
                Value::Array(a) if !a.is_empty() => a,
                _ => return fail("radio.cards", "must be a non-empty array"),
            };
            for (i, x) in a.iter().enumerate() {
                let w = format!("radio.cards[{i}]");
                let t = table(x, &w)?;
                check_keys(t, &w, &["usb_vid", "usb_pid", "index"])?;
                c.radio.cards.push(CardCfg {
                    usb_vid: get_int(t, "usb_vid", 0x0bda, 0, 0xFFFF, &w)? as u16,
                    usb_pid: get_int(t, "usb_pid", 0, 0, 0xFFFF, &w)? as u16,
                    index: get_i32(t, "index", 0, 0, 15, &w)?,
                });
            }
        } else {
            cards_absent = true;
        }
        if let Some(s) = section(r, "scan")? {
            let w = "radio.scan";
            check_keys(
                s,
                w,
                &[
                    "dwell_ms",
                    "settle_ms",
                    "min_rounds",
                    "search_ms",
                    "op_window_ms",
                    "search_after_ms",
                    "pick_margin",
                    "one_card_ms",
                    "max_ms",
                    "busy_dbm",
                    "blocked_pct",
                ],
            )?;
            let sc = &mut c.radio.scan;
            sc.dwell_ms = get_i32(s, "dwell_ms", 250, 50, 10000, w)?;
            sc.settle_ms = get_i32(s, "settle_ms", 30, 0, 1000, w)?;
            sc.min_rounds = get_i32(s, "min_rounds", 3, 1, 100, w)?;
            sc.search_ms = get_i32(s, "search_ms", 100, 40, 2000, w)?;
            sc.op_window_ms = get_i32(s, "op_window_ms", 300, 40, 10000, w)?;
            sc.search_after_ms = get_i32(s, "search_after_ms", 5000, 0, 600000, w)?;
            sc.pick_margin = get_i32(s, "pick_margin", 20, 0, 100000, w)?;
            sc.one_card_ms = get_i32(s, "one_card_ms", 5000, 0, 60000, w)?;
            sc.max_ms = get_i32(s, "max_ms", 30000, 1000, 600000, w)?;
            sc.busy.busy_dbm = get_i32(s, "busy_dbm", -83, -104, -70, w)?;
            if !NHM_EDGES.contains(&sc.busy.busy_dbm) {
                return fail(
                    "radio.scan.busy_dbm",
                    "must be an NHM bucket edge: -104 -101 -98 -95 -92 -89 -86 -83 -80 -75 -70",
                );
            }
            sc.busy.blocked_pct = get_num(s, "blocked_pct", 50.0, 1.0, 100.0, w)?;
        }
    }
    let _ = cards_absent;
    c.radio.auto_scan = c.radio.cards.is_empty();
    if !c.radio.auto_scan && c.radio.tx_card >= (c.radio.cards.len() + c.radio.relays.len()) as i32 {
        return fail("radio.tx_card", "no such card");
    }
    if let Some(h) = section(j, "hop")? {
        check_keys(
            h,
            "hop",
            &[
                "window_ms",
                "persist",
                "dwell_observe_ms",
                "dwell_period_ms",
                "rank_visits",
                "rank_max_age_ms",
                "confirm_ms",
                "confirm_extend_ms",
                "verify_ms",
                "cooldown_ms",
                "max_hops_per_min",
                "backoff_ms",
                "one_card_repeats",
                "relay_burst_period_ms",
                "verdict",
            ],
        )?;
        let hc = &mut c.hop;
        hc.window_ms = get_i32(h, "window_ms", 150, 50, 2000, "hop")?;
        hc.persist = get_i32(h, "persist", 2, 1, 3, "hop")?;
        hc.dwell_observe_ms = get_i32(h, "dwell_observe_ms", 5, 1, 250, "hop")?;
        hc.dwell_period_ms = get_i32(h, "dwell_period_ms", 333, 20, 60000, "hop")?;
        hc.rank_visits = get_i32(h, "rank_visits", 5, 1, 100, "hop")?;
        hc.rank_max_age_ms = get_i32(h, "rank_max_age_ms", 10000, 1000, 600000, "hop")?;
        hc.confirm_ms = get_i32(h, "confirm_ms", 500, 100, 5000, "hop")?;
        hc.confirm_extend_ms = get_i32(h, "confirm_extend_ms", 3000, 0, 30000, "hop")?;
        hc.verify_ms = get_i32(h, "verify_ms", 1000, 200, 10000, "hop")?;
        hc.cooldown_ms = get_i32(h, "cooldown_ms", 2000, 0, 60000, "hop")?;
        hc.max_hops_per_min = get_i32(h, "max_hops_per_min", 4, 1, 60, "hop")?;
        hc.backoff_ms = get_i32(h, "backoff_ms", 30000, 1000, 600000, "hop")?;
        hc.one_card_repeats = get_i32(h, "one_card_repeats", 5, 1, 50, "hop")?;
        hc.relay_burst_period_ms = get_i32(h, "relay_burst_period_ms", 1000, 500, 60000, "hop")?;
        if let Some(v) = section(h, "verdict")? {
            let w = "hop.verdict";
            check_keys(
                v,
                w,
                &[
                    "loss_pct",
                    "recovered_x",
                    "weak_rssi_dbm",
                    "weak_snr_db",
                    "fading_drop_db",
                    "foreign_pps",
                    "fa_pps",
                    "recovered_min",
                    "starved_frac",
                ],
            )?;
            let vc = &mut hc.verdict;
            vc.loss_pct = get_num(v, "loss_pct", 3.0, 0.1, 100.0, w)?;
            vc.recovered_x = get_num(v, "recovered_x", 3.0, 1.0, 100.0, w)?;
            vc.recovered_min = get_i32(v, "recovered_min", 8, 0, 1000, w)?;
            vc.weak_rssi_dbm = get_i32(v, "weak_rssi_dbm", -78, -110, -20, w)?;
            vc.weak_snr_db = get_i32(v, "weak_snr_db", 12, 0, 40, w)?;
            vc.fading_drop_db = get_i32(v, "fading_drop_db", 6, 1, 40, w)?;
            vc.foreign_pps = get_i32(v, "foreign_pps", 50, 1, 100000, w)?;
            vc.fa_pps = get_i32(v, "fa_pps", 100, 1, 100000, w)?;
            vc.starved_frac = get_num(v, "starved_frac", 0.25, 0.0, 1.0, w)?;
        }
    }
    if let Some(r) = section(j, "fec")? {
        check_keys(r, "fec", &["symbol_size", "seq_horizon"])?;
        match r.get("symbol_size") {
            None => {}
            Some(Value::Array(a)) => {
                if a.len() != 2 {
                    return fail("fec.symbol_size", "array must have 2 ints");
                }
                for (i, x) in a.iter().enumerate() {
                    let Value::Integer(v) = x else {
                        return fail("fec.symbol_size", "wrong type");
                    };
                    c.fec.symbol_size[i] = *v as usize;
                }
            }
            Some(Value::Integer(v)) => c.fec.symbol_size = [*v as usize; 2],
            Some(_) => return fail("fec.symbol_size", "not an integer"),
        }
        if c.fec.symbol_size.iter().any(|&v| !(32..=1500).contains(&v)) {
            return fail("fec.symbol_size", "must be in [32,1500]");
        }
        c.fec.seq_horizon = get_int(r, "seq_horizon", 512, 16, 65536, "fec")? as u32;
    }
    let mut inline_key = String::new();
    if let Some(r) = section(j, "link")? {
        check_keys(
            r,
            "link",
            &[
                "feedback_ms",
                "beacon_keepalive_ms",
                "static_mcs",
                "static_overhead_base",
                "static_overhead_enh",
                "static_bw",
                "ladder",
                "max_mcs",
                "down_util",
                "up_util",
                "confirm_ms",
                "clean_ms",
                "probation_ms",
                "penalty_base_ms",
                "penalty_max_ms",
                "hold_after_down_ms",
                "min_between_changes_ms",
                "feedback_timeout_ms",
                "starved_confirm_ms",
                "s3_demote",
                "s3_down_util",
                "s3_settle_ms",
                "s3_min_syms",
                "rung_stats",
                "fade",
                "probe",
                "rcf_slot_hold_ms",
                "arrival_guard_syms",
                "nack",
                "key_file",
                "key",
            ],
        )?;
        let l = &mut c.link;
        l.key_file = get_str(r, "key_file", "/etc/mabur.key", "link")?;
        if r.contains_key("key") {
            inline_key = get_str(r, "key", "", "link")?;
        }
        l.feedback_ms = get_i32(r, "feedback_ms", 100, 20, 5000, "link")?;
        l.rcf_slot_hold_ms = get_i32(r, "rcf_slot_hold_ms", 30, 0, 1000, "link")?;
        l.arrival_guard_syms = get_int(r, "arrival_guard_syms", 192, 16, 512, "link")? as u32;
        l.beacon_keepalive_ms = get_i32(r, "beacon_keepalive_ms", 1000, 100, 60000, "link")?;
        l.static_mcs = get_i32(r, "static_mcs", -1, -1, 7, "link")?;
        l.static_overhead_base = get_num(r, "static_overhead_base", 0.5, 0.1, 2.0, "link")?;
        l.static_overhead_enh = get_num(r, "static_overhead_enh", 0.5, 0.1, 2.0, "link")?;
        l.static_bw = get_i32(r, "static_bw", 20, 20, 40, "link")?;
        if l.static_bw != 20 && l.static_bw != 40 {
            return fail("link.static_bw", "must be 20 or 40");
        }
        let lc = &mut l.ladder_cfg;
        if let Some(v) = r.get("ladder") {
            let a = match v {
                Value::Array(a) if !a.is_empty() => a,
                _ => return fail("link.ladder", "must be a non-empty array"),
            };
            if a.len() > 8 {
                return fail("link.ladder", "must have at most 8 entries");
            }
            let mut parsed = Vec::new();
            for (i, x) in a.iter().enumerate() {
                let w = format!("link.ladder[{i}]");
                let t = table(x, &w)?;
                check_keys(t, &w, &["mcs", "bw", "overhead_base", "overhead_enh"])?;
                let mcs = get_i32(t, "mcs", 0, 0, 7, &w)?;
                if !t.contains_key("bw") {
                    return fail(&format!("{w}.bw"), "required: 20 or 40 (per-rung width)");
                }
                let bw = get_i32(t, "bw", 20, 20, 40, &w)?;
                if bw != 20 && bw != 40 {
                    return fail(&format!("{w}.bw"), "must be 20 or 40");
                }
                let ob = get_num(t, "overhead_base", 2.0, 0.1, 2.0, &w)?;
                let oe = get_num(t, "overhead_enh", 2.0, 0.1, 2.0, &w)?;
                if ob < oe {
                    return fail(&format!("{w}.overhead_base"), "must be >= overhead_enh");
                }
                parsed.push(Rung {
                    mcs,
                    overhead_base: ob,
                    overhead_enh: oe,
                    bw,
                });
            }
            lc.ladder = parsed;
        }
        let max_mcs = get_int(r, "max_mcs", 7, 0, 7, "link")? as i32;
        lc.ladder.retain(|x| x.mcs <= max_mcs);
        if lc.ladder.is_empty() {
            return fail("link.ladder", "empty after max_mcs filter");
        }
        lc.down_util = get_num(r, "down_util", 0.6, 0.0, 1.0, "link")?;
        lc.up_util = get_num(r, "up_util", 0.15, 0.0, 1.0, "link")?;
        if lc.up_util <= 0.0 {
            return fail("link.up_util", "must be > 0");
        }
        if lc.up_util >= lc.down_util {
            return fail("link.up_util", "must be < down_util");
        }
        lc.confirm_ms = get_i32(r, "confirm_ms", 250, 0, 600000, "link")?;
        lc.clean_ms = get_i32(r, "clean_ms", 5000, 0, 600000, "link")?;
        lc.probation_ms = get_i32(r, "probation_ms", 3000, 0, 600000, "link")?;
        lc.penalty_base_ms = get_i32(r, "penalty_base_ms", 10000, 0, 600000, "link")?;
        lc.penalty_max_ms = get_i32(r, "penalty_max_ms", 60000, 0, 600000, "link")?;
        lc.hold_after_down_ms = get_i32(r, "hold_after_down_ms", 4000, 0, 600000, "link")?;
        lc.min_between_changes_ms = get_i32(r, "min_between_changes_ms", 150, 0, 600000, "link")?;
        lc.feedback_timeout_ms = get_i32(r, "feedback_timeout_ms", 1000, 0, 600000, "link")?;
        lc.starved_confirm_ms = get_i32(r, "starved_confirm_ms", 300, 0, 600000, "link")?;
        lc.s3_min_syms = get_i32(r, "s3_min_syms", 50, 1, 100000, "link")?;
        if let Some(p) = section(r, "probe")? {
            let w = "link.probe";
            check_keys(
                p,
                w,
                &["enable", "rung_offset", "clean_bodies", "max_util", "min_syms", "silence_ms", "pin_mcs"],
            )?;
            let pc = &mut lc.probe;
            pc.enable = get_bool(p, "enable", pc.enable, w)?;
            pc.rung_offset = get_i32(p, "rung_offset", 1, 1, 7, w)?;
            pc.clean_bodies = get_i32(p, "clean_bodies", 90, 10, 100000, w)?;
            if p.contains_key("max_util") {
                pc.max_util = get_num(p, "max_util", 0.35, 0.01, 2.0, w)?;
            }
            pc.min_syms = get_i32(p, "min_syms", 16, 4, 100000, w)?;
            pc.silence_ms = get_i32(p, "silence_ms", 500, 100, 10000, w)?;
            pc.pin_mcs = get_i32(p, "pin_mcs", -1, -1, 7, w)?;
        }
        lc.s3_demote = get_bool(r, "s3_demote", lc.s3_demote, "link")?;
        if r.contains_key("s3_down_util") {
            lc.s3_down_util = get_num(r, "s3_down_util", 0.35, 0.01, 2.0, "link")?;
        }
        lc.s3_settle_ms = get_i32(r, "s3_settle_ms", 300, 0, 5000, "link")?;
        if let Some(f) = section(r, "fade")? {
            let w = "link.fade";
            check_keys(
                f,
                w,
                &["cascade", "predict", "hold_ms", "confirm_ms", "rssi_db", "snr_db", "trigger_ms", "min_rung"],
            )?;
            let fc = &mut lc.fade;
            fc.cascade = get_bool(f, "cascade", fc.cascade, w)?;
            fc.predict = get_bool(f, "predict", fc.predict, w)?;
            fc.hold_ms = get_i32(f, "hold_ms", 2500, 0, 60000, w)?;
            fc.confirm_ms = get_i32(f, "confirm_ms", 100, 20, 1000, w)?;
            fc.rssi_db = get_num(f, "rssi_db", 8.0, 0.5, 40.0, w)?;
            fc.snr_db = get_num(f, "snr_db", 4.0, 0.5, 40.0, w)?;
            fc.trigger_ms = get_i32(f, "trigger_ms", 300, 50, 5000, w)?;
            fc.min_rung = get_i32(f, "min_rung", 2, 0, 15, w)?;
        }
        if let Some(n) = section(r, "nack")? {
            let w = "link.nack";
            check_keys(n, w, &["enable", "lookback", "repeat_ms", "max_tries", "min_lead_ms"])?;
            let nc = &mut l.nack;
            nc.enable = get_bool(n, "enable", nc.enable, w)?;
            nc.lookback = get_i32(n, "lookback", 256, 8, 4096, w)?;
            nc.repeat_ms = get_i32(n, "repeat_ms", 16, 1, 1000, w)?;
            nc.max_tries = get_i32(n, "max_tries", 2, 0, 16, w)?;
            nc.min_lead_ms = get_i32(n, "min_lead_ms", 12, 1, 100, w)?;
        }
        if l.nack.lookback as u32 >= c.fec.seq_horizon {
            return fail(
                "link.nack.lookback",
                &format!("must be < fec.seq_horizon ({})", c.fec.seq_horizon),
            );
        }
        if let Some(rs) = section(r, "rung_stats")? {
            check_keys(rs, "link.rung_stats", &["half_life_samples"])?;
            l.ladder_cfg.rung_stats_half_life_samples =
                get_i32(rs, "half_life_samples", 600, 10, 100000, "link.rung_stats")?;
        }
    }
    let lc = &mut c.link.ladder_cfg;
    if lc.probe.max_util < 0.0 {
        lc.probe.max_util = lc.down_util;
    }
    if lc.s3_down_util < 0.0 {
        lc.s3_down_util = lc.down_util;
    }
    if !inline_key.is_empty() {
        let Some(k) = parse_key_hex(&inline_key) else {
            return fail("link.key", "not 32 hex characters");
        };
        c.link.key = k;
        c.link.key_is_default = false;
        c.link.key_source = "link.key".into();
    } else {
        match load_key_file(&c.link.key_file) {
            Ok((k, d, s)) => {
                c.link.key = k;
                c.link.key_is_default = d;
                c.link.key_source = s;
            }
            Err(e) => return fail("link.key_file", &e),
        }
    }
    if let Some((f, w)) = link_width_issue(&c.link, i32::from(c.radio.width)) {
        return fail(&f, &w);
    }
    if let Some(r) = section(j, "video")? {
        check_keys(r, "video", &["frame_gap_timeout_ms", "frame_gap_timeout_max_ms", "frame_lookahead"])?;
        c.video.frame_gap_timeout_ms = get_int(r, "frame_gap_timeout_ms", 50, 10, 1000, "video")? as u64;
        c.video.frame_gap_timeout_max_ms = get_int(r, "frame_gap_timeout_max_ms", 150, 0, 400, "video")? as u64;
        c.video.frame_lookahead = get_int(r, "frame_lookahead", 8, 2, 64, "video")? as u64;
    }
    if let Some(r) = section(j, "msp")? {
        check_keys(r, "msp", &["enable", "out", "symbol_size", "window"])?;
        c.msp.enable = get_bool(r, "enable", c.msp.enable, "msp")?;
        if let Some(o) = section(r, "out")? {
            check_keys(o, "msp.out", &["host", "port"])?;
            c.msp.out_host = get_str(o, "host", "127.0.0.1", "msp.out")?;
            c.msp.out_port = get_i32(o, "port", 14560, 1, 65535, "msp.out")?;
        }
        c.msp.symbol_size = get_int(r, "symbol_size", 1312, 16, 2048, "msp")? as usize;
        c.msp.window = get_int(r, "window", 16, 2, 255, "msp")? as usize;
    }
    if let Some(r) = section(j, "stats")? {
        check_keys(r, "stats", &["enable", "host", "port", "interval_ms", "out"])?;
        c.stats.enable = get_bool(r, "enable", c.stats.enable, "stats")?;
        c.stats.interval_ms = get_i32(r, "interval_ms", 500, 100, 10000, "stats")?;
        let legacy = r.contains_key("host") || r.contains_key("port");
        if let Some(v) = r.get("out") {
            if legacy {
                return fail("stats.out", "cannot be combined with stats.host / stats.port");
            }
            let Value::Array(a) = v else {
                return fail("stats.out", "not an array");
            };
            if a.is_empty() {
                return fail("stats.out", "must have at least one destination");
            }
            c.stats.out.clear();
            for e in a {
                let Value::Table(t) = e else {
                    return fail("stats.out[]", "not an object");
                };
                check_keys(t, "stats.out[]", &["host", "port"])?;
                if !t.contains_key("port") {
                    return fail("stats.out[].port", "missing");
                }
                c.stats.out.push(StatsOut {
                    host: get_str(t, "host", "127.0.0.1", "stats.out[]")?,
                    port: get_i32(t, "port", 8300, 1, 65535, "stats.out[]")?,
                });
            }
        } else {
            c.stats.out = vec![StatsOut {
                host: get_str(r, "host", "127.0.0.1", "stats")?,
                port: get_i32(r, "port", 8300, 1, 65535, "stats")?,
            }];
        }
    }
    if let Some(r) = section(j, "au_ring")? {
        check_keys(r, "au_ring", &["enable", "path", "socket", "slot_kb", "slot_count"])?;
        let a = &mut c.au_ring;
        a.enable = get_bool(r, "enable", a.enable, "au_ring")?;
        a.path = get_str(r, "path", "/dev/shm/mabur-au", "au_ring")?;
        a.socket = get_str(r, "socket", "/run/mabur-au.sock", "au_ring")?;
        a.slot_kb = get_i32(r, "slot_kb", 512, 64, 4096, "au_ring")?;
        a.slot_count = get_i32(r, "slot_count", 16, 4, 256, "au_ring")?;
    }
    if let Some(r) = section(j, "debug_log")? {
        check_keys(r, "debug_log", &["enable", "dir", "ctl_period_ms", "rung_period_s"])?;
        let d = &mut c.debug_log;
        d.enable = get_bool(r, "enable", d.enable, "debug_log")?;
        d.dir = get_str(r, "dir", "/media/dvr/log", "debug_log")?;
        d.ctl_period_ms = get_i32(r, "ctl_period_ms", 1000, 50, 60000, "debug_log")?;
        d.rung_period_s = get_i32(r, "rung_period_s", 10, 1, 600, "debug_log")?;
    }
    if let Some(r) = section(j, "mavlink")? {
        check_keys(r, "mavlink", &["enable", "bind", "port"])?;
        c.mavlink.enable = get_bool(r, "enable", c.mavlink.enable, "mavlink")?;
        c.mavlink.bind = get_str(r, "bind", "127.0.0.1", "mavlink")?;
        if !is_ipv4(&c.mavlink.bind) {
            return fail(
                "mavlink.bind",
                &format!("want a dotted IPv4 address, got \"{}\"", c.mavlink.bind),
            );
        }
        c.mavlink.port = get_i32(r, "port", 14550, 1, 65535, "mavlink")?;
    }
    Ok(c)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn repo(p: &str) -> std::path::PathBuf {
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..").join(p)
    }

    #[test]
    fn bundled_configs_load() {
        let c = load_config(&repo("configs/gs.toml"), None).unwrap();
        assert_eq!(c.radio.pin, Some(144));
        assert_eq!(c.radio.width, 40);
        let m = load_config(&repo("third_party/mabur/gs/bundle/maburgs.default.toml"), None).unwrap();
        assert!(m.link.ladder_cfg.ladder.len() >= 2);
        assert_eq!(m.uep_layers()[0].window, 128);
    }

    #[test]
    fn unknown_key_and_range_fail() {
        let e = parse_config_str("[video]\nframe_gap_timeout_ms = 5\n").unwrap_err();
        assert_eq!(e.0, "config: video.frame_gap_timeout_ms: out of range");
        let e = parse_config_str("[video]\nbogus = 1\n").unwrap_err();
        assert_eq!(e.0, "config: video.bogus: unknown key");
        let e = parse_config_str("[radio]\nwidth = 40\nchannels = [40, 165]\n").unwrap_err();
        assert!(e.0.contains("has no 40 MHz pair"), "{}", e.0);
    }

    #[test]
    fn ht40_table() {
        assert_eq!((ht40_offset(36), ht40_offset(40), ht40_offset(144)), (1, 2, 2));
        assert_eq!((ht40_offset(149), ht40_offset(165)), (1, 0));
        assert_eq!((ht40_pair_other(136), ht40_pair_other(165)), (132, 0));
    }
}
