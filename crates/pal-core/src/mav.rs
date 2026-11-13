//! MAVLink over the RC link, GS side: the T_MAV tunnel (mabur
//! `gs/src/mav_tunnel.cpp`), the extended-parameter client
//! (`gs/src/param_client.cpp`) and the parameter table it speaks
//! (`common/include/mabur/config_params.h`, `param_value.h`).
//!
//! The C++ classes send through callbacks; here every call that can send
//! returns or queues its output and the owner forwards it in the same order,
//! so the bytes and their order match.

use std::collections::VecDeque;

use pal_wire::mavlink::{
    self, Framing, Parser, MSG_HEARTBEAT, MSG_PARAM_EXT_ACK, MSG_PARAM_EXT_REQUEST_LIST,
    MSG_PARAM_EXT_REQUEST_READ, MSG_PARAM_EXT_SET, MSG_PARAM_EXT_VALUE, MSG_STATUSTEXT,
    PARAM_ACK_ACCEPTED, PARAM_ACK_FAILED, PARAM_ACK_IN_PROGRESS, PARAM_ACK_VALUE_UNSUPPORTED, STX,
};
use pal_wire::rc::{self, LinkKey, Mav, TagCtx, MAV_MAX_PAYLOAD};

// ---------------------------------------------------------------- table

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PType {
    Bool,
    Int32,
    Real32,
    Str,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Apply {
    Knob,
    Reprogram,
    Rebuild,
    /// Reports what the drone found (VENC_MODES); every set is refused there.
    ReadOnly,
}

pub struct Row {
    pub id: &'static str,
    pub section: &'static str,
    pub key: &'static str,
    pub ty: PType,
    pub apply: Apply,
    pub help: &'static str,
}

pub const TABLE: &[Row] = &[
    Row { id: "RADIO_WALL_MARGN", section: "radio", key: "wall_margin_db", ty: PType::Real32, apply: Apply::Reprogram, help: "Safety margin below the calibrated TX-power walls; raise to lower TX power" },
    Row { id: "RADIO_LDPC", section: "radio", key: "ldpc", ty: PType::Bool, apply: Apply::Reprogram, help: "LDPC coding on every frame; turn off only for a ground card that cannot decode it" },
    Row { id: "VENC_MODE", section: "venc", key: "fps", ty: PType::Str, apply: Apply::Rebuild, help: "Camera frame rate, encoder size and sensor readout, <fps>@<WxH>/binned|native; the rate and readout pick the sensor mode and the size must fit it" },
    Row { id: "VENC_SLICES", section: "venc", key: "slices", ty: PType::Int32, apply: Apply::Rebuild, help: "H.265 row slices on the link channel; 1 disables slice salvage" },
    Row { id: "ENC_BITRATE_MAX", section: "encoder", key: "bitrate_max_kbps", ty: PType::Int32, apply: Apply::Knob, help: "Bitrate ceiling, kbps" },
    Row { id: "ENC_AIRTIME", section: "encoder", key: "airtime_budget", ty: PType::Real32, apply: Apply::Knob, help: "Share of the link's real capacity the video may use" },
    Row { id: "REC_ENABLE", section: "record", key: "enable", ty: PType::Bool, apply: Apply::Rebuild, help: "Enable the onboard microSD recorder" },
    Row { id: "REC_BITRATE", section: "record", key: "bitrate_kbps", ty: PType::Int32, apply: Apply::Rebuild, help: "Constant bitrate of the onboard recording" },
    Row { id: "REC_FPS", section: "record", key: "fps", ty: PType::Int32, apply: Apply::Rebuild, help: "Onboard recording frame rate, at most the camera's fps" },
    Row { id: "REC_SIZE", section: "record", key: "size", ty: PType::Str, apply: Apply::Rebuild, help: "Onboard recording resolution, same aspect ratio as the camera's" },
    Row { id: "LOWPWR_ENABLE", section: "low_power", key: "enable", ty: PType::Bool, apply: Apply::Knob, help: "Lower bitrate and frame rate while disarmed to keep the VTX cool; needs msp.enable for arm state" },
    Row { id: "MSP_ENABLE", section: "msp", key: "enable", ty: PType::Bool, apply: Apply::Reprogram, help: "Flight-controller link for the OSD and arm state" },
    Row { id: "VENC_MIRROR", section: "venc", key: "mirror", ty: PType::Bool, apply: Apply::Rebuild, help: "Flip the picture left-right, for a camera mounted mirrored" },
    Row { id: "VENC_FLIP", section: "venc", key: "flip", ty: PType::Bool, apply: Apply::Rebuild, help: "Flip the picture upside down, for a camera mounted upside down" },
    Row { id: "RADIO_PWR_MODE", section: "radio", key: "power_mode", ty: PType::Str, apply: Apply::Reprogram, help: "TX power table: offset flies the calibrated walls, none the chip's own; none undoes a calibration" },
    Row { id: "VENC_MODES", section: "venc", key: "driver_modes", ty: PType::Str, apply: Apply::ReadOnly, help: "The camera driver's modes as VENC_MODE values at each mode's full size and top rate, comma-separated; empty until the camera starts" },
    Row { id: "VENC_MIN_IQP", section: "venc", key: "min_iqp", ty: PType::Int32, apply: Apply::Reprogram, help: "Keyframe quality floor, 0 to 51 (0 = the firmware's own); higher makes keyframes smaller and the moment after each one coarser" },
    Row { id: "VENC_GOP_S", section: "venc", key: "gop_s", ty: PType::Real32, apply: Apply::Rebuild, help: "Seconds between refresh starts, 0 or 0.5 to 10; 0 = none, the picture heals only when the ground station asks for a keyframe" },
    Row { id: "VENC_IREFRESH", section: "venc", key: "intra_refresh_frames", ty: PType::Int32, apply: Apply::Rebuild, help: "Frames the intra stripe of a refresh start is spread over, 0 to 255; 0 = a keyframe at each refresh start, 1 = one all-intra picture there; with temporal layering (venc.ref_base above 0) a longer stripe refreshes the top of the picture only; does nothing while VENC_GOP_S is 0" },
];
pub const COUNT: usize = 19;
const _: () = assert!(TABLE.len() == COUNT);
const MAX_ID_LEN: usize = 16;
const VENC_MODE_ROW: usize = 2;
const MODE_MAX_DIGITS: usize = 6;

fn mode_digits(b: &[u8], i: &mut usize) -> Option<i32> {
    let start = *i;
    let mut v: i32 = 0;
    while *i < b.len() && b[*i].is_ascii_digit() {
        if *i - start >= MODE_MAX_DIGITS {
            return None;
        }
        v = v * 10 + i32::from(b[*i] - b'0');
        *i += 1;
    }
    (*i != start).then_some(v)
}

/// `<fps>@<W>x<H>/binned` or `.../native`, each number at most 6 digits,
/// nothing else (mabur `cfgparam::parse_mode_text`). The last field is the
/// readout: true = binned.
pub fn parse_mode_text(t: &str) -> Option<(i32, i32, i32, bool)> {
    let b = t.as_bytes();
    let mut i = 0;
    let fps = mode_digits(b, &mut i)?;
    if b.get(i) != Some(&b'@') {
        return None;
    }
    i += 1;
    let w = mode_digits(b, &mut i)?;
    if b.get(i) != Some(&b'x') {
        return None;
    }
    i += 1;
    let h = mode_digits(b, &mut i)?;
    let binning = match &t[i..] {
        "/binned" => true,
        "/native" => false,
        _ => return None,
    };
    Some((fps, w, h, binning))
}

/// strncmp(id, id16, 16) == 0, id16 not necessarily NUL-terminated.
pub fn find(id16: &[u8]) -> Option<usize> {
    TABLE.iter().position(|r| {
        let a = r.id.as_bytes();
        for k in 0..MAX_ID_LEN {
            let x = a.get(k).copied().unwrap_or(0);
            let y = id16.get(k).copied().unwrap_or(0);
            if x != y {
                return false;
            }
            if x == 0 {
                return true;
            }
        }
        true
    })
}

pub fn mav_type(t: PType) -> u8 {
    match t {
        PType::Bool => 1,
        PType::Int32 => 6,
        PType::Real32 => 9,
        PType::Str => 11,
    }
}

pub fn type_name(t: PType) -> &'static str {
    match t {
        PType::Bool => "bool",
        PType::Int32 => "int32",
        PType::Real32 => "real32",
        PType::Str => "str",
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Bool(bool),
    Int32(i32),
    Real32(f32),
    Str(String),
}

impl Value {
    pub fn ty(&self) -> PType {
        match self {
            Value::Bool(_) => PType::Bool,
            Value::Int32(_) => PType::Int32,
            Value::Real32(_) => PType::Real32,
            Value::Str(_) => PType::Str,
        }
    }
}

pub fn decode_wire(idx: usize, mt: u8, wire: &[u8]) -> Option<Value> {
    let row = TABLE.get(idx)?;
    if mt != mav_type(row.ty) {
        return None;
    }
    match row.ty {
        PType::Bool => match wire[0] {
            0 => Some(Value::Bool(false)),
            1 => Some(Value::Bool(true)),
            _ => None,
        },
        PType::Int32 => Some(Value::Int32(i32::from_le_bytes(wire[..4].try_into().ok()?))),
        PType::Real32 => {
            let v = f32::from_le_bytes(wire[..4].try_into().ok()?);
            v.is_finite().then_some(Value::Real32(v))
        }
        PType::Str => {
            let len = wire[..128].iter().position(|&c| c == 0)?;
            if wire[..len].iter().any(|&c| !(0x20..=0x7e).contains(&c)) {
                return None;
            }
            let s = String::from_utf8(wire[..len].to_vec()).ok()?;
            if idx == VENC_MODE_ROW && parse_mode_text(&s).is_none() {
                return None;
            }
            Some(Value::Str(s))
        }
    }
}

pub fn encode_wire(v: &Value) -> [u8; 128] {
    let mut w = [0u8; 128];
    match v {
        Value::Bool(b) => w[0] = u8::from(*b),
        Value::Int32(i) => w[..4].copy_from_slice(&i.to_le_bytes()),
        Value::Real32(f) => w[..4].copy_from_slice(&f.to_le_bytes()),
        Value::Str(s) => {
            let n = s.len().min(127);
            w[..n].copy_from_slice(&s.as_bytes()[..n]);
        }
    }
    w
}

/// strtol/strtof-strict text parse (whole string, base 10).
pub fn parse_text(t: PType, text: &str) -> Option<Value> {
    match t {
        PType::Bool => match text.to_ascii_lowercase().as_str() {
            "1" | "true" => Some(Value::Bool(true)),
            "0" | "false" => Some(Value::Bool(false)),
            _ => None,
        },
        PType::Int32 => {
            // strtol skips leading whitespace and takes an optional sign.
            let s = text.trim_start_matches([' ', '\t', '\n', '\r', '\x0b', '\x0c']);
            if s.is_empty() || s.starts_with("++") || s.starts_with("+-") {
                return None;
            }
            s.parse::<i64>()
                .ok()
                .and_then(|v| i32::try_from(v).ok())
                .map(Value::Int32)
        }
        PType::Real32 => {
            let s = text.trim_start_matches([' ', '\t', '\n', '\r', '\x0b', '\x0c']);
            if s.is_empty() {
                return None;
            }
            let v: f32 = s.parse().ok()?;
            v.is_finite().then_some(Value::Real32(v))
        }
        PType::Str => {
            if text.is_empty()
                || text.len() > 127
                || text
                    .bytes()
                    .any(|c| !(0x20..=0x7e).contains(&c) || c == b'"' || c == b'\\')
            {
                return None;
            }
            Some(Value::Str(text.to_string()))
        }
    }
}

pub fn format_text(v: &Value) -> String {
    match v {
        Value::Bool(b) => if *b { "1" } else { "0" }.to_string(),
        Value::Int32(i) => i.to_string(),
        // Shortest round-trip, as float_text's %.6g..%.9g search lands on.
        Value::Real32(f) => format!("{f}"),
        Value::Str(s) => s.clone(),
    }
}

// ---------------------------------------------------------------- tunnel

#[derive(Clone, Copy, Debug, Default)]
pub struct MavTunnelStats {
    pub tx_frames: u64,
    pub tx_packets: u64,
    pub rx_frames: u64,
    pub rx_packets: u64,
    pub drop_unlinked: u64,
    pub drop_bad_packet: u64,
    pub drop_oversize: u64,
    pub rx_dup: u64,
    pub rx_uplink_heard: u64,
    /// Cal-lane frames sent (never in tx_frames/tx_packets).
    pub tx_cal_frames: u64,
    pub drop_cal_oversize: u64,
}

/// Feeds bytes through one parser channel keeping each packet's own wire
/// bytes, so a packet is forwarded exactly as it arrived.
fn split(
    p: &mut Parser,
    acc: &mut Vec<u8>,
    bytes: &[u8],
    mut on_packet: impl FnMut(&[u8]),
    mut on_bad: impl FnMut(),
) {
    for &c in bytes {
        let was_idle = p.is_idle();
        if was_idle {
            acc.clear();
        }
        acc.push(c);
        match p.feed(c) {
            Framing::Ok => {
                if acc[0] == STX {
                    on_packet(acc);
                } else {
                    on_bad();
                }
            }
            Framing::Incomplete => {
                if !was_idle && p.is_idle() {
                    on_bad();
                }
            }
            _ => on_bad(),
        }
    }
}

pub struct MavTunnel {
    key: LinkKey,
    ctx: TagCtx,
    vtx_seen: u32,
    counter: u32,
    have_rx: bool,
    rx_last: u32,
    queue: VecDeque<Vec<u8>>,
    /// The last cal frame's vtx_nonce, for the counter restart rule.
    cal_vtx_seen: u32,
    cal_counter: u32,
    cal_queue: VecDeque<(Vec<u8>, TagCtx)>,
    out_p: Parser,
    out_acc: Vec<u8>,
    in_p: Parser,
    in_acc: Vec<u8>,
    stats: MavTunnelStats,
}

impl MavTunnel {
    pub fn new(key: LinkKey) -> Self {
        Self {
            key,
            ctx: TagCtx::default(),
            vtx_seen: 0,
            counter: 0,
            have_rx: false,
            rx_last: 0,
            queue: VecDeque::new(),
            cal_vtx_seen: 0,
            cal_counter: 0,
            cal_queue: VecDeque::new(),
            out_p: Parser::default(),
            out_acc: Vec::new(),
            in_p: Parser::default(),
            in_acc: Vec::new(),
            stats: MavTunnelStats::default(),
        }
    }
    pub fn stats(&self) -> &MavTunnelStats {
        &self.stats
    }
    pub fn counter(&self) -> u32 {
        self.counter
    }

    pub fn set_session(&mut self, ctx: TagCtx) {
        if ctx.vtx_nonce != 0 && ctx.vtx_nonce != self.vtx_seen {
            self.vtx_seen = ctx.vtx_nonce;
            self.counter = 0;
            self.have_rx = false;
        }
        self.ctx = ctx;
    }

    pub fn push_outbound(&mut self, bytes: &[u8]) {
        let linked = self.ctx.vtx_nonce != 0;
        let (stats, queue) = (&mut self.stats, &mut self.queue);
        let bad = core::cell::Cell::new(0u64);
        split(
            &mut self.out_p,
            &mut self.out_acc,
            bytes,
            |pkt| {
                if !linked {
                    stats.drop_unlinked += 1;
                } else if pkt.len() > MAV_MAX_PAYLOAD {
                    stats.drop_oversize += 1;
                } else {
                    queue.push_back(pkt.to_vec());
                }
            },
            || bad.set(bad.get() + 1),
        );
        self.stats.drop_bad_packet += bad.get();
    }

    /// One calibration packet, tagged under the run's frozen pair. The cal
    /// lane never shares a frame with a live packet: the drone hands a
    /// mixed frame whole to its MAVLink component, which drops the cal ids.
    /// Unlike the live lane it goes out whether or not a session is live
    /// (every sweep outlasts the link's failsafe).
    pub fn push_cal(&mut self, packet: Vec<u8>, run_ctx: TagCtx) {
        if packet.len() > MAV_MAX_PAYLOAD {
            self.stats.drop_cal_oversize += 1;
            return;
        }
        self.cal_queue.push_back((packet, run_ctx));
    }

    /// Packs the cal queue into T_MAV frames, one pair per frame. The
    /// counter restarts only on a new vtx_nonce: the drone checks it
    /// against its per-pair high-water mark, so a second run on the same
    /// pair must keep counting. Call before `flush`.
    pub fn flush_cal(&mut self) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        while let Some((_, run_ctx)) = self.cal_queue.front() {
            let run_ctx = *run_ctx;
            if run_ctx.vtx_nonce != self.cal_vtx_seen {
                self.cal_vtx_seen = run_ctx.vtx_nonce;
                self.cal_counter = 0;
            }
            let mut m = Mav::default();
            while let Some((p, c)) = self.cal_queue.front() {
                if c.vrx_nonce != run_ctx.vrx_nonce
                    || c.vtx_nonce != run_ctx.vtx_nonce
                    || m.payload.len() + p.len() > MAV_MAX_PAYLOAD
                {
                    break;
                }
                m.payload.extend_from_slice(p);
                self.cal_queue.pop_front();
            }
            self.cal_counter = self.cal_counter.wrapping_add(1);
            m.counter = self.cal_counter;
            let mut tag = run_ctx;
            tag.seq32 = self.cal_counter;
            out.push(rc::pack_mav(&m, true, &self.key, &tag));
            self.stats.tx_cal_frames += 1;
        }
        out
    }

    /// Packs the queue greedily into T_MAV frames; returns them in send order.
    pub fn flush(&mut self) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        if self.ctx.vtx_nonce == 0 {
            self.stats.drop_unlinked += self.queue.len() as u64;
            self.queue.clear();
            return out;
        }
        while !self.queue.is_empty() {
            let mut m = Mav::default();
            let mut packets = 0u64;
            while let Some(f) = self.queue.front() {
                if m.payload.len() + f.len() > MAV_MAX_PAYLOAD {
                    break;
                }
                m.payload.extend_from_slice(f);
                self.queue.pop_front();
                packets += 1;
            }
            self.counter = self.counter.wrapping_add(1);
            m.counter = self.counter;
            let mut tag = self.ctx;
            tag.seq32 = self.counter;
            out.push(rc::pack_mav(&m, true, &self.key, &tag));
            self.stats.tx_frames += 1;
            self.stats.tx_packets += packets;
        }
        out
    }

    /// A T_MAV body from the drone; returns the whole packets to deliver.
    pub fn on_rc_frame(&mut self, body: &[u8]) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        if rc::parse_mav(body, true).is_some() {
            self.stats.rx_uplink_heard += 1;
            return out;
        }
        let Some(m) = rc::parse_mav(body, false) else {
            self.stats.drop_bad_packet += 1;
            return out;
        };
        if self.have_rx && m.counter == self.rx_last {
            self.stats.rx_dup += 1;
            return out;
        }
        self.have_rx = true;
        self.rx_last = m.counter;
        self.stats.rx_frames += 1;
        let mut bad = 0u64;
        split(
            &mut self.in_p,
            &mut self.in_acc,
            &m.payload,
            |pkt| out.push(pkt.to_vec()),
            || bad += 1,
        );
        self.stats.rx_packets += out.len() as u64;
        self.stats.drop_bad_packet += bad;
        if !self.in_p.is_idle() {
            self.stats.drop_bad_packet += 1;
            self.in_p.reset();
        }
        self.in_acc.clear();
        out
    }
}

// ---------------------------------------------------------------- params

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RowState {
    Idle,
    Sending,
    InProgress,
    Accepted,
    Failed,
    Unsupported,
    Timeout,
}

impl RowState {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Idle => "idle",
            Self::Sending => "sending",
            Self::InProgress => "in_progress",
            Self::Accepted => "accepted",
            Self::Failed => "failed",
            Self::Unsupported => "unsupported",
            Self::Timeout => "timeout",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ListState {
    None,
    Pending,
    Complete,
    Partial,
}

#[derive(Clone, Debug)]
pub struct ParamRow {
    pub value: Option<Value>,
    pub st: RowState,
    pub status: String,
    pub result_ms: u64,
}

impl Default for ParamRow {
    fn default() -> Self {
        Self {
            value: None,
            st: RowState::Idle,
            status: String::new(),
            result_ms: 0,
        }
    }
}

#[derive(Clone, Debug)]
pub struct ParamClientState {
    pub linked: bool,
    pub alive_ms: u64,
    pub table_mismatch: bool,
    pub list: ListState,
    pub pending_row: Option<usize>,
    pub rows: [ParamRow; COUNT],
}

impl Default for ParamClientState {
    fn default() -> Self {
        Self {
            linked: false,
            alive_ms: 0,
            table_mismatch: false,
            list: ListState::None,
            pending_row: None,
            rows: Default::default(),
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ParamClientStats {
    pub tx: u64,
    pub rx: u64,
    pub drop_unknown: u64,
    pub drop_bad: u64,
    pub timeouts: u64,
}

pub const LIST_RETRY_MS: u64 = 2000;
pub const SET_RETRY_MS: u64 = 2000;
pub const FINAL_MS: u64 = 15000;
pub const STATUS_MS: u64 = 1500;
pub const ALIVE_MS: u64 = 3000;
pub const LIST_ROUNDS: i32 = 3;
pub const SET_RETRIES: i32 = 3;
const DRONE_SYS: u8 = 1;
const DRONE_COMP: u8 = 191;

pub struct ParamClient {
    sysid: u8,
    compid: u8,
    /// MAVLINK_COMM_0's current_tx_seq: the C++ packs through the
    /// channel-0 status, which nothing else in the GS process uses.
    tx_seq: u8,
    out: Vec<Vec<u8>>,
    state: ParamClientState,
    stats: ParamClientStats,
    seq: u32,
    alive_cached: bool,
    list_last_req_ms: u64,
    list_rounds: i32,
    fresh: [bool; COUNT],
    set_value: Option<Value>,
    set_send_ms: u64,
    set_resends: i32,
    set_in_progress: bool,
    /// The set changes the row's held value, so a re-read showing `set_value`
    /// proves the apply. Setting the value already held proves nothing: that
    /// set ends only on the drone's own ACCEPTED or FAILED (mabur).
    set_reread_proves: bool,
    set_deadline_ms: u64,
    text_id: u16,
    text_next_seq: u8,
    text_buf: String,
}

impl Default for ParamClient {
    fn default() -> Self {
        Self::new(255, 190)
    }
}

impl ParamClient {
    pub fn new(sysid: u8, compid: u8) -> Self {
        Self {
            sysid,
            compid,
            tx_seq: 0,
            out: Vec::new(),
            state: ParamClientState::default(),
            stats: ParamClientStats::default(),
            seq: 0,
            alive_cached: false,
            list_last_req_ms: 0,
            list_rounds: 0,
            fresh: [false; COUNT],
            set_value: None,
            set_send_ms: 0,
            set_resends: 0,
            set_in_progress: false,
            set_reread_proves: false,
            set_deadline_ms: 0,
            text_id: 0,
            text_next_seq: 0,
            text_buf: String::new(),
        }
    }
    pub fn state(&self) -> &ParamClientState {
        &self.state
    }
    pub fn stats(&self) -> &ParamClientStats {
        &self.stats
    }
    pub fn seq(&self) -> u32 {
        self.seq
    }
    /// Packets sent since the last call, in order (the C++ send callback).
    pub fn take_out(&mut self) -> Vec<Vec<u8>> {
        core::mem::take(&mut self.out)
    }

    fn emit(&mut self, msgid: u32, payload: &[u8]) {
        let pkt = mavlink::pack_v2(msgid, self.sysid, self.compid, self.tx_seq, payload);
        self.tx_seq = self.tx_seq.wrapping_add(1);
        self.out.push(pkt);
        self.stats.tx += 1;
    }
    fn send_list_request(&mut self) {
        self.emit(MSG_PARAM_EXT_REQUEST_LIST, &[DRONE_SYS, DRONE_COMP]);
    }
    fn send_read_request(&mut self, idx: usize) {
        let mut p = [0u8; 20];
        p[..2].copy_from_slice(&(idx as i16).to_le_bytes());
        p[2] = DRONE_SYS;
        p[3] = DRONE_COMP;
        self.emit(MSG_PARAM_EXT_REQUEST_READ, &p);
    }
    fn send_set_packet(&mut self, idx: usize, v: &Value) {
        let mut p = [0u8; 147];
        p[0] = DRONE_SYS;
        p[1] = DRONE_COMP;
        let id = TABLE[idx].id.as_bytes();
        p[2..2 + id.len()].copy_from_slice(id);
        p[18..146].copy_from_slice(&encode_wire(v));
        p[146] = mav_type(TABLE[idx].ty);
        self.emit(MSG_PARAM_EXT_SET, &p);
    }
    fn begin_list(&mut self, now: u64) {
        self.state.list = ListState::Pending;
        self.fresh = [false; COUNT];
        self.list_last_req_ms = now;
        self.list_rounds = 0;
        self.send_list_request();
    }
    fn refuse(&mut self, row: usize, why: &str, now: u64) {
        if let Some(r) = self.state.rows.get_mut(row) {
            r.st = RowState::Failed;
            r.status = why.to_string();
            r.result_ms = now;
        }
    }
    fn timeout_pending(&mut self, now: u64) {
        let idx = self.state.pending_row.expect("pending");
        self.state.rows[idx].st = RowState::Timeout;
        self.state.rows[idx].result_ms = now;
        self.state.pending_row = None;
        self.set_in_progress = false;
        self.set_resends = 0;
        self.stats.timeouts += 1;
        self.send_read_request(idx);
    }

    pub fn set_linked(&mut self, linked: bool, now: u64) {
        if linked == self.state.linked {
            return;
        }
        // The pending row's "link lost" is overwritten by the full clear
        // just below, as in the C++.
        self.state = ParamClientState {
            linked,
            ..ParamClientState::default()
        };
        self.alive_cached = false;
        self.fresh = [false; COUNT];
        self.list_rounds = 0;
        self.list_last_req_ms = now;
        if linked {
            self.state.list = ListState::Pending;
            self.send_list_request();
        }
        self.seq += 1;
    }

    pub fn alive(&self, now: u64) -> bool {
        self.state.alive_ms != 0 && now.wrapping_sub(self.state.alive_ms) <= ALIVE_MS
    }

    pub fn on_packet(&mut self, p: &[u8], now: u64) {
        let mut parser = Parser::default();
        let mut ok = false;
        for &b in p {
            if parser.feed(b) == Framing::Ok {
                ok = true;
            }
        }
        if !ok {
            self.stats.drop_bad += 1;
            return;
        }
        self.stats.rx += 1;
        let m = parser.message().clone();
        if m.sysid != DRONE_SYS || m.compid != DRONE_COMP {
            return;
        }
        let pl = &m.payload;
        let mut changed = false;
        match m.msgid {
            MSG_HEARTBEAT => {
                let was = self.alive_cached;
                self.state.alive_ms = now;
                self.alive_cached = true;
                changed = !was;
            }
            MSG_PARAM_EXT_VALUE => {
                let count = u16::from_le_bytes([pl[0], pl[1]]);
                if usize::from(count) != COUNT && !self.state.table_mismatch {
                    self.state.table_mismatch = true;
                    changed = true;
                }
                match find(&pl[4..20]) {
                    None => self.stats.drop_unknown += 1,
                    Some(idx) => match decode_wire(idx, pl[148], &pl[20..148]) {
                        None => self.stats.drop_bad += 1,
                        Some(v) => {
                            // The drone sends the final ACCEPTED once; when
                            // the air drops it, the re-read tick() sends
                            // while InProgress finds the published value.
                            let done = self.state.pending_row == Some(idx)
                                && self.set_in_progress
                                && self.set_reread_proves
                                && self.set_value.as_ref() == Some(&v);
                            self.state.rows[idx].value = Some(v);
                            self.fresh[idx] = true;
                            changed = true;
                            if done {
                                self.accept_pending(idx, now);
                            }
                            if matches!(self.state.list, ListState::Pending | ListState::Partial)
                                && self.fresh.iter().all(|&f| f)
                            {
                                self.state.list = ListState::Complete;
                            }
                        }
                    },
                }
            }
            MSG_PARAM_EXT_ACK => match find(&pl[0..16]) {
                None => self.stats.drop_unknown += 1,
                Some(idx) => match decode_wire(idx, pl[144], &pl[16..144]) {
                    None => self.stats.drop_bad += 1,
                    Some(v) => {
                        self.state.rows[idx].value = Some(v);
                        self.fresh[idx] = true;
                        changed = true;
                        if self.state.pending_row == Some(idx) {
                            self.on_ack(idx, pl[145], now);
                        }
                    }
                },
            },
            MSG_STATUSTEXT => changed = self.on_statustext(pl, now),
            _ => {}
        }
        if changed {
            self.seq += 1;
        }
    }

    fn on_ack(&mut self, idx: usize, result: u8, now: u64) {
        let terminal = |s: &mut Self, st: RowState| {
            s.state.rows[idx].st = st;
            s.state.rows[idx].result_ms = now;
            s.state.pending_row = None;
            s.set_in_progress = false;
            s.set_resends = 0;
        };
        match result {
            PARAM_ACK_IN_PROGRESS => {
                if !self.set_in_progress {
                    self.set_in_progress = true;
                    self.set_deadline_ms = now + FINAL_MS;
                }
                self.state.rows[idx].st = RowState::InProgress;
            }
            PARAM_ACK_ACCEPTED => self.accept_pending(idx, now),
            PARAM_ACK_FAILED => terminal(self, RowState::Failed),
            PARAM_ACK_VALUE_UNSUPPORTED => terminal(self, RowState::Unsupported),
            _ => {}
        }
    }

    fn accept_pending(&mut self, idx: usize, now: u64) {
        let r = &mut self.state.rows[idx];
        r.st = RowState::Accepted;
        r.result_ms = now;
        self.state.pending_row = None;
        self.set_in_progress = false;
        self.set_resends = 0;
        if TABLE[idx].apply == Apply::Rebuild {
            self.begin_list(now);
        }
    }

    fn on_statustext(&mut self, pl: &[u8], now: u64) -> bool {
        let raw = &pl[1..51];
        let len = raw.iter().position(|&c| c == 0).unwrap_or(raw.len());
        let mut text = String::from_utf8_lossy(&raw[..len]).into_owned();
        let id = u16::from_le_bytes([pl[51], pl[52]]);
        let chunk_seq = pl[53];
        if id != 0 {
            if chunk_seq == 0 {
                self.text_id = id;
                self.text_next_seq = 0;
                self.text_buf.clear();
            }
            if id != self.text_id || chunk_seq != self.text_next_seq {
                self.text_id = 0;
                self.text_buf.clear();
                return false;
            }
            self.text_buf.push_str(&text);
            self.text_next_seq = self.text_next_seq.wrapping_add(1);
            if len == raw.len() {
                return false;
            }
            text = core::mem::take(&mut self.text_buf);
            self.text_id = 0;
        }
        if let Some(p) = self.state.pending_row {
            self.state.rows[p].status = text;
            return true;
        }
        let mut best: Option<(usize, u64)> = None;
        for (i, row) in self.state.rows.iter().enumerate() {
            if !matches!(
                row.st,
                RowState::Failed | RowState::Unsupported | RowState::Accepted
            ) {
                continue;
            }
            if now < row.result_ms || now - row.result_ms > STATUS_MS {
                continue;
            }
            if best.is_none_or(|(_, ms)| row.result_ms > ms) {
                best = Some((i, row.result_ms));
            }
        }
        if let Some((i, _)) = best {
            self.state.rows[i].status = text;
            return true;
        }
        false
    }

    pub fn tick(&mut self, now: u64) {
        let mut changed = false;
        if self.alive_cached && !self.alive(now) {
            self.alive_cached = false;
            changed = true;
        }
        if let Some(p) = self.state.pending_row {
            if self.set_in_progress {
                if now >= self.set_deadline_ms {
                    self.timeout_pending(now);
                    changed = true;
                } else if now.wrapping_sub(self.set_send_ms) >= SET_RETRY_MS {
                    // The drone answers a read with the file as published,
                    // so the requested value there means the apply succeeded.
                    self.send_read_request(p);
                    self.set_send_ms = now;
                }
            } else if now.wrapping_sub(self.set_send_ms) >= SET_RETRY_MS {
                if self.set_resends < SET_RETRIES {
                    let v = self.set_value.clone().expect("set value");
                    self.send_set_packet(p, &v);
                    self.set_resends += 1;
                    self.set_send_ms = now;
                } else {
                    self.timeout_pending(now);
                    changed = true;
                }
            }
        }
        if self.state.list == ListState::Pending
            && now.wrapping_sub(self.list_last_req_ms) >= LIST_RETRY_MS
        {
            if self.list_rounds >= LIST_ROUNDS {
                self.state.list = ListState::Partial;
                changed = true;
            } else {
                let mut any = false;
                for i in 0..COUNT {
                    if !self.fresh[i] {
                        self.send_read_request(i);
                        any = true;
                    }
                }
                self.list_last_req_ms = now;
                if any {
                    self.list_rounds += 1;
                } else {
                    self.state.list = ListState::Complete;
                    changed = true;
                }
            }
        }
        if changed {
            self.seq += 1;
        }
    }

    pub fn refresh(&mut self, now: u64) {
        if !self.state.linked {
            return;
        }
        let changed = self.state.list != ListState::Pending;
        self.begin_list(now);
        if changed {
            self.seq += 1;
        }
    }

    pub fn set(&mut self, row: usize, text: &str, now: u64) -> Result<(), String> {
        let reason = if !self.state.linked {
            "not linked".to_string()
        } else if self.state.pending_row.is_some() {
            "apply in progress".to_string()
        } else if row >= COUNT {
            "no such parameter".to_string()
        } else {
            let ty = TABLE[row].ty;
            match parse_text(ty, text) {
                None => format!("bad value for {}", type_name(ty)),
                Some(v) => {
                    self.set_reread_proves = self.state.rows[row].value.as_ref() != Some(&v);
                    self.send_set_packet(row, &v);
                    let r = &mut self.state.rows[row];
                    r.st = RowState::Sending;
                    r.status.clear();
                    r.result_ms = 0;
                    self.state.pending_row = Some(row);
                    self.set_value = Some(v);
                    self.set_send_ms = now;
                    self.set_resends = 0;
                    self.set_in_progress = false;
                    self.seq += 1;
                    return Ok(());
                }
            }
        };
        self.refuse(row, &reason, now);
        if row < COUNT {
            self.seq += 1;
        }
        Err(reason)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn link_sends_list_request_then_retries() {
        let mut pc = ParamClient::default();
        pc.set_linked(true, 1000);
        let o = pc.take_out();
        assert_eq!(o.len(), 1);
        assert_eq!(o[0][7], 0x41); // 321 & 0xFF
        pc.tick(3000);
        assert_eq!(pc.take_out().len(), COUNT);
        assert_eq!(pc.state().list, ListState::Pending);
    }

    #[test]
    fn tunnel_passes_packets_verbatim() {
        let mut pc = ParamClient::default();
        pc.set_linked(true, 0);
        let pkt = pc.take_out().remove(0);
        let mut t = MavTunnel::new(LinkKey::default());
        t.push_outbound(&pkt);
        assert_eq!(t.stats().drop_unlinked, 1);
        t.set_session(TagCtx {
            vrx_nonce: 1,
            vtx_nonce: 2,
            seq32: 0,
        });
        t.push_outbound(&pkt[..5]);
        t.push_outbound(&pkt[5..]);
        let f = t.flush();
        assert_eq!(f.len(), 1);
        let m = rc::parse_mav(&f[0], true).expect("tagged");
        assert_eq!(m.counter, 1);
        assert_eq!(m.payload, pkt);
    }

    fn drone_pkt(msgid: u32, payload: &[u8]) -> Vec<u8> {
        mavlink::pack_v2(msgid, DRONE_SYS, DRONE_COMP, 0, payload)
    }
    fn value_pkt(idx: usize, text: &str) -> Vec<u8> {
        let mut p = [0u8; 149];
        p[..2].copy_from_slice(&(COUNT as u16).to_le_bytes());
        p[2..4].copy_from_slice(&(idx as u16).to_le_bytes());
        let id = TABLE[idx].id.as_bytes();
        p[4..4 + id.len()].copy_from_slice(id);
        let v = parse_text(TABLE[idx].ty, text).expect("value");
        p[20..148].copy_from_slice(&encode_wire(&v));
        p[148] = mav_type(TABLE[idx].ty);
        drone_pkt(MSG_PARAM_EXT_VALUE, &p)
    }
    fn ack_pkt(idx: usize, text: &str, result: u8) -> Vec<u8> {
        let mut p = [0u8; 146];
        let id = TABLE[idx].id.as_bytes();
        p[..id.len()].copy_from_slice(id);
        let v = parse_text(TABLE[idx].ty, text).expect("value");
        p[16..144].copy_from_slice(&encode_wire(&v));
        p[144] = mav_type(TABLE[idx].ty);
        p[145] = result;
        drone_pkt(MSG_PARAM_EXT_ACK, &p)
    }

    /// Mirrors the C++ lost_accepted_is_recovered_by_the_reread.
    #[test]
    fn lost_accepted_is_recovered_by_the_reread() {
        let row = find(b"VENC_MODE\0\0\0\0\0\0\0").expect("row");
        let mut pc = ParamClient::default();
        pc.set_linked(true, 0);
        pc.on_packet(&value_pkt(row, "90@1920x1080/binned"), 10);
        // Let the list run out its re-read rounds so only the set talks.
        for t in [2000, 4000, 6000, 8000] {
            pc.tick(t);
        }
        assert_eq!(pc.state().list, ListState::Partial);
        let t0 = 10_000;
        pc.set(row, "120@1280x720/binned", t0).expect("set");
        pc.on_packet(
            &ack_pkt(row, "90@1920x1080/binned", PARAM_ACK_IN_PROGRESS),
            t0 + 100,
        );
        pc.take_out();
        pc.tick(t0 + 1999);
        assert!(pc.take_out().is_empty());
        pc.tick(t0 + 2000);
        let o = pc.take_out();
        assert_eq!(o.len(), 1);
        assert_eq!(o[0][7], 0x40); // PARAM_EXT_REQUEST_READ = 320
        pc.on_packet(&value_pkt(row, "90@1920x1080/binned"), t0 + 2050);
        assert_eq!(pc.state().rows[row].st, RowState::InProgress);
        pc.tick(t0 + 4000);
        assert_eq!(pc.take_out().len(), 1);
        pc.on_packet(&value_pkt(row, "120@1280x720/binned"), t0 + 4050);
        assert_eq!(pc.state().rows[row].st, RowState::Accepted);
        assert_eq!(pc.state().rows[row].result_ms, t0 + 4050);
        assert_eq!(pc.state().pending_row, None);
        assert_eq!(pc.state().list, ListState::Pending);
        assert_eq!(pc.stats().timeouts, 0);
    }

    /// Mirrors the C++ reread_of_an_unchanged_value_waits_for_the_ack.
    #[test]
    fn reread_of_an_unchanged_value_waits_for_the_ack() {
        let row = find(b"VENC_MODE\0\0\0\0\0\0\0").expect("row");
        let mut pc = ParamClient::default();
        pc.set_linked(true, 0);
        pc.on_packet(&value_pkt(row, "90@1920x1080/binned"), 10);
        for t in [2000, 4000, 6000, 8000] {
            pc.tick(t);
        }
        let t0 = 10_000;
        pc.set(row, "90@1920x1080/binned", t0).expect("set");
        pc.on_packet(
            &ack_pkt(row, "90@1920x1080/binned", PARAM_ACK_IN_PROGRESS),
            t0 + 100,
        );
        pc.take_out();
        pc.tick(t0 + 2000);
        assert_eq!(pc.take_out().len(), 1); // the re-read still goes out
        pc.on_packet(&value_pkt(row, "90@1920x1080/binned"), t0 + 2050);
        assert_eq!(pc.state().rows[row].st, RowState::InProgress);
        assert_eq!(pc.state().pending_row, Some(row));
        pc.on_packet(
            &ack_pkt(row, "90@1920x1080/binned", PARAM_ACK_FAILED),
            t0 + 2100,
        );
        assert_eq!(pc.state().rows[row].st, RowState::Failed);
        assert_eq!(pc.state().pending_row, None);
    }

    #[test]
    fn mode_text() {
        assert_eq!(
            parse_mode_text("120@1920x1080/binned"),
            Some((120, 1920, 1080, true))
        );
        assert_eq!(
            parse_mode_text("60@2944x1656/native"),
            Some((60, 2944, 1656, false))
        );
        assert_eq!(parse_mode_text("120@1920x1080"), None); // the old shape
        assert_eq!(parse_mode_text("120@1920x1080/"), None);
        assert_eq!(parse_mode_text("120@1920x1080/Binned"), None);
        assert_eq!(parse_mode_text("120@1920x1080/binned "), None);
        assert_eq!(parse_mode_text("120@1920x"), None);
        assert_eq!(parse_mode_text("1234567@1x1"), None);
    }

    #[test]
    fn cal_lane_keeps_its_own_counter_and_never_mixes() {
        let mut t = MavTunnel::new(LinkKey::default());
        let ctx = |vtx| TagCtx {
            vrx_nonce: 7,
            vtx_nonce: vtx,
            ..TagCtx::default()
        };
        // Unlinked: the cal lane still goes out.
        t.push_cal(vec![1, 2, 3], ctx(5));
        t.push_cal(vec![4, 5], ctx(5));
        let f = t.flush_cal();
        assert_eq!(f.len(), 1);
        let m = rc::parse_mav(&f[0], true).unwrap();
        assert_eq!(
            (m.counter, m.payload.as_slice()),
            (1, &[1u8, 2, 3, 4, 5][..])
        );
        // Same pair: a second run keeps counting.
        t.push_cal(vec![9], ctx(5));
        assert_eq!(rc::parse_mav(&t.flush_cal()[0], true).unwrap().counter, 2);
        // A new pair restarts at 1, and splits from the old pair's packets.
        t.push_cal(vec![1], ctx(5));
        t.push_cal(vec![2], ctx(6));
        let f = t.flush_cal();
        assert_eq!(f.len(), 2);
        assert_eq!(rc::parse_mav(&f[0], true).unwrap().counter, 3);
        assert_eq!(rc::parse_mav(&f[1], true).unwrap().counter, 1);
        // The live lane's counter is untouched.
        assert_eq!(t.counter(), 0);
        assert_eq!(t.stats().tx_cal_frames, 4);
        assert_eq!(t.stats().tx_frames, 0);
        t.push_cal(vec![0; MAV_MAX_PAYLOAD + 1], ctx(6));
        assert_eq!(t.stats().drop_cal_oversize, 1);
        assert!(t.flush_cal().is_empty());
    }
}
