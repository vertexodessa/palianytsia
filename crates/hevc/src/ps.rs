//! SPS/PPS subset for slice salvage (mabur `hevc_ps.cpp`). Streams outside
//! the encoder's feature subset parse as Unsupported and switch salvage off.
use crate::bits::{ceil_log2, unescape, BitReader};
use crate::nal::split_nals;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum PsStatus {
    #[default]
    None,
    Ok,
    Malformed,
    Unsupported,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Sps {
    pub sps_id: u32,
    pub chroma_format_idc: u32,
    pub width: u32,
    pub height: u32,
    pub log2_max_poc_lsb: u32,
    pub log2_min_cb: u32,
    pub log2_ctb: u32,
    pub sao: bool,
    pub tmvp: bool,
    pub num_st_rps: u32,
    pub st_num_delta_pocs: Vec<u32>,
    pub st_num_used: Vec<u32>,
    pub long_term: bool,
    pub num_lt_sps: u32,
    pub lt_used_sps: Vec<u8>,
}

impl Default for Sps {
    fn default() -> Self {
        Self {
            sps_id: 0,
            chroma_format_idc: 1,
            width: 0,
            height: 0,
            log2_max_poc_lsb: 4,
            log2_min_cb: 3,
            log2_ctb: 4,
            sao: false,
            tmvp: false,
            num_st_rps: 0,
            st_num_delta_pocs: Vec::new(),
            st_num_used: Vec::new(),
            long_term: false,
            num_lt_sps: 0,
            lt_used_sps: Vec::new(),
        }
    }
}

impl Sps {
    pub fn pic_w_ctbs(&self) -> u32 {
        (self.width + (1 << self.log2_ctb) - 1) >> self.log2_ctb
    }
    pub fn pic_h_ctbs(&self) -> u32 {
        (self.height + (1 << self.log2_ctb) - 1) >> self.log2_ctb
    }
    pub fn addr_bits(&self) -> u32 {
        ceil_log2(self.pic_w_ctbs() * self.pic_h_ctbs())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Pps {
    pub pps_id: u32,
    pub sps_id: u32,
    pub dependent_slices: bool,
    pub output_flag: bool,
    pub extra_bits: u32,
    pub cabac_init_present: bool,
    pub num_ref_idx_l0_default: u32,
    pub num_ref_idx_l1_default: u32,
    pub init_qp: i32,
    pub chroma_qp_offsets_present: bool,
    pub deblocking_override_enabled: bool,
    pub deblocking_disabled: bool,
    pub loop_filter_across_slices: bool,
    pub lists_modification: bool,
}

impl Default for Pps {
    fn default() -> Self {
        Self {
            pps_id: 0,
            sps_id: 0,
            dependent_slices: false,
            output_flag: false,
            extra_bits: 0,
            cabac_init_present: false,
            num_ref_idx_l0_default: 1,
            num_ref_idx_l1_default: 1,
            init_qp: 26,
            chroma_qp_offsets_present: false,
            deblocking_override_enabled: false,
            deblocking_disabled: false,
            loop_filter_across_slices: false,
            lists_modification: false,
        }
    }
}

fn skip_ptl(r: &mut BitReader, max_sub: u32) -> bool {
    r.u(32);
    r.u(32);
    r.u(32);
    let mut pp = vec![0u32; max_sub as usize];
    let mut lp = vec![0u32; max_sub as usize];
    for i in 0..max_sub as usize {
        pp[i] = r.u(1);
        lp[i] = r.u(1);
    }
    if max_sub > 0 {
        for _ in max_sub..8 {
            r.u(2);
        }
    }
    for i in 0..max_sub as usize {
        if pp[i] != 0 {
            r.u(32);
            r.u(32);
            r.u(24);
        }
        if lp[i] != 0 {
            r.u(8);
        }
    }
    !r.bad()
}

/// st_ref_pic_set(idx) (7.3.7). Returns (NumDeltaPocs, used_by_curr count).
pub fn parse_st_rps(
    r: &mut BitReader,
    idx: u32,
    num_sets: u32,
    num_delta_pocs: &[u32],
) -> Option<(u32, u32)> {
    let inter = idx != 0 && r.flag();
    let out = if inter {
        let delta_idx_m1 = if idx == num_sets { r.ue() } else { 0 };
        if delta_idx_m1 + 1 > idx || (idx - (delta_idx_m1 + 1)) as usize >= num_delta_pocs.len() {
            return None;
        }
        let rf = (idx - (delta_idx_m1 + 1)) as usize;
        r.u(1);
        r.ue();
        let (mut n, mut used) = (0, 0);
        for _ in 0..=num_delta_pocs[rf] {
            let u = r.u(1);
            let mut use_delta = 1;
            if u == 0 {
                use_delta = r.u(1);
            }
            if u != 0 || use_delta != 0 {
                n += 1;
            }
            if u != 0 {
                used += 1;
            }
        }
        (n, used)
    } else {
        let neg = r.ue();
        let pos = r.ue();
        if neg > 16 || pos > 16 {
            return None;
        }
        let mut used = 0;
        for _ in 0..neg + pos {
            r.ue();
            used += r.u(1);
        }
        (neg + pos, used)
    };
    (!r.bad()).then_some(out)
}

/// nal: from the 2-byte NAL header, escaped.
pub fn parse_sps(nal: &[u8]) -> Result<Sps, PsStatus> {
    if nal.len() < 3 {
        return Err(PsStatus::Malformed);
    }
    let rb = unescape(&nal[2..]);
    let mut r = BitReader::new(&rb);
    let mut s = Sps::default();
    r.u(4);
    let max_sub = r.u(3);
    r.u(1);
    if !skip_ptl(&mut r, max_sub) {
        return Err(PsStatus::Malformed);
    }
    s.sps_id = r.ue();
    s.chroma_format_idc = r.ue();
    if s.chroma_format_idc > 3 {
        return Err(PsStatus::Malformed);
    }
    if s.chroma_format_idc == 3 && r.flag() {
        return Err(PsStatus::Unsupported);
    }
    s.width = r.ue();
    s.height = r.ue();
    if r.flag() {
        r.ue();
        r.ue();
        r.ue();
        r.ue();
    }
    r.ue();
    r.ue();
    s.log2_max_poc_lsb = r.ue() + 4;
    let sub_info = r.u(1);
    let start = if sub_info != 0 { 0 } else { max_sub };
    for _ in start..=max_sub {
        r.ue();
        r.ue();
        r.ue();
    }
    s.log2_min_cb = r.ue() + 3;
    s.log2_ctb = s.log2_min_cb + r.ue();
    r.ue();
    r.ue();
    r.ue();
    r.ue();
    if r.flag() && r.flag() {
        return Err(PsStatus::Unsupported);
    }
    r.u(1);
    s.sao = r.flag();
    if r.flag() {
        r.u(4);
        r.u(4);
        r.ue();
        r.ue();
        r.u(1);
    }
    s.num_st_rps = r.ue();
    if s.num_st_rps > 64 {
        return Err(PsStatus::Malformed);
    }
    for i in 0..s.num_st_rps {
        let (ndp, used) = parse_st_rps(&mut r, i, s.num_st_rps, &s.st_num_delta_pocs)
            .ok_or(PsStatus::Malformed)?;
        s.st_num_delta_pocs.push(ndp);
        s.st_num_used.push(used);
    }
    s.long_term = r.flag();
    if s.long_term {
        s.num_lt_sps = r.ue();
        if s.num_lt_sps > 32 {
            return Err(PsStatus::Malformed);
        }
        for _ in 0..s.num_lt_sps {
            r.u(s.log2_max_poc_lsb);
            s.lt_used_sps.push(r.u(1) as u8);
        }
    }
    s.tmvp = r.flag();
    let m = (1u32 << s.log2_min_cb) - 1;
    if r.bad()
        || s.width == 0
        || s.height == 0
        || s.log2_ctb < 4
        || s.log2_ctb > 6
        || s.width & m != 0
        || s.height & m != 0
    {
        return Err(PsStatus::Malformed);
    }
    Ok(s)
}

pub fn parse_pps(nal: &[u8]) -> Result<Pps, PsStatus> {
    if nal.len() < 3 {
        return Err(PsStatus::Malformed);
    }
    let rb = unescape(&nal[2..]);
    let mut r = BitReader::new(&rb);
    let mut p = Pps {
        pps_id: r.ue(),
        sps_id: r.ue(),
        ..Default::default()
    };
    p.dependent_slices = r.flag();
    p.output_flag = r.flag();
    p.extra_bits = r.u(3);
    r.u(1);
    p.cabac_init_present = r.flag();
    p.num_ref_idx_l0_default = r.ue() + 1;
    p.num_ref_idx_l1_default = r.ue() + 1;
    p.init_qp = 26 + r.se();
    r.u(1);
    r.u(1);
    if r.flag() {
        r.ue();
    }
    r.se();
    r.se();
    p.chroma_qp_offsets_present = r.flag();
    let wp = r.flag();
    let wbp = r.flag();
    let tq = r.flag();
    let tiles = r.flag();
    let wpp = r.flag();
    if wp || wbp || tq || tiles || wpp {
        return Err(PsStatus::Unsupported);
    }
    p.loop_filter_across_slices = r.flag();
    if r.flag() {
        p.deblocking_override_enabled = r.flag();
        p.deblocking_disabled = r.flag();
        if !p.deblocking_disabled {
            r.se();
            r.se();
        }
    }
    if r.flag() {
        return Err(PsStatus::Unsupported);
    }
    p.lists_modification = r.flag();
    r.ue();
    if r.flag() {
        return Err(PsStatus::Unsupported);
    }
    if r.flag() {
        return Err(PsStatus::Unsupported);
    }
    if r.bad() {
        return Err(PsStatus::Malformed);
    }
    Ok(p)
}

/// Latest SPS/PPS seen in complete access units.
#[derive(Clone, Debug, Default)]
pub struct ParamTracker {
    sps: Sps,
    pps: Pps,
    sps_st: PsStatus,
    pps_st: PsStatus,
}

impl ParamTracker {
    pub fn feed(&mut self, au: &[u8]) {
        for v in split_nals(au) {
            let nal = &au[v.off..v.off + v.len];
            if v.ty == 33 {
                match parse_sps(nal) {
                    Ok(s) => {
                        self.sps = s;
                        self.sps_st = PsStatus::Ok;
                    }
                    Err(e) => self.sps_st = e,
                }
            } else if v.ty == 34 {
                match parse_pps(nal) {
                    Ok(p) => {
                        self.pps = p;
                        self.pps_st = PsStatus::Ok;
                    }
                    Err(e) => self.pps_st = e,
                }
            }
        }
    }
    pub fn usable(&self) -> bool {
        self.sps_st == PsStatus::Ok
            && self.pps_st == PsStatus::Ok
            && self.pps.sps_id == self.sps.sps_id
    }
    pub fn unsupported(&self) -> bool {
        self.sps_st == PsStatus::Unsupported || self.pps_st == PsStatus::Unsupported
    }
    pub fn sps(&self) -> &Sps {
        &self.sps
    }
    pub fn pps(&self) -> &Pps {
        &self.pps
    }
    pub fn reset(&mut self) {
        self.sps_st = PsStatus::None;
        self.pps_st = PsStatus::None;
    }
}
