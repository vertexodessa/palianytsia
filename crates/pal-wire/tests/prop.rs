//! Property tests of every radio-facing parser: arbitrary bytes never panic,
//! a valid frame bent by bit flips, truncation, extension and length-field
//! edits never panics, and parse(pack(x)) == x wherever a packer exists.
//!
//! Run in the test profile, where overflow checks are on: a wrapping
//! subtraction on hostile input fails here although release builds would
//! carry on with a wrapped value. Deterministic (fixed seed, no failure
//! files); `PAL_PROP_SCALE=N` multiplies the case counts and
//! `PAL_PROP_SEED=S` picks another seed for a long local run.
use pal_wire::crc16::crc16_ccitt;
use pal_wire::rc::{self, LinkKey, TagCtx};
use pal_wire::{cal, dot11, frame_wire, ht40, mavlink, probe, profile, sbi, sw};
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

/// One edit of a valid frame.
#[derive(Clone, Debug)]
enum Edit {
    Flip(usize, u8),
    Set(usize, u8),
    Truncate(usize),
    Extend(Vec<u8>),
    Insert(usize, u8),
    Remove(usize),
}

fn edit() -> impl Strategy<Value = Edit> {
    prop_oneof![
        (any::<usize>(), 0u8..8).prop_map(|(p, b)| Edit::Flip(p, b)),
        (any::<usize>(), any::<u8>()).prop_map(|(p, v)| Edit::Set(p, v)),
        any::<usize>().prop_map(Edit::Truncate),
        proptest::collection::vec(any::<u8>(), 1..64).prop_map(Edit::Extend),
        (any::<usize>(), any::<u8>()).prop_map(|(p, v)| Edit::Insert(p, v)),
        any::<usize>().prop_map(Edit::Remove),
    ]
}

fn edits() -> impl Strategy<Value = Vec<Edit>> {
    proptest::collection::vec(edit(), 1..6)
}

fn apply(mut b: Vec<u8>, es: &[Edit]) -> Vec<u8> {
    for e in es {
        let n = b.len();
        match e {
            Edit::Flip(p, bit) if n > 0 => b[p % n] ^= 1 << bit,
            Edit::Set(p, v) if n > 0 => b[p % n] = *v,
            Edit::Truncate(p) => b.truncate(p % (n + 1)),
            Edit::Extend(x) => b.extend_from_slice(x),
            Edit::Insert(p, v) => b.insert(p % (n + 1), *v),
            Edit::Remove(p) if n > 0 => {
                b.remove(p % n);
            }
            _ => {}
        }
    }
    b
}

/// An RC frame's CRC sits in its last two bytes; rewriting it lets a
/// mutated body past the CRC gate into the field parse, as a transmitter
/// that computes the CRC would.
fn recrc(mut b: Vec<u8>) -> Vec<u8> {
    if b.len() >= 2 {
        let n = b.len() - 2;
        let c = crc16_ccitt(&b[..n]);
        b[n..].copy_from_slice(&c.to_le_bytes());
    }
    b
}

fn bytes(max: usize) -> impl Strategy<Value = Vec<u8>> {
    proptest::collection::vec(any::<u8>(), 0..max)
}

fn key() -> impl Strategy<Value = LinkKey> {
    any::<[u8; 16]>()
}

fn ctx() -> impl Strategy<Value = TagCtx> {
    (any::<u32>(), any::<u32>(), any::<u32>()).prop_map(|(a, b, c)| TagCtx {
        vrx_nonce: a,
        vtx_nonce: b,
        seq32: c,
    })
}

fn rcf() -> impl Strategy<Value = rc::Rcf> {
    (
        any::<u16>(),
        any::<u8>(),
        5u8..=200,
        5u8..=200,
        any::<[u8; 5]>(),
        any::<i16>(),
    )
        .prop_map(|(seq, profile, ob, oe, m, snr_off)| rc::Rcf {
            seq,
            profile,
            fec_overhead_base: f64::from(ob) / 100.0,
            fec_overhead_enh: f64::from(oe) / 100.0,
            probe_profile: m[0],
            hop_ch: m[1],
            hop_epoch: m[2],
            rec: m[3],
            idr_epoch: m[4],
            snr_off,
        })
}

fn disc() -> impl Strategy<Value = rc::Disc> {
    (any::<u32>(), any::<[u8; 4]>(), any::<u16>(), any::<u16>()).prop_map(|(n, b, cap, seq)| {
        rc::Disc {
            vrx_nonce: n,
            op_channel: b[0],
            op_width: b[1],
            table_ver: b[2],
            init_profile: b[3],
            cap_bits: cap,
            seq,
        }
    })
}

fn disc_ack() -> impl Strategy<Value = rc::DiscAck> {
    (any::<(u32, u32, u16)>(), any::<[u8; 3]>(), any::<u16>()).prop_map(|((a, b, c), m, seq)| {
        rc::DiscAck {
            vrx_nonce: a,
            vtx_nonce: b,
            chip_caps: c,
            agreed_channel: m[0],
            agreed_width: m[1],
            flags: m[2],
            seq,
        }
    })
}

fn telem() -> impl Strategy<Value = rc::Telem> {
    (
        any::<[u16; 14]>(),
        any::<[u32; 2]>(),
        any::<u64>(),
        any::<[u8; 8]>(),
    )
        .prop_map(|(w, d, pts, b)| rc::Telem {
            tlm_seq: w[0],
            state: b[0],
            flags: b[1],
            rcf_age_ms: w[1],
            rcf_seq_echo: w[2],
            pts_at_build: pts,
            rcf_rx: d[0],
            cmd_kbps: w[3],
            txq_drops: d[1],
            txq_wait_max_ms: w[4],
            usb_fail: w[5],
            up_rssi: [b[2], b[3]],
            up_snr: [b[4] as i8, b[5] as i8],
            soc_temp_c: b[6] as i8,
            cpu_busy_x100: w[6],
            rx_own: w[7],
            rx_foreign: w[8],
            rx_crcfail: w[9],
            rec_status: b[7],
            nack_rx: w[10],
            retx_syms: w[11],
            retx_refused: w[12],
        })
}

fn nack() -> impl Strategy<Value = rc::Nack> {
    (
        any::<u32>(),
        any::<u8>(),
        any::<u8>(),
        1u8..=rc::MAX_NACK_ENTRIES as u8,
        any::<[(u32, u32); rc::MAX_NACK_ENTRIES]>(),
    )
        .prop_map(|(counter, sid, flags, n, es)| {
            let mut x = rc::Nack {
                counter,
                sid,
                flags,
                n,
                ..Default::default()
            };
            for (slot, &(first_seq, bitmap)) in x.e.iter_mut().zip(&es).take(usize::from(n)) {
                *slot = rc::NackEntry {
                    first_seq,
                    // The packer forces bit 0: the parsed bitmap has it set.
                    bitmap: bitmap | 1,
                };
            }
            x
        })
}

fn mav() -> impl Strategy<Value = rc::Mav> {
    (any::<u32>(), bytes(rc::MAV_MAX_PAYLOAD + 1))
        .prop_map(|(counter, payload)| rc::Mav { counter, payload })
}

/// Every RC frame type, packed: the base the mutation tests bend.
fn rc_frame() -> impl Strategy<Value = Vec<u8>> {
    (key(), ctx()).prop_flat_map(|(k, c)| {
        prop_oneof![
            rcf().prop_map(move |x| rc::pack_rcf(&x, &k, &c)),
            disc().prop_map(move |x| rc::pack_disc(&x, &k)),
            disc_ack().prop_map(|x| rc::pack_disc_ack(&x)),
            telem().prop_map(|x| rc::pack_telem(&x)),
            nack().prop_map(move |x| rc::pack_nack(&x, &k, &c)),
            (mav(), any::<bool>()).prop_map(move |(x, t)| rc::pack_mav(&x, t, &k, &c)),
        ]
    })
}

fn parse_all_rc(b: &[u8], k: &LinkKey, c: &TagCtx) {
    let _ = rc::parse_rcf(b);
    let _ = rc::parse_disc(b);
    let _ = rc::parse_disc_ack(b);
    let _ = rc::parse_telem(b);
    if let Some(n) = rc::parse_nack(b) {
        assert!((1..=rc::MAX_NACK_ENTRIES).contains(&usize::from(n.n)));
    }
    for tagged in [false, true] {
        if let Some(m) = rc::parse_mav(b, tagged) {
            assert!(m.payload.len() <= rc::MAV_MAX_PAYLOAD);
        }
    }
    let _ = rc::verify_control(b, k, c);
    let _ = rc::frame_type(b);
}

/// Whole sub-blocks a body of this length holds.
fn blocks(b: &[u8], bp: usize) -> usize {
    if bp == 0 || b.len() < sbi::HDR_LEN {
        0
    } else {
        (b.len() - sbi::HDR_LEN) / (2 + bp)
    }
}

fn sbi_body(envs: &[Vec<u8>], bpb: usize, sid: u8) -> Vec<Vec<u8>> {
    let bp = envs.first().map_or(1, Vec::len);
    let mut p = sbi::Packer::new(bp, bpb, sid);
    let mut out: Vec<Vec<u8>> = envs.iter().filter_map(|e| p.add(e)).collect();
    out.extend(p.flush());
    out
}

proptest! {
    #![proptest_config(cfg(512))]

    #[test]
    fn dot11_any_bytes(b in bytes(80), crc_err: bool, rate: u16) {
        if let dot11::Verdict::Body(p) = dot11::parse(&b, crc_err, rate) {
            prop_assert!(p.body_off < b.len());
            prop_assert!(p.mac_seq < 0x1000);
        }
        let _ = dot11::sa_canonical(&b);
    }

    #[test]
    fn dot11_control_frame_round_trip(seq: u16, body in bytes(600), rate: u16) {
        let f = dot11::control_frame(seq, &body);
        let v = dot11::parse(&f, false, rate);
        if body.is_empty() {
            prop_assert_eq!(v, dot11::Verdict::Short);
        } else {
            let dot11::Verdict::Body(p) = v else {
                return Err(TestCaseError::fail("own control frame not parsed"));
            };
            prop_assert_eq!(p.mac_seq, seq & 0xFFF);
            prop_assert_eq!(&f[p.body_off..], &body[..]);
        }
        prop_assert!(dot11::next_control_seq(seq) <= 0xFFF);
    }

    #[test]
    fn sbi_any_bytes(b in bytes(1500), bp in 0usize..1600) {
        let u = sbi::unpack(&b, bp);
        prop_assert_eq!(u.survivors.len() + u.n_failed, blocks(&b, bp));
        prop_assert!(u.survivors.iter().all(|s| s.len() == bp));
        let _ = sbi::peek_stream_id(&b);
    }

    #[test]
    fn sbi_round_trip_and_mutation(
        envs in (1usize..400).prop_flat_map(|bp| {
            proptest::collection::vec(proptest::collection::vec(any::<u8>(), bp), 1..12)
        }),
        bpb in 1usize..9,
        sid in 0u8..0x80,
        es in edits(),
    ) {
        let bp = envs[0].len();
        let bodies = sbi_body(&envs, bpb, sid);
        let mut got = Vec::new();
        for b in &bodies {
            prop_assert_eq!(sbi::peek_stream_id(b), Some(sid));
            let u = sbi::unpack(b, bp);
            prop_assert!(u.header_ok);
            prop_assert_eq!(u.n_failed, 0);
            got.extend(u.survivors.iter().map(|s| s.to_vec()));
        }
        prop_assert_eq!(&got, &envs);
        let bent = apply(bodies[0].clone(), &es);
        let u = sbi::unpack(&bent, bp);
        prop_assert_eq!(u.survivors.len() + u.n_failed, blocks(&bent, bp));
        let _ = probe::parse_body(&bent, bp);
    }

    #[test]
    fn probe_any_and_mutated(
        b in bytes(1200),
        bp in 0usize..600,
        hdr in any::<(u32, u8, u16)>(),
        n in 1usize..40,
        es in edits(),
    ) {
        let _ = probe::parse_body(&b, bp);
        let _ = probe::parse_hdr(&b);
        // A valid probe body: n blocks, each the 9 B header then fill.
        let bp = bp.max(probe::HDR_LEN);
        let mut blk = vec![0x5Au8; bp];
        blk[..2].copy_from_slice(&probe::MAGIC.to_le_bytes());
        blk[2..6].copy_from_slice(&hdr.0.to_le_bytes());
        blk[6] = hdr.1;
        blk[7..9].copy_from_slice(&hdr.2.to_le_bytes());
        let envs = vec![blk; n];
        let bodies = sbi_body(&envs, n, sbi::PROBE_STREAM_ID);
        let rx = probe::parse_body(&bodies[0], bp);
        if n <= 32 {
            let rx = rx.unwrap();
            prop_assert_eq!(rx.n_blocks, n);
            prop_assert_eq!(rx.n_ok as usize, n);
            prop_assert_eq!((rx.hdr.seq, rx.hdr.profile, rx.hdr.enh_fid), hdr);
        }
        let _ = probe::parse_body(&apply(bodies[0].clone(), &es), bp);
        prop_assert!(probe::body_len(n, bp) >= sbi::HDR_LEN);
    }

    #[test]
    fn rc_any_bytes(b in bytes(700), k in key(), c in ctx()) {
        parse_all_rc(&b, &k, &c);
    }

    #[test]
    fn rc_any_bytes_behind_a_valid_head(
        ty in 0u8..12,
        tail in bytes(700),
        k in key(),
        c in ctx(),
        fix in any::<bool>(),
    ) {
        let mut b = rc::RC_MAGIC.to_le_bytes().to_vec();
        b.push(rc::RC_VERSION);
        b.push(ty);
        b.extend_from_slice(&tail);
        let b = if fix { recrc(b) } else { b };
        parse_all_rc(&b, &k, &c);
    }

    #[test]
    fn rc_mutated(b in rc_frame(), es in edits(), fix: bool, k in key(), c in ctx()) {
        let bent = apply(b, &es);
        let bent = if fix { recrc(bent) } else { bent };
        parse_all_rc(&bent, &k, &c);
    }

    /// A count or length field decides where the parser looks for the CRC,
    /// so a bent frame passes the CRC with the field out of range only when
    /// its length happens to match: with the NACK count bound removed, the
    /// tests above stayed green at the default case count and failed at
    /// `PAL_PROP_SCALE=50`. Here the frame is resized to its own lying field
    /// and the CRC written where the parser looks, so the bound is all that
    /// stands between the field and the entry array.
    #[test]
    fn rc_count_and_length_fields_that_lie(
        n in nack(),
        cnt: u8,
        m in mav(),
        pl in 0usize..1400,
        tagged: bool,
        fill: u8,
        k in key(),
        c in ctx(),
    ) {
        let sized = |mut b: Vec<u8>, plen: usize| {
            b.resize(plen, fill);
            let crc = crc16_ccitt(&b);
            b.extend_from_slice(&crc.to_le_bytes());
            b
        };
        // The NACK layout, from the packer: fixed part, entries, tag, CRC.
        let len_of = |cnt: u8| rc::pack_nack(&rc::Nack { n: cnt, ..n }, &k, &c).len();
        let entry = len_of(2) - len_of(1);
        let fixed = len_of(1) - entry - rc::TAG_LEN - 2;
        let mut b = rc::pack_nack(&n, &k, &c);
        b[fixed - 1] = cnt;
        let b = sized(b, fixed + usize::from(cnt) * entry + rc::TAG_LEN);
        parse_all_rc(&b, &k, &c);
        prop_assert_eq!(
            rc::parse_nack(&b).is_some(),
            (1..=rc::MAX_NACK_ENTRIES).contains(&usize::from(cnt))
        );

        let mut b = rc::pack_mav(&m, tagged, &k, &c);
        b[8..10].copy_from_slice(&(pl as u16).to_le_bytes());
        let b = sized(b, rc::MAV_FIXED_LEN + pl + if tagged { rc::TAG_LEN } else { 0 });
        parse_all_rc(&b, &k, &c);
        prop_assert_eq!(rc::parse_mav(&b, tagged).is_some(), pl <= rc::MAV_MAX_PAYLOAD);
    }

    #[test]
    fn rc_round_trips(
        r in rcf(),
        d in disc(),
        a in disc_ack(),
        t in telem(),
        n in nack(),
        m in mav(),
        k in key(),
        c in ctx(),
        tagged: bool,
    ) {
        let w = rc::pack_rcf(&r, &k, &c);
        prop_assert_eq!(rc::frame_type(&w), Some(rc::T_RCF));
        prop_assert!(rc::verify_control(&w, &k, &c));
        prop_assert_eq!(rc::parse_rcf(&w), Some(r));
        let w = rc::pack_disc(&d, &k);
        prop_assert!(rc::verify_control(&w, &k, &TagCtx::default()));
        prop_assert_eq!(rc::parse_disc(&w), Some(d));
        prop_assert_eq!(rc::parse_disc_ack(&rc::pack_disc_ack(&a)), Some(a));
        prop_assert_eq!(rc::parse_telem(&rc::pack_telem(&t)), Some(t));
        let w = rc::pack_nack(&n, &k, &c);
        prop_assert!(rc::verify_control(&w, &k, &c));
        prop_assert_eq!(rc::parse_nack(&w), Some(n));
        let w = rc::pack_mav(&m, tagged, &k, &c);
        prop_assert_eq!(rc::verify_control(&w, &k, &c), tagged);
        prop_assert_eq!(rc::parse_mav(&w, tagged), Some(m));
        // Any key change must fail the tag.
        let mut k2 = k;
        k2[0] ^= 1;
        prop_assert!(!rc::verify_control(&rc::pack_rcf(&r, &k, &c), &k2, &c));
        let _ = rc::key_fingerprint(&k);
    }

    #[test]
    fn mavlink_any_stream(b in bytes(2000)) {
        let mut p = mavlink::Parser::default();
        for &c in &b {
            if p.feed(c) == mavlink::Framing::Ok {
                let m = p.message();
                prop_assert!(mavlink::entry(m.msgid).is_some());
            }
        }
        let _ = cal::all_packets_are_cal(&b);
        let _ = cal::decode_all(&b);
    }

    #[test]
    fn mavlink_round_trip_and_mutation(
        e in proptest::sample::select(mavlink::MSG_ENTRIES),
        payload in bytes(256),
        hdr in any::<(u8, u8, u8)>(),
        lead in bytes(8),
        es in edits(),
    ) {
        let (msgid, _, _, max) = e;
        let mut pl = payload;
        pl.truncate(usize::from(max));
        let pkt = mavlink::pack_v2(msgid, hdr.0, hdr.1, hdr.2, &pl);
        let mut p = mavlink::Parser::default();
        // Noise without a start byte first: the parser must stay idle.
        for &c in lead.iter().filter(|&&c| c != mavlink::STX && c != mavlink::STX_V1) {
            prop_assert_eq!(p.feed(c), mavlink::Framing::Incomplete);
        }
        let mut oks = 0;
        for (i, &c) in pkt.iter().enumerate() {
            let f = p.feed(c);
            if i + 1 < pkt.len() {
                prop_assert_eq!(f, mavlink::Framing::Incomplete);
            } else {
                prop_assert_eq!(f, mavlink::Framing::Ok);
                oks += 1;
            }
        }
        prop_assert_eq!(oks, 1);
        let m = p.message();
        prop_assert_eq!((m.msgid, m.sysid, m.compid, m.seq), (msgid, hdr.0, hdr.1, hdr.2));
        let mut want = pl.clone();
        want.resize(usize::from(max), 0);
        prop_assert_eq!(&m.payload[..usize::from(max)], &want[..]);
        prop_assert!(p.is_idle());
        let bent = apply(pkt, &es);
        let mut p = mavlink::Parser::default();
        for &c in &bent {
            let _ = p.feed(c);
        }
        let _ = cal::decode_all(&bent);
    }

    #[test]
    fn cal_payload_any_and_round_trip(
        b in bytes(1500),
        kind in 1u8..=3,
        rate in 0u8..=7,
        idx in cal::REL_MIN..=cal::REL_MAX,
        bw40: bool,
        flags in 0u8..4,
        seq in 0u32..(1 << 24),
        len in 0usize..1500,
        es in edits(),
    ) {
        if let Some(f) = cal::parse_payload(&b) {
            prop_assert!(f.len <= b.len() && f.len >= cal::HEADER_LEN);
        }
        let kind = cal::Kind::from_u8(kind).unwrap();
        let bw = if bw40 { 40 } else { 20 };
        let p = cal::build_payload(kind, rate, idx as i8, bw, flags, seq, len);
        let f = cal::parse_payload(&p).unwrap();
        prop_assert_eq!(
            (f.kind, f.rate, f.idx, f.bw, f.flags, f.seq, f.len),
            (kind, rate, idx as i8, bw, flags, seq, len.max(cal::HEADER_LEN))
        );
        let _ = cal::parse_payload(&apply(p, &es));
    }

    #[test]
    fn cal_decode_any(id in cal::MSG_ID_LO..=cal::MSG_ID_HI + 1, p in bytes(80)) {
        let _ = cal::decode(id, &p);
        let _ = cal::is_cal_msgid(id);
    }

    #[test]
    fn sw_header_any_and_round_trip(
        b in bytes(32),
        repair: bool,
        ss: u16,
        seq: u32,
        wl in 1u8..=255,
        rk: u32,
    ) {
        if let Some(h) = sw::Header::parse(&b) {
            prop_assert!(h.repair == (h.window_len != 0));
        }
        let h = if repair {
            sw::Header { repair, symbol_size: ss, seq, window_len: wl, repair_key: rk }
        } else {
            sw::Header { repair, symbol_size: ss, seq, window_len: 0, repair_key: 0 }
        };
        let mut w = [0u8; sw::HEADER_LEN];
        h.pack(&mut w);
        prop_assert_eq!(sw::Header::parse(&w), Some(h));
        let mut c = vec![0u8; usize::from(wl)];
        sw::repair_coeffs(rk, &mut c);
        prop_assert!(c.iter().all(|&x| x != 0));
    }

    #[test]
    fn frame_hdr_any_and_round_trip(b in bytes(16), h in any::<(u16, u8, u8, u32)>()) {
        let _ = frame_wire::FrameHdr::parse(&b);
        let h = frame_wire::FrameHdr { frame_id: h.0, flags: h.1, slice_rows: h.2, pts_us: h.3 };
        prop_assert_eq!(frame_wire::FrameHdr::parse(&h.pack()), Some(h));
    }

    #[test]
    fn profile_and_grid_any(b: u8, mcs: u8, bw: u8) {
        let p = profile::decode(b);
        prop_assert_eq!(profile::decode(profile::encode(p)), p);
        let _ = profile::phy_rate_mbps(p.mode, mcs, bw);
        let _ = profile::layer_mcs(p.mode, mcs);
        let o = ht40::ht40_pair_other(b);
        if o != 0 {
            prop_assert_eq!(ht40::ht40_pair_other(o), b);
        }
        let _ = ht40::grid_5g(bw);
    }
}

fn cal_msg() -> impl Strategy<Value = cal::CalMsg> {
    let kind = (1u8..=3).prop_map(|k| cal::Kind::from_u8(k).unwrap());
    let reason =
        proptest::collection::vec(1u8..0x80, 0..50).prop_map(|v| String::from_utf8(v).unwrap());
    prop_oneof![
        (
            any::<u32>(),
            any::<u8>(),
            kind,
            any::<u8>(),
            any::<[u16; 4]>(),
            any::<[i8; 2]>(),
            any::<u8>()
        )
            .prop_map(
                |(nonce, seq, kind, bw, w, idx, mcs_mask)| cal::CalMsg::Sweep(cal::SweepCmd {
                    nonce,
                    seq,
                    kind,
                    bw,
                    frames_per_cell: w[0],
                    settle_ms: w[1],
                    gap_us: w[2],
                    dwell_ms: w[3],
                    idx_lo: idx[0],
                    idx_hi: idx[1],
                    mcs_mask,
                })
            ),
        (any::<u32>(), any::<u8>(), 0u8..4, any::<u16>(), any::<u8>()).prop_map(
            |(nonce, seq, r, start_in_ms, flags)| cal::CalMsg::Ack(cal::SweepAck {
                nonce,
                seq,
                result: cal::AckResult::from_u8(r),
                start_in_ms,
                flags,
            })
        ),
        (
            any::<u32>(),
            any::<u8>(),
            any::<[u32; 8]>(),
            any::<[u8; 8]>()
        )
            .prop_map(|(nonce, seq, offered, blocked_pct)| cal::CalMsg::Done(
                cal::SweepDone {
                    nonce,
                    seq,
                    offered,
                    blocked_pct,
                }
            )),
        (
            any::<u32>(),
            any::<[u8; 2]>(),
            any::<[bool; 2]>(),
            any::<[i8; 8]>(),
            any::<i8>(),
            any::<[u16; 8]>()
        )
            .prop_map(
                |(nonce, sb, h, walls, legacy, eff_x1000)| cal::CalMsg::Apply(cal::ApplyCmd {
                    nonce,
                    seq: sb[0],
                    bw: sb[1],
                    has_walls: h[0],
                    has_eff: h[1],
                    walls,
                    legacy,
                    eff_x1000,
                })
            ),
        (any::<u32>(), any::<[u8; 2]>(), any::<bool>(), reason).prop_map(
            |(nonce, sb, ok, reason)| cal::CalMsg::Applied(cal::Applied {
                nonce,
                seq: sb[0],
                bw: sb[1],
                ok,
                reason,
            })
        ),
        any::<u32>().prop_map(|nonce| cal::CalMsg::Abort(cal::Abort { nonce })),
    ]
}

proptest! {
    #![proptest_config(cfg(256))]

    #[test]
    fn cal_messages_round_trip(
        ms in proptest::collection::vec(cal_msg(), 1..6),
        hdr in any::<(u8, u8, u8)>(),
        es in edits(),
    ) {
        let mut stream = Vec::new();
        for m in &ms {
            stream.extend(cal::encode(m, hdr.0, hdr.1, hdr.2));
        }
        prop_assert!(cal::all_packets_are_cal(&stream));
        prop_assert_eq!(&cal::decode_all(&stream), &ms);
        let _ = cal::decode_all(&apply(stream, &es));
    }
}
