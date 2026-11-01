//! MAVLink 2 framing for the T_MAV tunnel: the byte-at-a-time parser of
//! `mavlink_frame_char_buffer` (MAVLINK_CHECK_MESSAGE_LENGTH off, no
//! signing context) and the packer of `mavlink_finalize_message_buffer`.
//! The CRC_EXTRA table is generated from mabur's `mavlink/mabur/mabur.h`
//! (MAVLINK_MESSAGE_CRCS); a message id outside it fails its CRC, which is
//! how the tunnel rejects ids the dialect does not know.

use alloc::vec::Vec;

pub const STX: u8 = 0xFD;
pub const STX_V1: u8 = 0xFE;
const IFLAG_SIGNED: u8 = 0x01;
const IFLAG_MASK: u8 = 0x01;
const SIGNATURE_LEN: u8 = 13;
pub const MAX_PAYLOAD: usize = 255;

/// (msgid, crc_extra, min_len, max_len), ascending msgid.
pub const MSG_ENTRIES: &[(u32, u8, u8, u8)] = &[
    (0, 50, 9, 9),
    (1, 124, 31, 31),
    (2, 137, 12, 12),
    (4, 237, 14, 14),
    (5, 217, 28, 28),
    (6, 104, 3, 3),
    (7, 119, 32, 32),
    (11, 89, 6, 6),
    (20, 214, 20, 20),
    (21, 159, 2, 2),
    (22, 220, 25, 25),
    (23, 168, 23, 23),
    (24, 24, 30, 52),
    (25, 23, 101, 101),
    (26, 170, 22, 24),
    (27, 144, 26, 29),
    (28, 67, 16, 16),
    (29, 115, 14, 16),
    (30, 39, 28, 28),
    (31, 246, 32, 48),
    (32, 185, 28, 28),
    (33, 104, 28, 28),
    (34, 237, 22, 22),
    (35, 244, 22, 22),
    (36, 222, 21, 37),
    (37, 212, 6, 7),
    (38, 9, 6, 7),
    (39, 254, 37, 38),
    (40, 230, 4, 5),
    (41, 28, 4, 4),
    (42, 28, 2, 6),
    (43, 132, 2, 3),
    (44, 221, 4, 5),
    (45, 232, 2, 3),
    (46, 11, 2, 2),
    (47, 153, 3, 4),
    (48, 41, 13, 21),
    (49, 39, 12, 20),
    (50, 78, 37, 37),
    (51, 196, 4, 5),
    (54, 15, 27, 27),
    (55, 3, 25, 25),
    (61, 167, 72, 72),
    (62, 183, 26, 26),
    (63, 119, 181, 181),
    (64, 191, 225, 225),
    (65, 118, 42, 42),
    (66, 148, 6, 6),
    (67, 21, 4, 4),
    (69, 243, 11, 30),
    (70, 124, 18, 38),
    (73, 38, 37, 38),
    (74, 20, 20, 20),
    (75, 158, 35, 35),
    (76, 152, 33, 33),
    (77, 143, 3, 10),
    (81, 106, 22, 22),
    (82, 49, 39, 39),
    (83, 22, 37, 37),
    (84, 143, 53, 53),
    (85, 140, 51, 51),
    (86, 5, 53, 53),
    (87, 150, 51, 51),
    (89, 231, 28, 28),
    (90, 183, 56, 56),
    (91, 63, 42, 42),
    (92, 54, 33, 33),
    (93, 47, 81, 81),
    (100, 175, 26, 34),
    (101, 102, 32, 117),
    (102, 158, 32, 117),
    (103, 208, 20, 57),
    (104, 56, 32, 116),
    (105, 93, 62, 63),
    (106, 138, 44, 44),
    (107, 108, 64, 65),
    (108, 32, 84, 92),
    (109, 185, 9, 9),
    (110, 84, 254, 254),
    (111, 34, 16, 16),
    (112, 174, 12, 12),
    (113, 124, 36, 39),
    (114, 237, 44, 44),
    (115, 4, 64, 64),
    (116, 76, 22, 24),
    (117, 128, 6, 6),
    (118, 56, 14, 14),
    (119, 116, 12, 12),
    (120, 134, 97, 97),
    (121, 237, 2, 2),
    (122, 203, 2, 2),
    (123, 250, 113, 113),
    (124, 87, 35, 57),
    (125, 203, 6, 6),
    (126, 220, 79, 79),
    (127, 25, 35, 35),
    (128, 226, 35, 35),
    (129, 46, 22, 24),
    (130, 29, 13, 13),
    (131, 223, 255, 255),
    (132, 85, 14, 39),
    (133, 6, 18, 18),
    (134, 229, 43, 43),
    (135, 203, 8, 8),
    (136, 1, 22, 22),
    (137, 195, 14, 16),
    (138, 109, 36, 120),
    (139, 168, 43, 43),
    (140, 181, 41, 41),
    (141, 47, 32, 32),
    (142, 72, 243, 243),
    (143, 131, 14, 16),
    (144, 127, 93, 93),
    (146, 103, 100, 100),
    (147, 154, 36, 54),
    (148, 178, 60, 78),
    (149, 200, 30, 60),
    (162, 189, 8, 9),
    (192, 36, 44, 54),
    (225, 208, 65, 73),
    (230, 163, 42, 42),
    (231, 105, 40, 40),
    (232, 151, 63, 65),
    (233, 35, 182, 182),
    (234, 150, 40, 40),
    (235, 179, 42, 42),
    (241, 90, 32, 32),
    (242, 104, 52, 60),
    (243, 85, 53, 61),
    (244, 95, 6, 6),
    (245, 130, 2, 2),
    (246, 184, 38, 38),
    (247, 81, 19, 19),
    (248, 8, 254, 254),
    (249, 204, 36, 36),
    (250, 49, 30, 30),
    (251, 170, 18, 18),
    (252, 44, 18, 18),
    (253, 83, 51, 54),
    (254, 46, 9, 9),
    (256, 71, 42, 42),
    (257, 131, 9, 9),
    (258, 187, 32, 232),
    (259, 92, 235, 236),
    (260, 146, 5, 13),
    (261, 179, 27, 60),
    (262, 12, 18, 22),
    (263, 133, 255, 255),
    (264, 49, 28, 28),
    (265, 26, 16, 20),
    (266, 193, 255, 255),
    (267, 35, 255, 255),
    (268, 14, 4, 4),
    (269, 109, 213, 214),
    (270, 59, 19, 19),
    (271, 22, 52, 52),
    (275, 126, 31, 31),
    (276, 18, 49, 49),
    (277, 62, 30, 30),
    (280, 70, 33, 33),
    (281, 48, 13, 13),
    (282, 123, 35, 35),
    (283, 74, 144, 145),
    (284, 99, 32, 32),
    (285, 137, 40, 49),
    (286, 210, 53, 57),
    (287, 1, 23, 23),
    (288, 20, 23, 23),
    (299, 19, 96, 96),
    (301, 243, 58, 58),
    (310, 28, 17, 17),
    (311, 95, 116, 116),
    (320, 243, 20, 20),
    (321, 88, 2, 2),
    (322, 243, 149, 149),
    (323, 78, 147, 147),
    (324, 132, 146, 146),
    (330, 23, 158, 167),
    (331, 91, 230, 233),
    (332, 236, 239, 239),
    (333, 231, 109, 109),
    (335, 225, 24, 24),
    (339, 199, 5, 5),
    (340, 99, 70, 70),
    (350, 232, 20, 252),
    (370, 75, 87, 109),
    (373, 117, 42, 42),
    (375, 251, 140, 140),
    (376, 199, 8, 8),
    (385, 147, 133, 133),
    (386, 132, 16, 16),
    (387, 4, 72, 72),
    (388, 8, 37, 37),
    (9000, 113, 137, 137),
    (9005, 117, 34, 34),
    (12900, 114, 44, 44),
    (12901, 254, 59, 59),
    (12902, 140, 53, 53),
    (12903, 249, 46, 46),
    (12904, 77, 54, 54),
    (12905, 49, 43, 43),
    (12915, 94, 249, 249),
    (12918, 139, 51, 51),
    (12919, 7, 18, 18),
    (12920, 20, 5, 5),
    (60000, 193, 18, 18),
    (60001, 87, 9, 9),
    (60002, 127, 45, 45),
    (60003, 48, 32, 32),
    (60004, 5, 57, 57),
    (60005, 204, 4, 4),
];

pub fn entry(msgid: u32) -> Option<(u8, u8, u8)> {
    MSG_ENTRIES
        .binary_search_by_key(&msgid, |e| e.0)
        .ok()
        .map(|i| (MSG_ENTRIES[i].1, MSG_ENTRIES[i].2, MSG_ENTRIES[i].3))
}

fn crc_acc(crc: u16, b: u8) -> u16 {
    let mut t = b ^ (crc & 0xFF) as u8;
    t ^= t << 4;
    let t = u16::from(t);
    (crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Framing {
    Incomplete,
    Ok,
    BadCrc,
    BadSignature,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum St {
    Uninit,
    Idle,
    GotStx,
    GotLength,
    GotIncompat,
    GotCompat,
    GotSeq,
    GotSysid,
    GotCompid,
    GotMsgid1,
    GotMsgid2,
    GotMsgid3,
    GotPayload,
    GotCrc1,
    GotBadCrc1,
    SigWait,
    SigWaitBadCrc,
}

#[derive(Clone)]
pub struct Message {
    pub v1: bool,
    pub len: u8,
    pub incompat_flags: u8,
    pub seq: u8,
    pub sysid: u8,
    pub compid: u8,
    pub msgid: u32,
    /// Zero-extended to the message's max length (MAVLink 2 truncation).
    pub payload: [u8; MAX_PAYLOAD + 2],
}

impl Default for Message {
    fn default() -> Self {
        Self {
            v1: false,
            len: 0,
            incompat_flags: 0,
            seq: 0,
            sysid: 0,
            compid: 0,
            msgid: 0,
            payload: [0; MAX_PAYLOAD + 2],
        }
    }
}

/// One parser channel (`mavlink_status_t` + its rx buffer).
#[derive(Clone)]
pub struct Parser {
    st: St,
    msg: Message,
    crc: u16,
    idx: usize,
    sig_wait: u8,
}

impl Default for Parser {
    fn default() -> Self {
        Self {
            st: St::Uninit,
            msg: Message::default(),
            crc: 0xFFFF,
            idx: 0,
            sig_wait: 0,
        }
    }
}

impl Parser {
    pub fn reset(&mut self) {
        *self = Self::default();
    }
    pub fn is_idle(&self) -> bool {
        self.st <= St::Idle
    }
    pub fn message(&self) -> &Message {
        &self.msg
    }

    fn up(&mut self, c: u8) {
        self.crc = crc_acc(self.crc, c);
    }

    pub fn feed(&mut self, c: u8) -> Framing {
        let mut out = Framing::Incomplete;
        match self.st {
            St::Uninit | St::Idle => {
                if c == STX || c == STX_V1 {
                    self.st = St::GotStx;
                    self.msg.len = 0;
                    self.msg.v1 = c == STX_V1;
                    self.crc = 0xFFFF;
                }
            }
            St::GotStx => {
                self.msg.len = c;
                self.idx = 0;
                self.up(c);
                if self.msg.v1 {
                    self.msg.incompat_flags = 0;
                    self.st = St::GotCompat;
                } else {
                    self.st = St::GotLength;
                }
            }
            St::GotLength => {
                self.msg.incompat_flags = c;
                if c & !IFLAG_MASK != 0 {
                    self.st = St::Idle;
                } else {
                    self.up(c);
                    self.st = St::GotIncompat;
                }
            }
            St::GotIncompat => {
                self.up(c);
                self.st = St::GotCompat;
            }
            St::GotCompat => {
                self.msg.seq = c;
                self.up(c);
                self.st = St::GotSeq;
            }
            St::GotSeq => {
                self.msg.sysid = c;
                self.up(c);
                self.st = St::GotSysid;
            }
            St::GotSysid => {
                self.msg.compid = c;
                self.up(c);
                self.st = St::GotCompid;
            }
            St::GotCompid => {
                self.msg.msgid = u32::from(c);
                self.up(c);
                self.st = if !self.msg.v1 {
                    St::GotMsgid1
                } else if self.msg.len > 0 {
                    St::GotMsgid3
                } else {
                    St::GotPayload
                };
            }
            St::GotMsgid1 => {
                self.msg.msgid |= u32::from(c) << 8;
                self.up(c);
                self.st = St::GotMsgid2;
            }
            St::GotMsgid2 => {
                self.msg.msgid |= u32::from(c) << 16;
                self.up(c);
                self.st = if self.msg.len > 0 {
                    St::GotMsgid3
                } else {
                    St::GotPayload
                };
            }
            St::GotMsgid3 => {
                self.msg.payload[self.idx] = c;
                self.idx += 1;
                self.up(c);
                if self.idx == usize::from(self.msg.len) {
                    self.st = St::GotPayload;
                }
            }
            St::GotPayload => match entry(self.msg.msgid) {
                None => self.st = St::GotBadCrc1,
                Some((extra, _min, max)) => {
                    self.up(extra);
                    self.st = if c != (self.crc & 0xFF) as u8 {
                        St::GotBadCrc1
                    } else {
                        St::GotCrc1
                    };
                    let max = usize::from(max);
                    if self.idx < max {
                        self.msg.payload[self.idx..max].fill(0);
                    }
                }
            },
            St::GotCrc1 | St::GotBadCrc1 => {
                out = if self.st == St::GotBadCrc1 || c != (self.crc >> 8) as u8 {
                    Framing::BadCrc
                } else {
                    Framing::Ok
                };
                if self.msg.incompat_flags & IFLAG_SIGNED != 0 {
                    if out == Framing::BadCrc {
                        self.st = St::SigWaitBadCrc;
                    } else {
                        self.st = St::SigWait;
                        out = Framing::Incomplete;
                    }
                    self.sig_wait = SIGNATURE_LEN;
                } else {
                    self.st = St::Idle;
                }
            }
            St::SigWait | St::SigWaitBadCrc => {
                self.sig_wait -= 1;
                if self.sig_wait == 0 {
                    // No signing context: the check passes (mavlink_signature_check).
                    out = if self.st == St::SigWaitBadCrc {
                        Framing::BadCrc
                    } else {
                        Framing::Ok
                    };
                    self.st = St::Idle;
                }
            }
        }
        out
    }
}

/// `mavlink_finalize_message_buffer` + `mavlink_msg_to_send_buffer`,
/// MAVLink 2, unsigned. `payload` is the full-width struct; trailing zero
/// bytes are trimmed (never below 1).
pub fn pack_v2(msgid: u32, sysid: u8, compid: u8, seq: u8, payload: &[u8]) -> Vec<u8> {
    let extra = entry(msgid).map_or(0, |e| e.0);
    let mut len = payload.len();
    while len > 1 && payload[len - 1] == 0 {
        len -= 1;
    }
    let mut v = Vec::with_capacity(12 + len);
    v.extend_from_slice(&[
        STX,
        len as u8,
        0,
        0,
        seq,
        sysid,
        compid,
        msgid as u8,
        (msgid >> 8) as u8,
        (msgid >> 16) as u8,
    ]);
    v.extend_from_slice(&payload[..len]);
    let mut crc = 0xFFFFu16;
    for &b in &v[1..] {
        crc = crc_acc(crc, b);
    }
    crc = crc_acc(crc, extra);
    v.extend_from_slice(&crc.to_le_bytes());
    v
}

pub const MSG_HEARTBEAT: u32 = 0;
pub const MSG_STATUSTEXT: u32 = 253;
pub const MSG_PARAM_EXT_REQUEST_READ: u32 = 320;
pub const MSG_PARAM_EXT_REQUEST_LIST: u32 = 321;
pub const MSG_PARAM_EXT_VALUE: u32 = 322;
pub const MSG_PARAM_EXT_SET: u32 = 323;
pub const MSG_PARAM_EXT_ACK: u32 = 324;

pub const PARAM_ACK_ACCEPTED: u8 = 0;
pub const PARAM_ACK_VALUE_UNSUPPORTED: u8 = 1;
pub const PARAM_ACK_FAILED: u8 = 2;
pub const PARAM_ACK_IN_PROGRESS: u8 = 3;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_request_list() {
        let w = pack_v2(MSG_PARAM_EXT_REQUEST_LIST, 255, 190, 0, &[1, 191]);
        assert_eq!(w.len(), 10 + 2 + 2);
        let mut p = Parser::default();
        let mut ok = 0;
        for &b in &w {
            if p.feed(b) == Framing::Ok {
                ok += 1;
            }
        }
        assert_eq!(ok, 1);
        assert!(p.is_idle());
        assert_eq!(p.message().msgid, 321);
        assert_eq!(p.message().sysid, 255);
    }

    #[test]
    fn unknown_id_is_bad_crc() {
        let w = pack_v2(40000, 1, 1, 0, &[1, 2, 3]);
        let mut p = Parser::default();
        let r: Vec<Framing> = w.iter().map(|&b| p.feed(b)).collect();
        assert_eq!(*r.last().expect("last"), Framing::BadCrc);
    }
}
