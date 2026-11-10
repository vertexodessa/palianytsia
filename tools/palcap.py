"""Reader for palcap 1 captures (crates/flightlog/src/capture.rs) and the SBI / sliding-window
FEC envelope headers inside a body (third_party/mabur common: sbi.h, sw_wire.h)."""
import struct
from collections import namedtuple

Body = namedtuple("Body", "card mono rssi snr evm phy crc mac_seq tsfl mcs ch body")
Env = namedtuple("Env", "repair seq wlen key crc_ok")

SBI_MAGIC = 0xF5B0
SW_MAGIC = 0xF541


def records(path):
    """Yields Body records; ticks are skipped."""
    b = open(path, "rb").read()
    assert b[:9] == b"palcap 1\n", b[:9]
    i, n = 9, len(b)
    while i < n:
        tag = b[i]
        if tag == 2:
            i += 9
            continue
        if tag != 1 or i + 29 > n:
            break
        mono, = struct.unpack_from("<Q", b, i + 2)
        ln, = struct.unpack_from("<I", b, i + 25)
        if i + 29 + ln > n:
            break
        fl = b[i + 16]
        yield Body(b[i + 1], mono, (b[i + 10], b[i + 11]),
                   struct.unpack_from("<bb", b, i + 12), struct.unpack_from("<bb", b, i + 14),
                   bool(fl & 1), bool(fl & 2), struct.unpack_from("<H", b, i + 17)[0],
                   struct.unpack_from("<I", b, i + 19)[0], b[i + 23], b[i + 24],
                   b[i + 29:i + 29 + ln])
        i += 29 + ln


_CRC = []
for _i in range(256):
    _c = _i << 8
    for _ in range(8):
        _c = ((_c << 1) ^ 0x1021) & 0xFFFF if _c & 0x8000 else (_c << 1) & 0xFFFF
    _CRC.append(_c)


def crc16(data, c=0xFFFF):
    for x in data:
        c = ((c << 8) & 0xFFFF) ^ _CRC[(c >> 8) ^ x]
    return c


def sbi(body, check_crc=False):
    """(sid, retx, q_ms, enc_us, air_ms, [Env]) or None when the body is not SBI."""
    if len(body) < 13:
        return None
    magic, ver, sid, bp, nb, q, enc, air = struct.unpack_from("<HBBHBHHH", body, 0)
    if magic != SBI_MAGIC or ver != 2:
        return None
    envs = []
    off = 13
    for _ in range(nb):
        if off + 2 + bp > len(body):
            break
        p = body[off + 2:off + 2 + bp]
        ok = True
        if check_crc:
            ok = crc16(p) == struct.unpack_from("<H", body, off)[0]
        m, fl, ss, seq, wl, key = struct.unpack_from("<HBHIBI", p, 0)
        if m == SW_MAGIC or not ok:
            envs.append(Env(bool(fl & 1), seq, wl, key, ok and m == SW_MAGIC))
        off += 2 + bp
    return sid & 0x7F, bool(sid & 0x80), q, enc, air, envs
