#!/usr/bin/env python3
"""Repair window_len values in a maburd --dry-run frame file (u32 len | frame
records): SBI header (magic 0xF5B0, ver 2, block_payload at +4), then per
block <u16 crc, envelope>, the envelope opening with the sliding-window FEC
header <HBHIBI> (magic 0xF541, flags bit0 = repair, window_len at +9).
A dry-run file is never corrupted, so the block CRCs are not checked.
Prints one `sid window repairs` line per value.

usage: fec-windows.py <frames.bin> [--min-distinct N MIN_REPAIRS]
  exit 1 unless the base stream (sid 0) carries at least N distinct repair
  windows that each come on at least MIN_REPAIRS repairs; the floor keeps the
  encoder's ramp-up (a window clipped to the few symbols sent so far, a
  handful of repairs) from passing for a stream that varies.
"""
import collections
import struct
import sys

SBI_MAGIC = b"\xb0\xf5\x02"
SBI_HDR = 13
SW_MAGIC = 0xF541
SW_HDR = 14


def windows(path):
    b = open(path, "rb").read()
    i, out = 0, collections.Counter()
    while i + 4 <= len(b):
        n = struct.unpack_from("<I", b, i)[0]
        rec = b[i + 4:i + 4 + n]
        i += 4 + n
        j = rec.find(SBI_MAGIC)
        if j < 0 or len(rec) < j + SBI_HDR:
            continue
        sid = rec[j + 3] & 0x7F
        bp = struct.unpack_from("<H", rec, j + 4)[0]
        k = j + SBI_HDR
        while bp >= SW_HDR and k + 2 + bp <= len(rec):
            magic, flags, _ss, _seq, wl, _key = struct.unpack_from("<HBHIBI", rec, k + 2)
            k += 2 + bp
            if magic == SW_MAGIC and flags & 1:
                out[(sid, wl)] += 1
    return out


def main():
    a = sys.argv[1:]
    if len(a) not in (1, 4) or (len(a) == 4 and a[1] != "--min-distinct"):
        sys.exit(__doc__)
    w = windows(a[0])
    for (sid, wl), c in sorted(w.items()):
        print(f"sid={sid} window={wl} repairs={c}")
    if len(a) == 4:
        need, floor = int(a[2]), int(a[3])
        got = sorted(wl for (sid, wl), c in w.items() if sid == 0 and c >= floor)
        if len(got) < need:
            sys.exit(f"FAIL: base-stream repair windows on >= {floor} repairs in {a[0]}: {got}, "
                     f"want >= {need} distinct")


if __name__ == "__main__":
    main()
