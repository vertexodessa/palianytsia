#!/usr/bin/env python3
"""retxmark.py IN.bin OUT.bin [EVERY]: copy a palcap 1 capture, re-marking base-layer video
bodies as NACK retransmits (kSbiRetxMark, bit 7 of the SBI stream byte) wherever the 802.11
sequence number is a multiple of EVERY (default 50). The body reaches the decoder as before, so
the video is whole, but its symbols are booked as filled by a retransmit: `tools/m1-replay.sh`
uses this to compare the two cores on the retransmit accounting and on what the ladder makes of
it. Prints the number of records marked."""
import struct
import sys

SBI_MAGIC = 0xF5B0


def main():
    src, dst = sys.argv[1], sys.argv[2]
    every = int(sys.argv[3]) if len(sys.argv) > 3 else 50
    b = bytearray(open(src, "rb").read())
    assert b[:9] == b"palcap 1\n", b[:9]
    i, n, marked = 9, len(b), 0
    while i < n:
        tag = b[i]
        if tag == 2:
            i += 9
            continue
        if tag != 1 or i + 29 > n:
            break
        ln, = struct.unpack_from("<I", b, i + 25)
        if i + 29 + ln > n:
            break
        seq, = struct.unpack_from("<H", b, i + 17)
        body = i + 29
        if ln >= 13 and seq % every == 0:
            magic, ver, sid = struct.unpack_from("<HBB", b, body)
            if magic == SBI_MAGIC and ver == 2 and sid == 0:
                b[body + 3] = sid | 0x80
                marked += 1
        i += 29 + ln
    open(dst, "wb").write(b)
    print(marked)


if __name__ == "__main__":
    main()
