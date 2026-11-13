#!/usr/bin/env python3
"""retxmark.py IN.bin OUT.bin [EVERY [DELAY_MS]]: copy a palcap 1 capture, re-marking base-layer
video bodies as NACK retransmits (kSbiRetxMark, bit 7 of the SBI stream byte) wherever the 802.11
sequence number is a multiple of EVERY (default 50). The body reaches the decoder as before, so
the video is whole, but its symbols are booked as filled by a retransmit: `tools/m1-replay.sh`
uses this to compare the two cores on the retransmit accounting and on what the ladder makes of
it. With DELAY_MS each marked body is also moved DELAY_MS later in the stream (stamped at its new
place, so time stays monotonic), as a retransmit that answers a request: the seq goes missing,
the GS asks for it, and the fill lands after the bodies that followed it, past the NACK deadline
when DELAY_MS exceeds the gap timeout. Prints the number of records marked."""
import struct
import sys

SBI_MAGIC = 0xF5B0


def main():
    src, dst = sys.argv[1], sys.argv[2]
    every = int(sys.argv[3]) if len(sys.argv) > 3 else 50
    delay_us = int(sys.argv[4]) * 1000 if len(sys.argv) > 4 else 0
    b = bytearray(open(src, "rb").read())
    assert b[:9] == b"palcap 1\n", b[:9]
    out = bytearray(b[:9])
    held = []  # (due_us, record bytes), in due order: one constant delay keeps arrival order
    i, n, marked = 9, len(b), 0

    def release(now_us):
        while held and held[0][0] <= now_us:
            due, rec = held.pop(0)
            struct.pack_into("<Q", rec, 2, due)
            out.extend(rec)

    while i < n:
        tag = b[i]
        if tag == 2:
            if i + 9 > n:
                break
            now, = struct.unpack_from("<Q", b, i + 1)
            release(now)
            out.extend(b[i:i + 9])
            i += 9
            continue
        if tag != 1 or i + 29 > n:
            break
        ln, = struct.unpack_from("<I", b, i + 25)
        if i + 29 + ln > n:
            break
        mono, = struct.unpack_from("<Q", b, i + 2)
        release(mono)
        rec = bytearray(b[i:i + 29 + ln])
        seq, = struct.unpack_from("<H", b, i + 17)
        body = 29
        hold = False
        if ln >= 13 and seq % every == 0:
            magic, ver, sid = struct.unpack_from("<HBB", rec, body)
            if magic == SBI_MAGIC and ver == 2 and sid == 0:
                rec[body + 3] = sid | 0x80
                marked += 1
                hold = delay_us > 0
        if hold:
            held.append((mono + delay_us, rec))
        else:
            out.extend(rec)
        i += 29 + ln
    for due, rec in held:
        struct.pack_into("<Q", rec, 2, due)
        out.extend(rec)
    out.extend(b[i:])  # a torn tail record, copied as it was
    open(dst, "wb").write(out)
    print(marked)


if __name__ == "__main__":
    main()
