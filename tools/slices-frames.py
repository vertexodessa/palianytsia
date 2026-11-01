#!/usr/bin/env python3
"""Writes a maburd --dry-run input frame file from mabur's 4-row-slice
fixture (tests/fixtures/slices4_1080p.h265): the IDR AU once, then the
P-frame AUs cycled to N frames. The P cycle includes the AU that re-sends
VPS/SPS/PPS, so the GS's slice-salvage tracker gets usable parameter sets
even when the IDR is lost. Frame record: u32 len | <IBBH pts, codec, flags, 0>
| Annex-B (tools/genvectors/gen_vectors.py).

usage: slices-frames.py <fixture.h265> <out.bin> [frames]
"""
import struct
import sys

CODEC_H265 = 0x01
FLAG_IDR = 0x01
FLAG_ENHANCE = 0x04  # must agree with TRAIL_N, or maburd demotes the frame


def nals(d):
    out, j = [], 0
    while True:
        j = d.find(b"\x00\x00\x01", j)
        if j < 0:
            return out
        s = j - 1 if j > 0 and d[j - 1] == 0 else j
        out.append((s, (d[j + 3] >> 1) & 0x3F, d[j + 5] >> 7))
        j += 3


def aus(d):
    """Split at a parameter set or a first_slice_segment slice that does not
    follow a parameter set."""
    cuts, prev_ps = [], False
    for s, ty, first in nals(d):
        if ty >= 32:
            if not prev_ps:
                cuts.append(s)
            prev_ps = True
        else:
            if first and not prev_ps:
                cuts.append(s)
            prev_ps = False
    cuts.append(len(d))
    return [d[a:b] for a, b in zip(cuts, cuts[1:])]


def main():
    src, dst = sys.argv[1], sys.argv[2]
    n = int(sys.argv[3]) if len(sys.argv) > 3 else 240
    units = aus(open(src, "rb").read())
    idr, ps = units[0], units[1:]
    seq = [idr] + [ps[i % len(ps)] for i in range(n - 1)]
    with open(dst, "wb") as f:
        for i, au in enumerate(seq):
            types = [t for _, t, _ in nals(au)]
            flags = FLAG_IDR if 19 in types or 20 in types else 0
            if types and all(t == 0 for t in types if t < 32):
                flags |= FLAG_ENHANCE
            meta = struct.pack("<IBBH", (i * 16667) & 0xFFFFFFFF, CODEC_H265, flags, 0)
            f.write(struct.pack("<I", len(meta) + len(au)))
            f.write(meta + au)
    print(f"wrote {dst}: {len(seq)} frames from {len(units)} fixture AUs")


if __name__ == "__main__":
    main()
