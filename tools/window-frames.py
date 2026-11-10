#!/usr/bin/env python3
"""Writes a maburd --dry-run input whose FEC repair window changes over time,
from mabur's 4-row-slice fixture (tests/fixtures/slices4_1080p.h265): the IDR
AU, then `cycles` times a burst of the fixture's two largest P AUs (both
base layer, 13.7 and 20.9 KB: each spans more symbols than a small configured
window, so the drone widens that frame's repair window to frame + 32,
sw_encoder.cpp frame_window) followed by a run of its two smallest P AUs (one
per layer, a few symbols each, so their window stays near the configured
one). The GS's recent window falls back during each small run, so the run
has to outlast the decoder's two one-second epochs at the replay clock (900
us per air body), and the burst that follows starts with a recent window far
below the one on the air. Frame record as tools/slices-frames.py.

usage: window-frames.py <fixture.h265> <out.bin> [cycles big small]
  big: frames per burst, small: frames per small run (defaults 3 20 1200)
"""
import importlib.util
import os
import struct
import sys

# No tools/__pycache__ left in the tree by the import below.
sys.dont_write_bytecode = True
_spec = importlib.util.spec_from_file_location(
    "slices_frames", os.path.join(os.path.dirname(os.path.abspath(__file__)), "slices-frames.py"))
sf = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sf)


def main():
    if len(sys.argv) not in (3, 6):
        sys.exit(__doc__)
    src, dst = sys.argv[1], sys.argv[2]
    cycles, big, small = (int(x) for x in sys.argv[3:6]) if len(sys.argv) == 6 else (3, 20, 1200)
    units = sf.aus(open(src, "rb").read())
    idr, ps = units[0], sorted(units[1:], key=len)
    small_aus, big_aus = ps[:2], ps[-2:]
    seq = [idr]
    for _ in range(cycles):
        seq += [big_aus[i % 2] for i in range(big)]
        seq += [small_aus[i % 2] for i in range(small)]
    with open(dst, "wb") as f:
        for i, au in enumerate(seq):
            types = [t for _, t, _ in sf.nals(au)]
            flags = sf.FLAG_IDR if 19 in types or 20 in types else 0
            if types and all(t == 0 for t in types if t < 32):
                flags |= sf.FLAG_ENHANCE
            meta = struct.pack("<IBBH", (i * 16667) & 0xFFFFFFFF, sf.CODEC_H265, flags, 0)
            f.write(struct.pack("<I", len(meta) + len(au)))
            f.write(meta + au)
    print(f"wrote {dst}: {len(seq)} frames, {cycles} x ({big} big + {small} small)")


if __name__ == "__main__":
    main()
