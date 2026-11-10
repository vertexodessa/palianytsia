#!/usr/bin/env python3
"""Keyframes of a session: size, estimated air time at the rung flown, drone TX queue wait (SBI q_ms)
and queue drops (T_TELEM txq_drops) around each, complete against truncated.
A truncated AU's bytes (au.log) are the prefix that arrived, not what the drone sent: the
truncated rows' sizes and air times understate the keyframe.
Usage: idrwhy.py <session dir>"""
import sys, struct, bisect, collections as C
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import palcap  # noqa: E402
d = sys.argv[1].rstrip("/")
RATE = {0: 6.5, 1: 13, 2: 19.5, 3: 26, 4: 39, 5: 52}  # HT20 long GI 1 stream, Mb/s
vt, vm, vq = [], [], []       # video body time, mcs, q_ms
tt, td, tw, tk = [], [], [], []  # telem time, txq_drops, txq_wait_max, cmd_kbps
for r in palcap.records(d + "/bodies.bin"):
    if not r.crc:
        continue
    b = r.body
    s = palcap.sbi(b)
    if s:
        if s[0] in (0, 1) and not s[1] and r.card == 0:
            vt.append(r.mono); vm.append(r.mcs); vq.append(s[2])
    elif len(b) >= 48 and b[3] == 4 and r.card == 0:
        tt.append(r.mono); td.append(struct.unpack_from("<I", b, 26)[0])
        tw.append(struct.unpack_from("<H", b, 30)[0]); tk.append(struct.unpack_from("<H", b, 24)[0])
tend = vt[-1]
rows = []
for line in open(d + "/au.log"):
    if line[0] in "a#":
        continue
    t, sid, nb, fl, comp, salv, pts = map(int, line.split())
    if t > tend:
        break
    a, z = bisect.bisect_left(vt, t - 600000), bisect.bisect_right(vt, t)
    if z <= a:
        continue
    mcs = C.Counter(vm[a:z]).most_common(1)[0][0]
    ta, tz = bisect.bisect_left(tt, t - 600000), bisect.bisect_right(tt, t + 300000)
    drops = (td[tz - 1] - td[ta]) if tz - ta >= 2 else 0
    wait = max(tw[ta:tz]) if tz > ta else 0
    air_ms = nb * 8 * 1.5 / (RATE.get(mcs, 6.5) * 1e3)
    rows.append((fl & 1, comp, nb, mcs, air_ms, max(vq[a:z]), wait, drops))
med = lambda v: sorted(v)[len(v) // 2] if v else float("nan")
print(d.split("/")[-1], "| total txq_drops in capture", (td[-1] - td[0]) if td else None,
      "| cmd_kbps range", (min(tk), max(tk)) if tk else None)
for idr in (1, 0):
    for comp in (1, 0):
        v = [r for r in rows if r[0] == idr and r[1] == comp]
        if not v:
            continue
        print(f"   {'IDR' if idr else 'P  '} {'complete ' if comp else 'truncated'} n={len(v):5d}  bytes med {med([r[2] for r in v]):7d}"
              f"  air ms med {med([r[4] for r in v]):6.1f} p90 {sorted(r[4] for r in v)[int(.9 * (len(v) - 1))]:6.1f}"
              f"  q_ms max med {med([r[5] for r in v]):4d}  telem wait med {med([r[6] for r in v]):4d}"
              f"  with txq drops {100 * sum(1 for r in v if r[7] > 0) / len(v):5.1f} %"
              f"  mcs {dict(sorted(C.Counter(r[3] for r in v).items()))}")
