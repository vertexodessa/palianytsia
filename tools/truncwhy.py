#!/usr/bin/env python3
"""What the air looked like before every truncated AU of a session.
A truncated AU's size in au.log is the prefix that arrived, not the AU the drone sent, so the
size of a truncated keyframe is unknown; the IDR median below counts complete IDRs only.
Usage: truncwhy.py <session dir> <losstrace json>   (run losstrace.py --json first)"""
import sys, json, collections as C
d, j = sys.argv[1], json.load(open(sys.argv[2]))
runs = json.load(open(sys.argv[2].replace(".json", ".runs.json")))
t0, tend = j["t0"], j["t_end"]
runs = [(t0 + r[0], r[1], r[2]) for r in runs]
outs = [(o[0], o[0] + max(0, o[2])) for o in j["outage_list"]]
ev = [json.loads(l) for l in open(d + "/events.jsonl")][1:]
rung_t = [e["t_us"] for e in ev if e["ev"] == "rung"]
W = 250000
cls, n_all, idr_tr, idr_all = C.Counter(), 0, 0, 0
sizes = C.defaultdict(list)
for line in open(d + "/au.log"):
    if line[0] in "a#":
        continue
    t, sid, nbytes, flags, complete, salv, pts = map(int, line.split())
    if t > tend:
        break
    n_all += 1
    idr_all += flags & 1
    if complete:
        sizes[flags & 1].append(nbytes)
        continue
    idr_tr += flags & 1
    w = [r for r in runs if t - W <= r[0] <= t]
    if any(a <= t and b >= t - W for a, b in outs):
        c = "outage"
    elif w and max(r[1] for r in w) >= 6:
        c = "run >= 6 frames"
    elif w and max(r[1] for r in w) >= 3:
        c = "run 3-5 frames"
    elif len(w) >= 3:
        c = "3+ separate short runs"
    elif w:
        c = "1-2 short runs"
    else:
        c = "no loss on air seen"
    if any(t - 400000 <= r <= t for r in rung_t):
        c += " | rung change"
    cls[(c, "salvaged" if salv else "broken")] += 1
med = lambda v: sorted(v)[len(v) // 2] if v else 0
print(f"{d.rstrip('/').split('/')[-1]}: AUs {n_all}, IDR {idr_all} (median {med(sizes[1])} B vs P {med(sizes[0])} B), "
      f"truncated {sum(cls.values())} of which IDR {idr_tr}")
for k, v in sorted(cls.items(), key=lambda x: -x[1]):
    print(f"   {v:4d}  {k[0]} ({k[1]})")
