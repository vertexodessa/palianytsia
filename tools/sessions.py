#!/usr/bin/env python3
"""Per-session summary of pal flight logs (stats.jsonl, 1 Hz).
Usage: sessions.py <logs root> > sessions.tsv"""
import json, os, sys, statistics as st

root = sys.argv[1]
cols = ("session secs linked_s rssi_med rssi_p05 snr_med loss_s loss_mean loss_p95 loss_max "
        "resid_s trunc trunc_base salvaged aus idr_req rung_max rung_changes hops ncards "
        "nack_req nack_fill nack_late nack_waste uplink_pct bodies_mb").split()
print("\t".join(cols))
for d in sorted(os.listdir(root)):
    p = os.path.join(root, d, "stats.jsonl")
    if not os.path.isfile(p):
        continue
    rssi, snr, loss = [], [], []
    resid_s = linked = n = rung_changes = 0
    rung_max = 0
    last = None
    prev_rung = None
    ncards = 0
    for line in open(p, errors="replace"):
        try:
            j = json.loads(line)
        except Exception:
            continue
        if "t_us" not in j:
            continue
        n += 1
        last = j
        if j.get("session"):
            linked += 1
        if j.get("rssi_dbm") is not None:
            rssi.append(j["rssi_dbm"])
        if j.get("snr_db") is not None:
            snr.append(j["snr_db"])
        if j.get("pre_fec_loss") is not None:
            loss.append(j["pre_fec_loss"])
        if (j.get("residual") or 0) > 0:
            resid_s += 1
        r = j.get("rung")
        if r is not None:
            rung_max = max(rung_max, r)
            if prev_rung is not None and r != prev_rung:
                rung_changes += 1
            prev_rung = r
        ncards = max(ncards, len(j.get("cards") or []))
    if last is None:
        continue
    q = lambda v, f: (sorted(v)[int(f * (len(v) - 1))] if v else float("nan"))
    nk = last.get("nack") or {}
    bp = os.path.join(root, d, "bodies.bin")
    row = [d, n, linked,
           st.median(rssi) if rssi else float("nan"), q(rssi, 0.05),
           st.median(snr) if snr else float("nan"),
           sum(1 for x in loss if x > 0), (sum(loss) / len(loss)) if loss else float("nan"),
           q(loss, 0.95), max(loss) if loss else float("nan"),
           resid_s, last.get("trunc"), last.get("trunc_base"), last.get("salvaged"),
           last.get("aus"), last.get("idr_req"), rung_max, rung_changes,
           (last.get("hop") or {}).get("hops"), ncards,
           nk.get("req"), nk.get("fill"), nk.get("late"), nk.get("waste"),
           last.get("uplink_pct"),
           round(os.path.getsize(bp) / 1048576) if os.path.isfile(bp) else 0]
    print("\t".join(f"{x:.4g}" if isinstance(x, float) else str(x) for x in row))
