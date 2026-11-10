#!/usr/bin/env python3
"""Loss trace of a palcap capture from the drone's 802.11 sequence numbers.

The drone numbers every frame it sends (12-bit mac_seq, one counter), and every card hears the
same numbers, so the frames no card received with a good FCS are the union loss in transmit order.
Lost frames get the time and MCS of their received neighbours.

FCS-failed records are counted per card and left out of every loss figure. On the 2026-11-03
Quest capture (two cards, bench, 210657 records) they were 4.6 % of the records: 55 % of them
weak (<= -85 dBm) frames without an HT rate (mcs 255), i.e. other 802.11 traffic and noise the
cards demodulate, not the drone; 29 % at the drone's RSSI (>= -40 dBm) with an HT rate, the
drone's own frames damaged (most still start with the SBI magic; 10 % of all FCS-failed records
have a good copy on the other card). So the FCS-failed count is no measure of the drone's loss.

The cards also hear the GS's own uplink (RCF, DISC, NACK and its MAV frames), numbered by the
GS's counter; those are left out (GS frames heard), a MAV frame counts as the drone's when its
number continues the drone's counter. Left in, they broke the unwrap into made-up outages and
loss runs (the outage lists of fl-1813 and fl-1772 before this).

The 12-bit counter cannot be unwrapped across a silence: after more than OUTAGE_US with no frame
from any card, a new segment starts and the frames between are unknown (an outage with no count).

Usage: losstrace.py bodies.bin [--json out.json]"""
import sys, json, bisect, collections as C
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import palcap  # noqa: E402

path = sys.argv[1]
recs = []          # (mono, card, mac_seq, mcs, rssi_best, snr_best, sid, retx, nenv)
bad = C.Counter()  # FCS-failed per card
salv = C.Counter()
# The cards also hear the GS's own uplink (RCF, DISC, NACK, its MAV frames), numbered by the
# GS's counter: those frames are left out, or they break the unwrap of the drone's counter.
gs_frames = 0
MAV = -2  # sid of a MAV frame, sender decided by its sequence number below
for r in palcap.records(path):
    if not r.crc:
        bad[r.card] += 1
        s = palcap.sbi(r.body, check_crc=True)
        if s:
            salv["env_ok"] += sum(1 for e in s[5] if e.crc_ok)
            salv["env_bad"] += sum(1 for e in s[5] if not e.crc_ok)
        continue
    s = palcap.sbi(r.body)
    kind = palcap.sender(r.body, s)
    if kind == "gs":
        gs_frames += 1
        continue
    sid, retx, nenv = (s[0], s[1], len(s[5])) if s else (-1, False, 0)
    if kind == "mav":
        sid = MAV
    # RSSI and SNR are only meaningful when the PHY status was valid.
    rssi = max(r.rssi) - 110 if r.phy else None
    snr = max(r.snr) if r.phy else None
    recs.append((r.mono, r.card, r.mac_seq, r.mcs, rssi, snr, sid, retx, nenv))
recs.sort()
# A MAV frame is the drone's when its number continues the drone's counter (the frames around it
# in time are the drone's video), the GS's otherwise.
MAV_NEAR = 64
kept, last_seq = [], None
for x in recs:
    if x[6] == MAV:
        if last_seq is None or min((x[2] - last_seq) & 0xFFF, (last_seq - x[2]) & 0xFFF) > MAV_NEAR:
            gs_frames += 1
            continue
    else:
        last_seq = x[2]
    kept.append(x)
recs = kept
cards = sorted({r[1] for r in recs})

# An outage (no card hears anything for OUTAGE_US, or the counter jumps back: drone restart, hop)
# is not a loss run: its frames are taken out of every figure.
OUTAGE_US = 300000

# One global unwrap of the 12-bit counter over the time-ordered merge of all cards. After a
# silence longer than OUTAGE_US the count is unknown: the next frame starts a new segment, placed
# past the last one so the keys stay in order.
newest = None
last_mono = None
seg_start = set()  # first unwrapped seq of each segment after the first
got = {}  # unwrapped seq -> {card: (mono, mcs, rssi, sid, retx)}
for mono, card, seq, mcs, rssi, snr, sid, retx, nenv in recs:
    if newest is None:
        u = seq
    elif mono - last_mono > OUTAGE_US:
        u = (newest // 0x1000 + 2) * 0x1000 + seq
        seg_start.add(u)
    else:
        d = (seq - newest) & 0xFFF
        if d >= 0x800:
            d -= 0x1000
        u = newest + d
    last_mono = mono
    if newest is None or u > newest:
        newest = u
    got.setdefault(u, {})[card] = (mono, mcs, rssi, sid, retx, snr)
t_of = lambda u: min(v[0] for v in got[u].values())

out = {"file": path, "cards": cards, "secs": (recs[-1][0] - recs[0][0]) / 1e6}
out["fcs_failed"] = {str(c): bad[c] for c in cards}
out["gs_frames_heard"] = gs_frames
out["salvage_envelopes"] = dict(salv)
keys = sorted(got)


def is_outage(i):
    """The gap before keys[i] is an outage: a new segment, a long silence or time going back."""
    dt = t_of(keys[i]) - t_of(keys[i - 1])
    return keys[i] in seg_start or dt > OUTAGE_US or dt < 0


outages = []  # (start us, frames or None when the counter could not count them, dt us)
lost_in_runs = 0
for i in range(1, len(keys)):
    g = keys[i] - keys[i - 1] - 1
    if keys[i] in seg_start:
        outages.append((t_of(keys[i - 1]), None, t_of(keys[i]) - t_of(keys[i - 1])))
    elif g > 0 and is_outage(i):
        outages.append((t_of(keys[i - 1]), g, t_of(keys[i]) - t_of(keys[i - 1])))
    elif g > 0:
        lost_in_runs += g
out["outages"] = len(outages)
out["outage_secs"] = sum(max(0, d) for _, _, d in outages) / 1e6
out["outage_list"] = outages
out["t0"] = recs[0][0]
out["t_end"] = recs[-1][0]
sent = len(keys) + lost_in_runs
out["frames_sent"] = sent
per_card_lost = {c: sent - sum(1 for u in got if c in got[u]) for c in cards}
union_lost = sent - len(got)
out["loss_pct_card"] = {str(c): 100 * per_card_lost[c] / sent for c in cards}
out["loss_pct_union"] = 100 * union_lost / sent
if len(cards) == 2:
    a, b = cards
    pa, pb = per_card_lost[a] / sent, per_card_lost[b] / sent
    out["loss_pct_if_independent"] = 100 * pa * pb
    out["p_b_lost_given_a_lost"] = union_lost / per_card_lost[a] if per_card_lost[a] else None
    out["p_a_lost_given_b_lost"] = union_lost / per_card_lost[b] if per_card_lost[b] else None

# Runs of lost frames (union), with the time and MCS of the neighbours.
keys = sorted(got)
runs = []  # (start_time_us, length, dt_us, mcs_before, mcs_after, sid_before, sid_after)
for i in range(1, len(keys)):
    g = keys[i] - keys[i - 1] - 1
    if g > 0 and not is_outage(i):
        p, n = got[keys[i - 1]], got[keys[i]]
        pv, nv = next(iter(p.values())), next(iter(n.values()))
        runs.append((t_of(keys[i - 1]), g, t_of(keys[i]) - t_of(keys[i - 1]), pv[1], nv[1], pv[3], nv[3]))
buckets = [(1, 1), (2, 2), (3, 3), (4, 5), (6, 10), (11, 20), (21, 50), (51, 200), (201, 10**9)]
hist = []
for a, b in buckets:
    rs = [r for r in runs if a <= r[1] <= b]
    hist.append({"len": f"{a}-{b}" if b < 10**9 else f">{a - 1}", "runs": len(rs),
                 "frames": sum(r[1] for r in rs),
                 "share_of_lost_pct": 100 * sum(r[1] for r in rs) / union_lost if union_lost else 0,
                 "dt_ms_med": sorted(r[2] for r in rs)[len(rs) // 2] / 1e3 if rs else None})
out["run_hist"] = hist

# Would a sliding-window code with a given budget have covered the union loss? A span of S frames
# (sources + repairs in transmit order) is covered when it lost at most T frames. Frames are bodies
# of 4 symbols; window 64 symbols = 16 source frames.
lost_idx = []
for i in range(1, len(keys)):
    g = keys[i] - keys[i - 1] - 1
    if g > 0 and not is_outage(i):
        lost_idx.extend(range(keys[i - 1] + 1, keys[i]))
span_tab = {}
for name, S_, T_ in (("ov 0.5: 24 frames, 8 repairs", 24, 8), ("ov 0.25: 20 frames, 4 repairs", 20, 4),
                    ("ov 0.125: 18 frames, 2 repairs", 18, 2), ("ov 0.0625: 17 frames, 1 repair", 17, 1)):
    bad_spans, j0 = 0, 0
    worst = 0
    for j1 in range(len(lost_idx)):
        while lost_idx[j1] - lost_idx[j0] >= S_:
            j0 += 1
        worst = max(worst, j1 - j0 + 1)
        if j1 - j0 + 1 == T_ + 1:
            bad_spans += 1
    span_tab[name] = [bad_spans, worst]
out["spans_over_budget"] = span_tab
out["minutes"] = out["secs"] / 60

# Clustering in time: seconds with no union loss, and how much of the loss the worst 5 % hold.
t0 = recs[0][0]
per_s = C.Counter()
for r in runs:
    per_s[(r[0] - t0) // 1000000] += r[1]
nsec = int(out["secs"]) + 1
out["seconds_without_loss_pct"] = 100 * (nsec - len(per_s)) / nsec
worst = sorted(per_s.values(), reverse=True)[:max(1, nsec // 20)]
out["loss_in_worst_5pct_seconds_pct"] = 100 * sum(worst) / union_lost if union_lost else 0

# Loss by MCS: a lost frame counts for an MCS when both neighbours flew at it.
sent_mcs, lost_mcs = C.Counter(), C.Counter()
for u in keys:
    sent_mcs[next(iter(got[u].values()))[1]] += 1
for r in runs:
    if r[3] == r[4]:
        lost_mcs[r[3]] += r[1]
        sent_mcs[r[3]] += r[1]
out["loss_pct_by_mcs"] = {str(m): [100 * lost_mcs[m] / sent_mcs[m], sent_mcs[m]] for m in sorted(sent_mcs)}

# Probe frames are single frames one rung up: a lost single between video frames of the same MCS
# cannot be a probe, a run next to a probe may be. Count how much loss touches a probe neighbour.
out["lost_frames_next_to_probe_pct"] = (100 * sum(r[1] for r in runs if 5 in (r[5], r[6])) / union_lost
                                        if union_lost else 0)

# Per-card loss against that card's RSSI, per second. A frame received with no valid PHY status
# counts as received but adds nothing to the mean RSSI; a second with no valid RSSI is left out.
rs_tab = {}
for c in cards:
    sec = C.defaultdict(lambda: [0, 0.0, 0, 0])  # frames, rssi sum, frames with rssi, lost
    prev = None
    for u in keys:
        v = got[u].get(c)
        if v is None:
            continue
        s = (v[0] - t0) // 1000000
        sec[s][0] += 1
        if v[2] is not None:
            sec[s][1] += v[2]
            sec[s][2] += 1
        if prev is not None and u - prev - 1 < 400 and u not in seg_start:
            sec[s][3] += u - prev - 1
        prev = u
    tab = C.defaultdict(lambda: [0, 0])
    for s, (n, rsum, nr, lost) in sec.items():
        if nr == 0:
            continue
        k = int(rsum / nr // 5 * 5)
        tab[k][0] += n + lost
        tab[k][1] += lost
    rs_tab[str(c)] = {str(k): [100 * v[1] / v[0], v[0]] for k, v in sorted(tab.items())}
out["loss_pct_by_rssi_bucket_card"] = rs_tab

# Periodicity of loss-run starts: share of runs whose start falls in each 5 ms bin of a 50 ms period
# is meaningless without the phase, so use intervals between consecutive run starts instead.
iv = C.Counter()
for i in range(1, len(runs)):
    d = (runs[i][0] - runs[i - 1][0]) / 1e3
    if d < 400:
        iv[int(d // 5) * 5] += 1
out["run_start_interval_ms_hist"] = {str(k): v for k, v in sorted(iv.items())}
out["runs"] = len(runs)

# Retransmits (NACK lane) seen, and frames per stream id.
out["frames_by_sid"] = dict(C.Counter(str(next(iter(got[u].values()))[3]) for u in keys))
out["retx_frames"] = sum(1 for u in keys if next(iter(got[u].values()))[4])

if "--json" in sys.argv:
    json.dump(out, open(sys.argv[sys.argv.index("--json") + 1], "w"), indent=1)
    json.dump([(r[0] - t0, r[1], r[2], r[3], r[4]) for r in runs],
              open(sys.argv[sys.argv.index("--json") + 1].replace(".json", ".runs.json"), "w"))
h = lambda k: out[k]
print(f"{path.split('/')[-2]}/{path.split('/')[-1]}: {out['secs']:.0f} s, {sent} frames sent, outages {out['outages']} ({out['outage_secs']:.1f} s), FCS-failed {dict(bad)}, GS frames heard {gs_frames}")
print("  loss % per card", {k: round(v, 3) for k, v in out["loss_pct_card"].items()},
      "union", round(out["loss_pct_union"], 4),
      "if independent", round(out.get("loss_pct_if_independent", float('nan')), 4),
      "P(other lost | one lost)", round(out.get("p_b_lost_given_a_lost") or 0, 3),
      round(out.get("p_a_lost_given_b_lost") or 0, 3))
print("  runs", len(runs), "| seconds without loss %", round(h("seconds_without_loss_pct"), 1),
      "| loss in worst 5 % seconds %", round(h("loss_in_worst_5pct_seconds_pct"), 1),
      "| next to probe %", round(h("lost_frames_next_to_probe_pct"), 1))
print("  run length: runs / share of lost frames % / median gap ms")
print("   ", "  ".join(f"{x['len']}: {x['runs']}/{x['share_of_lost_pct']:.0f}/{x['dt_ms_med'] if x['dt_ms_med'] is None else round(x['dt_ms_med'], 1)}" for x in hist))
print("  loss % by MCS (frames)", {k: (round(v[0], 3), v[1]) for k, v in out["loss_pct_by_mcs"].items()})
print("  loss % by RSSI bucket", {c: {k: (round(v[0], 2), v[1]) for k, v in t.items()} for c, t in rs_tab.items()})
print("  loss events beyond a window budget [count, worst lost in span]:", span_tab)
print("  salvage", dict(salv), "retx frames", out["retx_frames"], "sid", out["frames_by_sid"])
