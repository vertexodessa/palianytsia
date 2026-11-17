#!/usr/bin/env python3
"""Writes mavlink.json beside this script: MAVLink frames packed by pymavlink
(the common dialect), the goldens crates/pal-wire/tests/vectors.rs checks the
Rust framer against. Run once; the output is committed.

    python3 crates/pal-wire/tests/vectors/mavlink_gen.py
"""
import json
import os

from pymavlink.dialects.v10 import common as v1
from pymavlink.dialects.v20 import common as v2


def mav(mod, sysid, compid, seq=0):
    m = mod.MAVLink(None, srcSystem=sysid, srcComponent=compid)
    m.seq = seq
    return m


def frame(name, msgid, sysid, compid, raw, v1=False, signed=False, unsigned_same=True):
    return {
        "name": name,
        "msgid": msgid,
        "sysid": sysid,
        "compid": compid,
        "v1": v1,
        "signed": signed,
        # Unsigned MAVLink 2 frames only: pal_wire::mavlink::pack_v2 of the
        # parsed payload gives these bytes back.
        "repack": unsigned_same and not v1 and not signed,
        "hex": bytes(raw).hex(),
    }


out = []

m = mav(v2, 1, 1, seq=7)
hb = v2.MAVLink_heartbeat_message(2, 3, 0x80 | 0x01, 12345, 4, 3)
out.append(frame("heartbeat_fc_armed", 0, 1, 1, hb.pack(m)))

m = mav(v2, 1, 1, seq=8)
hb = v2.MAVLink_heartbeat_message(2, 3, 0x01, 12345, 4, 3)
out.append(frame("heartbeat_fc_disarmed", 0, 1, 1, hb.pack(m)))

m = mav(v2, 1, 1, seq=9)
st = v2.MAVLink_statustext_message(4, b"PreArm: Hardware safety switch", 0, 0)
out.append(frame("statustext", 253, 1, 1, st.pack(m)))

m = mav(v2, 1, 1, seq=10)
pv = v2.MAVLink_param_ext_value_message(b"VENC_MODE", b"3", 9, 41, 12)
out.append(frame("param_ext_value", 322, 1, 1, pv.pack(m)))

m = mav(v2, 255, 190, seq=11)
ps = v2.MAVLink_param_ext_set_message(1, 191, b"VENC_MODE", b"4", 9)
out.append(frame("param_ext_set", 323, 255, 190, ps.pack(m)))

m = mav(v2, 1, 191, seq=12)
pa = v2.MAVLink_param_ext_ack_message(b"VENC_MODE", b"4", 9, 0)
out.append(frame("param_ext_ack", 324, 1, 191, pa.pack(m)))

m = mav(v1, 1, 1, seq=13)
hb = v1.MAVLink_heartbeat_message(2, 3, 0x80 | 0x01, 12345, 4, 3)
out.append(frame("heartbeat_v1_armed", 0, 1, 1, hb.pack(m), v1=True))

m = mav(v2, 1, 1, seq=14)
m.signing.secret_key = bytes(range(32))
m.signing.link_id = 3
m.signing.timestamp = 0x0102030405
m.signing.sign_outgoing = True
hb = v2.MAVLink_heartbeat_message(2, 3, 0x80 | 0x01, 12345, 4, 3)
out.append(frame("heartbeat_signed_armed", 0, 1, 1, hb.pack(m), signed=True))

path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mavlink.json")
with open(path, "w") as f:
    json.dump({"generator": "mavlink_gen.py (pymavlink, common dialect)", "frames": out}, f, indent=1)
    f.write("\n")
print(path, len(out))
