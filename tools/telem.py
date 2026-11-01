#!/usr/bin/env python3
"""Drone-side RX counters from a palcap capture: every 15th T_TELEM (state,
rcf_rx, uplink RSSI, rx_own/rx_foreign/rx_crcfail). rx_own = 0 with video
arriving means the drone hears nothing from our card (check the card is
still on USB). Usage: tools/telem.py bodies.bin"""
import struct,sys
b=open(sys.argv[1],'rb').read(); i=9; t0=None; last=None; n=0
magic=None
while i<len(b):
    tag=b[i]
    if tag==2: i+=9; continue
    mono=struct.unpack_from('<Q',b,i+2)[0]; flags=b[i+16]; ln=struct.unpack_from('<I',b,i+25)[0]
    body=b[i+29:i+29+ln]; i+=29+ln
    if t0 is None: t0=mono
    if not (flags&2) or len(body)<48 or body[3]!=4: continue
    seq,state=struct.unpack_from('<HB',body,5); rcf_rx=struct.unpack_from('<I',body,20)[0]
    up=body[34:36]; own,foreign,crc=struct.unpack_from('<HHH',body,41)
    n+=1
    if n%15==1: print(f"t={(mono-t0)/1e6:5.1f}s seq={seq} state={state} rcf_rx={rcf_rx} up_rssi={list(up)} rx_own={own} rx_foreign={foreign} rx_crcfail={crc} hdr={body[:4].hex()}")
print("telem",n)
