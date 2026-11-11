# Palianytsia: a Rust ground-station client for mabur

Status: Draft 2 of 2026-10-31, kept as written except for dated notes; the current state is "Status (2026-11-07)" below (Draft 1 reviewed: stale text after the desktop-first and strategy-C decisions fixed; capture format, maburd chip set and GS radio config corrected against the pinned tree). Name: **Palianytsia** (repo `palianytsia`, crate prefix `pal-`, CLI `pal`).
The name is a choice, not decoration: the word is the 2022 shibboleth used to tell a Russian speaker from a Ukrainian one, and no Russian military unit will want it on a ground station.
Paths like `exp-orig-backport/...`, `exp-mabur-fluke-pxp/...` and `devourer_stack/...` are on the author's machine, relative to a sibling checkout `../devourer_stack/` of palianytsia (`devourer_stack/...` relative to their common parent). Verified 2026-11-01: `exp-mabur-fluke-pxp/pixelpilot` is at e2d0c90, `exp-orig-backport/pixelpilot` at 33978f8, `devourer_stack/openipc-rs` at v0.1.40+1.
Home: `vertexodessa/palianytsia`, a public GitHub repo (not OpenIPC, not this workspace), as are the mabur and devourer forks it pins.

Sources read for this draft:
- mabur, original tree with backports, now pinned as `third_party/mabur` @ `51cea32f` (`RC_VERSION = 18`, `common/include/mabur/rc_proto.h:68`). This is the wire the quad flies today. Draft 1 read the same tree at its child 7bab3194, which only adds a bench debug endpoint.
- PixelPilot, unify fork `exp-mabur-fluke-pxp/pixelpilot` (e2d0c90), and the backport tree `exp-orig-backport/pixelpilot` (33978f8).
- openipc-rs `devourer_stack/openipc-rs` (v0.1.40, upstream last commit 2026-07-10).

The unify tree (RC 21, `T_CFG_SET`/`T_LOG`, `phone_wall_us`) is a reference only. This app does not speak it.

## Status (2026-11-07)

The dated milestone notes in sections 10 and 14 are history: each says what was true that day, and
their "not ported" / "still owed" lines are superseded by later ones. The current state:

- M0-M2 done. Desktop (`pal-desktop`, `pal`) flies the quad on the Rust core, 1..N cards; ref==rust
  byte parity on every capture (`tools/m1-replay.sh`, local only: captures never go to GitHub).
  NV12/I420 planes go to GL textures and convert in a shader (b7370f5); VA-API still untested.
- mabur pin (2026-11-08): `palianytsia/base-v2` rebased onto upstream master plus PR #119 (frame-tail FEC repair rule, `fec.repairs_with_frame` removed), the old line kept as branch `palianytsia/base-v2-pre-2026-11-08`. `tools/drone-install.py` replaces the shell installer and fits the config to the new maburd: it removes the key a maburd does not know and makes the one-time window raise to 64 that the removed key's build did at boot.
- mabur pin, later on 2026-11-08: a repair window per frame. The Rust core follows mabur's GS on it (`repair_window_recent`, the wait for the time left to send a frame). Unit tests cover that port; `tools/m0-parity.sh` and `tools/m1-replay.sh` also pass without it, because no capture has a window that varies per frame, so ref==rust on such traffic is unchecked. Resolved 2026-11-10: `tools/m0-parity.sh` has "window" cases on a synthetic stream whose window changes (`tools/window-frames.py`, drone `[fec] window = 32`, one block per body; the script fails unless `tools/fec-windows.py` finds two or more windows on the base stream); returning the all-time window from `repair_window_recent`, or dropping the frame wait, turns them red while the older cases stay green.
- mabur pin, 2026-11-08, after v0.10 (GS changes in the Rust core too): residual demotes wait 1200 ms after a rung change (the rung change's own IDR caused them), a hop with no frozen reference keeps the rung, IDR retries wait for the IDR's air time (`refresh.rs`, up to 1 s); drone: the air clock's backlog is bounded by what the TxQueue holds (a long overload kept every enhance frame shed: half the frame rate until maburd restarted). `VENC_MIN_IQP`, the keyframe QP floor, in the menu. Bench only, not flown.
- mabur pin, 2026-11-09: a failed width resync retries once a second instead of waiting for a reopen or a width flip, except 40 MHz on a channel with no pair. The Rust core follows it; `chan_trace` checks the retries against the C++ core with a failing card.
- mabur pin, 2026-11-10: the GS search covers the 5 GHz grid. The drone follows a GS to any grid channel and parks there on the next boot, so a drone another GS moved off the set was never found by a GS whose set lacks it (bench, 2026-11-09: drone parked on 100, `gs-auto` set 40/64/112/144, no session). After two passes over the set with no link every other search dwell bursts DISC on a grid channel outside the set; the link forms where the drone acks and the boot pick's relocation hops it back to a member. The Rust core follows it (`radio.use` no longer treats a link found off the set as outside the operator's choice); `chan_trace` checks a drone found on 100 and relocated, and random scripts open sessions there. Not yet checked on the bench. Resolved 2026-11-10 on the bench (one RTL8812AU on the desktop, drone parked on 161 at 20 MHz, `gs-auto` set 40/64/112/144): at 20 MHz the search found the drone on its first visit of 161, 31 s after start, and the relocation moved the link to 40; at 40 MHz it found nothing in 120 s until the devourer pin below, then 34 s. One card walks the grid in about 20 s per pass after its 5 s prelude; two cards (Quest) took 10 s.
- devourer pin, 2026-11-10: the 8812AU (jaguar1) 40 MHz centre of 153 and 161 was one pair too high (155 and 163 for 151 and 159), so a card tuned there at 40 MHz transmitted on the channel above: no handshake on 153 or 161 at 40 MHz with an 8812AU on either end (GS card, or a drone built from this tree), while 144, 149 and 157 linked. Found through the grid search above; bench after the fix: 153 and 161 link at 40 MHz. The 8822E and 8822C paths do not use that table.
- mabur pin, 2026-11-11: slice salvage refuses a slice span that wraps a uint32 to 0 (an SPS width of 2^31 at 128 rows per slice); mabur divided by it and died, the Rust port already took the geometry fallback, so this was the one known input where ref and rust differed (vector: `slice_span_of_a_huge_picture` in `pal-core/tests/regressions.rs`, `slice_span_wrapping_to_zero_passes_through` in mabur).
- M3: phone and Quest 3 apps fly the quad (releases v0.1-v0.11, signed APKs; v0.11, 2026-11-09: air clock bound, rung change hold, `VENC_MIN_IQP` in the menu, not flown yet; v0.10, 2026-11-08: per-frame FEC repair window, not flown yet; v0.9, 2026-11-08: the frame-tail FEC rule, not flown yet; v0.8.1, 2026-11-07: a drone config without `fec.repairs_with_frame` gets the mode with its window raised to 64; v0.8, 2026-11-04: maburd from mabur `release/v0.8` = `release/v0.7` + `fec.repairs_with_frame`, bundle `fec.window = 64`, built and packed locally, not published). The G2G and RCF-heard
  comparisons against PixelPilot and the 10 min Quest frame check are not measured yet.
  Planned (Ihor, 2026-11-03): Quest latency per stage with kernel tracing (the headset has root):
  one Perfetto trace with kernel USB/IRQ events (xhci/usb tracepoints, usbfs reap, kprobes) and
  pal-quest trace markers (devourer read, FEC/AU assembly, MediaCodec queue/dequeue, compositor
  submit, present), which gives stages t3-t7 of 13.2. Baseline first, in one session together with
  the G2G LED rig (13.4: medians within 3 ms). Then A/B one setting at a time, judged on the rig:
  performance governor or a higher min freq on policy2, SCHED_FIFO + CPU pinning for the RX and
  decode threads, USB IRQ affinity, `usbfs_memory_mb`, USB autosuspend off, fan forced high. All
  of them are runtime settings and reset on reboot.
- M3b (latency block in a matched mabur release): not started.
- M4, done: the channel core (Rust port of ChannelCore/scout/hop, checked against mabur's record by
  record in `pal-ref/tests/chan_trace.rs`), the channel menu and scan, TxSelector, per-card OSD
  (frames/s, CRC, DEAF) and `card_deaf` flight-log events, MAVLink telemetry widgets, TX-power
  calibration with the channel pick. Not done: GS-side DVR, VTX recording control beyond the
  REC_* parameters, preflight.
- M5: the parameter client is done; MSP OSD and Quest hand tracking are not.
- 2026-11-07: mabur `palianytsia/base-v2` rebased onto upstream master with #114 (hop verdict
  reads NHM airtime only: occupied route = blocked streak of `hop.verdict.occupied_windows`, detect
  route = a ring of `detect_windows` with median foreign % >= `detect_pct` and lossy slots; the
  store blank opens on the first blocked window; verdict loss = the worse video layer; own airtime
  scaled by seq delivery, best-chain RSSI) and #116 (no GS `radio.width`: the receiver width is
  derived from `[link]`; 40 MHz scans dwell once per pair on the primary, the scout never switches
  width; rankers order busy % first with `pick_margin_pct`; spare-card freshness bursts run on
  the in-flight thread, paced by `hop.burst_period_ms`). Ported to the Rust core; ref==rust holds
  (`chan_trace.rs` random scripts now jam op through NHM and decode frames;
  `pal-ref/tests/verdict_diff.rs` checks the verdict against mabur's on both routes). Not ported:
  mabur's live ChannelCore `set_width` (a Palianytsia width change restarts the link).
- Pins: the submodule gitlinks (`git submodule status`), each on the branch `.gitmodules` names; a release
  builds maburd from `release/MABUR_RELEASE`. `tools/doc-check.sh` checks both, so no sha is copied here.
- Owed in software: the review pass over the ported channel core (docs/channel-core-port.md, level 5).
- Owed on hardware (none run on air yet): the NHM-only hop verdict and the posted spare-card burst
  (2026-11-07 port), TxSelector switching, field calibration at distance,
  the Quest relink with the cards kept open, card_deaf on a real deaf card.
- Deferred (would change original mabur): the calibration measure card chosen by role at request
  time, a hop right after a rung climb.

---

## 1 Goal

One app that does PixelPilot's job for a mabur drone, written from scratch in Rust, keeping only what earns its place:

1. Open RTL88xx USB cards on a Linux PC (first platform, 5.7), an Android phone, or a Quest 3 (native OpenXR app, a main target); receive mabur; show H.265 video at the lowest glass latency the device allows.
2. Act as the drone's ground station: rendezvous, the RCF feedback loop, the ladder, NACK, IDR requests.
3. Show the link state and flight telemetry, record DVR and a flight log you can post-mortem.

Success is measured, not judged by reading:
- AU stream byte-identical to the C++ reference core on recorded captures (section 9).
- Glass-to-glass median and p95 on the G2G LED rig not worse than PixelPilot (backport tree) on the same phone, drone and rung.
- A per-frame latency breakdown on every frame, in flight, that agrees with the G2G rig (section 13). Without it, a latency regression is found only on the bench.
- "RCF heard %" (drone `Telem.rcf_rx` / GS `rcf_sent`) not worse than PixelPilot at the same distance.

## 2 Non-goals for v1

- wfb-ng in any form: no RTP, no UDP 5600/14550 loopback, no AF_UNIX hop, no VPN, no libsodium.
- H.264. mabur is H.265-only; there is no codec byte in `FrameHdr`.
- Audio (Opus). Nothing in mabur carries it.
- The drone web UI WebView, UDP video forwarding, pie charts, LDPC/STBC/FHSS toggles.
- The CPE510 relay radio, `MABUR_LOSS_SIM`. (The calibration lane was a non-goal in this draft; it was built in M4 after all, see `docs/calibration-port.md`.) The `Card` trait leaves room for a relay later.
- A browser build. Original mabur already ships a WebUSB + WASM GS (`web/`, gilankpam.github.io/mabur).
- A pure-Rust RTL88xx driver (see 5.2).
- armeabi-v7a. arm64-v8a only.
- Wire compatibility with anything except the pinned mabur commit (section 8).

## 3 Feature set

Priority: P0 = first flight, P1 = replaces PixelPilot for daily use, P2 = later.

| # | Feature | P | Where it comes from |
|---|---|---|---|
| F1 | N USB cards (N = 1 is just the small case), Linux and Android: devourer open (Android: per-card permission and fd), RX thread; every card receives, all bodies go into one Aggregator; one card transmits, picked by TxSelector (RSSI, 3 dB margin, 2000 ms hold, 1500 ms dead-card override) or pinned | P0 | PixelPilot `WfbNgLink.java` + `WfbngLink.cpp:580-606, 751-787`; mabur `aggregator.h:95-97`, `tx_selector.h` |
| F2 | Air RX: 802.11 parse, SA filter, RC frame vs SBI body, CRC-bad bodies kept for salvage | P0 | `gs/src/dot11.cpp`, `aggregator` |
| F3 | Sliding-window RLC FEC decode, two UEP layers (sid 0 base, sid 1 enh) | P0 | `common/sw_*`, `gf256`, `uep_decoder`, `sbi` |
| F4 | FrameStream: AU reassembly, adaptive gap timeout, lookahead advance, slice salvage | P0 | `gs/src/frame_stream`, `gap_timeout_policy.h`, `docs/slices.md` |
| F5 | Rendezvous (DISC/DISC_ACK), SipHash link key, key-mismatch detection; a visible warning while the default key `mabur-default-00` is in use (anyone with the default can command the drone) | P0 | `rendezvous`, `link_key.h`, `docs/link-pairing.md` |
| F6 | RCF loop: ladder controller, link health, RcfSlotter, RTT estimator, uplink delivery | P0 | `vrx_controller`, `ladder_controller`, `link_health`, `rcf_slot`, `rtt_estimator`, `uplink_delivery` |
| F7 | IDR request via RCF `idr_epoch`, plus decoder-reported damage | P0 | `refresh_requester.h` |
| F8 | Android decode: MediaCodec H.265, one whole AU per input buffer, low-latency keys, newest-frame render, 150 ms stale cap, no flush on loss | P0 | PixelPilot `VideoDecoder.cpp`, `AndroidMediaFormatHelper.h` |
| F9 | Link OSD: rung/MCS, RSSI/SNR per chain, pre/post-FEC loss, RTT, capture-to-AU latency, uplink %, drone temp, session/key state | P0 | `webgs::Stats` |
| F10 | Flight log: one structured file per session (section 7) | P0 | PixelPilot `FlightLog.*`, mabur session logs |
| F11 | NACK request side (part of the mabur protocol, not an app extra): `NackTracker` finds unrecovered sid-0 source symbols and sends `T_NACK` (direct, not slotted) inside the frame's gap time; the drone serves them from its `[nack]` ring (150 ms, 5 % air budget). Default on, as in the maburgs bundle; no capability bit, so a GS without it simply never asks. Repair only: a filled symbol still counts as loss for the ladder | P0 (core) | `nack_tracker.h`, `docs/fec-nack.md`, `[link.nack]` in `gs/bundle/maburgs.default.toml` |
| F12 | DVR: fMP4, one sample per AU, starts on an IRAP, IDR requested at arm, auto-record on arm | P1 | `raw_dvr.h`, `dvr_mux`, PixelPilot 33978f8 |
| F13 | MAVLink over `T_MAV`: telemetry widgets (attitude, battery, GPS, home, RSSI, STATUSTEXT, armed); on Linux also a UDP MAVLink endpoint for a GCS (QGroundControl, Mission Planner), as maburgs bridges to 127.0.0.1:14550 | P1 | `mav_tunnel`, `docs/mavlink.md`, PixelPilot widgets |
| F14 | Per-card health in the UI: RSSI/SNR/EVM per chain per card, which card transmits, dead-card warning, hot-plug of a second card mid-session without dropping the link | P1 | `CardTrack`, PixelPilot `releaseCard` (re-enumeration leak) |
| F15 | Channel core: channel set, boot pick, relocation, in-flight hop | P1 | `channel_core`, `channel_scout`, `hop_*`, `docs/channel-select.md`, `docs/inflight-channel-hop.md` |
| F16 | Spotter mode (receive only, follows hops) | P1 | `web/src/web_gs.h` Spotter, `spotter_follow` |
| F17 | VTX onboard recorder toggle (RCF `rec`) | P1 | `rec_control.h`, `docs/vtx-recorder.md` |
| F18 | Drone settings over `PARAM_EXT_*` (encoder, ladder, radio keys) | P2 | `param_client` |
| F19 | Quest 3 native OpenXR app (5.6): one decode into a compositor quad layer, display refresh matched to the video fps, head-locked goggle mode and world-locked screen mode, OSD as its own layer | P0 | replaces PixelPilot's 2D-panel approach (`VideoActivity.java:363-515, 646-689`: two SurfaceViews, two codecs) |
| F20 | MSP OSD stream (sid 4) rendering | P2 | `msp_dp`, `osd_screen`. msposd is off on every drone today, so P2. |
| F21 | Native Linux GS (5.7): the whole GS on a laptop, first platform to fly | P0 | the debug platform for everything that is not Android-specific |
| F22 | Preflight checklist: card health, per-chain RSSI/NF before takeoff, cross-polarised antenna warning | P1 | PixelPilot `PreflightPanel`, the Quest LHCP lesson (2026-09-19) |

## 4 What we know about the wire (RC 18)

The full detail lives in mabur. This is the summary the Rust core is built against; every number below was read from the pinned tree.

802.11:
- Drone video/probe/MSP bodies: QoS-Data `0x88`, 26 B header, DA ff:ff:.., SA = BSSID `57:42:75:05:d6:00`, QoS `0x20 0x00` (no-ack).
- Drone control (DISC_ACK, TELEM, MAV): probe-req `0x40`, 24 B.
- GS uplink: `0x40`, 24 B, same SA, HT MCS0 20 MHz, LDPC+STBC (`gs/src/dot11.cpp:13-45`).
- Body offset: 26 if FC is `0x88`, else 24. Bodies may carry the 4 B FCS.

RC frames: magic `0x5243` | ver | type | ... | [8 B SipHash-2-4 tag] | CRC16-CCITT.
- Types: RCF 1, DISC 2, DISC_ACK 3, TELEM 4, NACK 7, MAV 8.
- Sizes on air: RCF 27, DISC 27, DISC_ACK 22, TELEM 56, NACK 21+8n, MAV 12+len(+8 GS to drone).
- Tag over the frame bytes + `vrx_nonce || vtx_nonce || seq32`; key 16 B, default `"mabur-default-00"`.
- Required peer cap: `CAP_FRAME_WIRE` (0x1) in DISC_ACK, else no video.

SBI body: 13 B header (magic `0xF5B0`, ver 2, stream_id, block_payload, n_blocks, q_ms, enc_us, air_ms), then per block CRC16 + 14 B FEC envelope + symbol.
- stream_id: 0 base, 1 enh, 4 MSP, 5 probe; bit `0x80` = NACK retransmit.
- Deployed geometry: symbol 332, window 32, blocks_per_body [4,4], body 1405 B, cap 3760 B.

FEC: systematic sliding-window RLC over GF(256) poly `0x11D`; repair coefficients `(splitmix64(repair_key) & 0xFF) % 255 + 1`; incremental Gaussian elimination at the GS.

Video: `FrameHdr` 8 B (frame_id u16, flags IDR/DISCONT, slice_rows, pts_us u32) + Annex-B, cut into FRAG chunks of up to 324 B.

No encryption anywhere; only the uplink is authenticated.

## 5 Architecture

### 5.1 The one decision that shapes everything: where the protocol core comes from

mabur's wire changes without notice by design ("Compatibility policy: there isn't one", `mabur/CLAUDE.md`). A Rust port chases a moving C++ target. Three options:

| Option | For | Against |
|---|---|---|
| A. Link the C++ `webgs::WebGs` core through a C ABI | exact parity on day one; upstream changes arrive for free | the "Rust app" is a shell; C++ build inside cargo-ndk; GPL/licence questions identical to the C++ |
| B. Port the core to Rust, nothing else | one language, sans-IO, testable | months before first flight; every upstream wire change is a port |
| **C. Both behind one trait (decided)** | B is the product; A is kept as a test oracle that catches behaviour drift the golden vectors cannot see | two builds to keep green |

Decision C. The C++ core stays in the repo as a test dependency only (`pal-ref` crate). It never ships and no milestone flies on it. `WebGs` is already the right seam: devourer-free, single-threaded, clock-injected, `on_rx`/`tick`/`stats` plus an `Io` callback struct (`web/src/web_gs.h`). The Rust trait mirrors it.

```rust
// As built in crates/pal-core (M0). The core calls back into `io` from
// inside on_rx as well as tick (WebGs emits AUs from on_rx).
pub trait GsCore {
    fn on_rx(&mut self, body: &RxBody, io: &mut dyn CoreIo);  // body.mono_us = core clock
    fn tick(&mut self, now_us: u64, io: &mut dyn CoreIo);
    fn stats_json(&self) -> String;                           // webgs::stats_json shape
    // Added with the features that need them:
    // set_idr_requests, set_vtx_rec, mav_push_outbound, mav_take_inbound, typed stats.
}
pub trait CoreIo {
    fn on_au(&mut self, au: Au);
    fn send(&mut self, rc_body: &[u8]);
}
```

Both impls take the same captures and must emit the same AUs and the same sent bytes (with `rz_nonce` pinned, as `webgs` replay already supports).

Multi-card is in the trait from day one: `RxBody` carries `card_id`, the core owns one `Aggregator` sized to the roster and one `TxSelector`, and `send` returns which card to use. WebGs is single-card (TX always card 0, no TxSelector), so for multi-card the oracle is narrower: the Aggregator and TxSelector units are compared against the C++ units directly on a multi-card capture, and the WebGs parity gate runs per card. Two-card channel hop (lead/follow) lives in maburgs `main.cpp`, not in WebGs, and is ported in F15 with its own tests.

Android specifics of several cards (Linux needs only a powered hub): each card is its own USB permission and fd; a phone needs a powered USB-C hub for two 8812EU (TX on one card only keeps the draw down); a hub drop is a bus reset for every card at once, so the run lifecycle treats "all cards gone within 100 ms" as one event, not N (the Pixel USB-C port drop on 2026-09-10 looked like `rx_exit` plus a 7 s drone FAILSAFE).

### 5.2 Driver: devourer through a C shim, not a Rust driver

devourer is the driver with hardware proof on 8812EU, 8812AU and 8822C in this rig, including the traps we paid for (enable_with_tx on 8822E, the gauge-thread race). openipc-rs has a pure-Rust port (`openipc-rtl88xx`, nusb), but its README says live adapter testing is still owed and its native executor is a busy-spin (`device.rs:1352`). A second driver is a second set of bugs.

- `devourer-sys`: our own C ABI over `WiFiDriver::CreateRtlDevice`, about 300 lines in this repo: `dv_open_fd(fd, cfg)`, `dv_start_rx(cb, user)`, `dv_send(buf, len, rate)`, `dv_set_channel(ch, width, ht40_offset)`, `dv_rx_quality()`, `dv_close()`.
- Built by the `cmake` crate. Linux: system libusb, devourer's own CMake. Android: NDK toolchain, libusb from source with `libusb_wrap_sys_device` and `NO_DEVICE_DISCOVERY`, one explicit source list as PixelPilot does (devourer's own CMake needs pkg-config).
- Chips compiled into the GS: 8812AU (Jaguar1), 8822E and 8822C (Jaguar3). The GS may use any card it has; this is independent of the drone's chip.
- Config as maburgs sets it (`gs/src/radio_frontend.cpp:82-137`), not as PixelPilot does: `rx.enable_with_tx` (mandatory on 8822E), `rx.keep_corrupted`, `abs_noise_floor`, **CCA on** (`tuning.disable_cca = false`, `docs/findings/2026-09-23-cca-on.md`), `tx.no_cancel_multipkt`. The unify PixelPilot disables CCA; mabur measured and reversed that, so follow mabur. RX ring and URB count (PixelPilot: `RxMode::SpscFat`, 16 URBs, `tx.retry_limit = 0`) are tuning to re-measure, not copy. `usb_agg_max` is a drone TX setting and does not belong here.
- Revisit `openipc-rtl88xx` after v1 only with a hardware A/B (RX loss and RCF heard % on the same card).

### 5.3 Crates

The plan as drafted. As built (see `README.md` for the current layout): the channel core lives in
`pal-core::chan`, not in a `pal-chan` crate; there is no `radio` crate (cards are `devourer-sys` plus
`runtime::dev_card`) and no `dvr` crate yet (GS-side DVR is not done); `pal-android` holds the Android
link shared by the phone and the Quest; the phone shell is Java, not Kotlin.

```
crates/
  pal-wire       rc_proto, sbi, sw_wire envelope, frag, frame_wire, profile byte,
                 siphash, crc16. no_std + alloc. Golden vectors from mabur tests/vectors.
  pal-fec        gf256 (0x11D), sliding-window encoder and decoder (14); NEON + scalar,
                 bench vs C++.
  pal-core       sans-IO GsCore: aggregator, uep decoder, frame stream, gap policy,
                 rendezvous, vrx controller, ladder, link health, rcf slotter, rtt,
                 uplink delivery, refresh requester, nack, mav tunnel, param client.
                 No threads, no clock, no I/O.
  pal-chan       channel core + scout + hop (F15). Separate: it needs card energy
                 reads and its own thread.
  pal-ref        dev-only. Builds the C++ WebGs from third_party/mabur via cc/cmake,
                 implements GsCore. Never linked into a release APK.
  hevc           NAL walk, IRAP check, VPS/SPS/PPS capture, skip-slice fill for salvage.
  radio          trait Card { rx, send, tune, energy, caps } ; impls: devourer, replay file.
  devourer-sys   C shim + libusb + devourer build.
  dvr            fMP4 muxer, one sample per AU.
  flightlog      writer + reader (the reader is the post-mortem tool's library).
  runtime        threads, queues, snapshots (5.4). Platform-neutral.
  decode         trait Decoder; backends: android (NDK AMediaCodec on an ANativeWindow),
                 ffmpeg (libavcodec, VA-API or software, Linux).
  pal-osd        egui widgets and menus, shared by desktop, phone and Quest.
apps/
  android        phone: Kotlin shell + cdylib (5.5).
  quest          Quest 3: NativeActivity + OpenXR, pure Rust (5.6).
  desktop        native Linux GS, eframe (5.7).
  pal            Linux CLI: live (libusb), replay, dump .h265, print stats, parity diff.
```

### 5.4 Threads and data flow

```
card RX thread (devourer cb, one per card)
   | RxBody into a bounded SPSC ring (drop-newest + counter when full)
   v
core thread (SCHED_FIFO when allowed, else nice -16; tick every 2 ms)
   GsCore.on_rx / tick -> sends go back to the TX card through Card::send
   | Au into a 2-slot ring, drop-oldest + counter
   v
decode feed thread -> Decoder input (one AU per buffer)
decode output thread -> keep only the newest output; Android renders to the Surface,
  Linux hands the frame to the UI thread's texture

gauge thread (1 Hz): register reads (RX quality, NHM, temperature). Starts
  only after InitWrite. Never on the RX thread: 20-30 ms ring stalls when it was
  (fl-0047/0049), and starting it during bring-up gave a dead radio on ~40 % of
  starts.
chan thread (F15): scout dwells and energy reads.
log thread: flight log, bounded channel, drops counted, never blocks the core.
UI: reads the latest Stats snapshot (arc-swap / triple buffer). No mutex shared
  with the RX or core path; PixelPilot's shared `mabur_receive_mutex` lets a UI
  poll delay RX.
```

Rules that come from bugs we already paid for:
- One clock for RX stamps and `tick` (`CLOCK_MONOTONIC` in us).
- No blocking call on the RX or core thread: no logging I/O, no JNI, no allocation in steady state (pre-sized pools for bodies and AUs).
- The decoder is never flushed on loss: GDR/intra-refresh streams have no IDR to resync on. Drop stale AUs instead.
- A decoder restart bumps the IDR request count.

### 5.5 Android shell

Recommendation: a thin Kotlin app, everything else in Rust.

- Kotlin: one Activity, USB permission (explicit-package broadcast, Android 14), `UsbManager.openDevice().fileDescriptor` handed to Rust, a `SurfaceView` for the video and a second transparent surface above it for the OSD (drawn by Rust, see the last item).
- Rust: `ANativeWindow_fromSurface`, MediaCodec, link, logs. JNI surface kept to about 15 calls: start(fd, surface, config), stop, stats snapshot as a flat struct (not JSON), set_idr, set_rec, settings.
- Why not pure Rust egui on NativeActivity (openipc-rs's way): MediaCodec straight into a SurfaceView goes to a hardware composer overlay with no GL pass; decoding into an OES texture and drawing it with egui adds a GPU pass. Measure both on the G2G rig before locking this in; this row is the cheapest to get wrong.
- USB robustness, copied from what PixelPilot learnt: bounded 3 s join on stop, keep a wedged run's fd open as an orphan rather than reuse the number, release the card slot on re-enumeration, retry link every 3 s, survive a 30 s activity hide (Quest headset lift).
- minSdk 29 (ASurfaceControl and modern MediaCodec low-latency keys), targetSdk current Play level, arm64-v8a only.
- Low-latency MediaCodec keys: `low-latency`, `vendor.qti-ext-dec-low-latency.enable`, `vendor.qti-ext-dec-picture-order`; pin the display to its highest refresh rate.
- OSD and menus are drawn by Rust (egui) into a second, transparent surface above the video SurfaceView (`setZOrderMediaOverlay`), not by Compose. Reason: the same OSD code then draws the Quest OSD layer (5.6). Kotlin keeps only the Activity, USB permission and the two surfaces.

### 5.6 Quest 3: native OpenXR app

Quest 3 is a main target, not a port of the phone app. Separate APK (`apps/quest`), same `runtime`, `decode` (android backend), core and radio crates as the phone.

Stack: NativeActivity via `android-activity`, the `openxr` crate on the Khronos Android loader, GLES or Vulkan only for the OSD layer.

Starting skeleton: the `openxr` crate's own `examples/vulkan.rs` (Ralith/openxrs, Apache-2.0, crate 0.22.0, repo active as of 2026-10-23). It is one file, and the crate's Cargo.toml already builds it for Android as `vulkan-android` (aarch64). Grow the Quest app from that file: replace its debug pattern with the video quad layer and the OSD layer.

Prior art, likely over-engineered for us: leetvr/hotham (Apache-2.0/MIT, Rust engine for standalone Quest). It is a reference to read, not a dependency:
- Take from it the Rust + OpenXR + Android bootstrap (manifest, loader, packaging), the Vulkan swapchain wiring through `openxr`, egui rendered into an XR panel, and controller input.
- Do not depend on it. It is a whole game engine (PBR renderer, ECS, rapier physics, glTF, audio), and we need two layers. Its Android glue is `ndk-glue` 0.7, which `android-activity` replaced, and it uses egui 0.15 and openxr 0.17. The last code commit is 2025-07-02.
- A GitHub code search found no `XR_KHR_android_surface_swapchain` and no `CompositionLayerQuad` in it. Its panels are drawn into the eye buffers, so the zero-copy video path in this section is ours to build.

Manifest: `org.khronos.openxr.intent.category.IMMERSIVE_HMD`, `com.oculus.intent.category.VR`, `android.hardware.usb.host`.

Video path, zero-copy (the point of the whole exercise):
1. `XR_KHR_android_surface_swapchain`: `xrCreateSwapchainAndroidSurfaceKHR` gives a swapchain plus an Android `Surface`.
2. `ANativeWindow_fromSurface` on it, then the same `decode` android-backend MediaCodec code as on the phone renders into it. No GL pass by the app.
3. Each frame the app submits one `XrCompositionLayerQuad` that points at that swapchain, visible to both eyes. The camera is mono, so one quad serves both eyes. One decoder instead of PixelPilot's two.
4. The compositor latches the newest buffer at each display refresh.

Fallback if the runtime does not expose that extension: MediaCodec into an `AImageReader` (AHardwareBuffer), import as an EGLImage, blit into a normal swapchain image. That costs one GPU copy. **M0 spike: list the runtime's extensions on our Quest 3 and decode one capture through the primary path.** This spec does not claim that the primary path works until that spike runs.

Frame rate:
- `XR_FB_display_refresh_rate`: request the rate that the video fps divides evenly (120 Hz for 120 or 60 fps, 90 Hz for 90 or 30 fps); 100 fps, which no Quest 3 rate divides, gets 120 Hz (every frame shown, one in five twice) rather than 90 Hz (one in ten dropped). The measured fps rounds up to the camera mode it came from, since loss only lowers it. Re-request when the drone's camera mode changes (fps is known from the AU cadence or the encoder param).
- The XR frame loop (`xrWaitFrame`/`xrBeginFrame`/`xrEndFrame`) runs on its own thread and never waits for video: it submits whatever the swapchain holds. Video arrival and display refresh are decoupled, so a late AU repeats one refresh and never blocks the loop.
- Measured as: refreshes at video rate with no repeated or dropped video frame over 10 min at 120 fps (`XR_FB` performance counters or the compositor's frame stats), and G2G on the LED rig against PixelPilot on the same Quest.

Placement:
- Goggle mode (default): the quad sits in `XR_REFERENCE_SPACE_TYPE_VIEW`, head-locked like real FPV goggles, size and distance adjustable (PixelPilot's margin and distance sliders, made into quad size and z).
- Window mode (OSD tab, `View (Quest)`): world-locked in `LOCAL` space over `XR_FB_passthrough`, for flying with the room in view. It starts ahead of the pilot at eye height on every launch and every switch into the mode; size and distance are kept in `screen.json`, the place in the room is not. Right grip with the ray on the window carries it, the thumbsticks resize it and change its distance only while the ray is on it. The passthrough objects exist only in this mode, so goggle mode does not run the cameras. G2G against goggle mode is not measured yet.

OSD and menus:
- A second quad layer from a small GL swapchain. egui draws it at 30 Hz or on change, never at the video rate. The same widgets as the phone.
- Input: controller buttons for the common actions (record, IDR, menu, recenter), with ray-cast pointer on the menu panel. Hand tracking is P2.

Quest specifics:
- USB: one USB-C port, so cards plus charging go through a powered hub. Get USB permission without a dialog in immersive mode: a `USB_DEVICE_ATTACHED` intent filter with the device filter, so the system grants on attach. Verify on the device in the M0 spike; a permission dialog that cannot be seen from inside the headset is the failure to rule out.
- Keep the link up when the headset is lifted (the proximity sensor pauses the session). The radio and core threads ignore XR session state, and only the frame loop stops. PixelPilot learnt the same lesson with its 30 s hide.
- Development over wireless adb (`<quest-ip>:5555`; the address moves, find it with `nmap -p5555`).

### 5.7 Native Linux GS (first platform)

Why first: everything except MediaCodec, the Android USB fd and OpenXR is platform-neutral, and on Linux it can be debugged with gdb, rr, perf, ASan/TSan and plain `println`, without adb, USB permission dialogs or a headset on the head. The host already runs devourer on real cards (`rxdemo`, regress), so the radio side is proven there.

- App: `apps/desktop`, eframe (egui on wgpu). The same `pal-osd` widgets as the phone and the Quest, so the OSD is finished once.
- Radio: devourer over libusb on the host; card choice by bus/port or PID, as `DEVOURER_USB_BUS`/`DEVOURER_PID` do today. The kernel module must stay blacklisted (88XXau_wfb).
- Decode: the `decode` crate's ffmpeg backend, `libavcodec` HEVC (LGPL, fine under GPL-2.0) with VA-API when present, software otherwise. Latency settings: `flags low_delay`, no frame threading (`thread_type = slice`; frame threads add a frame of delay per thread), one AU in, at most one frame out.
- Display: NV12 planes to a wgpu texture. A CPU upload is acceptable here: the desktop is the debug platform, and its latency numbers are not the phone's. dmabuf import is a later optimisation.
- Debug tools here first: live plots of every stats field, the per-frame stage chain (13.2) as a timeline, capture on/off, replay a capture at 1x or stepped with the full UI.
- It also works as a real laptop ground station. That is a bonus, not a goal; the G2G target is set for the phone and the Quest.

Risk of going desktop-first: the parts where latency is actually won or lost (MediaCodec low-latency behaviour, the Quest surface swapchain, Android USB lifecycle) come later. That is why the Quest spike stays in M0 and the phone/Quest milestone comes right after the desktop flies, before DVR and the other P1 features.

## 6 UI (P0/P1 only)

- Fullscreen video, OSD chips: link (rung, MCS, width, session/key), signal (RSSI/SNR per chain, NF), loss (pre/post-FEC, truncated/salvaged AUs), latency (capture to AU complete, decode, render), uplink %, drone temp, DVR and VTX-rec state.
- Telemetry widgets from MAVLink: attitude, altitude, speed, battery V/A/mAh, GPS sats and home distance, flight mode, armed, STATUSTEXT.
- 60 s monitor plots: RSSI/SNR per chain, loss, rung, latency.
- Settings: link key, channel set / pin, width, card roles, record options, OSD layout.
- Pair stamp shown in settings and logged at start: app version, pinned mabur sha, `RC_VERSION`. RC frames rejected for a wrong version are counted and shown as a red banner ("drone speaks RC n, app speaks RC 18"): today a mismatched pair looks like the stale-caps deadlock and sends you to restart maburd.

## 7 Flight log

One directory per session on the device: `fl-NNNN_<yyyymmdd-HHMM>/`.
- `stats.jsonl`: one `webgs::stats_json`-compatible line per second. Whether mabur's `tools/flightreport.py` reads it as is, or needs the maburgs sideport shape, is checked in M0 (not verified for this draft).
- `events.jsonl`: session edges, rung changes, IDR requests, key mismatch, decoder restarts, USB detach/attach, stale-AU drops, channel moves.
- `au.log`: per AU t_first, t_complete, sid, size, complete/truncated/salvaged, decode-in, render.
- Optional raw capture (`bodies.bin`, size-capped; from two 64 MB segments up it is the first segment plus a ring of `bodies-tail-NNNNN.bin` segments, so the start and the end of a long session survive, each a whole capture that replays on its own): every `RxBody` as the core saw it (card id, mono stamp, CRC flag, RX metadata, body bytes), plus every `tick` time. This is the replay input for section 9. mabur's own replay format is not enough: `webgs replay` and `maburgs --dry-run` read a maburd `--dry-run --out` frame file (u32 length + radiotap + dot11 + body, `gs/src/frame_file_source.h`), which is TX-side and synthetic: no RX stamps, no real loss, no CRC-bad frames, no RSSI. The differential test feeds our capture to both cores directly through the `GsCore` trait, so no C++ reader is needed. Turning on a capture in the field is how a bug becomes a test.
- First line of every file: format marker with a version, as mabur does (`ctllog N`).
- A `pal log` subcommand summarises a session directory; with `--adb` it pulls one from the phone or the Quest first.

## 8 Wire pinning: submodules and matched releases

### 8.1 Submodules

```
third_party/mabur      -> vertexodessa/mabur  (fork of gilankpam/mabur)
third_party/devourer   -> vertexodessa/devourer (fork of OpenIPC/devourer; the pinned commit is gilankpam/devourer master)
```

- The gitlink is the pin. There is no separate `MABUR_SHA` file to drift from it.
- devourer is its own submodule, not `mabur/third_party/devourer`: in the backport tree that nested pin is stale (`f3b76eaf`, does not build with mabur `3eafcc3c`) and unused. The app pins the devourer that the pinned mabur actually builds against, the same way the backport PixelPilot tree does.
- Pin only commits on a named branch of the fork, never a bench-only commit (the backport tree's HEAD today is 7bab3194 "DEBUG (bench only, not for upstream)").
- `pal-ref`, the golden vectors (`third_party/mabur/tests/vectors/*.json`, read in place, no copies) and the maburd build all read the submodule.
- Bumping mabur is one PR: move the gitlink(s), run parity, port the diff. CI fails until the Rust core matches. No compat code for older RC versions.

### 8.2 Release = a matched pair

Every release tag builds both ends from the same gitlinks and ships them together, so "which maburd goes with this app" never has an answer other than "the one in the same release".

Release assets for tag `vX.Y.Z` (the plan; as built, `tools/pack-release.sh` packs `gs/` with both APKs, the
static desktop binary and `gs.toml`, `drone/` with `maburd`, `install.py` (from `tools/drone-install.py`),
`S96mabur`, `mabur.toml` and the IMX415 driver, plus `README.md` and `SHA256SUMS`; no CLI and no `PAIR.txt`,
the shas are in the README's Sources section):

| Asset | Built from |
|---|---|
| `palianytsia-phone-vX.Y.Z-arm64.apk`, `palianytsia-quest-vX.Y.Z.apk` | this repo |
| `palianytsia-vX.Y.Z-linux-x86_64` | the desktop GS |
| `maburd-vX.Y.Z-ssc338q-8822e` | `third_party/mabur` `tools/build-arm.sh`. At the current pin this builds **8822E only** (`build-arm.sh:85-87`), which fits the quad. The wing (8812AU) needs the chip switch (backport row A1) in the fork first; until then the wing stays on its own build and is not part of a Palianytsia release. |
| `mabur.default.toml`, `S96mabur` | `third_party/mabur/bundle/`, unchanged |
| `pal-vX.Y.Z-linux-x86_64` (+ aarch64) | the CLI |
| `PAIR.txt` | mabur sha, devourer sha, `RC_VERSION`, sha256 of every asset |
| `install-drone.sh` | the hand procedure as a script (8.3) |

- The pair stamp is the contract: maburd prints `maburd: build mabur <sha> devourer <sha> RC_VERSION <n>` at boot; the app shows and logs the same three values. The app compares them with the drone's if the drone can report them (today only the boot log has them; adding the two shas to DISC_ACK or Telem is a candidate upstream change, not a v1 dependency). Until then the RC-version banner (section 6) is the runtime check.
- The maburd build needs the OpenIPC SigmaStar toolchain. CI fetches one pinned archive and checks its sha256; local builds use `OPENIPC_HOST_BIN`, as today.
- Releases are built in CI only, from a clean checkout with `--recurse-submodules`. A locally built maburd is a dev build and says so in its stamp.

### 8.3 Drone install from a release

`install-drone.sh` encodes the procedure we already follow by hand, nothing new:
1. Back up the drone (`backup_drone.sh` equivalent) before anything.
2. Config before binary: diff the release `mabur.default.toml` keys against `/etc/mabur.toml`; refuse while the live file has keys the new parser rejects, or misses keys it needs (the parser refuses unknown keys and the respawn loop runs forever at 2 s).
3. `rm maburd`, stop, copy with `cat f | ssh ld 'cat > /tmp/f'`, check sha256 against `PAIR.txt` before the move into place (a pipe once landed an empty `mabur.toml`), start with `setsid`.
4. Print the boot stamp and compare it with `PAIR.txt`.
5. Keep the previous binary and config on the drone for roll-back (an old binary needs its old config).

The script never flashes, never touches MTD, and is run by the operator, not by CI.

## 9 Testing and acceptance

1. Unit: golden vectors for wire, FEC, SBI, profile (from mabur).
2. Differential replay: the same capture (section 7 format) through `pal-ref` and the Rust core; AU bytes, AU flags and sent RC bytes must be identical (`rz_nonce` pinned). This is the gate for every core change. Captures: a clean bench run, a 20 % body-loss run, a drone restart, a key mismatch; a channel move joins when the channel core is ported (M4). Before live captures exist (M0), maburd `--dry-run --out` frame files with `FrameFileSource`'s synthetic per-card loss stand in.
3. FEC bench on the host and on the phone's CPU: symbols/s and worst-case decode time per body (`crates/pal-fec/examples/fec_bench.rs`, self-timed with no dependency so the `tools/android-build.sh` build runs from `/data/local/tmp`; resolved 2026-11-10 except Rust vs C++, which needs a C shim for mabur's decoder: pal-ref wraps WebGs only).
4. Hardware, before any "replaces PixelPilot" claim: G2G median/p95 on the LED rig, RCF heard %, truncated-AU rate over a fixed 10 min bench run, both apps on the same phone, drone and rung.
5. CI: fmt, clippy `-D warnings`, `cargo test --locked`, the parity gate, an APK build. Taken from openipc-rs's CI, minus the platforms we do not ship.

## 10 Milestones

| M | Content | Exit criterion (measured) |
|---|---|---|
| M0 | Repo, mabur + devourer submodules (done 2026-10-31), `pal-ref` builds and wraps WebGs behind `GsCore`, golden vectors wired as tests, maburd `--dry-run` frame files as the first replay input, CI release job producing the maburd + `PAIR.txt` half of the pair, Quest spike (extension list, one capture decoded through the surface swapchain, USB permission without a dialog) | `pal-ref` on the dry-run files matches `webgs replay` output byte for byte (this checks the wrapper, not the port); the CI maburd's sha256 is reproducible across two runs |
| M1 | `pal-wire`, `pal-fec`, FrameStream; `pal` live RX on Linux via devourer, Spotter mode, dump `.h265`, record captures (section 7); record the 4 bench captures of section 9 | AU byte parity with `pal-ref` on the dry-run files and the live captures; ffplay shows the dump |
| M2 | Native Linux GS: full GS mode on 1..N cards with a pinned channel (F1-F7, F9-F11, F21), egui OSD, flight log | flies the quad from a laptop; AU parity with `pal-ref` holds on live captures; RCF heard % within PixelPilot's number at the same spot |
| M3 | Phone app and Quest app: MediaCodec path (F8), Android USB, OpenXR (F19), the same core and OSD | flies the quad from both devices; G2G and RCF heard % within PixelPilot's numbers on each; Quest: no repeated or dropped video frame over 10 min at 120 fps; with two cards, truncated-AU rate at range lower than with one (same flight path, A/B) |
| M3b | mabur fork with the latency block (13.3) in a matched release; t5-t7 stamps in all apps; `au.log` carries the stage chain | one bench session where the chain's t0-t7 matches the G2G rig, median within 3 ms (13.4) |
| M4 | DVR, MAVLink widgets, per-card UI, channel core, VTX rec, preflight (F12-F17, F22) | daily driver: one week of sessions without opening PixelPilot |
| M5 | Param client, MSP OSD, Quest hand tracking | per feature |

`pal-ref` is test-only (decision 3): no milestone flies on the C++ core.

M0 status (2026-10-31):
- Done: Cargo workspace; `pal-wire` crc16 and profile pass mabur's golden vectors; `pal-ref` builds `mabur_web_core` from the submodule with mabur's own CMake and wraps WebGs behind `GsCore` through a C ABI shim; `tools/m0-parity.sh` passes 9 cases (spotter fixed/adaptive gap, gs with fake ack; drop 0/5/15 %), AU files byte-identical to `webgs replay`, send counts equal, and a changed input is detected (negative check). `tools/build-maburd.sh` builds maburd from the submodules with the pinned OpenIPC toolchain (sha256 `0588f40e...`); two clean local builds gave the same sha256 (`d309a968...`), and no build path is embedded in the binary. CI (`.github/workflows/ci.yml`) green on run 41227350241: fmt, clippy, tests, parity, and two runner builds of maburd with the same sha256 as the local builds (`d309a968...`).
- Limits of the M0 gate: the dry-run fixture is 13 AUs; sent RC bytes are compared by count, not content (webgs replay prints only counts).
- Live check (drone on, maburd build 7bab3194 = pin plus debug-only files, no wire change): `webgs live --mode spotter` on the host 8812AU received 582 AUs in 20 s, ch 144/40 MHz, MCS3, rssi -25.5 dBm, residual loss 0.
- Quest spike (the probe app `spikes/quest-xr-probe`, in the tree up to tag `v0.11` and removed after it: `apps/quest` replaces it; Quest 3, runtime Oculus 207.190.0), 2026-10-31:
  - `XR_KHR_android_surface_swapchain` works; the runtime accepts only usage 0 and sampleCount/faceCount/arraySize/mipCount 0. 1080p60 H.265 from NDK MediaCodec into that surface on a quad layer: 60 fps, queue-to-output p50 3.7 ms, p95 about 5 ms; picture confirmed in the headset. The quad needs `XR_FB_composition_layer_image_layout` VERTICAL_FLIP (decoder surfaces are top-down).
  - 120 Hz granted (72/80/90/120 offered), 118-120 XR frames/s.
  - android-activity 0.6 puts the Application in ndk_context and openxr 0.19 passes it as applicationActivity; the runtime then holds the session in IDLE forever. The app must re-initialise ndk_context with the Activity before xrCreateInstance.
  - USB: a UsbManager scan from Rust over JNI sees 0bda:a81a; with permission it gets an fd (`openDevice().getFileDescriptor()`, 71 B raw descriptors), which is what `libusb_wrap_sys_device` takes. Permission without a dialog comes from the USB_DEVICE_ATTACHED filter plus the user's one-time "always" choice (as PixelPilot does today); for development the Horizon OS shell command `cmd usb grant-device-permission <pkg> 3034:43034` grants it persistently. The attach-chooser path is verified on the Quest for our package: after one "always" choice, a replug launches the app with USB_DEVICE_ATTACHED and permission is already granted, no dialog. The `cmd usb` grant does not survive replugs. Horizon OS traps: the chooser panel is a fixed 471 px and with four candidate apps its button bar collapses to 9 px with no buttons, so nothing can be confirmed; disabling a package clears its USB defaults; there is no settings UI to clear a default. The device filter must list RTL88xx VID:PIDs with a `0x` prefix: PixelPilot's ids without `0x` appear to be dropped as invalid decimals, which would make its filter match any device (it holds the default for an unrelated USB-C video adaptor).

Next steps (set by Ihor, 2026-11-01, after two-card support landed in 8bb9f5c), both done 2026-11-02:
1. TxSelector (F1), done in 1182a86: `pal-core::tx_selector` (mabur's unit tests), RCF/DISC/NACK/MAV/cal
   frames and the channel core's tick go to the selected card; stats `tx_card`/`tx_switches` and
   the OSD cards chip show it. One card stays card 0, replay parity unchanged. Not yet seen on air.
2. Calibration channel pick (docs/calibration-port.md row C2), done: `cal_round_pick` and the
   scout's measure round (6291e7f), the ChannelCore measure round and cal relocate (102596e), `CalSession` Picking through
   `cal::picker::CalPicker` (ChannelCore is the real one, `FixedPicker` without a channel core), the
   panel shows the pick with an Abort. Not yet run on air (needs the field, at distance).

## 11 Rough size

C++ units the Rust core replaces (h+cpp lines, pinned tree): rc_proto 787, sbi 140+, uep_decoder 162+, sw_decoder 451, gf256 325, aggregator 443, frame_stream 460, vrx_controller 345, rendezvous 141, rcf_slot 287, ladder_controller 1135, link_health 479, mav_tunnel 312, param_client 574. About 6k lines for M1-M2. The channel core (channel_core 1448, channel_scout 416, hop_*) is M4. PixelPilot's `MaburReceivePath` alone is 7.2k lines plus 3.1k of tests; most of that is unify-only and is not ported.

## 12 Decisions and open questions

Decided 2026-10-31:
1. **Licence:** not a blocker; gilankpam is a friend of the project. Still ask for a LICENSE file in mabur before anything derived from it goes public. Palianytsia itself links devourer (GPL-2.0, read as GPL-2.0-only), so it is GPL-2.0; `cargo-deny` in CI rejects Apache-2.0-only dependencies.
2. **Pins:** branch `palianytsia/base` in both public forks, pushed 2026-10-31 (superseded on 2026-11-03: mabur moved to `palianytsia/base-v2`, a rebase onto upstream master c8e2d20e; current pins: `git submodule status`):
   - `vertexodessa/mabur` @ `51cea32f`: gilankpam master `f901ee2a` + 4 commits (maburd build stamp, toml list, `venc.mirror`/`venc.flip`, MSP FC telemetry over T_MAV), RC 18, the build the quad flies.
   - `vertexodessa/devourer` @ `5deca84`: identical to gilankpam/devourer master.
3. **Core strategy C:** the Rust core is the product; `pal-ref` (C++ WebGs) is a test-only oracle and never ships.
4. **No upstream for now.** Revisit when something would be cheaper upstream than carried in the fork.
5. **Native Linux GS first** (5.7), then the phone and the Quest.

Still open:
6. Phone UI: Kotlin shell with two surfaces (start here) or NativeActivity. Decide on the G2G A/B when the phone app is built.

Known issues, deferred by Ihor (2026-11-01):
7. **Grey picture on loss, CPU decode.** libavcodec's software HEVC decoder fills a missing reference with grey, so on a lossy link the desktop picture goes grey until the next intra refresh; vdpau and MediaCodec keep the last picture. Seen on `2026-10-31-bench-gs-ch40w40-idr.bin` with `--hwdec sw`, same with the system FFmpeg. Fix: hold the last clean frame while the decoder reports missing refs.
8. **Desktop GPU decoders.** `--hwdec cuda` segfaults at exit (`cuCtxPopCurrent failed` in teardown); `--hwdec vaapi` stalls after 4 frames on NVIDIA's VA-API wrapper. Both reproduce with the system FFmpeg; `auto` picks vdpau first and is unaffected.
9. **The APKs ship pal-ref.** Resolved 2026-11-01: the APKs run the Rust port of ChannelCore (`docs/channel-core-port.md`, level 4) and pal-ref left the Android build.

## 13 Latency measurement (needs the mabur fork)

### 13.1 What original mabur gives today

- `FrameHdr.pts_us`: the capture stamp, u32, on the drone clock.
- SBI header, per body: `q_ms`, `enc_us`, `air_ms`, latched from fragment 0.
- `RttEstimator`: the drone-to-GS clock offset from the min-RTT RCF/Telem pair, which gives `cap_to_complete_us` (capture to AU complete) at the GS.

That is one number per frame, and it ends at "AU complete". It cannot answer where the time went: encoder, TX queue, USB, air, FEC wait, decoder, compositor.

### 13.2 The stage chain

| Stage | Clock | Where it comes from |
|---|---|---|
| t0 capture | drone | `pts_us` (exists) |
| t1 encoder output | drone | fork: per-frame stamp |
| t2 TX submit of the frame's first and last body | drone | fork: per-body stamp in the SBI header, written by the TX thread at USB submit |
| t3 first body received, t4 AU complete | GS | `RxBody.mono_us`, FrameStream (exist) |
| t5 decoder input, t6 decoder output | GS | MediaCodec queue/dequeue stamps; libavcodec send/receive |
| t7 present | GS | phone: Choreographer frame time for the frame that latched it; Quest: `predictedDisplayTime` from `xrWaitFrame` for the first refresh that shows it; Linux: Wayland `wp_presentation` feedback where available, else the swap time (an estimate, marked as such) |
| t8 photons | rig | G2G LED rig (ESP32-C3), bench only |

The drone-to-GS clock offset is the one `RttEstimator` already computes. The air link is half-duplex and roughly symmetric, so the min-RTT/2 error is bounded by the RTT asymmetry. Record it, do not hide it.

### 13.3 Fork changes (vertexodessa/mabur)

1. A per-frame latency block behind a `FrameHdr` flag. The unify tree has it as `kFlagLat` + `FrameLat`, 32 B (`exp-mabur-fluke-pxp/mabur/common/include/mabur/frame_wire.h:37-49`); port the idea, not the wall-clock fields. Use drone-mono stamps for t1, plus t0 widened to u64.
2. The per-body t2 goes in the SBI header, outside the FEC envelope. Check first whether any CRC covers the SBI header (the per-block CRC16 covers the blocks); a corrupted t2 must be detectable, or it must be guarded by its own check. The unify design learnt this the hard way: a stamp inside the envelope cannot be rewritten at USB submit without breaking FEC parity, so its `d6` equalled `d4` and air time was not measurable per frame (`frame_wire.h:43-48`).
3. Cost: 32 B per frame at 120 fps is about 31 kbit/s. A 4 B t2 per body at about 710 bodies/s (8 Mbit/s in 1405 B bodies) is about 23 kbit/s. Together under 0.7 % of an 8 Mbit/s rung. Always on, no config key.
4. RC_VERSION bump, both ends in one matched release (section 8). Offer it upstream to gilankpam as a PR; until it is merged, the fork is the wire.

### 13.4 Trust rule

The in-band chain is a model; the rig is the truth (the lesson behind "encoder telemetry is not the glass").
- Before any latency claim relies on the chain, run one bench session with both on. The chain's t0-to-t7 must match the rig's G2G minus the rig's fixed display and sensor offsets, median within 3 ms.
- Re-check after any change to the decoder, the compositor path or the clock code.
- The flight log records every stage per frame (`au.log`, section 7), so a post-mortem can say "the 180 ms spike was 150 ms of TX queue", not just "latency went up".

## 14 Long-term direction: a mabur rewrite

The current mabur code is hard to maintain; the plan is to rewrite it later, drone side included. Palianytsia goes first and is built so that the rewrite can grow out of it:

- `pal-wire` and `pal-fec` are symmetric: they encode as well as decode, have no GS-only assumptions, and are `no_std + alloc`, so they build for the armv7 drone. The FEC encoder is tested against the C++ encoder's golden vectors from the start, even though the GS does not need it.
- `pal-core` stays sans-IO, so the drone-side controller can be built the same way.
- A written wire spec (planned as docs/wire.md, not written yet) lives in the Palianytsia repo, derived from the code and checked by the golden vectors. Today the wire exists only as C++. The document becomes the contract once a Rust drone replaces maburd.
- After the rewrite, `pal-ref` (the C++ oracle) retires, and the wire may drop what the C++ carries for history (for example two 802.11 frame types for drone frames).
- Order: GS (this spec), then drone. Exit criterion for the drone rewrite: the same loss, ladder behaviour and latency chain numbers as maburd on the bench and in one flight, measured with section 13.
- Risk to manage: maburd holds hardware knowledge that is not in any interface (MI VENC quirks, IDR storms, the txq IDR gate, the devourer bring-up race). The rewrite carries it from `mabur/docs/*findings*`, the vault and file memory, read before the matching module is written, not rediscovered in flight.

M1 working notes (2026-10-31; devourer-sys, pal-wire::dot11, flightlog capture, `pal live/replay` landed):
- `devourer-sys` is a slim copy of mabur's `RadioFrontend::open_and_start` (`gs/src/radio_frontend.cpp:40-190`): libusb open by VID:PID list or fd, `devourer::claim_interface_then_reset(..., do_reset=true)`, DeviceConfig {rx.enable_with_tx, rx.keep_corrupted, rx.abs_noise_floor, tuning.disable_cca=false, tx.no_cancel_multipkt, usb.rx_zerocopy=false}, `CreateRadio` + `InitWrite(SelectedChannel{ch, ht40_offset, width})`, `StartRxLoop` on its own thread. The C callback hands up the raw dot11 frame plus RxAtrib (crc_err, data_rate, rssi[2], snr[2], evm[2], physt, tsfl); stamping mono_us happens in the callback.
- dot11 -> RxBody is Rust (`pal-wire::dot11`), a port of `gs/src/dot11.cpp` `fill_rx_body`: body offset 26 for fc 0x88 else 24, Foreign = CRC-good with SA != 57:42:75:05:d6:00, mcs from data_rate 0x0C..0x13 or 0x80..0x87 else 255, mac_seq = seq_ctl >> 4.
- Then `pal live --spotter --capture bodies.bin --dump out.h265` (AUs through pal-ref first), `pal replay bodies.bin` through both cores.
- Bench result: spotter on ch 144/40 (1793 bodies, 300 AUs, 0 truncated in 20 s) and GS mode (`pal live --mode gs`, host card sends the core's RCF) on ch 40/40: session up in under 1 s, 1080 AUs / 40 s, 66 truncated, IDR + VPS/SPS/PPS present, ffmpeg decodes all 1080 frames 1920x1080. `pal replay` reproduces both AU files byte for byte (`tools/m1-replay.sh`, in CI).
- Trap found: the pal-ref shim used `radio.channels.front()` as the core's current channel. With the card on 144 and `channel = "auto"` the core believed it was on 40 and steered the drone to 40, away from the card (session never formed, video stopped after ~10 s). Fix: `RefOpts.channel` carries the tuned channel; `configs/gs.toml` pins `channel = 144` until devourer-sys can retune (a core hop has no LinkCard to move). `pal live --ch` must equal the pin, else the core moves the drone off the card.
- Drone stays on 40 after that first run until its link times out or it restarts; the GS capture was taken there with a temporary config pinned to 40.

M1 status (2026-11-01): the Rust receive core runs (Spotter).
- Ported and checked against mabur goldens: `pal-fec` (GF256, SwDecoder, ArrivalTracker), `hevc` (NAL, SPS/PPS, slice header, skip slice), `pal-wire` rc/siphash/sw/sbi/frame_wire. `pal-core`: config loader (strict keys, ranges, defaults; loads `configs/gs.toml` and mabur's bundle), UepDecoder, Aggregator, FrameStream, SliceAssembler, GapTimeoutPolicy, DroneRestartDetector, `RustCore` wired as WebGs wires them.
- `pal replay --core rust|ref` and `pal-ref-replay --core rust`; `pal replay --drop-pct N --seed S` drops bodies with a seeded splitmix64, so both cores see the same survivors.
- Parity, all byte-identical: both live captures, ref vs Rust in Spotter at 0/10/20/35 % loss (`tools/m1-replay.sh`); maburd dry-run files against `webgs replay` itself, fixed and adaptive gap, 0/5/15 % (`tools/m0-parity.sh`); a 240-frame row-sliced 1080p dry-run input built from mabur's `slices4_1080p.h265` (`tools/slices-frames.py`) at 0/10/20/30 %, where slice salvage rebuilds 8, 36 and 14 AUs.
- The live GS capture never exercises salvage: its one IDR AU (VPS/SPS/PPS) arrived truncated, so FrameStream never got usable parameter sets (all 215 truncated AUs at 10 % loss fall back NoParams, in both cores). The sliced dry-run input covers salvage instead.
- Not in the Rust core yet: Spotter RCF follow, MSP OSD. (GS mode landed in M2, below.)
- Live captures 2026-11-01 (git-ignored, `captures/README.md`): salvage on real air (7 of 11 truncated AUs salvaged), maburd restart mid-run (session drops at 16 s, re-forms at 30 s), key mismatch (`key_mismatch` true in both cores). All ref==rust. Still owed: 20 % real loss (needs physical attenuation; no injector in original mabur, `wall_margin_db` caps at 6 dB).

M2 status (2026-11-01): the Rust core runs GS mode.
- Ported: VrxRendezvous + VrxController (DISC/keep-alive, RCF build, probe profile), LadderController + RungStore, LinkHealthAssembler (S1 loss window, transition edge, probe track), RcfSlotter, RttEstimator, UplinkDelivery, NackTracker, MavTunnel, ParamClient with the parameter table and value codec, and a MAVLink 2 framer (`pal-wire::mavlink`: `mavlink_frame_char_buffer` state machine, CRC_EXTRA table generated from mabur's `mavlink/mabur/mabur.h`, packer with MAVLink 2 zero-trim). `RustCore` wires them as WebGs does with no cards: frame-wire gate on SESSION + CAP_FRAME_WIRE, AU begin/end into the slotter and probe expectation, probe bodies into the slotter tail and LHA, telem into RTT/uplink/NACK counters, T_MAV into the tunnel and ParamClient, and per tick NACK poll, MAV flush through the slotter, LHA tick, `vrx.step`, slotter release.
- `pal replay --sends FILE` and `pal-ref-replay --sends FILE` write every transmitted RC body (u32 len | bytes).
- Parity, all byte-identical, AUs and sent bytes: the live GS capture in GS mode at 0/10/20/35/50 % loss (962 sends at 0 %: 795 RCF, 121 NACK, 42 DISC, 4 MAV; the ladder moves under loss), the Spotter capture in GS mode (952 DISCs, no session), and the maburd dry-run with and without the fake DISC_ACK at 0/5/15 % (`tools/m1-replay.sh`, `tools/m0-parity.sh`, both in CI).
- The one fix the comparison found: with `channel = 0` the shim proposes `radio.channels[0]`; RustCore now does the same.
- Flight log (`flightlog::session`, `pal live|replay --log-dir DIR [--capture-mb N]`, `pal log DIR`): `fl-NNNN_<yyyymmdd-HHMM>` (UTC) with `stats.jsonl` (one stamped `stats_json` line per core-clock second), `events.jsonl` (changes of session, key_mismatch, drone_state, rung, mcs, bw, channel, resets, derived from consecutive stats lines so both cores produce them; capture_cap, capture_seg), `au.log` (t_us sid bytes flags complete salvaged pts_us) and an optional size-capped capture (`bodies.bin` plus tail segments; `pal replay DIR` plays them in order); each file opens with a versioned format marker. Not yet: decode-in/render stamps in `au.log` (no decoder in pal yet), `pal log --adb`, a check that mabur's `flightreport.py` reads `stats.jsonl`.
- Not ported: ChannelCore/scout/hop (M4, needs cards), MSP OSD, the calibration MAV lane. Still owed for M2: `pal live --core rust` / `pal-desktop --live` flown against the quad (needs the drone), VA-API and NV12 upload, MAVLink telemetry widgets, settings.
- Decode: crate `decode`, libavcodec HEVC through a C shim (LOW_DELAY, slice threads, newest frame only, RGBA via swscale). The live GS capture decodes 1080/1080 AUs, 0 errors.
- Desktop: `apps/desktop` (`pal-desktop`), eframe on glow for now (wgpu later), `pal-osd` widgets from the stats line (link, signal, loss, video, drone, tx chips; 60 s SNR/RSSI/truncated/rung plots). Threads: card or 1x capture replay + core, decode, UI; AUs through an 8-slot queue (a 2-slot one dropped 6 AUs at start and broke references). `--snapshot out.ppm --secs N` renders one window and quits: on the GS capture at 25 s it shows the same picture as ffmpeg's decode of the AU dump, 0 drops, 0 decode errors. The capture starts with a truncated IDR, so its first ~10 s are grey in every decoder.
- 2026-11-01, on the real rig: `pal live --core rust` and `pal-desktop --live` hold a session with the quad (30 fps, 0 decode errors, MAVLink telemetry widget live). The Rust stats line is now `webgs::stats_json` byte for byte (nlohmann number spelling, `pal_core::json`), compared by `tools/m1-replay.sh`; LHA runs in Spotter too, as in WebGs. F7 IDR requests: `refresh::RefreshRequester` (port) + `IdrDriver` in both apps (base damage, decoder errors, mid-stream start), `GsCore::set_idr_requests` in both cores; on air a start request healed by the drone's IDR in 61 ms. Still owed for M2: VA-API/NV12, settings, default-key warning in the UI, N cards.
- 2026-11-01, later: F5 key banner in pal-desktop (red on `key_mismatch`, amber while `key_fp` is "default"), OSD chips for pre/post-FEC loss, uplink % + RTT, drone temp, NACK fill. Decode: libavcodec hwaccel by name (`--hwdec vdpau|vaapi|cuda`, software when the device does not open); NV12 goes through I420 before swscale, which has no fast NV12->RGBA path. `decbench` on the restart capture (871 frames 1080p, RTX 3060): decode + RGBA 5.7 s CPU software vs 2.7 s vdpau; decode alone 4.9 vs 0.33. VA-API is untested (no VA driver for NVIDIA on the dev host), and NV12 still goes through a CPU RGBA conversion, not a texture upload. F18 settings: `GsCore::params_json` / `param_set` / `param_refresh` in both cores (`webgs::params_json` byte for byte; `tools/m1-replay.sh` compares the table and a `--param-set` script), settings window in pal-desktop (P). Still owed for M2: N cards (F1), NV12 texture upload.
