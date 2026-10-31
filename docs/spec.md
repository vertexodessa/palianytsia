# Palianytsia: a Rust ground-station client for mabur

Status: Draft 2, 2026-10-31 (Draft 1 reviewed: stale text after the desktop-first and strategy-C decisions fixed; capture format, maburd chip set and GS radio config corrected against the pinned tree). Name: **Palianytsia** (repo `palianytsia`, crate prefix `pal-`, CLI `pal`).
The name is a choice, not decoration: the word is the 2022 shibboleth used to tell a Russian speaker from a Ukrainian one, and no Russian military unit will want it on a ground station.
Paths like `exp-orig-backport/...` and `exp-mabur-fluke-pxp/...` refer to the devourer_stack workspace on the author's machine, where this draft was written.
Home: `vertexodessa/palianytsia`, a **private** GitHub repo (not OpenIPC, not this workspace). The mabur and devourer forks it pins stay public.

Sources read for this draft:
- mabur, original tree with backports, now pinned as `third_party/mabur` @ `51cea32f` (`RC_VERSION = 18`, `common/include/mabur/rc_proto.h:68`). This is the wire the quad flies today. Draft 1 read the same tree at its child 7bab3194, which only adds a bench debug endpoint.
- PixelPilot, unify fork `exp-mabur-fluke-pxp/pixelpilot` (e2d0c90), and the backport tree `exp-orig-backport/pixelpilot` (33978f8).
- openipc-rs `devourer_stack/openipc-rs` (v0.1.40, upstream last commit 2026-07-10).

The unify tree (RC 21, `T_CFG_SET`/`T_LOG`, `phone_wall_us`) is a reference only. This app does not speak it.

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
- The calibration lane, the CPE510 relay radio, `MABUR_LOSS_SIM`. The `Card` trait leaves room for a relay later.
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
pub trait GsCore {
    fn on_rx(&mut self, body: &RxBody);          // body.mono_us = core clock
    fn tick(&mut self, now_us: u64, io: &mut dyn CoreIo);
    fn stats(&self) -> Stats;
    fn set_idr_requests(&mut self, n: u32);
    fn set_vtx_rec(&mut self, on: bool);
    fn mav_push_outbound(&mut self, bytes: &[u8]);
    fn mav_take_inbound(&mut self, out: &mut [u8]) -> usize;
}
pub trait CoreIo {
    fn on_au(&mut self, au: Au);
    fn send(&mut self, rc_body: &[u8]) -> bool;  // false = gated, never handed to a card
    fn on_log(&mut self, line: &str);
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
- `XR_FB_display_refresh_rate`: request the rate that the video fps divides evenly (120 Hz for 120 or 60 fps, 90 Hz for 90 or 30 fps). Re-request when the drone's camera mode changes (fps is known from the AU cadence or the encoder param).
- The XR frame loop (`xrWaitFrame`/`xrBeginFrame`/`xrEndFrame`) runs on its own thread and never waits for video: it submits whatever the swapchain holds. Video arrival and display refresh are decoupled, so a late AU repeats one refresh and never blocks the loop.
- Measured as: refreshes at video rate with no repeated or dropped video frame over 10 min at 120 fps (`XR_FB` performance counters or the compositor's frame stats), and G2G on the LED rig against PixelPilot on the same Quest.

Placement:
- Goggle mode (default): the quad sits in `XR_REFERENCE_SPACE_TYPE_VIEW`, head-locked like real FPV goggles, size and distance adjustable (PixelPilot's margin and distance sliders, made into quad size and z).
- Screen mode: world-locked in `LOCAL` space, a large screen for spotters.

OSD and menus:
- A second quad layer from a small GL swapchain. egui draws it at 30 Hz or on change, never at the video rate. The same widgets as the phone.
- Input: controller buttons for the common actions (record, IDR, menu, recenter), with ray-cast pointer on the menu panel. Hand tracking is P2.

Quest specifics:
- USB: one USB-C port, so cards plus charging go through a powered hub. Get USB permission without a dialog in immersive mode: a `USB_DEVICE_ATTACHED` intent filter with the device filter, so the system grants on attach. Verify on the device in the M0 spike; a permission dialog that cannot be seen from inside the headset is the failure to rule out.
- Keep the link up when the headset is lifted (the proximity sensor pauses the session). The radio and core threads ignore XR session state, and only the frame loop stops. PixelPilot learnt the same lesson with its 30 s hide.
- Development over wireless adb (`192.168.178.77:5555`).

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
- Optional raw capture (`bodies.bin`, size-capped): every `RxBody` as the core saw it (card id, mono stamp, CRC flag, RX metadata, body bytes), plus every `tick` time. This is the replay input for section 9. mabur's own replay format is not enough: `webgs replay` and `maburgs --dry-run` read a maburd `--dry-run --out` frame file (u32 length + radiotap + dot11 + body, `gs/src/frame_file_source.h`), which is TX-side and synthetic: no RX stamps, no real loss, no CRC-bad frames, no RSSI. The differential test feeds our capture to both cores directly through the `GsCore` trait, so no C++ reader is needed. Turning on a capture in the field is how a bug becomes a test.
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

Release assets for tag `vX.Y.Z`:

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
3. FEC bench on the host and on the phone's CPU: symbols/s and worst-case decode time per body, Rust vs C++ (`criterion`, run on device via `cargo ndk`).
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

## 11 Rough size

C++ units the Rust core replaces (h+cpp lines, pinned tree): rc_proto 787, sbi 140+, uep_decoder 162+, sw_decoder 451, gf256 325, aggregator 443, frame_stream 460, vrx_controller 345, rendezvous 141, rcf_slot 287, ladder_controller 1135, link_health 479, mav_tunnel 312, param_client 574. About 6k lines for M1-M2. The channel core (channel_core 1448, channel_scout 416, hop_*) is M4. PixelPilot's `MaburReceivePath` alone is 7.2k lines plus 3.1k of tests; most of that is unify-only and is not ported.

## 12 Decisions and open questions

Decided 2026-10-31:
1. **Licence:** not a blocker; gilankpam is a friend of the project. Still ask for a LICENSE file in mabur before anything derived from it goes public. Palianytsia itself links devourer (GPL-2.0, read as GPL-2.0-only), so it is GPL-2.0; `cargo-deny` in CI rejects Apache-2.0-only dependencies.
2. **Pins:** branch `palianytsia/base` in both public forks, pushed 2026-10-31:
   - `vertexodessa/mabur` @ `51cea32f`: gilankpam master `f901ee2a` + 4 commits (maburd build stamp, toml list, `venc.mirror`/`venc.flip`, MSP FC telemetry over T_MAV), RC 18, the build the quad flies.
   - `vertexodessa/devourer` @ `5deca84`: identical to gilankpam/devourer master.
3. **Core strategy C:** the Rust core is the product; `pal-ref` (C++ WebGs) is a test-only oracle and never ships.
4. **No upstream for now.** Revisit when something would be cheaper upstream than carried in the fork.
5. **Native Linux GS first** (5.7), then the phone and the Quest.

Still open:
6. Phone UI: Kotlin shell with two surfaces (start here) or NativeActivity. Decide on the G2G A/B when the phone app is built.

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
- A written wire spec, `docs/wire.md`, lives in the Palianytsia repo, derived from the code and checked by the golden vectors. Today the wire exists only as C++. The document becomes the contract once a Rust drone replaces maburd.
- After the rewrite, `pal-ref` (the C++ oracle) retires, and the wire may drop what the C++ carries for history (for example two 802.11 frame types for drone frames).
- Order: GS (this spec), then drone. Exit criterion for the drone rewrite: the same loss, ladder behaviour and latency chain numbers as maburd on the bench and in one flight, measured with section 13.
- Risk to manage: maburd holds hardware knowledge that is not in any interface (MI VENC quirks, IDR storms, the txq IDR gate, the devourer bring-up race). The rewrite carries it from `mabur/docs/*findings*`, the vault and file memory, read before the matching module is written, not rediscovered in flight.
