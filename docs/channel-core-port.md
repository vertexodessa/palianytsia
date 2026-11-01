# ChannelCore in Rust (port plan and status)

Goal: retire `pal-ref` from the APKs (spec 12, item 9). The channel search
(`channel = "auto"`) runs mabur's C++ ChannelCore through pal-ref today; the
Rust core gets its own port, held byte-identical to the C++ the same way the
rest of the core is (ref == rust).

## Scope

In: USB cards only, one card (phone, Quest) and several (desktop): boot
prelude and scout, boot pick, relocation (`auto_want_pick`), in-flight hop
(orders, verify, withdraw, backoff), the scout gate on sends, DISC retagging,
width resync, the window sampler, the channel store.

Out (no such hardware in Palianytsia): relay cards (`can_sweep`, CPE510,
`relay_sweep_map`, relay burst), the calibration hooks (`cal_*` on
ChannelCore; Palianytsia does not run maburcal). A config that asks for them
is refused at start, never silently ignored.

## Parity

1. Unit level: each module is ported with its mabur test file
   (`third_party/mabur/tests/test_<module>.cpp`) translated case by case into
   Rust tests. A case that does not translate (relay, cal) is listed here.
2. Core level: a trace harness. ChannelCore runs unthreaded
   (`ChannelCoreCfg::threaded = false`, injected `now_ms`/`now_us`/`sleep`,
   `run_scout_step` / `run_inflight_step` / `run_sampler_step`) against a
   scripted fake card (per-channel energy, NHM, own/foreign frames, sends
   recorded). The same script drives the C++ core through a pal-ref shim and
   the Rust port; every `LinkCard` call and every `ChannelSink` event is
   written to a trace, and the two traces must be equal. Scenarios: the
   mabur `test_channel_core.cpp` rigs plus the live captures' channel
   timelines.
3. Live: desktop `--core rust` with an auto config against the quad, then
   the APKs switch to the Rust core and pal-ref leaves the Android build.

## Order (bottom-up, each with its tests)

| Level | Modules (lines of C++) | Status |
|---|---|---|
| 0 | scout_pick (87), channel_ranker (144), s1_loss (141), width_resync (35), scout_radio (80), nhm_busy, nhm_window, snr_units, channel_plan (259), hop_verdict (320) | done: `chan::{scout_pick,ranker,radio,plan,verdict}`, s1_loss was already in `link_health.rs`; 87 tests |
| 1 | pair_pick (171), hop_ranker (171), hop_blank (80), hop_controller (537) | done: `chan::{pair,hop_ranker,hop}` (hop_blank lives in `hop`); 76 tests |
| 2 | hop_burst_gate (117), boot_pick (241), channel_scout (598), window_sampler (262), inflight_scout (246) | done: `chan::{gate,boot,scout,sampler,inflight}`; calibration parts and relay survey not ported; 79 tests |
| 3 | channel_core (1448), the WebGs wiring in `rust_core.rs`, the trace harness | |
| 4 | live test, APKs on the Rust core, pal-ref out of the Android build | |
| 5 | review pass over the ported code: simplify, improve structure, hunt bugs (Ihor, 2026-11-01); findings that are also mabur bugs go to the fork | |
