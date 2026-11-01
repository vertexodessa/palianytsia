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
   timelines. As built: `cpp/chan_trace.cpp` hosts the C++ core over
   `LinkCard`s that call back into Rust, so both cores run on the same
   `chan::fake::FakeCard` type (feature `fake`). After every step the sink
   records, card calls, tick outputs and vrx frames are compared; doubles
   print with their bits, so equal means bit-equal. The live-capture
   timelines are not scripted yet.
3. Live: desktop `--core rust` with an auto config against the quad, then
   the APKs switch to the Rust core and pal-ref leaves the Android build.

## Order (bottom-up, each with its tests)

| Level | Modules (lines of C++) | Status |
|---|---|---|
| 0 | scout_pick (87), channel_ranker (144), s1_loss (141), width_resync (35), scout_radio (80), nhm_busy, nhm_window, snr_units, channel_plan (259), hop_verdict (320) | done: `chan::{scout_pick,ranker,radio,plan,verdict}`, s1_loss was already in `link_health.rs`; 87 tests |
| 1 | pair_pick (171), hop_ranker (171), hop_blank (80), hop_controller (537) | done: `chan::{pair,hop_ranker,hop}` (hop_blank lives in `hop`); 76 tests |
| 2 | hop_burst_gate (117), boot_pick (241), channel_scout (598), window_sampler (262), inflight_scout (246) | done: `chan::{gate,boot,scout,sampler,inflight}`; calibration parts and relay survey not ported; 79 tests |
| 3 | channel_core (1448), the WebGs wiring in `rust_core.rs`, the trace harness | core done: `chan::core`, 36 tests (cal and relay cases dropped; the cal move-edge case kept as the hop hold); trace harness done: `crates/pal-ref/tests/chan_trace.rs` (12 scenarios + 200 random scripts, every record bit-equal); WebGs wiring done: `RustCore::with_cards` (ChannelCore hooks as web_gs.cpp: on_rc_body, is_link_video/note_video, note_au_end, on_session_opened, the per-tick block, DISC fan-out through `disc_targets`/`disc_for_card`, the `may_send` gate, sends through the card, the stats `scan_*`/`hop` section), `runtime::dev_card::DevCard` (PalCard in Rust over new devourer-sys `Ctl` calls, OwnAirAcc ported as `chan::own_air`), `rust_channel_core` attach; pal-desktop `--core rust` with an auto config uses it instead of the runtime sweep (a Rust spotter still sweeps). No cards = no channel core, so the parity gates are unchanged |
| 4 | live test, APKs on the Rust core, pal-ref out of the Android build | desktop live on the bench quad (2026-11-01, 30 s each): Rust and C++ cores log the same lines (prelude picks 144, commit 40 -> 144, pick frozen after 4 rounds); Rust: session at 7.1 s, 691 AUs, 0 truncated; ref: 663 AUs, 2 truncated. APKs attach `rust_channel_core`, pal-ref left the Android dependency tree; Quest 3 with the 8812EU (RTL8822E), release APK, 58 s: same channel-core lines (prelude picks 144, pick frozen after 4 rounds), session at 7.09 s, 3086 AUs, 0 truncated, rung 3, rtt 6.8 ms (min 3.9) |
| 5 | review pass over the ported code: simplify, improve structure, hunt bugs (Ihor, 2026-11-01); findings that are also mabur bugs go to the fork | |
