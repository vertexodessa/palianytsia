# TX-power calibration in the Rust GS (a "Calibrate" button)

Why: the bench quad (`ld`, RTL8822E) flies `power_mode = "none"`, so every
MCS airs at the card's default TX power. MCS4 arrives with an EVM floor of
about -14.5 dB at any RSSI from -20 to -70 dBm, on two different receivers
(8812AU on the desktop: 0 of 213 clean; 8822E on the Quest: 17 % clean),
while MCS0-3 are clean; the ladder's probe gate therefore never opens
rung 4 (2026-11-01, Quest logs fl-0009 and fl-0017). mabur's answer is a
per-drone calibration (`third_party/mabur/docs/calibration.md`): the GS
sweeps the drone's TX power per MCS, finds each PA wall and writes it into
the drone's config, which then flies `power_mode = "offset"`. It has to
run with the pair a few metres apart or more (Ihor, 2026-11-01), so it
belongs in the UI, not in an ssh session: one button on the Drone tab of
the desktop, phone and Quest.

The drone side already exists in maburd (`drone/src/cal_*.cpp`, CAP_CALIBRATE
in DISC_ACK). The GS side is maburgs' `CalSession` and its seams; WebGs
(pal-ref) has none of it, so the oracle is mabur's own unit tests plus a
trace harness over the C++ `CalSession` built into pal-ref.

## Order (bottom-up, each with its tests)

| Level | Modules (C++) | Status |
|---|---|---|
| C0 | `cal_wire.h` (stamp), `cal_proto.h`, `cal_park.h`, `cal_mav.cpp` (dialect 60000-60005) into `pal-wire::cal` | done: 11 tests, the 6 messages byte-equal to mabur's encoder (golden bytes from cal_mav.cpp) |
| C1 | `cal_plan`, `cal_analysis`, `cal_log` (kept readable by `maburcal report`) | done: `pal-core::cal::{plan, analysis, log}`, mabur's test_cal_plan/test_cal_analysis/test_cal_log cases ported (26 tests); a cal.log written by the Rust log renders in `maburcal report` unchanged |
| C2 | seams: MavTunnel cal lane, cal-body routing in the aggregator, ChannelCore `cal_running` + `CalMoveEdgeHold` + the calibration channel pick (`cal_picker`, `cal_round_pick`), `cal_cmd_clear` | done except the channel pick: MavTunnel `push_cal`/`flush_cal` (own counter, never mixed with live packets), sweep bodies routed before the aggregator (CRC-bad too), drone replies fed from the tunnel, ChannelCore `cal_running` (boot pick, plan, hop block, sampler, in-flight scout, the move-edge hold), `cal_cmd_clear`. `cal_picker`/`cal_round_pick` are not ported: mabur measures only with two or more cards (Mode::Auto); a one-card GS runs its OneCard path. They come with two-card support in the runtime |
| C3 | `CalSession` + the `CalControl` composition and refusals, wired into `RustCore` (radio silence gates every send); trace harness against the C++ session | done: `pal-core::cal::session` with mabur's test_cal_session cases (all but the pick ones), `GsCore::{cal_start, cal_abort, cal_json, set_cal_log_dir}`, `Gs::send` silent while a sweep airs, cal frames through `send_now`; a core test drives a run through a FakeCard (SweepCmd out, ack in, silence, sweep frame tallied, abort and 3 Aborts). No C++ trace harness: the session is pure and its own tests are mabur's |
| C4 | UI: `GsCore::{cal_start, cal_status, cal_abort}`, runtime commands, a Calibrate button with a confirm step (video stops for about 40 s; needs distance), live progress, the result table, Abort; `cal.log` into the flight-log directory | done: `pal-osd::cal` (Drone tab: Calibrate, confirm with the distance warning, progress, Abort, per-width result table; the link chip reads CALIBRATING while a run owns the air), `Cmd::{CalStart, CalAbort}`, runtime `CoreLoop::{set_core, cal}` (refusal shown, `cal`/`cal_start`/`cal_abort` flight-log events, cal.log in the flight-log directory). Desktop, phone and Quest share the panel |
| C5 | field run (Ihor, at distance), then the ladder reaching rung 4 | |

Every level keeps the gates green; no cards or no calibration request =
no change to the m0/m1 parity.
