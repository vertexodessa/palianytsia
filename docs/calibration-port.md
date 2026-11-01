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
| C1 | `cal_plan`, `cal_analysis`, `cal_log` (kept readable by `maburcal report`) | |
| C2 | seams: MavTunnel cal lane, cal-body routing in the aggregator, ChannelCore `cal_running` + `CalMoveEdgeHold` + the calibration channel pick (`cal_picker`, `cal_round_pick`), `cal_cmd_clear` | |
| C3 | `CalSession` + the `CalControl` composition and refusals, wired into `RustCore` (radio silence gates every send); trace harness against the C++ session | |
| C4 | UI: `GsCore::{cal_start, cal_status, cal_abort}`, runtime commands, a Calibrate button with a confirm step (video stops for about 40 s; needs distance), live progress, the result table, Abort; `cal.log` into the flight-log directory | |
| C5 | field run (Ihor, at distance), then the ladder reaching rung 4 | |

Every level keeps the gates green; no cards or no calibration request =
no change to the m0/m1 parity.
