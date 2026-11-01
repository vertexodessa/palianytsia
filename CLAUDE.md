# CLAUDE.md

Palianytsia: a Rust ground station for the mabur FPV link (spec and milestone
status: `docs/spec.md`; read its M1/M2 status sections first).

## Layout
| Path | Role |
|---|---|
| `crates/pal-wire` | wire formats: dot11, SBI, RC (RCF/DISC/Telem/NACK/MAV), MAVLink 2 framer |
| `crates/pal-fec` | GF(256), sliding-window RLC decoder |
| `crates/pal-core` | the Rust GS core (`rust_core.rs`, WebGs wiring), `GsCore` trait, ladder, LHA, NACK, MAV tunnel, refresh (IDR) |
| `crates/pal-ref` | mabur's C++ WebGs behind `GsCore` (C shim): the test oracle |
| `crates/devourer-sys` | devourer through a C shim (USB cards) |
| `crates/flightlog`, `decode`, `pal-osd`, `hevc` | flight log + `palcap 1` captures, libavcodec decode, OSD widgets, NAL/skip-slice |
| `crates/runtime` | shared by the apps: devourer RX thread, core loop (`CoreLoop`: GsCore + IdrDriver + log + stats/params), `run_live` |
| `apps/pal` | CLI: `pal live`, `pal replay`, `pal log` |
| `apps/phone` | phone app (M3): `rust/` = `pal-phone` cdylib (JNI start/stats/lastError/stop over `runtime`, MediaCodec into the Surface), `android/` = Java shell (USB permission, SurfaceView, text OSD). `tools/phone-apk.sh [cfg]` builds the APK |
| `apps/desktop` | `pal-desktop` (eframe): `--live` or `--replay`, `--snapshot out.ppm --secs N` for headless checks |
| `third_party/libusb` | upstream libusb v1.0.27, Android only (`tools/android-libusb.sh`; `tools/android-build.sh build -p <crate>` cross-builds for aarch64) |
| `third_party/{mabur,devourer}` | pinned submodules of the vertexodessa forks (branch `palianytsia/base`); change them only as a fork commit on that branch (Ihor allowed it when Palianytsia needs it), pushed, then bump the pointer here |

## Gates (run all before a commit)
```sh
cargo fmt --all --check && cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
tools/m0-parity.sh     # pal-ref == mabur's webgs replay on maburd dry-run files
tools/m1-replay.sh     # ref == rust: AUs, sent RC bytes, final stats line, --idr path
```
Every core change must keep ref==rust byte-identical. `captures/` is git-ignored
(no binary recordings in the repo); `tools/m1-replay.sh` reads `PAL_CAPTURES`
(default `captures/`) and exits 77 (skipped, not a pass) when absent. `captures/README.md` lists each
capture with its expected AU sha256; a `<name>.key` beside a capture is its
pairing key.

## Live rig (as of 2026-11-01)
- Host card: RTL8812AU 0bda:881a on USB 5-2. If sends fail with
  `error -4` and dmesg shows `USB disconnect` ~2 s after open, the card dropped
  off the bus; `authorized` toggling does not fix it, a physical replug did.
- Drone `ld` (192.168.1.10): RTL8822E quad, maburd RC 18, default key, parked on
  ch 40 / 40 MHz. `configs/gs.toml` pins ch 144, so live runs use a copy with
  `channel = 40` and `pal live --ch 40 --width 40 --mode gs`.
- maburd respawns 2 s after it exits (S96mabur loop); `kill $(pidof maburd)`
  is a drone restart. Ihor allowed drone actions; device commands still carry a
  `# predict:` claim.
- No loss injector in original mabur; `radio.wall_margin_db` caps at 6 dB, so a
  real 20 % loss capture needs physical attenuation.

## Helpers
- `configs/bench-ch40.toml`: `gs.toml` pinned to ch 40 for the bench quad.
- `tools/telem.py <capture>`: the drone's own RX counters (rx_own etc.) from T_TELEM in a capture.
- `tools/lpdiff.py a b`: first differing records of two `--aus`/`--sends` dumps.
- `pal-desktop ... --snapshot out.ppm --secs N`: headless UI check; `pal log DIR`: flight-log summary.

## Finding code
`zg query "<what it does>"` works here (index in `.zvec-grep/`, 87 files, submodules not indexed: use rg in `third_party/mabur`). Re-run `zg index` after large edits.

## Rules
- Commit with `git -c user.email=ivlev.igor@gmail.com commit`; no AI attribution.
- Do not push while the GitHub repo is being recreated (2026-11-01); ask first.
