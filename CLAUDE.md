# CLAUDE.md

Palianytsia: a Rust ground station for the mabur FPV link (spec and milestone
status: `docs/spec.md`; read its "Status (2026-11-07)" section first; the dated M-notes are history).

## Layout
| Path | Role |
|---|---|
| `crates/pal-wire` | wire formats: dot11, SBI, RC (RCF/DISC/Telem/NACK/MAV), MAVLink 2 framer |
| `crates/pal-fec` | GF(256), sliding-window RLC decoder |
| `crates/pal-core` | the Rust GS core (`rust_core.rs`, WebGs wiring; its NACK, calibration and parameter lanes in `rust_core/`), `GsCore` trait, ladder, LHA, NACK, MAV tunnel, refresh (IDR) |
| `crates/pal-ref` | mabur's C++ WebGs behind `GsCore` (C shim): the test oracle (desktop/host only); `RefCard` (`cpp/pal_card.cpp`) lends the devourer-sys card to it as a `maburgs::LinkCard`, so it runs mabur's ChannelCore live; `tests/chan_trace.rs` checks the Rust ChannelCore against it record by record |
| `crates/devourer-sys` | devourer through a C shim (USB cards) |
| `crates/flightlog`, `decode`, `pal-osd`, `hevc` | flight log + `palcap 1` captures, libavcodec decode, OSD widgets, NAL/skip-slice |
| `crates/runtime` | shared by the apps: devourer RX thread, core loop (`CoreLoop`: GsCore + IdrDriver + log + stats/params), `run_live` |
| `apps/pal` | CLI: `pal live`, `pal replay`, `pal log` |
| `apps/phone` | phone app (M3): `rust/` = `pal-phone` cdylib (JNI start/stats/lastError/stop over `runtime`, MediaCodec into the Surface), `android/` = Java shell (USB permission, SurfaceView, text OSD). `tools/phone-apk.sh [cfg]` builds the APK |
| `crates/pal-android` | Android link shared by phone and Quest: `Link::start(fd, window, config)` runs core + MediaCodec threads and the flight log, `Ports` for the UI, bounded `stop()` |
| `apps/quest` | Quest 3 app (M3): `rust/` = `pal-quest` NativeActivity (OpenXR: video via surface swapchain quad, egui OSD quad, controller ray + panel, USB supervisor with 3 s retry; up to 2 cards: card 0 = link/TX, all receive, the channel core scouts on the other; a card added or detached restarts the link), `android/` = manifest only. `tools/quest-apk.sh [cfg]` builds the APK (OpenXR loader pulled from Maven Central) |
| `apps/desktop` | `pal-desktop` (eframe): `--live` or `--replay`, `--snapshot out.ppm --secs N` for headless checks |
| `third_party/libusb` | upstream libusb v1.0.27, Android only (`tools/android-libusb.sh`; `tools/android-build.sh build -p <crate>` cross-builds for aarch64) |
| `third_party/{mabur,devourer}` | pinned submodules of the vertexodessa forks (mabur: branch `palianytsia/base-v2` = upstream master plus our commits (rebased 2026-11-07; the 2026-11-03 line is kept as branch `palianytsia/base-v2-pre-2026-11-07` and tags `palianytsia/v0.8`, `palianytsia/v0.8.1`), `palianytsia/base` is the pre-2026-11-03 line; devourer: `palianytsia/base`); change them only as a fork commit on that branch (Ihor allowed it when Palianytsia needs it), pushed, then bump the pointer here |

## Gates (run all before a commit)
```sh
cargo fmt --all --check && cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
tools/m0-parity.sh     # pal-ref == mabur's webgs replay on maburd dry-run files
tools/m1-replay.sh     # ref == rust: AUs, sent RC bytes, final stats line, --idr path
tools/doc-check.sh     # docs vs tree: .gitmodules branches, MABUR_RELEASE, RC_VERSION, release tag, paths
```
Docs: do not copy shas or versions into README/spec; point at the tree (`git submodule status`,
`release/MABUR_RELEASE`). A commit that bumps a pin, cuts a release or closes a Status item updates
`docs/spec.md` Status in the same commit; resolved items get "Resolved YYYY-MM-DD", not deletion.
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
  `channel = 40` and `pal live --ch 40 --width 40 --mode gs`; the phone/Quest APKs
  (`gs-auto.toml`) find it on their own.
- maburd respawns 2 s after it exits (S96mabur loop); `kill $(pidof maburd)`
  is a drone restart. Ihor allowed drone actions; device commands still carry a
  `# predict:` claim.
- No loss injector in original mabur; `radio.wall_margin_db` caps at 6 dB, so a
  real 20 % loss capture needs physical attenuation.

## Helpers
- `configs/bench-ch40.toml`: `gs.toml` pinned to ch 40 for the bench quad.
- `configs/gs-auto.toml` (the APK default): `gs.toml` with `channel = "auto"`. A GS on an auto
  config (pal-desktop with no `--ch`, the phone/Quest APKs) runs the channel core on the card through
  `runtime::LiveCfg::attach`: prelude ranking, boot pick, relocation, in-flight hops, transmitting
  through the card. `--card2 N` (desktop) / a second adapter (Quest) adds a receive+scout card (`LiveCfg::extra`, stats `cards`, OSD chip `cards`). `--core rust` (the desktop default, and the APKs) use the Rust port (`RustCore::with_cards` +
  `runtime::dev_card::rust_channel_core`, clock `runtime` mono); `--core ref` uses mabur's C++ one
  (`pal_ref::live`, clock `pal_ref::now_us`). pal-ref is no longer in the Android build. A Rust
  spotter on an auto config still uses the old `runtime::run_live` sweep (400 ms per channel, 3
  CRC-good drone frames = found, again after 8 s without drone frames; events `search`,
  `drone_found`, `drone_lost`). Pinned configs never search. On the bench the prelude ranking moves
  the quad from 40 to 144 (both cores, 2026-11-01).
- Release APKs: `PAL_RELEASE=1 tools/{phone,quest}-apk.sh`, signed from
  `~/.config/palianytsia/release-signing.properties` (key `~/.android/palianytsia-release.jks`, never in the
  repo; without it the APK is unsigned). v0.1 package: `dist/palianytsia-v0.1/` (drone maburd from mabur
  `release/v0.1` = `palianytsia/base` minus the bench `/sim` commit). `gs.toml` stays pinned for the parity gates.
- Release desktop: `tools/ffmpeg-static.sh` once, then `PAL_FFMPEG_STATIC=$PWD/target/ffmpeg-static cargo build --release -p pal-desktop`
  (HEVC + swscale linked in, no hwaccel; the system-FFmpeg build did not start on QA's machine). v0.2 package:
  `dist/palianytsia-v0.2/` (maburd from mabur `release/v0.2` = `release/v0.1` + DISC width agreement 7c3e2cf9);
  v0.3 = same maburd, `tools/drone-install.sh` that also does a first install on a stock OpenIPC FPV image
  (`--fresh` forces those steps, `--remove` undoes them; an update keeps the drone's mabur.toml), GPU decode in the desktop;
  v0.4 = maburd from mabur `release/v0.4` (24e9d368 = `release/v0.2` + native readout), `sensor_imx415_star6e.ko`
  installed by `drone-install.sh` (patches the drone's `load_sigmastar`, original kept as `.palianytsia`), APKs 0.4.0.
  v0.5 = maburd from mabur `release/v0.5` (61dba4ee = `release/v0.4` + RADIO_PWR_MODE + any 5 GHz grid channel), APKs 0.5.0;
  the drone toml and S96mabur are carried over from v0.4 (the release branch bundle files are older than base's).
  v0.6 = maburd from mabur `release/v0.6` (8d2056a0, + VENC_MODES), the six-mode driver of waybeam_venc 581804a, APKs 0.6.0.
  v0.7 = maburd from mabur `release/v0.7` (ae590193 = upstream master c8e2d20e + `palianytsia/base-v2` minus `/sim`),
  APKs 0.7.0; drone parameters renumbered after row 11, so GS and drone must be updated together.
  v0.7.1 = GS only (movable OSD chips, Quest OSD quad 1.5x the video), same maburd as v0.7, APKs 0.7.1.
  v0.8 = maburd from mabur `release/v0.8` (`release/v0.7` + FEC repairs at frame end, `fec.repairs_with_frame`), bundle
  `fec.window = 64`, APKs 0.8.0; GS and drone stay compatible with v0.7.x.
  v0.8.1 = `release/v0.8` + a config without `repairs_with_frame` gets the mode with window raised to 64 (a drone config kept on update), APKs 0.8.1.
- Releases: a tag `vX.Y`/`vX.Y.Z` runs `.github/workflows/release.yml` (gates, maburd from `release/MABUR_RELEASE`, signed APKs from the
  `PAL_RELEASE_*` secrets, static desktop) and `tools/pack-release.sh` publishes `dist/palianytsia-<tag>` as a GitHub release with
  `release/notes/<tag>.md`. Before tagging: run `tools/m1-replay.sh` locally (CI has no captures and never will: no recordings on GitHub), bump both APK `versionName`, head `release/README.md` with `# Palianytsia <tag>`, write the
  notes, pin the fork commit. The IMX415 driver is an asset of the `deps` release (sha in `release/`). Run workflow_dispatch with a tag
  for a dry run (package as an artifact, nothing published).
- Video width defaults to 20 MHz (`HudPrefs.link_bw`); the DISC proposes it (`DISC_CAP_WIDTH`) and the drone retunes its receiver.
- `tools/telem.py <capture>`: the drone's own RX counters (rx_own etc.) from T_TELEM in a capture.
- `tools/lpdiff.py a b`: first differing records of two `--aus`/`--sends` dumps.
- `pal-desktop ... --snapshot out.ppm --secs N`: headless UI check; `pal log DIR`: flight-log summary.

## Finding code
`zg query "<what it does>"` works here (index in `.zvec-grep/`, 87 files, submodules not indexed: use rg in `third_party/mabur`). Re-run `zg index` after large edits.

## Rules
- Commit with `git -c user.email=ivlev.igor@gmail.com commit`; no AI attribution.
- Do not push while the GitHub repo is being recreated (2026-11-01); ask first.
