# Palianytsia

A ground station for the mabur FPV video link, written in Rust. It runs on
Linux (desktop app and CLI), on Android phones and on Meta Quest 3 (native
OpenXR), with RTL8812AU/8812EU USB cards driven from user space by devourer.

Releases (the three ground station apps plus the matching drone daemon and
installer) are on the [releases page](https://github.com/vertexodessa/palianytsia/releases);
each package's README covers installing the GS and the drone. The design,
the decisions and the milestones are in [docs/spec.md](docs/spec.md).

## Layout

| Path | Role |
|---|---|
| `crates/pal-wire` | wire formats: 802.11, SBI, RC frames, MAVLink 2 framer, the 5 GHz channel grid |
| `crates/pal-fec` | GF(256) and the sliding-window RLC decoder |
| `crates/pal-core` | the GS core: link state, bitrate ladder, NACK repair, MAVLink tunnel, IDR refresh, channel core (search, pick, hops), TX-power calibration |
| `crates/pal-ref` | mabur's C++ WebGs behind the same `GsCore` trait: the test oracle (desktop/host only) |
| `crates/devourer-sys` | devourer through a C shim |
| `crates/runtime` | what the apps share: card RX thread, the core loop, flight log, stats |
| `crates/pal-osd` | the egui OSD and settings panel all three apps draw |
| `crates/pal-android` | the Android link (core + MediaCodec + flight log) shared by phone and Quest |
| `crates/flightlog`, `decode`, `hevc` | flight log and captures, libavcodec decode, HEVC NAL handling |
| `apps/desktop` | `pal-desktop` (eframe) |
| `apps/pal` | CLI: `pal live`, `pal replay`, `pal log` |
| `apps/phone` | phone app: Rust cdylib + Java shell |
| `apps/quest` | Quest 3 app: Rust NativeActivity (OpenXR) |
| `release/` | what a release carries besides the builds: package README, notes per tag, drone config and init script, the fork commit maburd is built from |
| `third_party/mabur`, `third_party/devourer` | pinned at branch `palianytsia/base` of the vertexodessa forks (RC_VERSION 18) |
| `third_party/libusb` | upstream libusb v1.0.27, Android builds only |

Clone with submodules:

```sh
git clone --recurse-submodules git@github.com:vertexodessa/palianytsia.git
```

The drone daemon (`maburd`) and the GS are a matched pair and are released
together. A release builds maburd from the fork commit in
`release/MABUR_RELEASE` (`palianytsia/base` without its bench-only debug
route), not from the submodule pin.

## Build

Host: Rust stable, cmake, pkg-config, libusb-1.0, libudev, and FFmpeg's
libavcodec/libswscale for the desktop app.

```sh
cargo build --release -p pal-desktop       # desktop GS (system FFmpeg)
cargo build --release -p pal               # CLI
tools/quest-apk.sh                         # Quest APK (Android SDK + NDK, ANDROID_HOME)
tools/phone-apk.sh                         # phone APK
PAL_FETCH_TOOLCHAIN=1 tools/build-maburd.sh   # drone maburd (pinned OpenIPC toolchain)
```

`PAL_RELEASE=1` makes release APKs, signed with the key named in
`~/.config/palianytsia/release-signing.properties` (or `$PAL_SIGNING`);
without one they are unsigned. A desktop binary that runs without the
system FFmpeg: `tools/ffmpeg-static.sh` once, then
`PAL_FFMPEG_STATIC=$PWD/target/ffmpeg-static cargo build --release -p pal-desktop`.

Run the desktop GS on a live card with `pal-desktop -c configs/gs-auto.toml --live --core rust`,
or on a capture with `--replay`.

## Tests

```sh
cargo fmt --all --check && cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
tools/m0-parity.sh     # pal-ref == mabur's own webgs replay
tools/m1-replay.sh     # Rust core == C++ reference on recorded captures
```

Every core change keeps the Rust core byte-identical to the C++ reference.
Captures are not in git; `tools/m1-replay.sh` reads `PAL_CAPTURES` and exits
77 (skipped) without them.

## Releases

Pushing a tag `vX.Y` or `vX.Y.Z` runs `.github/workflows/release.yml`: the
gates, maburd, signed APKs and the static desktop, packed by
`tools/pack-release.sh` and published with `release/notes/<tag>.md`. Before
tagging, bump both APKs' `versionName`, head `release/README.md` with
`# Palianytsia <tag>`, write the notes and pin the fork commit. Running the
workflow by hand with a tag is a dry run that keeps the package as an
artifact.
