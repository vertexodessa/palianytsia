# Palianytsia

A ground station for the mabur FPV video link, written in Rust: native Linux
first, then an Android phone app and a native OpenXR app for Quest 3.

Status: design. Nothing builds yet. The plan, the decisions and the milestones
are in [docs/spec.md](docs/spec.md).

## Layout

- `third_party/mabur`: mabur, pinned at branch `palianytsia/base` of
  vertexodessa/mabur (the wire this app speaks; RC_VERSION 18).
- `third_party/devourer`: the userspace RTL88xx driver, pinned at branch
  `palianytsia/base` of vertexodessa/devourer.

Clone with submodules:

```sh
git clone --recurse-submodules git@github.com:vertexodessa/palianytsia.git
```

The drone (`maburd`) and this app are a matched pair: both are built from the
pinned mabur commit and released together.

## Build (M0)

```sh
git submodule update --init
cargo test --workspace          # golden vectors (needs cmake, libusb-1.0-dev for pal-ref)
tools/m0-parity.sh              # pal-ref (C++ WebGs via GsCore) == webgs replay
PAL_FETCH_TOOLCHAIN=1 tools/build-maburd.sh   # dist/maburd-* + dist/PAIR.txt
```
