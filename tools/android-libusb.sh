#!/usr/bin/env bash
# Builds libusb for aarch64 Android (static, no udev, no discovery: the app
# hands devourer the fd from UsbManager via libusb_wrap_sys_device) and writes
# a libusb-1.0.pc so devourer's own CMake finds it through pkg-config.
# LIBUSB_SRC: an unpacked libusb release tree (1.0.27 tested).
# Out: target/android/libusb/{include,lib,lib/pkgconfig}.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
NDK=${ANDROID_NDK_HOME:-$(ls -d "$HOME"/Android/Sdk/ndk/* | sort -V | tail -1)}
API=${ANDROID_API:-29}
OUT=$ROOT/target/android/libusb
[[ -f $OUT/lib/libusb-1.0.a ]] && { echo "$OUT"; exit 0; }
SRC=${LIBUSB_SRC:?set LIBUSB_SRC to an unpacked libusb source tree}
TC=$NDK/toolchains/llvm/prebuilt/linux-x86_64/bin
B=$ROOT/target/android/libusb-build
# A private copy: the source tree may already be configured in place for
# another target, and configure refuses to build out of such a tree.
rm -rf "$B"; cp -r "$SRC" "$B"
cd "$B"
[[ -f Makefile ]] && make distclean >/dev/null 2>&1 || true
CC=$TC/aarch64-linux-android$API-clang AR=$TC/llvm-ar RANLIB=$TC/llvm-ranlib \
  CFLAGS="-O2 -fPIC" ./configure --host=aarch64-linux-android --prefix="$OUT" \
  --disable-udev --enable-static --disable-shared --disable-examples-build \
  --disable-tests-build >/dev/null
make -j"$(nproc)" >/dev/null
make install >/dev/null
echo "$OUT"
