#!/usr/bin/env bash
# Builds libusb (third_party/libusb, pinned v1.0.27) for aarch64 Android as a
# static archive from libusb's own Android source list (android/jni/
# libusb.mk) and android/config.h, and writes a libusb-1.0.pc so devourer's
# CMake finds it through pkg-config. No autotools. The app hands devourer the
# UsbManager fd (libusb_wrap_sys_device), so no udev and no discovery.
# Out: target/android/libusb/{include/libusb-1.0,lib,lib/pkgconfig}.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
SRC=$ROOT/third_party/libusb
NDK=${ANDROID_NDK_HOME:-$(ls -d "${ANDROID_HOME:-$HOME/Android/Sdk}"/ndk/* | sort -V | tail -1)}
API=${ANDROID_API:-29}
OUT=$ROOT/target/android/libusb
REV=$(git -C "$SRC" rev-parse HEAD 2>/dev/null || echo none)
if [[ -f $OUT/lib/libusb-1.0.a && $(cat "$OUT/rev" 2>/dev/null) == "$REV" ]]; then
  echo "$OUT"
  exit 0
fi
[[ -f $SRC/libusb/core.c ]] || { echo "third_party/libusb missing: git submodule update --init" >&2; exit 1; }
CC=$NDK/toolchains/llvm/prebuilt/linux-x86_64/bin/aarch64-linux-android$API-clang
AR=$NDK/toolchains/llvm/prebuilt/linux-x86_64/bin/llvm-ar
B=$ROOT/target/android/libusb-build
rm -rf "$B" "$OUT"
mkdir -p "$B" "$OUT/include/libusb-1.0" "$OUT/lib/pkgconfig"
objs=()
for f in core descriptor hotplug io sync strerror os/linux_usbfs os/events_posix \
  os/threads_posix os/linux_netlink; do
  o=$B/$(basename "$f").o
  "$CC" -O2 -fPIC -I"$SRC/android" -I"$SRC/libusb" -I"$SRC/libusb/os" \
    -c "$SRC/libusb/$f.c" -o "$o"
  objs+=("$o")
done
"$AR" rcs "$OUT/lib/libusb-1.0.a" "${objs[@]}"
cp "$SRC/libusb/libusb.h" "$OUT/include/libusb-1.0/"
cat > "$OUT/lib/pkgconfig/libusb-1.0.pc" <<PC
prefix=$OUT
libdir=\${prefix}/lib
includedir=\${prefix}/include

Name: libusb-1.0
Description: libusb for aarch64 Android (palianytsia tools/android-libusb.sh)
Version: 1.0.27
Libs: -L\${libdir} -lusb-1.0
Cflags: -I\${includedir}/libusb-1.0
PC
echo "$REV" > "$OUT/rev"
echo "$OUT"
