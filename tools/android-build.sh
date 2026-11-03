#!/usr/bin/env bash
# cargo for aarch64 Android with the NDK's clang as cc/c++/linker:
#   tools/android-build.sh build -p devourer-sys
# libusb comes from third_party/libusb via tools/android-libusb.sh.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
export ANDROID_NDK_HOME=${ANDROID_NDK_HOME:-$(ls -d "${ANDROID_HOME:-$HOME/Android/Sdk}"/ndk/* | sort -V | tail -1)}
API=${ANDROID_API:-29}
TC=$ANDROID_NDK_HOME/toolchains/llvm/prebuilt/linux-x86_64/bin
PAL_ANDROID_LIBUSB=$("$ROOT/tools/android-libusb.sh")
export PAL_ANDROID_LIBUSB
export CC_aarch64_linux_android=$TC/aarch64-linux-android$API-clang
export CXX_aarch64_linux_android=$TC/aarch64-linux-android$API-clang++
export AR_aarch64_linux_android=$TC/llvm-ar
export CARGO_TARGET_AARCH64_LINUX_ANDROID_LINKER=$CC_aarch64_linux_android
cmd=$1; shift
exec cargo "$cmd" --target aarch64-linux-android "$@"
