#!/usr/bin/env bash
# Builds the Quest APK: libpalquest (release, aarch64) by cargo, the NDK's
# libc++_shared and Khronos' libopenxr_loader next to it, the GS config as
# the gs.toml asset, then Gradle. The loader comes from the Maven Central
# AAR (prefab only, so Gradle would not package it), cached in target/.
#   tools/quest-apk.sh [config.toml]   (default configs/bench-ch40.toml)
# Out: apps/quest/android/app/build/outputs/apk/debug/app-debug.apk
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
CFG=${1:-$ROOT/configs/bench-ch40.toml}
NDK=${ANDROID_NDK_HOME:-$(ls -d "$HOME"/Android/Sdk/ndk/* | sort -V | tail -1)}
export ANDROID_NDK_HOME=$NDK
"$ROOT/tools/android-build.sh" build --release -p pal-quest
A=$ROOT/apps/quest/android/app/src/main
mkdir -p "$A/jniLibs/arm64-v8a" "$A/assets"
cp "$ROOT/target/aarch64-linux-android/release/libpalquest.so" "$A/jniLibs/arm64-v8a/"
cp "$NDK/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/lib/aarch64-linux-android/libc++_shared.so" \
  "$A/jniLibs/arm64-v8a/"
cp "$CFG" "$A/assets/gs.toml"
XR_VER=1.1.43
AAR=$ROOT/target/android/openxr_loader_for_android-$XR_VER.aar
if [[ ! -f $AAR ]]; then
  mkdir -p "$(dirname "$AAR")"
  curl -fsSL -o "$AAR.part" \
    "https://repo1.maven.org/maven2/org/khronos/openxr/openxr_loader_for_android/$XR_VER/openxr_loader_for_android-$XR_VER.aar"
  mv "$AAR.part" "$AAR"
fi
unzip -o -j -q "$AAR" prefab/modules/openxr_loader/libs/android.arm64-v8a/libopenxr_loader.so \
  -d "$A/jniLibs/arm64-v8a/"
cd "$ROOT/apps/quest/android"
[[ -f local.properties ]] || echo "sdk.dir=$HOME/Android/Sdk" > local.properties
./gradlew -q assembleDebug
ls -la app/build/outputs/apk/debug/app-debug.apk
