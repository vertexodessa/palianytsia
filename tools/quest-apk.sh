#!/usr/bin/env bash
# Builds the Quest APK: libpalquest (release, aarch64) by cargo, the NDK's
# libc++_shared and Khronos' libopenxr_loader next to it, the GS config as
# the gs.toml asset, then Gradle. The loader comes from the Maven Central
# AAR (prefab only, so Gradle would not package it), cached in target/.
#   tools/quest-apk.sh [config.toml]   (default configs/gs-auto.toml)
#   PAL_RELEASE=1: release APK signed with the release key (app/build.gradle)
# Out (debug): apps/quest/android/app/build/outputs/apk/debug/app-debug.apk
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
CFG=${1:-$ROOT/configs/gs-auto.toml}
NDK=${ANDROID_NDK_HOME:-$(ls -d "${ANDROID_HOME:-$HOME/Android/Sdk}"/ndk/* | sort -V | tail -1)}
export ANDROID_NDK_HOME=$NDK
# The drone maburd the app keeps a drone on (runtime::drone_update): a release
# APK carries its own release's build, from tools/build-maburd.sh.
if [[ -n ${PAL_MABURD:-} ]]; then
  PAL_MABURD=$(realpath "$PAL_MABURD"); export PAL_MABURD
  echo "drone maburd $PAL_MABURD sha256 $(sha256sum < "$PAL_MABURD" | cut -c1-12)"
elif [[ ${PAL_RELEASE:-0} == 1 ]]; then
  echo "a release APK needs PAL_MABURD (the release's maburd)" >&2
  exit 1
else
  echo "no PAL_MABURD: this APK updates no drone"
fi
"$ROOT/tools/android-build.sh" build --release -p pal-quest
A=$ROOT/apps/quest/android/app/src/main
mkdir -p "$A/jniLibs/arm64-v8a" "$A/assets"
cp "$ROOT/target/aarch64-linux-android/release/libpalquest.so" "$A/jniLibs/arm64-v8a/"
cp "$NDK/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/lib/aarch64-linux-android/libc++_shared.so" \
  "$A/jniLibs/arm64-v8a/"
cp "$CFG" "$A/assets/gs.toml"
XR_VER=1.1.43
# The loader .so ships inside the signed APK, so the AAR is held to the
# sha256 Maven Central publishes for it, checked on every run: a cached copy
# that changed stops the build as surely as a bad download.
XR_SHA256=7e1b36141f9a4f1fa4a7e061936344fd9fcd36bce6c47eae2ad09812736167b6
AAR=$ROOT/target/android/openxr_loader_for_android-$XR_VER.aar
if [[ ! -f $AAR ]]; then
  mkdir -p "$(dirname "$AAR")"
  curl -fsSL -o "$AAR.part" \
    "https://repo1.maven.org/maven2/org/khronos/openxr/openxr_loader_for_android/$XR_VER/openxr_loader_for_android-$XR_VER.aar"
  mv "$AAR.part" "$AAR"
fi
if ! echo "$XR_SHA256  $AAR" | sha256sum -c --quiet -; then
  rm -f "$AAR"
  echo "quest-apk: $AAR is not sha256 $XR_SHA256; deleted, a rerun downloads it again" >&2
  exit 1
fi
unzip -o -j -q "$AAR" prefab/modules/openxr_loader/libs/android.arm64-v8a/libopenxr_loader.so \
  -d "$A/jniLibs/arm64-v8a/"
cd "$ROOT/apps/quest/android"
[[ -f local.properties ]] || echo "sdk.dir=${ANDROID_HOME:-$HOME/Android/Sdk}" > local.properties
if [[ ${PAL_RELEASE:-0} == 1 ]]; then
  ./gradlew -q assembleRelease
  # Unsigned without a release key: app-release-unsigned.apk.
  ls -la app/build/outputs/apk/release/*.apk
else
  ./gradlew -q assembleDebug
  ls -la app/build/outputs/apk/debug/app-debug.apk
fi
