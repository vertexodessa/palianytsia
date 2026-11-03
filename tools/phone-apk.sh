#!/usr/bin/env bash
# Builds the phone APK: libpalphone (release, aarch64) by cargo, the NDK's
# libc++_shared next to it, the GS config as the gs.toml asset, then Gradle.
#   tools/phone-apk.sh [config.toml]   (default configs/gs-auto.toml)
#   PAL_RELEASE=1: release APK signed with the release key (app/build.gradle)
# Out (debug): apps/phone/android/app/build/outputs/apk/debug/app-debug.apk
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
CFG=${1:-$ROOT/configs/gs-auto.toml}
NDK=${ANDROID_NDK_HOME:-$(ls -d "${ANDROID_HOME:-$HOME/Android/Sdk}"/ndk/* | sort -V | tail -1)}
export ANDROID_NDK_HOME=$NDK
"$ROOT/tools/android-build.sh" build --release -p pal-phone
A=$ROOT/apps/phone/android/app/src/main
mkdir -p "$A/jniLibs/arm64-v8a" "$A/assets"
cp "$ROOT/target/aarch64-linux-android/release/libpalphone.so" "$A/jniLibs/arm64-v8a/"
cp "$NDK/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/lib/aarch64-linux-android/libc++_shared.so" \
  "$A/jniLibs/arm64-v8a/"
cp "$CFG" "$A/assets/gs.toml"
cd "$ROOT/apps/phone/android"
[[ -f local.properties ]] || echo "sdk.dir=${ANDROID_HOME:-$HOME/Android/Sdk}" > local.properties
if [[ ${PAL_RELEASE:-0} == 1 ]]; then
  ./gradlew -q assembleRelease
  ls -la app/build/outputs/apk/release/app-release.apk
else
  ./gradlew -q assembleDebug
  ls -la app/build/outputs/apk/debug/app-debug.apk
fi
