#!/usr/bin/env bash
# Builds the FFmpeg that release pal-desktop links statically: the HEVC
# decoder and parser, libswscale, and nothing else (no external libraries,
# no hwaccel), so the binary runs without the system's FFmpeg. The system
# libavcodec pulls ~100 shared libraries and its soname changes with every
# FFmpeg major, so a desktop build linked to it starts only on the distro it
# was built on.
#   tools/ffmpeg-static.sh            -> target/ffmpeg-static/{include,lib}
#   PAL_FFMPEG_STATIC=$PWD/target/ffmpeg-static cargo build --release -p pal-desktop
set -euo pipefail
VER=${FFMPEG_VER:-6.1.2}
SHA=${FFMPEG_SHA:-3b624649725ecdc565c903ca6643d41f33bd49239922e45c9b1442c63dca4e38}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
OUT=$ROOT/target/ffmpeg-static
SRC=$ROOT/target/ffmpeg-src
mkdir -p "$SRC"
TAR=$SRC/ffmpeg-$VER.tar.xz
[[ -f $TAR ]] || curl -fL -o "$TAR" "https://ffmpeg.org/releases/ffmpeg-$VER.tar.xz"
echo "$SHA  $TAR" | sha256sum -c -
rm -rf "$SRC/ffmpeg-$VER"
tar -C "$SRC" -xf "$TAR"
cd "$SRC/ffmpeg-$VER"
./configure --prefix="$OUT" --enable-static --disable-shared --enable-pic \
  --disable-autodetect --disable-programs --disable-doc --disable-network \
  --disable-everything --enable-decoder=hevc --enable-parser=hevc \
  --disable-avdevice --disable-avformat --disable-avfilter --disable-swresample \
  --enable-swscale
make -j"$(nproc)"
rm -rf "$OUT"
make install
echo "== $OUT"
