#!/usr/bin/env bash
# Builds the FFmpeg that release pal-desktop links statically: the HEVC
# decoder and parser, libswscale and the HEVC GPU decoders, so the binary
# runs without the system's FFmpeg. GPU decode: nvdec loads the NVIDIA
# driver with dlopen (no link dependency); vaapi (Intel/AMD) and vdpau link
# libva/libva-drm/libva-x11/libvdpau/libX11, which every desktop with a
# Mesa or NVIDIA driver has. Needs libva-dev, libvdpau-dev, libx11-dev, libxext-dev, libxv-dev, nasm. The system
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
NV=${NV_HEADERS_TAG:-n12.1.14.0}
rm -rf "$SRC/nv-codec-headers"
git clone -q --depth 1 -b "$NV" https://github.com/FFmpeg/nv-codec-headers "$SRC/nv-codec-headers"
make -C "$SRC/nv-codec-headers" PREFIX="$OUT" install >/dev/null
export PKG_CONFIG_PATH="$OUT/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
cd "$SRC/ffmpeg-$VER"
./configure --prefix="$OUT" --enable-static --disable-shared --enable-pic \
  --disable-autodetect --disable-programs --disable-doc --disable-network \
  --disable-everything --enable-decoder=hevc --enable-parser=hevc \
  --disable-avdevice --disable-avformat --disable-avfilter --disable-swresample \
  --enable-swscale \
  --enable-ffnvcodec --enable-nvdec --enable-vaapi --enable-vdpau --enable-xlib \
  --enable-hwaccel=hevc_nvdec,hevc_vaapi,hevc_vdpau
grep -E '#define CONFIG_HEVC_(NVDEC|VAAPI|VDPAU)_HWACCEL 1' config_components.h | wc -l | grep -qx 3 ||
  { echo "configure dropped a HEVC hwaccel (missing -dev package?)" >&2; exit 1; }
make -j"$(nproc)"
make install
echo "== $OUT"
