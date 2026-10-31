#!/usr/bin/env bash
# Builds the drone half of a matched pair from the pinned submodules:
# dist/maburd-<ver>-ssc338q-8822e, the bundle files, and dist/PAIR.txt.
#
# Toolchain: OPENIPC_HOST_BIN if set; with PAL_FETCH_TOOLCHAIN=1 the pinned
# OpenIPC archive is downloaded, checked against TOOLCHAIN_SHA256 and used.
# The archive sits behind a rolling release tag, so a changed upstream file
# fails the sha check loudly instead of silently changing the build.
#
# usage: tools/build-maburd.sh [version]   (version defaults to "dev")
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
MABUR=$ROOT/third_party/mabur
DEVOURER=$ROOT/third_party/devourer
VER=${1:-dev}
DIST=$ROOT/dist

TOOLCHAIN_URL=https://github.com/OpenIPC/firmware/releases/download/toolchain/toolchain.sigmastar-infinity6e.tgz
TOOLCHAIN_SHA256=${TOOLCHAIN_SHA256:-0588f40ee8fb193a096b0875b34834c19e72340d9b94bc1a49c1417ef7d73839}

if [ "${PAL_FETCH_TOOLCHAIN:-0}" = 1 ]; then
  TC=$ROOT/target/toolchain
  mkdir -p "$TC"
  [ -f "$TC/toolchain.tgz" ] || curl -fL -o "$TC/toolchain.tgz" "$TOOLCHAIN_URL"
  echo "$TOOLCHAIN_SHA256  $TC/toolchain.tgz" | sha256sum -c -
  [ -d "$TC/extracted" ] || { mkdir -p "$TC/extracted" && tar -xzf "$TC/toolchain.tgz" -C "$TC/extracted"; }
  OPENIPC_HOST_BIN=$(dirname "$(find "$TC/extracted" -name arm-openipc-linux-gnueabihf-gcc -type f -o -name arm-openipc-linux-gnueabihf-gcc -type l | head -1)")
fi
: "${OPENIPC_HOST_BIN:?set OPENIPC_HOST_BIN or PAL_FETCH_TOOLCHAIN=1}"
export OPENIPC_HOST_BIN

# build-arm.sh stages pkg-config with nix-build unless toolchain/pkg-config
# exists; off NixOS the host pkg-config is the same tool.
if ! command -v nix-build >/dev/null && [ ! -e "$MABUR/toolchain/pkg-config" ]; then
  mkdir -p "$MABUR/toolchain"
  ln -s /usr "$MABUR/toolchain/pkg-config"
fi

# build-arm.sh resolves ../devourer, which is third_party/devourer here.
(cd "$MABUR" && tools/build-arm.sh)

mkdir -p "$DIST"
BIN=maburd-$VER-ssc338q-8822e
cp "$MABUR/out/arm/maburd" "$DIST/$BIN"
cp "$MABUR/bundle/mabur.default.toml" "$MABUR/bundle/S96mabur" "$DIST/"

RC=$(sed -n 's/.*constexpr uint8_t RC_VERSION = \([0-9]*\);.*/\1/p' "$MABUR/common/include/mabur/rc_proto.h")
{
  echo "palianytsia $VER"
  echo "mabur $(git -C "$MABUR" rev-parse HEAD)"
  echo "devourer $(git -C "$DEVOURER" rev-parse HEAD)"
  echo "RC_VERSION $RC"
  echo "chips 8822E"
  echo
  (cd "$DIST" && sha256sum -- * | grep -v ' PAIR.txt$')
} > "$DIST/PAIR.txt"
cat "$DIST/PAIR.txt"
