#!/usr/bin/env bash
# Packs a release from already built parts: dist/palianytsia-<tag>/ (drone/,
# gs/, README.md, SHA256SUMS) and dist/palianytsia-<tag>.tar.gz. The release
# workflow runs it on a tag; it runs the same way by hand.
#
#   tools/pack-release.sh <tag> <maburd> <quest.apk> <phone.apk> <pal-desktop> <imx415.ko>
#
# Refuses, before writing anything, a part that does not match the release:
# a maburd not built from release/MABUR_RELEASE, an APK whose versionName is
# not the tag's, a sensor driver other than the pinned one, a README or notes
# file written for another tag.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
[[ $# == 6 ]] || { sed -n '2,12p' "$0" >&2; exit 2; }
TAG=$1 MABURD=$2 QUEST=$3 PHONE=$4 DESKTOP=$5 KO=$6
REL=$ROOT/release
die() { echo "pack-release: $*" >&2; exit 1; }

[[ $TAG =~ ^v([0-9]+)\.([0-9]+)(\.([0-9]+))?$ ]] || die "tag $TAG is not vX.Y or vX.Y.Z"
VER=${BASH_REMATCH[1]}.${BASH_REMATCH[2]}.${BASH_REMATCH[4]:-0}

MABUR_SHA=$(tr -d '[:space:]' < "$REL/MABUR_RELEASE")
DEVOURER_SHA=$(git -C "$ROOT/third_party/devourer" rev-parse HEAD)
PAL_SHA=$(git -C "$ROOT" rev-parse HEAD)
RC=$(sed -n 's/.*constexpr uint8_t RC_VERSION = \([0-9]*\);.*/\1/p' \
  "$ROOT/third_party/mabur/common/include/mabur/rc_proto.h")

# maburd prints its stamp from these two strings; a dirty or other build
# carries other ones.
grep -aq "${MABUR_SHA::8}" "$MABURD" || die "$MABURD is not built from mabur ${MABUR_SHA::8}"
grep -aq "dirty" "$MABURD" && die "$MABURD is a dirty build"
for apk in "$QUEST" "$PHONE"; do
  got=$(aapt2 dump badging "$apk" 2>/dev/null | sed -n "s/.*versionName='\([^']*\)'.*/\1/p")
  [[ $got == "$VER" ]] || die "$apk versionName '$got', tag wants '$VER'"
done
want_ko=$(cut -d' ' -f1 "$REL/sensor_imx415_star6e.sha256")
[[ $(sha256sum "$KO" | cut -d' ' -f1) == "$want_ko" ]] || die "$KO is not the pinned driver"
head -1 "$REL/README.md" | grep -qx "# Palianytsia $TAG" || die "release/README.md is not for $TAG"
[[ -s $REL/notes/$TAG.md ]] || die "release/notes/$TAG.md missing"

OUT=$ROOT/dist/palianytsia-$TAG
rm -rf "$OUT" "$OUT.tar.gz"
mkdir -p "$OUT/drone" "$OUT/gs"
install -m755 "$MABURD" "$OUT/drone/maburd"
install -m755 "$ROOT/tools/drone-install.sh" "$OUT/drone/install.sh"
install -m644 "$REL/drone/mabur.toml" "$REL/drone/S96mabur" "$OUT/drone/"
install -m644 "$KO" "$OUT/drone/sensor_imx415_star6e.ko"
install -m644 "$QUEST" "$OUT/gs/palianytsia-quest-$VER.apk"
install -m644 "$PHONE" "$OUT/gs/palianytsia-phone-$VER.apk"
install -m755 "$DESKTOP" "$OUT/gs/pal-desktop-linux-x86_64"
install -m644 "$ROOT/configs/gs-auto.toml" "$OUT/gs/gs.toml"
sed -e "s/@PAL_SHA@/${PAL_SHA::7}/g" -e "s/@MABUR_SHA@/${MABUR_SHA::8}/g" \
  -e "s/@DEVOURER_SHA@/${DEVOURER_SHA::8}/g" -e "s/@RC_VERSION@/$RC/g" \
  "$REL/README.md" > "$OUT/README.md"
grep -q '@[A-Z_]*@' "$OUT/README.md" && die "README.md keeps a placeholder"
(cd "$OUT" && find . -type f ! -name SHA256SUMS | sed 's|^\./||' | LC_ALL=C sort \
  | xargs sha256sum > SHA256SUMS)
tar -C "$ROOT/dist" -czf "$OUT.tar.gz" "palianytsia-$TAG"
cat "$OUT/SHA256SUMS"
ls -la "$OUT.tar.gz"
