#!/usr/bin/env bash
# M0 gate: pal-ref driven from Rust through GsCore produces the same AU file
# as mabur's own `webgs replay` on maburd --dry-run frame files. This checks
# the wrapper (FFI, callbacks, the replay loop), not a port.
#
# Builds mabur's host tools from the pinned submodule into target/mabur-host
# (webgs, maburd for the frame generator), then runs every case below.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
MABUR=$ROOT/third_party/mabur
BUILD=$ROOT/target/mabur-host
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

cmake -S "$MABUR" -B "$BUILD" -DDEVOURER_DIR="$ROOT/third_party/devourer" \
  -DMABUR_BUILD_LINKBENCH=OFF -DCMAKE_BUILD_TYPE=Release >"$TMP/configure.log" 2>&1 \
  || { cat "$TMP/configure.log"; exit 1; }
cmake --build "$BUILD" -j"$(nproc)" --target webgs maburd >"$TMP/build.log" 2>&1 \
  || { tail -30 "$TMP/build.log"; exit 1; }
cargo build -q --manifest-path "$ROOT/Cargo.toml" -p pal-ref --bin pal-ref-replay
PAL=$ROOT/target/debug/pal-ref-replay

# frames.bin + gs.toml, exactly as mabur's web_au_parity gate makes them.
(cd "$MABUR" && BUILD=$BUILD bash tests/integration/web_frames_gen.sh "$TMP")

fail=0
run_case() {  # name, then the shared replay flags
  local name=$1; shift
  rm -f "$TMP/web.aus" "$TMP/pal.aus"
  local web_args=() a
  for a in "$@"; do [[ $a == --core || $a == rust ]] || web_args+=("$a"); done
  "$BUILD/web/webgs" replay "$TMP/frames.bin" "$TMP/web.aus" -c "$TMP/gs.toml" "${web_args[@]}" >/dev/null \
    2>"$TMP/web.err"
  "$PAL" "$TMP/frames.bin" "$TMP/pal.aus" -c "$TMP/gs.toml" "$@" 2>"$TMP/pal.err"
  local wsends psends
  wsends=$(grep -o 'sends=[0-9]*' "$TMP/web.err")
  psends=$(grep -o 'sends=[0-9]*' "$TMP/pal.err")
  if [ ! -s "$TMP/web.aus" ]; then
    echo "FAIL $name: webgs wrote no AUs"; fail=1
  elif ! cmp -s "$TMP/web.aus" "$TMP/pal.aus"; then
    echo "FAIL $name: AU files differ"; fail=1
  elif [ "$wsends" != "$psends" ]; then
    echo "FAIL $name: webgs $wsends, pal-ref $psends"; fail=1
  else
    echo "ok   $name: $(stat -c %s "$TMP/web.aus") bytes, $(grep -o 'aus_out=[0-9]*' "$TMP/pal.err"), $psends"
  fi
}

# GS: the Rust core's transmitted RC bodies (RCF, DISC, NACK, MAV) must
# match pal-ref's byte for byte; webgs replay has no sends dump.
sends_case() {  # name, then the shared replay flags
  local name=$1; shift
  "$PAL" "$TMP/frames.bin" "$TMP/r.aus" -c "$TMP/gs.toml" "$@" --sends "$TMP/r.sends" 2>/dev/null
  "$PAL" "$TMP/frames.bin" "$TMP/u.aus" -c "$TMP/gs.toml" "$@" --core rust --sends "$TMP/u.sends" 2>/dev/null
  if cmp -s "$TMP/r.sends" "$TMP/u.sends" && cmp -s "$TMP/r.aus" "$TMP/u.aus"; then
    echo "ok   $name: $(stat -c %s "$TMP/r.sends") send bytes"
  else echo "FAIL $name: ref and rust sends or AUs differ"; fail=1; fi
}

for spec in "0 1" "5 7" "15 3"; do
  set -- $spec
  run_case "spotter fixed-gap drop=$1 seed=$2" --mode spotter --fixed-gap --drop-pct "$1" --seed "$2"
  run_case "spotter adaptive  drop=$1 seed=$2" --mode spotter --drop-pct "$1" --seed "$2"
  run_case "rust spotter fixed drop=$1 seed=$2" --mode spotter --fixed-gap --drop-pct "$1" --seed "$2" --core rust
  run_case "rust spotter adapt drop=$1 seed=$2" --mode spotter --drop-pct "$1" --seed "$2" --core rust
  run_case "gs fake-ack       drop=$1 seed=$2" --mode gs --fake-ack --drop-pct "$1" --seed "$2"
  run_case "rust gs fake-ack  drop=$1 seed=$2" --mode gs --fake-ack --drop-pct "$1" --seed "$2" --core rust
  run_case "rust gs fixed-ack drop=$1 seed=$2" --mode gs --fake-ack --fixed-gap --drop-pct "$1" --seed "$2" --core rust
  sends_case "gs sends ref==rust drop=$1 seed=$2" --mode gs --fake-ack --drop-pct "$1" --seed "$2"
  sends_case "gs sends no-ack    drop=$1 seed=$2" --mode gs --drop-pct "$1" --seed "$2"
done

# Row-sliced 1080p frames (mabur's slices4 fixture): truncated AUs go
# through slice salvage, which the 13-AU fixture never reaches.
python3 "$ROOT/tools/slices-frames.py" "$MABUR/tests/fixtures/slices4_1080p.h265" "$TMP/slices.in" 240 >/dev/null
(cd "$MABUR" && BUILD=$BUILD bash tests/integration/web_frames_gen.sh "$TMP" "$TMP/slices.in")
for spec in "0 1" "10 7" "20 3" "30 11"; do
  set -- $spec
  run_case "slices ref  drop=$1 seed=$2" --mode spotter --fixed-gap --drop-pct "$1" --seed "$2"
  run_case "slices rust drop=$1 seed=$2" --mode spotter --fixed-gap --drop-pct "$1" --seed "$2" --core rust
  run_case "slices rust adapt drop=$1 seed=$2" --mode spotter --drop-pct "$1" --seed "$2" --core rust
  grep -o '"salvaged":[0-9]*' "$TMP/pal.err" | sed 's/^/     /'
done

[ $fail -eq 0 ] && echo "== m0 parity passed ==" || { echo "== m0 parity FAILED =="; exit 1; }
