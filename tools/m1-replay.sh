#!/usr/bin/env bash
# M1 gate. Replays every capture in captures/README.md:
#  1. pal-ref in the capture's mode must reproduce the recorded sha256;
#  2. the Rust core must match it where it can run that mode (Spotter);
#  3. ref and Rust in Spotter mode must agree byte for byte on every capture
#     at 0/10/20/35 % synthetic body loss (same seed, same surviving bodies).
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cargo build -q --release --manifest-path "$ROOT/Cargo.toml" -p pal
PAL=$ROOT/target/release/pal
CFG=$ROOT/configs/gs.toml
TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT
fail=0
sha() { sha256sum "$1" | cut -d' ' -f1; }
while IFS='|' read -r _ file _ mode _ _ expect _; do
  file=$(echo "$file" | xargs); mode=$(echo "$mode" | xargs); expect=$(echo "$expect" | xargs)
  [[ $file == *.bin ]] || continue
  cap=$ROOT/captures/$file
  "$PAL" replay "$cap" -c "$CFG" --mode "$mode" --aus "$TMP/ref.aus" >/dev/null
  if [[ $(sha "$TMP/ref.aus") == "$expect" ]]; then echo "ok   ref  $mode $file"
  else echo "FAIL ref  $mode $file: $(sha "$TMP/ref.aus")"; fail=1; fi
  if [[ $mode == spotter ]]; then
    "$PAL" replay "$cap" -c "$CFG" --mode spotter --core rust --aus "$TMP/rust.aus" >/dev/null
    if [[ $(sha "$TMP/rust.aus") == "$expect" ]]; then echo "ok   rust $mode $file"
    else echo "FAIL rust $mode $file: $(sha "$TMP/rust.aus")"; fail=1; fi
  fi
  for d in 0 10 20 35; do
    "$PAL" replay "$cap" -c "$CFG" --mode spotter --drop-pct $d --seed 7 --aus "$TMP/r.aus" >/dev/null
    out=$("$PAL" replay "$cap" -c "$CFG" --mode spotter --core rust --drop-pct $d --seed 7 --aus "$TMP/u.aus")
    if cmp -s "$TMP/r.aus" "$TMP/u.aus"; then
      echo "ok   ref==rust spotter drop=$d $file ($(grep -o 'aus=[0-9]* truncated=[0-9]*' <<<"$out"))"
    else echo "FAIL ref!=rust spotter drop=$d $file"; fail=1; fi
  done
done < "$ROOT/captures/README.md"
exit $fail
