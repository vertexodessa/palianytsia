#!/usr/bin/env bash
# M1 gate. Replays every capture in captures/README.md:
#  1. pal-ref in the capture's mode must reproduce the recorded sha256;
#  2. the Rust core must reproduce it too;
#  3. ref and Rust must agree byte for byte, AUs and (GS) transmitted RC
#     bodies, in both modes on every capture at 0/10/20/35 % synthetic body
#     loss (same seed, same surviving bodies), and so must the final stats
#     line (webgs::stats_json key for key, nlohmann's number spelling).
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cargo build -q --release --manifest-path "$ROOT/Cargo.toml" -p pal
PAL=$ROOT/target/release/pal
TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT
fail=0
sha() { sha256sum "$1" | cut -d' ' -f1; }
while IFS='|' read -r _ file _ mode _ _ expect _; do
  file=$(echo "$file" | xargs); mode=$(echo "$mode" | xargs); expect=$(echo "$expect" | xargs)
  [[ $file == *.bin ]] || continue
  cap=$ROOT/captures/$file
  # A capture recorded under another pairing key carries it beside itself.
  CFG=$ROOT/configs/gs.toml
  if [[ -f ${cap%.bin}.key ]]; then
    CFG=$TMP/keyed.toml
    sed "s#^key_file *=.*#key_file = \"${cap%.bin}.key\"#" "$ROOT/configs/gs.toml" > "$CFG"
  fi
  "$PAL" replay "$cap" -c "$CFG" --mode "$mode" --aus "$TMP/ref.aus" >/dev/null
  if [[ $(sha "$TMP/ref.aus") == "$expect" ]]; then echo "ok   ref  $mode $file"
  else echo "FAIL ref  $mode $file: $(sha "$TMP/ref.aus")"; fail=1; fi
  "$PAL" replay "$cap" -c "$CFG" --mode "$mode" --core rust --aus "$TMP/rust.aus" >/dev/null
  if [[ $(sha "$TMP/rust.aus") == "$expect" ]]; then echo "ok   rust $mode $file"
  else echo "FAIL rust $mode $file: $(sha "$TMP/rust.aus")"; fail=1; fi
  for m in spotter gs; do
    for d in 0 10 20 35; do
      ref_out=$("$PAL" replay "$cap" -c "$CFG" --mode $m --drop-pct $d --seed 7 --aus "$TMP/r.aus" \
        --sends "$TMP/r.sends")
      out=$("$PAL" replay "$cap" -c "$CFG" --mode $m --core rust --drop-pct $d --seed 7 \
        --aus "$TMP/u.aus" --sends "$TMP/u.sends")
      if cmp -s "$TMP/r.aus" "$TMP/u.aus" && cmp -s "$TMP/r.sends" "$TMP/u.sends" \
        && [[ $(grep '^stats ' <<<"$ref_out") == $(grep '^stats ' <<<"$out") ]]; then
        echo "ok   ref==rust $m drop=$d $file ($(grep -o 'aus=[0-9]* truncated=[0-9]*.*sends=[0-9]*' <<<"$out" | sed 's/ au_bytes=[0-9]*//'))"
      else echo "FAIL ref!=rust $m drop=$d $file"; fail=1; fi
    done
  done
done < "$ROOT/captures/README.md"
exit $fail
