#!/usr/bin/env bash
# M1 gate. Replays every capture in captures/README.md:
#  1. pal-ref in the capture's mode must reproduce the recorded sha256;
#  2. the Rust core must reproduce it too;
#  3. ref and Rust must agree byte for byte, AUs and (GS) transmitted RC
#     bodies, in both modes on every capture at 0/10/20/35 % synthetic body
#     loss (same seed, same surviving bodies), and so must the final stats
#     line (webgs::stats_json key for key, nlohmann's number spelling) and,
#     in GS, the parameter table (webgs::params_json);
#  4. the F18 set path: the same --param-set script into both cores.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cargo build -q --release --manifest-path "$ROOT/Cargo.toml" -p pal
PAL=$ROOT/target/release/pal
CAPS=${PAL_CAPTURES:-$ROOT/captures}
if [[ ! -f $CAPS/README.md ]]; then
  # Not a pass: a fresh checkout has no captures, and exit 0 here read as green.
  echo "m1: SKIPPED, no captures in $CAPS (they live outside git; set PAL_CAPTURES)" >&2
  exit 77
fi
TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT
fail=0
sha() { sha256sum "$1" | cut -d' ' -f1; }
while IFS='|' read -r _ file _ mode _ _ expect _; do
  file=$(echo "$file" | xargs); mode=$(echo "$mode" | xargs); expect=$(echo "$expect" | xargs)
  [[ $file == *.bin ]] || continue
  cap=$CAPS/$file
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
        && [[ $(grep -E '^(stats|params) ' <<<"$ref_out") == "$(grep -E '^(stats|params) ' <<<"$out")" ]]; then
        echo "ok   ref==rust $m drop=$d $file ($(grep -o 'aus=[0-9]* truncated=[0-9]*.*sends=[0-9]*' <<<"$out" | sed 's/ au_bytes=[0-9]*//'))"
      else echo "FAIL ref!=rust $m drop=$d $file"; fail=1; fi
    done
  done
  # IDR request path (pal's IdrDriver into set_idr_requests) under loss.
  for d in 10 35; do
    ref_out=$("$PAL" replay "$cap" -c "$CFG" --mode gs --idr --drop-pct $d --seed 7 \
      --aus "$TMP/r.aus" --sends "$TMP/r.sends")
    out=$("$PAL" replay "$cap" -c "$CFG" --mode gs --core rust --idr --drop-pct $d --seed 7 \
      --aus "$TMP/u.aus" --sends "$TMP/u.sends")
    if cmp -s "$TMP/r.aus" "$TMP/u.aus" && cmp -s "$TMP/r.sends" "$TMP/u.sends" \
      && [[ $(grep '^stats ' <<<"$ref_out") == "$(grep '^stats ' <<<"$out")" ]]; then
      echo "ok   ref==rust gs --idr drop=$d $file ($(grep -o '"idr_req":[0-9]*' <<<"$out"))"
    else echo "FAIL ref!=rust gs --idr drop=$d $file"; fail=1; fi
  done
  # F18: a set, a refused set while it is pending, a bad value, a refresh-
  # free late set; sends, refusals and the final table must agree.
  PS=(--param-set 5,4,9000 --param-set 6,99,1 --param-set 20,1,maybe --param-set 32,0,2.5)
  ref_out=$("$PAL" replay "$cap" -c "$CFG" --mode gs --drop-pct 10 --seed 7 "${PS[@]}" \
    --sends "$TMP/r.sends")
  out=$("$PAL" replay "$cap" -c "$CFG" --mode gs --core rust --drop-pct 10 --seed 7 "${PS[@]}" \
    --sends "$TMP/u.sends")
  if cmp -s "$TMP/r.sends" "$TMP/u.sends" \
    && [[ $(grep -E '^(param|params|stats) ' <<<"$ref_out") == "$(grep -E '^(param|params|stats) ' <<<"$out")" ]]; then
    echo "ok   ref==rust gs --param-set $file ($(grep -o '"list":"[a-z]*"' <<<"$out"))"
  else echo "FAIL ref!=rust gs --param-set $file"; fail=1; fi
done < "$CAPS/README.md"
exit $fail
