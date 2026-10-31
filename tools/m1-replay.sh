#!/usr/bin/env bash
# Replays every capture in captures/README.md through `pal replay` and checks
# the AU file against the recorded sha256.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cargo build -q --manifest-path "$ROOT/Cargo.toml" -p pal
TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT
fail=0
while IFS='|' read -r _ file _ mode _ _ expect _; do
  file=$(echo "$file" | xargs); mode=$(echo "$mode" | xargs); expect=$(echo "$expect" | xargs)
  [[ $file == *.bin ]] || continue
  "$ROOT/target/debug/pal" replay "$ROOT/captures/$file" -c "$ROOT/configs/gs.toml" --mode "$mode" --aus "$TMP/out.aus" >/dev/null
  got=$(sha256sum "$TMP/out.aus" | cut -d' ' -f1)
  if [[ $got == "$expect" ]]; then echo "ok   $file"; else echo "FAIL $file: $got"; fail=1; fi
done < "$ROOT/captures/README.md"
exit $fail
