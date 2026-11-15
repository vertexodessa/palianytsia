#!/usr/bin/env python3
"""Every key of release/drone/mabur.toml must be one the release maburd knows.

maburd refuses a config with a key it does not know and does not start
(v0.12: the kept `radio.rate_walls_rel` stopped the drone, v0.12.1 fixed it).
The release takes maburd from mabur at release/MABUR_RELEASE, not from the
pinned submodule commit, so a key that main's toml gained with a newer pin
breaks a release cut before the release branch carries it. Run before a tag
(release.yml does). Exit 1 names each unknown key.

Usage: tools/release_toml_check.py [mabur_commit]   (default: release/MABUR_RELEASE)
"""
import re
import subprocess
import sys
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def known_keys(cpp: str) -> dict[str, set[str]]:
    """prefix -> keys, from each check_known_keys(j, {...}, "prefix") call."""
    out: dict[str, set[str]] = {}
    for body, prefix in re.findall(
        r'check_known_keys\(\s*\w+\s*,\s*\{(.*?)\}\s*,\s*"([^"]*)"\s*\)', cpp, re.S
    ):
        out.setdefault(prefix, set()).update(re.findall(r'"([^"]+)"', body))
    return out


def unknown(cfg: dict, known: dict[str, set[str]], prefix: str = "") -> list[str]:
    bad = []
    allowed = known.get(prefix)
    for k, v in cfg.items():
        field = f"{prefix}.{k}" if prefix else k
        if allowed is not None and k not in allowed:
            bad.append(field)
        elif isinstance(v, dict) and field in known:
            bad += unknown(v, known, field)
    return bad


def main() -> int:
    commit = sys.argv[1] if len(sys.argv) > 1 else (ROOT / "release/MABUR_RELEASE").read_text().split()[0]
    cpp = subprocess.run(
        ["git", "-C", str(ROOT / "third_party/mabur"), "show", f"{commit}:drone/src/config.cpp"],
        check=True, capture_output=True, text=True,
    ).stdout
    known = known_keys(cpp)
    if "" not in known:
        print(f"release-toml-check: no top-level key list in config.cpp at {commit[:8]}")
        return 1
    cfg = tomllib.loads((ROOT / "release/drone/mabur.toml").read_text())
    bad = unknown(cfg, known)
    for f in bad:
        print(f"release-toml-check: release/drone/mabur.toml key {f} is unknown to maburd at "
              f"{commit[:8]}: that maburd would not start; move the key into the release branch or out of the toml")
    if not bad:
        print(f"release-toml-check: ok (maburd {commit[:8]} knows every key)")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
