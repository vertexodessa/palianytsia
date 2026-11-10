#!/usr/bin/env bash
# Checks the facts the docs have to repeat against the tree they describe:
#   1. each submodule's .gitmodules branch contains its pinned commit;
#   2. release/MABUR_RELEASE is the tip of a mabur release/v* branch (a GS-only patch release
#      reuses the previous one's);
#   3. every `RC_VERSION N` in README.md, CLAUDE.md and release/README.md equals rc_proto.h;
#   4. the latest release/notes tag heads release/README.md, is release/VERSION (the one
#      version: both APKs' versionName and the native builds read it, no other copy) and is
#      named in docs/spec.md Status and CLAUDE.md;
#   5. every repo path in backticks in the docs exists, here or in third_party/mabur (the spec
#      quotes mabur's own docs/ and tools/ by their mabur paths).
# Checks 1-2 need the fork remotes. Offline they warn; with CI set they fail.
#   tools/doc-check.sh
set -uo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"
fail=0
err() { echo "doc-check: $*" >&2; fail=1; }
net() { if [ -n "${CI:-}" ]; then err "$*"; else echo "doc-check: warning: $*" >&2; fi; }

# 1. A pin off its named branch makes `git submodule update --remote` move it to an
# unrelated line (it happened: mabur on base-v2 while .gitmodules said base).
while read -r key br; do
  name=${key#submodule.}; name=${name%.branch}
  path=$(git config -f .gitmodules "submodule.$name.path")
  pin=$(git rev-parse "HEAD:$path" 2>/dev/null) || { err "$path: no gitlink"; continue; }
  if ! git -C "$path" rev-parse -q --verify "origin/$br" >/dev/null ||
     ! git -C "$path" merge-base --is-ancestor "$pin" "origin/$br" 2>/dev/null; then
    git -C "$path" fetch -q origin "+refs/heads/$br:refs/remotes/origin/$br" 2>/dev/null ||
      { net "$path: cannot fetch branch $br"; continue; }
  fi
  git -C "$path" merge-base --is-ancestor "$pin" "origin/$br" 2>/dev/null ||
    err "$path: pin ${pin:0:8} is not on branch $br named in .gitmodules"
done < <(git config -f .gitmodules --get-regexp '^submodule\..*\.branch$')

tag=$(ls release/notes | sed -n 's/^\(v[0-9.]*\)\.md$/\1/p' | sort -V | tail -1)
[ -n "$tag" ] || err "no release/notes/v*.md"

# 2. The release builds maburd from this commit, not from the submodule pin.
want=$(tr -d '[:space:]' < release/MABUR_RELEASE)
if tips=$(git -C third_party/mabur ls-remote origin 'refs/heads/release/*' 2>/dev/null); then
  br=$(awk -v w="$want" '$1 == w { sub("refs/heads/", "", $2); print $2 }' <<<"$tips" | head -1)
  [ -n "$br" ] || err "release/MABUR_RELEASE ${want:0:8} is not the tip of any mabur release/* branch"
else
  net "cannot reach the mabur remote for its release/* branches"
fi

# 3-5 are text checks.
rc=$(sed -n 's/.*constexpr uint8_t RC_VERSION = \([0-9]*\);.*/\1/p' \
  third_party/mabur/common/include/mabur/rc_proto.h)
[ -n "$rc" ] || err "RC_VERSION not found in rc_proto.h (submodule not checked out?)"

python3 -I - "$rc" "$tag" <<'EOF' || fail=1
import pathlib, re, sys
rc, tag = sys.argv[1], sys.argv[2]
bad = []
def read(p): return pathlib.Path(p).read_text()

for p in ["README.md", "CLAUDE.md", "release/README.md"]:
    for m in re.finditer(r"RC_VERSION\s*=?\s*(\d+)", read(p)):
        if m.group(1) != rc:
            bad.append(f"{p}: says RC_VERSION {m.group(1)}, rc_proto.h has {rc}")

if tag:
    head = read("release/README.md").splitlines()[0]
    if head != f"# Palianytsia {tag}":
        bad.append(f"release/README.md heading is '{head}', latest notes are {tag}")
    ver = tag[1:] if tag.count(".") == 2 else tag[1:] + ".0"
    have = read("release/VERSION").strip()
    if have != ver:
        bad.append(f"release/VERSION is {have}, latest notes are {tag} ({ver})")
    for app in ["phone", "quest"]:
        g = f"apps/{app}/android/app/build.gradle"
        m = re.search(r"^\s*versionName\s+(.*)$", read(g), re.M)
        if not m or "release/VERSION" not in m.group(1):
            bad.append(f"{g}: versionName does not read release/VERSION: {m and m.group(1)}")
    spec = read("docs/spec.md")
    status = spec.split("## Status", 1)[-1].split("\n---", 1)[0]
    if not re.search(rf"{re.escape(tag)}\b", status):
        bad.append(f"docs/spec.md Status does not name {tag}")
    if not re.search(rf"{re.escape(tag)}\b", read("CLAUDE.md")):
        bad.append(f"CLAUDE.md does not name {tag}")

# A path is checked when it starts with one of the repo's top directories. Words with a
# placeholder (<x>, *, $X, ...) are templates, not paths. git-ignored dirs are skipped.
TOP = ("crates/", "apps/", "tools/", "docs/", "configs/", "release/", "third_party/",
       ".github/")
def expand(w):
    m = re.search(r"\{([^{}]*)\}", w)
    if not m: return [w]
    return [x for part in m.group(1).split(",")
            for x in expand(w[:m.start()] + part + w[m.end():])]
docs = ["README.md", "CLAUDE.md", "release/README.md"] + sorted(
    str(p) for p in pathlib.Path("docs").glob("*.md"))
for p in docs:
    for n, line in enumerate(read(p).splitlines(), 1):
        for span in re.findall(r"`([^`\n]+)`", line):
            for w in span.split():
                w = w.rstrip(".,;:)'\"")
                if not w.startswith(TOP) or re.search(r"[<>*$]|\.\.\.|@", w):
                    continue
                if re.match(r"release/v[0-9X]", w):   # a mabur fork branch, not a path
                    continue
                w = re.sub(r":\d+(-\d+)?$", "", w)
                for x in expand(w):
                    if not (pathlib.Path(x).exists() or
                            pathlib.Path("third_party/mabur", x).exists()):
                        bad.append(f"{p}:{n}: `{x}` does not exist")
for b in bad: print("doc-check:", b, file=sys.stderr)
sys.exit(1 if bad else 0)
EOF

[ "$fail" = 0 ] && echo "doc-check: ok (RC_VERSION $rc, release $tag)"
exit "$fail"
