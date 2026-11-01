#!/usr/bin/env bash
# Installs maburd (and its mabur.toml) on an OpenIPC drone over ssh, from the PC.
#   drone-install.sh [-y] DRONE [DIR]   install DIR/maburd + DIR/mabur.toml (DIR defaults to this script's dir)
#   drone-install.sh [-y] DRONE BACKUP  rollback: same command with a backup dir the script saved earlier
# DRONE is user@host or host (root assumed). Needs ssh only: the files go up
# through `ssh 'cat > /tmp/x' < file` (dropbear images often lack scp/sftp).
#
# Order matters on the drone's tiny flash overlay: the old binary is removed
# BEFORE the copy, so the new one fits; the new files are checked by sha256
# in /tmp (RAM) first, so a broken upload never replaces anything. The
# config and the binary go together: maburd refuses config keys it does not
# know, so a new binary with an old config (or the reverse) may not start.
set -euo pipefail

YES=0
if [[ ${1:-} == -y ]]; then YES=1; shift; fi
[[ $# -ge 1 ]] || { sed -n '2,4p' "$0"; exit 2; }
DRONE=$1
[[ $DRONE == *@* ]] || DRONE=root@$DRONE
SRC=${2:-$(cd "$(dirname "$0")" && pwd)}
BIN=$SRC/maburd
CFG=$SRC/mabur.toml
[[ -f $BIN ]] || { echo "no $BIN" >&2; exit 2; }
[[ -f $CFG ]] || CFG=""

SSH=(ssh -o ConnectTimeout=8 -o ServerAliveInterval=5 "$DRONE")
sha() { sha256sum "$1" | cut -d' ' -f1; }
rsha() { "${SSH[@]}" "sha256sum $1 2>/dev/null | cut -d' ' -f1"; }

echo "== drone $DRONE"
"${SSH[@]}" 'test -x /etc/init.d/S96mabur' || { echo "no /etc/init.d/S96mabur on the drone: not a mabur image" >&2; exit 1; }
OLD_BIN=$(rsha /usr/bin/maburd)
OLD_CFG=$(rsha /etc/mabur.toml)
NEW_BIN=$(sha "$BIN")
NEW_CFG=${CFG:+$(sha "$CFG")}
printf '   maburd      now %s\n               new %s\n' "${OLD_BIN:-none}" "$NEW_BIN"
[[ -n $CFG ]] && printf '   mabur.toml  now %s\n               new %s\n' "${OLD_CFG:-none}" "$NEW_CFG"
if [[ $OLD_BIN == "$NEW_BIN" && ( -z $CFG || $OLD_CFG == "$NEW_CFG" ) ]]; then
  echo "== already installed, nothing to do"; exit 0
fi

# Backup of what runs now, on the PC (the drone has no room for a second copy).
BK=$PWD/drone-backup-$(date +%Y%m%d-%H%M%S)
mkdir -p "$BK"
[[ -n $OLD_BIN ]] && "${SSH[@]}" 'cat /usr/bin/maburd' > "$BK/maburd"
[[ -n $OLD_CFG ]] && "${SSH[@]}" 'cat /etc/mabur.toml' > "$BK/mabur.toml"
[[ -z $OLD_BIN || $(sha "$BK/maburd") == "$OLD_BIN" ]] || { echo "backup of maburd is corrupt" >&2; exit 1; }
[[ -z $OLD_CFG || $(sha "$BK/mabur.toml") == "$OLD_CFG" ]] || { echo "backup of mabur.toml is corrupt" >&2; exit 1; }
echo "== backup: $BK   (rollback: $0 ${DRONE#root@} $BK)"

if [[ $YES == 0 ]]; then
  read -r -p "install on $DRONE? [y/N] " a
  [[ $a == y || $a == Y ]] || { echo "aborted, nothing changed"; exit 1; }
fi

echo "== upload to /tmp"
"${SSH[@]}" 'cat > /tmp/maburd.new' < "$BIN"
[[ -n $CFG ]] && "${SSH[@]}" 'cat > /tmp/mabur.toml.new' < "$CFG"

echo "== install"
"${SSH[@]}" "sh -s" <<EOF
set -e
fail() { echo "   \$*"; rm -f /tmp/maburd.new /tmp/mabur.toml.new; exit 1; }
[ "\$(sha256sum /tmp/maburd.new | cut -d' ' -f1)" = "$NEW_BIN" ] || fail "upload of maburd corrupt, nothing changed"
if [ -n "$CFG" ]; then
  [ "\$(sha256sum /tmp/mabur.toml.new | cut -d' ' -f1)" = "$NEW_CFG" ] || fail "upload of mabur.toml corrupt, nothing changed"
fi
chmod 755 /tmp/maburd.new
# Free the overlay first, then stop (S96mabur respawns maburd until stopped).
rm -f /usr/bin/maburd
/etc/init.d/S96mabur stop >/dev/null 2>&1 || true
cp /tmp/maburd.new /usr/bin/maburd
chmod 755 /usr/bin/maburd
[ -n "$CFG" ] && cp /tmp/mabur.toml.new /etc/mabur.toml
sync
[ "\$(sha256sum /usr/bin/maburd | cut -d' ' -f1)" = "$NEW_BIN" ] || { echo "   maburd on flash differs (overlay full?)"; exit 1; }
if [ -n "$CFG" ]; then
  [ "\$(sha256sum /etc/mabur.toml | cut -d' ' -f1)" = "$NEW_CFG" ] || { echo "   mabur.toml on flash differs (overlay full?)"; exit 1; }
fi
rm -f /tmp/maburd.new /tmp/mabur.toml.new
# setsid: the init script's respawn loop must not die with this ssh session.
setsid /etc/init.d/S96mabur start </dev/null >/dev/null 2>&1
EOF

echo "== started, waiting for maburd"
for _ in $(seq 1 15); do
  sleep 1
  line=$("${SSH[@]}" 'pidof maburd >/dev/null && grep -m1 "build mabur" /tmp/mabur.log' 2>/dev/null || true)
  if [[ -n $line ]]; then
    echo "   $line"
    sleep 3
    if "${SSH[@]}" 'pidof maburd' >/dev/null; then
      echo "== done"; exit 0
    fi
    break
  fi
done
echo "== maburd is not running; last log lines:" >&2
"${SSH[@]}" 'tail -15 /tmp/mabur.log' >&2 || true
echo "== roll back with: $0 ${DRONE#root@} $BK" >&2
exit 1
