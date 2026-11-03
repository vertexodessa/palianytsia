#!/usr/bin/env bash
# Installs maburd on an OpenIPC drone over ssh, from the PC: a stock OpenIPC
# FPV image (majestic + wifibroadcast) or a drone that already runs mabur.
#   install.sh [-y] [--config] DRONE [DIR]
#                                    install or update from DIR (default: this script's dir);
#                                    an update keeps the drone's mabur.toml (the GS settings
#                                    tab writes into it) unless --config replaces it
#   install.sh [-y] DRONE BACKUP     roll back to a backup dir this script saved
#   install.sh [-y] --remove DRONE   back to the stock image: maburd out, majestic and
#                                    wifibroadcast back, reboot
#   --fresh                          run the first-install steps on a mabur drone too
# DRONE is user@host or host (root assumed). Needs ssh only (dropbear images
# often lack scp/sftp); the password is asked once per run.
#
# DIR holds maburd, and optionally mabur.toml, S96mabur (the init
# wrapper) and sensor_imx415_star6e.ko (waybeam's six-mode IMX415 driver: on
# an imx415 drone it is installed next to the stock module and
# /usr/bin/load_sigmastar loads it first, the stock one stays the fallback;
# maburd's native sensor readout above 60 fps needs it). A first install on
# a stock image:
#   - stops majestic, wifibroadcast and waybeam and moves their init scripts
#     to /etc/init.d.off (maburd runs the encoder and the radio itself; two
#     owners of the MI pipeline or the card and neither works);
#   - blacklists the kernel Wi-Fi drivers in /etc/modprobe.d/palianytsia.conf
#     (devourer drives the card from user space; a kernel driver that grabs
#     it on USB enumeration locks it out);
#   - installs the wrapper as /etc/init.d/S96mabur and reboots, so the
#     drone comes up clean with maburd alone.
# Order matters on the drone's tiny flash overlay: the old binary is removed
# BEFORE the copy, so the new one fits; the new files are checked by sha256
# in /tmp (RAM) first, so a broken upload never replaces anything. The
# config and the binary go together: maburd refuses config keys it does not
# know, so a new binary with an old config (or the reverse) may not start.
set -euo pipefail

YES=0
REMOVE=0
FORCE_CFG=0
FORCE_FRESH=0
while [[ ${1:-} == -* ]]; do
  case $1 in
    -y) YES=1 ;;
    --remove) REMOVE=1 ;;
    --config) FORCE_CFG=1 ;;
    --fresh) FORCE_FRESH=1 ;;
    *) sed -n '2,11p' "$0"; exit 2 ;;
  esac
  shift
done
[[ $# -ge 1 ]] || { sed -n '2,11p' "$0"; exit 2; }
DRONE=$1
[[ $DRONE == *@* ]] || DRONE=root@$DRONE
HERE=$(cd "$(dirname "$0")" && pwd)
SRC=${2:-$HERE}

CM=$(mktemp -d)
trap 'ssh -o ControlPath="$CM/s" -O exit "$DRONE" >/dev/null 2>&1 || true; rm -rf "$CM"' EXIT
SSH=(ssh -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ControlMaster=auto
     -o ControlPath="$CM/s" -o ControlPersist=120 "$DRONE")
sha() { sha256sum "$1" | cut -d' ' -f1; }
rsha() { "${SSH[@]}" "sha256sum $1 2>/dev/null | cut -d' ' -f1"; }
confirm() {
  [[ $YES == 1 ]] && return 0
  [[ -t 0 ]] || { echo "no terminal to confirm on; run with -y" >&2; exit 1; }
  read -r -p "$1 [y/N] " a
  [[ $a == y || $a == Y ]] || { echo "aborted, nothing changed"; exit 1; }
}
wait_back() {
  echo "== rebooting, waiting for the drone"
  sleep 10
  for _ in $(seq 1 30); do
    "${SSH[@]}" true 2>/dev/null && return 0
    sleep 3
  done
  echo "== the drone did not come back over ssh within 100 s" >&2
  exit 1
}

echo "== drone $DRONE"
"${SSH[@]}" true || { echo "cannot ssh to $DRONE" >&2; exit 1; }

if [[ $REMOVE == 1 ]]; then
  confirm "remove maburd from $DRONE and restore the stock services?"
  "${SSH[@]}" "sh -s" <<'EOF'
for s in S00mabur S96mabur; do [ -x /etc/init.d/$s ] && /etc/init.d/$s stop >/dev/null 2>&1; done
killall maburd 2>/dev/null
rm -f /usr/bin/maburd /etc/init.d/S00mabur /etc/init.d/S96mabur /etc/modprobe.d/palianytsia.conf
[ -f /usr/bin/load_sigmastar.palianytsia ] && mv /usr/bin/load_sigmastar.palianytsia /usr/bin/load_sigmastar
# Only our copy of the sensor module goes: an image that ships it (and loads
# it with no fallback) gets its own back, never a deletion mark over it.
for f in /lib/modules/*/sigmastar/sensor_imx415_star6e.ko; do
  [ -e "$f" ] || continue
  if [ -e "/rom$f" ]; then rm -f "/overlay/root$f"; else rm -f "$f"; fi
done
if [ -d /etc/init.d.off ]; then
  for f in /etc/init.d.off/*; do [ -e "$f" ] && mv "$f" /etc/init.d/; done
  rmdir /etc/init.d.off 2>/dev/null
fi
[ -f /etc/modules.palianytsia ] && mv /etc/modules.palianytsia /etc/modules
# A drone converted by hand deleted the stock files: drop the overlay's
# deletion marks so the read-only image's copies show again after reboot.
for f in etc/init.d/S95majestic etc/init.d/S98wifibroadcast usr/bin/majestic usr/bin/wfb_tx_cmd; do
  [ -c /overlay/root/$f ] && [ -e /rom/$f ] && rm -f /overlay/root/$f
done
sync
echo "   removed; /etc/mabur.toml kept"
reboot
EOF
  wait_back
  echo "== done: $("${SSH[@]}" 'ls /etc/init.d | tr "\n" " "')"
  exit 0
fi

BIN=$SRC/maburd
CFG=$SRC/mabur.toml
INIT=$SRC/S96mabur
[[ -f $INIT ]] || INIT=$HERE/S96mabur
[[ -f $INIT ]] || INIT=$HERE/drone/S96mabur
[[ -f $BIN ]] || { echo "no $BIN" >&2; exit 2; }
[[ -f $CFG ]] || CFG=""
[[ -f $INIT ]] || INIT=""
MOD=$SRC/sensor_imx415_star6e.ko
[[ -f $MOD ]] || MOD=""

# Preflight, read-only: what the drone is and what runs on it.
INFO=$("${SSH[@]}" 'sh -s' <<'EOF'
echo "soc=$(fw_printenv -n soc 2>/dev/null)"
echo "sensor=$(fw_printenv -n sensor 2>/dev/null)"
echo "mabur=$( [ -x /etc/init.d/S96mabur ] || [ -x /etc/init.d/S00mabur ] && echo yes || echo no)"
echo "stock=$(for s in S95majestic S98wifibroadcast S95waybeam; do [ -e /etc/init.d/$s ] && printf '%s ' $s; done)"
echo "cards=$(lsusb 2>/dev/null | grep -io '0bda:[0-9a-f]*' | tr '\n' ' ')"
echo "kmods=$(lsmod 2>/dev/null | awk '{print $1}' | grep -iE '^(88|87|rtl8|8812|8822|8821)' | tr '\n' ' ')"
echo "free_kb=$(df -k /overlay 2>/dev/null | awk 'NR==2{print $4}')"
echo "bin_kb=$( [ -f /usr/bin/maburd ] && du -k /usr/bin/maburd | cut -f1 || echo 0)"
echo "mod_sha=$(sha256sum /lib/modules/$(uname -r)/sigmastar/sensor_imx415_star6e.ko 2>/dev/null | cut -d' ' -f1)"
echo "ls_patched=$(grep -c sensor_imx415_star6e /usr/bin/load_sigmastar 2>/dev/null)"
EOF
)
get() { sed -n "s/^$1=//p" <<<"$INFO"; }
SOC=$(get soc); MABUR=$(get mabur); STOCK=$(get stock); CARDS=$(get cards)
FREE=$(get free_kb); OLDKB=$(get bin_kb)
printf '   soc %s, sensor %s, cards: %s\n' "${SOC:-?}" "$(get sensor)" "${CARDS:-none}"
[[ -n $(get kmods) ]] && echo "   kernel Wi-Fi drivers loaded: $(get kmods)"
[[ -n $STOCK ]] && echo "   stock services: $STOCK"

case $SOC in
  ssc338q) ;;
  ssc33*|ssc30k*) echo "   warning: maburd is tested on ssc338q only; $SOC is the same family" ;;
  *) echo "maburd is built for SigmaStar ssc338q (Infinity6E); this drone reports soc '${SOC:-?}'" >&2; exit 1 ;;
esac
[[ -n $CARDS ]] || echo "   warning: no Realtek USB card on the bus; maburd will not start without one"
NEEDKB=$(( $(du -k "$BIN" | cut -f1) + 64 ))
if (( ${FREE:-0} + ${OLDKB:-0} < NEEDKB )); then
  echo "not enough room on the overlay: ${FREE:-?} KB free + ${OLDKB} KB old maburd < ${NEEDKB} KB" >&2
  exit 1
fi

# The six-mode sensor module: only for an imx415 drone, only when shipped.
[[ -n $MOD && $(get sensor) != imx415 ]] && MOD=""
NEW_MOD=${MOD:+$(sha "$MOD")}
MOD_TODO=0
LSP=$(get ls_patched)
[[ -n $MOD && ( $(get mod_sha) != "$NEW_MOD" || ${LSP:-0} == 0 ) ]] && MOD_TODO=1
[[ $MOD_TODO == 1 ]] && echo "   sensor module: installs sensor_imx415_star6e.ko; load_sigmastar loads it first, the stock one stays the fallback"

FRESH=0
[[ $MABUR == no || -n $STOCK || $FORCE_FRESH == 1 ]] && FRESH=1

OLD_BIN=$(rsha /usr/bin/maburd)
OLD_CFG=$(rsha /etc/mabur.toml)
# A backup dir restores its config; an update keeps the drone's own.
if [[ -n $CFG && -n $OLD_CFG && $FORCE_CFG == 0 && ! -f $SRC/.drone-backup ]]; then
  echo "   keeping the drone's mabur.toml (tuned settings live there; --config replaces it)"
  CFG=""
fi
NEW_BIN=$(sha "$BIN")
NEW_CFG=${CFG:+$(sha "$CFG")}
printf '   maburd      now %s\n               new %s\n' "${OLD_BIN:-none}" "$NEW_BIN"
[[ -n $CFG ]] && printf '   mabur.toml  now %s\n               new %s\n' "${OLD_CFG:-none}" "$NEW_CFG"
if [[ $FRESH == 0 && $MOD_TODO == 0 && $OLD_BIN == "$NEW_BIN" && ( -z $CFG || $OLD_CFG == "$NEW_CFG" ) ]]; then
  echo "== already installed, nothing to do"; exit 0
fi
if [[ $FRESH == 1 && ( -z $INIT || ( -z $CFG && -z $OLD_CFG ) ) ]]; then
  echo "a first install needs S96mabur and mabur.toml next to maburd" >&2
  exit 2
fi

# Backup of what runs now, on the PC (the drone has no room for a second copy).
BK=$PWD/drone-backup-$(date +%Y%m%d-%H%M%S)
mkdir -p "$BK"
[[ -n $OLD_BIN ]] && "${SSH[@]}" 'cat /usr/bin/maburd' > "$BK/maburd"
[[ -n $OLD_CFG ]] && "${SSH[@]}" 'cat /etc/mabur.toml' > "$BK/mabur.toml"
touch "$BK/.drone-backup"
[[ -z $OLD_BIN || $(sha "$BK/maburd") == "$OLD_BIN" ]] || { echo "backup of maburd is corrupt" >&2; exit 1; }
[[ -z $OLD_CFG || $(sha "$BK/mabur.toml") == "$OLD_CFG" ]] || { echo "backup of mabur.toml is corrupt" >&2; exit 1; }
if [[ -n $OLD_BIN ]]; then
  echo "== backup: $BK   (rollback: $0 ${DRONE#root@} $BK)"
else
  rm -rf "$BK"
  echo "== no maburd on the drone yet   (undo: $0 --remove ${DRONE#root@})"
fi

if [[ $FRESH == 1 ]]; then
  echo "== first install: stops ${STOCK:-nothing}, blacklists the kernel Wi-Fi drivers, installs maburd, reboots"
fi
confirm "install on $DRONE?"

echo "== upload to /tmp"
"${SSH[@]}" 'cat > /tmp/maburd.new' < "$BIN"
[[ -n $CFG ]] && "${SSH[@]}" 'cat > /tmp/mabur.toml.new' < "$CFG"
[[ -n $INIT ]] && "${SSH[@]}" 'cat > /tmp/S96mabur.new' < "$INIT"
NEW_LS=""
LS_TODO=0
if [[ $MOD_TODO == 1 ]]; then
  "${SSH[@]}" 'cat > /tmp/sensor_imx415_star6e.ko.new' < "$MOD"
  # load_sigmastar differs between images: patch the drone's own copy here,
  # an imx415 branch first in its sensor case (the first match wins).
  LS=$CM/load_sigmastar
  "${SSH[@]}" 'cat /usr/bin/load_sigmastar.palianytsia 2>/dev/null || cat /usr/bin/load_sigmastar' > "$LS.orig"
  awk '{ print }
    /^[ \t]*case \$SENSOR in/ && !done {
      print "\t\timx415)"
      print "\t\t\t# palianytsia: waybeam six-mode IMX415 driver first, stock as the fallback"
      print "\t\t\tif [ -f $MODULE/sensor_imx415_star6e.ko ]; then"
      print "\t\t\t\tinsmod $MODULE/sensor_imx415_star6e.ko chmap=1"
      print "\t\t\telse"
      print "\t\t\t\tinsmod $MODULE/sensor_imx415_mipi.ko chmap=1"
      print "\t\t\tfi"
      print "\t\t\t;;"
      done = 1
    }' "$LS.orig" > "$LS"
  if grep -q sensor_imx415_star6e "$LS.orig"; then
    :  # this image's load_sigmastar loads it already (openipc-builder's mabur image)
  elif [[ $(grep -c sensor_imx415_star6e "$LS") != 2 ]] || ! sh -n "$LS"; then
    echo "   load_sigmastar has no 'case \$SENSOR in' to patch; sensor module skipped" >&2
    MOD_TODO=0
  else
    "${SSH[@]}" 'cat > /tmp/load_sigmastar.new' < "$LS"
    NEW_LS=$(sha "$LS")
    LS_TODO=1
  fi
fi

echo "== install"
"${SSH[@]}" "sh -s" <<EOF || { echo "== install failed on the drone (see above); rerun this script or roll back from the backup" >&2; exit 1; }
set -e
fail() { echo "   \$*"; rm -f /tmp/maburd.new /tmp/mabur.toml.new /tmp/S96mabur.new /tmp/sensor_imx415_star6e.ko.new /tmp/load_sigmastar.new; exit 1; }
[ "\$(sha256sum /tmp/maburd.new | cut -d' ' -f1)" = "$NEW_BIN" ] || fail "upload of maburd corrupt, nothing changed"
if [ -n "$CFG" ]; then
  [ "\$(sha256sum /tmp/mabur.toml.new | cut -d' ' -f1)" = "$NEW_CFG" ] || fail "upload of mabur.toml corrupt, nothing changed"
fi
chmod 755 /tmp/maburd.new
if [ "$MOD_TODO" = 1 ]; then
  [ "\$(sha256sum /tmp/sensor_imx415_star6e.ko.new | cut -d' ' -f1)" = "$NEW_MOD" ] || fail "upload of the sensor module corrupt, nothing changed"
  if [ "$LS_TODO" = 1 ]; then
    [ "\$(sha256sum /tmp/load_sigmastar.new | cut -d' ' -f1)" = "$NEW_LS" ] || fail "upload of load_sigmastar corrupt, nothing changed"
  fi
fi
if [ "$FRESH" = 1 ]; then
  mkdir -p /etc/init.d.off /etc/modprobe.d
  for s in S95majestic S98wifibroadcast S95waybeam; do
    if [ -e /etc/init.d/\$s ]; then
      /etc/init.d/\$s stop >/dev/null 2>&1 || true
      mv /etc/init.d/\$s /etc/init.d.off/
    fi
  done
  killall majestic wfb_tx wfb_rx waybeam 2>/dev/null || true
  : > /etc/modprobe.d/palianytsia.conf
  for m in \$(ls /lib/modules/*/extra 2>/dev/null | sed -n 's/\.ko\$//p' | grep -iE '^(88|87|rtl8|8812|8822|8821)'); do
    echo "blacklist \$m" >> /etc/modprobe.d/palianytsia.conf
  done
  if [ -f /etc/modules ] && grep -qiE '^(88|87|rtl8|8812|8822|8821)' /etc/modules; then
    cp /etc/modules /etc/modules.palianytsia
    sed -i -E 's/^((88|87|rtl8|8812|8822|8821)[^ ]*)/# \1/' /etc/modules
  fi
fi
# Free the overlay first, then stop (the wrapper respawns maburd until stopped).
rm -f /usr/bin/maburd
# "|| true": a stop with no maburd running ends in a failed killall, and
# under set -e that aborted here with /usr/bin/maburd already gone.
for s in S00mabur S96mabur; do if [ -x /etc/init.d/\$s ]; then /etc/init.d/\$s stop >/dev/null 2>&1 || true; fi; done
killall maburd 2>/dev/null || true
cp /tmp/maburd.new /usr/bin/maburd
chmod 755 /usr/bin/maburd
[ -n "$CFG" ] && cp /tmp/mabur.toml.new /etc/mabur.toml
if [ "$MOD_TODO" = 1 ]; then
  MD=/lib/modules/\$(uname -r)/sigmastar
  cp /tmp/sensor_imx415_star6e.ko.new \$MD/sensor_imx415_star6e.ko
  chmod 644 \$MD/sensor_imx415_star6e.ko
  if [ "$LS_TODO" = 1 ]; then
    [ -f /usr/bin/load_sigmastar.palianytsia ] || cp /usr/bin/load_sigmastar /usr/bin/load_sigmastar.palianytsia
    cp /tmp/load_sigmastar.new /usr/bin/load_sigmastar
    chmod 755 /usr/bin/load_sigmastar
  fi
  # Swap the running driver now (a first install reboots into it anyway),
  # once maburd has let go of the sensor: it can take seconds to exit. Never
  # fatal: on any failure the stock driver stays or comes back, and the next
  # boot loads the new one through load_sigmastar.
  # An update swaps a loaded six-mode driver too: otherwise the new file
  # sat on flash and the old modes ran until a reboot (2026-11-03).
  OLDMOD=\$(lsmod | grep -oE '^sensor_imx415_(mipi|star6e)' | head -1)
  if [ "$FRESH" != 1 ] && [ -n "\$OLDMOD" ]; then
    for i in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15; do pidof maburd >/dev/null || break; sleep 1; done
    if pidof maburd >/dev/null; then
      echo "   maburd still running: the six-mode driver loads on the next boot"
    elif rmmod \$OLDMOD; then
      insmod \$MD/sensor_imx415_star6e.ko chmap=1 || { echo "   six-mode module did not load, stock driver back"; insmod \$MD/sensor_imx415_mipi.ko chmap=1 || echo "   stock driver did not load either: reboot the drone"; }
    else
      echo "   \$OLDMOD busy: the new six-mode driver loads on the next boot"
    fi
  fi
  rm -f /tmp/sensor_imx415_star6e.ko.new /tmp/load_sigmastar.new
fi
if [ "$FRESH" = 1 ]; then
  rm -f /etc/init.d/S00mabur
  cp /tmp/S96mabur.new /etc/init.d/S96mabur
  chmod 755 /etc/init.d/S96mabur
fi
sync
[ "\$(sha256sum /usr/bin/maburd | cut -d' ' -f1)" = "$NEW_BIN" ] || { echo "   maburd on flash differs (overlay full?)"; exit 1; }
if [ -n "$CFG" ]; then
  [ "\$(sha256sum /etc/mabur.toml | cut -d' ' -f1)" = "$NEW_CFG" ] || { echo "   mabur.toml on flash differs (overlay full?)"; exit 1; }
fi
rm -f /tmp/maburd.new /tmp/mabur.toml.new /tmp/S96mabur.new
if [ "$FRESH" = 1 ]; then
  reboot
else
  # setsid: the init script's respawn loop must not die with this ssh session.
  W=S96mabur; [ -x /etc/init.d/S96mabur ] || W=S00mabur
  setsid /etc/init.d/\$W start </dev/null >/dev/null 2>&1
fi
EOF

[[ $FRESH == 1 ]] && wait_back
echo "== waiting for maburd"
for _ in $(seq 1 30); do
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
if [[ -n $OLD_BIN ]]; then
  echo "== roll back with: $0 ${DRONE#root@} $BK" >&2
else
  echo "== undo with: $0 --remove ${DRONE#root@}" >&2
fi
exit 1
