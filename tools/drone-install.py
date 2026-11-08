#!/usr/bin/env python3
"""Installs maburd on an OpenIPC drone over ssh, from the PC: a stock OpenIPC
FPV image (majestic + wifibroadcast) or a drone that already runs mabur.

  install.py [-y] [--config] DRONE [DIR]
                                   install or update from DIR (default: this script's dir);
                                   an update keeps the drone's mabur.toml (the GS settings
                                   tab writes into it) unless --config replaces it
  install.py [-y] DRONE BACKUP     roll back to a backup dir this script saved
  install.py [-y] --remove DRONE   back to the stock image: maburd out, majestic and
                                   wifibroadcast back, reboot
  --fresh                          run the first-install steps on a mabur drone too

DRONE is user@host or host (root assumed). Needs python3 and ssh on the PC
and ssh only on the drone (dropbear images often lack scp/sftp); the password
is asked once per run.

DIR holds maburd, and optionally mabur.toml, S96mabur (the init wrapper) and
sensor_imx415_star6e.ko (waybeam's six-mode IMX415 driver: on an imx415 drone
it is installed next to the stock module and /usr/bin/load_sigmastar loads it
first, the stock one stays the fallback; maburd's native sensor readout above
60 fps needs it). A first install on a stock image:
  - stops majestic, wifibroadcast and waybeam and moves their init scripts to
    /etc/init.d.off (maburd runs the encoder and the radio itself; two owners
    of the MI pipeline or the card and neither works);
  - blacklists the kernel Wi-Fi drivers in /etc/modprobe.d/palianytsia.conf
    (devourer drives the card from user space; a kernel driver that grabs it
    on USB enumeration locks it out);
  - installs the wrapper as /etc/init.d/S96mabur and reboots, so the drone
    comes up clean with maburd alone.

Order matters on the drone's tiny flash overlay: the old binary is removed
BEFORE the copy, so the new one fits; the new files are checked by sha256 in
/tmp (RAM) first, so a broken upload never replaces anything. The config and
the binary go together: maburd refuses config keys it does not know, so a new
binary with an old config (or the reverse) would never start. migrate_config
edits the config that lands on the drone to suit the new binary (the original
stays in the backup).
"""
import argparse
import fnmatch
import hashlib
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
import time
from datetime import datetime
from pathlib import Path

FEC_WINDOW = 64  # the window the frame-tail repair rule is measured with
REMOVED_KEY = b"repairs_with_frame"  # [fec], gone with the frame-tail rule
FRAME_TAIL_MARK = b"frame_end_us/rep="  # a log format only that rule's builds print
MODULE = "sensor_imx415_star6e.ko"

# These three run on the drone, which has busybox sh and no Python.
REMOVE_SH = r'''for s in S00mabur S96mabur; do [ -x /etc/init.d/$s ] && /etc/init.d/$s stop >/dev/null 2>&1; done
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
'''

PREFLIGHT_SH = r'''echo "soc=$(fw_printenv -n soc 2>/dev/null)"
echo "sensor=$(fw_printenv -n sensor 2>/dev/null)"
echo "mabur=$( [ -x /etc/init.d/S96mabur ] || [ -x /etc/init.d/S00mabur ] && echo yes || echo no)"
echo "stock=$(for s in S95majestic S98wifibroadcast S95waybeam; do [ -e /etc/init.d/$s ] && printf '%s ' $s; done)"
echo "cards=$(lsusb 2>/dev/null | grep -io '0bda:[0-9a-f]*' | tr '\n' ' ')"
echo "kmods=$(lsmod 2>/dev/null | awk '{print $1}' | grep -iE '^(88|87|rtl8|8812|8822|8821)' | tr '\n' ' ')"
echo "free_kb=$(df -k /overlay 2>/dev/null | awk 'NR==2{print $4}')"
echo "bin_kb=$( [ -f /usr/bin/maburd ] && du -k /usr/bin/maburd | cut -f1 || echo 0)"
echo "mod_sha=$(sha256sum /lib/modules/$(uname -r)/sigmastar/sensor_imx415_star6e.ko 2>/dev/null | cut -d' ' -f1)"
echo "ls_patched=$(grep -c sensor_imx415_star6e /usr/bin/load_sigmastar 2>/dev/null)"
echo "old_tail=$(grep -c 'frame_end_us/rep=' /usr/bin/maburd 2>/dev/null)"
'''

# Run with NEW_BIN, HAVE_CFG, NEW_CFG, MOD_TODO, NEW_MOD, LS_TODO, NEW_LS and
# FRESH set in front of it.
INSTALL_SH = r'''set -e
fail() { echo "   $*"; rm -f /tmp/maburd.new /tmp/mabur.toml.new /tmp/S96mabur.new /tmp/sensor_imx415_star6e.ko.new /tmp/load_sigmastar.new; exit 1; }
[ "$(sha256sum /tmp/maburd.new | cut -d' ' -f1)" = "$NEW_BIN" ] || fail "upload of maburd corrupt, nothing changed"
if [ -n "$HAVE_CFG" ]; then
  [ "$(sha256sum /tmp/mabur.toml.new | cut -d' ' -f1)" = "$NEW_CFG" ] || fail "upload of mabur.toml corrupt, nothing changed"
fi
chmod 755 /tmp/maburd.new
if [ "$MOD_TODO" = 1 ]; then
  [ "$(sha256sum /tmp/sensor_imx415_star6e.ko.new | cut -d' ' -f1)" = "$NEW_MOD" ] || fail "upload of the sensor module corrupt, nothing changed"
  if [ "$LS_TODO" = 1 ]; then
    [ "$(sha256sum /tmp/load_sigmastar.new | cut -d' ' -f1)" = "$NEW_LS" ] || fail "upload of load_sigmastar corrupt, nothing changed"
  fi
fi
if [ "$FRESH" = 1 ]; then
  mkdir -p /etc/init.d.off /etc/modprobe.d
  for s in S95majestic S98wifibroadcast S95waybeam; do
    if [ -e /etc/init.d/$s ]; then
      /etc/init.d/$s stop >/dev/null 2>&1 || true
      mv /etc/init.d/$s /etc/init.d.off/
    fi
  done
  killall majestic wfb_tx wfb_rx waybeam 2>/dev/null || true
  : > /etc/modprobe.d/palianytsia.conf
  for m in $(ls /lib/modules/*/extra 2>/dev/null | sed -n 's/\.ko$//p' | grep -iE '^(88|87|rtl8|8812|8822|8821)'); do
    echo "blacklist $m" >> /etc/modprobe.d/palianytsia.conf
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
for s in S00mabur S96mabur; do if [ -x /etc/init.d/$s ]; then /etc/init.d/$s stop >/dev/null 2>&1 || true; fi; done
killall maburd 2>/dev/null || true
cp /tmp/maburd.new /usr/bin/maburd
chmod 755 /usr/bin/maburd
[ -n "$HAVE_CFG" ] && cp /tmp/mabur.toml.new /etc/mabur.toml
if [ "$MOD_TODO" = 1 ]; then
  MD=/lib/modules/$(uname -r)/sigmastar
  cp /tmp/sensor_imx415_star6e.ko.new $MD/sensor_imx415_star6e.ko
  chmod 644 $MD/sensor_imx415_star6e.ko
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
  OLDMOD=$(lsmod | grep -oE '^sensor_imx415_(mipi|star6e)' | head -1)
  if [ "$FRESH" != 1 ] && [ -n "$OLDMOD" ]; then
    for i in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15; do pidof maburd >/dev/null || break; sleep 1; done
    if pidof maburd >/dev/null; then
      echo "   maburd still running: the six-mode driver loads on the next boot"
    elif rmmod $OLDMOD; then
      insmod $MD/sensor_imx415_star6e.ko chmap=1 || { echo "   six-mode module did not load, stock driver back"; insmod $MD/sensor_imx415_mipi.ko chmap=1 || echo "   stock driver did not load either: reboot the drone"; }
    else
      echo "   $OLDMOD busy: the new six-mode driver loads on the next boot"
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
[ "$(sha256sum /usr/bin/maburd | cut -d' ' -f1)" = "$NEW_BIN" ] || { echo "   maburd on flash differs (overlay full?)"; exit 1; }
if [ -n "$HAVE_CFG" ]; then
  [ "$(sha256sum /etc/mabur.toml | cut -d' ' -f1)" = "$NEW_CFG" ] || { echo "   mabur.toml on flash differs (overlay full?)"; exit 1; }
fi
rm -f /tmp/maburd.new /tmp/mabur.toml.new /tmp/S96mabur.new
if [ "$FRESH" = 1 ]; then
  reboot
else
  # setsid: the init script's respawn loop must not die with this ssh session.
  W=S96mabur; [ -x /etc/init.d/S96mabur ] || W=S00mabur
  setsid /etc/init.d/$W start </dev/null >/dev/null 2>&1
fi
'''

LOAD_SIGMASTAR_BRANCH = b"""\t\timx415)
\t\t\t# palianytsia: waybeam six-mode IMX415 driver first, stock as the fallback
\t\t\tif [ -f $MODULE/sensor_imx415_star6e.ko ]; then
\t\t\t\tinsmod $MODULE/sensor_imx415_star6e.ko chmap=1
\t\t\telse
\t\t\t\tinsmod $MODULE/sensor_imx415_mipi.ko chmap=1
\t\t\tfi
\t\t\t;;
"""


class Abort(Exception):
    """Stops the run with a message on stderr and an exit code."""

    def __init__(self, message, code=1):
        super().__init__(message)
        self.code = code


def say(message=""):
    print(message, flush=True)


def warn(message):
    print(message, file=sys.stderr, flush=True)


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


class Drone:
    """One multiplexed ssh connection, so the password is asked once."""

    def __init__(self, target, control_dir):
        self.target = target
        self.control = f"{control_dir}/s"
        self.ssh = [
            "ssh", "-o", "ConnectTimeout=8", "-o", "ServerAliveInterval=5",
            "-o", "ControlMaster=auto", "-o", f"ControlPath={self.control}",
            "-o", "ControlPersist=120", target,
        ]

    @property
    def name(self):
        """The target as the user would type it."""
        return self.target.removeprefix("root@")

    def close(self):
        subprocess.run(
            ["ssh", "-o", f"ControlPath={self.control}", "-O", "exit", self.target],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False,
        )

    def run(self, command, *, script=None, stdin=None, stdout=None, quiet=False, check=True):
        return subprocess.run(
            [*self.ssh, command],
            input=script.encode() if script is not None else None,
            stdin=stdin, stdout=stdout,
            stderr=subprocess.DEVNULL if quiet else None,
            check=check,
        )

    def reachable(self, quiet=False):
        return self.run("true", quiet=quiet, check=False).returncode == 0

    def read(self, command, *, script=None, quiet=False, check=True):
        """The command's stdout, without trailing newlines."""
        done = self.run(command, script=script, stdout=subprocess.PIPE, quiet=quiet, check=check)
        return done.stdout.decode(errors="replace").rstrip("\n")

    def sha256(self, path):
        return self.read(f"sha256sum {path} 2>/dev/null | cut -d' ' -f1")

    def fetch(self, remote, local):
        with open(local, "wb") as f:
            self.run(f"cat {remote}", stdout=f)

    def upload(self, local, remote):
        with open(local, "rb") as f:
            self.run(f"cat > {remote}", stdin=f)

    def wait_back(self):
        say("== rebooting, waiting for the drone")
        time.sleep(10)
        for _ in range(30):
            if self.reachable(quiet=True):
                return
            time.sleep(3)
        raise Abort("== the drone did not come back over ssh within 100 s")


def confirm(args, question):
    if args.yes:
        return
    if not sys.stdin.isatty():
        raise Abort("no terminal to confirm on; run with -y")
    if input(f"{question} [y/N] ") not in ("y", "Y"):
        say("aborted, nothing changed")
        raise SystemExit(1)


def migrate_config(text, strip_key, raise_window):
    """Fits a mabur.toml to the maburd about to run; returns (text, changes).

    A maburd that does not know fec.repairs_with_frame refuses to start on a
    config that has it. A drone leaving a build from before the frame-tail
    rule, on a config with no such key, had its window raised to 64 by that
    build at boot; the new maburd no longer does, so the one-time raise is
    made here. A window set by hand on a current build is left alone.
    """
    out, changes = [], []
    in_fec = False
    for line in text.splitlines(keepends=True):
        bare = line.rstrip(b"\r\n")
        if re.match(rb"[ \t]*\[", bare):
            in_fec = re.fullmatch(rb"[ \t]*\[fec\][ \t]*(#.*)?", bare) is not None
        if in_fec and strip_key and re.match(rb"[ \t]*repairs_with_frame[ \t]*=", bare):
            changes.append("removed fec.repairs_with_frame, this maburd does not have it")
            continue
        window = re.match(rb"[ \t]*window[ \t]*=[ \t]*(\d+)", bare) if in_fec and raise_window else None
        if window and int(window[1]) < FEC_WINDOW:
            changes.append(f"fec.window {int(window[1])} raised to {FEC_WINDOW}, the window this maburd is tested with")
            line = re.sub(rb"=[ \t]*\d+", b"= %d" % FEC_WINDOW, line, count=1)
        out.append(line)
    return b"".join(out), changes


def patched_load_sigmastar(original):
    """load_sigmastar differs between images: an imx415 branch goes first in
    the drone's own sensor case (the first match wins)."""
    lines = original.splitlines(keepends=True)
    for i, line in enumerate(lines):
        if re.match(rb"[ \t]*case \$SENSOR in", line):
            lines.insert(i + 1, LOAD_SIGMASTAR_BRANCH)
            return b"".join(lines)
    return original


def preflight(drone):
    """Read-only: what the drone is and what runs on it."""
    text = drone.read("sh -s", script=PREFLIGHT_SH)
    return dict(line.split("=", 1) for line in text.split("\n") if "=" in line)


def check_drone(info, binary):
    say(f"   soc {info['soc'] or '?'}, sensor {info['sensor']}, cards: {info['cards'] or 'none'}")
    if info["kmods"]:
        say(f"   kernel Wi-Fi drivers loaded: {info['kmods']}")
    if info["stock"]:
        say(f"   stock services: {info['stock']}")
    soc = info["soc"]
    if soc != "ssc338q":
        if not fnmatch.fnmatch(soc, "ssc33*") and not fnmatch.fnmatch(soc, "ssc30k*"):
            raise Abort(f"maburd is built for SigmaStar ssc338q (Infinity6E); this drone reports soc '{soc or '?'}'")
        say(f"   warning: maburd is tested on ssc338q only; {soc} is the same family")
    if not info["cards"]:
        say("   warning: no Realtek USB card on the bus; maburd will not start without one")
    need_kb = (binary.stat().st_blocks + 1) // 2 + 64
    free_kb, old_kb = int(info["free_kb"] or 0), int(info["bin_kb"] or 0)
    if free_kb + old_kb < need_kb:
        raise Abort(f"not enough room on the overlay: {info['free_kb'] or '?'} KB free + {info['bin_kb']} KB old maburd < {need_kb} KB")


def remove(args, drone):
    confirm(args, f"remove maburd from {drone.target} and restore the stock services?")
    drone.run("sh -s", script=REMOVE_SH)
    drone.wait_back()
    listing = drone.read('ls /etc/init.d | tr "\\n" " "')
    say(f"== done: {listing}")


def prepare_config(drone, cfg, old_cfg, binary_data, old_bin, info, workdir):
    """The config that lands on the drone must suit the new binary, a kept one
    too: rerunning this script also repairs a drone stuck on a refused key.
    Returns the path to upload, or None to leave the drone's file alone."""
    source = workdir / "mabur.toml.in"
    if cfg:
        shutil.copyfile(cfg, source)
    elif old_cfg:
        drone.fetch("/etc/mabur.toml", source)
        if sha256(source) != old_cfg:
            raise Abort("read of the drone's mabur.toml is corrupt")
    else:
        return None
    text = source.read_bytes()
    raise_window = (
        FRAME_TAIL_MARK in binary_data and bool(old_bin)
        and (info["old_tail"] or "0") == "0"
        and re.search(rb"(?m)^[ \t]*repairs_with_frame[ \t]*=", text) is None
    )
    migrated, changes = migrate_config(text, REMOVED_KEY not in binary_data, raise_window)
    for change in changes:
        warn(f"   mabur.toml: {change}")
    if migrated == text:
        return cfg
    target = workdir / "mabur.toml"
    target.write_bytes(migrated)
    return target


def backup(drone, old_bin, old_cfg, script):
    """What runs now goes to the PC: the drone has no room for a second copy."""
    path = Path.cwd() / f"drone-backup-{datetime.now():%Y%m%d-%H%M%S}"
    path.mkdir()
    if old_bin:
        drone.fetch("/usr/bin/maburd", path / "maburd")
    if old_cfg:
        drone.fetch("/etc/mabur.toml", path / "mabur.toml")
    (path / ".drone-backup").touch()
    if old_bin and sha256(path / "maburd") != old_bin:
        raise Abort("backup of maburd is corrupt")
    if old_cfg and sha256(path / "mabur.toml") != old_cfg:
        raise Abort("backup of mabur.toml is corrupt")
    if old_bin:
        say(f"== backup: {path}   (rollback: {script} {drone.name} {path})")
        return path
    shutil.rmtree(path)
    say(f"== no maburd on the drone yet   (undo: {script} --remove {drone.name})")
    return None


def upload_load_sigmastar(drone, workdir):
    """Returns the sha of the patched load_sigmastar uploaded, "" when this
    image already loads the module, None when it cannot be patched."""
    original_path = workdir / "load_sigmastar.orig"
    with open(original_path, "wb") as f:
        drone.run("cat /usr/bin/load_sigmastar.palianytsia 2>/dev/null || cat /usr/bin/load_sigmastar", stdout=f)
    original = original_path.read_bytes()
    if MODULE.encode() in original:
        return ""  # this image's load_sigmastar loads it already (openipc-builder's mabur image)
    patched_path = workdir / "load_sigmastar"
    patched = patched_load_sigmastar(original)
    patched_path.write_bytes(patched)
    matches = sum(MODULE.encode() in line for line in patched.splitlines())
    syntax = subprocess.run(["sh", "-n", str(patched_path)], stderr=subprocess.DEVNULL, check=False)
    if matches != 2 or syntax.returncode != 0:
        return None
    drone.upload(patched_path, "/tmp/load_sigmastar.new")
    return sha256(patched_path)


def wait_for_maburd(drone):
    say("== waiting for maburd")
    for _ in range(30):
        time.sleep(1)
        line = drone.read('pidof maburd >/dev/null && grep -m1 "build mabur" /tmp/mabur.log', quiet=True, check=False)
        if line:
            say(f"   {line}")
            time.sleep(3)
            if drone.run("pidof maburd", stdout=subprocess.DEVNULL, check=False).returncode == 0:
                return True
            break
    return False


def install(args, drone, workdir, script):
    here = Path(__file__).resolve().parent
    src = Path(args.dir) if args.dir else here
    binary = src / "maburd"
    if not binary.is_file():
        raise Abort(f"no {binary}", 2)
    cfg = src / "mabur.toml" if (src / "mabur.toml").is_file() else None
    init = next((p for p in (src / "S96mabur", here / "S96mabur", here / "drone" / "S96mabur") if p.is_file()), None)
    module = src / MODULE if (src / MODULE).is_file() else None

    info = preflight(drone)
    check_drone(info, binary)

    # The six-mode sensor module: only for an imx415 drone, only when shipped.
    if info["sensor"] != "imx415":
        module = None
    new_mod = sha256(module) if module else ""
    mod_todo = bool(module) and (info["mod_sha"] != new_mod or (info["ls_patched"] or "0") == "0")
    if mod_todo:
        say(f"   sensor module: installs {MODULE}; load_sigmastar loads it first, the stock one stays the fallback")

    fresh = info["mabur"] == "no" or bool(info["stock"]) or args.fresh
    old_bin = drone.sha256("/usr/bin/maburd")
    old_cfg = drone.sha256("/etc/mabur.toml")
    # A backup dir restores its config; an update keeps the drone's own.
    if cfg and old_cfg and not args.config and not (src / ".drone-backup").is_file():
        say("   keeping the drone's mabur.toml (tuned settings live there; --config replaces it)")
        cfg = None
    cfg = prepare_config(drone, cfg, old_cfg, binary.read_bytes(), old_bin, info, workdir)

    new_bin = sha256(binary)
    new_cfg = sha256(cfg) if cfg else ""
    say(f"   maburd      now {old_bin or 'none'}\n               new {new_bin}")
    if cfg:
        say(f"   mabur.toml  now {old_cfg or 'none'}\n               new {new_cfg}")
    if not fresh and not mod_todo and old_bin == new_bin and (not cfg or old_cfg == new_cfg):
        say("== already installed, nothing to do")
        return 0
    if fresh and (not init or (not cfg and not old_cfg)):
        raise Abort("a first install needs S96mabur and mabur.toml next to maburd", 2)

    saved = backup(drone, old_bin, old_cfg, script)
    if fresh:
        say(f"== first install: stops {info['stock'] or 'nothing'}, blacklists the kernel Wi-Fi drivers, installs maburd, reboots")
    confirm(args, f"install on {drone.target}?")

    say("== upload to /tmp")
    drone.upload(binary, "/tmp/maburd.new")
    if cfg:
        drone.upload(cfg, "/tmp/mabur.toml.new")
    if init:
        drone.upload(init, "/tmp/S96mabur.new")
    new_ls = ""
    if mod_todo:
        drone.upload(module, f"/tmp/{MODULE}.new")
        new_ls = upload_load_sigmastar(drone, workdir)
        if new_ls is None:
            warn("   load_sigmastar has no 'case $SENSOR in' to patch; sensor module skipped")
            mod_todo, new_ls = False, ""

    say("== install")
    values = {
        "NEW_BIN": new_bin, "HAVE_CFG": "1" if cfg else "", "NEW_CFG": new_cfg,
        "MOD_TODO": "1" if mod_todo else "0", "NEW_MOD": new_mod,
        "LS_TODO": "1" if new_ls else "0", "NEW_LS": new_ls,
        "FRESH": "1" if fresh else "0",
    }
    header = "".join(f"{k}={shlex.quote(v)}\n" for k, v in values.items())
    if drone.run("sh -s", script=header + INSTALL_SH, check=False).returncode != 0:
        raise Abort("== install failed on the drone (see above); rerun this script or roll back from the backup")

    if fresh:
        drone.wait_back()
    if wait_for_maburd(drone):
        say("== done")
        return 0
    warn("== maburd is not running; last log lines:")
    drone.run("tail -15 /tmp/mabur.log", stdout=sys.stderr, check=False)
    if old_bin:
        warn(f"== roll back with: {script} {drone.name} {saved}")
    else:
        warn(f"== undo with: {script} --remove {drone.name}")
    return 1


def parse_args():
    parser = argparse.ArgumentParser(
        usage="%(prog)s [-y] [--config] [--fresh] [--remove] DRONE [DIR]",
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("-y", dest="yes", action="store_true", help="do not ask for confirmation")
    parser.add_argument("--remove", action="store_true", help="back to the stock image")
    parser.add_argument("--config", action="store_true", help="replace the drone's mabur.toml")
    parser.add_argument("--fresh", action="store_true", help="run the first-install steps on a mabur drone too")
    parser.add_argument("drone", metavar="DRONE")
    parser.add_argument("dir", metavar="DIR", nargs="?")
    return parser.parse_args()


def main():
    args = parse_args()
    target = args.drone if "@" in args.drone else f"root@{args.drone}"
    with tempfile.TemporaryDirectory() as tmp:
        drone = Drone(target, tmp)
        try:
            say(f"== drone {target}")
            if not drone.reachable():
                raise Abort(f"cannot ssh to {target}")
            if args.remove:
                remove(args, drone)
                return 0
            return install(args, drone, Path(tmp), sys.argv[0])
        except Abort as e:
            warn(str(e))
            return e.code
        except subprocess.CalledProcessError as e:
            return e.returncode or 1
        finally:
            drone.close()


if __name__ == "__main__":
    sys.exit(main())
