#!/bin/sh
# Swaps maburd for the one a GS uploaded over the maintenance lane, in two
# steps, each driven by a maburd that runs (the GS talks to the drone only
# through maburd, and maburd is what is being replaced):
#
#   trial   started detached (setsid) by the old maburd's shell. Stops the
#           init loop and the old maburd, runs the new one from RAM with the
#           new config. Flash is not touched. Without a commit within
#           COMMIT_WAIT seconds, or when the new maburd exits, the old one
#           starts again from flash.
#   commit  the GS has reached the drone through the NEW maburd and creates
#           $UP/commit over its shell. The old binary is deleted (the overlay
#           has no room for two), the new files go to flash, the init loop
#           starts on them.
#
# Neither a rollback nor the commit restarts maburd unless the drone is known
# to be disarmed: $ARM is maburd's word for the FC's arm state (armed,
# disarmed, unknown). A drone armed during the trial keeps flying the new
# maburd from RAM; a power cut boots the old one from flash.
#
# The trial proves a build in the boot that installs it, not in the next one
# (a cold start, a config that only bites after a reboot). A drone with no
# cable must come back on its own from such a build, so the commit also
#   - keeps the outgoing set (maburd gzipped, mabur.toml, S96mabur) in $LKG
#     when it is proven: no $FLAG stands, so it was installed by cable or a
#     GS reached it in a boot after its commit. An unproven outgoing build
#     leaves the older set in place;
#   - writes $FLAG with this boot's id: the new set is unproven. A GS that
#     reaches the drone's shell in a later boot deletes it (the probe in
#     crates/runtime/src/drone_update.rs);
#   - installs this script as $GUARD, an init script. At boot it waits
#     LKG_WAIT seconds for that GS; without one, and with the drone not
#     armed, it puts the set in $LKG back.
#
# Every step is written to $UP/state (one word) and $UP/swap.log, which the GS
# reads over the lane. All paths can be moved for a host test.
UP=${UP:-/tmp/mabur-up}
BIN=${BIN:-/usr/bin/maburd}
CONF=${CONF:-/etc/mabur.toml}
INIT=${INIT:-/etc/init.d/S96mabur}
COMMIT_WAIT=${COMMIT_WAIT:-120}
STOP_WAIT=${STOP_WAIT:-10}
NAME=${NAME:-maburd}   # the process name the old daemon runs under
START_DELAY=${START_DELAY:-2}
ARM=${ARM:-/tmp/mabur.arm}
LKG=${LKG:-/etc/mabur-lkg}
FLAG=${FLAG:-/etc/mabur.unproven}
GUARD=${GUARD:-/etc/init.d/S97mabur-guard}
BOOT_ID=${BOOT_ID:-/proc/sys/kernel/random/boot_id}
LKG_WAIT=${LKG_WAIT:-300}
GUARD_POLL=${GUARD_POLL:-5}

state() {
  echo "$1" > "$UP/state"
  echo "$(date +%s) $1${2:+ $2}" >> "$UP/swap.log"
}

note() { echo "$(date +%s) $1" >> "$UP/swap.log"; }

sha() { sha256sum "$1" 2>/dev/null | cut -d' ' -f1; }

boot() { cat "$BOOT_ID" 2>/dev/null; }

# Identity of the file on flash without reading 1.5 MB: a commit or an
# install replaces it, which changes the inode.
stamp() { stat -c '%i %Y %s' "$1" 2>/dev/null; }

# Waits up to STOP_WAIT seconds for pid $1 to end, then kills it hard.
reap() {
  i=0
  while kill -0 "$1" 2>/dev/null && [ "$i" -lt "$STOP_WAIT" ]; do sleep 1; i=$((i + 1)); done
  kill -9 "$1" 2>/dev/null
  wait "$1" 2>/dev/null
}

start_init() { setsid "$INIT" start </dev/null >/dev/null 2>&1; }

arm() { w=$(cat "$ARM" 2>/dev/null); echo "${w:-unknown}"; }

# Returns 0 once maburd (pid $2) reports the drone disarmed, 1 when that
# maburd exits first; says once that it waits. A maburd without the word
# never writes the file (removed before it started): the GS's check of the
# FC's heartbeat, made just before it asked for the commit, stands.
wait_disarmed() {
  held=
  while [ "$(arm)" != disarmed ]; do
    [ -f "$ARM" ] || return 0
    kill -0 "$2" 2>/dev/null || return 1
    if [ -z "$held" ]; then state "$1" "drone $(arm)"; held=1; fi
    sleep 1
  done
}

trial() {
  rm -f "$UP/commit"
  # The shell that started this runs under the old maburd: its answer leaves
  # before that maburd stops.
  sleep "$START_DELAY"
  # The old maburd's word (none from a maburd without it): the GS checked
  # the FC's heartbeat, this catches an arm in the seconds since.
  if [ "$(arm)" = armed ]; then
    state aborted_armed
    return
  fi
  state stopping
  "$INIT" stop >/dev/null 2>&1
  killall "$NAME" 2>/dev/null
  i=0
  while pidof "$NAME" >/dev/null && [ "$i" -lt "$STOP_WAIT" ]; do sleep 1; i=$((i + 1)); done
  killall -9 "$NAME" 2>/dev/null
  rm -f "$ARM"   # from here on the word is the new maburd's
  state trial
  "$UP/maburd" -c "$UP/mabur.toml" > "$UP/trial.log" 2>&1 &
  pid=$!
  waited=0
  held=
  while [ ! -f "$UP/commit" ]; do
    if ! kill -0 "$pid" 2>/dev/null; then
      wait "$pid"
      rollback "the new maburd exited ($?)"
      return
    fi
    if [ "$waited" -ge "$COMMIT_WAIT" ] && [ "$(arm)" = disarmed ]; then
      kill "$pid" 2>/dev/null
      reap "$pid"
      rollback "no commit in ${COMMIT_WAIT} s"
      return
    fi
    if [ "$waited" -ge "$COMMIT_WAIT" ] && [ -z "$held" ]; then
      state trial_held "no commit, drone $(arm): the new maburd stays until it is disarmed"
      held=1
    fi
    sleep 1
    waited=$((waited + 1))
  done
  if ! wait_disarmed commit_held "$pid"; then
    wait "$pid"
    rollback "the new maburd exited ($?)"
    return
  fi
  kill "$pid" 2>/dev/null
  reap "$pid"
  commit
}

rollback() {
  state rollback "$1"
  tail -n 20 "$UP/trial.log" >> "$UP/swap.log" 2>/dev/null
  start_init
  state restored
}

# Keeps the outgoing set in $LKG when it is proven. The older set goes
# first: the overlay may have room for one fallback only.
keep_proven() {
  if [ -f "$FLAG" ]; then
    note "the outgoing maburd is unproven: fallback stays $(cut -c1-12 "$LKG/sha" 2>/dev/null || echo none)"
    return
  fi
  rm -rf "$LKG" "$LKG.new"
  if mkdir -p "$LKG.new" &&
    gzip -c "$BIN" > "$LKG.new/maburd.gz" &&
    cp "$CONF" "$LKG.new/mabur.toml" &&
    cp "$INIT" "$LKG.new/S96mabur" &&
    sync &&
    [ "$(gunzip -c "$LKG.new/maburd.gz" | sha256sum | cut -d' ' -f1)" = "$(sha "$BIN")" ] &&
    sha "$BIN" > "$LKG.new/sha" &&
    mv "$LKG.new" "$LKG"; then
    note "fallback: $(cut -c1-12 "$LKG/sha")"
  else
    rm -rf "$LKG.new"
    note "no room for a fallback copy: this commit has none"
  fi
}

# The uploaded set onto flash; false when a copy differs.
install_new() {
  rm -f "$BIN"
  cp "$UP/maburd" "$BIN" && chmod 755 "$BIN"
  cp "$UP/mabur.toml" "$CONF"
  cp "$UP/S96mabur" "$INIT" && chmod 755 "$INIT"
  sync
  [ "$(sha "$BIN")" = "$want" ] && [ "$(sha "$CONF")" = "$(sha "$UP/mabur.toml")" ]
}

commit() {
  state committing
  want=$(sha "$UP/maburd")
  keep_proven
  # Before the old binary goes: a flash copy that fails below is recovered
  # by the guard on the next boot too.
  boot > "$FLAG"
  # The temporary name must not match the init system's S??* glob.
  g=${GUARD%/*}/.mabur-guard.new
  cp "$0" "$g" && chmod 755 "$g" && mv "$g" "$GUARD"
  if ! install_new && [ -d "$LKG" ]; then
    # The new set comes first: without room for both, the fallback goes.
    note "the new files do not fit beside the fallback: the fallback goes"
    rm -rf "$LKG"
    install_new
  fi
  if [ "$(sha "$BIN")" != "$want" ] || [ "$(sha "$CONF")" != "$(sha "$UP/mabur.toml")" ]; then
    # The old binary is gone already: keep the drone reachable on the copy in
    # RAM until someone looks (it does not survive a reboot).
    state commit_failed "flash copy differs (overlay full?)"
    "$UP/maburd" -c "$UP/mabur.toml" > "$UP/trial.log" 2>&1 &
    return
  fi
  start_init
  state committed
}

# At boot, as $GUARD: the set committed in an earlier boot has LKG_WAIT
# seconds to be reached by a GS, else the proven set comes back. It stands
# aside for an install or update that replaces maburd meanwhile (a GS whose
# probe does not know $FLAG), and holds while a trial runs or the drone is
# armed.
guard() {
  [ -f "$FLAG" ] && [ -f "$LKG/maburd.gz" ] || return 0
  [ "$(cat "$FLAG")" = "$(boot)" ] && return 0
  mkdir -p "$UP"
  at=$(stamp "$BIN")
  note "guard: maburd $(sha "$BIN" | cut -c1-12) unproven; fallback $(cut -c1-12 "$LKG/sha") in ${LKG_WAIT} s without a GS"
  waited=0
  while :; do
    if [ ! -f "$FLAG" ]; then note "guard: a GS reached the drone, maburd proven"; return 0; fi
    if [ "$(stamp "$BIN")" != "$at" ]; then note "guard: maburd replaced meanwhile, guard ends"; return 0; fi
    if [ "$waited" -ge "$LKG_WAIT" ] && [ "$(arm)" != armed ]; then
      case "$(cat "$UP/state" 2>/dev/null)" in
        stopping|trial|trial_held|commit_held|committing) ;;
        *) break ;;
      esac
    fi
    sleep "$GUARD_POLL"
    waited=$((waited + GUARD_POLL))
  done
  restore_lkg
}

restore_lkg() {
  state guard_restoring "no GS reached maburd $(sha "$BIN" | cut -c1-12) since a boot: back to $(cut -c1-12 "$LKG/sha")"
  "$INIT" stop >/dev/null 2>&1
  killall "$NAME" 2>/dev/null
  i=0
  while pidof "$NAME" >/dev/null && [ "$i" -lt "$STOP_WAIT" ]; do sleep 1; i=$((i + 1)); done
  killall -9 "$NAME" 2>/dev/null
  rm -f "$BIN"
  gunzip -c "$LKG/maburd.gz" > "$BIN" && chmod 755 "$BIN"
  cp "$LKG/mabur.toml" "$CONF"
  cp "$LKG/S96mabur" "$INIT" && chmod 755 "$INIT"
  sync
  if [ "$(sha "$BIN")" = "$(cat "$LKG/sha")" ]; then
    rm -f "$FLAG"
    start_init
    state guard_restored
  else
    # The next boot tries again; whatever is on flash runs meanwhile.
    start_init
    state guard_restore_failed "flash copy differs (overlay full?)"
  fi
}

case "$1" in
  trial) trial ;;
  # The init system's call at boot: must return at once.
  start) setsid "$0" guard </dev/null >/dev/null 2>&1 & ;;
  guard) guard ;;
  stop|restart) ;;
  *) echo "usage: $0 trial|start|guard" >&2; exit 2 ;;
esac
