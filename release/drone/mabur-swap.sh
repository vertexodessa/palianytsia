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

state() {
  echo "$1" > "$UP/state"
  echo "$(date +%s) $1${2:+ $2}" >> "$UP/swap.log"
}

sha() { sha256sum "$1" 2>/dev/null | cut -d' ' -f1; }

# Waits up to STOP_WAIT seconds for pid $1 to end, then kills it hard.
reap() {
  i=0
  while kill -0 "$1" 2>/dev/null && [ "$i" -lt "$STOP_WAIT" ]; do sleep 1; i=$((i + 1)); done
  kill -9 "$1" 2>/dev/null
  wait "$1" 2>/dev/null
}

start_init() { setsid "$INIT" start </dev/null >/dev/null 2>&1; }

trial() {
  rm -f "$UP/commit"
  # The shell that started this runs under the old maburd: its answer leaves
  # before that maburd stops.
  sleep "$START_DELAY"
  state stopping
  "$INIT" stop >/dev/null 2>&1
  killall "$NAME" 2>/dev/null
  i=0
  while pidof "$NAME" >/dev/null && [ "$i" -lt "$STOP_WAIT" ]; do sleep 1; i=$((i + 1)); done
  killall -9 "$NAME" 2>/dev/null
  state trial
  "$UP/maburd" -c "$UP/mabur.toml" > "$UP/trial.log" 2>&1 &
  pid=$!
  waited=0
  while [ ! -f "$UP/commit" ]; do
    if ! kill -0 "$pid" 2>/dev/null; then
      wait "$pid"
      rollback "the new maburd exited ($?)"
      return
    fi
    if [ "$waited" -ge "$COMMIT_WAIT" ]; then
      kill "$pid" 2>/dev/null
      reap "$pid"
      rollback "no commit in ${COMMIT_WAIT} s"
      return
    fi
    sleep 1
    waited=$((waited + 1))
  done
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

commit() {
  state committing
  want=$(sha "$UP/maburd")
  rm -f "$BIN"
  cp "$UP/maburd" "$BIN" && chmod 755 "$BIN"
  cp "$UP/mabur.toml" "$CONF"
  cp "$UP/S96mabur" "$INIT" && chmod 755 "$INIT"
  sync
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

case "$1" in
  trial) trial ;;
  *) echo "usage: $0 trial" >&2; exit 2 ;;
esac
