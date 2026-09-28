#!/bin/sh
# Point the conformance tool at a vehicle built on this SDK, over a real socket.
#
# A green run proves very little on its own, so every case here also breaks the vehicle
# on purpose and insists the matching check goes red. A check nobody has watched fail is
# not a check.

set -e
cd "$(dirname "$0")"

CONFORM=../conformance/ardudeck-conform
PORT=${PORT:-14790}
fails=0

run() {
  broken="$1"
  port=$2
  if [ -z "$broken" ]; then
    ./fake_vehicle --port "$port" >/dev/null 2>&1 &
  else
    ./fake_vehicle --port "$port" --break "$broken" >/dev/null 2>&1 &
  fi
  pid=$!
  sleep 1
  NO_COLOR=1 $CONFORM --udp "$port" >/tmp/conform-out.$$ 2>&1 || true
  kill $pid 2>/dev/null || true
  wait $pid 2>/dev/null || true
}

# expect <label> <break> <port> <rung name> <PASS|FAIL>
expect() {
  label="$1"; broken="$2"; port=$3; rung="$4"; want="$5"
  run "$broken" "$port"
  got=$(grep -E "  ${rung} " /tmp/conform-out.$$ | awk '{print $1}' | head -1)
  if [ "$got" = "$want" ]; then
    printf 'ok   %s (%s is %s)\n' "$label" "$rung" "$got"
  else
    printf 'FAIL %s: %s is %s, expected %s\n' "$label" "$rung" "$got" "$want"
    sed 's/^/       /' /tmp/conform-out.$$
    fails=$((fails + 1))
  fi
  rm -f /tmp/conform-out.$$
}

echo "conformance, against a vehicle on a real UDP socket"
echo

expect "a correct vehicle passes missions"   ""               $((PORT))     missions    PASS
expect "a correct vehicle passes identity"   ""               $((PORT+2))   identity    PASS
expect "a mode with no name fails"           modes            $((PORT+4))   identity    FAIL
expect "parameters with no help fail"        units            $((PORT+6))   parameters  FAIL
expect "a position at 0,0 fails"             zero-island      $((PORT+8))   position    FAIL
expect "accepting any command fails"         silent-command   $((PORT+10))  commands    FAIL
expect "declaring missions with no commands fails" no-mission-cmds $((PORT+12)) missions FAIL
expect "missions with no mode marked fails"   no-mission-mode  $((PORT+14))  identity    FAIL
expect "a calibration that ignores cancel fails" ignore-cancel  $((PORT+16))  calibration FAIL
expect "a false link failsafe fails"          rtl-on-silence   $((PORT+18))  link        FAIL

echo
if [ $fails -eq 0 ]; then
  echo "all cases behaved, including the ones that had to go red"
else
  echo "$fails case(s) did not behave"
  exit 1
fi
