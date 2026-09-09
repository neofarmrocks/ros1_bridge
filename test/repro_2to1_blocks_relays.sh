#!/usr/bin/env bash
# Regression check: a stuck 2->1 service forward must not stop the bridge's
# topic relays.
#
# The 2->1 forward calls roscpp's client.call(), which has no timeout, and
# upstream ran that callback in the node's default (mutually exclusive)
# callback group, so one unanswered ROS 1 call froze every other callback of
# the bridge node -- including the /clock relay every rospy node on the island
# waits on. That deadlock is self-locking: no /clock means the ROS 1 server
# never answers, so the call never returns.
#
# Method: hijack an allowlisted 2->1 service on the gantry island with a server
# that never answers, fire the call from the greenhouse island, and check that
# /clock keeps advancing on the gantry island while the call is outstanding.
#
# Status: this check REPRODUCES the deadlock, and still FAILS with the forward
# isolation applied (reentrant group + deadline). The timeout fires and is
# contained, but the island's whole 2->1 relay set stays dead afterwards, so
# there is a second blocker below rclcpp -- most likely the leaked, still
# blocked client.call() holding a roscpp lock on the publish path.
#
# Usage: test/repro_2to1_blocks_relays.sh [service_name]
# Requires the distributed simulator to be up (docker/compose/sim_distributed.yaml).

set -uo pipefail

SERVICE=${1:-/MAINTENANCE_GARDEN_CALIBRATION}
PROJECT=${COMPOSE_PROJECT_NAME:-neofarm_ros}
GANTRY=${PROJECT}-d_gantry_roscore-1
GREENHOUSE=${PROJECT}-d_greenhouse_roscore-1
SOURCE_ENV='source /application/neofarm/lib/nfar_bringup_common/common.bash'
OBSERVE_SECONDS=${OBSERVE_SECONDS:-40}

clock_secs() {  # island container -> current /clock seconds, empty on timeout
  docker exec "$1" bash -c \
    "$SOURCE_ENV; timeout 20 rostopic echo -n1 /clock/clock/secs 2>/dev/null" 2>/dev/null |
    tr -dc '0-9'
}

cleanup() {
  docker exec "$GANTRY" bash -c "pkill -f never_answers.py" >/dev/null 2>&1
  docker exec "$GREENHOUSE" bash -c "pkill -f 'rosservice call $SERVICE'" >/dev/null 2>&1
}
trap cleanup EXIT

# A bridge that CRASHES also gets its /clock back (restart: always), so the
# restart count has to be part of the assertion.
restart_count() { docker inspect -f '{{.RestartCount}}' "$1"; }

echo "== baseline: /clock on the gantry island"
RESTARTS_BEFORE=$(restart_count "$PROJECT-d_gantry_bridge-1")
BEFORE=$(clock_secs "$GANTRY")
[ -n "$BEFORE" ] || { echo "FAIL(setup): no /clock on the gantry island to begin with"; exit 2; }
echo "   secs=$BEFORE"

echo "== hijacking $SERVICE on the gantry master with a server that never answers"
docker exec "$GANTRY" bash -c "$SOURCE_ENV; cat > /tmp/never_answers.py <<'PY'
import rospy
from std_srvs.srv import Trigger

rospy.init_node('never_answers', disable_signals=True)
rospy.Service('$SERVICE', Trigger, lambda _req: rospy.sleep(1e6))
rospy.spin()
PY
nohup python3 /tmp/never_answers.py > /tmp/never_answers.log 2>&1 &
sleep 5; grep -q Traceback /tmp/never_answers.log && cat /tmp/never_answers.log; true"

echo "== firing the cross-island call from the greenhouse island (expected to hang)"
docker exec -d "$GREENHOUSE" bash -c "$SOURCE_ENV; rosservice call $SERVICE"

echo "== watching /clock on the gantry island for ${OBSERVE_SECONDS}s"
sleep "$OBSERVE_SECONDS"
AFTER=$(clock_secs "$GANTRY")
echo "   secs=${AFTER:-<none>}"

RESTARTS_AFTER=$(restart_count "$PROJECT-d_gantry_bridge-1")
if [ "$RESTARTS_AFTER" != "$RESTARTS_BEFORE" ]; then
  echo "FAIL: the gantry bridge restarted ($RESTARTS_BEFORE -> $RESTARTS_AFTER):" \
       "the forward took the process down instead of being contained"
  exit 1
fi
if [ -z "$AFTER" ]; then
  echo "FAIL: /clock stopped on the gantry island while a 2->1 forward was outstanding"
  exit 1
fi
if [ "$AFTER" -le "$BEFORE" ]; then
  echo "FAIL: /clock did not advance ($BEFORE -> $AFTER)"
  exit 1
fi
echo "PASS: /clock advanced $BEFORE -> $AFTER, bridge never restarted, with a stuck 2->1 forward outstanding"
