#!/usr/bin/env bash
# Compiles and runs test/check_stream_helpers.cpp against the vector<bool>
# stream helpers as they are written in resource/interface_factories.cpp.em.
#
# The helpers live inside an empy template, so they cannot be included directly
# and are not compiled by anything until a message package generates a factory
# from them. This extracts that one plain-C++ region of the template verbatim,
# so the check fails if the shipped text regresses -- in particular if the
# bounds-checking advance() moves back after the element-wise copy, which is an
# out-of-bounds read of attacker-controlled length on the receive path.
#
# Needs ROS 1 headers (roscpp_serialization) and a compiler with AddressSanitizer.
# Run it inside the build image, e.g.
#   docker run --rm -v "$PWD":/src:ro local/kilted_noetic_noble /src/test/check_stream_helpers.sh

set -euo pipefail

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
TEMPLATE="$HERE/../resource/interface_factories.cpp.em"
ROS1=${ROS1_INSTALL_PATH:-}
if [ -z "$ROS1" ]; then
  for candidate in /opt/ros/one /opt/ros/noetic; do
    if [ -r "$candidate/include/ros/serialization.h" ]; then
      ROS1=$candidate
      break
    fi
  done
fi
[ -n "$ROS1" ] || { echo "FAIL: no ROS 1 install with ros/serialization.h found"; exit 2; }
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

# The serialization helpers are the one region of the template with no empy
# markup in it: from the "ROS1 serialization functions" comment up to the loop
# that starts generating per-message code.
START=$(grep -n '^// ROS1 serialization functions$' "$TEMPLATE" | cut -d: -f1)
END=$(awk -v s="$START" 'NR > s && /^@\[for m in mapped_msgs\]@$/ {print NR; exit}' "$TEMPLATE")
[ -n "$START" ] && [ -n "$END" ] || { echo "FAIL: cannot find the helper region in $TEMPLATE"; exit 2; }

sed -n "$((START + 1)),$((END - 1))p" "$TEMPLATE" > "$WORK/stream_helpers.inc"
if grep -q '@' "$WORK/stream_helpers.inc"; then
  echo "FAIL: extracted region contains empy markup, the template layout changed"
  exit 2
fi
# The region opens 'namespace ros1_bridge {' and the generated file closes it
# much later, after the per-message code this check does not need.
echo '}  // namespace ros1_bridge' >> "$WORK/stream_helpers.inc"

g++ -std=c++17 -Wall -Wextra -Wno-unused-function -fsanitize=address -g \
  -I"$WORK" -I"$ROS1/include" \
  -o "$WORK/check_stream_helpers" "$HERE/check_stream_helpers.cpp" \
  -L"$ROS1/lib" -lroscpp_serialization -lrostime -lcpp_common \
  -Wl,-rpath,"$ROS1/lib"

"$WORK/check_stream_helpers"
