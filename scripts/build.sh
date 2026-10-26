#!/usr/bin/env bash
# Build the EMBODIED-X ROS 2 workspace. Sources stay in the repo (may be on /mnt/c);
# build/install/log go to a Linux-native dir ($EX_WS_OUT) for speed and correct permissions.
set -eo pipefail
source "$(dirname "$0")/ex_env.sh"
cd "$EX_ROOT"
colcon --log-base "$EX_WS_OUT/log" build \
  --base-paths ros2_ws/src \
  --build-base "$EX_WS_OUT/build" --install-base "$EX_WS_OUT/install" \
  --event-handlers console_cohesion+ "$@"
echo "built -> $EX_WS_OUT/install   (source scripts/ex_env.sh)"
