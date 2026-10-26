#!/usr/bin/env bash
# usage: scripts/run_demo.sh [simulation|replay|live_observe] [extra launch args...]
set -eo pipefail
source "$(dirname "$0")/ex_env.sh"
MODE="${1:-simulation}"; shift || true
exec ros2 launch ex_bringup embodied_x.launch.py mode:="$MODE" "$@"
