#!/usr/bin/env bash
# Record the robot-state pipeline. usage: scripts/record_bag.sh <name> [seconds]
# Raw source (/joint_states_robot) is what REPLAY plays back; the rest is for analysis.
set -eo pipefail
source "$(dirname "$0")/ex_env.sh"
NAME="${1:?bag name}"; DUR="${2:-0}"
OUT="$EX_ROOT/data/bags/$NAME"
[ -e "$OUT" ] && { echo "$OUT exists"; exit 1; }
TOPICS=(/joint_states_robot /joint_states /tf /tf_static /diagnostics /ex/tcp_pose
        /ex/dobot/tcp_controller /ex/dobot/tool_vector_raw /robot_description)
if [ "$DUR" != "0" ]; then
  timeout --signal=INT "$DUR" ros2 bag record -o "$OUT" "${TOPICS[@]}" || true
else
  ros2 bag record -o "$OUT" "${TOPICS[@]}"
fi
ros2 bag info "$OUT"
