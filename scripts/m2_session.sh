#!/usr/bin/env bash
# MoveIt simulation session management (M2). Guarantees exactly ONE move_group.
#   bash scripts/m2_session.sh start [true|false] [launch args...]   # e.g. start false plant:=mujoco
#   bash scripts/m2_session.sh stop
#   bash scripts/m2_session.sh check                # exit 1 unless exactly one /move_group node
#
# Why: stale move_group instances from earlier runs keep serving /plan_kinematic_path and
# /check_state_validity under the same names; requests are then answered by random instances
# holding DIFFERENT planning scenes (observed 2026-10-27: validity flipping, "goal invalid").
set -eo pipefail
source "$(dirname "$0")/ex_env.sh"
cd "$EX_ROOT"
PATTERNS='^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch ex_moveit_config|^/opt/ros/humble/lib/moveit_ros_move_group/move_group|^/opt/ros/humble/lib/controller_manager/ros2_control_node|^/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher|^/opt/ros/humble/lib/rviz2/rviz2|^/usr/bin/python3 /opt/ros/humble/lib/controller_manager/spawner|^/usr/bin/python3 /home/.*/ex_mujoco_sim/mujoco_sim'

stop() {
  pkill -INT -f "$PATTERNS" 2>/dev/null || true
  for _ in $(seq 20); do pgrep -f "$PATTERNS" >/dev/null || break; sleep 0.5; done
  pkill -KILL -f "$PATTERNS" 2>/dev/null || true
  ros2 daemon stop >/dev/null 2>&1 || true   # drop cached discovery of the killed nodes
  sleep 0.5
  ! pgrep -af "$PATTERNS"
}

# --no-daemon: the ros2 daemon caches discovery and reports just-killed nodes for a while
# and a one-shot --no-daemon discovery can return before move_group is seen (observed: "found 0"),
# so give it a longer spin window and retry.
count_move_group() {
  local n=0
  for _ in 1 2 3; do
    n=$(timeout 20 ros2 node list --no-daemon --spin-time 3 2>/dev/null | grep -c '^/move_group$' || true)
    [ "$n" -ge 1 ] && break
  done
  echo "$n"
}

case "${1:-}" in
start)
  stop >/dev/null || { echo "could not stop stale processes"; exit 1; }
  nohup ros2 launch ex_moveit_config sim_moveit.launch.py "rviz:=${2:-false}" "${@:3}" > /tmp/m2_session.log 2>&1 &
  for _ in $(seq 60); do grep -q "You can start planning now" /tmp/m2_session.log && break; sleep 1; done
  grep -q "You can start planning now" /tmp/m2_session.log || { echo "move_group not ready"; tail -20 /tmp/m2_session.log; exit 1; }
  sleep 2
  n=$(count_move_group); [ "$n" = 1 ] || { echo "expected 1 move_group, found $n"; exit 1; }
  echo "MoveIt ready (1 move_group)" ;;
stop)
  stop && echo "stopped" ;;
check)
  n=$(count_move_group); echo "move_group nodes: $n"; [ "$n" = 1 ] ;;
*)
  sed -n 2,9p "$0" ;;
esac
