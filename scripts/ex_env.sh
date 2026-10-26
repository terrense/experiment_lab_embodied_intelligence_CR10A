#!/usr/bin/env bash
# source this:  source scripts/ex_env.sh
export EX_ROOT="${EX_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
export EX_WS_OUT="${EX_WS_OUT:-$HOME/ex_ws_out}"
# ROS setup scripts reference unset vars; don't let a caller's `set -u` kill them
set +u
source /opt/ros/humble/setup.bash
[ -f "$EX_WS_OUT/install/setup.bash" ] && source "$EX_WS_OUT/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"   # keep DDS off the lab network
export RCUTILS_COLORIZED_OUTPUT=1
