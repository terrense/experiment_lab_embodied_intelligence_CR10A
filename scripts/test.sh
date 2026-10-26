#!/usr/bin/env bash
# Unit tests (ROS-free core + safety tests). Results -> data/test_results/.
set -eo pipefail
source "$(dirname "$0")/ex_env.sh"
cd "$EX_ROOT"
mkdir -p data/test_results
PYTEST_TMP=$(mktemp -d) python3 -m pytest ros2_ws/src/ex_robot_state/test ros2_ws/src/ex_planning_lab/test -p no:cacheprovider -q \
  --junitxml=data/test_results/unit_tests.xml "$@"
