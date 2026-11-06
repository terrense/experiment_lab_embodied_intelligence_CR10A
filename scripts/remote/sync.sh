#!/usr/bin/env bash
# Sync the repository to the remote compute node (run LOCALLY, in WSL or Git Bash).
#
#   bash scripts/remote/sync.sh
#
# Only what a simulation job needs is sent:
#   * learning/, evaluation/, scripts/remote/
#   * ros2_ws/src/ex_description  (the URDF and the meshes)
#   * data/cache/                 (the pre-expanded URDF, so the server needs no ROS)
#
# NOT sent, on purpose:
#   * ros2_ws/src/ex_dobot_motion - the ONLY motion-capable package. A compute node must not
#     have a copy of the code that can move a robot;
#   * ros2_ws/src/ex_bringup, ros2_ws/src/ex_moveit_config - not needed for training;
#   * data/reports/learning/ppo_* - the already-trained policies (the server makes its own);
#   * .git, build/, install/, log/ - nothing a training job reads.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
CFG="$HERE/remote_config.local.sh"
if [ ! -f "$CFG" ]; then
  echo "no $CFG - copy remote_config.example.sh and fill in the SSH target" >&2
  exit 1
fi
# shellcheck disable=SC1090
source "$CFG"
: "${REMOTE_HOST:?REMOTE_HOST is not set in $CFG}"
: "${REMOTE_DIR:?REMOTE_DIR is not set in $CFG}"

SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=10)
[ -n "${REMOTE_PORT:-}" ] && SSH_OPTS+=(-p "$REMOTE_PORT")

echo "== syncing to ${REMOTE_HOST}:${REMOTE_DIR}"
ssh "${SSH_OPTS[@]}" "$REMOTE_HOST" "mkdir -p '$REMOTE_DIR'"

rsync -az --delete \
  --exclude '.git' --exclude 'build' --exclude 'install' --exclude 'log' \
  --exclude '__pycache__' --exclude '*.pyc' --exclude '.pytest_cache' \
  --exclude 'ros2_ws/src/ex_dobot_motion' \
  --exclude 'ros2_ws/src/ex_bringup' \
  --exclude 'ros2_ws/src/ex_moveit_config' \
  --exclude 'data/reports/learning/ppo_*' \
  --exclude 'data/reports/learning/gen*' \
  --exclude 'data/bags' --exclude 'data/live*' --exclude 'data/sessions' \
  --exclude 'third_party' \
  "$ROOT/learning" "$ROOT/evaluation" "$ROOT/scripts" "$ROOT/data/cache" \
  "$REMOTE_HOST:$REMOTE_DIR/"

# ex_description must keep its directory name, so it is synced separately
rsync -az --delete --exclude '__pycache__' \
  "$ROOT/ros2_ws/src/ex_description" "$REMOTE_HOST:$REMOTE_DIR/ros2_ws/src/"

echo "== done; the remote tree is"
ssh "${SSH_OPTS[@]}" "$REMOTE_HOST" "ls '$REMOTE_DIR' && du -sh '$REMOTE_DIR'"
