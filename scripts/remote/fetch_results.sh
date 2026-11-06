#!/usr/bin/env bash
# Fetch the policies trained on the remote node back to the local machine (run LOCALLY).
#
#   bash scripts/remote/fetch_results.sh [seed ...]
#
# Only the three files a checkpoint needs are pulled: the policy, the observation normaliser
# and the evaluation report. The training logs stay on the server unless REMOTE_LOGS=1.
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

SEEDS=("$@")
if [ ${#SEEDS[@]} -eq 0 ]; then
  # every seed that exists on the server
  mapfile -t SEEDS < <(ssh "${SSH_OPTS[@]}" "$REMOTE_HOST" \
    "ls '$REMOTE_DIR/data/reports/learning' 2>/dev/null | grep -o 'ppo_wall_rand_s[0-9]*' | sed 's/.*_s//' | sort -n")
fi

mkdir -p "$ROOT/data/reports/learning"
for S in "${SEEDS[@]}"; do
  DEST="$ROOT/data/reports/learning/ppo_wall_rand_s${S}"
  mkdir -p "$DEST"
  echo "== fetching seed ${S}"
  rsync -az \
    --include 'ppo.zip' --include 'vecnormalize.pkl' --include 'eval.json' --include 'monitor_*.csv' \
    --exclude '*' \
    "$REMOTE_HOST:$REMOTE_DIR/data/reports/learning/ppo_wall_rand_s${S}/" "$DEST/"
  if [ "${REMOTE_LOGS:-0}" = "1" ]; then
    rsync -az "$REMOTE_HOST:$REMOTE_DIR/logs/remote/train_s${S}.log" "$ROOT/logs/remote/" 2>/dev/null || true
  fi
done
echo "== done; the local policies are"
ls -d "$ROOT"/data/reports/learning/ppo_wall_rand_s* 2>/dev/null | sed 's#.*/##'
