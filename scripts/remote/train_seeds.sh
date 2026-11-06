#!/usr/bin/env bash
# Train PPO seeds on the remote compute node (run ON THE SERVER).
#
#   bash scripts/remote/train_seeds.sh
#
# The MuJoCo plant is CPU-bound and the policy is a small MLP, so the GPU is NOT what makes
# this faster - the server's CPU cores are. The script therefore runs several seeds in
# parallel, each with its own 12 environment processes, and prints the recommendation it
# computed from nproc.
#
# Each job writes to data/reports/learning/ppo_wall_rand_s<SEED>, exactly like a local run, so
# the results are interchangeable with the locally trained ones.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
CFG="$HERE/remote_config.local.sh"
if [ -f "$CFG" ]; then
  # shellcheck disable=SC1090
  source "$CFG"
fi

PY="${REMOTE_PY:-$ROOT/../ex_rlenv_remote/bin/python}"
[ -x "$PY" ] || PY="python3"
TIMESTEPS="${REMOTE_TIMESTEPS:-5000000}"
N_ENVS="${REMOTE_N_ENVS:-12}"
TASK="${REMOTE_TASK:-wall_rand}"
SEEDS="${REMOTE_SEEDS:-9 10 11 12 13 14 15 16 17 18 19 20}"
JOBS="${REMOTE_JOBS:-$(( $(nproc) / (N_ENVS + 1) ))}"
[ "$JOBS" -lt 1 ] && JOBS=1

mkdir -p "$ROOT/logs/remote"
cd "$ROOT/learning"
echo "== training seeds [$SEEDS] with $JOBS parallel job(s); $PY; ${TIMESTEPS} timesteps"

run_one() {
  local S="$1"
  local OUT="$ROOT/data/reports/learning/ppo_wall_rand_s${S}"
  if [ -f "$OUT/ppo.zip" ] && [ -f "$OUT/eval.json" ]; then
    echo "[seed ${S}] already trained - skipping"
    return 0
  fi
  echo "[seed ${S}] start $(date +%H:%M:%S)"
  "$PY" -u -m ex_learning.train_ppo \
    --task "$TASK" --timesteps "$TIMESTEPS" --n-envs "$N_ENVS" --seed "$S" \
    --eval-episodes 50 --out "$OUT" > "$ROOT/logs/remote/train_s${S}.log" 2>&1
  echo "[seed ${S}] done  $(date +%H:%M:%S)  $(tail -1 "$ROOT/logs/remote/train_s${S}.log")"
}

export -f run_one
export PY ROOT TIMESTEPS N_ENVS TASK
export REMOTE_SEEDS_UNUSED=1

echo "$SEEDS" | tr ' ' '\n' | grep -v '^$' | xargs -P "$JOBS" -I{} bash -c 'run_one "$@"' _ {}

echo "== all requested seeds finished $(date +%H:%M:%S)"
for S in $SEEDS; do
  F="$ROOT/data/reports/learning/ppo_wall_rand_s${S}/eval.json"
  [ -f "$F" ] && echo "  seed ${S}: $(grep -o '"success_rate":[0-9.]*' "$F" | head -1) $(grep -o '"collision_rate":[0-9.]*' "$F" | head -1)"
done
