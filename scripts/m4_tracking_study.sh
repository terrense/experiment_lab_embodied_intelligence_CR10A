#!/usr/bin/env bash
# M4 tracking study (SIMULATION ONLY): one planned trajectory, replayed on
#   mock plant                       x speed {1.0, 1.8}
#   MuJoCo plant, servo bandwidth {4, 8, 16} Hz x payload {0, 5, 10} kg x speed {1.0, 1.8}
# Paired design: the SAME trajectory everywhere, only one factor changes at a time; REPS repeats each.
# Output: data/reports/m4/<RUN>/<tag>/execute_report.json (+ csv), summary via evaluation/plot_m4_tracking.py
set -o pipefail
source "$(dirname "$0")/ex_env.sh"
cd "$EX_ROOT"
RUN="${RUN:-run01}"
OUT="data/reports/m4/$RUN"; mkdir -p "$OUT"
TRAJ="$EX_ROOT/$OUT/traj_wall_sweep60.json"
SPEEDS=(1.0 1.8)
REPS="${REPS:-3}"   # repeats per condition: outcomes near torque limits are not deterministic
log() { echo "[m4] $*" | tee -a "$OUT/log.txt"; }

run_exec() {  # run_exec <tag> <speed> [extra params...]
  local tag=$1 speed=$2; shift 2
  timeout 300 ros2 run ex_planning_lab execute_demo --ros-args -p scene:=wall -p query:=sweep_60 \
    -p load_trajectory:="$TRAJ" -p speed_scale:="$speed" -p tag:="$tag" -p out_dir:="$EX_ROOT/$OUT/$tag" "$@" \
    > "$OUT/$tag.log" 2>&1
  local rc=$?
  python3 - "$OUT/$tag/execute_report.json" "$tag" "$rc" <<'PY' | tee -a "$OUT/log.txt"
import json, sys
try:
    r = json.load(open(sys.argv[1]))
    if r.get("positioning_failed"):
        print(f"[m4]   {sys.argv[2]:22s} POSITIONING FAILED (code {r['moveit_error_code']})")
    else:
        print(f"[m4]   {sys.argv[2]:22s} ok={r['execution_ok']} code={r['moveit_error_code']} tcp rms {r['tcp_tracking_rms_mm']:.2f} "
              f"max {r['tcp_tracking_max_mm']:.2f} mm  sat={r['torque_saturation_steps_during_move']} "
              f"sim_lag_ms={r.get('sim_max_lag_ms')} load={r.get('loadavg_1m')}")
except Exception as e:
    print(f"[m4]   {sys.argv[2]:22s} NO REPORT (exit {sys.argv[3]}): {e}")
PY
}

log "plan reference trajectory on the mock plant"
bash scripts/m2_session.sh start false plant:=mock monitor:=false || exit 1
timeout 300 ros2 run ex_planning_lab execute_demo --ros-args -p scene:=wall -p query:=sweep_60 -p planner:=RRTConnect \
  -p save_trajectory:="$TRAJ" -p out_dir:="$EX_ROOT/$OUT/plan" > "$OUT/plan.log" 2>&1 || { log "planning failed"; exit 1; }
for s in "${SPEEDS[@]}"; do for r in $(seq "$REPS"); do run_exec "mock_s${s}_r$r" "$s"; done; done
bash scripts/m2_session.sh stop >/dev/null

for bw in 4 8 16; do
  for pl in 0 5 10; do
    log "MuJoCo bandwidth ${bw} Hz payload ${pl} kg"
    bash scripts/m2_session.sh start false plant:=mujoco monitor:=false bandwidth_hz:="$bw" payload_kg:="$pl" | tee -a "$OUT/log.txt" || { log "start failed"; continue; }
    for s in "${SPEEDS[@]}"; do for r in $(seq "$REPS"); do run_exec "mj_bw${bw}_pl${pl}_s${s}_r$r" "$s"; done; done
    bash scripts/m2_session.sh stop >/dev/null
  done
done
python3 evaluation/plot_m4_tracking.py "$OUT" --out assets/m4 | tee -a "$OUT/log.txt"
