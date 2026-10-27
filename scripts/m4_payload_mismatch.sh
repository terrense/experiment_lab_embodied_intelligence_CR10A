#!/usr/bin/env bash
# M4 payload-mismatch study (SIMULATION ONLY): the controller's configured payload (SetPayload)
# differs from the true one. Same trajectory as the tracking study, servo 8 Hz, speed 1.0, REPS each.
#   (true, configured) in: (10,10) (10,8) (10,5) (10,0) (0,0) (0,10)
set -o pipefail
source "$(dirname "$0")/ex_env.sh"
cd "$EX_ROOT"
RUN="${RUN:-pm01}"; REPS="${REPS:-3}"
OUT="data/reports/m4/$RUN"; mkdir -p "$OUT"
TRAJ="${TRAJ:-$EX_ROOT/data/reports/m4/run03/traj_wall_sweep60.json}"
log() { echo "[pm] $*" | tee -a "$OUT/log.txt"; }
for pair in "10 10" "10 8" "10 5" "10 0" "0 0" "0 10"; do
  set -- $pair; true_kg=$1; cfg_kg=$2
  log "true payload ${true_kg} kg, configured ${cfg_kg} kg"
  bash scripts/m2_session.sh start false plant:=mujoco monitor:=false bandwidth_hz:=8 \
       payload_kg:="$true_kg" payload_configured_kg:="$cfg_kg" | tee -a "$OUT/log.txt" || { log "start failed"; continue; }
  for r in $(seq "$REPS"); do
    tag="pm_t${true_kg}_c${cfg_kg}_r$r"
    timeout 300 ros2 run ex_planning_lab execute_demo --ros-args -p scene:=wall -p query:=sweep_60 \
      -p load_trajectory:="$TRAJ" -p speed_scale:=1.0 -p tag:="$tag" -p out_dir:="$EX_ROOT/$OUT/$tag" > "$OUT/$tag.log" 2>&1
    python3 - "$OUT/$tag/execute_report.json" "$tag" <<'PY' | tee -a "$OUT/log.txt"
import json, sys
try:
    r = json.load(open(sys.argv[1]))
    if r.get("positioning_failed"):
        print(f"[pm]   {sys.argv[2]:18s} POSITIONING FAILED: static hold error {r['static_hold_error_tcp_mm']:.2f} mm, "
              f"joints {r['static_hold_error_deg']} deg"); raise SystemExit
    print(f"[pm]   {sys.argv[2]:18s} ok={r['execution_ok']} code={r['moveit_error_code']} tcp max {r['tcp_tracking_max_mm']:.2f} "
          f"rms {r['tcp_tracking_rms_mm']:.2f} final {r['final_tcp_error_mm']:.2f} mm sat={r['torque_saturation_steps_during_move']} lag={r.get('sim_max_lag_ms')}")
except FileNotFoundError:
    print(f"[pm]   {sys.argv[2]:18s} NO REPORT")
PY
  done
  bash scripts/m2_session.sh stop > /dev/null
done
log "done"
