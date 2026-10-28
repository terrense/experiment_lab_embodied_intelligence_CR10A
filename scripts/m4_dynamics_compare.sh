#!/usr/bin/env bash
# M4 with identified dynamics (SIMULATION ONLY): the same trajectory as the tracking study on two
# plants - "placeholder" (M4 defaults) vs "identified_run02" (link masses x1.636 + Coulomb friction
# measured on the real CR10A, docs/DYNAMICS_ID.md). 0.4 kg payload (the camera), REPS each.
set -o pipefail
source "$(dirname "$0")/ex_env.sh"
cd "$EX_ROOT"
RUN="${RUN:-dyn01}"; REPS="${REPS:-3}"
OUT="data/reports/m4/$RUN"; mkdir -p "$OUT"
TRAJ="${TRAJ:-$EX_ROOT/data/reports/m4/run03/traj_wall_sweep60.json}"
log() { echo "[dyn] $*" | tee -a "$OUT/log.txt"; }
for dyn in placeholder identified_run02; do
  for bw in 8 16; do
    log "dynamics $dyn, bandwidth $bw Hz, payload 0.4 kg"
    bash scripts/m2_session.sh start false plant:=mujoco monitor:=false bandwidth_hz:="$bw" payload_kg:=0.4 \
         dynamics:="$dyn" | tee -a "$OUT/log.txt" || { log "start failed"; continue; }
    for s in 1.0 1.8; do
      for r in $(seq "$REPS"); do
        tag="${dyn}_bw${bw}_s${s}_r$r"
        timeout 300 ros2 run ex_planning_lab execute_demo --ros-args -p scene:=wall -p query:=sweep_60 \
          -p load_trajectory:="$TRAJ" -p speed_scale:="$s" -p tag:="$tag" -p out_dir:="$EX_ROOT/$OUT/$tag" > "$OUT/$tag.log" 2>&1
        python3 - "$OUT/$tag/execute_report.json" "$tag" <<'PY' | tee -a "$OUT/log.txt"
import json, sys
try:
    r = json.load(open(sys.argv[1]))
    if r.get("positioning_failed"):
        print(f"[dyn]   {sys.argv[2]:30s} POSITIONING FAILED: static hold error {r['static_hold_error_tcp_mm']:.2f} mm")
    else:
        print(f"[dyn]   {sys.argv[2]:30s} ok={r['execution_ok']} code={r['moveit_error_code']} tcp max {r['tcp_tracking_max_mm']:.2f} "
              f"rms {r['tcp_tracking_rms_mm']:.2f} final {r['final_tcp_error_mm']:.2f} mm sat={r['torque_saturation_steps_during_move']} "
              f"lag={r.get('sim_max_lag_ms')} load={r.get('loadavg_1m')}")
except FileNotFoundError:
    print(f"[dyn]   {sys.argv[2]:30s} NO REPORT")
PY
      done
    done
    grep -a -m1 'link mass' /tmp/m2_session.log | sed 's/^/[dyn]   plant: /' | tee -a "$OUT/log.txt" || true
    bash scripts/m2_session.sh stop >/dev/null
  done
done
log "done"
