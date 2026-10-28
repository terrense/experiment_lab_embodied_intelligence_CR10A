#!/usr/bin/env bash
# Hands-on 4 (SIMULATION ONLY): MoveIt plans around the wall, RViz shows plan + executed path,
# the MuJoCo plant (physics) executes it in its own viewer window.
#   bash hands_on/4_moveit_rviz_mujoco.sh [placeholder|identified_run02] [rounds]
set -o pipefail
cd "$(dirname "$0")/.." && source scripts/ex_env.sh
DYN="${1:-placeholder}"; ROUNDS="${2:-3}"
OUT="data/hands_on/$(date +%Y%m%d_%H%M%S)"; mkdir -p "$OUT"
echo "== starting MoveIt + RViz + MuJoCo (dynamics: $DYN) - two windows will open, ~20 s"
bash scripts/m2_session.sh start true plant:=mujoco mujoco_viewer:=true monitor:=false dynamics:="$DYN" || { echo "start failed, log: /tmp/m2_session.log"; exit 1; }
trap 'echo "== stopping"; bash scripts/m2_session.sh stop >/dev/null; echo "stopped"' EXIT
for r in $(seq "$ROUNDS"); do
  for q in sweep_60 sweep_45; do
    echo "== round $r: plan + execute wall/$q (orange = planned TCP path, cyan = executed)"
    timeout 300 ros2 run ex_planning_lab execute_demo --ros-args -p scene:=wall -p query:="$q" -p planner:=RRTConnect \
      -p tag:="hands_on_${q}_r$r" -p out_dir:="$PWD/$OUT/${q}_r$r" > "$OUT/${q}_r$r.log" 2>&1
    python3 - "$OUT/${q}_r$r/execute_report.json" <<'PY'
import json, sys
try:
    r = json.load(open(sys.argv[1]))
    if r.get("positioning_failed"):
        print("   could not reach the start pose"); raise SystemExit
    print(f"   executed ok={r['execution_ok']}  TCP tracking error: max {r['tcp_tracking_max_mm']:.1f} mm, rms {r['tcp_tracking_rms_mm']:.1f} mm")
except FileNotFoundError:
    print("   no report (see the .log next to it)")
PY
  done
done
read -r -p "== done. Look around in RViz / MuJoCo, then press Enter to close everything " _
