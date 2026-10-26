#!/usr/bin/env bash
# End-to-end integration test, no hardware, headless (rviz:=false).
#   1 SIMULATION   launch + pipeline_check (+ rosbag2 record)
#   2 REPLAY       play the bag from stage 1 + pipeline_check
#   3 FAULTS       degrees / dropout / jump must be caught by the adapter
#   4 LIVE (mock)  read-only observer against mock_dobot_controller on 127.0.0.1
# Reports -> data/reports/<run>/ ; exits non-zero if any stage fails.
set -o pipefail
source "$(dirname "$0")/ex_env.sh"
cd "$EX_ROOT"
RUN="${RUN:-$(date +%Y%m%d_%H%M%S)}"
OUT="data/reports/$RUN"; mkdir -p "$OUT"
BAG="data/bags/sim_demo_$RUN"
SUMMARY="$OUT/summary.txt"; : > "$SUMMARY"
FAIL=0
PIDS=()

log() { echo "[integration] $*" | tee -a "$SUMMARY"; }
start() { setsid "$@" > "$OUT/$(date +%s%N).log" 2>&1 & PIDS+=($!); }
stop_all() {
  for p in "${PIDS[@]}"; do kill -INT -- "-$p" 2>/dev/null; done
  sleep 3
  for p in "${PIDS[@]}"; do kill -KILL -- "-$p" 2>/dev/null; done
  PIDS=()
  sleep 1
}
trap stop_all EXIT
expect() {  # expect <PASS|FAIL> <name> <cmd...>
  local want=$1 name=$2; shift 2
  "$@" > "$OUT/$name.txt" 2>&1; local rc=$?
  local got=$([ $rc -eq 0 ] && echo PASS || echo FAIL)
  if [ "$got" = "$want" ]; then log "OK    $name (pipeline_check=$got, expected $want)"
  else log "ERROR $name (pipeline_check=$got, expected $want)"; FAIL=1; fi
  grep -E '^\[(PASS|FAIL)\]' "$OUT/$name.txt" | sed 's/^/        /' | tee -a "$SUMMARY"
}
check() { ros2 run ex_robot_state pipeline_check --ros-args -p report_path:="$EX_ROOT/$OUT/$1.json" "${@:2}"; }

URDF="$(xacro "$EX_WS_OUT/install/ex_description/share/ex_description/urdf/ex_cr10.urdf.xacro")"
echo "$URDF" > "$OUT/ex_cr10_expanded.urdf"
# URDF via a params file: XML passed with -p on the command line breaks rcl's argument parser
python3 -c "import yaml,sys; yaml.safe_dump({'/**': {'ros__parameters': {'robot_description': open(sys.argv[1]).read()}}}, open(sys.argv[2],'w'))" \
  "$OUT/ex_cr10_expanded.urdf" "$OUT/robot_description.yaml"
DESC="$EX_ROOT/$OUT/robot_description.yaml"
log "run=$RUN ros=$ROS_DISTRO domain=$ROS_DOMAIN_ID"

# ---------------------------------------------------------------- 1 SIMULATION
log "stage 1: SIMULATION"
start ros2 launch ex_bringup embodied_x.launch.py mode:=simulation rviz:=false log_dir:="$EX_ROOT/$OUT"
sleep 4
start bash -c "timeout --signal=INT 22 ros2 bag record -o '$BAG' /joint_states_robot /joint_states /tf /tf_static /diagnostics /ex/tcp_pose /robot_description"
expect PASS sim_pipeline_check check sim_pipeline_check -p duration_s:=25.0 -p expect_mock_frequencies:=true -p min_rate_hz:=100.0
ros2 topic info /joint_states -v > "$OUT/sim_topic_info_joint_states.txt" 2>&1
ros2 node list > "$OUT/sim_nodes.txt" 2>&1
stop_all
ros2 bag info "$BAG" > "$OUT/bag_info.txt" 2>&1 && log "bag recorded: $BAG" || { log "ERROR bag missing"; FAIL=1; }
grep -E 'Duration|Messages|joint_states_robot' "$OUT/bag_info.txt" | sed 's/^/        /' | tee -a "$SUMMARY"

# ---------------------------------------------------------------- 2 REPLAY
log "stage 2: REPLAY"
start ros2 launch ex_bringup embodied_x.launch.py mode:=replay bag:="$EX_ROOT/$BAG" rviz:=false log_dir:="$EX_ROOT/$OUT"
sleep 2
expect PASS replay_pipeline_check check replay_pipeline_check -p duration_s:=15.0 -p expect_mock_frequencies:=true -p min_rate_hz:=100.0
stop_all

# ---------------------------------------------------------------- 3 FAULTS
for fault in degrees dropout jump; do
  log "stage 3: fault=$fault (adapter must flag it -> check FAIL expected)"
  start ros2 launch ex_bringup embodied_x.launch.py mode:=simulation rviz:=false fault:=$fault log_dir:="$EX_ROOT/$OUT"
  sleep 4
  expect FAIL fault_${fault} check fault_${fault} -p duration_s:=14.0
  ros2 topic echo --once /diagnostics diagnostic_msgs/msg/DiagnosticArray > "$OUT/fault_${fault}_diag.txt" 2>&1 || true
  stop_all
done

# ---------------------------------------------------------------- 4 LIVE (mock)
log "stage 4: LIVE_OBSERVE path against mock controller (127.0.0.1 only)"
MOCK_IP=127.0.0.1
[[ "$MOCK_IP" == 127.* ]] || { echo "refusing non-loopback IP"; exit 2; }
start python3 -m ex_robot_state.tools.mock_dobot_controller --host $MOCK_IP --port 30004 \
  --urdf "$OUT/ex_cr10_expanded.urdf" --chunking random --garbage-every 500 --report "$EX_ROOT/$OUT/mock_controller_report.json"
MOCK_PID=${PIDS[-1]}
sleep 1
start ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file "$DESC"
start ros2 run ex_robot_state joint_state_adapter --ros-args --params-file "$DESC"
start ros2 run ex_robot_state dobot_feedback_observer --ros-args -p enable_live_connection:=true \
  -p operator_ack:=I_AM_ON_SITE_AND_AUTHORIZED -p robot_ip:=$MOCK_IP -p feedback_port:=30004
start ros2 run ex_robot_state kinematics_crosscheck --ros-args --params-file "$DESC" \
  -p summary_path:="$EX_ROOT/$OUT/mock_crosscheck_summary.json"
sleep 3
LBAG="data/bags/live_mock_$RUN"
start bash -c "timeout --signal=INT 12 ros2 bag record -o '$LBAG' /joint_states_robot /ex/dobot/tool_vector_raw /ex/dobot/tcp_controller /diagnostics"
expect PASS live_mock_pipeline_check check live_mock_pipeline_check -p duration_s:=20.0 -p expect_mock_frequencies:=true -p min_rate_hz:=100.0
ros2 topic echo --once /diagnostics diagnostic_msgs/msg/DiagnosticArray > "$OUT/live_mock_diag.txt" 2>&1 || true
# stop the mock first so it writes its report, then the rest
kill -INT -- "-$MOCK_PID" 2>/dev/null; sleep 2
stop_all
if python3 -c "import json,sys; r=json.load(open('$OUT/mock_controller_report.json')); print('mock controller:', r); sys.exit(0 if r['client_bytes_received']==0 and r['packets_sent']>1000 else 1)" | tee -a "$SUMMARY"; then
  log "OK    observer sent 0 bytes to the (mock) controller"
else
  log "ERROR observer sent data or too few packets"; FAIL=1
fi
[ -f "$OUT/mock_crosscheck_summary.json" ] && log "crosscheck (mock, plumbing only): $(cat "$OUT/mock_crosscheck_summary.json" | tr -d '\n ' | cut -c1-200)"

# offline model identification on the mock recording: the mock TCP is generated
# from the CR10 URDF, so the method must pick official_cr10 (~0 mm) over cr10af (~41.9 mm)
if python3 evaluation/crosscheck_bag.py "$LBAG" --out "$OUT/live_mock_model_identification.json" > /dev/null 2>&1 &&
   python3 -c "import json,sys; r=json.load(open('$OUT/live_mock_model_identification.json')); c=r['candidates']; print('model id (mock):', {k: round(v['pos_err_mm_rms'],3) for k,v in c.items()}, 'n=', r['samples']); sys.exit(0 if r['best_position_match']=='official_cr10' and c['official_cr10']['pos_err_mm_rms']<0.01 else 1)" | tee -a "$SUMMARY"; then
  log "OK    offline model identification picks the generating model"
else
  log "ERROR offline model identification"; FAIL=1
fi

log "RESULT: $([ $FAIL -eq 0 ] && echo ALL STAGES OK || echo FAILURES) -> $OUT"
exit $FAIL
