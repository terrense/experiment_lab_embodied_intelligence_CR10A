#!/usr/bin/env bash
# Guided, READ-ONLY on-site session with the real CR10A (docs/LIVE_ROBOT_CHECKLIST.md).
# Never touches port 29999. Every step only reads.
#
#   bash scripts/live_session.sh net                 # 1. reachability + which feedback ports stream
#   bash scripts/live_session.sh peek [port]         # 2. decode ONE packet, print joints/TCP/mode (no ROS)
#   bash scripts/live_session.sh observe [port]      # 3. RViz live sync (Ctrl-C to stop)
#   bash scripts/live_session.sh record NAME [sec]   # 4. record (run while 'observe' is running)
#   bash scripts/live_session.sh analyze NAME        # 5. offline model identification
# env: ROBOT_IP (default 192.168.5.1)
set -eo pipefail
source "$(dirname "$0")/ex_env.sh"
cd "$EX_ROOT"
IP="${ROBOT_IP:-192.168.5.1}"
cmd="${1:-help}"; shift || true

case "$cmd" in
net)
  echo "== route to $IP"; ip route get "$IP" || true
  echo "== ping (ICMP may be blocked; not fatal)"; ping -c 2 -W 1 "$IP" || true
  for p in 30004 30005 30006; do   # feedback ports only - 29999 is deliberately NOT probed
    n=$(timeout 3 bash -c "cat < /dev/tcp/$IP/$p | head -c 2880 | wc -c" 2>/dev/null || true)
    echo "port $p -> ${n:-0} bytes in 3 s   (>=1440 means streaming)"
  done ;;
peek)
  PORT="${1:-30004}"
  python3 - "$IP" "$PORT" <<'PY'
import math, sys, time
sys.path.insert(0, "ros2_ws/src/ex_robot_state")
from ex_robot_state.core.readonly_socket import ReadOnlyFeedbackSocket
from ex_robot_state.core.dobot_feedback import FrameAssembler
ip, port = sys.argv[1], int(sys.argv[2])
s = ReadOnlyFeedbackSocket(ip, port, timeout=3.0, half_close=False)   # half-close tested later
s.connect(); a = FrameAssembler(); p = None; t0 = time.time()
while p is None and time.time() - t0 < 5:
    p = a.latest(s.recv())
s.close()
if p is None: sys.exit("no valid packet in 5 s")
print(f"robot_mode     : {p.robot_mode} {p.robot_mode_name}")
print(f"QActual (deg)  : " + "  ".join(f"J{i+1}={v:+.3f}" for i, v in enumerate(p.q_actual_deg)))
print(f"ToolVectorAct. : xyz=({p.tool_vector_actual[0]:.2f},{p.tool_vector_actual[1]:.2f},{p.tool_vector_actual[2]:.2f}) mm  "
      f"rxyz=({p.tool_vector_actual[3]:.2f},{p.tool_vector_actual[4]:.2f},{p.tool_vector_actual[5]:.2f}) deg")
print(f"ActualQuat     : {tuple(round(v, 5) for v in p.actual_quaternion)}")
print(f"tool/user id   : {p.tool_coordinate}/{p.user_coordinate}   enable={p.enable_status} error={p.error_status} load={p.load_kg:.2f}kg")
print(f"ctrl timestamp : {p.timestamp_ms} (host-ctrl offset {time.time()*1000 - p.timestamp_ms:.0f} ms)")
print(f"bytes sent by us: 0 (socket has no send API)   resyncs={a.resyncs}")
PY
  ;;
observe)
  PORT="${1:-30004}"
  RATE=$([ "$PORT" = 30005 ] && echo 5.0 || ([ "$PORT" = 30006 ] && echo 1.0 || echo 125.0))
  CFG=/tmp/ex_live_observe_onsite.yaml
  python3 - "$CFG" "$RATE" "${HALF_CLOSE:-true}" <<'PY'
import sys, yaml
src = yaml.safe_load(open("ros2_ws/src/ex_bringup/config/modes/live_observe.yaml"))
src["observer"]["enable_live_connection"] = True          # session-only; committed file stays false
src["observer"]["half_close_write"] = sys.argv[3] == "true"
src["adapter"]["expected_rate_hz"] = float(sys.argv[2])
if float(sys.argv[2]) < 10: src["adapter"]["stale_after_s"], src["adapter"]["lost_after_s"] = 0.6, 2.0
yaml.safe_dump(src, open(sys.argv[1], "w"))
PY
  echo "LIVE_OBSERVE $IP:$PORT (read-only). Ctrl-C to stop."
  exec ros2 launch ex_bringup embodied_x.launch.py mode:=live_observe robot_ip:="$IP" feedback_port:="$PORT" \
       live_ack:=I_AM_ON_SITE_AND_AUTHORIZED live_config:="$CFG" ;;
record)
  exec bash scripts/record_bag.sh "${1:?name}" "${2:-120}" ;;
analyze)
  python3 evaluation/crosscheck_bag.py "data/bags/${1:?name}" --out "data/reports/model_id_${1}.json" ;;
*)
  sed -n '2,12p' "$0" ;;
esac
