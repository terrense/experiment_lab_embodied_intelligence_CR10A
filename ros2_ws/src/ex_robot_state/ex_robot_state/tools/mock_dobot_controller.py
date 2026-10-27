"""Mock Dobot V4 feedback server (SIMULATED data, no ROS needed).

Streams 1440-byte packets like port 30004 so the LIVE_OBSERVE path can be
exercised end-to-end without hardware. It also records every byte a client
sends — the read-only observer must send exactly zero.

  python3 -m ex_robot_state.tools.mock_dobot_controller --port 30004 --urdf /tmp/cr10.urdf
  options: --period-ms 8 --chunking random --garbage-every 500 --stop-after 10
"""

from __future__ import annotations

import argparse
import json
import math
import random
import signal
import socket
import threading
import time
from typing import List, Optional

import numpy as np

from ..core.dobot_feedback import encode
from ..core.mock_motion import MockMotion, MockMotionConfig
from ..core.se3 import matrix_to_quat, matrix_to_rpy
from ..core.urdf_model import JointSpec, UrdfModel


def _default_joints() -> List[JointSpec]:
    lim = [6.27, 6.27, 2.861, 6.27, 6.27, 6.27]
    return [JointSpec(f"joint{i + 1}", "revolute", "p", "c", np.eye(4), np.array([0, 0, 1.0]), -l, l)
            for i, l in enumerate(lim)]


class MockController:
    def __init__(self, port: int, period_ms: float, model: Optional[UrdfModel], chunking: str,
                 garbage_every: int, stop_after: float, host: str = "127.0.0.1",
                 base_link: str = "base_link", flange_link: str = "Link6") -> None:
        self.model, self.base, self.flange = model, base_link, flange_link
        joints = model.actuated_joints(flange_link, base_link) if model else _default_joints()
        self.motion = MockMotion(joints, MockMotionConfig(home=[0, 0, math.radians(90), 0, math.radians(-90), 0]))
        self.period = period_ms / 1000.0
        self.chunking, self.garbage_every, self.stop_after = chunking, garbage_every, stop_after
        self.srv = socket.create_server((host, port), reuse_port=False)
        self.port = self.srv.getsockname()[1]
        self.client_bytes = 0
        self.clients = 0
        self.packets_sent = 0
        self.robot = None  # optional MockRobot (tools/mock_dashboard.py): stream ITS state instead
        self._stop = threading.Event()

    def packet(self, t: float) -> bytes:
        if self.robot is not None:
            qd_deg, qdd_deg, mode, cid, err = self.robot.state()
            q = [math.radians(v) for v in qd_deg]
            fields = {
                "robot_mode": mode, "timestamp_ms": int(time.time() * 1000), "run_time_ms": int(t * 1000),
                "q_actual_deg": list(qd_deg), "q_target_deg": list(qd_deg), "qd_actual": list(qdd_deg),
                "speed_scaling": self.robot.speed_factor / 100.0, "enable_status": int(mode in (5, 7)),
                "error_status": err, "current_command_id": cid, "tool_coordinate": 0, "user_coordinate": 0,
                # synthetic currents/torques so the recording path is exercised (NOT physics)
                "i_actual": [0.01 * v for v in qdd_deg], "m_actual": [0.1 * v for v in qdd_deg],
            }
        else:
            q = self.motion.positions(t)
            fields = {
                "robot_mode": 5, "timestamp_ms": int(time.time() * 1000), "run_time_ms": int(t * 1000),
                "q_actual_deg": [math.degrees(v) for v in q], "q_target_deg": [math.degrees(v) for v in q],
                "qd_actual": [math.degrees(v) for v in self.motion.velocities(t)],
                "speed_scaling": 1.0, "enable_status": 1, "tool_coordinate": 0, "user_coordinate": 0,
            }
        if self.model is not None:
            T = self.model.fk(dict(zip([f"joint{i}" for i in range(1, 7)], q)), self.flange, self.base)
            r, p, y = (math.degrees(a) for a in matrix_to_rpy(T[:3, :3]))
            fields["tool_vector_actual"] = [*(T[:3, 3] * 1000.0), r, p, y]
            qx, qy, qz, qw = matrix_to_quat(T[:3, :3])
            fields["actual_quaternion"] = [qw, qx, qy, qz]
        return encode(fields)

    def _serve_client(self, conn: socket.socket) -> None:
        conn.settimeout(0.0)
        t0 = time.monotonic()
        try:
            while not self._stop.is_set():
                t = time.monotonic() - t0
                if self.stop_after and t > self.stop_after:
                    break
                try:  # non-blocking drain of anything the client sends
                    while True:
                        b = conn.recv(4096)
                        if not b:
                            break  # client half-closed (FIN) - expected from the read-only observer
                        self.client_bytes += len(b)
                except (BlockingIOError, InterruptedError):
                    pass
                data = self.packet(t)
                if self.garbage_every and self.packets_sent and self.packets_sent % self.garbage_every == 0:
                    data = bytes(random.randrange(256) for _ in range(37)) + data
                conn.setblocking(True)
                if self.chunking == "random":
                    i = 0
                    while i < len(data):
                        n = random.randint(1, 700)
                        conn.sendall(data[i:i + n])
                        i += n
                else:
                    conn.sendall(data)
                conn.setblocking(False)
                self.packets_sent += 1
                time.sleep(self.period)
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            conn.close()

    def serve_forever(self) -> None:
        self.srv.settimeout(0.5)
        while not self._stop.is_set():
            try:
                conn, _ = self.srv.accept()
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():  # socket closed by stop()
                    return
                raise
            self.clients += 1
            threading.Thread(target=self._serve_client, args=(conn,), daemon=True).start()

    def stop(self) -> None:
        self._stop.set()
        self.srv.close()

    def report(self) -> dict:
        return {"clients": self.clients, "packets_sent": self.packets_sent, "client_bytes_received": self.client_bytes}


def main(argv: Optional[List[str]] = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=30004)
    ap.add_argument("--period-ms", type=float, default=8.0)
    ap.add_argument("--urdf", default="", help="expanded URDF; enables FK-consistent ToolVectorActual")
    ap.add_argument("--chunking", choices=["whole", "random"], default="whole")
    ap.add_argument("--garbage-every", type=int, default=0)
    ap.add_argument("--stop-after", type=float, default=0.0, help="close each client after N s (disconnect test)")
    ap.add_argument("--report", default="", help="write JSON report here on exit")
    a = ap.parse_args(argv)
    model = UrdfModel.from_file(a.urdf) if a.urdf else None
    mc = MockController(a.port, a.period_ms, model, a.chunking, a.garbage_every, a.stop_after, a.host)
    print(f"[mock_dobot_controller] SIMULATED feedback on {a.host}:{mc.port}", flush=True)

    def _graceful(signum, frame):  # background jobs may start with SIGINT ignored
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, _graceful)
    signal.signal(signal.SIGTERM, _graceful)
    try:
        mc.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        mc.stop()
        rep = mc.report()
        print(f"[mock_dobot_controller] report {json.dumps(rep)}", flush=True)
        if a.report:
            with open(a.report, "w") as f:
                json.dump(rep, f)


if __name__ == "__main__":
    main()
