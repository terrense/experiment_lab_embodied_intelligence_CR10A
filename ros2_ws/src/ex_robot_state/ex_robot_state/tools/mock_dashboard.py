"""Mock Dobot V4 Dashboard (29999) + kinematic robot, SIMULATED. For offline tests of the
motion tooling (ex_dobot_motion) - the read-only stack never talks to this port.

Implements the subset EMBODIED-X may send: RequestControl, EnableRobot, DisableRobot,
SpeedFactor, AccJ, MovJ(joint={..},a,v,cp), Stop, RobotMode, GetErrorID, GetAngle.
Anything else is answered with ErrorID -1 and recorded (tests assert it never happens).
Motion: queued MovJ targets are followed at  max_speed * v% * SpeedFactor%  (rate-limited
joint interpolation, 125 Hz); currentCommandId advances when a target is reached.
"""

from __future__ import annotations

import re
import socket
import threading
import time
from collections import deque
from typing import Deque, List, Optional, Tuple

import numpy as np

MAX_JOINT_SPEED_DEG_S = 180.0


class MockRobot:
    def __init__(self, q0_deg, host: str = "127.0.0.1", port: int = 29999) -> None:
        self.q = np.asarray(q0_deg, float)
        self.qd = np.zeros(6)
        self.mode = 4  # DISABLED
        self.tcp_mode = False
        self.speed_factor = 100
        self.queue: Deque[Tuple[int, np.ndarray, float]] = deque()
        self.next_id = 1
        self.current_cmd_id = 0
        self.commands: List[str] = []
        self.rejected: List[str] = []
        self.fail_after_moves: Optional[int] = None  # inject a controller error
        self.error = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.srv = socket.create_server((host, port))
        self.port = self.srv.getsockname()[1]
        threading.Thread(target=self._accept, daemon=True).start()
        threading.Thread(target=self._motion, daemon=True).start()

    # ------------------------------------------------------------ protocol
    def handle(self, cmd: str) -> str:
        self.commands.append(cmd)
        m = re.match(r"^\s*(\w+)\((.*)\)\s*$", cmd)
        if not m:
            self.rejected.append(cmd)
            return f"-1,{{}},{cmd};"
        name, args = m.group(1), m.group(2)
        with self._lock:
            if name == "RequestControl":
                if self.mode in (3, 4):
                    self.tcp_mode = True
                    return "0,{},RequestControl();"
                return "-1,{},RequestControl();"
            if not self.tcp_mode and name not in ("RobotMode", "GetErrorID", "GetAngle"):
                return f"-2,{{}},{name}({args});"
            if name == "EnableRobot":
                self.mode = 5
                return f"0,{{}},EnableRobot({args});"
            if name == "DisableRobot":
                self.mode, self.queue = 4, deque()
                return "0,{},DisableRobot();"
            if name == "SpeedFactor":
                self.speed_factor = int(args)
                return f"0,{{}},SpeedFactor({args});"
            if name == "AccJ":
                return f"0,{{}},AccJ({args});"
            if name == "Stop":
                self.queue.clear()
                if self.mode == 7:
                    self.mode = 5
                return "0,{},Stop();"
            if name == "RobotMode":
                return f"0,{{{self.mode}}},RobotMode();"
            if name == "GetErrorID":
                return f"0,{{[{self.error}]}},GetErrorID();" if self.error else "0,{[]},GetErrorID();"
            if name == "GetAngle":
                return f"0,{{{','.join(f'{v:.4f}' for v in self.q)}}},GetAngle();"
            if name == "MovJ":
                mm = re.match(r"joint=\{([^}]*)\}.*?,v=(\d+)", args)
                if self.mode not in (5, 7) or mm is None:
                    return f"-1,{{}},MovJ({args});"
                target = np.array([float(x) for x in mm.group(1).split(",")])
                cid = self.next_id
                self.next_id += 1
                self.queue.append((cid, target, float(mm.group(2))))
                self.mode = 7
                return f"0,{{{cid}}},MovJ({args});"
        self.rejected.append(cmd)
        return f"-1,{{}},{name}({args});"

    def _accept(self) -> None:
        self.srv.settimeout(0.5)
        while not self._stop.is_set():
            try:
                conn, _ = self.srv.accept()
            except (socket.timeout, OSError):
                if self._stop.is_set():
                    return
                continue
            threading.Thread(target=self._client, args=(conn,), daemon=True).start()

    def _client(self, conn: socket.socket) -> None:
        buf = b""
        with conn:
            while not self._stop.is_set():
                try:
                    chunk = conn.recv(1024)
                except OSError:
                    return
                if not chunk:
                    return
                buf += chunk
                while b")" in buf:
                    i = buf.index(b")") + 1
                    # a command ends at the ")" that closes its argument list (braces are inside)
                    depth = 0
                    for j, c in enumerate(buf):
                        depth += c == ord("(")
                        depth -= c == ord(")")
                        if depth == 0 and c == ord(")"):
                            i = j + 1
                            break
                    cmd, buf = buf[:i].decode().strip(), buf[i:]
                    conn.sendall(self.handle(cmd).encode())

    # -------------------------------------------------------------- motion
    def _motion(self) -> None:
        dt = 0.008
        moves = 0
        while not self._stop.is_set():
            time.sleep(dt)
            with self._lock:
                if not self.queue or self.mode != 7:
                    self.qd[:] = 0
                    continue
                cid, target, v = self.queue[0]
                vmax = MAX_JOINT_SPEED_DEG_S * v / 100.0 * self.speed_factor / 100.0
                d = target - self.q
                step = np.clip(d, -vmax * dt, vmax * dt)
                self.q = self.q + step
                self.qd = step / dt
                if np.max(np.abs(target - self.q)) < 1e-6:
                    self.queue.popleft()
                    self.current_cmd_id = cid
                    moves += 1
                    if self.fail_after_moves is not None and moves >= self.fail_after_moves:
                        self.error, self.mode = 1, 9
                        self.queue.clear()
                    elif not self.queue:
                        self.mode = 5

    def state(self):
        with self._lock:
            return self.q.copy(), self.qd.copy(), self.mode, self.current_cmd_id, self.error

    def stop(self) -> None:
        self._stop.set()
        self.srv.close()
