"""Whitelisted Dobot V4 Dashboard client (TCP 29999) - the ONLY code in EMBODIED-X that can
command the real robot. Kept in its own package, separate from the read-only stack.

Protocol (Dobot TCP_IP V4.6.5 doc, verified): ASCII command `Name(args)`, reply
`ErrorID,{values},Name(args);`  ErrorID 0 = accepted.

Safety properties (unit tested):
  * command whitelist - anything else raises before a byte is sent
  * MovJ only in joint form `MovJ(joint={...},a=..,v=..,cp=..)` and only with a, v <= caps
  * SpeedFactor / AccJ capped
  * every exchange is logged (time, command, reply)
"""

from __future__ import annotations

import re
import socket
import time
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

ALLOWED = {"RequestControl", "EnableRobot", "DisableRobot", "SpeedFactor", "AccJ", "MovJ", "Stop",
           "RobotMode", "GetErrorID", "GetAngle"}
MAX_SPEED_PERCENT = 30
MAX_ACC_PERCENT = 30

_REPLY = re.compile(r"^\s*(-?\d+)\s*,\s*\{(.*?)\}\s*,\s*(\w+)\(.*\)\s*;?\s*$", re.S)


class CommandRejected(PermissionError):
    pass


class DashboardError(RuntimeError):
    pass


@dataclass
class Reply:
    error_id: int
    values: List[str]
    name: str
    raw: str


def parse_reply(raw: str) -> Reply:
    m = _REPLY.match(raw.strip())
    if not m:
        raise DashboardError(f"unparseable reply {raw!r}")
    vals = [v.strip() for v in m.group(2).split(",")] if m.group(2).strip() else []
    return Reply(int(m.group(1)), vals, m.group(3), raw.strip())


def fmt(x: float) -> str:
    return f"{x:.4f}".rstrip("0").rstrip(".")


def movj_joint(q_deg: Sequence[float], v: int, a: int, cp: int = 0) -> str:
    if len(q_deg) != 6:
        raise ValueError("6 joints required")
    return f"MovJ(joint={{{','.join(fmt(x) for x in q_deg)}}},a={int(a)},v={int(v)},cp={int(cp)})"


def check_command(cmd: str) -> None:
    """Raise CommandRejected unless `cmd` is whitelisted and within the caps."""
    m = re.match(r"^\s*(\w+)\((.*)\)\s*$", cmd)
    if not m:
        raise CommandRejected(f"malformed command {cmd!r}")
    name, args = m.group(1), m.group(2)
    if name not in ALLOWED:
        raise CommandRejected(f"{name} is not whitelisted")
    if name == "MovJ":
        if not re.match(r"^joint=\{[-0-9.,eE ]+\}(,a=\d+)?(,v=\d+)?(,cp=\d+)?$", args.replace(" ", "")):
            raise CommandRejected(f"only joint-form MovJ with a/v/cp is allowed: {cmd!r}")
        for key, cap in (("v", MAX_SPEED_PERCENT), ("a", MAX_ACC_PERCENT)):
            mm = re.search(rf",{key}=(\d+)", args)
            if mm is None or int(mm.group(1)) > cap or int(mm.group(1)) < 1:
                raise CommandRejected(f"MovJ {key} must be given and in [1,{cap}]: {cmd!r}")
    if name == "SpeedFactor" and not (args.isdigit() and 1 <= int(args) <= MAX_SPEED_PERCENT):
        raise CommandRejected(f"SpeedFactor must be in [1,{MAX_SPEED_PERCENT}]")
    if name == "AccJ" and not (args.isdigit() and 1 <= int(args) <= MAX_ACC_PERCENT):
        raise CommandRejected(f"AccJ must be in [1,{MAX_ACC_PERCENT}]")


@dataclass
class DashboardClient:
    host: str
    port: int = 29999
    timeout: float = 5.0
    log: List[Tuple[float, str, str]] = field(default_factory=list)
    _sock: Optional[socket.socket] = None

    def connect(self) -> None:
        self._sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self._sock.settimeout(self.timeout)

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

    def send(self, cmd: str) -> Reply:
        check_command(cmd)  # before anything touches the socket
        if self._sock is None:
            raise DashboardError("not connected")
        self._sock.sendall(cmd.encode("ascii"))
        buf = b""
        t0 = time.monotonic()
        while not buf.rstrip().endswith(b";"):
            if time.monotonic() - t0 > self.timeout:
                raise DashboardError(f"timeout waiting for reply to {cmd!r} (got {buf!r})")
            chunk = self._sock.recv(1024)
            if not chunk:
                raise DashboardError("dashboard connection closed")
            buf += chunk
        raw = buf.decode("ascii", "replace")
        self.log.append((time.time(), cmd, raw.strip()))
        return parse_reply(raw)

    def must(self, cmd: str) -> Reply:
        r = self.send(cmd)
        if r.error_id != 0:
            raise DashboardError(f"{cmd} -> ErrorID {r.error_id} ({r.raw})")
        return r
