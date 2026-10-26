"""A TCP client that can only *receive*, restricted to Dobot feedback ports.

Safety properties (each covered by tests):
1. Port allowlist: only 30004/30005/30006. The Dashboard/command port 29999 and
   legacy motion port 30003 are rejected before any connection attempt.
2. No send API: the wrapper exposes recv() and close() only; the raw socket
   is private.
3. Optional TCP half-close (shutdown(SHUT_WR)) right after connect: the kernel
   then refuses any later write on this socket, so even a bug elsewhere cannot
   put bytes on the wire. Whether the controller keeps streaming after
   receiving FIN is UNVERIFIED on a real CR10A -> see LIVE_ROBOT_CHECKLIST.md.
"""

from __future__ import annotations

import socket
from typing import Optional

FEEDBACK_PORTS = frozenset({30004, 30005, 30006})
FORBIDDEN_PORTS = frozenset({29999, 30003})


class ReadOnlyFeedbackSocket:
    def __init__(self, host: str, port: int, timeout: float = 2.0, half_close: bool = True) -> None:
        if port in FORBIDDEN_PORTS or port not in FEEDBACK_PORTS:
            raise PermissionError(f"port {port} is not a read-only feedback port {sorted(FEEDBACK_PORTS)}")
        self.host, self.port, self.timeout, self.half_close = host, port, timeout, half_close
        self.__sock: Optional[socket.socket] = None
        self.bytes_received = 0

    def connect(self) -> None:
        self.close()
        s = socket.create_connection((self.host, self.port), timeout=self.timeout)
        s.settimeout(self.timeout)
        if self.half_close:
            s.shutdown(socket.SHUT_WR)
        self.__sock = s

    @property
    def connected(self) -> bool:
        return self.__sock is not None

    def recv(self, n: int = 65536) -> bytes:
        """Returns b'' on orderly close by peer (and closes). Raises socket.timeout."""
        if self.__sock is None:
            raise ConnectionError("not connected")
        data = self.__sock.recv(n)
        if not data:
            self.close()
        self.bytes_received += len(data)
        return data

    def close(self) -> None:
        if self.__sock is not None:
            try:
                self.__sock.close()
            finally:
                self.__sock = None
