"""Safety tests for the LIVE_OBSERVE network path."""

import re
import socket
import threading
import time
from pathlib import Path

import pytest

from ex_robot_state.core.dobot_feedback import FrameAssembler
from ex_robot_state.core.readonly_socket import FEEDBACK_PORTS, ReadOnlyFeedbackSocket
from ex_robot_state.tools.mock_dobot_controller import MockController

PKG = Path(__file__).resolve().parents[1] / "ex_robot_state"
LIVE_PATH_FILES = [PKG / "core" / "readonly_socket.py", PKG / "nodes" / "dobot_feedback_observer.py",
                   PKG / "core" / "dobot_feedback.py"]


@pytest.mark.parametrize("port", [29999, 30003, 22, 80, 502, 30007])
def test_command_and_other_ports_refused(port):
    with pytest.raises(PermissionError):
        ReadOnlyFeedbackSocket("127.0.0.1", port)


def test_allowlist_is_exactly_feedback_ports():
    assert FEEDBACK_PORTS == {30004, 30005, 30006}


def test_no_send_calls_in_live_path():
    """Static guard: no code in the live path may write to a socket."""
    pat = re.compile(r"\.(send|sendall|sendto|sendmsg|write|makefile)\s*\(")
    for f in LIVE_PATH_FILES:
        code = re.sub(r'"""[\s\S]*?"""', "", f.read_text())
        code = "\n".join(line.split("#")[0] for line in code.splitlines())
        assert not pat.search(code), f"{f.name} contains a socket write call"


def test_no_raw_socket_attribute_exposed():
    s = ReadOnlyFeedbackSocket("127.0.0.1", 30004)
    public = {n for n in dir(s) if not n.startswith("_")}
    assert public == {"connect", "connected", "recv", "close", "host", "port", "timeout", "half_close",
                      "bytes_received"}


def _free(port):
    with socket.socket() as s:
        try:
            s.bind(("127.0.0.1", port))
            return True
        except OSError:
            return False


@pytest.mark.parametrize("half_close", [True, False])
def test_end_to_end_with_mock_controller_sends_zero_bytes(half_close):
    port = next((p for p in (30006, 30005, 30004) if _free(p)), None)
    if port is None:
        pytest.skip("no feedback port free on localhost")
    mc = MockController(port, 8.0, None, "random", 40, 0.0)
    th = threading.Thread(target=mc.serve_forever, daemon=True)
    th.start()
    try:
        s = ReadOnlyFeedbackSocket("127.0.0.1", port, timeout=1.0, half_close=half_close)
        s.connect()
        if half_close:  # kernel-level guarantee: writing is impossible after SHUT_WR
            raw = s._ReadOnlyFeedbackSocket__sock
            with pytest.raises(OSError):
                raw.send(b"EnableRobot()")
        asm, pk, t0 = FrameAssembler(), [], time.monotonic()
        while len(pk) < 100 and time.monotonic() - t0 < 5:
            pk += list(asm.feed(s.recv()))
        s.close()
        time.sleep(0.05)
        assert len(pk) >= 100
        assert asm.resyncs > 0  # garbage was injected and recovered from
        assert mc.client_bytes == 0, "observer must never send data to the controller"
    finally:
        mc.stop()
