"""Dobot V4 real-time feedback packet (ports 30004 / 30005 / 30006).

Layout source (both agree):
  * ``dobot_bringup_v4/include/dobot_bringup/command.h`` struct RealTimeData
    (DOBOT_6Axis_ROS2_V4 @ ec201c40, see third_party/MANIFEST.yaml)
  * "Dobot TCP_IP二次开发接口文档 V4.6.5 (2025-10-15)", 实时反馈 section:
    1440-byte packet, little-endian, TestValue = 0x0123456789ABCDEF.

Units: q_actual is in DEGREES (the official driver applies deg2Rad), tool vector
x/y/z in mm and rx/ry/rz in degrees (Euler convention NOT documented — compare
against ActualQuaternion on site before relying on rx/ry/rz).

This module only *decodes bytes*. It contains no socket code and can never
send anything.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Iterator, List, Optional, Tuple

PACKET_SIZE = 1440
TEST_VALUE = 0x0123456789ABCDEF

# name: (byte offset, struct format)  -- all little-endian
FIELDS = {
    "len": (0, "<H"),
    "digital_inputs": (8, "<Q"),
    "digital_outputs": (16, "<Q"),
    "robot_mode": (24, "<Q"),
    "timestamp_ms": (32, "<Q"),
    "run_time_ms": (40, "<Q"),
    "test_value": (48, "<Q"),
    "speed_scaling": (64, "<d"),
    "v_robot": (88, "<d"),
    "i_robot": (96, "<d"),
    "program_state": (104, "<d"),
    "q_target_deg": (192, "<6d"),
    "qd_target": (240, "<6d"),
    "q_actual_deg": (432, "<6d"),
    "qd_actual": (480, "<6d"),
    "i_actual": (528, "<6d"),
    "tool_vector_actual": (624, "<6d"),
    "tcp_speed_actual": (672, "<6d"),
    "tool_vector_target": (768, "<6d"),
    "motor_temperatures": (864, "<6d"),
    "user_coordinate": (1012, "<b"),
    "tool_coordinate": (1013, "<b"),
    "brake_status": (1025, "<b"),
    "enable_status": (1026, "<b"),
    "drag_status": (1027, "<b"),
    "running_status": (1028, "<b"),
    "error_status": (1029, "<b"),
    "jog_status": (1030, "<b"),
    "collision_state": (1038, "<b"),
    "current_command_id": (1112, "<Q"),
    "m_actual": (1120, "<6d"),
    "load_kg": (1168, "<d"),
    "user_frame": (1200, "<6d"),
    "tool_frame": (1248, "<6d"),
    "target_quaternion": (1352, "<4d"),
    "actual_quaternion": (1384, "<4d"),
    "auto_manual_mode": (1416, "<H"),
}

# Dobot TCP/IP protocol doc, RobotMode() table
ROBOT_MODES = {
    1: "INIT", 2: "BRAKE_OPEN", 3: "POWEROFF", 4: "DISABLED", 5: "ENABLE",
    6: "BACKDRIVE", 7: "RUNNING", 8: "SINGLE_MOVE", 9: "ERROR", 10: "PAUSE", 11: "COLLISION",
}


class FeedbackDecodeError(ValueError):
    pass


@dataclass(frozen=True)
class FeedbackPacket:
    len: int
    robot_mode: int
    timestamp_ms: int
    run_time_ms: int
    test_value: int
    speed_scaling: float
    q_target_deg: Tuple[float, ...]
    q_actual_deg: Tuple[float, ...]
    qd_actual: Tuple[float, ...]
    tool_vector_actual: Tuple[float, ...]
    tool_vector_target: Tuple[float, ...]
    actual_quaternion: Tuple[float, ...]
    user_coordinate: int
    tool_coordinate: int
    enable_status: int
    error_status: int
    drag_status: int
    running_status: int
    collision_state: int
    current_command_id: int
    load_kg: float
    tool_frame: Tuple[float, ...]
    motor_temperatures: Tuple[float, ...]
    auto_manual_mode: int

    @property
    def robot_mode_name(self) -> str:
        return ROBOT_MODES.get(self.robot_mode, f"UNKNOWN({self.robot_mode})")

    @property
    def q_actual_rad(self) -> List[float]:
        return [math.radians(v) for v in self.q_actual_deg]


def _get(buf: bytes, name: str, base: int = 0):
    off, fmt = FIELDS[name]
    vals = struct.unpack_from(fmt, buf, base + off)
    return vals[0] if len(vals) == 1 else tuple(vals)


def decode(buf: bytes, offset: int = 0, check_test_value: bool = True) -> FeedbackPacket:
    if len(buf) - offset < PACKET_SIZE:
        raise FeedbackDecodeError(f"need {PACKET_SIZE} bytes, have {len(buf) - offset}")
    n = _get(buf, "len", offset)
    if n != PACKET_SIZE:
        raise FeedbackDecodeError(f"MessageSize {n} != {PACKET_SIZE}")
    tv = _get(buf, "test_value", offset)
    if check_test_value and tv != TEST_VALUE:
        raise FeedbackDecodeError(f"TestValue {tv:#018x} != {TEST_VALUE:#018x}")
    kw = {name: _get(buf, name, offset) for name in FeedbackPacket.__dataclass_fields__}
    q = kw["q_actual_deg"]
    if not all(math.isfinite(v) for v in q):
        raise FeedbackDecodeError(f"non-finite q_actual {q}")
    return FeedbackPacket(**kw)


def encode(fields: dict) -> bytes:
    """Build a synthetic packet (tests / mock controller only)."""
    buf = bytearray(PACKET_SIZE)
    struct.pack_into("<H", buf, 0, PACKET_SIZE)
    struct.pack_into("<Q", buf, 48, TEST_VALUE)
    for name, value in fields.items():
        off, fmt = FIELDS[name]
        if isinstance(value, (list, tuple)):
            struct.pack_into(fmt, buf, off, *value)
        else:
            struct.pack_into(fmt, buf, off, value)
    return bytes(buf)


class FrameAssembler:
    """Turn an arbitrary TCP byte stream into packets, resynchronising on the
    (MessageSize == 1440, TestValue) signature after partial reads or garbage.

    The official driver reads fixed 1440-byte blocks; after one short read all
    later blocks are misaligned and silently discarded. This class does not
    have that failure mode.
    """

    _SIG_LEN = struct.pack("<H", PACKET_SIZE)
    _SIG_TV = struct.pack("<Q", TEST_VALUE)

    def __init__(self, check_test_value: bool = True, max_buffer: int = 8 * PACKET_SIZE) -> None:
        self.check_test_value = check_test_value
        self.max_buffer = max_buffer
        self._buf = bytearray()
        self.packets = 0
        self.resyncs = 0
        self.discarded_bytes = 0
        self.decode_errors = 0

    def _aligned(self, i: int) -> bool:
        if self._buf[i : i + 2] != self._SIG_LEN:
            return False
        if self.check_test_value and len(self._buf) >= i + 56:
            return self._buf[i + 48 : i + 56] == self._SIG_TV
        return True

    def feed(self, data: bytes) -> Iterator[FeedbackPacket]:
        self._buf.extend(data)
        while len(self._buf) >= PACKET_SIZE:
            if not self._aligned(0):
                # scan for the next plausible packet start
                j = 1
                while j + 56 <= len(self._buf) and not self._aligned(j):
                    j += 1
                skip = j if j + 56 <= len(self._buf) else max(len(self._buf) - 55, 1)
                del self._buf[:skip]
                self.discarded_bytes += skip
                self.resyncs += 1
                continue
            frame = bytes(self._buf[:PACKET_SIZE])
            del self._buf[:PACKET_SIZE]
            try:
                pkt = decode(frame, check_test_value=self.check_test_value)
            except FeedbackDecodeError:
                self.decode_errors += 1
                continue
            self.packets += 1
            yield pkt
        if len(self._buf) > self.max_buffer:  # cannot happen with sane input; bound memory anyway
            self.discarded_bytes += len(self._buf) - PACKET_SIZE
            del self._buf[: len(self._buf) - PACKET_SIZE]

    def latest(self, data: bytes) -> Optional[FeedbackPacket]:
        """Feed data and return only the newest packet (we never want a backlog)."""
        last = None
        for p in self.feed(data):
            last = p
        return last
