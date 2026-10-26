import math
import random
import re
import struct

import pytest

from ex_robot_state.core.dobot_feedback import (FIELDS, PACKET_SIZE, TEST_VALUE, FeedbackDecodeError,
                                                FrameAssembler, decode, encode)

# our field name -> member name in the official RealTimeData struct
OFFICIAL_NAME = {
    "len": "len", "digital_inputs": "digital_input_bits", "digital_outputs": "digital_outputs",
    "robot_mode": "robot_mode", "timestamp_ms": "controller_timer", "run_time_ms": "run_time",
    "test_value": "test_value", "speed_scaling": "speed_scaling", "v_robot": "v_robot", "i_robot": "i_robot",
    "program_state": "program_state", "q_target_deg": "q_target", "qd_target": "qd_target",
    "q_actual_deg": "q_actual", "qd_actual": "qd_actual", "i_actual": "i_actual",
    "tool_vector_actual": "tool_vector_actual", "tcp_speed_actual": "TCP_speed_actual",
    "tool_vector_target": "tool_vector_target", "motor_temperatures": "motor_temperatures",
    "user_coordinate": "userCoordinate", "tool_coordinate": "toolCoordinate", "brake_status": "BrakeStatus",
    "enable_status": "EnableStatus", "drag_status": "DragStatus", "running_status": "RunningStatus",
    "error_status": "ErrorStatus", "jog_status": "JogStatus", "collision_state": "CollisionStates",
    "current_command_id": "currentCommandId", "m_actual": "m_actual", "load_kg": "load",
    "user_frame": "user", "tool_frame": "tool", "target_quaternion": "TargetQuaternion",
    "actual_quaternion": "ActualQuaternion", "auto_manual_mode": "AutoManualMode",
}


def test_offsets_match_official_header(official_header):
    """Every offset we decode must equal the byte position documented in the
    official driver's struct comments (e.g. `double q_actual[6]; // 0432 ~ 0479`)."""
    found = {}
    for m in re.finditer(r"^\s*\w+\s+(\w+)(?:\[\d+\])?;\s*//\s*(\d+)", official_header, re.M):
        found[m.group(1)] = int(m.group(2))
    assert len(OFFICIAL_NAME) == len(FIELDS)
    for ours, theirs in OFFICIAL_NAME.items():
        assert theirs in found, theirs
        assert FIELDS[ours][0] == found[theirs], f"{ours}: {FIELDS[ours][0]} != official {found[theirs]}"
    assert "#pragma pack(pop)" in official_header
    assert re.search(r"Reserve3\[22\];\s*//\s*1418 ~ 1439", official_header)


def test_field_sizes_fit_packet():
    for name, (off, fmt) in FIELDS.items():
        assert off + struct.calcsize(fmt) <= PACKET_SIZE, name


def test_roundtrip():
    q = [10.0, -20.5, 90.0, 0.25, -90.0, 179.0]
    buf = encode({"q_actual_deg": q, "robot_mode": 5, "timestamp_ms": 1_700_000_000_123,
                  "tool_vector_actual": [100, 200, 300, 180, 0, 90], "tool_coordinate": 2})
    assert len(buf) == PACKET_SIZE
    p = decode(buf)
    assert p.q_actual_deg == tuple(q)
    assert p.q_actual_rad == pytest.approx([math.radians(v) for v in q])
    assert p.robot_mode_name == "ENABLE" and p.tool_coordinate == 2
    assert p.timestamp_ms == 1_700_000_000_123
    # little-endian per protocol doc: MessageSize 1440 = 0x05A0 -> bytes A0 05
    assert buf[:2] == b"\xa0\x05"
    assert struct.unpack_from("<Q", buf, 48)[0] == TEST_VALUE


@pytest.mark.parametrize("mutate,msg", [
    (lambda b: b[:100], "need"),
    (lambda b: struct.pack("<H", 1000) + b[2:], "MessageSize"),
    (lambda b: b[:48] + struct.pack("<Q", 0) + b[56:], "TestValue"),
    (lambda b: b[:432] + struct.pack("<d", math.nan) + b[440:], "non-finite"),
])
def test_decode_rejects(mutate, msg):
    with pytest.raises(FeedbackDecodeError, match=msg):
        decode(mutate(encode({})))


def _stream(n):
    return [encode({"q_actual_deg": [i, 0, 0, 0, 0, 0], "timestamp_ms": i}) for i in range(n)]


def test_assembler_random_chunks():
    rng = random.Random(1)
    data = b"".join(_stream(50))
    a, out, i = FrameAssembler(), [], 0
    while i < len(data):
        n = rng.randint(1, 900)
        out += list(a.feed(data[i:i + n]))
        i += n
    assert [p.timestamp_ms for p in out] == list(range(50))
    assert a.resyncs == 0 and a.discarded_bytes == 0


def test_assembler_resyncs_after_garbage_and_midstream_join():
    pk = _stream(10)
    data = pk[0][700:] + b"\x00\xff" * 33 + b"".join(pk[1:5]) + bytes(range(97)) + b"".join(pk[5:])
    a = FrameAssembler()
    got = [p.timestamp_ms for p in a.feed(data)]
    assert got == list(range(1, 10))
    assert a.resyncs > 0 and a.discarded_bytes == 740 + 66 + 97


def test_assembler_latest_returns_newest():
    a = FrameAssembler()
    assert a.latest(b"".join(_stream(5))).timestamp_ms == 4
