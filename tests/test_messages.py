import struct

import pytest

from fusai_sdk.constants import (
    AckFlags,
    AckMotionFlags,
    Command,
    CoordType,
    Mode,
    Owner,
    ReadyFlags,
    StatusFlags,
)
from fusai_sdk.exceptions import ProtocolError
from fusai_sdk.framing import Frame
from fusai_sdk.messages import (
    Ack,
    AckMotion,
    ChassisStatus,
    ErrorEvent,
    GimbalStatus,
    MotorStatus,
    NackBusy,
    NackSeqLost,
    Ready,
    decode_ack,
    decode_ack_motion,
    decode_chassis_status,
    decode_error,
    decode_gimbal_status,
    decode_motor_status,
    decode_nack_busy,
    decode_nack_seq_lost,
    decode_ready,
    encode_heartbeat,
    encode_set_delta,
    encode_set_mode,
    encode_set_target,
    encode_set_target_stream,
    encode_set_velocity,
    parse_inbound,
)


def test_set_target_payload() -> None:
    data = encode_set_target(2, 16, 32)
    assert data == bytes.fromhex("02 02 10 00 20 00")
    assert struct.unpack("<BBhh", data) == (2, CoordType.GIMBAL_ANGLE, 16, 32)


def test_set_delta_payload() -> None:
    assert encode_set_delta(1, 16, 32) == bytes.fromhex("01 10 00 20 00")


def test_set_velocity_payload() -> None:
    data = encode_set_velocity(9, -1000, 0, 1000)
    assert struct.unpack("<Bhhh", data) == (9, -1000, 0, 1000)


def test_set_mode_accepts_only_documented_values() -> None:
    assert encode_set_mode(Mode.RELEASE) == b"\x00"
    assert encode_set_mode(Mode.TRACK_GIMBAL) == b"\x01"
    assert encode_set_mode(Mode.EMERGENCY_STOP) == b"\x03"
    with pytest.raises(ValueError):
        encode_set_mode(2)


def test_stream_payload_shape_and_limit() -> None:
    points = [(10, 20), (-10, -20)]
    data = encode_set_target_stream(1, points)
    assert data == bytes.fromhex("01 02 02 0A 00 14 00 F6 FF EC FF")


def test_stream_rejects_bad_point_count() -> None:
    with pytest.raises(ValueError):
        encode_set_target_stream(1, [])
    with pytest.raises(ValueError):
        encode_set_target_stream(1, [(0, 0)] * 11)


def test_heartbeat_is_empty() -> None:
    assert encode_heartbeat() == b""


def test_decode_ack_and_flags() -> None:
    ack = decode_ack(bytes.fromhex("01 02 06"))
    assert ack == Ack(command=1, seq=2, flags=6)
    assert ack.accepted
    assert ack.clamped
    assert not ack.at_target
    assert not ack.rejected


def test_decode_ack_motion() -> None:
    ack = decode_ack_motion(bytes.fromhex("03 01 03"))
    assert ack == AckMotion(seq=3, motion_state=1, flags=3)
    assert ack.accepted
    assert ack.clamped
    assert not ack.busy_rejected

    busy = AckMotion(seq=4, motion_state=5, flags=0)
    assert busy.busy_rejected


def test_decode_nacks() -> None:
    assert decode_nack_busy(bytes.fromhex("03 04 02")) == NackBusy(3, 4, Owner.CHASSIS)
    assert decode_nack_seq_lost(bytes.fromhex("03 08")) == NackSeqLost(3, 8)


def test_decode_error() -> None:
    assert decode_error(bytes.fromhex("04 00")) == ErrorEvent(4, 0)


def test_gimbal_status_round_trip() -> None:
    payload = struct.pack(
        "<BBhhHB",
        1,
        2,
        -123,
        456,
        7,
        StatusFlags.PI_HEARTBEAT_OK | StatusFlags.CHASSIS_STOPPED,
    )
    status = decode_gimbal_status(payload)
    assert status == GimbalStatus(
        gimbal_state=1,
        owner=2,
        pan=-123,
        tilt=456,
        error_count=7,
        flags=0x0C,
    )
    assert status.heartbeat_ok
    assert status.pi_heartbeat_ok
    assert not status.bt_heartbeat_ok
    assert status.chassis_stopped
    assert not status.emergency
    assert status.gimbal_state_name == "TRACKING"
    assert status.owner_name == "CHASSIS"


def test_chassis_status_round_trip() -> None:
    payload = struct.pack(
        "<BBhhhHB",
        1,
        2,
        -1000,
        0,
        1000,
        7,
        StatusFlags.EMERGENCY | StatusFlags.BT_HEARTBEAT_OK,
    )
    status = decode_chassis_status(payload)
    assert status == ChassisStatus(
        motion_state=1,
        owner=2,
        velocity_x=-1000,
        velocity_y=0,
        omega=1000,
        error_count=7,
        flags=0x12,
    )
    assert status.heartbeat_ok
    assert status.bt_heartbeat_ok
    assert not status.pi_heartbeat_ok
    assert status.emergency
    assert status.moving
    assert status.motion_state_name == "MOVING"
    assert status.owner_name == "CHASSIS"


def test_status_enum_labels_and_unknown_values() -> None:
    gimbal = GimbalStatus(9, 9, 0, 0, 0, 0)
    assert gimbal.owner_name == "9"
    assert gimbal.gimbal_state_name == "9"

    chassis = ChassisStatus(9, 9, 0, 0, 0, 0, 0)
    assert chassis.owner_name == "9"
    assert chassis.motion_state_name == "9"


def test_ready_and_flags() -> None:
    ready = decode_ready(bytes.fromhex("02 00 07"))
    assert ready == Ready(
        2,
        0,
        ReadyFlags.HAS_GIMBAL
        | ReadyFlags.HAS_CHASSIS
        | ReadyFlags.HAS_MOTOR_TELEMETRY,
    )
    assert ready.version == (2, 0)
    assert ready.has_gimbal
    assert ready.has_chassis
    assert ready.has_motor_telemetry


def test_motor_status_round_trip() -> None:
    payload = struct.pack("<Bhhhh", 0x01, 120, -45, 0, 0)
    status = decode_motor_status(payload)
    assert status == MotorStatus(present_mask=0x01, rpm=(120, -45, 0, 0))
    assert status.present(0)
    assert not status.present(1)
    with pytest.raises(IndexError):
        status.present(4)


def test_bad_lengths_raise_protocol_error() -> None:
    decoders = [
        decode_ack,
        decode_ack_motion,
        decode_nack_busy,
        decode_nack_seq_lost,
        decode_error,
        decode_gimbal_status,
        decode_chassis_status,
        decode_ready,
        decode_motor_status,
    ]
    for decoder in decoders:
        with pytest.raises(ProtocolError):
            decoder(b"")


def test_parse_inbound_known_and_unknown() -> None:
    assert isinstance(parse_inbound(Frame(Command.ACK, bytes.fromhex("01 02 04"))), Ack)
    assert isinstance(parse_inbound(Frame(Command.ACK_MOTION, bytes.fromhex("02 01 01"))), AckMotion)
    assert isinstance(parse_inbound(Frame(Command.NACK_BUSY, bytes.fromhex("01 02 02"))), NackBusy)
    assert isinstance(parse_inbound(Frame(Command.NACK_SEQ_LOST, bytes.fromhex("01 02"))), NackSeqLost)
    assert isinstance(parse_inbound(Frame(Command.ERROR, bytes.fromhex("04 00"))), ErrorEvent)
    assert isinstance(parse_inbound(Frame(Command.GIMBAL_STATUS, b"\x00" * 9)), GimbalStatus)
    assert isinstance(parse_inbound(Frame(Command.CHASSIS_STATUS, b"\x00" * 11)), ChassisStatus)
    assert isinstance(parse_inbound(Frame(Command.READY, bytes.fromhex("02 00 05"))), Ready)
    assert isinstance(parse_inbound(Frame(Command.MOTOR_STATUS, b"\x01" + b"\x00" * 8)), MotorStatus)
    frame = Frame(0x11, b"")
    assert parse_inbound(frame) is frame
