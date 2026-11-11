"""Constants and enums for the Fusai v2.0 two-link serial protocol.

All values in this module are wire-level values.  They are deliberately kept
free of any hardware or OS dependency so the codec can be tested on a host.

Version 2.0 splits the old single link into two physical endpoints:

* Link.PI (Raspberry Pi) carries gimbal commands and link heartbeat.
* Link.BT (HC-05 Bluetooth SPP) carries chassis commands and link heartbeat.

Both endpoints share the frame format, CRC, owner arbiter and emergency stop.
"""

from __future__ import annotations

from enum import IntEnum, IntFlag

PROTOCOL_VERSION_MAJOR = 2
PROTOCOL_VERSION_MINOR = 0
PROTOCOL_VERSION = (PROTOCOL_VERSION_MAJOR, PROTOCOL_VERSION_MINOR)
PROTOCOL_VERSION_STRING = "2.0"

DEFAULT_BAUDRATE = 115200

SOF_FIRST = 0xAA
SOF_SECOND = 0x55
EOF_BYTE = 0x0D

MAX_LEN = 64
MAX_DATA_LEN = 63
MAX_WIRE_LEN = 137

MAX_DELTA = 80
MAX_STREAM_POINTS = 10
MOTOR_COUNT = 4

COORD_TYPE_GIMBAL_ANGLE = 2


class Link(IntEnum):
    """Physical endpoint.  The same firmware serves both."""

    PI = 0
    BT = 1


class CoordType(IntEnum):
    """Coordinate system used by gimbal point commands."""

    GIMBAL_ANGLE = COORD_TYPE_GIMBAL_ANGLE


class Command(IntEnum):
    """Command byte values (CMD)."""

    SET_TARGET = 0x01
    SET_DELTA = 0x02
    SET_VELOCITY = 0x03
    SET_MODE = 0x04
    SET_TARGET_STREAM = 0x05
    HEARTBEAT = 0x10
    RESERVED_CALIBRATE = 0x11
    READY = 0x80
    GIMBAL_STATUS = 0x81
    ACK = 0x82
    ACK_MOTION = 0x83
    NACK_BUSY = 0x84
    NACK_SEQ_LOST = 0x85
    ERROR = 0x86
    MOTOR_STATUS = 0x87
    CHASSIS_STATUS = 0x88


class Mode(IntEnum):
    """SET_MODE payload values accepted by protocol v2.0."""

    RELEASE = 0
    TRACK_GIMBAL = 1
    EMERGENCY_STOP = 3


class Owner(IntEnum):
    """Motion arbiter owner, shared by both links."""

    NONE = 0
    GIMBAL = 1
    CHASSIS = 2
    ESTOP_FAULT = 3


class GimbalState(IntEnum):
    IDLE = 0
    TRACKING = 1
    RESERVED = 2
    ERROR = 3
    ESTOP = 4


class MotionState(IntEnum):
    STOPPED = 0
    MOVING = 1
    TIMEOUT_DECEL = 2
    FAULT = 3
    ESTOP = 4
    BUSY_REJECT = 5


class ErrorCode(IntEnum):
    CRC = 0x01
    FRAME = 0x02
    STATE = 0x03
    HEARTBEAT = 0x04
    RESERVED_05 = 0x05
    RESERVED_06 = 0x06
    UNKNOWN_COMMAND = 0x07
    WRONG_LINK = 0x08


class AckFlags(IntFlag):
    AT_TARGET = 0x01
    CLAMPED = 0x02
    ACCEPTED = 0x04
    REJECTED = 0x08


class AckMotionFlags(IntFlag):
    ACCEPTED = 0x01
    CLAMPED = 0x02
    STOPPED = 0x04


class StatusFlags(IntFlag):
    """Flags shared by GIMBAL_STATUS and CHASSIS_STATUS."""

    RESERVED_0 = 0x01
    EMERGENCY = 0x02
    PI_HEARTBEAT_OK = 0x04
    CHASSIS_STOPPED = 0x08
    BT_HEARTBEAT_OK = 0x10
    PI_LINK_LOST = 0x20
    BT_LINK_LOST = 0x40


class ReadyFlags(IntFlag):
    HAS_GIMBAL = 0x01
    HAS_CHASSIS = 0x02
    HAS_MOTOR_TELEMETRY = 0x04
