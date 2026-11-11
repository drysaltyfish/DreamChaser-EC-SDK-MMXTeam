"""Typed exceptions raised by the Fusai SDK."""

from __future__ import annotations

from typing import Optional


class FusaiError(Exception):
    """Base class for every SDK error."""


class TransportError(FusaiError):
    """The serial link could not be opened, read, or written."""


class ProtocolError(FusaiError):
    """A well-formed frame carried an invalid protocol payload."""


class CommandTimeout(FusaiError):
    """No response arrived before the command deadline."""

    def __init__(self, command: int, seq: int, attempts: int, message: Optional[str] = None):
        self.command = command
        self.seq = seq
        self.attempts = attempts
        if message is None:
            message = (
                "no response for command 0x%02X seq %d after %d attempt(s); "
                "the command may or may not have executed"
                % (command, seq, attempts)
            )
        super().__init__(message)


class SequenceUnknownError(FusaiError):
    """A previous timeout left the server-side sequence baseline unknown."""


class WrongLinkError(FusaiError):
    """A command was attempted on the physical link that does not own it.

    The firmware reserves gimbal commands for the Raspberry Pi link and
    chassis commands for the Bluetooth link.  The SDK rejects the wrong call
    locally instead of sending a frame the firmware would answer with
    ERROR(WRONG_LINK).
    """

    def __init__(self, command: int, link: int):
        self.command = command
        self.link = link
        super().__init__(
            "command 0x%02X is not available on link %d; "
            "use Link.PI for gimbal commands and Link.BT for chassis commands"
            % (command, link)
        )


class EmergencyStop(FusaiError):
    """A pending command was interrupted by a caller-requested emergency stop."""


class DeviceError(FusaiError):
    """The firmware sent an ERROR frame."""

    def __init__(self, code: int, detail: int):
        self.code = code
        self.detail = detail
        super().__init__(
            "device ERROR code=0x%02X (%s) detail=0x%02X"
            % (code, _error_name(code), detail)
        )


class BusyError(FusaiError):
    """The motion arbiter rejected a command with NACK_BUSY."""

    def __init__(self, command: int, seq: int, owner: int):
        self.command = command
        self.seq = seq
        self.owner = owner
        super().__init__(
            "motion busy: command 0x%02X seq %d rejected; owner=%d"
            % (command, seq, owner)
        )


class SequenceLostError(FusaiError):
    """The firmware reported NACK_SEQ_LOST."""

    def __init__(self, command: int, expected_seq: int):
        self.command = command
        self.expected_seq = expected_seq
        super().__init__(
            "sequence lost for command 0x%02X; firmware expects seq %d"
            % (command, expected_seq)
        )


class MotionRejected(FusaiError):
    """A motion command was acknowledged but not accepted."""

    def __init__(self, seq: int, motion_state: Optional[int], flags: int, message: Optional[str] = None):
        self.seq = seq
        self.motion_state = motion_state
        self.flags = flags
        if message is None:
            message = "motion rejected: seq=%d motion_state=%s flags=0x%02X" % (
                seq,
                motion_state,
                flags,
            )
        super().__init__(message)


class ReadyTimeout(FusaiError):
    """No READY frame arrived before the deadline."""


class StatusTimeout(FusaiError):
    """No matching STATUS frame arrived before the deadline."""


class CalibrationError(FusaiError):
    """A calibration sample set is incomplete or numerically singular."""


_ERROR_NAMES = {
    0x01: "CRC",
    0x02: "FRAME",
    0x03: "STATE",
    0x04: "HEARTBEAT",
    0x05: "RESERVED_05",
    0x06: "RESERVED_06",
    0x07: "UNKNOWN_COMMAND",
    0x08: "WRONG_LINK",
}


def _error_name(code: int) -> str:
    return _ERROR_NAMES.get(code, "UNKNOWN_0x%02X" % code)
