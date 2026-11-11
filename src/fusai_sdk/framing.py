"""Frame encoding and the streaming frame decoder.

Wire format (protocol v1.4):

    AA 55 | LEN | CMD | DATA(LEN-1) | CRC16 low | CRC16 high | 0D

LEN counts CMD plus DATA, so LEN is 1..64 and DATA is at most 63 bytes.  The
CRC is CRC-16/MODBUS over LEN, CMD and DATA, transmitted little-endian.  Body
bytes (LEN through CRC high, but not SOF/EOF) escape 0xAA as AA 00.  A literal
AA 55 inside a body means a new SOF: the current frame is discarded and
decoding restarts at the LEN byte.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Iterator, Optional

from .constants import (
    EOF_BYTE,
    MAX_DATA_LEN,
    MAX_LEN,
    SOF_FIRST,
    SOF_SECOND,
)
from .crc import crc16_modbus


@dataclass(frozen=True)
class Frame:
    """A decoded frame's command byte and payload."""

    cmd: int
    data: bytes = b""

    @property
    def command(self) -> int:
        """Alias for cmd."""

        return self.cmd

    @property
    def data_len(self) -> int:
        return len(self.data)

    @property
    def length(self) -> int:
        """LEN field value (CMD plus DATA bytes)."""

        return len(self.data) + 1


def encode_frame(cmd: int, data: bytes = b"") -> bytes:
    """Encode a complete, escaped frame including SOF and EOF."""

    payload = bytes(data)
    if not isinstance(cmd, int) or not 0 <= cmd <= 0xFF:
        raise ValueError("command must be an 8-bit integer")
    if len(payload) > MAX_DATA_LEN:
        raise ValueError("payload is %d bytes; maximum is %d" % (len(payload), MAX_DATA_LEN))

    body = bytearray()
    body.append(len(payload) + 1)
    body.append(cmd)
    body.extend(payload)
    crc = crc16_modbus(body)
    body.append(crc & 0xFF)
    body.append((crc >> 8) & 0xFF)

    wire = bytearray((SOF_FIRST, SOF_SECOND))
    for byte in body:
        wire.append(byte)
        if byte == SOF_FIRST:
            wire.append(0x00)
    wire.append(EOF_BYTE)
    return bytes(wire)


class _State(IntEnum):
    WAIT_SOF = 0
    WAIT_SOF_SECOND = 1
    LENGTH = 2
    BODY = 3
    CRC_LOW = 4
    CRC_HIGH = 5
    EOF = 6


class FrameDecoder:
    """Incremental, resynchronizing frame decoder.

    Feed arbitrary byte chunks with feed_bytes; valid frames are yielded as
    soon as their EOF byte arrives.  Counters are diagnostic and do not affect
    protocol processing.
    """

    def __init__(self) -> None:
        self._state = _State.WAIT_SOF
        self._body = bytearray()
        self._expected = 0
        self._escaped = False
        self._crc_low = 0
        self._crc_high = 0
        self.bytes_fed = 0
        self.frames_decoded = 0
        self.format_errors = 0
        self.crc_errors = 0
        self.consecutive_crc_errors = 0
        self.crc_alarm = False
        self.dropped_frames = 0

    def reset(self) -> None:
        """Reset parser state and every diagnostic counter."""

        self._reset_parser()
        self.bytes_fed = 0
        self.frames_decoded = 0
        self.format_errors = 0
        self.crc_errors = 0
        self.consecutive_crc_errors = 0
        self.crc_alarm = False
        self.dropped_frames = 0

    def clear_crc_alarm(self) -> None:
        """Clear the latched five-consecutive-CRC-errors alarm."""

        self.crc_alarm = False

    def feed(self, byte: int) -> Optional[Frame]:
        """Feed one byte.  Return a frame when one completes, else None."""

        if not isinstance(byte, int) or not 0 <= byte <= 0xFF:
            raise ValueError("byte must be an integer in 0..255")
        self.bytes_fed += 1

        if self._state == _State.WAIT_SOF:
            if byte == SOF_FIRST:
                self._state = _State.WAIT_SOF_SECOND
            return None

        if self._state == _State.WAIT_SOF_SECOND:
            if byte == SOF_FIRST:
                return None
            if byte == SOF_SECOND:
                self._state = _State.LENGTH
            else:
                self._state = _State.WAIT_SOF
            return None

        if self._state in (_State.BODY, _State.CRC_LOW, _State.CRC_HIGH):
            if self._escaped:
                self._escaped = False
                if byte == 0x00:
                    return self._feed_body_byte(SOF_FIRST)
                if byte == SOF_SECOND:
                    self.dropped_frames += 1
                    self._reset_parser()
                    self._state = _State.LENGTH
                    return None
                self._format_error()
                return None
            if byte == SOF_FIRST:
                self._escaped = True
                return None

        return self._feed_body_byte(byte)

    def feed_bytes(self, data: bytes) -> Iterator[Frame]:
        """Feed a byte string and yield every frame completed in it."""

        for byte in data:
            frame = self.feed(byte)
            if frame is not None:
                yield frame

    def _feed_body_byte(self, byte: int) -> Optional[Frame]:
        if self._state == _State.LENGTH:
            if byte < 1 or byte > MAX_LEN:
                self._format_error()
                return None
            self._body = bytearray((byte,))
            self._expected = byte + 1
            self._state = _State.BODY
            return None

        if self._state == _State.BODY:
            self._body.append(byte)
            if len(self._body) >= self._expected:
                self._state = _State.CRC_LOW
            return None

        if self._state == _State.CRC_LOW:
            self._crc_low = byte
            self._state = _State.CRC_HIGH
            return None

        if self._state == _State.CRC_HIGH:
            self._crc_high = byte
            self._state = _State.EOF
            return None

        if self._state == _State.EOF:
            if byte != EOF_BYTE:
                self._format_error()
                return None
            return self._finish()

        return None

    def _finish(self) -> Optional[Frame]:
        received_crc = self._crc_low | (self._crc_high << 8)
        expected_crc = crc16_modbus(bytes(self._body))
        if received_crc != expected_crc:
            self.crc_errors += 1
            self.consecutive_crc_errors += 1
            if self.consecutive_crc_errors >= 5 and not self.crc_alarm:
                self.crc_alarm = True
            self._reset_parser()
            return None

        self.consecutive_crc_errors = 0
        self.frames_decoded += 1
        frame = Frame(cmd=self._body[1], data=bytes(self._body[2:]))
        self._reset_parser()
        return frame

    def _format_error(self) -> None:
        self.format_errors += 1
        self._reset_parser()

    def _reset_parser(self) -> None:
        self._state = _State.WAIT_SOF
        self._body = bytearray()
        self._expected = 0
        self._escaped = False
        self._crc_low = 0
        self._crc_high = 0
