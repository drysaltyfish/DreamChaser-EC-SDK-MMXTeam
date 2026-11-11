"""Transport abstraction and the pyserial-backed implementation."""

from __future__ import annotations

import threading
from typing import Any, Optional, Protocol, runtime_checkable

from .exceptions import TransportError


@runtime_checkable
class Transport(Protocol):
    """Minimal byte-stream interface used by FusaiClient."""

    def open(self) -> None:
        ...

    def read(self, size: int = 4096) -> bytes:
        ...

    def write(self, data: bytes) -> None:
        ...

    def close(self) -> None:
        ...


class SerialTransport:
    """A pyserial byte stream.

    pyserial is imported lazily so protocol-only unit tests do not need it
    installed.  The serial port is opened by open, called by FusaiClient.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        read_timeout: float = 0.05,
        write_timeout: float = 1.0,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.read_timeout = read_timeout
        self.write_timeout = write_timeout
        self._serial: Optional[Any] = None
        self._write_lock = threading.Lock()

    @property
    def is_open(self) -> bool:
        return self._serial is not None and bool(self._serial.is_open)

    def open(self) -> None:
        if self.is_open:
            return
        try:
            import serial
        except ImportError as exc:  # pragma: no cover - depends on environment
            raise TransportError("pyserial is required: install fusai-sdk with its dependencies") from exc

        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.read_timeout,
                write_timeout=self.write_timeout,
            )
        except Exception as exc:
            raise TransportError("cannot open serial port %s: %s" % (self.port, exc)) from exc

    def read(self, size: int = 4096) -> bytes:
        serial_obj = self._serial
        if serial_obj is None:
            raise TransportError("serial port is not open")
        try:
            return bytes(serial_obj.read(size))
        except Exception as exc:
            raise TransportError("serial read failed: %s" % exc) from exc

    def write(self, data: bytes) -> None:
        serial_obj = self._serial
        if serial_obj is None:
            raise TransportError("serial port is not open")
        try:
            with self._write_lock:
                written = serial_obj.write(data)
        except Exception as exc:
            raise TransportError("serial write failed: %s" % exc) from exc
        if written is not None and written != len(data):
            raise TransportError("serial write was incomplete (%s of %d bytes)" % (written, len(data)))

    def close(self) -> None:
        serial_obj = self._serial
        self._serial = None
        if serial_obj is None:
            return
        try:
            if serial_obj.is_open:
                serial_obj.close()
        except Exception as exc:  # pragma: no cover - close failures are diagnostic
            raise TransportError("serial close failed: %s" % exc) from exc

    def __enter__(self) -> "SerialTransport":
        self.open()
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()
