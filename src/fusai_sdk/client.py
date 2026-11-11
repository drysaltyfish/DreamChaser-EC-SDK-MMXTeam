"""Synchronous Fusai protocol client.

The client owns two background threads:

* a reader thread that decodes incoming frames and completes pending commands;
* an optional heartbeat thread that sends HEARTBEAT every 100 ms.

One client instance drives one physical link.  Protocol 2.0 splits the command
set: the Raspberry Pi link (Link.PI) controls the gimbal, and the Bluetooth
link (Link.BT) controls the chassis.  A client refuses commands that belong to
the other link with WrongLinkError before anything is written to the wire.

Commands are serialized by one lock, so the SDK never has two command
transactions in flight.  Sequence numbers mirror the firmware's rules and are
advanced only when the firmware acknowledges the command.  A command timeout
leaves the server-side sequence baseline unknown; further motion commands are
blocked until the caller performs a safe resynchronization with release(),
track_gimbal(), or stop_chassis().
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable, Optional, Sequence, Tuple

from .constants import Command, DEFAULT_BAUDRATE, Link, Mode
from .exceptions import (
    BusyError,
    CommandTimeout,
    DeviceError,
    EmergencyStop,
    MotionRejected,
    ProtocolError,
    ReadyTimeout,
    SequenceLostError,
    SequenceUnknownError,
    StatusTimeout,
    TransportError,
    WrongLinkError,
)
from .framing import Frame, FrameDecoder, encode_frame
from .messages import (
    Ack,
    AckMotion,
    ChassisStatus,
    ErrorEvent,
    GimbalStatus,
    LinkStatus,
    MotorStatus,
    Ready,
    decode_ack,
    decode_ack_motion,
    decode_chassis_status,
    decode_error,
    decode_gimbal_status,
    decode_nack_busy,
    decode_motor_status,
    decode_nack_seq_lost,
    decode_ready,
    encode_heartbeat,
    encode_set_delta,
    encode_set_mode,
    encode_set_target,
    encode_set_target_stream,
    encode_set_velocity,
)
from .transport import SerialTransport, Transport

_LOG = logging.getLogger(__name__)

StatusCallback = Callable[[LinkStatus], None]
GimbalStatusCallback = Callable[[GimbalStatus], None]
ChassisStatusCallback = Callable[[ChassisStatus], None]
ReadyCallback = Callable[[Ready], None]
ErrorCallback = Callable[[ErrorEvent], None]
MotorStatusCallback = Callable[[MotorStatus], None]


class _Pending:
    __slots__ = ("command", "seq", "event", "response", "error")

    def __init__(self, command: int, seq: int) -> None:
        self.command = command
        self.seq = seq
        self.event = threading.Event()
        self.response: Any = None
        self.error: Optional[BaseException] = None


class FusaiClient:
    """Thread-safe, synchronous host-side client for protocol v2.0.

    Provide either port (a pyserial port name such as /dev/ttyAMA0 or COM5) or
    transport (a custom Transport, useful for tests).  The constructor does not
    open the link; call open() or use the object as a context manager.

    link selects the physical endpoint: Link.PI (Raspberry Pi, gimbal) or
    Link.BT (Bluetooth, chassis).  Each endpoint gets its own client and its
    own serial port.
    """

    def __init__(
        self,
        port: Optional[str] = None,
        *,
        transport: Optional[Transport] = None,
        link: Link = Link.PI,
        baudrate: int = DEFAULT_BAUDRATE,
        read_timeout: float = 0.05,
        heartbeat_interval: float = 0.1,
        auto_heartbeat: bool = True,
        command_timeout: float = 0.3,
        retries: int = 2,
        resync_on_seq_lost: bool = True,
        on_status: Optional[StatusCallback] = None,
        on_gimbal_status: Optional[GimbalStatusCallback] = None,
        on_chassis_status: Optional[ChassisStatusCallback] = None,
        on_motor_status: Optional[MotorStatusCallback] = None,
        on_ready: Optional[ReadyCallback] = None,
        on_error: Optional[ErrorCallback] = None,
    ) -> None:
        if (port is None) == (transport is None):
            raise ValueError("provide exactly one of port or transport")
        try:
            self._link = Link(link)
        except ValueError as exc:
            raise ValueError("link must be Link.PI or Link.BT") from exc
        if command_timeout <= 0:
            raise ValueError("command_timeout must be positive")
        if heartbeat_interval <= 0:
            raise ValueError("heartbeat_interval must be positive")

        self._transport: Transport = (
            transport
            if transport is not None
            else SerialTransport(
                port=port or "",
                baudrate=baudrate,
                read_timeout=read_timeout,
            )
        )
        self._command_timeout = float(command_timeout)
        self._retries = max(0, int(retries))
        self._heartbeat_interval = float(heartbeat_interval)
        self._auto_heartbeat = bool(auto_heartbeat)
        self._resync_on_seq_lost = bool(resync_on_seq_lost)

        self._on_status = on_status
        self._on_gimbal_status = on_gimbal_status
        self._on_chassis_status = on_chassis_status
        self._on_motor_status = on_motor_status
        self._on_ready = on_ready
        self._on_error = on_error

        self._condition = threading.Condition()
        self._command_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._stop = threading.Event()

        self._reader_thread: Optional[threading.Thread] = None
        self._heartbeat_thread: Optional[threading.Thread] = None
        self._opened = False
        self._link_error: Optional[BaseException] = None
        self._last_rx = 0.0

        self._gimbal_status: Optional[GimbalStatus] = None
        self._chassis_status: Optional[ChassisStatus] = None
        self._ready: Optional[Ready] = None
        self._motor_status: Optional[MotorStatus] = None
        self._pending: Optional[_Pending] = None

        self._gimbal_next_seq = 0
        self._chassis_next_seq = 0
        self._gimbal_seq_unknown = False
        self._chassis_seq_unknown = False

    # ------------------------------------------------------------------
    # lifecycle

    def open(self) -> "FusaiClient":
        """Open the transport, start background threads, and send a heartbeat."""

        with self._lifecycle_lock:
            if self._opened:
                return self

            with self._condition:
                self._link_error = None
                self._last_rx = 0.0
                self._gimbal_status = None
                self._chassis_status = None
                self._ready = None
                self._motor_status = None
                self._pending = None
                self._gimbal_next_seq = 0
                self._chassis_next_seq = 0
                self._gimbal_seq_unknown = False
                self._chassis_seq_unknown = False
            self._stop.clear()
            self._transport.open()
            self._opened = True

            reader = threading.Thread(
                target=self._reader_loop,
                name="fusai-reader",
                daemon=True,
            )
            self._reader_thread = reader
            reader.start()

            try:
                self._write_frame(encode_frame(int(Command.HEARTBEAT), encode_heartbeat()))
            except BaseException:
                self._opened = False
                self._stop.set()
                try:
                    self._transport.close()
                except Exception:
                    pass
                raise

            if self._auto_heartbeat:
                heartbeat = threading.Thread(
                    target=self._heartbeat_loop,
                    name="fusai-heartbeat",
                    daemon=True,
                )
                self._heartbeat_thread = heartbeat
                heartbeat.start()
            return self

    def close(self) -> None:
        """Stop background threads and close the transport.  Idempotent."""

        with self._lifecycle_lock:
            reader = self._reader_thread
            heartbeat = self._heartbeat_thread
            self._reader_thread = None
            self._heartbeat_thread = None
            self._stop.set()
            self._opened = False

            try:
                self._transport.close()
            except Exception:
                _LOG.debug("transport close failed", exc_info=True)

            with self._condition:
                pending = self._pending
                self._pending = None
                if pending is not None:
                    pending.error = TransportError("client closed")
                    pending.event.set()
                self._condition.notify_all()

        for thread in (reader, heartbeat):
            if thread is not None and thread.is_alive():
                thread.join(timeout=2.0)

    def __enter__(self) -> "FusaiClient":
        return self.open()

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()

    def _ensure_open(self) -> None:
        if not self._opened:
            raise TransportError("client is not open")
        with self._condition:
            error = self._link_error
        if error is not None:
            if isinstance(error, TransportError):
                raise error
            raise TransportError("link failed: %s" % error) from error

    # ------------------------------------------------------------------
    # raw I/O

    def _write_frame(self, data: bytes) -> None:
        with self._write_lock:
            if not self._opened:
                raise TransportError("client is not open")
            try:
                self._transport.write(data)
            except TransportError:
                raise
            except Exception as exc:
                raise TransportError("transport write failed: %s" % exc) from exc

    def _heartbeat_loop(self) -> None:
        while not self._stop.wait(self._heartbeat_interval):
            try:
                self._write_frame(encode_frame(int(Command.HEARTBEAT), encode_heartbeat()))
            except Exception as exc:
                if not self._stop.is_set():
                    if isinstance(exc, TransportError):
                        self._set_link_error(exc)
                    else:
                        self._set_link_error(TransportError(str(exc)))
                return

    def _reader_loop(self) -> None:
        decoder = FrameDecoder()
        while not self._stop.is_set():
            try:
                chunk = self._transport.read(4096)
            except Exception as exc:
                if not self._stop.is_set():
                    self._set_link_error(TransportError("transport read failed: %s" % exc))
                return
            if not chunk:
                continue

            with self._condition:
                self._last_rx = time.monotonic()
            for frame in decoder.feed_bytes(chunk):
                try:
                    self._dispatch(frame)
                except Exception:
                    _LOG.exception("failed to dispatch frame cmd=0x%02X", frame.cmd)

    def _set_link_error(self, error: BaseException) -> None:
        with self._condition:
            self._link_error = error
            pending = self._pending
            self._pending = None
            if pending is not None:
                pending.error = error
                pending.event.set()
            self._condition.notify_all()

    def _dispatch(self, frame: Frame) -> None:
        command = frame.cmd

        if command == int(Command.GIMBAL_STATUS):
            gimbal_status = decode_gimbal_status(frame.data)
            with self._condition:
                self._gimbal_status = gimbal_status
                self._condition.notify_all()
            self._safe_callback(self._on_gimbal_status, gimbal_status)
            self._safe_callback(self._on_status, gimbal_status)
            return

        if command == int(Command.CHASSIS_STATUS):
            chassis_status = decode_chassis_status(frame.data)
            with self._condition:
                self._chassis_status = chassis_status
                self._condition.notify_all()
            self._safe_callback(self._on_chassis_status, chassis_status)
            self._safe_callback(self._on_status, chassis_status)
            return

        if command == int(Command.READY):
            ready = decode_ready(frame.data)
            with self._condition:
                self._ready = ready
                self._condition.notify_all()
            self._safe_callback(self._on_ready, ready)
            return

        if command == int(Command.MOTOR_STATUS):
            motor_status = decode_motor_status(frame.data)
            with self._condition:
                self._motor_status = motor_status
                self._condition.notify_all()
            self._safe_callback(self._on_motor_status, motor_status)
            return

        if command == int(Command.ACK):
            ack = decode_ack(frame.data)
            with self._condition:
                pending = self._pending
                if (
                    pending is not None
                    and pending.command == ack.command
                    and pending.seq == ack.seq
                ):
                    self._pending = None
                    pending.response = ack
                    pending.event.set()
            return

        if command == int(Command.ACK_MOTION):
            ack_motion = decode_ack_motion(frame.data)
            with self._condition:
                pending = self._pending
                if (
                    pending is not None
                    and pending.command == int(Command.SET_VELOCITY)
                    and pending.seq == ack_motion.seq
                ):
                    self._pending = None
                    pending.response = ack_motion
                    pending.event.set()
            return

        if command == int(Command.NACK_BUSY):
            busy = decode_nack_busy(frame.data)
            with self._condition:
                pending = self._pending
                if (
                    pending is not None
                    and pending.command == busy.command
                    and pending.seq == busy.seq
                ):
                    self._pending = None
                    pending.error = BusyError(busy.command, busy.seq, busy.owner)
                    pending.event.set()
            return

        if command == int(Command.NACK_SEQ_LOST):
            lost = decode_nack_seq_lost(frame.data)
            with self._condition:
                pending = self._pending
                if pending is not None and pending.command == lost.command:
                    self._pending = None
                    pending.error = SequenceLostError(lost.command, lost.expected_seq)
                    pending.event.set()
            return

        if command == int(Command.ERROR):
            event = decode_error(frame.data)
            with self._condition:
                pending = self._pending
                if pending is not None and pending.command == event.detail:
                    self._pending = None
                    pending.error = DeviceError(event.code, event.detail)
                    pending.event.set()
            self._safe_callback(self._on_error, event)
            return

        _LOG.debug("ignoring unknown inbound command 0x%02X", command)

    @staticmethod
    def _safe_callback(callback: Optional[Callable[[Any], None]], value: Any) -> None:
        if callback is None:
            return
        try:
            callback(value)
        except Exception:
            _LOG.exception("SDK callback failed")

    # ------------------------------------------------------------------
    # request/response

    def _exchange(
        self,
        frame_bytes: bytes,
        command: int,
        seq: int,
        timeout: Optional[float] = None,
    ) -> Any:
        wait_timeout = self._command_timeout if timeout is None else float(timeout)
        attempts = self._retries + 1

        for attempt in range(attempts):
            pending = _Pending(command, seq)
            with self._condition:
                if self._link_error is not None:
                    error = self._link_error
                    if isinstance(error, TransportError):
                        raise error
                    raise TransportError("link failed: %s" % error) from error
                self._pending = pending

            try:
                self._write_frame(frame_bytes)
            except BaseException:
                with self._condition:
                    if self._pending is pending:
                        self._pending = None
                raise

            if not pending.event.wait(wait_timeout):
                with self._condition:
                    if self._pending is pending:
                        self._pending = None
                    canceled = pending.error
                if canceled is not None:
                    raise canceled
                if attempt + 1 >= attempts:
                    raise CommandTimeout(command, seq, attempt + 1)
                continue

            with self._condition:
                if self._pending is pending:
                    self._pending = None
            if pending.error is not None:
                raise pending.error
            return pending.response

        raise CommandTimeout(command, seq, attempts)

    def _gimbal_transaction(
        self,
        build: Callable[[int], bytes],
        command: int,
    ) -> Ack:
        with self._command_lock:
            self._ensure_open()
            if self._link != Link.PI:
                raise WrongLinkError(command, int(self._link))
            if self._gimbal_seq_unknown:
                raise SequenceUnknownError(
                    "previous timeout left the gimbal sequence unknown; "
                    "call release() (SET_MODE 0) or track_gimbal() to resynchronize"
                )

            resyncs = 0
            while True:
                seq = self._gimbal_next_seq
                frame = encode_frame(command, build(seq))
                try:
                    response = self._exchange(frame, command, seq)
                except SequenceLostError as exc:
                    if not self._resync_on_seq_lost or resyncs >= 2:
                        raise
                    self._gimbal_next_seq = exc.expected_seq & 0xFF
                    resyncs += 1
                    continue
                except CommandTimeout:
                    self._gimbal_seq_unknown = True
                    raise

                if not isinstance(response, Ack):
                    raise ProtocolError("unexpected response for gimbal command")
                if not response.accepted:
                    raise MotionRejected(seq, None, response.flags)
                self._gimbal_next_seq = (seq + 1) & 0xFF
                return response

    def _velocity_transaction(
        self,
        build: Callable[[int], bytes],
        allow_unknown_sequence: bool = False,
    ) -> AckMotion:
        with self._command_lock:
            self._ensure_open()
            if self._link != Link.BT:
                raise WrongLinkError(int(Command.SET_VELOCITY), int(self._link))
            if self._chassis_seq_unknown and not allow_unknown_sequence:
                raise SequenceUnknownError(
                    "previous velocity timeout left the chassis sequence unknown; "
                    "call stop_chassis() or release() to resynchronize"
                )

            # When the previous sequence is unknown, advance the candidate.
            # A stop with a new sequence cannot be mistaken by the firmware
            # for the duplicate of the command whose ACK was lost.
            seq = self._chassis_next_seq
            resyncs = 0
            while True:
                frame = encode_frame(int(Command.SET_VELOCITY), build(seq))
                try:
                    response = self._exchange(frame, int(Command.SET_VELOCITY), seq)
                except SequenceLostError as exc:
                    if not self._resync_on_seq_lost or resyncs >= 2:
                        raise
                    self._chassis_next_seq = exc.expected_seq & 0xFF
                    self._chassis_seq_unknown = False
                    seq = self._chassis_next_seq
                    resyncs += 1
                    continue
                except CommandTimeout:
                    # The server may have executed this sequence even though
                    # its ACK was lost.  Advance past it so a later safe stop
                    # is not classified as a duplicate of this command.
                    self._chassis_next_seq = (seq + 1) & 0xFF
                    self._chassis_seq_unknown = True
                    raise

                if not isinstance(response, AckMotion):
                    raise ProtocolError("unexpected response for velocity command")
                if response.busy_rejected:
                    raise MotionRejected(
                        seq,
                        response.motion_state,
                        response.flags,
                        "chassis velocity rejected (motion_state=5, BUSY_REJECT)",
                    )
                if not response.accepted:
                    raise MotionRejected(seq, response.motion_state, response.flags)

                self._chassis_next_seq = (seq + 1) & 0xFF
                self._chassis_seq_unknown = False
                return response

    # ------------------------------------------------------------------
    # public commands

    def set_target(self, pan: int, tilt: int) -> Ack:
        """Move the gimbal to an absolute angle in 0.01 degree units."""

        return self._gimbal_transaction(
            lambda seq: encode_set_target(seq, pan, tilt),
            int(Command.SET_TARGET),
        )

    def set_delta(self, dpan: int, dtilt: int) -> Ack:
        """Move the gimbal by a delta.  Firmware clamps each axis to +/-80."""

        return self._gimbal_transaction(
            lambda seq: encode_set_delta(seq, dpan, dtilt),
            int(Command.SET_DELTA),
        )

    def set_target_stream(self, points: Sequence[Tuple[int, int]]) -> Ack:
        """Send up to 10 absolute gimbal waypoints for sequential execution."""

        return self._gimbal_transaction(
            lambda seq: encode_set_target_stream(seq, points),
            int(Command.SET_TARGET_STREAM),
        )

    def set_velocity(self, vx: int, vy: int, omega: int) -> AckMotion:
        """Set normalized chassis velocity components in -1000..1000.

        The firmware clamps out-of-range values and reports the clamp in the
        ACK_MOTION flags.  All-zero velocity is a safe stop and is accepted
        even when the chassis sequence is unknown.
        """

        all_zero = int(vx) == 0 and int(vy) == 0 and int(omega) == 0
        return self._velocity_transaction(
            lambda seq: encode_set_velocity(seq, vx, vy, omega),
            allow_unknown_sequence=all_zero,
        )

    def stop_chassis(self) -> AckMotion:
        """Send an all-zero velocity command; safe resynchronization for chassis."""

        return self.set_velocity(0, 0, 0)

    def set_mode(self, mode: int) -> Optional[Ack]:
        """Send SET_MODE.  Mode 3 is fire-and-forget and returns None."""

        value = int(mode)
        if value not in (
            int(Mode.RELEASE),
            int(Mode.TRACK_GIMBAL),
            int(Mode.EMERGENCY_STOP),
        ):
            raise ValueError("mode must be 0, 1, or 3 in protocol v2.0")
        if value == int(Mode.TRACK_GIMBAL) and self._link != Link.PI:
            raise WrongLinkError(int(Command.SET_MODE), int(self._link))
        if value == int(Mode.EMERGENCY_STOP):
            self.emergency_stop()
            return None

        with self._command_lock:
            self._ensure_open()
            frame = encode_frame(int(Command.SET_MODE), encode_set_mode(value))
            response = self._exchange(frame, int(Command.SET_MODE), 0)
            if not isinstance(response, Ack):
                raise ProtocolError("unexpected response for SET_MODE")
            if not response.accepted:
                raise MotionRejected(0, None, response.flags)

            if value == int(Mode.RELEASE):
                # The firmware only clears the sequence baseline of the link
                # that issued the release, so mirror that per-link rule.
                if self._link == Link.PI:
                    self._gimbal_next_seq = 0
                    self._gimbal_seq_unknown = False
                else:
                    self._chassis_next_seq = 0
                    self._chassis_seq_unknown = False
            else:
                self._gimbal_next_seq = 0
                self._gimbal_seq_unknown = False
            return response

    def release(self) -> Ack:
        """SET_MODE(0): release motion ownership and clear sequence baselines."""

        response = self.set_mode(int(Mode.RELEASE))
        if response is None:
            raise ProtocolError("SET_MODE(0) unexpectedly had no response")
        return response

    def track_gimbal(self) -> Ack:
        """SET_MODE(1): give motion ownership to the gimbal."""

        response = self.set_mode(int(Mode.TRACK_GIMBAL))
        if response is None:
            raise ProtocolError("SET_MODE(1) unexpectedly had no response")
        return response

    def emergency_stop(self) -> None:
        """SET_MODE(3): immediate emergency stop, no ACK is sent by firmware.

        This bypasses the command lock because waiting behind an in-flight
        motion transaction would delay the stop.  Any pending command is
        failed with EmergencyStop, and both sequence baselines are marked
        unknown.  SET_MODE(0) is required to leave the emergency state.
        """

        self._ensure_open()
        with self._condition:
            pending = self._pending
            self._pending = None
            if pending is not None:
                pending.error = EmergencyStop("command interrupted by emergency stop")
                pending.event.set()
            self._gimbal_seq_unknown = True
            self._chassis_seq_unknown = True
            self._condition.notify_all()

        self._write_frame(encode_frame(int(Command.SET_MODE), encode_set_mode(3)))

    def heartbeat(self) -> None:
        """Send one HEARTBEAT frame."""

        self._ensure_open()
        self._write_frame(encode_frame(int(Command.HEARTBEAT), encode_heartbeat()))

    # ------------------------------------------------------------------
    # state waits and snapshots

    def wait_ready(self, timeout: float = 5.0) -> Ready:
        """Wait for the one-shot READY frame sent after boot."""

        deadline = time.monotonic() + float(timeout)
        with self._condition:
            while self._ready is None:
                if self._link_error is not None:
                    error = self._link_error
                    if isinstance(error, TransportError):
                        raise error
                    raise TransportError("link failed: %s" % error) from error
                if not self._opened:
                    raise TransportError("client is not open")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise ReadyTimeout("READY not received within %.3f s" % timeout)
                self._condition.wait(remaining)
            return self._ready

    def _wait_for_status(
        self,
        role: Link,
        predicate: Optional[Callable[[LinkStatus], bool]],
        timeout: float,
        label: str,
    ) -> LinkStatus:
        deadline = time.monotonic() + float(timeout)
        while True:
            with self._condition:
                if role == Link.PI:
                    status: Optional[LinkStatus] = self._gimbal_status
                else:
                    status = self._chassis_status
                error = self._link_error
                opened = self._opened

            if error is not None:
                if isinstance(error, TransportError):
                    raise error
                raise TransportError("link failed: %s" % error) from error
            if not opened:
                raise TransportError("client is not open")
            if status is not None and (predicate is None or predicate(status)):
                return status

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise StatusTimeout(
                    "matching %s not received within %.3f s" % (label, timeout)
                )
            with self._condition:
                self._condition.wait(min(remaining, 0.1))

    def wait_status(
        self,
        predicate: Optional[Callable[[LinkStatus], bool]] = None,
        timeout: float = 5.0,
    ) -> LinkStatus:
        """Wait for the status frame of this client's link."""

        return self._wait_for_status(self._link, predicate, timeout, "status")

    def wait_gimbal_status(
        self,
        predicate: Optional[Callable[[GimbalStatus], bool]] = None,
        timeout: float = 5.0,
    ) -> GimbalStatus:
        """Wait for GIMBAL_STATUS (normally only available on Link.PI)."""

        return self._wait_for_status(Link.PI, predicate, timeout, "GIMBAL_STATUS")  # type: ignore[return-value]

    def wait_chassis_status(
        self,
        predicate: Optional[Callable[[ChassisStatus], bool]] = None,
        timeout: float = 5.0,
    ) -> ChassisStatus:
        """Wait for CHASSIS_STATUS (normally only available on Link.BT)."""

        return self._wait_for_status(Link.BT, predicate, timeout, "CHASSIS_STATUS")  # type: ignore[return-value]

    def wait_motor_status(self, timeout: float = 5.0) -> MotorStatus:
        """Wait for the next MOTOR_STATUS frame."""

        deadline = time.monotonic() + float(timeout)
        with self._condition:
            while self._motor_status is None:
                if self._link_error is not None:
                    error = self._link_error
                    if isinstance(error, TransportError):
                        raise error
                    raise TransportError("link failed: %s" % error) from error
                if not self._opened:
                    raise TransportError("client is not open")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise StatusTimeout(
                        "MOTOR_STATUS not received within %.3f s" % timeout
                    )
                self._condition.wait(remaining)
            return self._motor_status

    @property
    def status(self) -> Optional[LinkStatus]:
        """Latest status frame of this client's own link."""

        with self._condition:
            if self._link == Link.PI:
                return self._gimbal_status
            return self._chassis_status

    @property
    def gimbal_status(self) -> Optional[GimbalStatus]:
        with self._condition:
            return self._gimbal_status

    @property
    def chassis_status(self) -> Optional[ChassisStatus]:
        with self._condition:
            return self._chassis_status

    @property
    def link(self) -> Link:
        """Physical endpoint this client drives."""

        return self._link

    @property
    def motor_status(self) -> Optional[MotorStatus]:
        with self._condition:
            return self._motor_status

    @property
    def ready(self) -> Optional[Ready]:
        with self._condition:
            return self._ready

    @property
    def link_error(self) -> Optional[BaseException]:
        with self._condition:
            return self._link_error

    @property
    def is_link_alive(self) -> bool:
        with self._condition:
            last_rx = self._last_rx
            error = self._link_error
        return error is None and last_rx > 0 and (time.monotonic() - last_rx) < 2.0
