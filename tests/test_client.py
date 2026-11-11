import queue
import struct
import threading
import time
from typing import Callable, List, Optional

import pytest

from fusai_sdk.client import FusaiClient
from fusai_sdk.constants import (
    AckFlags,
    AckMotionFlags,
    Command,
    Link,
    Mode,
    Owner,
    StatusFlags,
)
from fusai_sdk.exceptions import (
    BusyError,
    CommandTimeout,
    EmergencyStop,
    MotionRejected,
    SequenceLostError,
    SequenceUnknownError,
    WrongLinkError,
)
from fusai_sdk.framing import Frame, FrameDecoder, encode_frame
from fusai_sdk.messages import (
    decode_ready,
    encode_heartbeat,
)


class FakeTransport:
    """In-memory byte transport with a write callback for a fake server."""

    def __init__(self) -> None:
        self._inbound: "queue.Queue[bytes]" = queue.Queue()
        self._lock = threading.Lock()
        self._closed = False
        self.opened = False
        self.writes: List[bytes] = []
        self.on_write: Optional[Callable[[bytes], None]] = None

    def open(self) -> None:
        self.opened = True
        self._closed = False

    def close(self) -> None:
        self._closed = True
        self._inbound.put(b"")

    def feed(self, data: bytes) -> None:
        self._inbound.put(bytes(data))

    def read(self, size: int) -> bytes:
        if self._closed:
            return b""
        try:
            return self._inbound.get(timeout=0.05)
        except queue.Empty:
            return b""

    def write(self, data: bytes) -> None:
        with self._lock:
            self.writes.append(bytes(data))
        if self.on_write is not None:
            self.on_write(bytes(data))

    def frames(self) -> List[Frame]:
        decoder = FrameDecoder()
        with self._lock:
            writes = list(self.writes)
        frames: List[Frame] = []
        for data in writes:
            frames.extend(decoder.feed_bytes(data))
        return frames


def make_ack(command: int, seq: int, flags: int = int(AckFlags.ACCEPTED)) -> bytes:
    return encode_frame(Command.ACK, struct.pack("<BBB", int(command), seq, flags))


def make_ack_motion(
    seq: int,
    motion_state: int,
    flags: int = int(AckMotionFlags.ACCEPTED),
) -> bytes:
    return encode_frame(
        Command.ACK_MOTION,
        struct.pack("<BBB", seq, motion_state, flags),
    )


def make_busy(command: int, seq: int, owner: int = int(Owner.CHASSIS)) -> bytes:
    return encode_frame(Command.NACK_BUSY, struct.pack("<BBB", int(command), seq, owner))


def make_seq_lost(command: int, expected: int) -> bytes:
    return encode_frame(Command.NACK_SEQ_LOST, struct.pack("<BB", int(command), expected))


def make_motor_status(mask: int, rpm: tuple[int, int, int, int]) -> bytes:
    return encode_frame(Command.MOTOR_STATUS, struct.pack("<Bhhhh", mask, *rpm))


def open_client(transport: FakeTransport, **kwargs: object) -> FusaiClient:
    client = FusaiClient(transport=transport, auto_heartbeat=False, **kwargs)
    client.open()
    transport.writes.clear()
    return client


def close_client(client: FusaiClient) -> None:
    client.close()


def test_initial_heartbeat_and_frame_encoding() -> None:
    transport = FakeTransport()
    client = FusaiClient(transport=transport, auto_heartbeat=False)
    client.open()
    frames = transport.frames()
    assert frames
    assert frames[0].cmd == Command.HEARTBEAT
    assert transport.frames()[0].data == b""
    close_client(client)


def test_set_target_advances_gimbal_sequence_only_after_ack() -> None:
    transport = FakeTransport()
    client = open_client(transport)
    seen = []

    def server(data: bytes) -> None:
        for frame in FrameDecoder().feed_bytes(data):
            if frame.cmd == Command.SET_TARGET:
                seq, coord, pan, tilt = struct.unpack("<BBhh", frame.data)
                seen.append((seq, coord, pan, tilt))
                transport.feed(make_ack(Command.SET_TARGET, seq))

    transport.on_write = server
    first = client.set_target(100, -50)
    second = client.set_target(200, 300)

    assert first.accepted and second.accepted
    assert [item[0] for item in seen] == [0, 1]
    assert seen[0][1] == 2
    assert seen[0][2:] == (100, -50)
    close_client(client)


def test_busy_reply_keeps_candidate_sequence() -> None:
    transport = FakeTransport()
    client = open_client(transport)
    sequences = []

    def server(data: bytes) -> None:
        for frame in FrameDecoder().feed_bytes(data):
            if frame.cmd != Command.SET_TARGET:
                continue
            seq = struct.unpack("<BBhh", frame.data)[0]
            sequences.append(seq)
            if len(sequences) == 1:
                transport.feed(make_busy(Command.SET_TARGET, seq))
            else:
                transport.feed(make_ack(Command.SET_TARGET, seq))

    transport.on_write = server
    with pytest.raises(BusyError):
        client.set_target(1, 2)
    assert client.set_target(3, 4).accepted
    assert sequences == [0, 0]
    close_client(client)


def test_seq_lost_resynchronizes_to_expected_sequence() -> None:
    transport = FakeTransport()
    client = open_client(transport)
    sequences = []

    def server(data: bytes) -> None:
        for frame in FrameDecoder().feed_bytes(data):
            if frame.cmd != Command.SET_TARGET:
                continue
            seq = struct.unpack("<BBhh", frame.data)[0]
            sequences.append(seq)
            if seq == 0:
                transport.feed(make_seq_lost(Command.SET_TARGET, 7))
            else:
                transport.feed(make_ack(Command.SET_TARGET, seq))

    transport.on_write = server
    assert client.set_target(1, 2).accepted
    assert sequences == [0, 7]
    close_client(client)


def test_seq_lost_without_resync_raises() -> None:
    transport = FakeTransport()
    client = open_client(transport, resync_on_seq_lost=False)

    def server(data: bytes) -> None:
        for frame in FrameDecoder().feed_bytes(data):
            if frame.cmd == Command.SET_TARGET:
                seq = struct.unpack("<BBhh", frame.data)[0]
                transport.feed(make_seq_lost(Command.SET_TARGET, 9))

    transport.on_write = server
    with pytest.raises(SequenceLostError):
        client.set_target(1, 2)
    close_client(client)


def test_command_timeout_blocks_gimbal_until_track_mode() -> None:
    transport = FakeTransport()
    client = open_client(transport, command_timeout=0.05, retries=0)

    with pytest.raises(CommandTimeout):
        client.set_target(1, 2)
    with pytest.raises(SequenceUnknownError):
        client.set_target(3, 4)

    def server(data: bytes) -> None:
        for frame in FrameDecoder().feed_bytes(data):
            if frame.cmd == Command.SET_MODE:
                transport.feed(make_ack(Command.SET_MODE, 0))
            elif frame.cmd == Command.SET_TARGET:
                seq = struct.unpack("<BBhh", frame.data)[0]
                transport.feed(make_ack(Command.SET_TARGET, seq))

    transport.on_write = server
    client.track_gimbal()
    assert client.set_target(5, 6).accepted
    close_client(client)


def test_velocity_timeout_uses_new_sequence_for_safe_stop() -> None:
    transport = FakeTransport()
    client = open_client(
        transport, link=Link.BT, command_timeout=0.05, retries=0
    )
    sequences = []
    respond = [False]

    def server(data: bytes) -> None:
        for frame in FrameDecoder().feed_bytes(data):
            if frame.cmd != Command.SET_VELOCITY:
                continue
            seq, vx, vy, omega = struct.unpack("<Bhhh", frame.data)
            sequences.append((seq, vx, vy, omega))
            if respond[0]:
                transport.feed(make_ack_motion(seq, 0, int(AckMotionFlags.ACCEPTED | AckMotionFlags.STOPPED)))

    transport.on_write = server
    with pytest.raises(CommandTimeout):
        client.set_velocity(100, 0, 0)
    with pytest.raises(SequenceUnknownError):
        client.set_velocity(200, 0, 0)

    respond[0] = True
    stop = client.stop_chassis()
    assert stop.accepted
    assert stop.stopped
    assert sequences[0][0] == 0
    assert sequences[1][0] == 1
    assert sequences[1][1:] == (0, 0, 0)
    close_client(client)


def test_busy_velocity_reply_raises_motion_rejected() -> None:
    transport = FakeTransport()
    client = open_client(transport, link=Link.BT)

    def server(data: bytes) -> None:
        for frame in FrameDecoder().feed_bytes(data):
            if frame.cmd == Command.SET_VELOCITY:
                seq = struct.unpack("<Bhhh", frame.data)[0]
                transport.feed(make_ack_motion(seq, 5, 0))

    transport.on_write = server
    with pytest.raises(MotionRejected):
        client.set_velocity(100, 0, 0)
    close_client(client)


def test_emergency_stop_is_fire_and_forget_without_ack() -> None:
    transport = FakeTransport()
    client = open_client(transport)

    client.emergency_stop()
    frames = transport.frames()
    assert frames[-1].cmd == Command.SET_MODE
    assert frames[-1].data == bytes([int(Mode.EMERGENCY_STOP)])
    close_client(client)


def test_emergency_stop_cancels_pending_command_and_blocks_motion() -> None:
    transport = FakeTransport()
    client = open_client(transport, command_timeout=2.0, retries=0)
    command_seen = threading.Event()

    def server(data: bytes) -> None:
        for frame in FrameDecoder().feed_bytes(data):
            if frame.cmd == Command.SET_TARGET:
                command_seen.set()

    transport.on_write = server
    errors: List[BaseException] = []

    def move() -> None:
        try:
            client.set_target(1, 2)
        except BaseException as exc:  # noqa: BLE001 - captured for the assertion
            errors.append(exc)

    worker = threading.Thread(target=move)
    worker.start()
    assert command_seen.wait(1.0), "SET_TARGET was not written"

    client.emergency_stop()
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert len(errors) == 1
    assert isinstance(errors[0], EmergencyStop)
    with pytest.raises(SequenceUnknownError):
        client.set_target(3, 4)
    assert transport.frames()[-1].data == bytes([int(Mode.EMERGENCY_STOP)])
    close_client(client)


def test_reopen_resets_sequence_and_cached_state() -> None:
    transport = FakeTransport()
    client = open_client(transport)
    sequences = []

    def server(data: bytes) -> None:
        for frame in FrameDecoder().feed_bytes(data):
            if frame.cmd == Command.SET_TARGET:
                seq = struct.unpack("<BBhh", frame.data)[0]
                sequences.append(seq)
                transport.feed(make_ack(Command.SET_TARGET, seq))

    transport.on_write = server
    client.set_target(1, 2)
    client.close()
    client.open()
    transport.writes.clear()
    client.set_target(3, 4)
    assert sequences == [0, 0]
    close_client(client)


def test_status_and_ready_are_delivered_to_waiters_and_callbacks() -> None:
    transport = FakeTransport()
    statuses = []
    readies = []
    client = FusaiClient(
        transport=transport,
        auto_heartbeat=False,
        on_status=statuses.append,
        on_ready=readies.append,
    )
    client.open()
    transport.writes.clear()

    status_payload = struct.pack(
        "<BBhhHB", 1, 1, 123, -45, 3, int(StatusFlags.PI_HEARTBEAT_OK)
    )
    ready_payload = struct.pack("<BBB", 2, 0, 0x07)
    transport.feed(encode_frame(Command.READY, ready_payload))
    transport.feed(encode_frame(Command.GIMBAL_STATUS, status_payload))

    ready = client.wait_ready(timeout=1.0)
    status = client.wait_status(lambda value: value.pan == 123, timeout=1.0)
    assert ready.version == (2, 0)
    assert ready.has_motor_telemetry
    assert status.tilt == -45
    assert status.pi_heartbeat_ok
    assert client.status is status
    assert statuses and statuses[-1].pan == 123
    assert readies and readies[-1].version == (2, 0)
    close_client(client)


def test_motor_status_is_delivered_to_waiter_and_callback() -> None:
    transport = FakeTransport()
    motor_statuses = []
    client = FusaiClient(
        transport=transport,
        auto_heartbeat=False,
        on_motor_status=motor_statuses.append,
    )
    client.open()
    transport.writes.clear()

    transport.feed(make_motor_status(0x01, (120, -45, 0, 0)))

    status = client.wait_motor_status(timeout=1.0)
    assert status.rpm == (120, -45, 0, 0)
    assert status.present(0)
    assert not status.present(3)
    assert motor_statuses and motor_statuses[-1].rpm == (120, -45, 0, 0)
    assert client.motor_status == status
    close_client(client)


def test_set_mode_rejects_reserved_value() -> None:
    transport = FakeTransport()
    client = open_client(transport)
    with pytest.raises(ValueError):
        client.set_mode(2)
    close_client(client)


def test_chassis_status_is_delivered_on_bt_client() -> None:
    transport = FakeTransport()
    statuses = []
    chassis_statuses = []
    client = FusaiClient(
        transport=transport,
        link=Link.BT,
        auto_heartbeat=False,
        on_status=statuses.append,
        on_chassis_status=chassis_statuses.append,
    )
    client.open()
    transport.writes.clear()

    payload = struct.pack(
        "<BBhhhHB",
        0,
        0,
        0,
        0,
        0,
        0,
        int(StatusFlags.BT_HEARTBEAT_OK | StatusFlags.CHASSIS_STOPPED),
    )
    transport.feed(encode_frame(Command.CHASSIS_STATUS, payload))

    status = client.wait_chassis_status(timeout=1.0)
    assert status.motion_state_name == "STOPPED"
    assert status.bt_heartbeat_ok
    assert status.chassis_stopped
    assert client.status is status
    assert client.gimbal_status is None
    assert statuses and statuses[-1] is status
    assert chassis_statuses and chassis_statuses[-1] is status
    close_client(client)


def test_gimbal_commands_are_rejected_on_the_bt_link() -> None:
    transport = FakeTransport()
    client = open_client(transport, link=Link.BT)

    with pytest.raises(WrongLinkError):
        client.set_target(10, 20)
    with pytest.raises(WrongLinkError):
        client.set_delta(1, 2)
    with pytest.raises(WrongLinkError):
        client.set_target_stream([(0, 0)])
    with pytest.raises(WrongLinkError):
        client.track_gimbal()
    assert transport.frames() == []
    close_client(client)


def test_chassis_commands_are_rejected_on_the_pi_link() -> None:
    transport = FakeTransport()
    client = open_client(transport, link=Link.PI)

    with pytest.raises(WrongLinkError):
        client.set_velocity(100, 0, 0)
    with pytest.raises(WrongLinkError):
        client.stop_chassis()
    assert transport.frames() == []
    close_client(client)


def test_mode_one_is_rejected_on_the_bt_link() -> None:
    transport = FakeTransport()
    client = open_client(transport, link=Link.BT)

    with pytest.raises(WrongLinkError):
        client.set_mode(Mode.TRACK_GIMBAL)
    assert transport.frames() == []
    close_client(client)


def test_release_is_allowed_on_both_links() -> None:
    for link in (Link.PI, Link.BT):
        transport = FakeTransport()
        client = open_client(transport, link=link)

        def server(data: bytes) -> None:
            for frame in FrameDecoder().feed_bytes(data):
                if frame.cmd == Command.SET_MODE:
                    transport.feed(make_ack(Command.SET_MODE, 0))

        transport.on_write = server
        assert client.release().accepted
        assert transport.frames()[-1].data == bytes([int(Mode.RELEASE)])
        close_client(client)
