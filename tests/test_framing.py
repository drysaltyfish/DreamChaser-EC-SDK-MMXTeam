import pytest

from fusai_sdk.constants import Command, MAX_DATA_LEN
from fusai_sdk.framing import Frame, FrameDecoder, encode_frame


HEARTBEAT_WIRE = bytes.fromhex("AA 55 01 10 01 EC 0D")
SET_MODE_1_WIRE = bytes.fromhex("AA 55 02 04 01 13 00 0D")
SET_MODE_3_WIRE = bytes.fromhex("AA 55 02 04 03 92 C1 0D")
SET_DELTA_WIRE = bytes.fromhex("AA 55 06 02 01 10 00 20 00 5C 22 0D")
SET_TARGET_WIRE = bytes.fromhex("AA 55 07 01 02 02 10 00 20 00 74 0F 0D")


def decode_one(wire: bytes) -> Frame:
    frames = list(FrameDecoder().feed_bytes(wire))
    assert len(frames) == 1
    return frames[0]


def test_reference_heartbeat_frame() -> None:
    assert encode_frame(Command.HEARTBEAT, b"") == HEARTBEAT_WIRE


def test_reference_set_mode_frames() -> None:
    assert encode_frame(Command.SET_MODE, bytes.fromhex("01")) == SET_MODE_1_WIRE
    assert encode_frame(Command.SET_MODE, bytes.fromhex("03")) == SET_MODE_3_WIRE


def test_reference_set_delta_frame() -> None:
    data = bytes.fromhex("01 10 00 20 00")
    assert encode_frame(Command.SET_DELTA, data) == SET_DELTA_WIRE


def test_reference_set_target_frame() -> None:
    data = bytes.fromhex("02 02 10 00 20 00")
    assert encode_frame(Command.SET_TARGET, data) == SET_TARGET_WIRE


def test_decode_reference_frames() -> None:
    heartbeat = decode_one(HEARTBEAT_WIRE)
    assert heartbeat.cmd == Command.HEARTBEAT
    assert heartbeat.data == b""
    assert heartbeat.length == 1

    target = decode_one(SET_TARGET_WIRE)
    assert target.cmd == Command.SET_TARGET
    assert target.data == bytes.fromhex("02 02 10 00 20 00")
    assert target.data_len == 6


def test_escapes_aa_in_every_body_position() -> None:
    data = bytes.fromhex("AA 01 AA 02 AA")
    wire = encode_frame(0x42, data)
    assert bytes.fromhex("AA 00") in wire
    assert wire.count(0xAA) == 4
    assert decode_one(wire) == Frame(cmd=0x42, data=data)


def test_eof_byte_inside_payload_is_not_special() -> None:
    data = bytes.fromhex("0D 0D 0D")
    assert decode_one(encode_frame(0x42, data)) == Frame(cmd=0x42, data=data)


def test_chunked_feed_and_multiple_frames() -> None:
    decoder = FrameDecoder()
    frames = []
    wire = HEARTBEAT_WIRE + SET_TARGET_WIRE
    for byte in wire:
        frame = decoder.feed(byte)
        if frame is not None:
            frames.append(frame)
    assert [frame.cmd for frame in frames] == [Command.HEARTBEAT, Command.SET_TARGET]
    assert decoder.frames_decoded == 2


def test_aa55_inside_body_restarts_at_length() -> None:
    stream = bytes.fromhex("AA 55 03 01 00 AA 55 01 10 01 EC 0D")
    decoder = FrameDecoder()
    frames = list(decoder.feed_bytes(stream))
    assert len(frames) == 1
    assert frames[0].cmd == Command.HEARTBEAT
    assert decoder.dropped_frames == 1


def test_invalid_length_is_format_error() -> None:
    decoder = FrameDecoder()
    assert list(decoder.feed_bytes(bytes.fromhex("AA 55 00"))) == []
    assert decoder.format_errors == 1
    assert decoder.frames_decoded == 0


def test_invalid_eof_is_format_error() -> None:
    bad = bytearray(HEARTBEAT_WIRE)
    bad[-1] = 0x00
    decoder = FrameDecoder()
    assert list(decoder.feed_bytes(bytes(bad))) == []
    assert decoder.format_errors == 1


def test_bad_crc_is_counted_and_alarm_latches_after_five() -> None:
    bad = bytearray(HEARTBEAT_WIRE)
    bad[5] ^= 0xFF
    decoder = FrameDecoder()
    for _ in range(5):
        assert list(decoder.feed_bytes(bytes(bad))) == []
    assert decoder.crc_errors == 5
    assert decoder.consecutive_crc_errors == 5
    assert decoder.crc_alarm is True


def test_valid_frame_clears_consecutive_crc_errors() -> None:
    bad = bytearray(HEARTBEAT_WIRE)
    bad[5] ^= 0xFF
    decoder = FrameDecoder()
    for _ in range(4):
        list(decoder.feed_bytes(bytes(bad)))
    assert decoder.crc_alarm is False
    assert list(decoder.feed_bytes(HEARTBEAT_WIRE))[0].cmd == Command.HEARTBEAT
    assert decoder.consecutive_crc_errors == 0
    assert decoder.crc_alarm is False


def test_reset_clears_counters() -> None:
    decoder = FrameDecoder()
    list(decoder.feed_bytes(HEARTBEAT_WIRE))
    decoder.reset()
    assert decoder.frames_decoded == 0
    assert decoder.bytes_fed == 0
    assert decoder.crc_alarm is False


def test_encode_rejects_oversized_payload() -> None:
    with pytest.raises(ValueError):
        encode_frame(0x01, b"x" * (MAX_DATA_LEN + 1))
