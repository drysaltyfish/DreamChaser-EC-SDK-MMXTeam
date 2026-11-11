import pytest

from fusai_sdk.crc import crc16_modbus, crc16_modbus_update


def test_modbus_check_value() -> None:
    assert crc16_modbus(b"123456789") == 0x4B37


def test_empty_input_is_initial_value() -> None:
    assert crc16_modbus(b"") == 0xFFFF


def test_update_matches_one_shot() -> None:
    part = b"hello "
    crc = crc16_modbus_update(0xFFFF, part)
    assert crc16_modbus_update(crc, b"world") == crc16_modbus(part + b"world")


def test_rejects_non_byte_input() -> None:
    with pytest.raises(ValueError):
        crc16_modbus(b"ok", crc=0x10000)
