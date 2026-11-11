"""CRC-16/MODBUS used by the Fusai wire protocol.

Polynomial 0xA001 (reflected 0x8005), initial value 0xFFFF, no final xor.
"""

from __future__ import annotations


def crc16_modbus(data: bytes, crc: int = 0xFFFF) -> int:
    """Return CRC-16/MODBUS for data, optionally continuing from crc."""

    if not isinstance(crc, int) or not 0 <= crc <= 0xFFFF:
        raise ValueError("crc must be an integer in 0..65535")
    value = crc
    for byte in data:
        value ^= byte
        for _ in range(8):
            if value & 0x0001:
                value = (value >> 1) ^ 0xA001
            else:
                value >>= 1
    return value & 0xFFFF


def crc16_modbus_update(crc: int, data: bytes) -> int:
    """Continue a CRC-16/MODBUS computation from an existing value."""

    return crc16_modbus(data, crc)
