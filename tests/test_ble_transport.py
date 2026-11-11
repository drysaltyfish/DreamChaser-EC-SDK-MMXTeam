"""Unit tests for BLE GATT endpoint selection and transport guards."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Optional

import pytest

from fusai_sdk import (
    BleDevice,
    BleTransport,
    GattEndpoint,
    TransportError,
    VENDOR_SERVICE_UUID,
    choose_gatt_endpoint,
    is_ble_address,
    normalize_uuid,
)


@dataclass
class FakeCharacteristic:
    uuid: str
    properties: List[str]
    handle: int = 1


@dataclass
class FakeService:
    uuid: str
    characteristics: List[FakeCharacteristic]


def service(uuid: str, *characteristics: FakeCharacteristic) -> FakeService:
    return FakeService(uuid=uuid, characteristics=list(characteristics))


def test_normalize_uuid_expands_short_forms() -> None:
    assert normalize_uuid("FFE1") == "0000ffe1-0000-1000-8000-00805f9b34fb"
    assert normalize_uuid("0000FFE1") == "0000ffe1-0000-1000-8000-00805f9b34fb"
    assert normalize_uuid("6E400001-B5A3-F393-E0A9-E50E24DCCA9E") == (
        "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
    )


def test_is_ble_address() -> None:
    assert is_ble_address("F0:72:D1:80:FB:EB")
    assert is_ble_address("f0:72:d1:80:fb:eb")
    assert not is_ble_address("COM5")
    assert not is_ble_address("F0:72:D1:80:FB")


def test_ble_device_label_falls_back_to_address() -> None:
    named = BleDevice("F0:72:D1:80:FB:EB", "原神？启动！", -40)
    assert named.display_name == "原神？启动！"
    assert named.label == "原神？启动！ [F0:72:D1:80:FB:EB]"
    unnamed = BleDevice("F0:72:D1:80:FB:EB")
    assert unnamed.display_name == "F0:72:D1:80:FB:EB"
    assert unnamed.label == "F0:72:D1:80:FB:EB"


def test_vendor_ffe1_is_used_for_both_directions() -> None:
    ffe1 = FakeCharacteristic("0000ffe1-0000-1000-8000-00805f9b34fb", ["write", "notify"])
    ffe2 = FakeCharacteristic("0000ffe2-0000-1000-8000-00805f9b34fb", ["write"])
    endpoint = choose_gatt_endpoint([service(VENDOR_SERVICE_UUID, ffe1, ffe2)])
    assert endpoint.tx_uuid == "0000ffe1-0000-1000-8000-00805f9b34fb"
    assert endpoint.rx_uuid == endpoint.tx_uuid
    # FFE1 advertises write (with response), not write-without-response.
    assert endpoint.write_with_response is True


def test_vendor_ffE1_prefers_write_without_response() -> None:
    ffe1 = FakeCharacteristic(
        "0000ffe1-0000-1000-8000-00805f9b34fb",
        ["write-without-response", "notify"],
    )
    endpoint = choose_gatt_endpoint([service(VENDOR_SERVICE_UUID, ffe1)])
    assert endpoint.write_with_response is False


def test_vendor_ffE1_split_write_and_notify() -> None:
    ffe1 = FakeCharacteristic("0000ffe1-0000-1000-8000-00805f9b34fb", ["notify"])
    ffe2 = FakeCharacteristic("0000ffe2-0000-1000-8000-00805f9b34fb", ["write"])
    endpoint = choose_gatt_endpoint([service(VENDOR_SERVICE_UUID, ffe1, ffe2)])
    assert endpoint.tx_uuid == "0000ffe2-0000-1000-8000-00805f9b34fb"
    assert endpoint.rx_uuid == "0000ffe1-0000-1000-8000-00805f9b34fb"


def test_nordic_uart_service() -> None:
    rx = FakeCharacteristic("6e400002-b5a3-f393-e0a9-e50e24dcca9e", ["write"])
    tx = FakeCharacteristic("6e400003-b5a3-f393-e0a9-e50e24dcca9e", ["notify"])
    endpoint = choose_gatt_endpoint(
        [
            service("00001800-0000-1000-8000-00805f9b34fb"),
            service("6e400001-b5a3-f393-e0a9-e50e24dcca9e", rx, tx),
        ]
    )
    assert endpoint.tx_uuid == rx.uuid
    assert endpoint.rx_uuid == tx.uuid


def test_generic_service_ignores_gap_and_gatt() -> None:
    gap = service(
        "00001800-0000-1000-8000-00805f9b34fb",
        FakeCharacteristic("00002a00-0000-1000-8000-00805f9b34fb", ["read"]),
    )
    vendor = service(
        "12345678-1234-5678-1234-56789abcdef0",
        FakeCharacteristic(
            "12345678-1234-5678-1234-56789abcdef1",
            ["write-without-response", "notify"],
        ),
    )
    endpoint = choose_gatt_endpoint([gap, vendor])
    assert endpoint.service_uuid == vendor.uuid
    assert endpoint.tx_uuid == "12345678-1234-5678-1234-56789abcdef1"


def test_explicit_characteristic_selection() -> None:
    ffe1 = FakeCharacteristic("0000ffe1-0000-1000-8000-00805f9b34fb", ["write", "notify"])
    endpoint = choose_gatt_endpoint(
        [service(VENDOR_SERVICE_UUID, ffe1)],
        tx_uuid="ffe1",
        rx_uuid="ffe1",
    )
    assert endpoint.tx_uuid == ffe1.uuid
    assert endpoint.rx_uuid == ffe1.uuid


def test_explicit_missing_characteristic_is_an_error() -> None:
    ffe1 = FakeCharacteristic("0000ffe1-0000-1000-8000-00805f9b34fb", ["write", "notify"])
    with pytest.raises(TransportError, match="notify characteristic"):
        choose_gatt_endpoint(
            [service(VENDOR_SERVICE_UUID, ffe1)],
            rx_uuid="fff1",
        )


def test_empty_gatt_tree_is_an_error() -> None:
    with pytest.raises(TransportError, match="no GATT characteristics"):
        choose_gatt_endpoint([])


def test_transport_rejects_invalid_parameters() -> None:
    with pytest.raises(ValueError):
        BleTransport("")
    with pytest.raises(ValueError):
        BleTransport("F0:72:D1:80:FB:EB", read_timeout=0)
    with pytest.raises(ValueError):
        BleTransport("F0:72:D1:80:FB:EB", chunk_size=0)


def test_transport_read_before_open_fails() -> None:
    transport = BleTransport("F0:72:D1:80:FB:EB")
    assert transport.is_open is False
    with pytest.raises(TransportError, match="not open"):
        transport.read()
    transport.close()


def test_windows_client_target_bypasses_scan(monkeypatch: pytest.MonkeyPatch) -> None:
    import fusai_sdk.ble_transport as ble_transport

    transport = BleTransport("F0:72:D1:80:FB:EB")
    monkeypatch.setattr(ble_transport.sys, "platform", "win32")
    target = transport._client_target()
    assert getattr(target, "address", None) == "F0:72:D1:80:FB:EB"

    monkeypatch.setattr(ble_transport.sys, "platform", "linux")
    assert transport._client_target() == "F0:72:D1:80:FB:EB"


def test_ble_device_connection_flags_default_to_false() -> None:
    device = BleDevice("F0:72:D1:80:FB:EB", "HC-05")
    assert device.connected is False
    assert device.serial is False
