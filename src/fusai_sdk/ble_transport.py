"""BLE GATT transport for the HC-05-compatible chassis module.

The module sold as "HC-05" with firmware hc05V2.3_le is a Bluetooth 4.0
BLE module, not a Classic SPP device.  It exposes the common HM-10 style
vendor service FFE0 with one FFE1 characteristic used for both
directions.  This module implements the same byte-stream contract as
fusai_sdk.transport.SerialTransport, so FusaiClient can use it
without knowing anything about GATT.

Only the transport is BLE-specific.  Frame encoding, CRC, sequence numbers and
motion arbitration stay in the protocol layer.  The STM32 firmware still sees
plain UART bytes on UART5.
"""

from __future__ import annotations

import asyncio
import concurrent.futures
import json
import os
import queue
import re
import subprocess
import sys
import tempfile
import threading
from dataclasses import dataclass, replace
from typing import Any, Dict, Iterable, List, Optional, Tuple

from .exceptions import TransportError

# HM-10 / hc05V2.3_le family.
VENDOR_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
VENDOR_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

# Nordic UART Service, used by several BLE serial modules.
NORDIC_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NORDIC_RX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
NORDIC_TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

_WRITE_PROPERTIES = ("write", "write-without-response")
_NOTIFY_PROPERTIES = ("notify", "indicate")
_ADDRESS_RE = re.compile(r"^[0-9A-Fa-f]{2}(?::[0-9A-Fa-f]{2}){5}$")
_INSTANCE_ADDRESS_RE = re.compile(r"DEV_([0-9A-Fa-f]{12})", re.IGNORECASE)


@dataclass(frozen=True)
class BleDevice:
    """One result from a BLE scan.

    address is the stable identity used to connect.  The advertised name
    can be empty or mojibake; scan_ble_devices fills it from Windows
    PnP when possible, and the GUI falls back to the address.
    """

    address: str
    name: str = ""
    rssi: Optional[int] = None
    connected: bool = False
    serial: bool = False

    @property
    def display_name(self) -> str:
        return self.name.strip() or self.address

    @property
    def label(self) -> str:
        if self.name.strip():
            return "%s [%s]" % (self.name.strip(), self.address)
        return self.address


@dataclass(frozen=True)
class GattEndpoint:
    """The two GATT characteristics used to move bytes in each direction."""

    service_uuid: str
    tx_uuid: str
    rx_uuid: str
    write_with_response: bool


def normalize_uuid(value: Any) -> str:
    """Return a lowercase 128-bit UUID string.

    Short 16-bit (ffe1) and 32-bit forms are expanded with the Bluetooth
    base UUID so callers can use the names printed by nRF Connect.
    """

    text = str(value).strip().lower()
    if not text:
        return text
    if len(text) == 4:
        return "0000%s-0000-1000-8000-00805f9b34fb" % text
    if len(text) == 8:
        return "%s-0000-1000-8000-00805f9b34fb" % text
    return text


def is_ble_address(value: str) -> bool:
    return bool(_ADDRESS_RE.match(value.strip()))


def _characteristic_properties(characteristic: Any) -> Tuple[str, ...]:
    properties = getattr(characteristic, "properties", ()) or ()
    return tuple(str(item).lower() for item in properties)


def _can_write(characteristic: Any) -> bool:
    return any(item in _characteristic_properties(characteristic) for item in _WRITE_PROPERTIES)


def _can_notify(characteristic: Any) -> bool:
    return any(item in _characteristic_properties(characteristic) for item in _NOTIFY_PROPERTIES)


def _write_with_response(characteristic: Any) -> bool:
    # Prefer write-without-response when the peer supports it: a 50 Hz command
    # stream would otherwise wait for one ATT response per 20-byte chunk.
    return "write-without-response" not in _characteristic_properties(characteristic)


def _endpoint(service: Any, tx: Any, rx: Any) -> GattEndpoint:
    return GattEndpoint(
        service_uuid=str(getattr(service, "uuid", "")),
        tx_uuid=str(getattr(tx, "uuid", "")),
        rx_uuid=str(getattr(rx, "uuid", "")),
        write_with_response=_write_with_response(tx),
    )


def _pairs(services: Iterable[Any]) -> List[Tuple[Any, Any]]:
    result: List[Tuple[Any, Any]] = []
    for service in services:
        for characteristic in getattr(service, "characteristics", ()) or ():
            result.append((service, characteristic))
    return result


def _same_uuid(left: Any, right: Any) -> bool:
    return normalize_uuid(left) == normalize_uuid(right)


def _choose_from_service(service: Any) -> Optional[GattEndpoint]:
    characteristics = list(getattr(service, "characteristics", ()) or ())
    writable = [item for item in characteristics if _can_write(item)]
    notifiable = [item for item in characteristics if _can_notify(item)]
    if not writable or not notifiable:
        return None

    # The usual FFE1 is both writable and notifiable.  Prefer one
    # characteristic for both directions when it exists.
    for characteristic in characteristics:
        if _can_write(characteristic) and _can_notify(characteristic):
            return _endpoint(service, characteristic, characteristic)

    return _endpoint(service, writable[0], notifiable[0])


def choose_gatt_endpoint(
    services: Iterable[Any],
    *,
    service_uuid: Optional[str] = None,
    tx_uuid: Optional[str] = None,
    rx_uuid: Optional[str] = None,
) -> GattEndpoint:
    """Pick the byte-stream characteristics from a discovered GATT tree.

    Selection order:

    1. explicit service/characteristic UUIDs,
    2. HM-10 vendor service FFE0 (FFE1 for both directions),
    3. Nordic UART Service,
    4. any service with one writable+notifiable characteristic,
    5. any service with a writable and a notifiable characteristic.

    The function works on bleak's service objects but only relies on uuid,
    characteristics and properties, which keeps it unit-testable.
    """

    pairs = _pairs(services)
    if not pairs:
        raise TransportError("BLE device exposes no GATT characteristics")

    if tx_uuid is not None or rx_uuid is not None or service_uuid is not None:
        candidates = pairs
        if service_uuid is not None:
            candidates = [
                (service, characteristic)
                for service, characteristic in candidates
                if _same_uuid(getattr(service, "uuid", ""), service_uuid)
            ]
        if not candidates:
            raise TransportError("BLE service %s was not found" % service_uuid)

        tx = None
        rx = None
        if tx_uuid is not None:
            for _, characteristic in candidates:
                if _same_uuid(getattr(characteristic, "uuid", ""), tx_uuid) and _can_write(characteristic):
                    tx = characteristic
                    break
            if tx is None:
                raise TransportError("BLE write characteristic %s was not found" % tx_uuid)
        if rx_uuid is not None:
            for _, characteristic in candidates:
                if _same_uuid(getattr(characteristic, "uuid", ""), rx_uuid) and _can_notify(characteristic):
                    rx = characteristic
                    break
            if rx is None:
                raise TransportError("BLE notify characteristic %s was not found" % rx_uuid)
        if tx is None and rx is not None and _can_write(rx):
            tx = rx
        if rx is None and tx is not None and _can_notify(tx):
            rx = tx
        if tx is None or rx is None:
            raise TransportError("specify both a writable and a notifiable BLE characteristic")
        service = next(
            service
            for service, characteristic in candidates
            if characteristic is tx or characteristic is rx
        )
        return _endpoint(service, tx, rx)

    ordered = list(services)

    # 1. HM-10 / hc05V2.3_le vendor service.
    for service in ordered:
        if _same_uuid(getattr(service, "uuid", ""), VENDOR_SERVICE_UUID):
            endpoint = _choose_from_service(service)
            if endpoint is not None:
                return endpoint

    # 2. Nordic UART Service has separate write and notify characteristics.
    for service in ordered:
        if _same_uuid(getattr(service, "uuid", ""), NORDIC_SERVICE_UUID):
            tx = next(
                (
                    item
                    for item in getattr(service, "characteristics", ()) or ()
                    if _same_uuid(getattr(item, "uuid", ""), NORDIC_RX_CHAR_UUID)
                ),
                None,
            )
            rx = next(
                (
                    item
                    for item in getattr(service, "characteristics", ()) or ()
                    if _same_uuid(getattr(item, "uuid", ""), NORDIC_TX_CHAR_UUID)
                ),
                None,
            )
            if tx is not None and rx is not None:
                return _endpoint(service, tx, rx)

    # 3. Same-characteristic serial services under other vendor UUIDs.
    for service in ordered:
        if _is_generic_service(service):
            continue
        endpoint = _choose_from_service(service)
        if endpoint is not None:
            return endpoint

    # 4. Last resort: a writable and a notifiable characteristic anywhere.
    writable_pair = next(((service, item) for service, item in pairs if _can_write(item)), None)
    notifiable_pair = next(((service, item) for service, item in pairs if _can_notify(item)), None)
    if writable_pair is not None and notifiable_pair is not None:
        service, characteristic = writable_pair
        return _endpoint(service, characteristic, notifiable_pair[1])

    raise TransportError("no writable + notifiable BLE characteristic found")


def _is_generic_service(service: Any) -> bool:
    return normalize_uuid(getattr(service, "uuid", "")) in (
        "00001800-0000-1000-8000-00805f9b34fb",
        "00001801-0000-1000-8000-00805f9b34fb",
    )


def _windows_known_devices() -> List[BleDevice]:
    """Return Windows PnP BLE devices, including ones that are not advertising.

    A connected BLE peripheral stops advertising, so BleakScanner cannot see it
    even though Windows still knows the device and can connect to it.  PnP is
    the only local inventory that keeps the address and user-visible name.
    """

    if sys.platform != "win32":
        return []
    fd, path = tempfile.mkstemp(prefix="fusai-pnp-", suffix=".json")
    os.close(fd)
    quoted = path.replace("'", "''")
    script = (
        "Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue | "
        "Where-Object { $_.InstanceId -like 'BTHLE\\DEV_*' } | "
        "Select-Object FriendlyName,InstanceId | ConvertTo-Json -Compress | "
        "Set-Content -LiteralPath '%s' -Encoding UTF8" % quoted
    )
    try:
        completed = subprocess.run(
            ["powershell", "-NoProfile", "-NonInteractive", "-Command", script],
            capture_output=True,
            timeout=6,
            check=False,
        )
        if completed.returncode != 0 or not os.path.exists(path):
            return []
        with open(path, "rb") as handle:
            raw = handle.read()
        text = raw.decode("utf-8-sig").strip()
        payload = json.loads(text) if text else []
    except Exception:
        return []
    finally:
        try:
            os.unlink(path)
        except OSError:
            pass
    if isinstance(payload, dict):
        payload = [payload]
    result: List[BleDevice] = []
    for item in payload:
        if not isinstance(item, dict):
            continue
        name = str(item.get("FriendlyName") or "").strip()
        instance = str(item.get("InstanceId") or "")
        match = _INSTANCE_ADDRESS_RE.search(instance)
        if not name or match is None:
            continue
        raw = match.group(1).upper()
        address = ":".join(raw[index : index + 2] for index in range(0, 12, 2))
        result.append(BleDevice(address=address, name=name))
    return result


def _windows_friendly_names() -> Dict[str, str]:
    """Map BLE addresses to Windows PnP names."""

    return {device.address: device.name for device in _windows_known_devices()}


def _device_address(device: Any) -> str:
    return str(getattr(device, "address", "") or "").strip().upper()


def _device_name(device: Any) -> str:
    return str(getattr(device, "name", "") or "").strip()


def _device_rssi(device: Any) -> Optional[int]:
    value = getattr(device, "rssi", None)
    try:
        return int(value) if value is not None else None
    except (TypeError, ValueError):
        return None


async def _mark_connected(devices: List[BleDevice]) -> List[BleDevice]:
    """Mark devices that Windows currently reports as connected.

    A connected peripheral is quiet and would otherwise be buried in the scan
    list.  WinRT exposes the live connection status without connecting again.
    """

    if sys.platform != "win32" or not devices:
        return devices
    try:
        from winrt.windows.devices.bluetooth import (
            BluetoothConnectionStatus,
            BluetoothLEDevice,
        )
    except Exception:  # pragma: no cover - optional dependency
        return devices

    serial_services = {
        normalize_uuid(VENDOR_SERVICE_UUID),
        normalize_uuid(NORDIC_SERVICE_UUID),
    }

    async def probe(device: BleDevice) -> BleDevice:
        try:
            raw = int(device.address.replace(":", ""), 16)
            native = await asyncio.wait_for(
                BluetoothLEDevice.from_bluetooth_address_async(raw), timeout=2.0
            )
            if native is None:
                return device
            connected = native.connection_status == BluetoothConnectionStatus.CONNECTED
            if not connected:
                return replace(device, connected=False)
            serial = False
            try:
                result = await asyncio.wait_for(native.get_gatt_services_async(), timeout=2.0)
                serial = any(
                    normalize_uuid(getattr(service, "uuid", "")) in serial_services
                    for service in (getattr(result, "services", ()) or ())
                )
            except Exception:
                pass
            return replace(device, connected=True, serial=serial)
        except Exception:
            return device

    return list(await asyncio.gather(*(probe(device) for device in devices)))


async def _scan_async(timeout: float) -> List[BleDevice]:
    try:
        from bleak import BleakScanner
    except ImportError as exc:  # pragma: no cover - optional dependency
        raise TransportError(
            "bleak is required for BLE: install fusai-sdk[ble] or run uv sync --extra ble"
        ) from exc

    discovered = await BleakScanner.discover(timeout=timeout)
    known = await _mark_connected(_windows_known_devices())
    friendly = {device.address: device.name for device in known if device.name}
    # Seed the list with PnP devices first.  A connected peripheral stops
    # advertising, but Windows still knows it and can reconnect by address.
    by_address: Dict[str, BleDevice] = {device.address: device for device in known}
    for raw_device in discovered:
        address = _device_address(raw_device)
        if not address:
            continue
        name = _device_name(raw_device)
        # PnP names are authoritative on Windows; advertised names win when
        # PnP has no entry (non-Windows or a device that is not present yet).
        name = friendly.get(address, name)
        candidate = BleDevice(address=address, name=name, rssi=_device_rssi(raw_device))
        previous = by_address.get(address)
        if previous is None or (not previous.name and candidate.name):
            by_address[address] = candidate
    return sorted(
        by_address.values(),
        key=lambda item: (
            not item.serial,
            not item.connected,
            not bool(item.name),
            -(item.rssi or -999),
            item.display_name.casefold(),
        ),
    )


def scan_ble_devices(timeout: float = 10.0) -> List[BleDevice]:
    """Scan nearby BLE devices, including devices that are not paired."""

    if timeout <= 0:
        raise ValueError("timeout must be positive")
    try:
        return asyncio.run(_scan_async(float(timeout)))
    except TransportError:
        raise
    except Exception as exc:
        raise TransportError("BLE scan failed: %s" % exc) from exc


class BleTransport:
    """A synchronous byte stream backed by one BLE GATT connection.

    A dedicated asyncio event-loop thread owns the BleakClient.  Reads come
    from a queue filled by GATT notifications, and writes are chunked to the
    default BLE payload size unless a larger chunk is configured.  The class is
    intentionally transport-only: reconnection and protocol recovery belong to
    FusaiClient / the caller.
    """

    def __init__(
        self,
        address: str,
        *,
        service_uuid: Optional[str] = None,
        tx_uuid: Optional[str] = None,
        rx_uuid: Optional[str] = None,
        read_timeout: float = 0.05,
        connect_timeout: float = 20.0,
        write_timeout: float = 5.0,
        chunk_size: int = 20,
        chunk_delay: float = 0.0,
    ) -> None:
        if not str(address).strip():
            raise ValueError("BLE address is required")
        if read_timeout <= 0:
            raise ValueError("read_timeout must be positive")
        if connect_timeout <= 0:
            raise ValueError("connect_timeout must be positive")
        if write_timeout <= 0:
            raise ValueError("write_timeout must be positive")
        if chunk_size <= 0:
            raise ValueError("chunk_size must be positive")
        if chunk_delay < 0:
            raise ValueError("chunk_delay cannot be negative")

        self.address = str(address).strip()
        self.service_uuid = normalize_uuid(service_uuid) if service_uuid else None
        self.tx_uuid = normalize_uuid(tx_uuid) if tx_uuid else None
        self.rx_uuid = normalize_uuid(rx_uuid) if rx_uuid else None
        self.read_timeout = float(read_timeout)
        self.connect_timeout = float(connect_timeout)
        self.write_timeout = float(write_timeout)
        self.chunk_size = int(chunk_size)
        self.chunk_delay = float(chunk_delay)

        self._lifecycle_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._read_lock = threading.Lock()
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._loop_thread: Optional[threading.Thread] = None
        self._client: Optional[Any] = None
        self._endpoint: Optional[GattEndpoint] = None
        self._rx_queue: "queue.Queue[Optional[bytes]]" = queue.Queue()
        self._read_buffer = bytearray()
        self._opened = False
        self._disconnected = threading.Event()
        self._last_error: Optional[str] = None

    @property
    def is_open(self) -> bool:
        return self._opened

    @property
    def endpoint(self) -> Optional[GattEndpoint]:
        return self._endpoint

    @property
    def last_error(self) -> Optional[str]:
        return self._last_error

    def _ensure_loop(self) -> asyncio.AbstractEventLoop:
        with self._lifecycle_lock:
            loop = self._loop
            thread = self._loop_thread
            if loop is not None and not loop.is_closed() and thread is not None and thread.is_alive():
                return loop
            loop = asyncio.new_event_loop()
            thread = threading.Thread(
                target=self._run_loop,
                args=(loop,),
                name="fusai-ble-loop",
                daemon=True,
            )
            self._loop = loop
            self._loop_thread = thread
            thread.start()
            return loop

    @staticmethod
    def _run_loop(loop: asyncio.AbstractEventLoop) -> None:
        asyncio.set_event_loop(loop)
        loop.run_forever()

    def _run_coroutine(self, coroutine: Any, timeout: float) -> Any:
        loop = self._ensure_loop()
        future = asyncio.run_coroutine_threadsafe(coroutine, loop)
        try:
            return future.result(timeout=timeout)
        except concurrent.futures.TimeoutError as exc:
            future.cancel()
            raise TransportError("BLE operation timed out after %.1f s" % timeout) from exc

    def open(self) -> None:
        if self.is_open:
            return
        self._disconnected.clear()
        self._last_error = None
        self._rx_queue = queue.Queue()
        self._read_buffer.clear()
        try:
            self._run_coroutine(self._open_async(), self.connect_timeout + 5.0)
        except TransportError:
            raise
        except Exception as exc:
            raise TransportError("cannot open BLE device %s: %s" % (self.address, exc)) from exc
        self._opened = True

    def _client_target(self) -> Any:
        """Return a Bleak target that works for a connected, quiet device.

        On Windows, passing an address makes Bleak scan first; a device that is
        already connected does not advertise, so that scan fails.  Passing a
        BLEDevice makes the WinRT backend call BluetoothLEDevice directly from
        the cached address, which works for both advertising and quiet devices.
        """

        if sys.platform == "win32":
            try:
                from bleak.backends.device import BLEDevice

                return BLEDevice(self.address, None, None)
            except Exception:
                pass
        return self.address

    async def _open_async(self) -> None:
        try:
            from bleak import BleakClient
        except ImportError as exc:  # pragma: no cover - optional dependency
            raise TransportError(
                "bleak is required for BLE: install fusai-sdk[ble] or run uv sync --extra ble"
            ) from exc

        client: Any = None
        try:
            client = BleakClient(
                self._client_target(),
                disconnected_callback=self._on_disconnected,
                timeout=self.connect_timeout,
            )
            await client.connect()
            if not getattr(client, "is_connected", False):
                raise TransportError("BLE connection to %s was not established" % self.address)
            services = list(getattr(client, "services", ()) or ())
            endpoint = choose_gatt_endpoint(
                services,
                service_uuid=self.service_uuid,
                tx_uuid=self.tx_uuid,
                rx_uuid=self.rx_uuid,
            )
            await client.start_notify(endpoint.rx_uuid, self._on_notify)
        except TransportError:
            await self._disconnect_quietly(client)
            raise
        except Exception as exc:
            await self._disconnect_quietly(client)
            raise TransportError("BLE setup failed: %s" % exc) from exc

        self._client = client
        self._endpoint = endpoint

    @staticmethod
    async def _disconnect_quietly(client: Any) -> None:
        if client is None or not getattr(client, "is_connected", False):
            return
        try:
            await client.disconnect()
        except Exception:
            pass

    def _on_notify(self, characteristic: Any, data: Any) -> None:
        chunk = bytes(data)
        if chunk:
            self._rx_queue.put(chunk)

    def _on_disconnected(self, client: Any) -> None:
        # A sentinel wakes a reader blocked on an empty queue.  The reader
        # turns it into a TransportError so FusaiClient reports a link error.
        self._disconnected.set()
        self._rx_queue.put(None)

    def read(self, size: int = 4096) -> bytes:
        if not self.is_open:
            raise TransportError("BLE transport is not open")
        if size <= 0:
            return b""
        with self._read_lock:
            if not self._read_buffer:
                timeout = self.read_timeout
                try:
                    item = self._rx_queue.get(timeout=timeout)
                except queue.Empty:
                    if self._disconnected.is_set():
                        raise TransportError(self._disconnect_message())
                    return b""
                if item is None:
                    raise TransportError(self._disconnect_message())
                self._read_buffer.extend(item)
                while len(self._read_buffer) < size:
                    try:
                        extra = self._rx_queue.get_nowait()
                    except queue.Empty:
                        break
                    if extra is None:
                        break
                    self._read_buffer.extend(extra)
            result = bytes(self._read_buffer[:size])
            del self._read_buffer[:size]
            return result

    def _disconnect_message(self) -> str:
        if self._last_error:
            return "BLE link disconnected: %s" % self._last_error
        return "BLE link disconnected"

    def write(self, data: bytes) -> None:
        if not self.is_open:
            raise TransportError("BLE transport is not open")
        payload = bytes(data)
        if not payload:
            return
        with self._write_lock:
            chunks = (len(payload) + self.chunk_size - 1) // self.chunk_size
            # write_timeout covers the first ATT write; later chunks only add
            # a small grace period instead of multiplying the whole timeout.
            timeout = self.write_timeout + 0.1 * (chunks - 1)
            try:
                self._run_coroutine(self._write_async(payload), timeout)
            except TransportError:
                raise
            except Exception as exc:
                raise TransportError("BLE write failed: %s" % exc) from exc

    async def _write_async(self, data: bytes) -> None:
        client = self._client
        endpoint = self._endpoint
        if client is None or endpoint is None:
            raise TransportError("BLE transport is not open")
        for offset in range(0, len(data), self.chunk_size):
            chunk = data[offset : offset + self.chunk_size]
            await client.write_gatt_char(
                endpoint.tx_uuid,
                chunk,
                response=endpoint.write_with_response,
            )
            if self.chunk_delay:
                await asyncio.sleep(self.chunk_delay)

    def close(self) -> None:
        with self._lifecycle_lock:
            opened = self._opened
            loop = self._loop
            thread = self._loop_thread
            self._opened = False
            self._disconnected.set()
            self._rx_queue.put(None)

        if opened and loop is not None and not loop.is_closed():
            try:
                self._run_coroutine(self._close_async(), 8.0)
            except Exception:
                pass

        with self._lifecycle_lock:
            self._loop = None
            self._loop_thread = None
        if loop is not None and not loop.is_closed():
            try:
                loop.call_soon_threadsafe(loop.stop)
            except RuntimeError:
                pass
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        if loop is not None and not loop.is_closed():
            try:
                loop.close()
            except Exception:
                pass
        self._client = None
        self._endpoint = None

    async def _close_async(self) -> None:
        client = self._client
        endpoint = self._endpoint
        self._client = None
        self._endpoint = None
        if client is None:
            return
        try:
            if endpoint is not None and getattr(client, "is_connected", False):
                await client.stop_notify(endpoint.rx_uuid)
        except Exception:
            pass
        try:
            if getattr(client, "is_connected", False):
                await client.disconnect()
        except Exception:
            pass

    def __enter__(self) -> "BleTransport":
        self.open()
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()
