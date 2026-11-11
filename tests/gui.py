"""Fusai v2.0 蓝牙底盘测试台（Windows / Tkinter）。

运行（在 sdk/ 目录下）：
    uv run python tests/gui.py
    uv run python tests/gui.py --port F0:72:D1:80:FB:EB

这个工具只用于台架联调：它通过 sdk/ 中的 FusaiClient 在蓝牙链路（BLE / UART5）
上发送底盘命令。固件使用的蓝牙模块是 hc05V2.3_le（BLE 4.0，无 SPP），
因此连接使用 BLE GATT（FFE0/FFE1）而不是经典蓝牙串口。
云台由树莓派链路（USART3）控制，不在此界面中。
不会烧录固件，也不会自动清除真实急停。
"""

from __future__ import annotations

import argparse
import csv
import queue
import struct
import threading
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from tkinter import filedialog, messagebox, ttk
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple

from fusai_sdk import (
    Ack,
    AckFlags,
    AckMotion,
    AckMotionFlags,
    BusyError,
    ChassisStatus,
    Command,
    DeviceError,
    EmergencyStop,
    ErrorCode,
    ErrorEvent,
    Frame,
    FrameDecoder,
    FusaiClient,
    FusaiError,
    Link,
    Mode,
    MotionRejected,
    MotionState,
    MotorStatus,
    NackBusy,
    NackSeqLost,
    Owner,
    ProtocolError,
    Ready,
    WrongLinkError,
    BleDevice,
    BleTransport,
    encode_frame,
    is_ble_address,
    parse_inbound,
    scan_ble_devices,
)
MAX_LOG_ROWS = 2000
MAX_LOG_ENTRIES = 5000
HEARTBEAT = int(Command.HEARTBEAT)

CMD_NAMES = {int(command): command.name for command in Command}


@dataclass(frozen=True)
class FrameLog:
    timestamp: str
    direction: str
    cmd: int
    data: bytes
    wire: bytes


def command_name(cmd: int) -> str:
    return CMD_NAMES.get(int(cmd), "0x%02X" % int(cmd))


def enum_name(enum_type: Any, value: int) -> str:
    try:
        return enum_type(value).name
    except ValueError:
        return str(value)


def flag_names(value: int, enum_type: Any) -> str:
    names = [flag.name for flag in enum_type if value & int(flag)]
    return "|".join(names) if names else "0"


def hex_bytes(data: bytes) -> str:
    return " ".join("%02X" % byte for byte in data)


def hexdump(data: bytes) -> str:
    if not data:
        return "(空)"
    lines = []
    for offset in range(0, len(data), 16):
        chunk = data[offset : offset + 16]
        lines.append("%04X  %s" % (offset, hex_bytes(chunk)))
    return chr(10).join(lines)


def describe_tx(frame: Frame) -> str:
    data = frame.data
    cmd = int(frame.cmd)
    if cmd == int(Command.HEARTBEAT):
        return "心跳"
    if cmd == int(Command.SET_MODE) and len(data) == 1:
        return "SET_MODE %s" % enum_name(Mode, data[0])
    if cmd == int(Command.SET_VELOCITY) and len(data) == 7:
        seq, vx, vy, omega = struct.unpack("<Bhhh", data)
        return "SET_VELOCITY seq=%d vx=%d vy=%d wz=%d" % (seq, vx, vy, omega)
    if cmd == int(Command.SET_TARGET) and len(data) == 6:
        seq, coord, pan, tilt = struct.unpack("<BBhh", data)
        return "SET_TARGET seq=%d coord=%d pan=%d tilt=%d" % (seq, coord, pan, tilt)
    if cmd == int(Command.SET_DELTA) and len(data) == 5:
        seq, dpan, dtilt = struct.unpack("<Bhh", data)
        return "SET_DELTA seq=%d dpan=%d dtilt=%d" % (seq, dpan, dtilt)
    if cmd == int(Command.SET_TARGET_STREAM) and len(data) >= 3:
        seq, coord, count = struct.unpack("<BBB", data[:3])
        points = []
        offset = 3
        for _ in range(count):
            if offset + 4 > len(data):
                break
            pan, tilt = struct.unpack("<hh", data[offset : offset + 4])
            points.append("(%d,%d)" % (pan, tilt))
            offset += 4
        return "SET_TARGET_STREAM seq=%d coord=%d count=%d %s" % (
            seq,
            coord,
            count,
            " ".join(points),
        )
    return "data=%s" % hex_bytes(data)


def describe_rx(frame: Frame) -> str:
    try:
        message = parse_inbound(frame)
    except Exception as exc:  # noqa: BLE001 - 日志展示不能因坏帧中断
        return "解析失败: %s" % exc
    if isinstance(message, Ack):
        return "ACK %s seq=%d flags=%s" % (
            command_name(message.command),
            message.seq,
            flag_names(message.flags, AckFlags),
        )
    if isinstance(message, AckMotion):
        return "ACK_MOTION seq=%d state=%s flags=%s" % (
            message.seq,
            enum_name(MotionState, message.motion_state),
            flag_names(message.flags, AckMotionFlags),
        )
    if isinstance(message, NackBusy):
        return "NACK_BUSY cmd=%s seq=%d owner=%s" % (
            command_name(message.command),
            message.seq,
            enum_name(Owner, message.owner),
        )
    if isinstance(message, NackSeqLost):
        return "NACK_SEQ_LOST cmd=%s expected=%d" % (
            command_name(message.command),
            message.expected_seq,
        )
    if isinstance(message, ErrorEvent):
        return "ERROR code=%s detail=%d" % (
            enum_name(ErrorCode, message.code),
            message.detail,
        )
    if isinstance(message, ChassisStatus):
        return "CHASSIS_STATUS owner=%s motion=%s vx=%d vy=%d wz=%d err=%d flags=0x%02X" % (
            message.owner_name,
            message.motion_state_name,
            message.velocity_x,
            message.velocity_y,
            message.omega,
            message.error_count,
            message.flags,
        )
    if isinstance(message, Ready):
        return "READY v%d.%d flags=0x%02X" % (
            message.version_major,
            message.version_minor,
            message.flags,
        )
    if isinstance(message, MotorStatus):
        wheels = " ".join(
            ("%s=%+d" % (label, message.rpm[index]) if message.present(index) else "%s=--" % label)
            for index, label in enumerate(("A", "B", "C", "D"))
        )
        return "MOTOR_STATUS mask=0x%02X %s" % (message.present_mask, wheels)
    return "%s data=%s" % (command_name(frame.cmd), hex_bytes(frame.data))


def describe_frame(entry: FrameLog) -> str:
    frame = Frame(cmd=entry.cmd, data=entry.data)
    if entry.direction == "TX":
        return describe_tx(frame)
    return describe_rx(frame)


class TappedTransport:
    """包一层 Transport，在不影响 FusaiClient 的情况下解出收发帧。"""

    def __init__(self, inner: Any, on_frame: Callable[[str, Frame, bytes], None]) -> None:
        self._inner = inner
        self._on_frame = on_frame
        self._lock = threading.Lock()
        self._tx_decoder = FrameDecoder()
        self._rx_decoder = FrameDecoder()

    def open(self) -> None:
        with self._lock:
            self._tx_decoder.reset()
            self._rx_decoder.reset()
        self._inner.open()

    def read(self, size: int = 4096) -> bytes:
        try:
            data = self._inner.read(size)
        except Exception:
            with self._lock:
                self._rx_decoder.reset()
            raise
        if data:
            with self._lock:
                frames = list(self._rx_decoder.feed_bytes(data))
            for frame in frames:
                self._emit("RX", frame, encode_frame(frame.cmd, frame.data))
        return data

    def write(self, data: bytes) -> None:
        try:
            self._inner.write(data)
        except Exception:
            with self._lock:
                self._tx_decoder.reset()
            raise
        with self._lock:
            frames = list(self._tx_decoder.feed_bytes(data))
        for frame in frames:
            wire = bytes(data) if len(frames) == 1 else encode_frame(frame.cmd, frame.data)
            self._emit("TX", frame, wire)

    def close(self) -> None:
        self._inner.close()

    def stats(self) -> Dict[str, int]:
        with self._lock:
            return {
                "tx_frames": self._tx_decoder.frames_decoded,
                "rx_frames": self._rx_decoder.frames_decoded,
                "rx_crc_errors": self._rx_decoder.crc_errors,
                "rx_format_errors": self._rx_decoder.format_errors,
            }

    def _emit(self, direction: str, frame: Frame, wire: bytes) -> None:
        try:
            self._on_frame(direction, frame, wire)
        except Exception:
            pass


class CommandWorker(threading.Thread):
    """串行执行 SDK 命令，避免 Tk 主线程被 ACK 等待阻塞。"""

    def __init__(self, events: "queue.Queue[Tuple[Any, ...]]") -> None:
        super().__init__(name="fusai-command-worker", daemon=True)
        self._events = events
        self._jobs: "queue.Queue[Tuple[str, Callable[[], Any]]]" = queue.Queue()
        self._stop_event = threading.Event()

    def submit(self, label: str, action: Callable[[], Any]) -> None:
        if not self._stop_event.is_set():
            self._jobs.put((label, action))

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        while not self._stop_event.is_set():
            try:
                label, action = self._jobs.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                result = action()
            except Exception as exc:  # noqa: BLE001 - 错误交给界面线程展示
                self._events.put(("job_error", label, exc))
            else:
                self._events.put(("job_ok", label, result))


class TestApp:
    def __init__(self, root: tk.Tk, port: str = "") -> None:
        self.root = root
        self._initial_port = port
        self._ble_devices: List[BleDevice] = []
        self._scanning = False
        self._events: "queue.Queue[Tuple[Any, ...]]" = queue.Queue()
        self._client: Optional[FusaiClient] = None
        self._tap: Optional[TappedTransport] = None
        self._worker = CommandWorker(self._events)
        self._worker.start()
        self._connected = False
        self._connecting = False
        self._disconnecting = False
        self._closing = False
        self._velocity_busy = False
        self._last_live: Tuple[int, int, int] = (0, 0, 0)
        self._entries: Deque[FrameLog] = deque(maxlen=MAX_LOG_ENTRIES)
        self._entry_by_iid: Dict[str, FrameLog] = {}
        self._last_link_error: Optional[str] = None

        self.root.title("Fusai v2.0 蓝牙底盘测试台（BLE / UART5）")
        self.root.geometry("1180x780")
        self.root.minsize(960, 640)
        self._setup_style()
        self._build()
        self._rebuild_ble_choices()
        self._set_ui_state()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(50, self._poll_events)
        self.root.after(100, self._live_tick)

    def _setup_style(self) -> None:
        style = ttk.Style(self.root)
        for theme in ("vista", "clam", "default"):
            try:
                style.theme_use(theme)
                break
            except tk.TclError:
                continue
        style.configure("Hint.TLabel", foreground="#666666")
        style.configure("Good.TLabel", foreground="#1b5e20")
        style.configure("Warn.TLabel", foreground="#b26a00")
        style.configure("Bad.TLabel", foreground="#b00020")

    def _build(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(2, weight=1)

        self._build_connection_bar()
        self._build_status_bar()
        self._build_main_area()
        self._build_footer()

    def _build_connection_bar(self) -> None:
        bar = ttk.Frame(self.root, padding=(8, 8, 8, 4))
        bar.grid(row=0, column=0, sticky="ew")
        bar.columnconfigure(9, weight=1)

        ttk.Label(bar, text="BLE 设备").grid(row=0, column=0, padx=(0, 4))
        self._port_var = tk.StringVar(value=self._initial_port)
        self._port_combo = ttk.Combobox(bar, textvariable=self._port_var, width=36, state="normal")
        self._port_combo.grid(row=0, column=1, padx=(0, 4))

        self._refresh_button = ttk.Button(
            bar, text="扫描", width=6, command=self._scan_ble_devices
        )
        self._refresh_button.grid(row=0, column=2, padx=(0, 4))

        self._connect_button = ttk.Button(bar, text="连接", command=self._toggle_connection)
        self._connect_button.grid(row=0, column=3, padx=(0, 12))

        self._link_label = ttk.Label(bar, text="未连接", style="Hint.TLabel")
        self._link_label.grid(row=0, column=9, sticky="e")

    def _build_status_bar(self) -> None:
        bar = ttk.Frame(self.root, padding=(8, 0, 8, 4))
        bar.grid(row=1, column=0, sticky="ew")
        for column in range(6):
            bar.columnconfigure(column, weight=1)

        self._ready_var = tk.StringVar(value="READY: 未收到")
        self._owner_var = tk.StringVar(value="owner: -")
        self._motion_var = tk.StringVar(value="motion: -")
        self._velocity_var = tk.StringVar(value="vx/vy/wz: - / - / -")
        self._flag_var = tk.StringVar(value="flags: -")
        self._error_var = tk.StringVar(value="err: -")

        labels = (
            (self._ready_var, 0),
            (self._owner_var, 1),
            (self._motion_var, 2),
            (self._velocity_var, 3),
            (self._flag_var, 4),
            (self._error_var, 5),
        )
        for variable, column in labels:
            ttk.Label(bar, textvariable=variable, style="Hint.TLabel").grid(
                row=0, column=column, sticky="w", padx=(0, 8)
            )

    def _build_main_area(self) -> None:
        paned = ttk.Panedwindow(self.root, orient=tk.HORIZONTAL)
        paned.grid(row=2, column=0, sticky="nsew", padx=8, pady=4)

        left = ttk.Frame(paned, padding=(0, 0, 8, 0))
        right = ttk.Frame(paned, padding=(8, 0, 0, 0))
        paned.add(left, weight=0)
        paned.add(right, weight=1)
        self._build_left_panel(left)
        self._build_log_panel(right)

    def _build_left_panel(self, parent: ttk.Frame) -> None:
        speed = ttk.LabelFrame(parent, text="底盘速度（归一化 -1000..1000）", padding=8)
        speed.pack(fill="x")

        self._slider_vars: Dict[str, tk.IntVar] = {}
        self._value_labels: Dict[str, ttk.Label] = {}
        rows = (("vx", "vx"), ("vy", "vy"), ("wz", "wz"))
        for key, label in rows:
            row = ttk.Frame(speed)
            row.pack(fill="x", pady=3)
            ttk.Label(row, text=label, width=4).pack(side="left")
            variable = tk.IntVar(value=0)
            self._slider_vars[key] = variable
            scale = tk.Scale(
                row,
                from_=-1000,
                to=1000,
                orient=tk.HORIZONTAL,
                resolution=1,
                showvalue=False,
                highlightthickness=0,
                bd=0,
                length=270,
                variable=variable,
                command=lambda value, name=key: self._on_slider(name, value),
            )
            scale.pack(side="left", fill="x", expand=True)
            value_label = ttk.Label(row, text="0", width=6, anchor="e")
            value_label.pack(side="right")
            self._value_labels[key] = value_label

        hint = ttk.Label(
            speed,
            text="vx/vy/wz 对应 SET_VELOCITY 的三轴，单位是固件归一化输入。",
            style="Hint.TLabel",
            wraplength=320,
        )
        hint.pack(anchor="w", pady=(4, 0))

        self._build_motor_panel(parent)

        self._live_var = tk.BooleanVar(value=False)
        self._live_check = ttk.Checkbutton(
            speed,
            text="实时发送（约 10 Hz，等待上一条 ACK）",
            variable=self._live_var,
            command=self._on_live_toggle,
        )
        self._live_check.pack(anchor="w", pady=(6, 4))

        actions = ttk.Frame(speed)
        actions.pack(fill="x", pady=(2, 0))
        actions.columnconfigure(0, weight=1)
        actions.columnconfigure(1, weight=1)
        self._apply_button = ttk.Button(actions, text="应用速度", command=self._apply_velocity)
        self._apply_button.grid(row=0, column=0, sticky="ew", padx=(0, 4))
        self._stop_button = ttk.Button(actions, text="归零并停止", command=self._zero_and_stop)
        self._stop_button.grid(row=0, column=1, sticky="ew", padx=(4, 0))

        modes = ttk.LabelFrame(parent, text="模式 / 安全", padding=8)
        modes.pack(fill="x", pady=(8, 0))
        modes.columnconfigure(0, weight=1)
        modes.columnconfigure(1, weight=1)

        self._release_button = ttk.Button(modes, text="解除故障 / 释放", command=self._release)
        self._release_button.grid(row=0, column=0, sticky="ew", padx=(0, 4), pady=3)
        self._estop_button = ttk.Button(modes, text="急停 SET_MODE 3", command=self._emergency_stop)
        self._estop_button.grid(row=0, column=1, sticky="ew", padx=(4, 0), pady=3)

        warning = ttk.Label(
            modes,
            text="解除故障会清除真实急停。只在确认现场安全时使用。",
            style="Warn.TLabel",
            wraplength=320,
        )
        warning.grid(row=1, column=0, columnspan=2, sticky="w", pady=(4, 0))

        self._hardware_hint = ttk.Label(
            parent,
            text="注意：本界面连接蓝牙链路（UART5），只发送 SET_VELOCITY 等底盘命令；云台命令会回 WRONG_LINK。",
            style="Hint.TLabel",
            wraplength=320,
        )
        self._hardware_hint.pack(anchor="w", pady=(8, 0))

        self._controls: List[tk.Widget] = [
            self._live_check,
            self._apply_button,
            self._stop_button,
            self._release_button,
            self._estop_button,
        ]
        self._scales = [child for child in speed.winfo_children() if isinstance(child, tk.Scale)]
        for child in speed.winfo_children():
            if isinstance(child, ttk.Frame):
                self._scales.extend([sub for sub in child.winfo_children() if isinstance(sub, tk.Scale)])
        self._controls.extend(self._scales)

    def _build_motor_panel(self, parent: ttk.Frame) -> None:
        motors = ttk.LabelFrame(parent, text="电机实时转速（RPM）", padding=8)
        motors.pack(fill="x", pady=(8, 0))

        self._motor_vars: List[tk.StringVar] = []
        for label in ("A", "B", "C", "D"):
            row = ttk.Frame(motors)
            row.pack(fill="x", pady=1)
            ttk.Label(row, text="M%s" % label, width=4).pack(side="left")
            variable = tk.StringVar(value="-- rpm")
            ttk.Label(row, textvariable=variable, anchor="e").pack(side="right")
            self._motor_vars.append(variable)

        ttk.Label(
            motors,
            text="M2-M4 暂无驱动，mask 未置位时显示占位。",
            style="Hint.TLabel",
            wraplength=320,
        ).pack(anchor="w", pady=(4, 0))

    def _reset_motor_view(self) -> None:
        for variable in getattr(self, "_motor_vars", []):
            variable.set("-- rpm")

    def _build_log_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="收发帧日志", padding=6)
        frame.pack(fill="both", expand=True)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(1, weight=1)

        toolbar = ttk.Frame(frame)
        toolbar.grid(row=0, column=0, sticky="ew", pady=(0, 4))

        self._paused_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            toolbar,
            text="暂停",
            variable=self._paused_var,
        ).pack(side="left")
        self._show_heartbeat_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            toolbar,
            text="显示心跳",
            variable=self._show_heartbeat_var,
        ).pack(side="left", padx=(10, 0))
        ttk.Button(toolbar, text="清空", command=self._clear_log).pack(side="left", padx=(10, 0))
        ttk.Button(toolbar, text="保存...", command=self._save_log).pack(side="left", padx=(4, 0))
        self._stats_var = tk.StringVar(value="TX 0  RX 0  CRC 0  格式 0")
        ttk.Label(toolbar, textvariable=self._stats_var, style="Hint.TLabel").pack(side="right")

        tree_frame = ttk.Frame(frame)
        tree_frame.grid(row=1, column=0, sticky="nsew")
        tree_frame.rowconfigure(0, weight=1)
        tree_frame.columnconfigure(0, weight=1)

        columns = ("time", "dir", "cmd", "len", "summary", "data")
        self._tree = ttk.Treeview(tree_frame, columns=columns, show="headings", selectmode="browse")
        headings = {
            "time": ("时间", 92),
            "dir": ("方向", 48),
            "cmd": ("命令", 128),
            "len": ("长度", 44),
            "summary": ("摘要", 360),
            "data": ("载荷 (hex)", 260),
        }
        for column, (text, width) in headings.items():
            self._tree.heading(column, text=text)
            self._tree.column(column, width=width, minwidth=40, stretch=column in ("summary", "data"))
        self._tree.tag_configure("TX", foreground="#0b5394")
        self._tree.tag_configure("RX", foreground="#1b5e20")
        self._tree.tag_configure("NACK", foreground="#b26a00")
        self._tree.tag_configure("ERROR", foreground="#b00020")
        self._tree.grid(row=0, column=0, sticky="nsew")

        yscroll = ttk.Scrollbar(tree_frame, orient=tk.VERTICAL, command=self._tree.yview)
        yscroll.grid(row=0, column=1, sticky="ns")
        xscroll = ttk.Scrollbar(tree_frame, orient=tk.HORIZONTAL, command=self._tree.xview)
        xscroll.grid(row=1, column=0, sticky="ew")
        self._tree.configure(yscrollcommand=yscroll.set, xscrollcommand=xscroll.set)
        self._tree.bind("<<TreeviewSelect>>", self._on_tree_select)

        detail_frame = ttk.LabelFrame(frame, text="选中帧详情", padding=4)
        detail_frame.grid(row=2, column=0, sticky="ew", pady=(6, 0))
        detail_frame.columnconfigure(0, weight=1)
        self._detail = tk.Text(
            detail_frame,
            height=8,
            wrap="none",
            font=("Consolas", 9),
            state="disabled",
        )
        self._detail.grid(row=0, column=0, sticky="ew")
        detail_scroll = ttk.Scrollbar(detail_frame, orient=tk.VERTICAL, command=self._detail.yview)
        detail_scroll.grid(row=0, column=1, sticky="ns")
        self._detail.configure(yscrollcommand=detail_scroll.set)

    def _build_footer(self) -> None:
        self._message_var = tk.StringVar(value="就绪")
        footer = ttk.Label(
            self.root,
            textvariable=self._message_var,
            anchor="w",
            relief="sunken",
            padding=4,
        )
        footer.grid(row=3, column=0, sticky="ew")

    def _on_slider(self, key: str, value: str) -> None:
        self._value_labels[key].configure(text=str(int(float(value))))

    def _slider_values(self) -> Tuple[int, int, int]:
        return (
            int(self._slider_vars["vx"].get()),
            int(self._slider_vars["vy"].get()),
            int(self._slider_vars["wz"].get()),
        )

    def _zero_sliders(self) -> None:
        for key in ("vx", "vy", "wz"):
            self._slider_vars[key].set(0)
            self._value_labels[key].configure(text="0")

    def _on_live_toggle(self) -> None:
        if self._live_var.get():
            self._last_live = (0, 0, 0)
            self._live_tick()

    def _live_tick(self) -> None:
        if self._closing:
            return
        if self._connected and self._live_var.get() and not self._velocity_busy:
            current = self._slider_values()
            if current != self._last_live:
                self._apply_velocity()
        self.root.after(100, self._live_tick)

    def _apply_velocity(self) -> None:
        client = self._client
        if not self._connected or client is None:
            self._set_message("请先连接蓝牙")
            return
        if self._velocity_busy:
            return
        vx, vy, wz = self._slider_values()
        self._last_live = (vx, vy, wz)
        self._velocity_busy = True
        if vx == 0 and vy == 0 and wz == 0:
            self._worker.submit("stop", client.stop_chassis)
            self._set_message("发送归零速度...")
        else:
            self._worker.submit("velocity", lambda: client.set_velocity(vx, vy, wz))
            self._set_message("发送 SET_VELOCITY vx=%d vy=%d wz=%d..." % (vx, vy, wz))

    def _zero_and_stop(self) -> None:
        self._zero_sliders()
        client = self._client
        if not self._connected or client is None:
            self._set_message("请先连接蓝牙")
            return
        self._last_live = (0, 0, 0)
        self._velocity_busy = True
        self._worker.submit("stop", client.stop_chassis)
        self._set_message("发送停止命令（新序号）...")

    def _release(self) -> None:
        client = self._client
        if not self._connected or client is None:
            self._set_message("请先连接蓝牙")
            return
        if not messagebox.askyesno(
            "解除故障",
            "这会发送 SET_MODE(0)，清除急停/故障并释放运动权。\n只确认现场安全后继续。",
            parent=self.root,
        ):
            return
        self._worker.submit("release", client.release)
        self._set_message("发送 SET_MODE(0)...")

    def _emergency_stop(self) -> None:
        client = self._client
        if not self._connected or client is None:
            self._set_message("请先连接蓝牙")
            return
        try:
            client.emergency_stop()
        except Exception as exc:  # noqa: BLE001
            self._set_message("急停发送失败: %s" % self._format_exception(exc))
            return
        self._set_message("已直接发送 SET_MODE(3) 急停；需要 SET_MODE(0) 才能退出")

    def _toggle_connection(self) -> None:
        if self._connected:
            self._begin_disconnect()
            return
        if self._connecting:
            return
        address = self._selected_ble_address()
        if not address:
            messagebox.showwarning(
                "缺少 BLE 设备",
                "请点击“扫描”选择设备，或直接输入 BLE 地址，例如\n"
                "F0:72:D1:80:FB:EB。",
                parent=self.root,
            )
            return
        self._connecting = True
        self._set_ui_state()
        self._set_message("正在连接 BLE %s..." % address)
        threading.Thread(
            target=self._connect_worker,
            args=(address,),
            name="fusai-connect",
            daemon=True,
        ).start()

    def _selected_ble_address(self) -> str:
        selection = self._port_var.get().strip()
        if is_ble_address(selection):
            return selection
        for device in self._ble_devices:
            if selection in (device.label, device.display_name, device.address):
                return device.address
        # The combo label is "name [AA:BB:CC:DD:EE:FF]"; accept an address
        # at the end even when the name/status prefix is stale.
        tail = selection[:-1] if selection.endswith("]") else selection
        if len(tail) >= 17 and is_ble_address(tail[-17:]):
            return tail[-17:]
        return ""

    def _connect_worker(self, address: str) -> None:
        client: Optional[FusaiClient] = None
        try:
            tap = TappedTransport(
                BleTransport(address),
                self._on_frame_thread,
            )
            client = FusaiClient(
                transport=tap,
                link=Link.BT,
                auto_heartbeat=True,
                on_status=lambda status: self._events.put(("status", status)),
                on_motor_status=lambda status: self._events.put(("motor_status", status)),
                on_ready=lambda ready: self._events.put(("ready", ready)),
                on_error=lambda error: self._events.put(("device_error", error)),
            )
            client.open()
        except Exception as exc:  # noqa: BLE001
            if client is not None:
                try:
                    client.close()
                except Exception:
                    pass
            self._events.put(("connect_error", exc))
            return
        if self._closing:
            try:
                client.close()
            except Exception:
                pass
            return
        self._tap = tap
        self._client = client
        self._events.put(("connected", address))

    def _begin_disconnect(self) -> None:
        client = self._client
        self._connected = False
        self._connecting = True
        self._disconnecting = True
        self._client = None
        self._tap = None
        self._velocity_busy = False
        self._set_ui_state()
        self._set_message("正在断开...")
        if client is None:
            self._events.put(("disconnected",))
            return
        threading.Thread(
            target=self._close_client_worker,
            args=(client,),
            name="fusai-disconnect",
            daemon=True,
        ).start()

    def _close_client_worker(self, client: FusaiClient) -> None:
        try:
            client.close()
        except Exception as exc:  # noqa: BLE001
            self._events.put(("disconnect_error", exc))
        finally:
            self._events.put(("disconnected",))

    def _on_frame_thread(self, direction: str, frame: Frame, wire: bytes) -> None:
        entry = FrameLog(
            timestamp=datetime.now().strftime("%H:%M:%S.%f")[:-3],
            direction=direction,
            cmd=int(frame.cmd),
            data=bytes(frame.data),
            wire=bytes(wire),
        )
        self._events.put(("frame", entry))

    def _poll_events(self) -> None:
        if self._closing:
            return
        for _ in range(500):
            try:
                event = self._events.get_nowait()
            except queue.Empty:
                break
            try:
                self._handle_event(event)
            except Exception as exc:  # noqa: BLE001
                self._set_message("界面处理异常: %s" % exc)
        self._refresh_link_view()
        self.root.after(50, self._poll_events)

    def _handle_event(self, event: Tuple[Any, ...]) -> None:
        kind = event[0]
        if kind == "frame":
            self._append_frame(event[1])
        elif kind == "status":
            self._apply_status(event[1])
        elif kind == "motor_status":
            self._apply_motor_status(event[1])
        elif kind == "ready":
            self._apply_ready(event[1])
        elif kind == "device_error":
            self._apply_device_error(event[1])
        elif kind == "ble_scan_done":
            self._scanning = False
            self._ble_devices = list(event[1])
            self._rebuild_ble_choices()
            self._set_ui_state()
            named = sum(1 for device in self._ble_devices if device.name)
            connected = sum(1 for device in self._ble_devices if device.connected)
            serial = sum(1 for device in self._ble_devices if device.serial)
            if self._ble_devices:
                self._set_message(
                    "发现 %d 个 BLE 设备（%d 个有名，%d 个已连接，%d 个可串口）；"
                    "选择后点“连接”"
                    % (len(self._ble_devices), named, connected, serial)
                )
            else:
                self._set_message(
                    "未发现 BLE 设备：确认模块已上电且未被手机占用，再点“扫描”"
                )
        elif kind == "ble_scan_error":
            self._scanning = False
            self._set_ui_state()
            self._set_message("扫描失败: %s" % self._format_exception(event[1]))
            messagebox.showerror(
                "扫描失败", self._format_exception(event[1]), parent=self.root
            )
        elif kind == "connected":
            self._connected = True
            self._connecting = False
            self._disconnecting = False
            self._reset_motor_view()
            self._set_ui_state()
            self._set_message("已连接 BLE %s" % event[1])
        elif kind == "connect_error":
            self._connecting = False
            self._disconnecting = False
            self._client = None
            self._tap = None
            self._reset_motor_view()
            self._set_ui_state()
            self._set_message("连接失败: %s" % self._format_exception(event[1]))
            messagebox.showerror("连接失败", self._format_exception(event[1]), parent=self.root)
        elif kind == "disconnected":
            self._connected = False
            self._connecting = False
            self._disconnecting = False
            self._client = None
            self._tap = None
            self._velocity_busy = False
            self._reset_motor_view()
            self._set_ui_state()
            self._set_message("蓝牙已断开")
        elif kind == "disconnect_error":
            self._set_message("断开时出错: %s" % self._format_exception(event[1]))
        elif kind == "job_ok":
            self._on_job_ok(event[1], event[2])
        elif kind == "job_error":
            self._on_job_error(event[1], event[2])

    def _on_job_ok(self, label: str, result: Any) -> None:
        if label in ("velocity", "stop"):
            self._velocity_busy = False
        if label == "release":
            self._set_message("已发送 SET_MODE(0)，故障/运动权已释放")
        elif isinstance(result, AckMotion):
            self._set_message(
                "ACK_MOTION seq=%d state=%s flags=%s"
                % (
                    result.seq,
                    enum_name(MotionState, result.motion_state),
                    flag_names(result.flags, AckMotionFlags),
                )
            )
        elif isinstance(result, Ack):
            self._set_message(
                "ACK %s seq=%d flags=%s"
                % (
                    command_name(result.command),
                    result.seq,
                    flag_names(result.flags, AckFlags),
                )
            )
        elif label == "stop":
            self._set_message("停止命令已确认")
        else:
            self._set_message("%s 完成" % label)

    def _on_job_error(self, label: str, exc: Exception) -> None:
        if label in ("velocity", "stop"):
            self._velocity_busy = False
        self._set_message("%s 失败: %s" % (label, self._format_exception(exc)))

    def _apply_status(self, status: ChassisStatus) -> None:
        self._owner_var.set("owner: %s" % status.owner_name)
        self._motion_var.set("motion: %s" % status.motion_state_name)
        self._velocity_var.set(
            "vx/vy/wz: %d / %d / %d"
            % (status.velocity_x, status.velocity_y, status.omega)
        )
        self._error_var.set("err: %d" % status.error_count)
        flags = []
        if status.emergency:
            flags.append("EMERGENCY")
        if status.bt_heartbeat_ok:
            flags.append("蓝牙心跳正常")
        if status.chassis_stopped:
            flags.append("底盘已停")
        if status.pi_link_lost:
            flags.append("树莓派链路丢失")
        if status.bt_link_lost:
            flags.append("蓝牙链路丢失")
        if not flags:
            flags.append("flags=0x%02X" % status.flags)
        self._flag_var.set("flags: %s" % " ".join(flags))
        if status.emergency:
            self._set_message("CHASSIS_STATUS: 急停/故障中（%s）" % " ".join(flags))
        else:
            self._set_message("CHASSIS_STATUS: %s" % " ".join(flags))

    def _apply_motor_status(self, status: MotorStatus) -> None:
        for index, variable in enumerate(self._motor_vars):
            if status.present(index):
                variable.set("%+d rpm" % status.rpm[index])
            else:
                variable.set("占位")

    def _apply_ready(self, ready: Ready) -> None:
        self._ready_var.set(
            "READY: v%d.%d flags=0x%02X" % (ready.version_major, ready.version_minor, ready.flags)
        )

    def _apply_device_error(self, error: ErrorEvent) -> None:
        self._set_message(
            "设备 ERROR: %s detail=%d" % (enum_name(ErrorCode, error.code), error.detail)
        )

    def _append_frame(self, entry: FrameLog) -> None:
        if self._paused_var.get():
            return
        if entry.cmd == HEARTBEAT and not self._show_heartbeat_var.get():
            return
        summary = describe_frame(entry)
        tag = entry.direction
        if entry.cmd in (int(Command.NACK_BUSY), int(Command.NACK_SEQ_LOST)):
            tag = "NACK"
        elif entry.cmd == int(Command.ERROR):
            tag = "ERROR"
        iid = self._tree.insert(
            "",
            "end",
            values=(
                entry.timestamp,
                entry.direction,
                command_name(entry.cmd),
                len(entry.data),
                summary,
                hex_bytes(entry.data),
            ),
            tags=(tag,),
        )
        self._entry_by_iid[iid] = entry
        self._entries.append(entry)
        children = self._tree.get_children()
        if len(children) > MAX_LOG_ROWS:
            for old in children[: len(children) - MAX_LOG_ROWS]:
                self._entry_by_iid.pop(old, None)
                self._tree.delete(old)
        self._tree.see(iid)

    def _on_tree_select(self, _event: Any = None) -> None:
        selection = self._tree.selection()
        if not selection:
            return
        entry = self._entry_by_iid.get(selection[0])
        if entry is None:
            return
        lines = [
            "时间: %s" % entry.timestamp,
            "方向: %s" % entry.direction,
            "命令: %s (0x%02X)" % (command_name(entry.cmd), entry.cmd),
            "数据长度: %d" % len(entry.data),
            "摘要: %s" % describe_frame(entry),
            "",
            "原始帧 (%d 字节):" % len(entry.wire),
            hexdump(entry.wire),
            "",
            "载荷 (%d 字节):" % len(entry.data),
            hexdump(entry.data),
        ]
        self._detail.configure(state="normal")
        self._detail.delete("1.0", "end")
        self._detail.insert("1.0", chr(10).join(lines))
        self._detail.configure(state="disabled")

    def _clear_log(self) -> None:
        self._tree.delete(*self._tree.get_children())
        self._entry_by_iid.clear()
        self._entries.clear()
        self._detail.configure(state="normal")
        self._detail.delete("1.0", "end")
        self._detail.configure(state="disabled")

    def _save_log(self) -> None:
        if not self._entries:
            messagebox.showinfo("没有日志", "当前没有可保存的帧。", parent=self.root)
            return
        path = filedialog.asksaveasfilename(
            parent=self.root,
            title="保存收发日志",
            defaultextension=".tsv",
            filetypes=(("TSV 文件", "*.tsv"), ("文本文件", "*.txt"), ("全部文件", "*.*")),
            initialfile="fusai-log-%s.tsv" % datetime.now().strftime("%Y%m%d-%H%M%S"),
        )
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8-sig", newline="") as handle:
                writer = csv.writer(handle, delimiter=chr(9))
                writer.writerow(("时间", "方向", "命令", "长度", "摘要", "载荷 hex", "原始帧 hex"))
                for entry in self._entries:
                    writer.writerow(
                        (
                            entry.timestamp,
                            entry.direction,
                            command_name(entry.cmd),
                            len(entry.data),
                            describe_frame(entry),
                            hex_bytes(entry.data),
                            hex_bytes(entry.wire),
                        )
                    )
        except OSError as exc:
            messagebox.showerror("保存失败", str(exc), parent=self.root)
            return
        self._set_message("日志已保存到 %s" % path)

    def _scan_ble_devices(self) -> None:
        if self._connected or self._connecting or self._scanning:
            return
        self._scanning = True
        self._set_ui_state()
        self._set_message("正在扫描附近 BLE 设备（约 10 秒）...")
        threading.Thread(
            target=self._ble_scan_worker,
            name="fusai-ble-scan",
            daemon=True,
        ).start()

    def _ble_scan_worker(self) -> None:
        try:
            devices = scan_ble_devices(10.0)
        except Exception as exc:  # noqa: BLE001
            self._events.put(("ble_scan_error", exc))
            return
        self._events.put(("ble_scan_done", devices))

    def _ble_device_label(self, device: BleDevice) -> str:
        markers = []
        if device.connected:
            markers.append("已连接")
        if device.serial:
            markers.append("串口")
        name = device.name.strip()
        if not name:
            return device.address
        suffix = "（%s）" % " / ".join(markers) if markers else ""
        # Keep the address at the end so a pasted label still resolves.
        return "%s%s [%s]" % (name, suffix, device.address)

    def _rebuild_ble_choices(self) -> None:
        named = [device for device in self._ble_devices if device.name]
        shown = named or list(self._ble_devices)
        values = [self._ble_device_label(device) for device in shown]
        current = self._port_var.get().strip()
        self._port_combo.configure(values=values)
        if current in values or is_ble_address(current):
            return
        # Prefer a connected, serial-capable peripheral (the HC-05 case)
        # over a mouse/headset that happens to be connected to Windows.
        preferred = next(
            (device for device in shown if device.connected and device.serial), None
        )
        if preferred is None:
            preferred = next((device for device in shown if device.serial), None)
        if preferred is None and len(shown) == 1:
            preferred = shown[0]
        if preferred is not None:
            self._port_var.set(self._ble_device_label(preferred))

    def _refresh_link_view(self) -> None:
        client = self._client
        if client is None:
            if self._connected:
                self._link_label.configure(text="已连接", style="Good.TLabel")
            else:
                self._link_label.configure(text="未连接", style="Hint.TLabel")
            return
        error = client.link_error
        error_text = str(error) if error is not None else None
        if error_text and error_text != self._last_link_error:
            self._last_link_error = error_text
            self._set_message("链路错误: %s" % error_text)
        if error_text:
            self._link_label.configure(text="链路错误", style="Bad.TLabel")
        elif client.is_link_alive:
            self._link_label.configure(text="已连接 / 链路正常", style="Good.TLabel")
        else:
            self._link_label.configure(text="已连接 / 等待帧", style="Warn.TLabel")
        if self._tap is not None:
            stats = self._tap.stats()
            self._stats_var.set(
                "TX %d  RX %d  CRC 错 %d  格式错 %d"
                % (
                    stats["tx_frames"],
                    stats["rx_frames"],
                    stats["rx_crc_errors"],
                    stats["rx_format_errors"],
                )
            )

    def _set_ui_state(self) -> None:
        connected = self._connected
        connecting = self._connecting
        busy = connected or connecting or self._scanning
        input_state = "disabled" if busy else "normal"
        self._port_combo.configure(state=input_state)
        self._refresh_button.configure(state=input_state)
        if self._disconnecting:
            self._connect_button.configure(text="断开中...", state="disabled")
        elif connecting:
            self._connect_button.configure(text="连接中...", state="disabled")
        elif self._scanning:
            self._connect_button.configure(text="扫描中...", state="disabled")
        elif connected:
            self._connect_button.configure(text="断开", state="normal")
        else:
            self._connect_button.configure(text="连接", state="normal")
        control_state = "normal" if connected else "disabled"
        for widget in getattr(self, "_controls", []):
            try:
                widget.configure(state=control_state)
            except tk.TclError:
                pass

    def _set_message(self, text: str) -> None:
        self._message_var.set(text)

    def _format_exception(self, exc: Exception) -> str:
        if isinstance(exc, BusyError):
            return "忙: cmd=%s seq=%d owner=%s" % (
                command_name(exc.command),
                exc.seq,
                enum_name(Owner, exc.owner),
            )
        if isinstance(exc, MotionRejected):
            return "运动被拒绝: seq=%d state=%s flags=0x%02X" % (
                exc.seq,
                enum_name(MotionState, exc.motion_state),
                exc.flags,
            )
        if isinstance(exc, DeviceError):
            return "设备错误: %s detail=%d" % (enum_name(ErrorCode, exc.code), exc.detail)
        if isinstance(exc, WrongLinkError):
            return "命令 0x%02X 不属于当前链路（蓝牙仅支持底盘命令）" % exc.command
        if isinstance(exc, EmergencyStop):
            return "命令被急停中断"
        if isinstance(exc, ProtocolError):
            return "协议错误: %s" % exc
        if isinstance(exc, FusaiError):
            return "%s: %s" % (type(exc).__name__, exc)
        return "%s: %s" % (type(exc).__name__, exc)

    def _on_close(self) -> None:
        if self._closing:
            return
        self._closing = True
        self._worker.stop()
        client = self._client
        self._client = None
        if client is not None:
            try:
                client.close()
            except Exception:
                pass
        self.root.destroy()


def run_self_test() -> int:
    root = tk.Tk()
    root.withdraw()
    app = TestApp(root)
    root.update_idletasks()
    root.update()
    app._on_close()
    print("SELFTEST_OK")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Fusai v2.0 蓝牙底盘测试台")
    parser.add_argument("--port", default="", help="初始 BLE 地址，例如 F0:72:D1:80:FB:EB")
    parser.add_argument("--self-test", action="store_true", help="只构造界面并退出，不打开 BLE")
    args = parser.parse_args()

    if args.self_test:
        return run_self_test()

    root = tk.Tk()
    TestApp(root, port=args.port)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
