#!/usr/bin/env python3
"""Drive the chassis with normalized velocity commands, then stop safely.

Run from the sdk directory:

    uv run python examples/chassis_velocity.py F0:72:D1:80:FB:EB --vx 300 --duration 3

The chassis link uses a BLE 4.0 module (firmware hc05V2.3_le, GATT FFE0/FFE1).
Pass --serial COM5 only for a legacy SPP module or a USB-UART adapter.

vx/vy are -1000..1000 and omega is also -1000..1000.  The firmware clamps
out-of-range values.  Only vx currently reaches MotorA; vy and omega are
accepted by the protocol but have no actuator yet.
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Optional, Sequence

from fusai_sdk import BleTransport, FusaiClient, Link, MotionRejected


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "address",
        nargs="?",
        default="F0:72:D1:80:FB:EB",
        help="BLE device address, for example F0:72:D1:80:FB:EB",
    )
    parser.add_argument(
        "--serial",
        default="",
        help="use a legacy Bluetooth serial port (for example COM5) instead of BLE",
    )
    parser.add_argument("--baud", type=int, default=115200, help="serial baud rate (--serial only)")
    parser.add_argument("--vx", type=int, default=300, help="forward velocity, -1000..1000")
    parser.add_argument("--vy", type=int, default=0, help="lateral velocity, -1000..1000")
    parser.add_argument("--omega", type=int, default=0, help="yaw rate, -1000..1000")
    parser.add_argument("--duration", type=float, default=2.0, help="seconds to drive")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    if args.serial:
        client = FusaiClient(args.serial, link=Link.BT, baudrate=args.baud)
    else:
        client = FusaiClient(transport=BleTransport(args.address), link=Link.BT)
    with client as robot:
        robot.release()  # known sequence baseline and no stale owner
        emergency = False
        try:
            ack = robot.set_velocity(args.vx, args.vy, args.omega)
            print("velocity accepted, flags=0x%02X" % ack.flags)
            deadline = time.monotonic() + max(0.0, args.duration)
            while time.monotonic() < deadline:
                time.sleep(0.1)
                status = robot.status
                if status is not None:
                    print(
                        "owner=%s motion=%s errors=%d"
                        % (status.owner_name, status.motion_state_name, status.error_count)
                    )
        except MotionRejected as exc:
            print("velocity rejected: %s" % exc, file=sys.stderr)
            return 2
        except KeyboardInterrupt:
            emergency = True
            robot.emergency_stop()
            print("emergency stop sent; call release() after inspection", file=sys.stderr)
            return 130
        finally:
            # stop_chassis() is an all-zero SET_VELOCITY.  It is accepted even
            # if a velocity ACK was lost, and it is not a duplicate of a new
            # motion because the client advances the sequence after a timeout.
            if not emergency and robot.link_error is None:
                try:
                    robot.stop_chassis()
                except Exception as exc:  # keep the release attempt below
                    print("stop_chassis failed: %s" % exc, file=sys.stderr)
                try:
                    robot.release()
                except Exception as exc:
                    print("release failed: %s" % exc, file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
