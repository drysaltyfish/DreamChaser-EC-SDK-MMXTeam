#!/usr/bin/env python3
"""Move the gimbal to one absolute angle and print telemetry.

Run from the sdk directory:

    uv run python examples/basic_gimbal.py /dev/ttyAMA0 --pan 100 --tilt 200

Angles are 0.01 degree units, so 100 is 1.00 degree.
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Optional, Sequence

from fusai_sdk import FusaiClient, Link, ReadyTimeout


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", nargs="?", default="/dev/ttyAMA0", help="serial port")
    parser.add_argument("--baud", type=int, default=115200, help="baud rate")
    parser.add_argument("--pan", type=int, default=0, help="pan angle, 0.01 degree")
    parser.add_argument("--tilt", type=int, default=3000, help="tilt angle, 0.01 degree")
    parser.add_argument(
        "--settle",
        type=float,
        default=2.0,
        help="seconds to wait before printing GIMBAL_STATUS",
    )
    return parser.parse_args(argv)


def run(robot: FusaiClient, args: argparse.Namespace) -> None:
    try:
        ready = robot.wait_ready(timeout=2.0)
        print("READY firmware %d.%d flags=0x%02X" % (ready.version_major, ready.version_minor, ready.flags))
    except ReadyTimeout:
        print("READY not observed (the STM32 may have booted before the Pi)", file=sys.stderr)

    # SET_MODE(0) on the Pi link clears the gimbal sequence baseline and any
    # gimbal fault, giving this script a known starting point.
    robot.release()
    robot.track_gimbal()
    ack = robot.set_target(args.pan, args.tilt)
    print("SET_TARGET accepted, flags=0x%02X" % ack.flags)

    time.sleep(max(0.0, args.settle))
    status = robot.status
    if status is None:
        print("no GIMBAL_STATUS received yet", file=sys.stderr)
        return
    print(
        "GIMBAL_STATUS owner=%s state=%s pan=%d tilt=%d errors=%d"
        % (
            status.owner_name,
            status.gimbal_state_name,
            status.pan,
            status.tilt,
            status.error_count,
        )
    )
    robot.release()


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    with FusaiClient(args.port, link=Link.PI, baudrate=args.baud) as robot:
        try:
            run(robot, args)
        except KeyboardInterrupt:
            # Do not silently release an emergency stop: inspect the robot first,
            # then call robot.release() from a deliberate operator action.
            robot.emergency_stop()
            print("emergency stop sent; call release() after inspection", file=sys.stderr)
            return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
