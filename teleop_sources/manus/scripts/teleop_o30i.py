#!/usr/bin/env python3
"""Drive only the right O30i from MANUS with explicit keyboard activation."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "teleop_sources" / "pico" / "src"))
sys.path.insert(0, str(REPO_ROOT / "teleop_sources" / "manus" / "python"))

from manus_teleop import RightOnlyManusHandPipeline  # noqa: E402
from pico_bimanual_franka_teleop.xr_input import KeyboardActivation  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5570)
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--stale-timeout", type=float, default=0.25)
    parser.add_argument(
        "--filter-alpha",
        type=float,
        default=0.85,
        help="EMA response in (0, 1]; 1 disables retarget-output smoothing",
    )
    parser.add_argument("--keyboard-device", default="/dev/tty")
    parser.add_argument("--debug-log")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="stop after this many seconds; zero runs until Q or Ctrl-C",
    )
    args = parser.parse_args()
    if args.duration < 0.0:
        parser.error("--duration must not be negative")

    pipeline = None
    keyboard = None
    try:
        pipeline = RightOnlyManusHandPipeline(
            host=args.host,
            port=args.port,
            rate=args.rate,
            stale_timeout=args.stale_timeout,
            filter_alpha=args.filter_alpha,
            debug_log=args.debug_log,
            models={"left": "g20", "right": "o30i"},
        )
        keyboard = KeyboardActivation(args.keyboard_device, sides=("right",))
        keyboard.show(
            "Right O30i starts DISENGAGED. Space/R toggles following; "
            "X stops; O requests open while disengaged; Q exits."
        )
        started = time.monotonic()
        deadline = started + args.duration if args.duration > 0.0 else None
        next_report = started
        previous_sent = 0
        while deadline is None or time.monotonic() < deadline:
            loop_started = time.monotonic()
            active = keyboard.poll()
            requests = keyboard.take_requests()
            if requests["open_hands"]:
                pipeline.request_open(loop_started)
            pipeline.tick(loop_started, active=active)

            if loop_started >= next_report:
                status = pipeline.status.sides["right"]
                elapsed = max(loop_started - (next_report - 1.0), 1e-6)
                send_rate = (status.sent - previous_sent) / elapsed
                previous_sent = status.sent
                detail = (
                    f"sending {send_rate:.1f} Hz, solve "
                    f"{status.solve_seconds * 1e3:.1f} ms"
                    if status.sending
                    else f"not sending ({status.fault})"
                )
                keyboard.show(f"Right O30i: {detail}")
                next_report = loop_started + 1.0

            remaining = 0.01 - (time.monotonic() - loop_started)
            if remaining > 0.0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        if keyboard is not None:
            keyboard.close()
        if pipeline is not None:
            pipeline.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
