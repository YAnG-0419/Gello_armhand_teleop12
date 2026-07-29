#!/usr/bin/env python3
"""Drive the LinkerHands from MANUS gloves, no arms - per-side activation.

`--sides left` tests only the left G20, `--sides right` only the right
O30i, `both` (default) runs bimanual. Keys: `L`/`R` toggle one side,
`Space` toggles the selected sides together, `X` stops, `O` requests the
open pose while disengaged, `Q` exits.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "teleop_sources" / "pico" / "src"))
sys.path.insert(0, str(REPO_ROOT / "teleop_sources" / "manus" / "python"))

from manus_teleop import ManusHandPipeline  # noqa: E402
from pico_bimanual_franka_teleop.xr_input import KeyboardActivation  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sides",
        default="both",
        choices=("left", "right", "both"),
        help="which hands follow their gloves (default: both)",
    )
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
    parser.add_argument(
        "--left-thumb",
        default="fixed",
        choices=("fixed", "full"),
        help="left G20 thumb: fixed opposition (deployed default) or the "
        "full CMC solve under feel-check (default: fixed)",
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
    sides = ("left", "right") if args.sides == "both" else (args.sides,)

    pipeline = None
    keyboard = None
    try:
        pipeline = ManusHandPipeline(
            dynamic_sides=sides,
            host=args.host,
            port=args.port,
            rate=args.rate,
            stale_timeout=args.stale_timeout,
            filter_alpha=args.filter_alpha,
            debug_log=args.debug_log,
            models={"left": "g20", "right": "o30i"},
            left_thumb_mode=args.left_thumb,
        )
        keyboard = KeyboardActivation(args.keyboard_device, sides=sides)
        keyboard.show(
            f"MANUS hands ({', '.join(sides)}) start DISENGAGED. "
            "L/R toggles one side, Space toggles together; X stops; "
            "O requests open while disengaged; Q exits."
        )
        started = time.monotonic()
        deadline = started + args.duration if args.duration > 0.0 else None
        next_report = started
        previous_sent = {side: 0 for side in sides}
        while deadline is None or time.monotonic() < deadline:
            loop_started = time.monotonic()
            active = keyboard.poll()
            requests = keyboard.take_requests()
            if requests["open_hands"]:
                pipeline.request_open(loop_started)
            pipeline.tick(loop_started, active=active)

            if loop_started >= next_report:
                parts = []
                elapsed = max(loop_started - (next_report - 1.0), 1e-6)
                for side in sides:
                    status = pipeline.status.sides[side]
                    send_rate = (status.sent - previous_sent[side]) / elapsed
                    previous_sent[side] = status.sent
                    parts.append(
                        f"{side}: sending {send_rate:.1f} Hz, solve "
                        f"{status.solve_seconds * 1e3:.1f} ms"
                        if status.sending
                        else f"{side}: not sending ({status.fault})"
                    )
                keyboard.show(" | ".join(parts))
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
