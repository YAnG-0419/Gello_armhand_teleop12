#!/usr/bin/env python3
"""Record a labelled, hardware-disabled MANUS hand-accuracy protocol.

The UDP output is captured by a private local sink, so this utility never
commands a hand. Stop the normal operator first: only one MANUS client may run.
"""

from __future__ import annotations

import argparse
import socket
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "teleop_sources" / "pico" / "src"))
sys.path.insert(0, str(REPO_ROOT / "teleop_sources" / "manus" / "python"))

from manus_teleop import ManusHandPipeline  # noqa: E402


PHASES = (
    (
        "fully_open",
        "Fully open and straighten all five human fingers.",
    ),
    (
        "four_fingers_fully_curled",
        "Fully curl index, middle, ring and pinky; keep the thumb open.",
    ),
    (
        "thumb_index_pinch",
        "Touch the thumb tip to the index fingertip in a firm pinch.",
    ),
    (
        "thumb_fully_curled",
        "Fully curl the thumb into the palm; keep the other fingers open.",
    ),
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument(
        "--sides", choices=("left", "right", "both"), default="both"
    )
    parser.add_argument("--filter-alpha", type=float, default=0.85)
    args = parser.parse_args()
    if args.seconds <= 0.0:
        parser.error("--seconds must be positive")
    if args.output.exists():
        parser.error(f"refusing to overwrite existing output: {args.output}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    sides = ("left", "right") if args.sides == "both" else (args.sides,)

    # An ephemeral private receiver guarantees that no live hand bridge can see
    # these packets. It also makes accidental use during a Compose bringup safe.
    sink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sink.bind(("127.0.0.1", 0))
    sink.setblocking(False)
    pipeline = None
    try:
        pipeline = ManusHandPipeline(
            host="127.0.0.1",
            port=sink.getsockname()[1],
            rate=30.0,
            filter_alpha=args.filter_alpha,
            debug_log=args.output,
            models={"left": "g20", "right": "o30i"},
            dynamic_sides=sides,
        )
        print("Connected to MANUS. No robot commands can leave this process.")
        print(f"Recording {', '.join(sides)} to {args.output}")
        active = {side: side in sides for side in ("left", "right")}
        for phase, instruction in PHASES:
            pipeline.set_debug_phase(None)
            input(f"\n{instruction}\nHold the pose, then press Enter to record: ")
            pipeline.set_debug_phase(phase)
            deadline = time.monotonic() + args.seconds
            while time.monotonic() < deadline:
                started = time.monotonic()
                pipeline.tick(started, active=active)
                try:
                    while sink.recv(65_535):
                        pass
                except BlockingIOError:
                    pass
                remaining = 0.01 - (time.monotonic() - started)
                if remaining > 0.0:
                    time.sleep(remaining)
            print(f"Captured {phase}.")
        pipeline.set_debug_phase(None)
    finally:
        if pipeline is not None:
            pipeline.close()
        sink.close()
    print(f"Saved {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
