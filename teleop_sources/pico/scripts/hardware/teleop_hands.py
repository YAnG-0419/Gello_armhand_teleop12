#!/usr/bin/env python3
"""Drive the Linker Hands from live PICO optical hand tracking, hands only.

The same HandPipeline that `teleop_dual_fr3.py --hands` ticks from the arm loop,
ticked here from a plain loop instead. This process owns the single XRoboToolkit
SDK client, so it must not run at the same time as `teleop_dual_fr3.py`.

Sends hand commands to `linker_hand_bridge`. Nothing reaches the hands unless
that bridge was launched enabled, so this is safe to run against a dry-run
bridge. When a hand stops being usable the pipeline stops sending for that side,
the bridge's watchdog stops publishing, and the hand holds position; there is no
automatic return-home.
"""

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO_ROOT / "teleop_sources" / "pico" / "src"))

from pico_bimanual_franka_teleop.hand_teleop import HandPipeline  # noqa: E402

SIDES = ("left", "right")


def _desktop_gui_pids() -> list[int]:
    pids = []
    for process in Path("/proc").iterdir():
        if not process.name.isdigit():
            continue
        try:
            command = (process / "cmdline").read_bytes().replace(b"\0", b" ")
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if b"RobotLinuxDemo.x86_64" in command:
            pids.append(int(process.name))
    return sorted(pids)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument(
        "--port",
        type=int,
        default=5570,
        help="where linker_hand_bridge listens (default: 5570)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=30.0,
        help="hand commands per second per side; keep at or below 60 (default: 30)",
    )
    parser.add_argument("--sides", default="both", choices=["left", "right", "both"])
    parser.add_argument("--stale-timeout", type=float, default=0.25)
    parser.add_argument("--frozen-timeout", type=float, default=1.0)
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="stop after this many seconds; 0 runs until interrupted",
    )
    args = parser.parse_args()

    gui_pids = _desktop_gui_pids()
    if gui_pids:
        print(
            "Refusing to start while the desktop RobotLinuxDemo GUI is running "
            f"(PID(s): {gui_pids}). Close only the desktop GUI; keep "
            "RoboticsService and the headset app running.",
            file=sys.stderr,
        )
        return 2

    sides = SIDES if args.sides == "both" else (args.sides,)

    import xrobotoolkit_sdk as xrt

    print(f"Sending hand commands to udp://{args.host}:{args.port} at {args.rate:g} Hz")
    print("Hands reach hardware only if the bridge was launched enabled.")
    pipeline = None
    try:
        xrt.init()
        pipeline = HandPipeline(
            xrt,
            assets_dir=REPO_ROOT / "assets" / "linkerhand_l20",
            host=args.host,
            port=args.port,
            rate=args.rate,
            sides=sides,
            stale_timeout=args.stale_timeout,
            frozen_timeout=args.frozen_timeout,
        )
        print("SDK initialized. Waiting for hand tracking...")
        started = time.monotonic()
        deadline = started + args.duration if args.duration > 0 else None
        next_report = started + 2.0
        previous_sent = {side: 0 for side in sides}
        previous_sending = {side: False for side in sides}

        # Tick well above the send rate so per-side scheduling stays punctual.
        interval = 1.0 / 100.0
        while deadline is None or time.monotonic() < deadline:
            loop_started = time.monotonic()
            pipeline.tick(loop_started)

            for side in sides:
                status = pipeline.status.sides[side]
                if status.sending and not previous_sending[side]:
                    print(f"  {side}: tracking, sending")
                elif not status.sending and previous_sending[side]:
                    print(f"  {side}: stopped, {status.fault}")
                previous_sending[side] = status.sending

            now = time.monotonic()
            if now >= next_report:
                elapsed = now - (next_report - 2.0)
                parts = []
                for side in sides:
                    status = pipeline.status.sides[side]
                    rate = (status.sent - previous_sent[side]) / elapsed
                    previous_sent[side] = status.sent
                    detail = (
                        f"solve={status.solve_seconds * 1e3:.1f}ms"
                        if status.sending
                        else f"({status.fault})"
                    )
                    parts.append(f"{side}={rate:.1f}Hz {detail}")
                print(f"  t+{now - started:5.1f}s  " + "  ".join(parts))
                next_report = now + 2.0

            remaining = interval - (time.monotonic() - loop_started)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("\nStopped by operator.")
    finally:
        if pipeline is not None:
            pipeline.close()
        print("Closing the XRoboToolkit SDK.")
        xrt.close()
        print("SDK closed. The bridge watchdog will stop publishing.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
