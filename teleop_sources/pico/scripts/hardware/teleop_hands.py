#!/usr/bin/env python3
"""Drive the Linker Hands from live PICO optical hand tracking.

Hand-only teleoperation. This process owns the single XRoboToolkit SDK client,
so it must not run at the same time as `teleop_dual_fr3.py`, which owns its own
client for the controllers or wrist trackers. For simultaneous arm and hand
control the acquisition has to be shared inside one process instead; hand
skeletons and Object Motion Tracking do coexist in one client, verified on
hardware, but two Python clients have never been shown safe.

Sends hand qpos datagrams to `linker_hand_bridge`. Nothing reaches the hands
until that bridge is launched with `enabled:=true`, so this script is safe to run
against a dry-run bridge.

Each side is independent. When a hand stops being usable this stops sending for
that side, the bridge's watchdog stops publishing, and the hand holds position;
there is no automatic return-home. Reacquisition re-anchors on measured state.
"""

import argparse
import socket
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO_ROOT / "teleop_sources" / "pico" / "src"))

from pico_bimanual_franka_teleop.hand_input import HandSkeletonReader  # noqa: E402
from pico_bimanual_franka_teleop.hand_retarget import L20Retargeter  # noqa: E402
from pico_bimanual_franka_teleop.hand_stream import build_hand_packet  # noqa: E402

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
    parser.add_argument("--port", type=int, default=5570)
    parser.add_argument(
        "--rate",
        type=float,
        default=30.0,
        help="datagrams per second per side; keep at or below 60 (default: 30)",
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
    if args.rate <= 0 or args.rate > 60.0:
        parser.error("--rate must be in (0, 60]")
    if args.stale_timeout <= 0 or args.frozen_timeout <= 0:
        parser.error("timeouts must be positive")

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
    assets = REPO_ROOT / "assets" / "linkerhand_l20"

    import xrobotoolkit_sdk as xrt

    retargeters = {
        side: L20Retargeter(assets / side / f"linkerhand_l20_{side}.urdf", side)
        for side in sides
    }
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sequence = {side: 0 for side in sides}
    sending = {side: False for side in sides}
    interval = 1.0 / args.rate

    print(f"Sending hand qpos to udp://{args.host}:{args.port} at {args.rate:g} Hz")
    print("Hands reach hardware only if the bridge was launched with enabled:=true.")
    try:
        xrt.init()
        reader = HandSkeletonReader(
            xrt,
            stale_timeout=args.stale_timeout,
            frozen_timeout=args.frozen_timeout,
        )
        print("SDK initialized. Waiting for hand tracking...")
        started = time.monotonic()
        deadline = started + args.duration if args.duration > 0 else None
        next_report = started + 2.0
        sent = {side: 0 for side in sides}

        while deadline is None or time.monotonic() < deadline:
            loop_started = time.monotonic()
            samples = reader.sample(now=loop_started)
            for side in sides:
                sample = samples[side]
                if sample is None:
                    if sending[side]:
                        sending[side] = False
                        print(f"  {side}: stopped, {reader.faults[side]}")
                    # Deliberately send nothing. The bridge watchdog takes over.
                    retargeters[side].reset()
                    continue
                if not sending[side]:
                    sending[side] = True
                    print(f"  {side}: tracking, sending")
                qpos, _ = retargeters[side].retarget(sample.landmarks)
                sock.sendto(
                    build_hand_packet(
                        f"pico-hand-{side}",
                        sequence[side],
                        time.time(),
                        side,
                        retargeters[side].joint_names,
                        qpos,
                    ),
                    (args.host, args.port),
                )
                sequence[side] += 1
                sent[side] += 1

            now = time.monotonic()
            if now >= next_report:
                elapsed = now - (next_report - 2.0)
                status = "  ".join(
                    f"{side}={sent[side] / elapsed:.1f}Hz"
                    + ("" if sending[side] else f" ({reader.faults[side]})")
                    for side in sides
                )
                print(f"  t+{now - started:5.1f}s  {status}")
                sent = {side: 0 for side in sides}
                next_report = now + 2.0

            remaining = interval - (time.monotonic() - loop_started)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("\nStopped by operator.")
    finally:
        sock.close()
        for retargeter in retargeters.values():
            retargeter.close()
        print("Closing the XRoboToolkit SDK.")
        xrt.close()
        print("SDK closed. The bridge watchdog will stop publishing.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
