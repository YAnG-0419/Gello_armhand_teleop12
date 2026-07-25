#!/usr/bin/env python3
"""Retarget forwarded PICO hand skeletons and send hand commands onward.

Receives raw skeletons from whichever process owns the XRoboToolkit client and
emits L20 joint poses to `linker_hand_bridge`. It never opens an SDK client, so it
can run alongside arm teleoperation without breaking the one-client rule.

This exists as a separate process rather than a thread because retargeting both
hands costs about 17 ms and holds the GIL. Measured here, running it inside the arm
process turned a 100 Hz loop with a p99 of 10.18 ms into one with a p99 of 29 ms,
late on 23% of ticks. A separate interpreter removes that coupling entirely.

    skeletons in   udp 5571   <- teleop_dual_fr3.py --hands
    commands out   udp 5570   -> linker_hand_bridge

Liveness is judged here, on the same rules a live client would use: `isActive` must
be 1, because complete and plausible pose arrays keep being served after tracking is
lost, and the skeleton must be changing, because optical jitter means a hand held
deliberately still still moves while a frozen cache does not. A side that fails
either test simply stops being sent, so the bridge watchdog holds that hand.
"""

import argparse
import socket
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO_ROOT / "teleop_sources" / "pico" / "src"))

from pico_bimanual_franka_teleop.hand_input import SkeletonLiveness  # noqa: E402
from pico_bimanual_franka_teleop.hand_retarget import L20Retargeter  # noqa: E402
from pico_bimanual_franka_teleop.hand_skeleton_stream import (  # noqa: E402
    MAX_DATAGRAM_BYTES,
    decode_skeleton_packet,
)
from pico_bimanual_franka_teleop.hand_stream import build_hand_packet  # noqa: E402

SIDES = ("left", "right")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--listen-host", default="127.0.0.1")
    parser.add_argument("--listen-port", type=int, default=5571)
    parser.add_argument("--bridge-host", default="127.0.0.1")
    parser.add_argument("--bridge-port", type=int, default=5570)
    parser.add_argument("--sides", default="both", choices=("left", "right", "both"))
    parser.add_argument("--stale-timeout", type=float, default=0.25)
    parser.add_argument("--frozen-timeout", type=float, default=1.0)
    parser.add_argument(
        "--iterations",
        type=int,
        default=25,
        help="retargeting optimizer iterations per frame (default: 25)",
    )
    parser.add_argument("--log-period", type=float, default=2.0)
    args = parser.parse_args()
    if args.stale_timeout <= 0 or args.frozen_timeout <= 0 or args.log_period <= 0:
        parser.error("timeouts and --log-period must be positive")

    sides = SIDES if args.sides == "both" else (args.sides,)
    assets = REPO_ROOT / "assets" / "linkerhand_l20"
    retargeters = {
        side: L20Retargeter(
            assets / side / f"linkerhand_l20_{side}.urdf",
            side,
            max_iterations=args.iterations,
        )
        for side in sides
    }
    # The same liveness rules the in-process reader uses, not a second copy.
    trackers = {
        side: SkeletonLiveness(args.stale_timeout, args.frozen_timeout)
        for side in sides
    }
    last_sequence: dict[str, int | None] = {side: None for side in sides}

    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.bind((args.listen_host, args.listen_port))
    listener.settimeout(0.5)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(
        f"Listening for skeletons on udp://{args.listen_host}:{args.listen_port}; "
        f"sending commands to udp://{args.bridge_host}:{args.bridge_port}; "
        f"sides={sides}"
    )
    print("Nothing reaches the hands unless linker_hand_bridge was started enabled.", flush=True)

    outgoing = {side: 0 for side in sides}
    sent = {side: 0 for side in sides}
    received = {side: 0 for side in sides}
    solve_total = {side: 0.0 for side in sides}
    invalid = 0
    last_log = time.monotonic()
    try:
        while True:
            try:
                payload, _ = listener.recvfrom(MAX_DATAGRAM_BYTES + 1)
            except socket.timeout:
                payload = None
            now = time.monotonic()

            if payload is not None:
                try:
                    packet = decode_skeleton_packet(payload)
                except ValueError as error:
                    invalid += 1
                    if invalid <= 3:
                        print(f"  ignoring invalid skeleton packet: {error}", flush=True)
                    packet = None
                if packet is not None and packet.side in trackers:
                    side = packet.side
                    received[side] += 1
                    previous = last_sequence[side]
                    if previous is not None and packet.sequence <= previous:
                        landmarks = None
                        trackers[side].fault = "out-of-order skeleton"
                    else:
                        last_sequence[side] = packet.sequence
                        landmarks = trackers[side].accept(
                            packet.joints, packet.is_active, now
                        )
                    if landmarks is None:
                        retargeters[side].reset()
                    else:
                        started = time.monotonic()
                        qpos, _ = retargeters[side].retarget(landmarks)
                        solve_total[side] += time.monotonic() - started
                        sender.sendto(
                            build_hand_packet(
                                f"pico-hand-{side}",
                                outgoing[side],
                                time.time(),
                                side,
                                retargeters[side].joint_names,
                                qpos,
                            ),
                            (args.bridge_host, args.bridge_port),
                        )
                        outgoing[side] += 1
                        sent[side] += 1

            if now - last_log >= args.log_period:
                elapsed = now - last_log
                parts = []
                for side in sides:
                    solve_ms = (
                        solve_total[side] / sent[side] * 1e3 if sent[side] else 0.0
                    )
                    state = trackers[side].fault or "tracking"
                    parts.append(
                        f"{side}: rx={received[side] / elapsed:.1f}Hz "
                        f"tx={sent[side] / elapsed:.1f}Hz "
                        f"solve={solve_ms:.1f}ms {state}"
                    )
                    received[side] = 0
                    sent[side] = 0
                    solve_total[side] = 0.0
                print("  " + "; ".join(parts) + f"; invalid={invalid}", flush=True)
                last_log = now
    except KeyboardInterrupt:
        print("\nStopped by operator.", flush=True)
        return 0
    finally:
        listener.close()
        sender.close()
        for retargeter in retargeters.values():
            retargeter.close()


if __name__ == "__main__":
    raise SystemExit(main())
