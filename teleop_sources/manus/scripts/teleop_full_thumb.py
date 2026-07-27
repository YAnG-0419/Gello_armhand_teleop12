#!/usr/bin/env python3
"""Retarget the right MANUS raw skeleton with every L20 thumb DoF free."""

from __future__ import annotations

import argparse
import socket
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
PICO_SRC = REPO_ROOT / "teleop_sources" / "pico" / "src"
sys.path.insert(0, str(PICO_SRC))

from pico_bimanual_franka_teleop.env_guard import ensure_ros_free_process  # noqa: E402

ensure_ros_free_process()

from pico_bimanual_franka_teleop.hand_landmarks import chirality, palm_scale  # noqa: E402
from pico_bimanual_franka_teleop.hand_retarget import (  # noqa: E402
    CANONICAL_FINGERS,
    L20Retargeter,
    chain_bend_angle,
)
from pico_bimanual_franka_teleop.hand_stream import build_hand_packet  # noqa: E402

from manus_teleop.pipeline import ManusBridge, canonical_landmarks  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5570)
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--timeout", type=float, default=0.25)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument(
        "--send",
        action="store_true",
        help="send UDP commands; default is solve-and-print only",
    )
    parser.add_argument(
        "--library",
        type=Path,
        default=REPO_ROOT
        / "teleop_sources"
        / "manus"
        / "build"
        / "libmanus_skeleton_bridge.so",
    )
    parser.add_argument(
        "--calibration-dir",
        type=Path,
        default=REPO_ROOT / "teleop_sources" / "manus" / "config",
    )
    args = parser.parse_args()
    if not 0.0 < args.rate <= 60.0:
        parser.error("--rate must be in (0, 60]")
    if args.timeout <= 0.0:
        parser.error("--timeout must be positive")
    if args.duration < 0.0:
        parser.error("--duration must not be negative")
    return args


def main() -> int:
    args = parse_args()
    bridge = ManusBridge(args.library.resolve())
    retargeter = L20Retargeter(
        REPO_ROOT / "assets" / "linkerhand_l20" / "right" / "linkerhand_l20_right.urdf",
        "right",
        thumb_opposition_fixed=None,
    )
    output = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    address = (args.host, args.port)
    sequence = 0
    period = 1.0 / args.rate
    next_send = 0.0
    started = time.monotonic()
    next_report = started + 2.0
    report_sent = 0
    diagnosed = False

    try:
        bridge.connect(args.calibration_dir.resolve())
        print("Powered by Manus. Full-thumb right skeleton retargeting connected.")
        while args.duration <= 0.0 or time.monotonic() - started < args.duration:
            frame = bridge.read(args.timeout)
            if frame is None:
                retargeter.reset()
                print("MANUS frame timeout; output stopped.", file=sys.stderr)
                continue
            now = time.monotonic()
            if now < next_send:
                continue
            points = canonical_landmarks(frame)
            if not diagnosed:
                print(
                    f"canonical palm_scale={palm_scale(points):.4f}m "
                    f"chirality={chirality(points):+.3e}"
                )
                for finger, indices in CANONICAL_FINGERS.items():
                    chain = points[list(indices)]
                    lengths = np.linalg.norm(np.diff(chain, axis=0), axis=1)
                    print(
                        f"  {finger}: lengths="
                        + ",".join(f"{value:.4f}" for value in lengths)
                        + f"m bend={chain_bend_angle(chain):.3f}rad"
                    )
                diagnosed = True
            qpos, stats = retargeter.retarget(points)
            if args.send:
                output.sendto(
                    build_hand_packet(
                        "manus-right-full-thumb",
                        sequence,
                        time.time(),
                        "right",
                        retargeter.joint_names,
                        qpos,
                    ),
                    address,
                )
            sequence += 1
            report_sent += 1
            if next_send <= 0.0 or now - next_send >= period:
                next_send = now + period
            else:
                next_send += period
            if now >= next_report:
                values = dict(zip(retargeter.joint_names, qpos))
                print(
                    f"sent={report_sent / 2.0:.1f}Hz "
                    f"thumb yaw={values['thumb_cmc_yaw']:.2f} "
                    f"roll={values['thumb_cmc_roll']:.2f} "
                    f"pitch={values['thumb_cmc_pitch']:.2f} "
                    f"mcp={values['thumb_mcp']:.2f} "
                    f"dip={values['thumb_dip']:.2f} "
                    f"human_bend={stats['thumb_bend']:.2f} "
                    f"tip_err={stats['thumb_tip_error'] * 1000:.1f}mm"
                )
                report_sent = 0
                next_report = now + 2.0
    except KeyboardInterrupt:
        print("\nStopped by operator.")
    finally:
        output.close()
        retargeter.close()
        bridge.close()
        print("Stopped; the LinkerHand watchdog will hold position.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
