#!/usr/bin/env python3
"""Interactively tune the fixed thumb opposition pose on the physical hand.

Streams a full-hand pose to the running linker_hand_bridge (same packet path
as teleoperation, so the watchdog and slew limiter stay active) while single
keys adjust the pose live:

  a / d   thumb cmc yaw   - / +
  w / s   thumb cmc roll  + / -
  j / k   thumb curl      - / +   (drives cmc pitch + MCP/IP flex together,
                                   exactly like fixed-opposition teleop)
  i       toggle the recorded index-pinch finger pose
  f       toggle all four fingers between open and half-curled around a tool
  p       print the current values
  q       quit and print the values to paste into hand_retarget.py

Requires the hand-control container (linker_hand_bridge) to be running.
Refuses to start while a PICO teleop sender is running, exactly like
inspect_thumb_configuration.py. The FR3 arms are never involved.
"""

from __future__ import annotations

import argparse
import os
import select
import socket
import sys
import termios
import time
import tty
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO_ROOT / "teleop_sources" / "pico" / "src"))

from pico_bimanual_franka_teleop.env_guard import ensure_ros_free_process  # noqa: E402

ensure_ros_free_process()

from pico_bimanual_franka_teleop.hand_retarget import (  # noqa: E402
    L20Retargeter,
    THUMB_OPPOSITION_YAW_ROLL,
)
from pico_bimanual_franka_teleop.hand_stream import build_hand_packet  # noqa: E402

FINGER_HALF_CURL = 0.9
ANGLE_STEP = 0.02
CURL_STEP = 0.02


def teleop_processes() -> list[str]:
    matches = []
    for process in Path("/proc").iterdir():
        if not process.name.isdigit():
            continue
        try:
            command = (process / "cmdline").read_bytes().replace(b"\0", b" ")
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if any(
            name in command
            for name in (
                b"teleop_dual_fr3.py",
                b"teleop_hands.py",
                b"teleop_manus_hands.py",
            )
        ):
            matches.append(command.decode(errors="replace").strip())
    return matches


class ThumbTuner:
    def __init__(self, retargeter: L20Retargeter, *, pinch: bool = False) -> None:
        self.retargeter = retargeter
        self.names = list(retargeter.joint_names)
        self.index = {name: i for i, name in enumerate(self.names)}
        self.yaw, self.roll = (
            (0.67, 0.94)
            if pinch and retargeter.side == "left"
            else tuple(
                float(v) for v in THUMB_OPPOSITION_YAW_ROLL[retargeter.side]
            )
        )
        self.curl = 0.42 if pinch and retargeter.side == "left" else 0.5
        self.pinch_pose = bool(pinch)
        self.fingers_curled = False

    def _limit(self, name: str) -> tuple[float, float]:
        i = self.index[name]
        return float(self.retargeter.lower[i]), float(self.retargeter.upper[i])

    def qpos(self) -> np.ndarray:
        values = np.zeros(self.retargeter.dof, dtype=np.float64)

        def put(name: str, value: float) -> None:
            low, high = self._limit(name)
            values[self.index[name]] = float(np.clip(value, low, high))

        put("thumb_cmc_yaw", self.yaw)
        put("thumb_cmc_roll", self.roll)
        pitch_low, pitch_high = self._limit("thumb_cmc_pitch")
        put("thumb_cmc_pitch", pitch_low + self.curl * (pitch_high - pitch_low))
        mcp_low, mcp_high = self._limit("thumb_mcp")
        put("thumb_mcp", mcp_low + self.curl * (mcp_high - mcp_low))
        if self.pinch_pose:
            put("index_mcp_roll", -0.17)
            put("index_mcp_pitch", 0.94)
            put("index_pip", 0.57)
        elif self.fingers_curled:
            for finger in ("index", "middle", "ring", "pinky"):
                put(f"{finger}_mcp_pitch", FINGER_HALF_CURL)
                put(f"{finger}_pip", FINGER_HALF_CURL)
        # Fill every URDF mimic joint from its source so the bridge sees the
        # same fully-expanded pose teleoperation emits.
        by_name = dict(zip(self.names, values))
        for name, (source, multiplier, offset) in self.retargeter._mimics.items():
            value = multiplier * by_name[source] + offset
            low, high = self._limit(name)
            values[self.index[name]] = float(np.clip(value, low, high))
            by_name[name] = values[self.index[name]]
        return values

    def status(self) -> str:
        yaw_low, yaw_high = self._limit("thumb_cmc_yaw")
        roll_low, roll_high = self._limit("thumb_cmc_roll")
        return (
            f"yaw {self.yaw:5.2f} [{yaw_low:.2f},{yaw_high:.2f}]   "
            f"roll {self.roll:5.2f} [{roll_low:.2f},{roll_high:.2f}]   "
            f"curl {self.curl:4.2f}   "
            f"fingers "
            f"{'index-pinch' if self.pinch_pose else ('half-curled' if self.fingers_curled else 'open')}"
        )

    def handle(self, key: str) -> bool:
        """Apply one key; return False when the session should end."""
        yaw_low, yaw_high = self._limit("thumb_cmc_yaw")
        roll_low, roll_high = self._limit("thumb_cmc_roll")
        if key == "a":
            self.yaw = float(np.clip(self.yaw - ANGLE_STEP, yaw_low, yaw_high))
        elif key == "d":
            self.yaw = float(np.clip(self.yaw + ANGLE_STEP, yaw_low, yaw_high))
        elif key == "s":
            self.roll = float(np.clip(self.roll - ANGLE_STEP, roll_low, roll_high))
        elif key == "w":
            self.roll = float(np.clip(self.roll + ANGLE_STEP, roll_low, roll_high))
        elif key == "j":
            self.curl = float(np.clip(self.curl - CURL_STEP, 0.0, 1.0))
        elif key == "k":
            self.curl = float(np.clip(self.curl + CURL_STEP, 0.0, 1.0))
        elif key == "i":
            self.pinch_pose = not self.pinch_pose
            self.fingers_curled = False
        elif key == "f":
            self.fingers_curled = not self.fingers_curled
            self.pinch_pose = False
        elif key == "q":
            return False
        return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", default="right", choices=("left", "right"))
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument(
        "--port",
        type=int,
        default=5570,
        help="where linker_hand_bridge listens (default: 5570)",
    )
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument(
        "--pinch",
        action="store_true",
        help="start from the recorded left index-pinch finger/thumb anchor",
    )
    args = parser.parse_args()

    running = teleop_processes()
    if running:
        print(
            "Refusing to run while a PICO teleop sender is running:",
            file=sys.stderr,
        )
        for command in running:
            print(f"  {command}", file=sys.stderr)
        return 2

    assets = REPO_ROOT / "assets" / "linkerhand_l20"
    retargeter = L20Retargeter(
        assets / args.side / f"linkerhand_l20_{args.side}.urdf", args.side
    )
    tuner = ThumbTuner(retargeter, pinch=args.pinch)

    print(__doc__)
    print("The hand WILL move. Keep it clear of the arm and the table.")
    print(tuner.status())

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    address = (args.host, args.port)
    interval = 1.0 / float(args.rate)
    sequence = 0
    stream_id = f"thumb-opposition-tuner-{args.side}"

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        alive = True
        while alive:
            started = time.monotonic()
            while select.select([fd], [], [], 0)[0]:
                key = os.read(fd, 1).decode(errors="ignore").lower()
                if not key:
                    break
                before = tuner.status()
                alive = tuner.handle(key)
                if key == "p" or tuner.status() != before:
                    print("\r" + tuner.status())
                if not alive:
                    break
            sock.sendto(
                build_hand_packet(
                    stream_id,
                    sequence,
                    time.time(),
                    args.side,
                    tuner.names,
                    tuner.qpos(),
                ),
                address,
            )
            sequence += 1
            time.sleep(max(0.0, interval - (time.monotonic() - started)))
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        sock.close()

    print()
    print("Final values - paste into hand_retarget.py to keep them:")
    label = (
        "MANUS_LEFT_PINCH_OPPOSITION"
        if args.side == "left" and args.pinch
        else f'THUMB_OPPOSITION_YAW_ROLL["{args.side}"] entry'
    )
    print(f"  {label}: ({tuner.yaw:.2f}, {tuner.roll:.2f})")
    print(f"  pinch curl fraction: {tuner.curl:.2f}")
    print(
        "Stream stopped; the bridge watchdog now stops publishing and the "
        "hand holds its last slewed position."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
