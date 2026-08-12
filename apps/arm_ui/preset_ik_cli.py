"""Solve one recorded relative action through Arm UI's MoveIt IK path."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

from .recording import ActionStore
from .runtime import ArmRosRuntime


RESULT_PREFIX = "PRESET_IK_RESULT="


def _wait_until_ready(runtime: ArmRosRuntime, timeout: float = 8.0) -> None:
    deadline = time.monotonic() + timeout
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            for side in ("left", "right"):
                runtime.snapshot(side)
            runtime._lookup_tool_transform("left")
            runtime._lookup_tool_transform("right")
            return
        except RuntimeError as error:
            last_error = error
            time.sleep(0.05)
    raise RuntimeError(f"robot state/TF not ready: {last_error}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", default="/data/arm_ui")
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--action", required=True)
    parser.add_argument("--speed-scale", type=float, default=0.5)
    parser.add_argument("--max-joint-speed", type=float, default=0.5)
    args = parser.parse_args()
    if not math.isfinite(args.speed_scale) or not 0.05 <= args.speed_scale <= 1.0:
        raise ValueError("speed scale must be between 5% and 100%")
    if not math.isfinite(args.max_joint_speed) or args.max_joint_speed <= 0.0:
        raise ValueError("maximum joint speed must be positive")

    store = ActionStore(Path(args.data_root))
    action = store.load(args.side, args.action)
    runtime = ArmRosRuntime(robot_type="fr3", service_timeout_sec=8.0)
    try:
        _wait_until_ready(runtime)
        start_q = tuple(
            value
            for side in ("left", "right")
            for value in runtime.snapshot(side).positions
        )
        solution = runtime.solve_relative_action(action)
        times = solution.time_sec
        initial_offset = max(
            abs(value - start)
            for value, start in zip(
                solution.positions[0], solution.start_positions, strict=True
            )
        )
        if initial_offset > 0.05:
            raise RuntimeError(
                f"first IK frame is {math.degrees(initial_offset):.1f} deg "
                "from the measured start"
            )
        for index in range(1, len(times)):
            elapsed = (times[index] - times[index - 1]) / args.speed_scale
            speed = max(
                abs(current - previous) / elapsed
                for current, previous in zip(
                    solution.positions[index],
                    solution.positions[index - 1],
                    strict=True,
                )
            )
            if speed > args.max_joint_speed:
                raise RuntimeError(
                    f"trajectory exceeds {args.max_joint_speed:.2f} rad/s "
                    f"at frame {index + 1}: {speed:.3f} rad/s"
                )
        payload = {
            "name": action.name,
            "side": action.side,
            "time_sec": list(times),
            "positions": [list(values) for values in solution.positions],
            "start_q": list(start_q),
            "speed_scale": args.speed_scale,
            "checked_frames": solution.validation.checked_frames,
            "max_joint_step_rad": solution.validation.max_joint_step_rad,
        }
        print(RESULT_PREFIX + json.dumps(payload, separators=(",", ":")))
        return 0
    finally:
        runtime.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
