#!/usr/bin/env python3
"""Capture MANUS geometry and O30i IK output without commanding hardware."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "teleop_sources" / "pico" / "src"))
sys.path.insert(0, str(REPO_ROOT / "teleop_sources" / "manus" / "python"))

from manus_teleop.o30i_retarget import O30IRetargeter  # noqa: E402
from manus_teleop.pipeline import (  # noqa: E402
    ManusBridge,
    canonical_landmarks,
)
from pico_bimanual_franka_teleop.hand_retarget import (  # noqa: E402
    CANONICAL_FINGERS,
)


def bend_angle(first: np.ndarray, second: np.ndarray) -> float:
    first_norm = float(np.linalg.norm(first))
    second_norm = float(np.linalg.norm(second))
    if first_norm < 1e-8 or second_norm < 1e-8:
        return math.nan
    cosine = float(np.dot(first, second) / (first_norm * second_norm))
    return math.acos(float(np.clip(cosine, -1.0, 1.0)))


def geometry_angles(landmarks: np.ndarray) -> dict[str, dict[str, float]]:
    result = {}
    for finger, chain in CANONICAL_FINGERS.items():
        points = landmarks[list(chain)]
        segments = np.diff(points, axis=0)
        result[finger] = {
            "proximal_bend": bend_angle(segments[0], segments[1]),
            "distal_bend": bend_angle(segments[1], segments[2]),
        }
    return result


def summarize(samples: list[dict], joint_names: list[str]) -> dict:
    qpos = np.asarray([sample["qpos"] for sample in samples], dtype=np.float64)
    summary = {
        "sample_count": len(samples),
        "qpos_mean": dict(zip(joint_names, np.mean(qpos, axis=0), strict=True)),
        "qpos_std": dict(zip(joint_names, np.std(qpos, axis=0), strict=True)),
        "geometry_mean": {},
        "geometry_std": {},
    }
    for finger in CANONICAL_FINGERS:
        for angle_name in ("proximal_bend", "distal_bend"):
            values = np.asarray(
                [sample["geometry"][finger][angle_name] for sample in samples],
                dtype=np.float64,
            )
            summary["geometry_mean"].setdefault(finger, {})[angle_name] = float(
                np.mean(values)
            )
            summary["geometry_std"].setdefault(finger, {})[angle_name] = float(
                np.std(values)
            )
    return summary


def capture_phase(
    bridge: ManusBridge,
    retargeter: O30IRetargeter,
    duration: float,
) -> list[dict]:
    samples = []
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        frame = bridge.read(timeout_s=0.1)
        if frame is None:
            continue
        landmarks = canonical_landmarks(frame)
        qpos, stats = retargeter.retarget(landmarks)
        samples.append(
            {
                "sequence": int(frame.sequence),
                "timestamp_ns": int(frame.timestamp_ns),
                "landmarks": landmarks.tolist(),
                "geometry": geometry_angles(landmarks),
                "qpos": qpos.tolist(),
                "stats": stats,
            }
        )
    if not samples:
        raise RuntimeError("no calibrated right MANUS frames were received")
    return samples


def wait_for_phase(phase: str, prompt: str, duration: float) -> None:
    expected = phase.replace("_", "-")
    print(prompt)
    while True:
        answer = input(
            f"Type {expected!r} and press Enter to start "
            f"{duration:g} seconds: "
        ).strip().lower()
        if answer == expected:
            return
        print(f"Waiting for the exact text {expected!r}; no capture started.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=4.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("/tmp/o30_manus_retarget_diagnostic.json"),
    )
    args = parser.parse_args()
    if args.seconds <= 0.0:
        parser.error("--seconds must be positive")

    library = (
        REPO_ROOT
        / "teleop_sources"
        / "manus"
        / "build"
        / "libmanus_skeleton_bridge.so"
    )
    calibration = REPO_ROOT / "teleop_sources" / "manus" / "config"
    urdf = (
        REPO_ROOT
        / "assets"
        / "linkerhand_o30i"
        / "right"
        / "linkerhand_o30i_right.urdf"
    )
    bridge = ManusBridge(library.resolve())
    retargeter = O30IRetargeter(
        urdf,
        "right",
        filter_alpha=1.0,
    )
    phases = {}
    try:
        bridge.connect(calibration.resolve())
        prompts = (
            ("straight", "Hold the MANUS hand fully straight and still."),
            ("half_curl", "Hold the MANUS hand at a half curl and keep it still."),
            ("fist", "Hold the MANUS hand in a fist and keep it still."),
        )
        for phase, prompt in prompts:
            wait_for_phase(phase, prompt, args.seconds)
            print(f"Capturing {phase}...")
            phases[phase] = capture_phase(bridge, retargeter, args.seconds)
    finally:
        retargeter.close()
        bridge.close()

    report = {
        "schema": "o30-manus-retarget-diagnostic.v1",
        "joint_names": retargeter.joint_names,
        "phases": {
            phase: {
                "summary": summarize(samples, retargeter.joint_names),
                "samples": samples,
            }
            for phase, samples in phases.items()
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, separators=(",", ":")),
        encoding="utf-8",
    )
    print(f"Saved diagnostic data to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
