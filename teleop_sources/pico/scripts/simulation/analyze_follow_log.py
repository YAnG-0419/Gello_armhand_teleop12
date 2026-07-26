#!/usr/bin/env python3
import sys

# A sourced ROS environment leaves /opt/ros/... on PYTHONPATH; see env_guard.
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO_ROOT / "teleop_sources" / "pico" / "src"))

from pico_bimanual_franka_teleop.env_guard import ensure_ros_free_process  # noqa: E402

ensure_ros_free_process()

"""Attribute following deficits to the stage that loses them.

Reads a --debug-log recording and reports, per side and per engaged segment:

  raw -> tracker      optical wrist noise removed by the EMA
  tracker -> target   the mapper; must be exactly one-to-one
  target  -> EE(cmd)  IK and its joint-speed clamp; lag and amplitude loss here
                      mean the commanded motion never asked the arm to go
  EE(cmd) -> EE(meas) real-arm following through the gateway slew limit and
                      impedance controller

plus the operator's actual hand speeds, how often the commanded configuration
moves at the clamp, and how close joints come to their limits.
"""

import argparse  # noqa: E402
import json  # noqa: E402

import numpy as np  # noqa: E402

SIDES = ("left", "right")
CLAMP_DEFAULT = 0.5  # keep in sync with host.max_joint_speed in config/pico.yaml


def load(path):
    rows = []
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            if "schema" in row:
                continue
            rows.append(row)
    return rows


def segments(rows, side):
    """Contiguous stretches where the side is engaged with a target."""
    out = []
    current = []
    for row in rows:
        entry = row.get(side) or {}
        if entry.get("engaged") and entry.get("target") and entry.get("tracker"):
            current.append(row)
        elif current:
            out.append(current)
            current = []
    if current:
        out.append(current)
    return out


def positions(seg, side, key):
    return np.array([row[side][key]["p"] for row in seg], dtype=float)


def rotations(seg, side, key):
    return np.array([row[side][key]["r"] for row in seg], dtype=float)


def has_pose(seg, side, key):
    return all((row.get(side) or {}).get(key) is not None for row in seg)


def detrend(values):
    """Remove constant velocity so residuals expose tremor, not intended motion."""
    values = np.asarray(values, dtype=float)
    samples = np.arange(len(values), dtype=float)
    design = np.column_stack((samples, np.ones(len(samples))))
    trend = design @ np.linalg.lstsq(design, values, rcond=None)[0]
    return values - trend


def quiet_windows(seg, side, count=10, duration=1.0):
    """Select non-overlapping seconds with the least filtered-hand movement."""
    times = np.array([row["t"] for row in seg], dtype=float)
    dt = np.median(np.diff(times))
    size = max(10, int(round(duration / dt)))
    if len(seg) < size:
        return []
    tracker = positions(seg, side, "tracker")
    spans = np.array(
        [
            np.linalg.norm(np.ptp(tracker[start : start + size], axis=0))
            for start in range(len(seg) - size + 1)
        ]
    )
    selected = []
    for start in np.argsort(spans):
        if all(abs(int(start) - previous) >= size for previous in selected):
            selected.append(int(start))
        if len(selected) >= count:
            break
    return [(start, size, spans[start]) for start in sorted(selected)]


def report_quiet_jitter(seg, side):
    windows = quiet_windows(seg, side)
    if not windows:
        return
    print(
        "        quiet-window filtered-hand span range "
        f"{min(item[2] for item in windows)*1e3:.1f}-"
        f"{max(item[2] for item in windows)*1e3:.1f} mm"
    )
    for key in ("raw_tracker", "tracker", "target", "ee_cmd", "ee_meas"):
        if not has_pose(seg, side, key):
            continue
        rms = []
        span = []
        for start, size, _ in windows:
            residual = detrend(positions(seg[start : start + size], side, key))
            rms.append(np.sqrt(np.mean(np.sum(residual**2, axis=1))) * 1e3)
            span.append(np.linalg.norm(np.ptp(residual, axis=0)) * 1e3)
        print(
            f"        quiet {key:<11} detrended RMS p50/p95 "
            f"{np.median(rms):.2f}/{np.percentile(rms, 95):.2f} mm; "
            f"span {np.median(span):.2f}/{np.percentile(span, 95):.2f} mm"
        )
        angular_rms = []
        angular_span = []
        for start, size, _ in windows:
            # Within a quiet one-second window, detrending the logged rotation
            # vectors is a stable small-angle tremor estimate.
            residual = detrend(rotations(seg[start : start + size], side, key))
            angular_rms.append(
                np.sqrt(np.mean(np.sum(residual**2, axis=1))) * 1e3
            )
            angular_span.append(
                np.linalg.norm(np.ptp(residual, axis=0)) * 1e3
            )
        print(
            f"        quiet {key:<11} angular RMS p50/p95 "
            f"{np.median(angular_rms):.2f}/"
            f"{np.percentile(angular_rms, 95):.2f} mrad; "
            f"span {np.median(angular_span):.2f}/"
            f"{np.percentile(angular_span, 95):.2f} mrad"
        )
    for key in ("q_cmd", "q_meas"):
        rms = []
        span = []
        for start, size, _ in windows:
            values = np.array(
                [row[key] for row in seg[start : start + size]], dtype=float
            )
            residual = detrend(values)
            rms.append(
                np.max(np.sqrt(np.mean(residual**2, axis=0))) * 1e3
            )
            span.append(np.max(np.ptp(residual, axis=0)) * 1e3)
        print(
            f"        quiet {key:<11} worst-joint RMS p50/p95 "
            f"{np.median(rms):.2f}/{np.percentile(rms, 95):.2f} mrad; "
            f"span {np.median(span):.2f}/{np.percentile(span, 95):.2f} mrad"
        )


def analyze(path, clamp):
    rows = load(path)
    if not rows:
        print("empty log")
        return 1
    times = np.array([row["t"] for row in rows])
    dt = np.median(np.diff(times)) if len(times) > 1 else 0.01
    print(f"{len(rows)} ticks over {times[-1] - times[0]:.1f} s (median dt {dt*1e3:.1f} ms)")

    q_cmd = np.array([row["q_cmd"] for row in rows], dtype=float)
    q_meas = np.array([row["q_meas"] for row in rows], dtype=float)
    speed = np.abs(np.diff(q_cmd, axis=0)) / np.diff(times)[:, None]
    saturated = float(np.mean(speed.max(axis=1) > clamp * 0.98) * 100)
    print(f"commanded joint speed at the {clamp:g} rad/s clamp on {saturated:.1f}% of ticks")
    cmd_vs_meas = np.abs(q_cmd - q_meas).max(axis=1)
    print(
        f"command minus measured joints: median "
        f"{np.median(cmd_vs_meas):.3f} rad, p95 {np.percentile(cmd_vs_meas, 95):.3f} rad"
    )

    for side in SIDES:
        segs = [s for s in segments(rows, side) if len(s) >= 100]
        print()
        print(f"=== {side}: {len(segs)} engaged segment(s) >= 1 s ===")
        for index, seg in enumerate(segs):
            t = np.array([row["t"] for row in seg])
            tracker = positions(seg, side, "tracker")
            target = positions(seg, side, "target")
            ee_cmd_key = "ee_cmd" if has_pose(seg, side, "ee_cmd") else "ee"
            ee_cmd = positions(seg, side, ee_cmd_key)

            hand_v = np.linalg.norm(np.diff(tracker, axis=0), axis=1) / np.diff(t)
            hand_v = hand_v[np.isfinite(hand_v)]

            # tracker -> target: subtract each stream's own start (anchors)
            tracker_delta = tracker - tracker[0]
            target_delta = target - target[0]
            mapper_err = np.linalg.norm(tracker_delta - target_delta, axis=1)

            ik_err = np.linalg.norm(ee_cmd - target, axis=1)

            tracker_span = tracker_delta.max(axis=0) - tracker_delta.min(axis=0)
            ee_span = (
                (ee_cmd - ee_cmd[0]).max(axis=0)
                - (ee_cmd - ee_cmd[0]).min(axis=0)
            )
            with np.errstate(divide="ignore", invalid="ignore"):
                transfer = np.where(tracker_span > 0.02, ee_span / tracker_span, np.nan)

            print(
                f"  seg{index}: {t[-1] - t[0]:5.1f} s  hand speed p50/p90 "
                f"{np.median(hand_v):.2f}/{np.percentile(hand_v, 90):.2f} m/s"
            )
            print(
                f"        mapper (tracker->target) error max {mapper_err.max()*1e3:.1f} mm"
                f"   <- must be ~0"
            )
            print(
                f"        IK lag (target->EE cmd)  median {np.median(ik_err)*1e3:.0f} mm"
                f"  p95 {np.percentile(ik_err, 95)*1e3:.0f} mm  max {ik_err.max()*1e3:.0f} mm"
            )
            print(
                f"        amplitude transfer x/y/z: "
                + "/".join("-" if not np.isfinite(r) else f"{r*100:.0f}%" for r in transfer)
            )
            if has_pose(seg, side, "raw_tracker"):
                raw = positions(seg, side, "raw_tracker")
                filter_delta = np.linalg.norm(raw - tracker, axis=1)
                raw_step = np.linalg.norm(np.diff(raw, axis=0), axis=1)
                filtered_step = np.linalg.norm(np.diff(tracker, axis=0), axis=1)
                print(
                    f"        EMA raw->filtered offset median "
                    f"{np.median(filter_delta)*1e3:.1f} mm  "
                    f"p95 {np.percentile(filter_delta, 95)*1e3:.1f} mm"
                )
                print(
                    f"        per-tick wrist step p95 raw/filtered "
                    f"{np.percentile(raw_step, 95)*1e3:.1f}/"
                    f"{np.percentile(filtered_step, 95)*1e3:.1f} mm"
                )
            if has_pose(seg, side, "ee_meas"):
                ee_meas = positions(seg, side, "ee_meas")
                following_err = np.linalg.norm(ee_meas - ee_cmd, axis=1)
                print(
                    f"        EE commanded->measured error median "
                    f"{np.median(following_err)*1e3:.1f} mm  "
                    f"p95 {np.percentile(following_err, 95)*1e3:.1f} mm  "
                    f"max {following_err.max()*1e3:.1f} mm"
                )
            report_quiet_jitter(seg, side)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log")
    parser.add_argument("--clamp", type=float, default=CLAMP_DEFAULT)
    args = parser.parse_args()
    return analyze(args.log, args.clamp)


if __name__ == "__main__":
    raise SystemExit(main())
