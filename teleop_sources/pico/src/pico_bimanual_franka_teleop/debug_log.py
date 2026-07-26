"""Per-tick JSONL logging of the whole arm-following chain.

One row per 100 Hz control tick, capturing every stage between the tracker and
the command: the world-frame tracker pose, engagement, the mapped target, the
commanded configuration and its end-effector pose, and the measured robot
state. With all stages present, an offline analysis can attribute a following
deficit to the specific link that loses it: input, mapping, IK rate limiting,
or the real arm lagging the command.

Logging must never affect control: rows are buffered and flushed periodically,
and any I/O failure disables the logger rather than raising into the loop.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np
import pinocchio as pin

from .types import Pose, SIDES


def _pose_record(pose: Pose | None):
    if pose is None:
        return None
    return {
        "p": [round(float(v), 6) for v in pose.position],
        "r": [round(float(v), 6) for v in pin.log3(pose.rotation)],
    }


class FollowDebugLogger:
    """Append one row per control tick; safe to leave enabled for whole runs."""

    def __init__(self, path: str | Path, flush_every: int = 100) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("w", encoding="utf-8")
        self._flush_every = int(flush_every)
        self._rows = 0
        self._failed = False
        self._file.write(
            json.dumps(
                {
                    "schema": "follow-debug.v2",
                    "written_at": time.time(),
                    "fields": "t monotonic; q_measured, q_commanded 14 joints; "
                    "per side: engaged, raw_tracker/tracker/target/ee_cmd/"
                    "ee_meas poses with rotations as world-frame rotation "
                    "vectors",
                }
            )
            + "\n"
        )

    def record(
        self,
        now: float,
        q_measured,
        q_commanded,
        tracker_poses: dict,
        engaged: dict,
        targets: dict,
        ee_poses: dict,
        raw_tracker_poses: dict | None = None,
        measured_ee_poses: dict | None = None,
    ) -> None:
        if self._failed:
            return
        try:
            row = {
                "t": round(float(now), 4),
                "q_meas": [round(float(v), 5) for v in np.asarray(q_measured)],
                "q_cmd": [round(float(v), 5) for v in np.asarray(q_commanded)],
            }
            raw_tracker_poses = raw_tracker_poses or {}
            measured_ee_poses = measured_ee_poses or {}
            for side in SIDES:
                row[side] = {
                    "engaged": bool(engaged.get(side, False)),
                    "raw_tracker": _pose_record(raw_tracker_poses.get(side)),
                    "tracker": _pose_record(tracker_poses.get(side)),
                    "target": _pose_record(targets.get(side)),
                    # Keep `ee` for compatibility with v1 analysis tools.
                    "ee": _pose_record(ee_poses.get(side)),
                    "ee_cmd": _pose_record(ee_poses.get(side)),
                    "ee_meas": _pose_record(measured_ee_poses.get(side)),
                }
            self._file.write(json.dumps(row, separators=(",", ":")) + "\n")
            self._rows += 1
            if self._rows % self._flush_every == 0:
                self._file.flush()
        except Exception:  # noqa: BLE001 - logging must never break control
            self._failed = True
            try:
                self._file.close()
            except Exception:  # noqa: BLE001
                pass

    def close(self) -> None:
        if not self._failed:
            try:
                self._file.flush()
                self._file.close()
            except Exception:  # noqa: BLE001
                pass


class HandRetargetDebugLogger:
    """Buffered JSONL logger for live skeleton-to-joint fidelity analysis."""

    def __init__(self, path: str | Path, flush_every: int = 60) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("w", encoding="utf-8")
        self._flush_every = int(flush_every)
        self._rows = 0
        self._failed = False
        self._file.write(
            json.dumps(
                {
                    "schema": "hand-retarget-debug.v1",
                    "written_at": time.time(),
                    "fields": "t monotonic; side; canonical landmarks in metres; "
                    "emitted 21-joint qpos in radians; solver fidelity stats",
                }
            )
            + "\n"
        )

    def record(self, now: float, side: str, landmarks, qpos, stats: dict) -> None:
        if self._failed:
            return
        try:
            row = {
                "t": round(float(now), 4),
                "side": str(side),
                "landmarks": np.asarray(landmarks, dtype=float)
                .round(6)
                .tolist(),
                "qpos": np.asarray(qpos, dtype=float).round(5).tolist(),
                "stats": {
                    key: (
                        bool(value)
                        if isinstance(value, (bool, np.bool_))
                        else (
                            int(value)
                            if isinstance(value, (int, np.integer))
                            else round(float(value), 7)
                        )
                    )
                    for key, value in stats.items()
                },
            }
            self._file.write(json.dumps(row, separators=(",", ":")) + "\n")
            self._rows += 1
            if self._rows % self._flush_every == 0:
                self._file.flush()
        except Exception:  # noqa: BLE001 - logging must never break control
            self._failed = True
            try:
                self._file.close()
            except Exception:  # noqa: BLE001
                pass

    def close(self) -> None:
        if not self._failed:
            try:
                self._file.flush()
                self._file.close()
            except Exception:  # noqa: BLE001
                pass
