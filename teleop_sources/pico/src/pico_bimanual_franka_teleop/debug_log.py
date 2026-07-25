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
                    "schema": "follow-debug.v1",
                    "written_at": time.time(),
                    "fields": "t monotonic; q_measured, q_commanded 14 joints; "
                    "per side: engaged, tracker/target/ee poses with rotations "
                    "as world-frame rotation vectors",
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
    ) -> None:
        if self._failed:
            return
        try:
            row = {
                "t": round(float(now), 4),
                "q_meas": [round(float(v), 5) for v in np.asarray(q_measured)],
                "q_cmd": [round(float(v), 5) for v in np.asarray(q_commanded)],
            }
            for side in SIDES:
                row[side] = {
                    "engaged": bool(engaged.get(side, False)),
                    "tracker": _pose_record(tracker_poses.get(side)),
                    "target": _pose_record(targets.get(side)),
                    "ee": _pose_record(ee_poses.get(side)),
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
