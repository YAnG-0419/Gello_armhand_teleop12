import json

import numpy as np

from pico_bimanual_franka_teleop.debug_log import (
    FollowDebugLogger,
    HandRetargetDebugLogger,
)
from pico_bimanual_franka_teleop.types import Pose


def test_follow_debug_log_records_raw_and_both_fk_streams(tmp_path):
    path = tmp_path / "follow.jsonl"
    identity = Pose(np.array([1.0, 2.0, 3.0]), np.eye(3))
    logger = FollowDebugLogger(path, flush_every=1)
    logger.record(
        12.5,
        np.arange(14),
        np.arange(14) + 0.5,
        {"left": identity},
        {"left": True},
        {"left": identity},
        {"left": identity},
        raw_tracker_poses={"left": identity},
        measured_ee_poses={"left": identity},
    )
    logger.close()

    header, row = [
        json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()
    ]
    assert header["schema"] == "follow-debug.v2"
    assert row["left"]["raw_tracker"]["p"] == [1.0, 2.0, 3.0]
    assert row["left"]["tracker"]["p"] == [1.0, 2.0, 3.0]
    assert row["left"]["ee_cmd"] == row["left"]["ee"]
    assert row["left"]["ee_meas"]["p"] == [1.0, 2.0, 3.0]
    assert row["right"]["raw_tracker"] is None


def test_hand_debug_log_records_landmarks_commands_and_metrics(tmp_path):
    path = tmp_path / "hands.jsonl"
    logger = HandRetargetDebugLogger(path, flush_every=1)
    logger.record(
        4.2,
        "right",
        np.zeros((21, 3)),
        np.arange(21) / 10,
        {"success": True, "iterations": 3, "thumb_tip_error": 0.012345678},
    )
    logger.close()

    header, row = [
        json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()
    ]
    assert header["schema"] == "hand-retarget-debug.v1"
    assert row["side"] == "right"
    assert np.asarray(row["landmarks"]).shape == (21, 3)
    assert len(row["qpos"]) == 21
    assert row["stats"]["success"] is True
    assert row["stats"]["iterations"] == 3
    assert row["stats"]["thumb_tip_error"] == 0.0123457
