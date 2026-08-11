import json
from pathlib import Path

import pytest

from apps.wuji_ui.models import (
    HandPose,
    ManusTrigger,
    WujiPoseRepository,
    summarize_gap_samples,
)


def names(side: str) -> tuple[str, ...]:
    return tuple(f"{side}_joint_{index}" for index in range(20))


def repository(path: Path) -> WujiPoseRepository:
    return WujiPoseRepository(
        path, {"left": names("left"), "right": names("right")}
    )


def test_left_and_right_poses_have_independent_names(tmp_path: Path) -> None:
    path = tmp_path / "poses.json"
    poses = repository(path)
    poses.save(HandPose.create("left_grasp", "left", [0.1] * 20))
    poses.save(HandPose.create("right_pinch", "right", [0.2] * 20))

    reloaded = repository(path)
    assert reloaded.pose("left", "left_grasp").qpos == pytest.approx([0.1] * 20)
    assert reloaded.pose("right", "right_pinch").qpos == pytest.approx([0.2] * 20)
    data = json.loads(path.read_text())
    assert data["units"] == "rad"
    assert data["joint_order"] == "device"


def test_pose_save_does_not_silently_overwrite(tmp_path: Path) -> None:
    poses = repository(tmp_path / "poses.json")
    poses.save(HandPose.create("pinch", "left", [0.1] * 20))
    with pytest.raises(ValueError, match="已存在"):
        poses.save(HandPose.create("pinch", "left", [0.2] * 20))


def test_rejects_pose_file_from_different_joint_contract(tmp_path: Path) -> None:
    path = tmp_path / "poses.json"
    repository(path).save(HandPose.create("pose", "left", [0.0] * 20))
    with pytest.raises(ValueError, match="关节顺序"):
        WujiPoseRepository(
            path,
            {
                "left": tuple(reversed(names("left"))),
                "right": names("right"),
            },
        )


def test_manus_trigger_round_trip_and_pose_reference(tmp_path: Path) -> None:
    path = tmp_path / "poses.json"
    poses = repository(path)
    poses.save(HandPose.create("pinch_pose", "right", [0.2] * 20))
    poses.save_trigger(
        ManusTrigger.create(
            "index_pinch",
            "right",
            "index",
            "pinch_pose",
            enter_max_m=0.025,
            exit_min_m=0.032,
            calibration={"sample_count": 100, "p95_m": 0.023},
        )
    )

    trigger = repository(path).triggers("right")[0]
    assert trigger.pose_name == "pinch_pose"
    assert trigger.enter_max_m == pytest.approx(0.025)
    with pytest.raises(ValueError, match="触发器引用"):
        poses.delete("right", "pinch_pose")


def test_gap_summary_uses_robust_percentiles_and_hysteresis() -> None:
    stats = summarize_gap_samples([0.010 + index * 0.001 for index in range(20)])
    assert stats["sample_count"] == 20
    assert stats["recommended_enter_max_m"] > stats["p95_m"]
    assert stats["recommended_exit_min_m"] > stats["recommended_enter_max_m"]
