from pathlib import Path

import pytest
import yaml

from apps.arm_ui.absolute_recording import (
    AbsoluteTrajectoryStore,
    DualArmSample,
    build_absolute_trajectory,
)


def test_absolute_trajectory_is_task_named_and_written_atomically(tmp_path: Path):
    samples = tuple(
        DualArmSample.create(
            timestamp,
            {"left": [timestamp] * 7, "right": [-timestamp] * 7},
        )
        for timestamp in (4.0, 4.1, 4.3)
    )
    names = {
        side: [f"{side}_fr3_joint{index}" for index in range(1, 8)]
        for side in ("left", "right")
    }

    trajectory = build_absolute_trajectory("assembly", samples, names)
    path = AbsoluteTrajectoryStore(tmp_path).save(trajectory)

    assert path == tmp_path / "absolute_trajectories" / "assembly.yaml"
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert document["task"] == "assembly"
    assert document["label"] == "装配"
    assert [frame["time_sec"] for frame in document["samples"]] == pytest.approx(
        [0.0, 0.1, 0.3]
    )
    assert not tuple(path.parent.glob(".*.tmp"))
