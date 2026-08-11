import math

import pytest

from apps.arm_ui.recording import (
    ActionStore,
    MotionSample,
    build_recorded_action,
    compose_pose,
)


def _sample(time_sec: float, joint4: float, x: float) -> MotionSample:
    return MotionSample.create(
        time_sec,
        [0.0, 0.0, 0.0, joint4, 0.0, 0.0, 0.0],
        [1.0 + x, 2.0, 3.0],
        [0.0, 0.0, 0.0, 1.0],
    )


def test_builds_relative_motion_and_detects_active_joint() -> None:
    action = build_recorded_action(
        name="scoop",
        side="left",
        base_frame="world",
        tool_frame="left_fr3_link8",
        samples=[
            _sample(0.0, 0.0, 0.0),
            _sample(0.1, 0.0, 0.0),
            _sample(0.2, 0.1, 0.02),
            _sample(0.3, 0.2, 0.04),
            _sample(0.4, 0.2, 0.04),
        ],
    )

    assert action.active_joints == (4,)
    assert action.relative_frames[0].position == pytest.approx((0.0, 0.0, 0.0))
    assert action.relative_frames[-1].position == pytest.approx((0.04, 0.0, 0.0))
    assert action.relative_frames[-1].joint_delta[3] == pytest.approx(0.2)


def test_relative_position_is_expressed_in_start_tool_frame() -> None:
    half = math.sqrt(0.5)
    samples = [
        MotionSample.create(0.0, [0.0] * 7, [0.0, 0.0, 0.0], [0, 0, half, half]),
        MotionSample.create(0.1, [0, 0, 0, 0.1, 0, 0, 0], [0.0, 1.0, 0.0], [0, 0, half, half]),
        MotionSample.create(0.2, [0, 0, 0, 0.2, 0, 0, 0], [0.0, 2.0, 0.0], [0, 0, half, half]),
    ]
    action = build_recorded_action(
        name="rotated",
        side="right",
        base_frame="world",
        tool_frame="right_fr3_link8",
        samples=samples,
        trim_padding_sec=1.0,
    )

    assert action.relative_frames[-1].position == pytest.approx((2.0, 0.0, 0.0))


def test_action_yaml_round_trip_summary(tmp_path) -> None:
    action = build_recorded_action(
        name="盛取动作",
        side="left",
        base_frame="world",
        tool_frame="left_fr3_link8",
        samples=[_sample(0.0, 0.0, 0.0), _sample(0.1, 0.1, 0.01), _sample(0.2, 0.2, 0.02)],
        trim_padding_sec=1.0,
    )
    store = ActionStore(tmp_path)
    path = store.save(action)

    assert path.exists()
    assert store.summaries("left")[0]["name"] == "盛取动作"
    loaded = store.load("left", "盛取动作")
    assert loaded.relative_frames == action.relative_frames
    assert loaded.raw_samples == action.raw_samples
    with pytest.raises(ValueError, match="已存在"):
        store.save(action)


def test_compose_pose_applies_relative_motion_in_start_tool_frame() -> None:
    half = math.sqrt(0.5)
    position, orientation = compose_pose(
        (1.0, 2.0, 3.0),
        (0.0, 0.0, half, half),
        (1.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )

    assert position == pytest.approx((1.0, 3.0, 3.0))
    assert orientation == pytest.approx((0.0, 0.0, half, half))


def test_rejects_stationary_capture() -> None:
    with pytest.raises(ValueError, match="没有检测到"):
        build_recorded_action(
            name="empty",
            side="left",
            base_frame="world",
            tool_frame="left_fr3_link8",
            samples=[_sample(0.0, 0.0, 0.0), _sample(0.1, 0.0, 0.0), _sample(0.2, 0.0, 0.0)],
        )
