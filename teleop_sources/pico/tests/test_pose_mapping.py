import numpy as np
import pinocchio as pin
import pytest

from pico_bimanual_franka_teleop.pose_mapping import (
    RelativePoseMapper,
    is_valid_xr_pose,
    xr_pose_to_world,
)
from pico_bimanual_franka_teleop.types import Pose


def pose(position=(0.0, 0.0, 0.0), rotation=None):
    return Pose(np.asarray(position), np.eye(3) if rotation is None else rotation)


def test_zero_xr_quaternion_is_rejected():
    raw = np.zeros(7)
    assert not is_valid_xr_pose(raw)
    with pytest.raises(ValueError, match="non-zero quaternion"):
        xr_pose_to_world(raw)


def test_valid_xr_pose_converts_to_world():
    raw = np.array([0.1, -0.2, 0.3, 0.0, 0.0, 0.0, 1.0])
    assert is_valid_xr_pose(raw)
    converted = xr_pose_to_world(raw)
    assert converted.position.shape == (3,)
    assert converted.rotation.shape == (3, 3)


def test_headset_axes_map_to_operator_aligned_world():
    # Headset +X (right) -> world +Y; +Y (up) -> world +Z; +Z (back) -> world +X
    # so headset forward (-Z) -> world -X.
    from pico_bimanual_franka_teleop.pose_mapping import R_HEADSET_TO_WORLD

    right = R_HEADSET_TO_WORLD @ np.array([1.0, 0.0, 0.0])
    up = R_HEADSET_TO_WORLD @ np.array([0.0, 1.0, 0.0])
    forward = R_HEADSET_TO_WORLD @ np.array([0.0, 0.0, -1.0])
    np.testing.assert_allclose(right, [0.0, 1.0, 0.0])
    np.testing.assert_allclose(up, [0.0, 0.0, 1.0])
    np.testing.assert_allclose(forward, [-1.0, 0.0, 0.0])


def test_clutch_engages_without_a_jump_and_resets_on_release():
    mapper = RelativePoseMapper(translation_scale=0.5)
    robot = pose((0.4, -0.2, 0.8))
    controller = pose((1.0, 2.0, 3.0))

    assert mapper.update(controller, 0.0, robot) is None
    target = mapper.update(controller, 1.0, robot)
    assert target is not None
    np.testing.assert_allclose(target.position, robot.position)
    np.testing.assert_allclose(target.rotation, robot.rotation)

    moved = mapper.update(pose((1.2, 2.0, 3.0)), 1.0, robot)
    assert moved is not None
    np.testing.assert_allclose(moved.position, [0.5, -0.2, 0.8])

    assert mapper.update(controller, 0.0, robot) is None
    reengaged = mapper.update(pose((9.0, 8.0, 7.0)), 1.0, robot)
    assert reengaged is not None
    np.testing.assert_allclose(reengaged.position, robot.position)


def test_rotation_delta_is_applied_in_world_coordinates():
    mapper = RelativePoseMapper(translation_scale=1.0)
    robot = pose(rotation=pin.exp3(np.array([0.2, 0.0, 0.0])))
    mapper.update(pose(), 1.0, robot)
    delta = pin.exp3(np.array([0.0, 0.3, 0.0]))
    target = mapper.update(pose(rotation=delta), 1.0, robot)
    assert target is not None
    np.testing.assert_allclose(target.rotation, delta @ robot.rotation, atol=1e-7)
