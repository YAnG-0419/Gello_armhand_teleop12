from pathlib import Path

import numpy as np
import pytest

from manus_teleop.o30i_retarget import O30IRetargeter

REPO_ROOT = Path(__file__).resolve().parents[3]
URDF = (
    REPO_ROOT
    / "assets"
    / "linkerhand_o30i"
    / "right"
    / "linkerhand_o30i_right.urdf"
)


def test_o30i_retargeter_emits_twenty_urdf_radian_joints():
    retargeter = O30IRetargeter(URDF, "right")
    landmarks = retargeter.robot_landmarks(np.zeros(20))
    qpos, stats = retargeter.retarget(landmarks)
    assert len(retargeter.joint_names) == 20
    assert qpos.shape == (20,)
    assert np.isfinite(qpos).all()
    assert np.all(qpos >= retargeter.lower)
    assert np.all(qpos <= retargeter.upper)
    assert stats["loss"] >= 0.0


def test_o30i_retargeter_rejects_wrong_side_and_bad_landmarks():
    try:
        O30IRetargeter(URDF, "left")
    except ValueError as error:
        assert "right only" in str(error)
    else:
        raise AssertionError("left O30i unexpectedly accepted")
    retargeter = O30IRetargeter(URDF, "right")
    try:
        retargeter.retarget(np.zeros((20, 3)))
    except ValueError as error:
        assert "shape" in str(error)
    else:
        raise AssertionError("bad landmarks unexpectedly accepted")


def test_o30i_length_normalization_preserves_each_segment_direction():
    retargeter = O30IRetargeter(URDF, "right")
    robot = retargeter.robot_landmarks(np.zeros(20))
    shortened = robot.copy()
    chains = {
        "thumb": (1, 2, 3, 4),
        "index": (5, 6, 7, 8),
        "middle": (9, 10, 11, 12),
        "ring": (13, 14, 15, 16),
        "pinky": (17, 18, 19, 20),
    }
    for chain in chains.values():
        cursor = shortened[chain[0]].copy()
        segments = [
            robot[chain[index + 1]] - robot[chain[index]]
            for index in range(3)
        ]
        for index, segment in enumerate(segments):
            cursor = cursor + 0.6 * segment
            shortened[chain[index + 1]] = cursor

    normalized = retargeter.target_positions(shortened)

    for chain in chains.values():
        for index in range(3):
            expected = robot[chain[index + 1]] - robot[chain[index]]
            actual = normalized[chain[index + 1]] - normalized[chain[index]]
            expected /= np.linalg.norm(expected)
            actual /= np.linalg.norm(actual)
            assert np.allclose(actual, expected, atol=1e-8)


def test_o30i_ordinary_finger_targets_follow_anatomical_joint_frames():
    retargeter = O30IRetargeter(URDF, "right")
    for finger in ("index", "middle", "ring", "pinky"):
        expected = [
            retargeter.model.getFrameId(f"{finger}_proximal"),
            retargeter.model.getFrameId(f"{finger}_middle"),
            retargeter.model.getFrameId(f"{finger}_distal"),
        ]
        assert retargeter._frames[finger][:-1] == expected


def test_o30i_retargeting_assigns_distal_motion_to_dip():
    retargeter = O30IRetargeter(
        URDF,
        "right",
        smooth_weight=0.0,
        filter_alpha=1.0,
        max_iterations=200,
    )
    desired = np.zeros(20)
    dip_index = retargeter.joint_names.index("index_dip")
    for angle in np.linspace(0.0, 0.8, 41):
        desired[dip_index] = angle
        landmarks = retargeter.robot_landmarks(desired)
        solved, stats = retargeter.retarget(landmarks)

    assert stats["success"]
    assert solved[dip_index] == pytest.approx(0.8, abs=1e-5)

