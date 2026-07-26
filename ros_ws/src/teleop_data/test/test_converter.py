import numpy as np

from teleop_data.converter import normalize_episode


def _samples(width, offset=0.0):
    return [
        (0.0, np.arange(width, dtype=float) + offset),
        (1.0, np.arange(width, dtype=float) + offset + 1.0),
    ]


def test_normalize_episode_contains_franka_and_linker_state_and_action():
    arm_names = [
        *(f"left_fr3v2_joint{index}" for index in range(1, 8)),
        *(f"right_fr3v2_joint{index}" for index in range(1, 8)),
    ]
    raw = {
        "left_state": _samples(7),
        "right_state": _samples(7, 10.0),
        "action": [
            (0.0, (arm_names, np.arange(14, dtype=float))),
            (1.0, (arm_names, np.arange(14, dtype=float) + 1.0)),
        ],
        "active": [(0.0, [True, True]), (1.0, [True, False])],
        "left_hand_state": _samples(20, 20.0),
        "right_hand_state": _samples(20, 40.0),
        "left_hand_action": _samples(20, 60.0),
        "right_hand_action": _samples(20, 80.0),
    }
    episode = normalize_episode(raw, fps=2)
    assert str(episode["schema_version"]) == "franka_linker.teleop.normalized.v2"
    assert episode["observation_arm_joint_position"].shape == (3, 14)
    assert episode["action_arm_joint_position"].shape == (3, 14)
    assert episode["observation_hand_joint_position"].shape == (3, 40)
    assert episode["action_hand_joint_position"].shape == (3, 40)
    assert episode["observation_joint_position"].shape == (3, 54)
    assert episode["action_joint_position"].shape == (3, 54)
    np.testing.assert_array_equal(
        episode["action_joint_position"][:, 14:34],
        np.vstack(
            (
                np.arange(20) + 60.0,
                np.arange(20) + 60.0,
                np.arange(20) + 61.0,
            )
        ),
    )
