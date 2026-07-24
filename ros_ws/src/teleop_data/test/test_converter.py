import numpy as np

from teleop_core.contract import COMMAND_JOINT_NAMES
from teleop_data.converter import normalize_episode


def test_normalize_episode_preserves_observations_actions_and_active_sides():
    left_names = [f"left_fr3_joint{index}" for index in range(1, 8)]
    right_names = [f"right_fr3_joint{index}" for index in range(1, 8)]
    left_start = np.arange(7, dtype=float)
    right_start = np.arange(7, 14, dtype=float)
    target = np.arange(14, dtype=float) + 0.25
    raw = {
        "left_state": [(0.0, left_start), (1.0, left_start + 1.0)],
        "right_state": [(0.0, right_start), (1.0, right_start + 1.0)],
        "action": [
            (0.0, (COMMAND_JOINT_NAMES, target)),
            (1.0, (COMMAND_JOINT_NAMES, target + 1.0)),
        ],
        "active": [(0.0, [True, False]), (1.0, [True, True])],
    }

    episode = normalize_episode(raw, fps=2)

    assert episode["observation_arm_joint_position"].shape == (3, 14)
    assert episode["action_arm_joint_position"].shape == (3, 14)
    np.testing.assert_allclose(episode["action_arm_joint_position"][0], target)
    np.testing.assert_array_equal(
        episode["active_sides"],
        [[True, False], [True, False], [True, True]],
    )
