import numpy as np

from teleop_data.replay import load_complete_episode, load_episode


def test_load_npz_episode_rebases_time(tmp_path):
    path = tmp_path / "episode.npz"
    np.savez(
        path,
        schema_version=np.asarray("franka.teleop.normalized.v1"),
        timestamp=np.asarray([4.0, 4.1]),
        action_arm_joint_position=np.zeros((2, 14), dtype=np.float32),
        active_sides=np.asarray([[True, False], [True, True]]),
    )
    timestamp, action, active = load_episode(path)
    np.testing.assert_allclose(timestamp, [0.0, 0.1])
    assert action.shape == (2, 14)
    assert active.tolist() == [[True, False], [True, True]]


def test_load_v2_npz_slices_arm_actions_for_arm_replay(tmp_path):
    path = tmp_path / "episode-v2.npz"
    combined = np.arange(108, dtype=np.float32).reshape(2, 54)
    np.savez(
        path,
        schema_version=np.asarray("franka_linker.teleop.normalized.v2"),
        timestamp=np.asarray([2.0, 2.1]),
        action_joint_position=combined,
        active_sides=np.asarray([[True, True], [False, True]]),
    )
    _, action, _ = load_episode(path)
    np.testing.assert_array_equal(action, combined[:, :14])
    _, arm_action, hand_action, _ = load_complete_episode(path)
    np.testing.assert_array_equal(arm_action, combined[:, :14])
    np.testing.assert_array_equal(hand_action, combined[:, 14:])
