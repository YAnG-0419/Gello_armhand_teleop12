import numpy as np

from teleop_data.replay import load_episode, preposition_trajectory


def test_preposition_starts_and_ends_exactly():
    start = np.zeros(14)
    target = np.full(14, 0.2)
    trajectory = preposition_trajectory(start, target, speed=0.1, rate=100)
    np.testing.assert_allclose(trajectory[0], start)
    np.testing.assert_allclose(trajectory[-1], target)
    assert np.max(np.abs(np.diff(trajectory, axis=0))) < 0.002


def test_load_episode_includes_active_side_mask(tmp_path):
    episode = tmp_path / "episode.npz"
    np.savez_compressed(
        episode,
        schema_version=np.asarray("franka.teleop.normalized.v1"),
        timestamp=np.asarray([0.0, 0.1]),
        action_arm_joint_position=np.zeros((2, 14)),
        active_sides=np.asarray([[True, False], [True, True]]),
    )

    timestamp, action, active = load_episode(episode)

    assert timestamp.shape == (2,)
    assert action.shape == (2, 14)
    np.testing.assert_array_equal(active, [[True, False], [True, True]])
