import sys

import numpy as np

from pico_bimanual_franka_teleop import xr_input


class _FakeKeyboard:
    def __init__(self, _device: str) -> None:
        self.active = {"left": False, "right": False}

    def poll(self) -> dict[str, bool]:
        return dict(self.active)

    def disable_all(self, _reason: str) -> None:
        self.active = {"left": False, "right": False}

    def close(self) -> None:
        pass


class _FakeXrt:
    def __init__(self) -> None:
        self.closed = False
        self.timestamp = 123
        self.serials = ["RIGHT-SN", "LEFT-SN"]
        self.poses = [
            [0.4, 0.5, 0.6, 0.0, 0.0, 0.0, 1.0],
            [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
        ]

    def init(self) -> None:
        pass

    def close(self) -> None:
        self.closed = True

    def get_motion_timestamp_ns(self) -> int:
        return self.timestamp

    def num_motion_data_available(self) -> int:
        return len(self.serials)

    def get_motion_tracker_serial_numbers(self) -> list[str]:
        return list(self.serials)

    def get_motion_tracker_pose(self) -> list[list[float]]:
        return list(self.poses)


def _identity_transforms() -> dict[str, dict[str, list[float]]]:
    transform = {
        "translation_xyz": [0.0, 0.0, 0.0],
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
    return {"left": dict(transform), "right": dict(transform)}


def _create_tracker_input() -> xr_input.MotionTrackerInput:
    return xr_input.MotionTrackerInput(
        serials={"left": "LEFT-SN", "right": "RIGHT-SN"},
        tracker_to_control=_identity_transforms(),
        ready_timeout=0.1,
        stale_timeout=0.25,
        frozen_timeout=1.0,
        max_position_jump=0.2,
        max_rotation_jump=1.0,
        max_linear_speed=3.0,
        max_angular_speed=12.0,
        keyboard_device="/dev/null",
    )


def test_motion_trackers_are_mapped_by_serial(monkeypatch) -> None:
    fake_xrt = _FakeXrt()
    monkeypatch.setitem(sys.modules, "xrobotoolkit_sdk", fake_xrt)
    monkeypatch.setattr(xr_input, "KeyboardActivation", _FakeKeyboard)
    tracker_input = _create_tracker_input()
    tracker_input.keyboard.active = {"left": True, "right": True}

    sample = tracker_input.sample()

    assert sample is not None
    np.testing.assert_allclose(sample.poses["left"].position, [-0.3, -0.1, 0.2])
    np.testing.assert_allclose(sample.poses["right"].position, [-0.6, -0.4, 0.5])
    assert sample.activations == {"left": True, "right": True}
    tracker_input.close()
    assert fake_xrt.closed


def test_motion_snapshot_rejects_inconsistent_count() -> None:
    fake_xrt = _FakeXrt()
    fake_xrt.serials = ["LEFT-SN"]
    tracker_input = object.__new__(xr_input.MotionTrackerInput)
    tracker_input.xrt = fake_xrt
    tracker_input.serials = {"left": "LEFT-SN", "right": "RIGHT-SN"}

    assert tracker_input._snapshot() is None


def test_motion_snapshot_accepts_five_trackers() -> None:
    fake_xrt = _FakeXrt()
    fake_xrt.serials = [f"SN-{index}" for index in range(5)]
    fake_xrt.poses = [
        [float(index), 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        for index in range(5)
    ]
    tracker_input = object.__new__(xr_input.MotionTrackerInput)
    tracker_input.xrt = fake_xrt
    tracker_input.serials = {"left": "SN-3", "right": "SN-1"}

    snapshot = tracker_input._snapshot()

    assert snapshot is not None
    _, selected, serials = snapshot
    assert serials == fake_xrt.serials
    np.testing.assert_allclose(selected["left"][:3], [3.0, 0.0, 0.0])
    np.testing.assert_allclose(selected["right"][:3], [1.0, 0.0, 0.0])


def test_motion_input_accepts_bounded_motion(monkeypatch) -> None:
    fake_xrt = _FakeXrt()
    monkeypatch.setitem(sys.modules, "xrobotoolkit_sdk", fake_xrt)
    monkeypatch.setattr(xr_input, "KeyboardActivation", _FakeKeyboard)
    tracker_input = _create_tracker_input()
    tracker_input.keyboard.active = {"left": True, "right": True}
    fake_xrt.timestamp += 20_000_000
    fake_xrt.poses[1][0] += 0.01

    sample = tracker_input.sample()

    assert sample is not None
    assert sample.activations == {"left": True, "right": True}


def test_motion_input_disengages_on_pose_jump(monkeypatch) -> None:
    fake_xrt = _FakeXrt()
    monkeypatch.setitem(sys.modules, "xrobotoolkit_sdk", fake_xrt)
    monkeypatch.setattr(xr_input, "KeyboardActivation", _FakeKeyboard)
    tracker_input = _create_tracker_input()
    tracker_input.keyboard.active = {"left": True, "right": True}
    fake_xrt.timestamp += 20_000_000
    assert tracker_input.sample() is not None
    fake_xrt.timestamp += 20_000_000
    fake_xrt.poses[1][0] += 0.3

    assert tracker_input.sample() is None
    assert tracker_input.keyboard.active == {"left": False, "right": False}


def test_motion_input_recovers_after_timestamp_restart(monkeypatch) -> None:
    fake_xrt = _FakeXrt()
    monkeypatch.setitem(sys.modules, "xrobotoolkit_sdk", fake_xrt)
    monkeypatch.setattr(xr_input, "KeyboardActivation", _FakeKeyboard)
    tracker_input = _create_tracker_input()
    tracker_input.keyboard.active = {"left": True, "right": True}
    fake_xrt.timestamp = 1

    assert tracker_input.sample() is None
    assert tracker_input.keyboard.active == {"left": False, "right": False}

    tracker_input.keyboard.active = {"left": True, "right": True}
    fake_xrt.timestamp += 20_000_000
    sample = tracker_input.sample()

    assert sample is not None
    assert sample.activations == {"left": True, "right": True}


def test_inactive_tracker_can_move_freely(monkeypatch) -> None:
    fake_xrt = _FakeXrt()
    monkeypatch.setitem(sys.modules, "xrobotoolkit_sdk", fake_xrt)
    monkeypatch.setattr(xr_input, "KeyboardActivation", _FakeKeyboard)
    tracker_input = _create_tracker_input()
    tracker_input.keyboard.active = {"left": False, "right": True}
    fake_xrt.timestamp += 20_000_000
    fake_xrt.poses[1][0] += 0.5

    sample = tracker_input.sample()

    assert sample is not None
    assert sample.activations == {"left": False, "right": True}


def test_reenabled_tracker_reanchors_safety_baseline(monkeypatch) -> None:
    fake_xrt = _FakeXrt()
    monkeypatch.setitem(sys.modules, "xrobotoolkit_sdk", fake_xrt)
    monkeypatch.setattr(xr_input, "KeyboardActivation", _FakeKeyboard)
    tracker_input = _create_tracker_input()
    tracker_input.keyboard.active = {"left": False, "right": True}
    fake_xrt.timestamp += 20_000_000
    fake_xrt.poses[1][0] += 0.5
    assert tracker_input.sample() is not None

    tracker_input.keyboard.active = {"left": True, "right": True}
    fake_xrt.timestamp += 20_000_000
    fake_xrt.poses[1][0] += 0.5
    sample = tracker_input.sample()

    assert sample is not None
    assert sample.activations == {"left": True, "right": True}


def test_motion_input_disengages_on_frozen_pose(monkeypatch) -> None:
    fake_xrt = _FakeXrt()
    monkeypatch.setitem(sys.modules, "xrobotoolkit_sdk", fake_xrt)
    monkeypatch.setattr(xr_input, "KeyboardActivation", _FakeKeyboard)
    tracker_input = _create_tracker_input()
    tracker_input.keyboard.active = {"left": True, "right": True}
    fake_xrt.timestamp += 20_000_000
    assert tracker_input.sample() is not None
    tracker_input.last_position_changed_at["left"] -= 2.0
    tracker_input.last_rotation_changed_at["left"] -= 2.0
    fake_xrt.timestamp += 20_000_000

    assert tracker_input.sample() is None
    assert tracker_input.keyboard.active == {"left": False, "right": False}


def test_inactive_frozen_tracker_does_not_stop_active_side(monkeypatch) -> None:
    fake_xrt = _FakeXrt()
    monkeypatch.setitem(sys.modules, "xrobotoolkit_sdk", fake_xrt)
    monkeypatch.setattr(xr_input, "KeyboardActivation", _FakeKeyboard)
    tracker_input = _create_tracker_input()
    tracker_input.keyboard.active = {"left": False, "right": True}
    tracker_input.last_position_changed_at["left"] -= 2.0
    tracker_input.last_rotation_changed_at["left"] -= 2.0
    fake_xrt.timestamp += 20_000_000
    fake_xrt.poses[0][0] += 0.01

    sample = tracker_input.sample()

    assert sample is not None
    assert sample.activations == {"left": False, "right": True}


def test_frozen_position_with_live_rotation_disengages(monkeypatch) -> None:
    # The failure recorded on 2026-07-25: the tracker lost its optical fix, so
    # position updated 37 times in 45 s while the IMU kept streaming rotation
    # on 84% of ticks. A combined alive-if-anything-moves clock never faulted
    # and the arms tracked a sub-hertz position stream for half a minute.
    # Position and rotation liveness must be judged independently.
    fake_xrt = _FakeXrt()
    monkeypatch.setitem(sys.modules, "xrobotoolkit_sdk", fake_xrt)
    monkeypatch.setattr(xr_input, "KeyboardActivation", _FakeKeyboard)
    tracker_input = _create_tracker_input()
    tracker_input.keyboard.active = {"left": True, "right": True}
    fake_xrt.timestamp += 20_000_000
    assert tracker_input.sample() is not None

    # Rotation keeps jittering, position never moves, and the position clock
    # has aged past the frozen timeout.
    tracker_input.last_position_changed_at["left"] -= 2.0
    fake_xrt.poses[1][3] += 0.01  # IMU wiggle on the left tracker's quaternion
    fake_xrt.timestamp += 20_000_000

    assert tracker_input.sample() is None
    assert tracker_input.keyboard.active == {"left": False, "right": False}


def test_live_position_with_frozen_rotation_disengages(monkeypatch) -> None:
    # The mirror failure: a dead IMU with a live optical fix must fault too.
    fake_xrt = _FakeXrt()
    monkeypatch.setitem(sys.modules, "xrobotoolkit_sdk", fake_xrt)
    monkeypatch.setattr(xr_input, "KeyboardActivation", _FakeKeyboard)
    tracker_input = _create_tracker_input()
    tracker_input.keyboard.active = {"left": True, "right": True}
    fake_xrt.timestamp += 20_000_000
    assert tracker_input.sample() is not None

    tracker_input.last_rotation_changed_at["left"] -= 2.0
    fake_xrt.poses[1][0] += 0.005  # position moves, under the jump limit
    fake_xrt.timestamp += 20_000_000

    assert tracker_input.sample() is None
    assert tracker_input.keyboard.active == {"left": False, "right": False}


def test_create_pico_input_refuses_next_to_the_desktop_gui(monkeypatch) -> None:
    # The GUI and the Python SDK compete for the PC Service feedback stream; a
    # recorded session with the GUI in use degraded tracker positions to
    # sub-hertz while the GUI showed them moving accurately. Teleoperation is
    # where degraded input moves hardware, so it must refuse to start.
    monkeypatch.setattr(xr_input, "desktop_gui_pids", lambda: [4242])

    class _Config:
        controllers = None
        motion_trackers = None

    try:
        xr_input.create_pico_input(_Config(), "motion-trackers")
    except RuntimeError as error:
        assert "RobotLinuxDemo" in str(error)
        assert "4242" in str(error)
    else:
        raise AssertionError("expected create_pico_input to refuse")
