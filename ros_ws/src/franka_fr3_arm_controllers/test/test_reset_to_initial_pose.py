import importlib.util
import math
from pathlib import Path

import pytest
import yaml


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "reset_to_initial_pose.py"
)
SPEC = importlib.util.spec_from_file_location("reset_to_initial_pose", SCRIPT_PATH)
RESET = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(RESET)


@pytest.mark.parametrize("distance", [0.001, 0.01, 0.1, 1.0, 2.0, 5.0])
def test_reset_duration_respects_peak_limits(distance: float) -> None:
    duration = RESET.reset_duration(distance)

    peak_speed = distance * RESET.SMOOTHERSTEP_PEAK_SPEED / duration
    peak_acceleration = (
        distance * RESET.SMOOTHERSTEP_PEAK_ACCELERATION / duration**2
    )
    assert duration >= RESET.RESET_MIN_DURATION
    assert peak_speed <= RESET.RESET_MAX_SPEED
    assert peak_acceleration <= RESET.RESET_MAX_ACCELERATION


def test_reset_duration_scales_with_large_move() -> None:
    assert math.isclose(RESET.reset_duration(1.0), 9.375)
    assert math.isclose(RESET.reset_duration(2.0), 18.75)


def test_main_does_not_publish_after_context_shutdown(monkeypatch) -> None:
    calls = []

    class FakeNode:
        def _set_active(self, active):
            calls.append(("active", active))

        def destroy_node(self):
            calls.append(("destroy",))

    class FakeExecutor:
        def __init__(self, num_threads):
            calls.append(("executor", num_threads))

        def add_node(self, node):
            calls.append(("add", node))

        def spin(self):
            calls.append(("spin",))

        def shutdown(self):
            calls.append(("executor_shutdown",))

    node = FakeNode()
    monkeypatch.setattr(RESET, "get_package_share_directory", lambda _package: "/tmp")
    monkeypatch.setattr(RESET, "load_targets", lambda _path: {})
    monkeypatch.setattr(RESET.rclpy, "init", lambda: calls.append(("init",)))
    monkeypatch.setattr(RESET.rclpy, "ok", lambda: False)
    monkeypatch.setattr(
        RESET.rclpy,
        "shutdown",
        lambda: calls.append(("rclpy_shutdown",)),
    )
    monkeypatch.setattr(RESET, "InitialPoseReset", lambda _path, _targets: node)
    monkeypatch.setattr(RESET, "MultiThreadedExecutor", FakeExecutor)

    RESET.main()

    # The initial state publication is allowed while the context is valid;
    # cleanup must not try a second publication after SIGINT invalidates it.
    assert calls.count(("active", False)) == 1
    assert ("executor_shutdown",) in calls
    assert ("destroy",) in calls
    assert ("rclpy_shutdown",) not in calls


def test_retime_trajectory_preserves_path_and_only_extends_fast_intervals() -> None:
    positions = lambda value: {"left": [value] * 7, "right": [-value] * 7}
    samples = [
        (0.0, positions(0.0)),
        (0.02, positions(0.01)),
        (0.12, positions(0.02)),
    ]

    retimed = RESET.retime_trajectory(samples, max_speed=0.25)

    assert [sample[1] for sample in retimed] == [sample[1] for sample in samples]
    assert math.isclose(retimed[0][0], 0.0)
    assert math.isclose(retimed[1][0], 0.04)
    assert math.isclose(retimed[2][0], 0.14)
    assert RESET.maximum_trajectory_speed(retimed) <= 0.25 + 1e-9


@pytest.mark.parametrize(
    "arguments",
    [
        (-1.0,),
        (1.0, 0.0),
        (1.0, 0.2, 0.0),
        (math.inf,),
        (math.nan,),
    ],
)
def test_reset_duration_rejects_invalid_limits(arguments: tuple[float, ...]) -> None:
    with pytest.raises(ValueError):
        RESET.reset_duration(*arguments)


def test_write_targets_round_trips_atomically(tmp_path: Path) -> None:
    path = tmp_path / "initial_pose.yaml"
    targets = {
        "left": [float(index) for index in range(7)],
        "right": [float(-index) for index in range(7)],
    }

    RESET.write_targets(path, targets)

    assert RESET.load_targets(path) == targets
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert document["captured_utc"]
    assert not tuple(tmp_path.glob(".initial_pose.*.tmp"))


def test_operator_pose_store_seeds_three_homes_and_requires_ready_capture(
    tmp_path: Path,
) -> None:
    path = tmp_path / "operator_gui" / "poses.yaml"
    legacy = {
        "left": [0.1] * 7,
        "right": [-0.1] * 7,
    }

    tasks, ready = RESET.load_operator_poses(path, legacy)

    assert set(tasks) == {"powder_weighing", "assembly", "bean_picking"}
    assert tasks["powder_weighing"] == legacy
    assert tasks["assembly"] == {"left": None, "right": None}
    assert tasks["bean_picking"] == {"left": None, "right": None}
    assert ready is None
    assert path.exists()

    recorded_ready = {"left": [0.2] * 7, "right": [-0.2] * 7}
    RESET.write_operator_poses(path, tasks, recorded_ready)
    loaded_tasks, loaded_ready = RESET.load_operator_poses(path, legacy)
    assert loaded_tasks == tasks
    assert loaded_ready == recorded_ready


def test_load_absolute_trajectory_validates_task_and_dual_arm_shape(
    tmp_path: Path,
) -> None:
    path = tmp_path / "assembly.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "version": 1,
                "kind": "dual_arm_absolute_joint_trajectory",
                "task": "assembly",
                "joint_names": {
                    side: [f"{side}_fr3_joint{index}" for index in range(1, 8)]
                    for side in ("left", "right")
                },
                "samples": [
                    {
                        "time_sec": timestamp,
                        "positions": {
                            "left": [timestamp] * 7,
                            "right": [-timestamp] * 7,
                        },
                    }
                    for timestamp in (0.0, 0.1, 0.3)
                ],
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    samples = RESET.load_absolute_trajectory(path, "assembly")

    assert [sample[0] for sample in samples] == [0.0, 0.1, 0.3]
