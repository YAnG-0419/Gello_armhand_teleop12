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
