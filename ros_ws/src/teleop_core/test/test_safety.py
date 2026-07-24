import numpy as np
import pytest

from teleop_core.contract import COMMAND_JOINT_NAMES
from teleop_core.safety import CommandSafetyGate


HOME = np.array(
    [
        -0.1064695120, 0.5432978868, -0.7150393724, -2.0424106121,
        0.7801840901, 2.3256008625, 0.3307797611,
        -1.2620162964, -0.3890800774, 2.3749103546, -2.3138155937,
        -0.6073474288, 2.2486193180, 0.0537403040,
    ]
)


def test_first_target_must_match_measured_state():
    gate = CommandSafetyGate(max_initial_delta=0.05)
    with pytest.raises(ValueError, match="first target"):
        gate.validate(
            ("left", "right"),
            COMMAND_JOINT_NAMES,
            HOME + 0.1,
            HOME,
            1.0,
        )


def test_joint_speed_is_limited():
    gate = CommandSafetyGate(max_joint_speed=0.5, nominal_dt=0.01)
    gate.validate(("left", "right"), COMMAND_JOINT_NAMES, HOME, HOME, 1.0)
    target = HOME.copy()
    target[0] += 0.04
    result = gate.validate(
        ("left", "right"), COMMAND_JOINT_NAMES, target, HOME, 1.01
    )
    assert result.positions[0] == pytest.approx(HOME[0] + 0.005)
