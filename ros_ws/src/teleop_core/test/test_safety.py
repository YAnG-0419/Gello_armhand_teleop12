"""ROS-free tests for the command safety gate.

Run with:
    PYTHONPATH=ros_ws/src/teleop_core python3 -m pytest -q \
        ros_ws/src/teleop_core/test/test_safety.py
"""

import numpy as np
import pytest

from teleop_core.contract import command_names
from teleop_core.safety import CommandSafetyGate


# A valid FR3 ready pose (joint 4 only accepts [-3.077, -0.117] rad).
HOME = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785] * 2)


def make_gate(**overrides):
    settings = dict(
        max_joint_speed=0.5, max_initial_delta=0.05, nominal_dt=0.01
    )
    settings.update(overrides)
    return CommandSafetyGate(**settings)


def send(gate, target, measured, now, sides=("left",)):
    names = command_names(sides)
    positions = [target[i] for i in range(len(names))]
    return gate.validate(sides, names, positions, measured, now)


def test_slew_limits_each_step():
    gate = make_gate()
    first = send(gate, HOME[:7], HOME, now=0.0)
    assert np.allclose(first.positions, HOME[:7])
    target = HOME[:7] + 0.3
    second = send(gate, target, HOME, now=0.01)
    assert np.allclose(second.positions, HOME[:7] + 0.5 * 0.01)


def test_deviation_cap_disabled_by_default():
    gate = make_gate()
    target = HOME[:7] + 0.3
    send(gate, HOME[:7], HOME, now=0.0)
    # The measured state never moves; without a cap the command walks all the
    # way to the target at max_joint_speed.
    now, out = 0.0, None
    for _ in range(100):
        now += 0.01
        out = send(gate, target, HOME, now=now)
    assert np.allclose(out.positions, target)


def test_deviation_cap_bounds_command_minus_measured():
    gate = make_gate(max_command_deviation=0.04)
    target = HOME[:7] + 0.3
    send(gate, HOME[:7], HOME, now=0.0)
    now, out = 0.0, None
    for _ in range(100):
        now += 0.01
        out = send(gate, target, HOME, now=now)
    assert np.max(np.abs(np.asarray(out.positions) - HOME[:7])) <= 0.04 + 1e-12


def test_deviation_cap_releases_when_measured_catches_up():
    gate = make_gate(max_command_deviation=0.04)
    target = HOME[:7] + 0.1
    send(gate, HOME[:7], HOME, now=0.0)
    now = 0.0
    for _ in range(100):
        now += 0.01
        send(gate, target, HOME, now=now)
    # Once the obstruction is gone the measured state reaches the target and
    # the command follows it without a residual offset.
    measured = HOME.copy()
    measured[:7] = target
    out = None
    for _ in range(100):
        now += 0.01
        out = send(gate, target, measured, now=now)
    assert np.allclose(out.positions, target)


def test_deviation_cap_applies_per_joint():
    caps = [0.06, 0.06, 0.06, 0.06, 0.12, 0.2, 0.3]
    gate = make_gate(max_command_deviation=caps)
    target = HOME[:7] + 0.5
    send(gate, HOME[:7], HOME, now=0.0)
    now, out = 0.0, None
    for _ in range(200):
        now += 0.01
        out = send(gate, target, HOME, now=now)
    deviation = np.asarray(out.positions) - HOME[:7]
    assert np.allclose(deviation, caps)


def test_zero_entries_leave_joints_uncapped():
    gate = make_gate(max_command_deviation=[0.06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    target = HOME[:7] + 0.3
    send(gate, HOME[:7], HOME, now=0.0)
    now, out = 0.0, None
    for _ in range(100):
        now += 0.01
        out = send(gate, target, HOME, now=now)
    deviation = np.asarray(out.positions) - HOME[:7]
    assert deviation[0] == pytest.approx(0.06)
    assert np.allclose(deviation[1:], 0.3)


def test_rejects_invalid_deviation_caps():
    with pytest.raises(ValueError):
        make_gate(max_command_deviation=-0.01)
    with pytest.raises(ValueError):
        make_gate(max_command_deviation=[0.05, 0.05])
