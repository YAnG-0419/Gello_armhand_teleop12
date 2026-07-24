import pytest

from teleop_core.contract import (
    COMMAND_JOINT_NAMES,
    LEFT_COMMAND_JOINT_NAMES,
    RIGHT_COMMAND_JOINT_NAMES,
    command_names,
)


def test_command_joint_order():
    assert COMMAND_JOINT_NAMES == (
        LEFT_COMMAND_JOINT_NAMES + RIGHT_COMMAND_JOINT_NAMES
    )
    assert len(COMMAND_JOINT_NAMES) == 14


def test_active_side_order_is_preserved():
    assert command_names(("right",)) == RIGHT_COMMAND_JOINT_NAMES
    assert command_names(("left", "right")) == COMMAND_JOINT_NAMES


def test_invalid_active_sides_are_rejected():
    with pytest.raises(ValueError):
        command_names(("left", "left"))
    with pytest.raises(ValueError):
        command_names(("arm",))
