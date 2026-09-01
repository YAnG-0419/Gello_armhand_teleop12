from types import SimpleNamespace

import pytest

from teleop_core.contract import (
    LEFT_COMMAND_JOINT_NAMES,
    RIGHT_COMMAND_JOINT_NAMES,
    VALIDATED_COMMAND_TOPIC,
)
from teleop_data_collector.rosbag_to_lerobot import (
    _depth_image_specs_from_config,
    _feature_specs_from_config,
    _message_to_source_value,
    _mode_conversion_config,
    _source_names,
    _video_specs_from_config,
)


def _joint_state(names, positions):
    return SimpleNamespace(
        name=list(names),
        position=list(positions),
        velocity=[0.0] * len(names),
        effort=[],
    )


def test_gello_schema_is_exact_harvest_54_108_and_three_cameras():
    config = _mode_conversion_config("gello")
    features = {
        spec.column: spec for spec in _feature_specs_from_config(config)
    }
    assert len(_source_names(features["action"])) == 54
    assert len(_source_names(features["observation.state"])) == 108
    assert {spec.column for spec in _video_specs_from_config(config)} == {
        "observation.images.cam0",
        "observation.images.cam1",
        "observation.images.cam2",
    }
    depth_specs = _depth_image_specs_from_config(config)
    assert len(depth_specs) == 1
    assert depth_specs[0].column == "observation.depths.cam0"
    assert depth_specs[0].topic == "/cam0/depth/image_raw"
    assert depth_specs[0].format == "raw16"
    assert depth_specs[0].fps == 10


def test_combined_validated_action_splits_sides_by_name_and_normalizes_names():
    config = _mode_conversion_config("gello")
    action = _feature_specs_from_config(config)[0]
    assert action.sources[0].topic == VALIDATED_COMMAND_TOPIC
    assert action.sources[1].topic == VALIDATED_COMMAND_TOPIC
    message = _joint_state(
        [*RIGHT_COMMAND_JOINT_NAMES, *LEFT_COMMAND_JOINT_NAMES],
        [*range(7, 14), *range(7)],
    )
    left = _message_to_source_value(
        "sensor_msgs/msg/JointState",
        message,
        "position",
        expected_joint_names=list(LEFT_COMMAND_JOINT_NAMES),
        allow_extra_joint_names=True,
    )
    right = _message_to_source_value(
        "sensor_msgs/msg/JointState",
        message,
        "position",
        expected_joint_names=list(RIGHT_COMMAND_JOINT_NAMES),
        allow_extra_joint_names=True,
    )
    assert left == [float(value) for value in range(7)]
    assert right == [float(value) for value in range(7, 14)]


def test_inactive_side_is_missing_not_fabricated():
    message = _joint_state(LEFT_COMMAND_JOINT_NAMES, range(7))
    assert _message_to_source_value(
        "sensor_msgs/msg/JointState",
        message,
        "position",
        expected_joint_names=list(RIGHT_COMMAND_JOINT_NAMES),
        allow_extra_joint_names=True,
    ) is None


def test_partial_named_side_is_rejected():
    message = _joint_state(LEFT_COMMAND_JOINT_NAMES[:-1], range(6))
    with pytest.raises(ValueError, match="partial named group"):
        _message_to_source_value(
            "sensor_msgs/msg/JointState",
            message,
            "position",
            expected_joint_names=list(LEFT_COMMAND_JOINT_NAMES),
            allow_extra_joint_names=True,
        )
