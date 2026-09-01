import pytest

pytest.importorskip("rclpy")

from teleop_data_collector.rosbag_recording_node import (
    _has_qos_overrides_arg,
    _topic_qos_overrides_yaml,
)


def test_default_qos_expands_to_each_configured_topic():
    yaml_text = _topic_qos_overrides_yaml(
        topics=("/cam0/color/image_raw", "/cam0/depth/image_raw"),
        qos={"history": "keep_last", "depth": 100},
    )

    assert "/cam0/color/image_raw:" in yaml_text
    assert "/cam0/depth/image_raw:" in yaml_text
    assert yaml_text.count("history: keep_last") == 2
    assert yaml_text.count("depth: 100") == 2


def test_explicit_qos_overrides_arg_is_preserved():
    assert _has_qos_overrides_arg(
        ["--qos-profile-overrides-path", "custom_qos.yaml"]
    )
    assert not _has_qos_overrides_arg(["--storage-preset-profile", "fastwrite"])
