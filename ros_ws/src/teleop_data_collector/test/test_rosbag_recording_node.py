import pytest

pytest.importorskip("rclpy")

from teleop_data_collector import rosbag_recording_node
from teleop_data_collector.rosbag_recording_node import (
    _has_qos_overrides_arg,
    _topic_qos_overrides_yaml,
    _wait_for_required_topics,
)


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)


class _Node:
    def __init__(self, failures):
        self._failures = iter(failures)
        self.calls = 0
        self.logger = _Logger()

    def preflight_failures(self):
        self.calls += 1
        return next(self._failures)

    def get_logger(self):
        return self.logger


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


def test_ready_waits_for_required_topics_to_stay_stable(monkeypatch):
    clock = {"now": 0.0}
    node = _Node(
        [
            ("/cam2/color/image_raw: missing",),
            (),
            (),
            (),
        ]
    )
    monkeypatch.setattr(rosbag_recording_node.rclpy, "ok", lambda: True)
    monkeypatch.setattr(
        rosbag_recording_node.time,
        "monotonic",
        lambda: clock["now"],
    )
    monkeypatch.setattr(
        rosbag_recording_node.time,
        "sleep",
        lambda seconds: clock.__setitem__("now", clock["now"] + seconds),
    )

    assert _wait_for_required_topics(node, stable_sec=2.0, poll_sec=1.0)
    assert node.calls == 4
    assert any("/cam2/color/image_raw: missing" in text for text in node.logger.messages)
