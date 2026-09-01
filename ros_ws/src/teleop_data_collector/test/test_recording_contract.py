from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from teleop_core.contract import (
    LEFT_COMMAND_JOINT_NAMES,
    VALIDATED_COMMAND_TOPIC,
)
from teleop_data_collector.bag_validation import (
    StreamSamples,
    _inspect_message_content,
    stream_timing_report,
    timing_failures,
)
from teleop_data_collector.collector_contract import (
    topic_contracts,
    validate_recording_contract,
)


REPO_ROOT = Path(__file__).resolve().parents[4]
CONFIG = REPO_ROOT / "data_collection/config/record_gello.yaml"


def _config():
    return yaml.safe_load(CONFIG.read_text(encoding="utf-8"))[
        "teleop_data_collector"
    ]["ros__parameters"]


def test_recording_contract_has_only_post_gateway_action_and_12_required_streams():
    config = _config()
    validate_recording_contract(config)
    assert config["teleoperator"] == "gello"
    topics = topic_contracts(config["topics"])
    assert len(topics) == 12
    assert VALIDATED_COMMAND_TOPIC in {item.topic for item in topics}
    assert "/teleop/arm_commands" not in {item.topic for item in topics}
    assert "/cam0/depth/image_raw" in {item.topic for item in topics}
    assert all(item.max_gap_ms == 150.0 for item in topics)
    by_topic = {item.topic: item for item in topics}
    assert by_topic["/cam0/color/image_raw"].min_frequency_hz == 18.0
    assert by_topic["/cam0/depth/image_raw"].min_frequency_hz == 18.0


def test_timing_report_rejects_missing_low_rate_and_stale_interval():
    report = stream_timing_report(
        StreamSamples(
            bag_times_ns=[1_000_000_000, 1_040_000_000, 1_300_000_000],
            header_times_ns=[10, 20, 30],
        ),
        global_start_ns=1_000_000_000,
        global_end_ns=1_300_000_000,
    )
    failures = timing_failures(
        "camera", report, min_frequency_hz=10.0, max_gap_ms=150.0
    )
    assert any("below" in failure for failure in failures)
    assert any("max_internal_gap_ms" in failure for failure in failures)


def test_partial_arm_side_is_rejected_and_inactive_side_is_not_invented():
    streams = {}
    counters = {
        "max_sender_dropped_packets": 0,
        "max_receiver_lost_packets": 0,
        "max_receiver_duplicate_packets": 0,
        "max_receiver_out_of_order_packets": 0,
        "max_receiver_stale_packets": 0,
        "max_receiver_invalid_packets": 0,
        "velocity_sources": set(),
    }
    partial = SimpleNamespace(
        name=list(LEFT_COMMAND_JOINT_NAMES[:-1]),
        position=[0.0] * 6,
    )
    with pytest.raises(ValueError, match="partial left"):
        _inspect_message_content(
            VALIDATED_COMMAND_TOPIC,
            partial,
            100,
            90,
            streams,
            counters,
        )

    complete = SimpleNamespace(
        name=list(LEFT_COMMAND_JOINT_NAMES),
        position=[0.0] * 7,
    )
    from collections import defaultdict

    actual_streams = defaultdict(StreamSamples)
    _inspect_message_content(
        VALIDATED_COMMAND_TOPIC,
        complete,
        100,
        90,
        actual_streams,
        counters,
    )
    assert actual_streams[f"{VALIDATED_COMMAND_TOPIC}#left"].bag_times_ns == [100]
    assert actual_streams[f"{VALIDATED_COMMAND_TOPIC}#right"].bag_times_ns == []


def test_head_depth_requires_native_packed_16uc1():
    streams = {}
    counters = {}
    valid = SimpleNamespace(
        width=640,
        height=400,
        encoding="16UC1",
        step=1280,
        data=bytes(640 * 400 * 2),
    )
    _inspect_message_content(
        "/cam0/depth/image_raw", valid, 100, 90, streams, counters
    )
    invalid = SimpleNamespace(
        width=640,
        height=400,
        encoding="32FC1",
        step=2560,
        data=bytes(640 * 400 * 4),
    )
    with pytest.raises(ValueError, match="16UC1"):
        _inspect_message_content(
            "/cam0/depth/image_raw", invalid, 100, 90, streams, counters
        )
