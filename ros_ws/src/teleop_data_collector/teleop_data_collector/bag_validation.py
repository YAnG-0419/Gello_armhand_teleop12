from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Any, Iterable

from teleop_core.contract import (
    DATA_LEFT_ARM_JOINT_NAMES,
    DATA_RIGHT_ARM_JOINT_NAMES,
    LEFT_COMMAND_JOINT_NAMES,
    RIGHT_COMMAND_JOINT_NAMES,
    VALIDATED_COMMAND_TOPIC,
    WUJI_COMMAND_TOPIC,
    WUJI_LEFT_JOINT_NAMES,
    WUJI_RIGHT_JOINT_NAMES,
    WUJI_STATE_TOPIC,
    WUJI_TELEMETRY_STATUS_TOPIC,
)


@dataclass
class StreamSamples:
    bag_times_ns: list[int] = field(default_factory=list)
    header_times_ns: list[int] = field(default_factory=list)


def stream_timing_report(
    samples: StreamSamples,
    *,
    global_start_ns: int,
    global_end_ns: int,
) -> dict[str, Any]:
    times = samples.bag_times_ns
    gaps = [current - previous for previous, current in zip(times, times[1:])]
    duration_ns = max(0, global_end_ns - global_start_ns)
    observed_duration_ns = max(0, times[-1] - times[0]) if times else 0
    frequency_hz = (
        (len(times) - 1) * 1_000_000_000 / observed_duration_ns
        if len(times) > 1 and observed_duration_ns > 0
        else 0.0
    )
    header_nonmonotonic = sum(
        current <= previous
        for previous, current in zip(
            samples.header_times_ns, samples.header_times_ns[1:]
        )
    )
    return {
        "message_count": len(times),
        "first_bag_time_ns": times[0] if times else None,
        "last_bag_time_ns": times[-1] if times else None,
        "recording_duration_ns": duration_ns,
        "frequency_hz": frequency_hz,
        "max_internal_gap_ms": (max(gaps) / 1_000_000 if gaps else None),
        "leading_gap_ms": (
            (times[0] - global_start_ns) / 1_000_000 if times else None
        ),
        "trailing_gap_ms": (
            (global_end_ns - times[-1]) / 1_000_000 if times else None
        ),
        "source_header_count": len(samples.header_times_ns),
        "first_source_header_ns": (
            samples.header_times_ns[0] if samples.header_times_ns else None
        ),
        "last_source_header_ns": (
            samples.header_times_ns[-1] if samples.header_times_ns else None
        ),
        "source_header_nonmonotonic": header_nonmonotonic,
    }


def timing_failures(
    name: str,
    report: dict[str, Any],
    *,
    min_frequency_hz: float,
    max_gap_ms: float,
) -> list[str]:
    if report["message_count"] == 0:
        return [f"{name}: no messages in bag"]
    failures = []
    if report["message_count"] < 2:
        failures.append(f"{name}: fewer than two messages")
    elif report["frequency_hz"] < min_frequency_hz:
        failures.append(
            f"{name}: {report['frequency_hz']:.2f} Hz below "
            f"{min_frequency_hz:.2f} Hz"
        )
    for field_name in ("max_internal_gap_ms", "leading_gap_ms", "trailing_gap_ms"):
        gap = report[field_name]
        if gap is not None and gap > max_gap_ms:
            failures.append(
                f"{name}: {field_name} {gap:.1f} ms exceeds {max_gap_ms:.1f} ms"
            )
    if report["source_header_count"] != report["message_count"]:
        failures.append(f"{name}: source header timestamps are missing")
    if report["source_header_nonmonotonic"]:
        failures.append(f"{name}: source header timestamps are not monotonic")
    return failures


def _message_header_ns(message: Any) -> int | None:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    timestamp = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return timestamp if timestamp > 0 else None


def _validate_named_message(
    message: Any,
    expected_names: tuple[str, ...],
    *,
    fields: tuple[str, ...],
    label: str,
    allow_extra_names: bool = False,
) -> None:
    names = tuple(str(name) for name in message.name)
    if len(names) != len(set(names)):
        raise ValueError(f"{label}: duplicate joint names")
    if allow_extra_names:
        if not set(expected_names).issubset(names):
            raise ValueError(f"{label}: required joint group is partial")
    elif set(names) != set(expected_names):
        missing = sorted(set(expected_names).difference(names))
        unknown = sorted(set(names).difference(expected_names))
        raise ValueError(
            f"{label}: joint-name mismatch missing={missing}, unknown={unknown}"
        )
    for selected in fields:
        values = getattr(message, selected)
        if len(values) != len(names):
            raise ValueError(
                f"{label}: {selected} length does not match joint names"
            )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError(f"{label}: {selected} contains a non-finite value")


def inspect_bag(
    bag_path: Path,
    topic_contracts: Iterable[Any],
) -> tuple[tuple[str, ...], dict[str, Any]]:
    """Read every recorded message and return completeness failures/report."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    configured = {item.topic: item for item in topic_contracts}
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag_path),
            storage_id=_storage_id(bag_path),
        ),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    actual_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    message_classes = {
        topic: get_message(type_name)
        for topic, type_name in actual_types.items()
        if topic in configured
    }
    streams: dict[str, StreamSamples] = defaultdict(StreamSamples)
    content_failures: list[str] = []
    telemetry_counters = {
        "max_sender_dropped_packets": 0,
        "max_receiver_lost_packets": 0,
        "max_receiver_duplicate_packets": 0,
        "max_receiver_out_of_order_packets": 0,
        "max_receiver_stale_packets": 0,
        "max_receiver_invalid_packets": 0,
        "velocity_sources": set(),
    }

    while reader.has_next():
        topic, serialized, bag_time_ns = reader.read_next()
        if topic not in configured or topic not in message_classes:
            continue
        message = deserialize_message(serialized, message_classes[topic])
        bag_time_ns = int(bag_time_ns)
        stream = streams[topic]
        stream.bag_times_ns.append(bag_time_ns)
        header_ns = _message_header_ns(message)
        if header_ns is not None:
            stream.header_times_ns.append(header_ns)
        try:
            _inspect_message_content(
                topic,
                message,
                bag_time_ns,
                header_ns,
                streams,
                telemetry_counters,
            )
        except ValueError as error:
            if len(content_failures) < 100:
                content_failures.append(str(error))

    required = [item for item in configured.values() if item.required]
    all_times = [
        timestamp
        for item in required
        for timestamp in streams[item.topic].bag_times_ns
    ]
    if not all_times:
        return ("bag contains no required messages",), {"streams": {}}
    global_start_ns = min(all_times)
    global_end_ns = max(all_times)
    failures = list(content_failures)
    reports = {}
    for item in required:
        report = stream_timing_report(
            streams[item.topic],
            global_start_ns=global_start_ns,
            global_end_ns=global_end_ns,
        )
        reports[item.topic] = report
        failures.extend(
            timing_failures(
                item.topic,
                report,
                min_frequency_hz=float(item.min_frequency_hz),
                max_gap_ms=float(item.max_gap_ms),
            )
        )

    # The combined validated action bus must independently cover both sides.
    arm_policy = configured.get(VALIDATED_COMMAND_TOPIC)
    if arm_policy is not None:
        for side in ("left", "right"):
            name = f"{VALIDATED_COMMAND_TOPIC}#{side}"
            report = stream_timing_report(
                streams[name],
                global_start_ns=global_start_ns,
                global_end_ns=global_end_ns,
            )
            reports[name] = report
            failures.extend(
                timing_failures(
                    name,
                    report,
                    min_frequency_hz=float(arm_policy.min_frequency_hz),
                    max_gap_ms=float(arm_policy.max_gap_ms),
                )
            )

    for key, value in telemetry_counters.items():
        if key == "velocity_sources":
            continue
        if int(value) > 0:
            failures.append(f"hand telemetry reports {key}={value}")
    velocity_sources = sorted(telemetry_counters["velocity_sources"])
    if velocity_sources and velocity_sources != ["finite_difference"]:
        failures.append(
            "hand state velocity provenance is not exclusively finite_difference"
        )

    report = {
        "bag_start_time_ns": global_start_ns,
        "bag_end_time_ns": global_end_ns,
        "streams": reports,
        "telemetry": {
            **{
                key: value
                for key, value in telemetry_counters.items()
                if key != "velocity_sources"
            },
            "velocity_sources": velocity_sources,
        },
    }
    return tuple(dict.fromkeys(failures)), report


def _inspect_message_content(
    topic: str,
    message: Any,
    bag_time_ns: int,
    header_ns: int | None,
    streams: dict[str, StreamSamples],
    telemetry_counters: dict[str, Any],
) -> None:
    hand_names = {
        "left": WUJI_LEFT_JOINT_NAMES,
        "right": WUJI_RIGHT_JOINT_NAMES,
    }
    if topic == "/cam0/depth/image_raw":
        height = int(message.height)
        width = int(message.width)
        step = int(message.step)
        if (width, height) != (640, 400):
            raise ValueError(
                f"head depth shape must be 640x400, got {width}x{height}"
            )
        if str(message.encoding) != "16UC1":
            raise ValueError(
                f"head depth encoding must be 16UC1, got {message.encoding!r}"
            )
        if step != width * 2 or len(message.data) != step * height:
            raise ValueError("head depth byte layout is not packed uint16")
        return
    if topic == VALIDATED_COMMAND_TOPIC:
        names = set(str(name) for name in message.name)
        for side, expected in (
            ("left", LEFT_COMMAND_JOINT_NAMES),
            ("right", RIGHT_COMMAND_JOINT_NAMES),
        ):
            overlap = names.intersection(expected)
            if overlap and not set(expected).issubset(names):
                raise ValueError(f"validated arm action has partial {side} side")
            if set(expected).issubset(names):
                _validate_named_message(
                    message,
                    expected,
                    fields=("position",),
                    label=f"validated {side} arm action",
                    allow_extra_names=True,
                )
                virtual = streams[f"{VALIDATED_COMMAND_TOPIC}#{side}"]
                virtual.bag_times_ns.append(bag_time_ns)
                if header_ns is not None:
                    virtual.header_times_ns.append(header_ns)
        return
    for side in ("left", "right"):
        if topic == f"/{side}/franka/joint_states":
            expected = (
                DATA_LEFT_ARM_JOINT_NAMES
                if side == "left"
                else DATA_RIGHT_ARM_JOINT_NAMES
            )
            _validate_named_message(
                message,
                expected,
                fields=("position", "velocity"),
                label=f"{side} arm state",
            )
            return
        if topic == WUJI_COMMAND_TOPIC.format(side=side):
            _validate_named_message(
                message,
                hand_names[side],
                fields=("position",),
                label=f"{side} hand command",
            )
            return
        if topic == WUJI_STATE_TOPIC.format(side=side):
            _validate_named_message(
                message,
                hand_names[side],
                fields=("position", "velocity"),
                label=f"{side} hand state",
            )
            return
    if topic == WUJI_TELEMETRY_STATUS_TOPIC:
        for field_name in (
            "sender_dropped_packets",
            "receiver_lost_packets",
            "receiver_duplicate_packets",
            "receiver_out_of_order_packets",
            "receiver_stale_packets",
            "receiver_invalid_packets",
        ):
            key = f"max_{field_name}"
            telemetry_counters[key] = max(
                int(telemetry_counters[key]), int(getattr(message, field_name))
            )
        if bool(message.state_valid):
            telemetry_counters["velocity_sources"].add(
                str(message.state_velocity_source)
            )


def _storage_id(bag_path: Path) -> str:
    import yaml

    metadata = yaml.safe_load(
        (bag_path / "metadata.yaml").read_text(encoding="utf-8")
    )
    return str(
        metadata["rosbag2_bagfile_information"].get(
            "storage_identifier", "sqlite3"
        )
    )
