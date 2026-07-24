import argparse
from pathlib import Path

import numpy as np
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
from teleop_core.contract import (
    ARM_STATE_TOPIC,
    VALIDATED_COMMAND_TOPIC,
)
from teleop_core.joint_state import ordered_arm_positions

from .config import load_config
from .timeseries import TimeSeries, fixed_rate_times


def message_time(message, receive_ns):
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    if stamp is not None and (stamp.sec or stamp.nanosec):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return float(receive_ns) * 1e-9


def read_series(path):
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(Path(path).resolve()), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    required = {
        ARM_STATE_TOPIC.format(side="left"),
        ARM_STATE_TOPIC.format(side="right"),
        VALIDATED_COMMAND_TOPIC,
    }
    missing = sorted(required.difference(types))
    if missing:
        raise ValueError("Bag is missing required topics: " + ", ".join(missing))
    raw = {"left_state": [], "right_state": [], "action": [], "active": []}
    while reader.has_next():
        topic, payload, receive_ns = reader.read_next()
        if topic not in required:
            continue
        message = deserialize_message(payload, get_message(types[topic]))
        timestamp = message_time(message, receive_ns)
        if topic == VALIDATED_COMMAND_TOPIC:
            active = tuple(
                side for side in ("left", "right")
                if any(side in name.lower() for name in message.name)
            )
            raw["action"].append((timestamp, (message.name, message.position)))
            raw["active"].append((timestamp, [side in active for side in ("left", "right")]))
        else:
            side = "left" if topic == ARM_STATE_TOPIC.format(side="left") else "right"
            raw[f"{side}_state"].append(
                (timestamp, ordered_arm_positions(message.name, message.position, side))
            )
    if any(not raw[key] for key in raw):
        raise ValueError("Bag does not contain complete states and validated actions.")
    return raw


def normalize_episode(raw, fps):
    left = TimeSeries.from_samples(raw["left_state"])
    right = TimeSeries.from_samples(raw["right_state"])
    action_time = np.asarray([sample[0] for sample in raw["action"]])
    start = max(left.time[0], right.time[0], action_time[0])
    end = min(left.time[-1], right.time[-1], action_time[-1])
    timeline = fixed_rate_times(start, end, fps)
    observation = np.concatenate((left.linear(timeline), right.linear(timeline)), axis=1)
    action = observation.copy()
    active = np.zeros((timeline.size, 2), dtype=np.bool_)
    event_index = 0
    held = observation[0].copy()
    held_active = np.zeros(2, dtype=np.bool_)
    for frame, timestamp in enumerate(timeline):
        while event_index < len(raw["action"]) and raw["action"][event_index][0] <= timestamp:
            _, (names, positions) = raw["action"][event_index]
            for side_index, side in enumerate(("left", "right")):
                if any(side in name.lower() for name in names):
                    held[side_index * 7:(side_index + 1) * 7] = ordered_arm_positions(
                        names, positions, side
                    )
            held_active = np.asarray(raw["active"][event_index][1], dtype=np.bool_)
            event_index += 1
        action[frame] = held
        active[frame] = held_active
    return {
        "timestamp": (timeline - timeline[0]).astype(np.float64),
        "observation_arm_joint_position": observation.astype(np.float32),
        "action_arm_joint_position": action.astype(np.float32),
        "active_sides": active,
        "fps": np.asarray(fps, dtype=np.int32),
        "schema_version": np.asarray("franka.teleop.normalized.v1"),
    }


def convert(input_path, output_path, fps):
    episode = normalize_episode(read_series(input_path), fps)
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output, **episode)
    return output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()
    config = load_config(args.config)
    print(convert(args.input, args.output, config.conversion_fps))
