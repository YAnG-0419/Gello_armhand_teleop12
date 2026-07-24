import argparse
import time
import uuid
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from teleop_core.contract import (
    ARM_STATE_TOPIC,
    COMMAND_JOINT_NAMES,
    SOURCE_COMMAND_TOPIC,
)
from teleop_core.joint_state import ordered_arm_positions
from teleop_interfaces.msg import ArmCommand


def smootherstep(value):
    value = np.clip(value, 0.0, 1.0)
    return value ** 3 * (value * (value * 6.0 - 15.0) + 10.0)


def preposition_trajectory(start, target, speed, rate):
    start = np.asarray(start, dtype=float)
    target = np.asarray(target, dtype=float)
    if start.shape != (14,) or target.shape != (14,):
        raise ValueError("Preposition states must contain 14 joints.")
    if speed <= 0 or rate <= 0:
        raise ValueError("Preposition speed and rate must be positive.")
    duration = max(float(np.max(np.abs(target - start))) / speed, 1.0 / rate)
    count = int(np.ceil(duration * rate)) + 1
    progress = smootherstep(np.linspace(0.0, 1.0, count))[:, None]
    return start + progress * (target - start)


class ReplayPublisher(Node):
    def __init__(self):
        super().__init__("teleop_replay")
        self.publisher = self.create_publisher(ArmCommand, SOURCE_COMMAND_TOPIC, 10)
        self.state = {"left": None, "right": None}
        self.state_at = {"left": None, "right": None}
        for side in ("left", "right"):
            self.create_subscription(
                JointState,
                ARM_STATE_TOPIC.format(side=side),
                lambda message, selected=side: self._state(selected, message),
                qos_profile_sensor_data,
            )
        self.session = uuid.uuid4().hex
        self.sequence = 0

    def _state(self, side, message):
        self.state[side] = ordered_arm_positions(message.name, message.position, side)
        self.state_at[side] = time.monotonic()

    def measured(self):
        if any(self.state[side] is None for side in ("left", "right")):
            return None
        if any(time.monotonic() - self.state_at[side] > 0.25 for side in ("left", "right")):
            return None
        return np.concatenate((self.state["left"], self.state["right"]))

    def publish(self, positions, active_sides=("left", "right")):
        message = ArmCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.source = "replay"
        message.session_id = self.session
        message.sequence = self.sequence
        message.active_sides = list(active_sides)
        names = [
            name for name in COMMAND_JOINT_NAMES
            if any(side in name for side in active_sides)
        ]
        message.joint_names = names
        indices = [COMMAND_JOINT_NAMES.index(name) for name in names]
        message.positions = [float(positions[index]) for index in indices]
        self.publisher.publish(message)
        self.sequence += 1


def load_episode(path):
    with np.load(path, allow_pickle=False) as data:
        schema = str(data["schema_version"])
        if schema != "franka.teleop.normalized.v1":
            raise ValueError(f"Unsupported episode schema: {schema}")
        timestamp = np.asarray(data["timestamp"], dtype=float)
        action = np.asarray(data["action_arm_joint_position"], dtype=float)
        active = np.asarray(data["active_sides"], dtype=np.bool_)
    if (
        timestamp.ndim != 1
        or action.shape != (timestamp.size, 14)
        or active.shape != (timestamp.size, 2)
    ):
        raise ValueError("Episode timestamps and actions have invalid dimensions.")
    if timestamp.size < 2 or np.any(np.diff(timestamp) < 0):
        raise ValueError("Episode needs at least two ordered frames.")
    if not np.all(np.isfinite(timestamp)) or not np.all(np.isfinite(action)):
        raise ValueError("Episode contains non-finite values.")
    return timestamp, action, active


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", type=Path)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--preposition-speed", type=float, default=0.1)
    parser.add_argument("--rate", type=float, default=100.0)
    args = parser.parse_args()
    if args.speed <= 0:
        raise ValueError("Replay speed must be positive.")
    timestamp, action, active = load_episode(args.episode)
    rclpy.init()
    node = ReplayPublisher()
    try:
        deadline = time.monotonic() + 10
        while node.measured() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        measured = node.measured()
        if measured is None:
            raise TimeoutError("No fresh dual-FR3 state was received.")
        period = 1.0 / args.rate
        for target in preposition_trajectory(
            measured, action[0], args.preposition_speed, args.rate
        ):
            started = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0)
            if node.measured() is None:
                raise RuntimeError("Robot state became stale during prepositioning.")
            node.publish(target)
            time.sleep(max(0.0, period - (time.monotonic() - started)))
        started = time.monotonic()
        for frame_time, target, active_mask in zip(timestamp, action, active):
            deadline = started + frame_time / args.speed
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=min(0.01, deadline - time.monotonic()))
            if node.measured() is None:
                raise RuntimeError("Robot state became stale during replay.")
            sides = tuple(
                side
                for side, selected in zip(("left", "right"), active_mask)
                if selected
            )
            node.publish(target, sides)
        node.publish(action[-1], ())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
