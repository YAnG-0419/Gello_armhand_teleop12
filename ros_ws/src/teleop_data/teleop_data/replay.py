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

from .config import load_config
from .lerobot_io import load_lerobot_episode
from .portable_bag import (
    G20_JOINT_NAMES,
    HAND_ACTION_TOPIC,
    HAND_STATE_TOPIC,
    ordered_hand_positions,
)


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
    def __init__(self, state_timeout):
        super().__init__("teleop_replay")
        self.state_timeout = state_timeout
        self.publisher = self.create_publisher(ArmCommand, SOURCE_COMMAND_TOPIC, 10)
        self.state = {"left": None, "right": None}
        self.state_at = {"left": None, "right": None}
        self.hand_state = {"left": None, "right": None}
        self.hand_state_at = {"left": None, "right": None}
        self.hand_publishers = {
            side: self.create_publisher(
                JointState, HAND_ACTION_TOPIC.format(side=side), 10
            )
            for side in ("left", "right")
        }
        for side in ("left", "right"):
            self.create_subscription(
                JointState,
                ARM_STATE_TOPIC.format(side=side),
                lambda message, selected=side: self._state(selected, message),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                JointState,
                HAND_STATE_TOPIC.format(side=side),
                lambda message, selected=side: self._hand_state(selected, message),
                10,
            )
        self.session = uuid.uuid4().hex
        self.sequence = 0

    def _state(self, side, message):
        self.state[side] = ordered_arm_positions(message.name, message.position, side)
        self.state_at[side] = time.monotonic()

    def _hand_state(self, side, message):
        try:
            positions = ordered_hand_positions(message.name, message.position)
        except ValueError:
            return
        self.hand_state[side] = positions
        self.hand_state_at[side] = time.monotonic()

    def measured(self):
        if any(self.state[side] is None for side in ("left", "right")):
            return None
        if any(
            time.monotonic() - self.state_at[side] > self.state_timeout
            for side in ("left", "right")
        ):
            return None
        return np.concatenate((self.state["left"], self.state["right"]))

    def measured_hands(self):
        if any(self.hand_state[side] is None for side in ("left", "right")):
            return None
        if any(
            time.monotonic() - self.hand_state_at[side] > self.state_timeout
            for side in ("left", "right")
        ):
            return None
        return np.concatenate((self.hand_state["left"], self.hand_state["right"]))

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

    def publish_hands(self, positions, active_sides=("left", "right")):
        positions = np.asarray(positions, dtype=float)
        if (
            positions.shape != (40,)
            or not np.all(np.isfinite(positions))
            or np.any(positions < 0.0)
            or np.any(positions > 255.0)
        ):
            raise ValueError("Replay hand actions must be 40 values in [0, 255].")
        for side_index, side in enumerate(("left", "right")):
            if side not in active_sides:
                continue
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(G20_JOINT_NAMES)
            start = side_index * 20
            message.position = positions[start:start + 20].tolist()
            self.hand_publishers[side].publish(message)


def load_episode(path, episode_index=0):
    timestamp, arm_action, _, active = load_complete_episode(path, episode_index)
    return timestamp, arm_action, active


def load_complete_episode(path, episode_index=0):
    path = Path(path)
    if path.is_dir() and (path / "meta" / "info.json").is_file():
        timestamp, action, active = load_lerobot_episode(path, episode_index)
    else:
        timestamp, action, active = _load_npz_episode(path)
    if action.ndim != 2 or action.shape[1] not in {14, 54}:
        raise ValueError("Episode actions must contain 14 arm or 54 arm/hand values.")
    arm_action = action[:, :14]
    hand_action = action[:, 14:] if action.shape[1] == 54 else None
    if (
        timestamp.ndim != 1
        or arm_action.shape != (timestamp.size, 14)
        or (hand_action is not None and hand_action.shape != (timestamp.size, 40))
        or active.shape != (timestamp.size, 2)
    ):
        raise ValueError("Episode timestamps and actions have invalid dimensions.")
    if timestamp.size < 2 or np.any(np.diff(timestamp) < 0):
        raise ValueError("Episode needs at least two ordered frames.")
    if not np.all(np.isfinite(timestamp)) or not np.all(np.isfinite(action)):
        raise ValueError("Episode contains non-finite values.")
    if hand_action is not None and (
        np.any(hand_action < 0.0) or np.any(hand_action > 255.0)
    ):
        raise ValueError("Episode hand actions must be within [0, 255].")
    return timestamp - timestamp[0], arm_action, hand_action, active


def _load_npz_episode(path):
    with np.load(path, allow_pickle=False) as data:
        schema = str(data["schema_version"])
        if schema == "franka.teleop.normalized.v1":
            action = np.asarray(data["action_arm_joint_position"], dtype=float)
        elif schema == "franka_linker.teleop.normalized.v2":
            action = np.asarray(data["action_joint_position"], dtype=float)
        else:
            raise ValueError(f"Unsupported episode schema: {schema}")
        timestamp = np.asarray(data["timestamp"], dtype=float)
        active = np.asarray(data["active_sides"], dtype=np.bool_)
    return timestamp, action, active


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", type=Path)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--episode-index", type=int, default=0)
    args = parser.parse_args()
    config = load_config(args.config)
    timestamp, action, hand_action, active = load_complete_episode(
        args.episode, args.episode_index
    )
    rclpy.init()
    node = ReplayPublisher(config.replay_state_timeout)
    try:
        deadline = time.monotonic() + config.replay_discovery_timeout
        while (
            node.measured() is None
            or (hand_action is not None and node.measured_hands() is None)
        ) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        measured = node.measured()
        if measured is None:
            raise TimeoutError("No fresh dual-FR3 state was received.")
        measured_hands = node.measured_hands() if hand_action is not None else None
        if hand_action is not None and measured_hands is None:
            raise TimeoutError("No fresh dual-LinkerHand state was received.")
        period = 1.0 / config.replay_rate
        arm_duration = max(
            float(np.max(np.abs(action[0] - measured)))
            / config.replay_preposition_speed,
            period,
        )
        hand_duration = (
            float(np.max(np.abs(hand_action[0] - measured_hands)))
            / config.replay_hand_preposition_speed
            if hand_action is not None
            else 0.0
        )
        duration = max(arm_duration, hand_duration, period)
        count = int(np.ceil(duration * config.replay_rate)) + 1
        progress = smootherstep(np.linspace(0.0, 1.0, count))[:, None]
        arm_targets = measured + progress * (action[0] - measured)
        hand_targets = (
            measured_hands + progress * (hand_action[0] - measured_hands)
            if hand_action is not None
            else None
        )
        last_hand_publish = -np.inf
        for index, target in enumerate(arm_targets):
            started = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0)
            if node.measured() is None:
                raise RuntimeError("Robot state became stale during prepositioning.")
            if hand_action is not None and node.measured_hands() is None:
                raise RuntimeError("Hand state became stale during prepositioning.")
            node.publish(target)
            if hand_targets is not None and (
                started - last_hand_publish >= 1.0 / 30.0
                or index == len(arm_targets) - 1
            ):
                node.publish_hands(hand_targets[index])
                last_hand_publish = started
            time.sleep(max(0.0, period - (time.monotonic() - started)))
        started = time.monotonic()
        for frame_index, (frame_time, target, active_mask) in enumerate(
            zip(timestamp, action, active)
        ):
            deadline = started + frame_time / config.replay_speed
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=min(0.01, deadline - time.monotonic()))
            if node.measured() is None:
                raise RuntimeError("Robot state became stale during replay.")
            if hand_action is not None and node.measured_hands() is None:
                raise RuntimeError("Hand state became stale during replay.")
            sides = tuple(
                side
                for side, selected in zip(("left", "right"), active_mask)
                if selected
            )
            node.publish(target, sides)
            if hand_action is not None:
                node.publish_hands(hand_action[frame_index], sides)
        node.publish(action[-1], ())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
