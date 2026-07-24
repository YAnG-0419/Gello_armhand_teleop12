#!/usr/bin/env python3
import math
import os
import threading
import time
from datetime import datetime, timezone

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


SIDES = ("left", "right")
JOINT_COUNT = 7


def load_targets(path):
    with open(path, "r", encoding="utf-8") as config_file:
        root = yaml.safe_load(config_file)
    if not isinstance(root, dict) or not isinstance(root.get("initial_pose"), dict):
        raise ValueError("Missing initial_pose mapping")

    targets = {}
    for side in SIDES:
        entry = root["initial_pose"].get(side)
        expected_names = [
            f"{side}_fr3_joint{index}" for index in range(1, JOINT_COUNT + 1)
        ]
        if not isinstance(entry, dict) or entry.get("joint_names") != expected_names:
            raise ValueError(f"Invalid {side} joint_names")
        positions = entry.get("positions")
        if (
            not isinstance(positions, list)
            or len(positions) != JOINT_COUNT
            or not all(
                isinstance(value, (int, float)) and math.isfinite(value)
                for value in positions
            )
        ):
            raise ValueError(f"Invalid {side} positions")
        targets[side] = [float(value) for value in positions]
    return targets


def smootherstep(progress):
    progress = min(1.0, max(0.0, progress))
    return progress**3 * (progress * (progress * 6.0 - 15.0) + 10.0)


class InitialPoseReset(Node):
    def __init__(self, config_path, targets):
        super().__init__("reset_to_initial_pose")
        self.config_path = os.path.realpath(config_path)
        self.targets = targets
        self.states = {}
        self.state_times = {}
        self.running = False
        self.lock = threading.Lock()
        callbacks = ReentrantCallbackGroup()

        self.command_names = [
            f"fr3_joint{index}" for index in range(1, JOINT_COUNT + 1)
        ]
        self.command_publishers = {}
        for side in SIDES:
            self.create_subscription(
                JointState,
                f"/{side}/franka/joint_states",
                lambda message, selected=side: self._state(selected, message),
                qos_profile_sensor_data,
                callback_group=callbacks,
            )
            self.command_publishers[side] = self.create_publisher(
                JointState, f"/{side}/gello/joint_states", 10
            )

        active_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.active_publisher = self.create_publisher(
            Bool, "/reset_to_initial_pose/active", active_qos
        )
        self.create_service(
            Trigger,
            "/reset_to_initial_pose",
            self._reset,
            callback_group=callbacks,
        )
        self.create_service(
            Trigger,
            "/capture_initial_pose",
            self._capture,
            callback_group=callbacks,
        )

    def _state(self, side, message):
        positions = dict(zip(message.name, message.position))
        names = [
            f"{side}_fr3_joint{index}" for index in range(1, JOINT_COUNT + 1)
        ]
        if all(name in positions and math.isfinite(positions[name]) for name in names):
            self.states[side] = [positions[name] for name in names]
            self.state_times[side] = time.monotonic()

    def _set_active(self, active):
        message = Bool()
        message.data = active
        self.active_publisher.publish(message)

    def _publish(self, positions):
        stamp = self.get_clock().now().to_msg()
        for side in SIDES:
            message = JointState()
            message.header.stamp = stamp
            message.name = self.command_names
            message.position = positions[side]
            self.command_publishers[side].publish(message)

    def _wait_for_fresh_states(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            now = time.monotonic()
            if all(
                side in self.states and now - self.state_times[side] < 0.5
                for side in SIDES
            ):
                return
            time.sleep(0.05)
        raise RuntimeError("Timed out waiting for fresh dual-arm joint states")

    def _check_controllers(self):
        missing = [
            side
            for side in SIDES
            if self.count_subscribers(f"/{side}/gello/joint_states") < 1
        ]
        if missing:
            raise RuntimeError("No active joint controller for: " + ", ".join(missing))

    def _move(self, speed=0.15, rate=50.0, tolerance=0.04):
        starts = {side: list(self.states[side]) for side in SIDES}
        max_distance = max(
            abs(target - current)
            for side in SIDES
            for current, target in zip(starts[side], self.targets[side])
        )
        duration = 0.0 if max_distance == 0.0 else 1.875 * max_distance / speed
        period = 1.0 / rate
        started = time.monotonic()

        while rclpy.ok():
            cycle_started = time.monotonic()
            elapsed = cycle_started - started
            progress = 1.0 if duration == 0.0 else elapsed / duration
            blend = smootherstep(progress)
            positions = {
                side: [
                    current + blend * (target - current)
                    for current, target in zip(starts[side], self.targets[side])
                ]
                for side in SIDES
            }
            self._publish(positions)
            if progress >= 1.0:
                break
            time.sleep(max(0.0, period - (time.monotonic() - cycle_started)))

        deadline = time.monotonic() + 10.0
        settled_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            cycle_started = time.monotonic()
            self._publish(self.targets)
            error = max(
                abs(target - current)
                for side in SIDES
                for current, target in zip(self.states[side], self.targets[side])
            )
            if error <= tolerance:
                settled_since = settled_since or time.monotonic()
                if time.monotonic() - settled_since >= 0.5:
                    return duration, error
            else:
                settled_since = None
            time.sleep(max(0.0, period - (time.monotonic() - cycle_started)))
        raise RuntimeError(
            f"Reset did not settle; maximum joint error is {error:.4f} rad"
        )

    def _reset(self, _request, response):
        if not self.lock.acquire(blocking=False):
            response.success = False
            response.message = "Reset is already running."
            return response
        self.running = True
        try:
            self._wait_for_fresh_states()
            self._check_controllers()
            self._set_active(True)
            time.sleep(0.25)
            duration, error = self._move()
            response.success = True
            response.message = (
                f"Initial pose reached in {duration:.1f} s "
                f"(maximum error {error:.4f} rad)."
            )
        except RuntimeError as exception:
            self.get_logger().error(str(exception))
            response.success = False
            response.message = str(exception)
        finally:
            self._set_active(False)
            self.running = False
            self.lock.release()
        return response

    def _capture(self, _request, response):
        if not self.lock.acquire(blocking=False):
            response.success = False
            response.message = "Reset or capture is already running."
            return response
        try:
            self._wait_for_fresh_states()
            targets = {side: list(self.states[side]) for side in SIDES}
            document = {
                "captured_utc": datetime.now(timezone.utc).isoformat(),
                "initial_pose": {
                    side: {
                        "joint_names": [
                            f"{side}_fr3_joint{index}"
                            for index in range(1, JOINT_COUNT + 1)
                        ],
                        "positions": targets[side],
                    }
                    for side in SIDES
                },
            }
            with open(self.config_path, "w", encoding="utf-8") as config_file:
                yaml.safe_dump(document, config_file, sort_keys=False)
            self.targets = targets
            response.success = True
            response.message = f"Measured pose saved to {self.config_path}."
        except (OSError, RuntimeError) as exception:
            self.get_logger().error(str(exception))
            response.success = False
            response.message = str(exception)
        finally:
            self.lock.release()
        return response


def main():
    config_path = os.path.join(
        get_package_share_directory("franka_fr3_arm_controllers"),
        "config",
        "initial_pose.yaml",
    )
    targets = load_targets(config_path)
    rclpy.init()
    node = InitialPoseReset(config_path, targets)
    node._set_active(False)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._set_active(False)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
