#!/usr/bin/env python3
"""Move only the right FR3 arm to the configured initial joint pose."""

import math
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


STATE_TOPIC = "/right/franka/joint_states"
COMMAND_TOPIC = "/right/gello/joint_states"
RESET_ACTIVE_TOPIC = "/reset_to_initial_pose/active"
STATE_NAMES = tuple(f"right_fr3_joint{index}" for index in range(1, 8))
COMMAND_NAMES = tuple(f"fr3_joint{index}" for index in range(1, 8))
CONFIG_PATH = (
    Path(__file__).resolve().parents[1]
    / "ros_ws/src/franka_fr3_arm_controllers/config/initial_pose.yaml"
)
WORKCELL_PATH = Path(
    os.environ.get(
        "FRANKA_ROBOT_CONFIG",
        Path(__file__).resolve().parents[1] / "config/current_workcell.yaml",
    )
)
CONTROLLERS_PATH = (
    Path(__file__).resolve().parents[1]
    / "ros_ws/src/franka_fr3_arm_controllers/config/controllers.yaml"
)

STATE_MAX_AGE = 0.5
MAX_SPEED = 0.2
MAX_ACCELERATION = 0.4
MIN_DURATION = 1.0
RATE = 50.0
TOLERANCE = 0.04


def load_target():
    with CONFIG_PATH.open(encoding="utf-8") as config_file:
        entry = yaml.safe_load(config_file)["initial_pose"]["right"]
    if tuple(entry["joint_names"]) != STATE_NAMES:
        raise ValueError(f"Unexpected right-arm joint names in {CONFIG_PATH}")
    target = tuple(float(value) for value in entry["positions"])
    if len(target) != 7 or not all(math.isfinite(value) for value in target):
        raise ValueError(f"Invalid right-arm positions in {CONFIG_PATH}")
    return target


def load_robot_config():
    with WORKCELL_PATH.open(encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)["RIGHT"]
    required = (
        "arm_id",
        "arm_prefix",
        "controller_cpus",
        "fake_sensor_commands",
        "namespace",
        "robot_ip",
        "urdf_file",
        "use_fake_hardware",
    )
    if any(not str(config.get(name, "")).strip() for name in required):
        raise ValueError(f"Invalid RIGHT configuration in {WORKCELL_PATH}")
    if config["namespace"] != "right" or config["arm_prefix"] != "right":
        raise ValueError(f"RIGHT configuration must target only right in {WORKCELL_PATH}")
    return config


def smootherstep(progress):
    progress = min(1.0, max(0.0, progress))
    return progress**3 * (progress * (progress * 6.0 - 15.0) + 10.0)


def trajectory_duration(distance):
    peak_speed = 1.875
    peak_acceleration = 10.0 * math.sqrt(3.0) / 3.0
    if distance == 0.0:
        return 0.0
    return max(
        MIN_DURATION,
        peak_speed * distance / MAX_SPEED,
        math.sqrt(peak_acceleration * distance / MAX_ACCELERATION),
    )


class RightArmReset(Node):
    def __init__(self):
        super().__init__("reset_right_arm_once")
        self.positions = None
        self.state_time = None
        self.create_subscription(
            JointState, STATE_TOPIC, self._state, qos_profile_sensor_data
        )
        self.command_publisher = self.create_publisher(
            JointState, COMMAND_TOPIC, 10
        )
        active_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.active_publisher = self.create_publisher(
            Bool, RESET_ACTIVE_TOPIC, active_qos
        )

    def _state(self, message):
        positions = dict(zip(message.name, message.position))
        if all(
            name in positions and math.isfinite(positions[name])
            for name in STATE_NAMES
        ):
            self.positions = tuple(positions[name] for name in STATE_NAMES)
            self.state_time = time.monotonic()

    def state_is_fresh(self):
        return (
            self.positions is not None
            and self.state_time is not None
            and time.monotonic() - self.state_time < STATE_MAX_AGE
        )

    def set_active(self, active):
        message = Bool()
        message.data = active
        self.active_publisher.publish(message)

    def publish(self, positions):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(COMMAND_NAMES)
        message.position = list(positions)
        self.command_publisher.publish(message)


def spin_until_fresh(node, timeout):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.state_is_fresh():
            return
    raise RuntimeError("Timed out waiting for fresh right-arm joint state")


def start_right_controller():
    config = load_robot_config()
    launch_command = [
        "ros2",
        "launch",
        "franka_fr3_arm_controllers",
        "franka.launch.py",
        f"arm_id:={config['arm_id']}",
        f"arm_prefix:={config['arm_prefix']}",
        f"controller_cpus:={config['controller_cpus']}",
        f"fake_sensor_commands:={config['fake_sensor_commands']}",
        f"namespace:={config['namespace']}",
        f"robot_ip:={config['robot_ip']}",
        f"urdf_file:={config['urdf_file']}",
        f"use_fake_hardware:={config['use_fake_hardware']}",
    ]
    launch_process = subprocess.Popen(launch_command, start_new_session=True)
    try:
        subprocess.run(
            [
                "ros2",
                "run",
                "controller_manager",
                "spawner",
                "joint_impedance_controller",
                "--controller-manager",
                "/right/controller_manager",
                "--param-file",
                str(CONTROLLERS_PATH),
                "--controller-manager-timeout",
                "30",
            ],
            check=True,
        )
    except BaseException:
        stop_right_controller(launch_process)
        raise
    return launch_process


def stop_right_controller(launch_process):
    if launch_process.poll() is not None:
        return
    try:
        os.killpg(launch_process.pid, signal.SIGINT)
        launch_process.wait(timeout=15.0)
    except subprocess.TimeoutExpired:
        os.killpg(launch_process.pid, signal.SIGTERM)
        launch_process.wait(timeout=5.0)
    except ProcessLookupError:
        pass


def reset(node, target):
    spin_until_fresh(node, timeout=30.0)
    if node.count_subscribers(COMMAND_TOPIC) < 1:
        raise RuntimeError("No active right-arm joint controller")

    start = node.positions
    distance = max(abs(goal - current) for current, goal in zip(start, target))
    duration = trajectory_duration(distance)
    started = time.monotonic()
    period = 1.0 / RATE

    while rclpy.ok():
        cycle_started = time.monotonic()
        rclpy.spin_once(node, timeout_sec=0.0)
        if not node.state_is_fresh():
            raise RuntimeError("Right-arm joint state became stale during reset")
        elapsed = cycle_started - started
        progress = 1.0 if duration == 0.0 else elapsed / duration
        blend = smootherstep(progress)
        node.publish(
            current + blend * (goal - current)
            for current, goal in zip(start, target)
        )
        if progress >= 1.0:
            break
        time.sleep(max(0.0, period - (time.monotonic() - cycle_started)))

    deadline = time.monotonic() + 10.0
    settled_since = None
    error = math.inf
    while rclpy.ok() and time.monotonic() < deadline:
        cycle_started = time.monotonic()
        rclpy.spin_once(node, timeout_sec=0.0)
        if not node.state_is_fresh():
            raise RuntimeError("Right-arm joint state became stale while settling")
        node.publish(target)
        error = max(
            abs(goal - current) for current, goal in zip(node.positions, target)
        )
        if error <= TOLERANCE:
            settled_since = settled_since or time.monotonic()
            if time.monotonic() - settled_since >= 0.5:
                return duration, error
        else:
            settled_since = None
        time.sleep(max(0.0, period - (time.monotonic() - cycle_started)))
    raise RuntimeError(f"Right arm did not settle; maximum error is {error:.4f} rad")


def main():
    target = load_target()
    rclpy.init()
    node = RightArmReset()
    launch_process = None
    try:
        try:
            spin_until_fresh(node, timeout=1.0)
            print("Using the running right-arm controller.")
        except RuntimeError:
            print("Starting the right-arm driver and controller.")
            launch_process = start_right_controller()
        node.set_active(True)
        time.sleep(0.25)
        duration, error = reset(node, target)
        print(
            f"Right initial pose reached in {duration:.1f} s "
            f"(maximum error {error:.4f} rad)."
        )
    finally:
        node.set_active(False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if launch_process is not None:
            stop_right_controller(launch_process)


if __name__ == "__main__":
    main()
