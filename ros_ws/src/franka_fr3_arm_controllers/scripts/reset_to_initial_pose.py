#!/usr/bin/env python3
import math
import os
import tempfile
import threading
import time
from bisect import bisect_right
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
TASKS = {
    "powder_weighing": "粉末称量",
    "assembly": "装配",
    "bean_picking": "夹豆",
}
DEFAULT_TASK = "powder_weighing"
JOINT_COUNT = 7
STATE_MAX_AGE = 0.5
RESET_MAX_SPEED = 0.20
RESET_MAX_ACCELERATION = 0.40
RESET_MIN_DURATION = 1.0
READY_TOLERANCE = 0.05
# A separately captured Home and the final hand-guided recording frame can
# differ slightly after settling.  Keep the robot-at-Ready gate tight, while
# allowing this modest recording endpoint tolerance.
TRAJECTORY_ENDPOINT_TOLERANCE = 0.075
TRAJECTORY_MAX_RECORDED_SPEED = 1.0
SMOOTHERSTEP_PEAK_SPEED = 1.875
SMOOTHERSTEP_PEAK_ACCELERATION = 10.0 * math.sqrt(3.0) / 3.0


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


def _pose_entry(side, positions):
    return {
        "joint_names": [
            f"{side}_fr3_joint{index}" for index in range(1, JOINT_COUNT + 1)
        ],
        "positions": list(positions),
    }


def _validate_pose_mapping(value, label, allow_unrecorded=False):
    if not isinstance(value, dict):
        raise ValueError(f"Missing {label} mapping")
    targets = {}
    for side in SIDES:
        entry = value.get(side)
        if entry is None and allow_unrecorded:
            targets[side] = None
            continue
        expected = [
            f"{side}_fr3_joint{index}" for index in range(1, JOINT_COUNT + 1)
        ]
        if not isinstance(entry, dict) or entry.get("joint_names") != expected:
            raise ValueError(f"Invalid {label} {side} joint_names")
        positions = entry.get("positions")
        if (
            not isinstance(positions, list)
            or len(positions) != JOINT_COUNT
            or not all(isinstance(item, (int, float)) and math.isfinite(item) for item in positions)
        ):
            raise ValueError(f"Invalid {label} {side} positions")
        targets[side] = [float(item) for item in positions]
    return targets


def operator_pose_document(tasks, ready):
    return {
        "version": 1,
        "updated_utc": datetime.now(timezone.utc).isoformat(),
        "tasks": {
            task: {
                "label": TASKS[task],
                "home": {
                    side: (
                        None
                        if tasks[task][side] is None
                        else _pose_entry(side, tasks[task][side])
                    )
                    for side in SIDES
                },
            }
            for task in TASKS
        },
        "ready": (
            None
            if ready is None
            else {side: _pose_entry(side, ready[side]) for side in SIDES}
        ),
    }


def write_operator_poses(path, tasks, ready):
    directory = os.path.dirname(os.path.realpath(path))
    os.makedirs(directory, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=".operator_poses.", suffix=".tmp", dir=directory
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            yaml.safe_dump(
                operator_pose_document(tasks, ready),
                stream,
                allow_unicode=True,
                sort_keys=False,
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def load_operator_poses(path, legacy_targets):
    if not os.path.exists(path):
        tasks = {
            task: {
                side: (
                    list(legacy_targets[side]) if task == DEFAULT_TASK else None
                )
                for side in SIDES
            }
            for task in TASKS
        }
        write_operator_poses(path, tasks, None)
        return tasks, None
    with open(path, "r", encoding="utf-8") as stream:
        root = yaml.safe_load(stream)
    if not isinstance(root, dict) or root.get("version") != 1:
        raise ValueError("Invalid operator pose document version")
    configured = root.get("tasks")
    if not isinstance(configured, dict) or set(configured) != set(TASKS):
        raise ValueError("Operator pose document must contain all configured tasks")
    tasks = {}
    for task in TASKS:
        entry = configured[task]
        if not isinstance(entry, dict):
            raise ValueError(f"Invalid operator task: {task}")
        tasks[task] = _validate_pose_mapping(
            entry.get("home"), f"{task} Home", allow_unrecorded=True
        )
    ready_value = root.get("ready")
    ready = None if ready_value is None else _validate_pose_mapping(ready_value, "Ready")
    return tasks, ready


def load_absolute_trajectory(path, task):
    with open(path, "r", encoding="utf-8") as stream:
        root = yaml.safe_load(stream)
    if (
        not isinstance(root, dict)
        or root.get("version") != 1
        or root.get("kind") != "dual_arm_absolute_joint_trajectory"
        or root.get("task") != task
    ):
        raise ValueError("Invalid dual-arm absolute trajectory header")
    joint_names = root.get("joint_names")
    for side in SIDES:
        expected = [f"{side}_fr3_joint{index}" for index in range(1, 8)]
        if not isinstance(joint_names, dict) or joint_names.get(side) != expected:
            raise ValueError(f"Invalid trajectory {side} joint_names")
    raw_samples = root.get("samples")
    if not isinstance(raw_samples, list) or len(raw_samples) < 3:
        raise ValueError("Absolute trajectory must contain at least 3 samples")
    samples = []
    for item in raw_samples:
        if not isinstance(item, dict) or not isinstance(item.get("positions"), dict):
            raise ValueError("Invalid absolute trajectory sample")
        timestamp = float(item.get("time_sec"))
        positions = {
            side: [float(value) for value in item["positions"].get(side, [])]
            for side in SIDES
        }
        if (
            not math.isfinite(timestamp)
            or timestamp < 0.0
            or any(len(positions[side]) != JOINT_COUNT for side in SIDES)
            or any(
                not math.isfinite(value)
                for side in SIDES
                for value in positions[side]
            )
        ):
            raise ValueError("Invalid absolute trajectory sample values")
        samples.append((timestamp, positions))
    times = [item[0] for item in samples]
    if abs(times[0]) > 1e-6 or any(
        times[index] <= times[index - 1] for index in range(1, len(times))
    ):
        raise ValueError("Absolute trajectory times must start at zero and increase")
    if times[-1] < 0.2 or times[-1] > 120.0:
        raise ValueError("Absolute trajectory duration is outside 0.2-120 seconds")
    return samples


def smootherstep(progress):
    progress = min(1.0, max(0.0, progress))
    return progress**3 * (progress * (progress * 6.0 - 15.0) + 10.0)


def reset_duration(
    max_distance,
    max_speed=RESET_MAX_SPEED,
    max_acceleration=RESET_MAX_ACCELERATION,
    min_duration=RESET_MIN_DURATION,
):
    values = (max_distance, max_speed, max_acceleration, min_duration)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Reset trajectory limits must be finite")
    if max_distance < 0 or max_speed <= 0 or max_acceleration <= 0:
        raise ValueError("Reset distance must be non-negative and limits positive")
    if min_duration < 0:
        raise ValueError("Reset minimum duration must be non-negative")
    if max_distance == 0:
        return 0.0
    velocity_duration = (
        SMOOTHERSTEP_PEAK_SPEED * max_distance / max_speed
    )
    acceleration_duration = math.sqrt(
        SMOOTHERSTEP_PEAK_ACCELERATION
        * max_distance
        / max_acceleration
    )
    return max(min_duration, velocity_duration, acceleration_duration)


def capture_document(targets):
    return {
        "captured_utc": datetime.now(timezone.utc).isoformat(),
        "initial_pose": {
            side: {
                "joint_names": [
                    f"{side}_fr3_joint{index}"
                    for index in range(1, JOINT_COUNT + 1)
                ],
                "positions": list(targets[side]),
            }
            for side in SIDES
        },
    }


def write_targets(path, targets):
    """Atomically replace the persisted Home pose."""
    directory = os.path.dirname(os.path.realpath(path))
    descriptor, temporary = tempfile.mkstemp(
        prefix=".initial_pose.", suffix=".tmp", dir=directory
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as config_file:
            yaml.safe_dump(
                capture_document(targets), config_file, sort_keys=False
            )
            config_file.flush()
            os.fsync(config_file.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


class InitialPoseReset(Node):
    def __init__(
        self,
        config_path,
        targets,
        operator_config_path="/data/operator_gui/poses.yaml",
        trajectory_root="/data/arm_ui/absolute_trajectories",
    ):
        super().__init__("reset_to_initial_pose")
        self.config_path = os.path.realpath(config_path)
        self.targets = targets
        self.operator_config_path = os.path.realpath(operator_config_path)
        self.trajectory_root = os.path.realpath(trajectory_root)
        self.task_targets, self.ready_target = load_operator_poses(
            self.operator_config_path, targets
        )
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
        for task in TASKS:
            self.create_service(
                Trigger,
                f"/reset_to_home/{task}",
                lambda request, response, selected=task: self._reset_home(
                    request, response, selected
                ),
                callback_group=callbacks,
            )
            self.create_service(
                Trigger,
                f"/ready_to_home/{task}",
                lambda request, response, selected=task: self._ready_to_home(
                    request, response, selected
                ),
                callback_group=callbacks,
            )
            for side in SIDES:
                self.create_service(
                    Trigger,
                    f"/reset_to_home/{task}/{side}",
                    lambda request, response, selected_task=task, selected_side=side: self._reset_home(
                        request, response, selected_task, sides=(selected_side,)
                    ),
                    callback_group=callbacks,
                )
                self.create_service(
                    Trigger,
                    f"/capture_home/{task}/{side}",
                    lambda request, response, selected_task=task, selected_side=side: self._capture_task_home(
                        request, response, selected_task, selected_side
                    ),
                    callback_group=callbacks,
                )
        self.create_service(
            Trigger,
            "/reset_to_ready",
            self._reset_ready,
            callback_group=callbacks,
        )
        self.create_service(
            Trigger,
            "/capture_ready",
            self._capture_ready,
            callback_group=callbacks,
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
        # Per-side homing: only the named arm moves; the session-wide
        # active flag still pauses teleop, so the other arm just holds.
        for side in SIDES:
            self.create_service(
                Trigger,
                f"/reset_to_initial_pose/{side}",
                lambda request, response, selected=side: self._reset(
                    request, response, sides=(selected,)
                ),
                callback_group=callbacks,
            )
        self.create_service(
            Trigger,
            "/capture_initial_pose",
            self._capture,
            callback_group=callbacks,
        )
        for side in SIDES:
            self.create_service(
                Trigger,
                f"/capture_initial_pose/{side}",
                lambda request, response, selected=side: self._capture(
                    request, response, sides=(selected,)
                ),
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
        for side in positions:
            message = JointState()
            message.header.stamp = stamp
            message.name = self.command_names
            message.position = positions[side]
            self.command_publishers[side].publish(message)

    def _states_are_fresh(self, sides=SIDES):
        now = time.monotonic()
        return all(
            side in self.states
            and side in self.state_times
            and now - self.state_times[side] < STATE_MAX_AGE
            for side in sides
        )

    def _wait_for_fresh_states(self, sides=SIDES, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._states_are_fresh(sides):
                return
            time.sleep(0.05)
        raise RuntimeError(
            "Timed out waiting for fresh joint states: " + ", ".join(sides)
        )

    def _check_controllers(self, sides=SIDES):
        missing = [
            side
            for side in sides
            if self.count_subscribers(f"/{side}/gello/joint_states") < 1
        ]
        if missing:
            raise RuntimeError("No active joint controller for: " + ", ".join(missing))

    def _move(
        self,
        sides=SIDES,
        targets=None,
        max_speed=RESET_MAX_SPEED,
        max_acceleration=RESET_MAX_ACCELERATION,
        rate=50.0,
        tolerance=0.04,
    ):
        selected_targets = self.targets if targets is None else targets
        starts = {side: list(self.states[side]) for side in sides}
        max_distance = max(
            abs(target - current)
            for side in sides
            for current, target in zip(starts[side], selected_targets[side])
        )
        duration = reset_duration(
            max_distance,
            max_speed=max_speed,
            max_acceleration=max_acceleration,
        )
        self.get_logger().info(
            f"Reset trajectory: maximum joint distance {max_distance:.3f} rad, "
            f"duration {duration:.1f} s, peak limits {max_speed:.2f} rad/s "
            f"and {max_acceleration:.2f} rad/s^2."
        )
        period = 1.0 / rate
        started = time.monotonic()

        while rclpy.ok():
            cycle_started = time.monotonic()
            if not self._states_are_fresh(sides):
                raise RuntimeError("Joint state became stale during reset")
            elapsed = cycle_started - started
            progress = 1.0 if duration == 0.0 else elapsed / duration
            blend = smootherstep(progress)
            positions = {
                side: [
                    current + blend * (target - current)
                    for current, target in zip(starts[side], selected_targets[side])
                ]
                for side in sides
            }
            self._publish(positions)
            if progress >= 1.0:
                break
            time.sleep(max(0.0, period - (time.monotonic() - cycle_started)))

        deadline = time.monotonic() + 10.0
        settled_since = None
        error = math.inf
        while rclpy.ok() and time.monotonic() < deadline:
            cycle_started = time.monotonic()
            if not self._states_are_fresh(sides):
                raise RuntimeError("Joint state became stale during reset")
            self._publish({side: selected_targets[side] for side in sides})
            error = max(
                abs(target - current)
                for side in sides
                for current, target in zip(self.states[side], selected_targets[side])
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

    def _run_target_reset(self, response, targets, label, sides=SIDES):
        if not self.lock.acquire(blocking=False):
            response.success = False
            response.message = "Reset is already running."
            return response
        self.running = True
        try:
            self._wait_for_fresh_states(sides)
            self._check_controllers(sides)
            self._set_active(True)
            time.sleep(0.25)
            duration, error = self._move(sides, targets=targets)
            response.success = True
            response.message = (
                f"{label} reached for {', '.join(sides)} in "
                f"{duration:.1f} s (maximum error {error:.4f} rad)."
            )
        except (OSError, RuntimeError, ValueError) as exception:
            self.get_logger().error(str(exception))
            response.success = False
            response.message = str(exception)
        finally:
            self._set_active(False)
            self.running = False
            self.lock.release()
        return response

    def _reset(self, _request, response, sides=SIDES):
        return self._run_target_reset(response, self.targets, "Initial pose", sides)

    def _reset_home(self, _request, response, task, sides=SIDES):
        missing = [side for side in sides if self.task_targets[task][side] is None]
        if missing:
            response.success = False
            response.message = (
                f"{TASKS[task]} Home has not been recorded for: "
                + ", ".join(missing)
            )
            return response
        return self._run_target_reset(
            response, self.task_targets[task], f"{TASKS[task]} Home", sides
        )

    def _reset_ready(self, _request, response):
        if self.ready_target is None:
            response.success = False
            response.message = "Ready pose has not been recorded."
            return response
        return self._run_target_reset(response, self.ready_target, "Ready", SIDES)

    def _capture(self, _request, response, sides=SIDES):
        if not self.lock.acquire(blocking=False):
            response.success = False
            response.message = "Reset or capture is already running."
            return response
        try:
            self._wait_for_fresh_states(sides)
            targets = {side: list(self.targets[side]) for side in SIDES}
            for side in sides:
                targets[side] = list(self.states[side])
            write_targets(self.config_path, targets)
            self.targets = targets
            response.success = True
            response.message = (
                f"Measured Home pose saved for {', '.join(sides)} to "
                f"{self.config_path}."
            )
        except (OSError, RuntimeError) as exception:
            self.get_logger().error(str(exception))
            response.success = False
            response.message = str(exception)
        finally:
            self.lock.release()
        return response

    def _capture_task_home(self, _request, response, task, side):
        if not self.lock.acquire(blocking=False):
            response.success = False
            response.message = "Reset or capture is already running."
            return response
        try:
            self._wait_for_fresh_states((side,))
            self.task_targets[task][side] = list(self.states[side])
            write_operator_poses(
                self.operator_config_path, self.task_targets, self.ready_target
            )
            response.success = True
            response.message = (
                f"Measured {TASKS[task]} Home saved for {side} to "
                f"{self.operator_config_path}."
            )
        except (OSError, RuntimeError, ValueError) as exception:
            self.get_logger().error(str(exception))
            response.success = False
            response.message = str(exception)
        finally:
            self.lock.release()
        return response

    def _capture_ready(self, _request, response):
        if not self.lock.acquire(blocking=False):
            response.success = False
            response.message = "Reset or capture is already running."
            return response
        try:
            self._wait_for_fresh_states(SIDES)
            self.ready_target = {
                side: list(self.states[side]) for side in SIDES
            }
            write_operator_poses(
                self.operator_config_path, self.task_targets, self.ready_target
            )
            response.success = True
            response.message = (
                f"Measured dual-arm Ready pose saved to {self.operator_config_path}."
            )
        except (OSError, RuntimeError, ValueError) as exception:
            self.get_logger().error(str(exception))
            response.success = False
            response.message = str(exception)
        finally:
            self.lock.release()
        return response

    @staticmethod
    def _maximum_pose_error(left, right):
        return max(
            abs(left[side][joint] - right[side][joint])
            for side in SIDES
            for joint in range(JOINT_COUNT)
        )

    def _play_absolute_samples(self, samples, speed_scale, rate=50.0):
        source_times = [sample[0] * speed_scale for sample in samples]
        period = 1.0 / rate
        started = time.monotonic()
        while rclpy.ok():
            cycle_started = time.monotonic()
            if not self._states_are_fresh(SIDES):
                raise RuntimeError("Joint state became stale during Ready-to-Home")
            elapsed = cycle_started - started
            if elapsed >= source_times[-1]:
                self._publish(samples[-1][1])
                break
            upper = max(1, bisect_right(source_times, elapsed))
            upper = min(upper, len(samples) - 1)
            lower = upper - 1
            span = source_times[upper] - source_times[lower]
            blend = (elapsed - source_times[lower]) / span
            positions = {
                side: [
                    samples[lower][1][side][joint]
                    + blend
                    * (
                        samples[upper][1][side][joint]
                        - samples[lower][1][side][joint]
                    )
                    for joint in range(JOINT_COUNT)
                ]
                for side in SIDES
            }
            self._publish(positions)
            time.sleep(max(0.0, period - (time.monotonic() - cycle_started)))
        return source_times[-1]

    def _ready_to_home(self, _request, response, task):
        if not self.lock.acquire(blocking=False):
            response.success = False
            response.message = "Reset or trajectory is already running."
            return response
        self.running = True
        try:
            self._wait_for_fresh_states(SIDES)
            self._check_controllers(SIDES)
            if self.ready_target is None:
                raise RuntimeError("Ready pose has not been recorded")
            missing_home = [
                side for side in SIDES if self.task_targets[task][side] is None
            ]
            if missing_home:
                raise RuntimeError(
                    f"{TASKS[task]} Home has not been recorded for: "
                    + ", ".join(missing_home)
                )
            trajectory_path = os.path.join(self.trajectory_root, f"{task}.yaml")
            samples = load_absolute_trajectory(trajectory_path, task)
            measured = {side: self.states[side] for side in SIDES}
            ready_error = self._maximum_pose_error(measured, self.ready_target)
            if ready_error > READY_TOLERANCE:
                raise RuntimeError(
                    f"Robot is not at Ready (maximum error {ready_error:.4f} rad; "
                    f"limit {READY_TOLERANCE:.4f} rad)"
                )
            first_error = self._maximum_pose_error(
                samples[0][1], self.ready_target
            )
            if first_error > TRAJECTORY_ENDPOINT_TOLERANCE:
                raise RuntimeError(
                    f"Trajectory start does not match Ready ({first_error:.4f} rad)"
                )
            last_error = self._maximum_pose_error(
                samples[-1][1], self.task_targets[task]
            )
            if last_error > TRAJECTORY_ENDPOINT_TOLERANCE:
                raise RuntimeError(
                    f"Trajectory end does not match {TASKS[task]} Home "
                    f"({last_error:.4f} rad)"
                )
            maximum_speed = max(
                abs(samples[index][1][side][joint] - samples[index - 1][1][side][joint])
                / (samples[index][0] - samples[index - 1][0])
                for index in range(1, len(samples))
                for side in SIDES
                for joint in range(JOINT_COUNT)
            )
            if maximum_speed > TRAJECTORY_MAX_RECORDED_SPEED:
                raise RuntimeError(
                    f"Recorded trajectory contains a {maximum_speed:.3f} rad/s jump"
                )
            speed_scale = max(1.0, maximum_speed / RESET_MAX_SPEED)
            self._set_active(True)
            time.sleep(0.25)
            duration = self._play_absolute_samples(samples, speed_scale)
            _, error = self._move(
                SIDES,
                targets=self.task_targets[task],
                max_speed=RESET_MAX_SPEED,
                max_acceleration=RESET_MAX_ACCELERATION,
            )
            response.success = True
            response.message = (
                f"Ready to {TASKS[task]} Home completed in {duration:.1f} s "
                f"(maximum error {error:.4f} rad)."
            )
        except (OSError, RuntimeError, ValueError) as exception:
            self.get_logger().error(str(exception))
            response.success = False
            response.message = str(exception)
        finally:
            self._set_active(False)
            self.running = False
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
