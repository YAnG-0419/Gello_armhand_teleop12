import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from teleop_interfaces.msg import ArmCommand

from .arbitration import SourceArbiter
from .contract import (
    ARM_COMMAND_TOPIC,
    ARM_STATE_TOPIC,
    RESET_ACTIVE_TOPIC,
    SOURCE_COMMAND_TOPIC,
    VALIDATED_COMMAND_TOPIC,
)
from .joint_state import ordered_arm_positions
from .safety import CommandSafetyGate


class SafetyGateway(Node):
    def __init__(self):
        super().__init__("teleop_safety_gateway")
        allowed_sources = self._required_parameter("allowed_sources")
        self.state_timeout = float(self._required_parameter("state_timeout"))
        command_timeout = float(self._required_parameter("command_timeout"))
        if self.state_timeout <= 0 or command_timeout <= 0:
            raise ValueError("State and command timeouts must be positive.")
        self.arbiter = SourceArbiter(
            allowed_sources, command_timeout
        )
        self.gate = CommandSafetyGate(
            max_joint_speed=float(self._required_parameter("max_joint_speed")),
            max_initial_delta=float(self._required_parameter("max_initial_delta")),
            nominal_dt=float(self._required_parameter("nominal_dt")),
            max_command_deviation=self.declare_parameter(
                "max_command_deviation", [0.0]
            ).value,
        )
        if self.gate.max_command_deviation is not None:
            self.get_logger().info(
                "Command deviation cap active (rad per joint): "
                + ", ".join(
                    f"{value:.3f}" for value in self.gate.max_command_deviation
                )
            )
        self.state = {"left": None, "right": None}
        self.state_at = {"left": None, "right": None}
        self.rejected = 0
        self.reset_active = False

        for side in ("left", "right"):
            self.create_subscription(
                JointState,
                ARM_STATE_TOPIC.format(side=side),
                lambda message, selected=side: self._state(selected, message),
                qos_profile_sensor_data,
            )
        self.create_subscription(ArmCommand, SOURCE_COMMAND_TOPIC, self._command, 10)
        reset_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, RESET_ACTIVE_TOPIC, self._reset_state, reset_qos)
        self.validated_publisher = self.create_publisher(
            JointState, VALIDATED_COMMAND_TOPIC, 10
        )
        self.hardware_publisher = self.create_publisher(
            JointState, ARM_COMMAND_TOPIC, 10
        )
        self.get_logger().info(
            "Teleoperation safety gateway is forwarding validated commands."
        )

    def _required_parameter(self, name):
        parameter = self.declare_parameter(name)
        if parameter.type_ == Parameter.Type.NOT_SET:
            raise ValueError(f"Required parameter '{name}' is missing")
        return parameter.value

    def _reset_state(self, message):
        self.reset_active = bool(message.data)
        self.arbiter.reset()
        self.gate.reset()

    def _state(self, side, message):
        try:
            self.state[side] = ordered_arm_positions(
                message.name, message.position, side
            )
            self.state_at[side] = time.monotonic()
        except ValueError as exc:
            self._reject(str(exc))

    def _measured(self, now):
        if any(self.state[side] is None for side in ("left", "right")):
            return None
        if any(now - self.state_at[side] > self.state_timeout for side in ("left", "right")):
            return None
        return np.concatenate((self.state["left"], self.state["right"]))

    def _reject(self, reason):
        self.rejected += 1
        if self.rejected <= 3 or self.rejected % 100 == 0:
            self.get_logger().warn(f"Rejected command: {reason}")

    def _command(self, message):
        if self.reset_active:
            return
        now = time.monotonic()
        measured = self._measured(now)
        if measured is None:
            self.arbiter.reset()
            self.gate.reset()
            self._reject("dual-arm state is missing or stale")
            return
        try:
            new_session = self.arbiter.accept(
                message.source,
                message.session_id,
                int(message.sequence),
                now,
            )
            if new_session:
                self.gate.reset()
            validated = self.gate.validate(
                message.active_sides,
                message.joint_names,
                message.positions,
                measured,
                now,
            )
        except ValueError as exc:
            self._reject(str(exc))
            return
        if validated is None:
            self.gate.reset()
            return
        output = JointState()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = message.source
        output.name = list(validated.names)
        output.position = list(validated.positions)
        self.validated_publisher.publish(output)
        self.hardware_publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
