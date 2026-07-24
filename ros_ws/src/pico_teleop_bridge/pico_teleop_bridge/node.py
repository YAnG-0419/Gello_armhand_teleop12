import socket
import time
import uuid

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from teleop_core.contract import (
    ARM_STATE_TOPIC,
    COMMAND_JOINT_NAMES,
    SOURCE_COMMAND_TOPIC,
)
from teleop_core.joint_state import ordered_arm_positions
from teleop_core.protocol import (
    MAX_PACKET_BYTES,
    JointPacket,
    decode_packet,
    encode_packet,
)
from teleop_interfaces.msg import ArmCommand


class PicoTeleopBridge(Node):
    def __init__(self):
        super().__init__("pico_teleop_bridge")
        listen_host = str(self._required_parameter("listen_host"))
        command_port = int(self._required_parameter("command_port"))
        feedback_host = str(self._required_parameter("feedback_host"))
        feedback_port = int(self._required_parameter("feedback_port"))
        self.state_timeout = float(self._required_parameter("state_timeout"))
        if self.state_timeout <= 0:
            raise ValueError("State timeout must be positive.")

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((listen_host, command_port))
        self.socket.setblocking(False)
        self.feedback_address = (feedback_host, feedback_port)
        self.state = {"left": None, "right": None}
        self.state_at = {"left": None, "right": None}
        self.state_stream_id = uuid.uuid4().hex
        self.state_sequence = 0
        self.command_stream_id = None
        self.command_sequence = -1
        self.rejected = 0

        for side in ("left", "right"):
            self.create_subscription(
                JointState,
                ARM_STATE_TOPIC.format(side=side),
                lambda message, selected=side: self._state(selected, message),
                qos_profile_sensor_data,
            )
        self.publisher = self.create_publisher(
            ArmCommand, SOURCE_COMMAND_TOPIC, 10
        )
        self.create_timer(0.01, self._tick)
        self.get_logger().info(
            f"PICO adapter listening on udp://{listen_host}:{command_port}."
        )

    def _required_parameter(self, name):
        parameter = self.declare_parameter(name)
        if parameter.type_ == Parameter.Type.NOT_SET:
            raise ValueError(f"Required parameter '{name}' is missing")
        return parameter.value

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
            self.get_logger().warn(f"Rejected PICO packet: {reason}")

    def _send_state(self, measured, now):
        packet = JointPacket(
            kind="state",
            stream_id=self.state_stream_id,
            sequence=self.state_sequence,
            timestamp=now,
            active_sides=("left", "right"),
            names=COMMAND_JOINT_NAMES,
            positions=tuple(float(value) for value in measured),
        )
        self.socket.sendto(encode_packet(packet), self.feedback_address)
        self.state_sequence += 1

    def _receive_latest(self):
        latest = None
        while True:
            try:
                payload, _ = self.socket.recvfrom(MAX_PACKET_BYTES + 1)
            except BlockingIOError:
                break
            try:
                packet = decode_packet(payload)
                if packet.kind != "command":
                    continue
                if packet.stream_id != self.command_stream_id:
                    self.command_stream_id = packet.stream_id
                    self.command_sequence = -1
                if packet.sequence <= self.command_sequence:
                    continue
                self.command_sequence = packet.sequence
                latest = packet
            except ValueError as exc:
                self._reject(str(exc))
        return latest

    def _tick(self):
        now = time.monotonic()
        measured = self._measured(now)
        if measured is not None:
            self._send_state(measured, now)
        packet = self._receive_latest()
        if packet is None:
            return
        output = ArmCommand()
        output.header.stamp = self.get_clock().now().to_msg()
        output.source = "pico"
        output.session_id = packet.stream_id
        output.sequence = packet.sequence
        output.active_sides = list(packet.active_sides)
        output.joint_names = list(packet.names)
        output.positions = list(packet.positions)
        self.publisher.publish(output)

    def destroy_node(self):
        self.socket.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PicoTeleopBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
