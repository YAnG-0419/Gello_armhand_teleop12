"""ROS 2 node bridging hand qpos datagrams to LinkerHand G20 commands.

Ported from the sibling WiLoR repository's bridge node. Safety behaviour is
deliberately preserved: output is disabled unless explicitly enabled at startup,
a fresh valid packet is required before anything is published, motion is
slew-limited, and there is no automatic return-home on watchdog expiry.

This node is the only place hand commands reach the vendor driver. It is
separate from the arm safety gateway on purpose: the arm gateway owns the FR3
command bus and must not be coupled to optical hand tracking, whose loss is an
independent event from losing a wrist tracker.
"""

from __future__ import annotations

import math
import socket
import time
from dataclasses import dataclass

import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .core import (
    COMMAND_SLOTS,
    G20_JOINT_NAMES,
    CommandLimiter,
    G20Mapper,
    decode_qpos_packet,
    validate_hand_state,
)

VENDOR_MAX_PUBLISH_RATE = 30.0


@dataclass
class SideState:
    """Runtime state tracked independently for one hand."""

    limiter: CommandLimiter
    target: tuple[float, ...] | None = None
    received_at: float | None = None
    stream_id: str | None = None
    sequence: int = -1
    was_fresh: bool = False
    feedback: tuple[float, ...] | None = None
    received_count: int = 0
    published_count: int = 0
    rejected_feedback: int = 0


class LinkerHandBridge(Node):
    """Receive retargeted hand poses and optionally publish G20 commands."""

    def __init__(self) -> None:
        super().__init__("linker_hand_bridge")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 5570)
        self.declare_parameter("sides", "both")
        self.declare_parameter("enabled", False)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("watchdog_timeout", 0.25)
        self.declare_parameter("max_command_rate", 200.0)
        self.declare_parameter("log_period", 2.0)
        self.declare_parameter("initial_speed", 30)
        self.declare_parameter("abduction_invert", False)

        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        sides_value = str(self.get_parameter("sides").value).lower()
        # Read once at construction. Enabling hardware output must not be
        # possible through a live parameter update.
        self.enabled = bool(self.get_parameter("enabled").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.watchdog_timeout = float(self.get_parameter("watchdog_timeout").value)
        max_command_rate = float(self.get_parameter("max_command_rate").value)
        self.log_period = float(self.get_parameter("log_period").value)
        self.initial_speed = int(self.get_parameter("initial_speed").value)
        abduction_invert = bool(self.get_parameter("abduction_invert").value)
        if not 0 <= self.initial_speed <= 255:
            raise ValueError("initial_speed must be in [0, 255]; 0 disables")

        if sides_value == "both":
            self.sides = ("left", "right")
        elif sides_value in {"left", "right"}:
            self.sides = (sides_value,)
        else:
            raise ValueError("sides must be left, right, or both")
        if (
            publish_rate <= 0.0
            or self.watchdog_timeout <= 0.0
            or self.log_period <= 0.0
            or max_command_rate <= 0.0
        ):
            raise ValueError(
                "publish_rate, watchdog_timeout, log_period and max_command_rate "
                "must be positive"
            )
        if publish_rate > VENDOR_MAX_PUBLISH_RATE:
            raise ValueError(
                "publish_rate must not exceed the vendor driver's "
                f"{VENDOR_MAX_PUBLISH_RATE:.0f} Hz limit; above roughly 100 Hz it "
                "silently drops commands"
            )

        self.mapper = G20Mapper(abduction_invert=abduction_invert)
        self.state = {
            side: SideState(CommandLimiter(self.mapper.home(side), max_command_rate))
            for side in self.sides
        }
        self.command_publishers = {
            side: self.create_publisher(JointState, f"/cb_{side}_hand_control_cmd", 10)
            for side in self.sides
        }
        self.debug_publishers = {
            side: self.create_publisher(
                JointState, f"/linker_hand_bridge/{side}/mapped_command", 10
            )
            for side in self.sides
        }
        self.feedback_subscriptions = [
            self.create_subscription(
                JointState,
                f"/cb_{side}_hand_state",
                lambda message, selected_side=side: self._feedback_callback(
                    selected_side, message
                ),
                10,
            )
            for side in self.sides
        ]

        # The vendor driver never initializes speed or torque for G20, because
        # its startup-pose code has no G20 branch. Without this the hand would
        # execute the first position command at whatever the firmware last held,
        # possibly 255. Sending it from here means it cannot be forgotten.
        self.setting_publisher = self.create_publisher(String, "/cb_hand_setting_cmd", 10)
        self.speed_timer = None
        if self.initial_speed > 0:
            self.speed_timer = self.create_timer(2.0, self._send_initial_speed)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.socket.bind((host, port))
        self.timer = self.create_timer(1.0 / publish_rate, self._tick)
        self.last_log_time = time.monotonic()
        self.invalid_count = 0
        self.out_of_order_count = 0

        mode = "ENABLED" if self.enabled else "DRY-RUN (no vendor commands published)"
        self.get_logger().info(
            f"Listening for hand qpos on udp://{host}:{port}; "
            f"sides={self.sides}; rate={publish_rate:g}Hz; "
            f"slew={max_command_rate:g} units/s; "
            f"abduction_invert={abduction_invert}; mode={mode}"
        )
        if self.enabled:
            self.get_logger().warn(
                "Hardware output is enabled. Commands begin only after a fresh "
                "valid packet and are slew-limited. There is no automatic "
                "return-home on watchdog expiry."
            )

    def _send_initial_speed(self) -> None:
        """Set a conservative joint speed on each side, once, then stop.

        Fires on a delay so the vendor driver has had time to subscribe. The
        driver's own console reports the value it applied.

        Note that `/cb_hand_setting_cmd` is a single global topic and the vendor
        driver applies every message to its own hand regardless of the
        `hand_type` field: both branches of its handler assign the same API
        object. With two drivers running, each therefore applies both messages.
        That is harmless while both sides get the same speed, but per-side speeds
        do not work, and the last message received would win.
        """
        if self.speed_timer is not None:
            self.speed_timer.cancel()
            self.speed_timer = None
        for side in self.sides:
            message = String()
            message.data = json.dumps(
                {
                    "setting_cmd": "set_speed",
                    "params": {
                        "hand_type": side,
                        "speed": [self.initial_speed] * 5,
                    },
                }
            )
            self.setting_publisher.publish(message)
        self.get_logger().info(
            f"Requested joint speed {self.initial_speed}/255 on {list(self.sides)}"
        )

    def _feedback_callback(self, side: str, message: JointState) -> None:
        """Record measured hand state, rejecting sentinels and partial messages.

        The vendor driver publishes a 10-value initializer before its first
        hardware poll and uses -1 as a no-data sentinel. Accepting either would
        seed the slew limiter from a fictitious pose; -1 clamped to 0 would look
        like a fully flexed hand, so the limiter would then slew away from a
        closed fist that the hand was never in.
        """
        validated = validate_hand_state(message.position)
        if validated is None:
            side_state = self.state[side]
            side_state.rejected_feedback += 1
            if side_state.rejected_feedback <= 2:
                self.get_logger().info(
                    f"Ignoring unusable {side} hand state with "
                    f"{len(message.position)} values; waiting for measured data"
                )
            return
        self.state[side].feedback = validated

    def _receive_available(self, now: float) -> None:
        while True:
            try:
                payload, _ = self.socket.recvfrom(16_385)
            except BlockingIOError:
                return
            try:
                packet = decode_qpos_packet(payload)
                if packet.side not in self.state:
                    continue
                side_state = self.state[packet.side]
                if (
                    packet.stream_id == side_state.stream_id
                    and packet.sequence <= side_state.sequence
                ):
                    self.out_of_order_count += 1
                    continue
                target = self.mapper.map_packet(packet)
            except (TypeError, ValueError) as error:
                self.invalid_count += 1
                if self.invalid_count <= 3:
                    self.get_logger().warn(f"Ignoring invalid UDP packet: {error}")
                continue
            side_state.target = target
            side_state.received_at = now
            side_state.stream_id = packet.stream_id
            side_state.sequence = packet.sequence
            side_state.received_count += 1

    def _tick(self) -> None:
        now = time.monotonic()
        self._receive_available(now)
        for side, side_state in self.state.items():
            fresh = (
                side_state.target is not None
                and side_state.received_at is not None
                and now - side_state.received_at <= self.watchdog_timeout
            )
            if not fresh:
                # Stop publishing and require re-acquisition. Deliberately no
                # automatic motion back to a home pose.
                side_state.was_fresh = False
                continue
            if not side_state.was_fresh:
                # Re-anchor on measured state where available so reacquisition
                # cannot produce a command jump.
                start = (
                    side_state.feedback
                    if side_state.feedback is not None
                    else self.mapper.home(side)
                )
                side_state.limiter.reset(start, now)
                side_state.was_fresh = True
            command = side_state.limiter.step(side_state.target, now)
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(G20_JOINT_NAMES)
            message.position = list(command)
            self.debug_publishers[side].publish(message)
            if not self.enabled:
                continue
            self.command_publishers[side].publish(message)
            side_state.published_count += 1

        if now - self.last_log_time >= self.log_period:
            elapsed = now - self.last_log_time
            summaries = []
            for side, side_state in self.state.items():
                age = (
                    math.inf
                    if side_state.received_at is None
                    else now - side_state.received_at
                )
                status = "fresh" if age <= self.watchdog_timeout else "stale"
                feedback = "measured" if side_state.feedback is not None else "no-state"
                summaries.append(
                    f"{side}: rx={side_state.received_count / elapsed:.1f}Hz "
                    f"pub={side_state.published_count / elapsed:.1f}Hz "
                    f"{status} {feedback}"
                )
                side_state.received_count = 0
                side_state.published_count = 0
            self.get_logger().info(
                "; ".join(summaries)
                + f"; invalid={self.invalid_count} "
                f"out_of_order={self.out_of_order_count}"
            )
            self.last_log_time = now

    def destroy_node(self) -> bool:
        self.socket.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LinkerHandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
