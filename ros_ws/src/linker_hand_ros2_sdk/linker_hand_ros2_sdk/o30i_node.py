"""Safety-conscious ROS 2 driver for a right Linker Hand O30i."""

from __future__ import annotations

import math
from pathlib import Path
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .LinkerHand import o30i_control
from .o30i_contract import (
    O30I_DRIVER_JOINT_NAMES,
    O30I_DRIVER_TO_URDF,
    O30I_RIGHT_LOWER,
    O30I_RIGHT_UPPER,
    O30I_URDF_JOINT_NAMES,
    radians_to_ticks,
    ticks_to_radians,
)
from .o30i_transport import (
    bundled_libcanbus_path,
    make_libcanbus_communication,
)


class O30IDriver(Node):
    """Translate canonical URDF radians to the O30i HOP tick protocol."""

    def __init__(self) -> None:
        super().__init__("linker_hand_o30i")
        self.declare_parameter("hand_type", "right")
        self.declare_parameter("transport", "libcanbus")
        self.declare_parameter("can", "can1")
        self.declare_parameter("frame_id", 1)
        self.declare_parameter("canfd_device", 0)
        self.declare_parameter("canfd_channel", 0)
        self.declare_parameter("libcanbus_path", "")
        self.declare_parameter("bitrate", 1_000_000)
        self.declare_parameter("data_bitrate", 5_000_000)
        self.declare_parameter("auto_setup_can", False)
        self.declare_parameter("output_enabled", False)
        self.declare_parameter("calibration_verified", False)
        self.declare_parameter("tick_at_lower", [0.0] * 20)
        self.declare_parameter("tick_at_upper", [255.0] * 20)
        self.declare_parameter("command_timeout", 0.25)
        self.declare_parameter("state_timeout", 0.5)
        self.declare_parameter("state_publish_rate", 30.0)

        side = str(self.get_parameter("hand_type").value).strip().lower()
        if side != "right":
            raise ValueError("the checked-in O30i contract currently supports right only")
        transport = str(self.get_parameter("transport").value).strip().lower()
        if transport not in {"socketcan", "libcanbus"}:
            raise ValueError("transport must be socketcan or libcanbus")
        self.output_enabled = bool(self.get_parameter("output_enabled").value)
        self.calibration_verified = bool(
            self.get_parameter("calibration_verified").value
        )
        if self.output_enabled and not self.calibration_verified:
            raise ValueError(
                "refusing real O30i control until calibration_verified:=true"
            )
        self.tick_at_lower = self._calibration_vector("tick_at_lower")
        self.tick_at_upper = self._calibration_vector("tick_at_upper")
        if any(
            lower == upper
            for lower, upper in zip(
                self.tick_at_lower, self.tick_at_upper, strict=True
            )
        ):
            raise ValueError("O30i calibration endpoints must differ for every joint")
        timeout = float(self.get_parameter("command_timeout").value)
        state_timeout = float(self.get_parameter("state_timeout").value)
        publish_rate = float(self.get_parameter("state_publish_rate").value)
        if (
            not math.isfinite(timeout)
            or timeout <= 0.0
            or not math.isfinite(state_timeout)
            or state_timeout <= 0.0
            or not math.isfinite(publish_rate)
            or not 0.0 < publish_rate <= 100.0
        ):
            raise ValueError(
                "timeouts must be positive and state_publish_rate in (0, 100]"
            )
        self.command_timeout = timeout
        self.state_timeout = state_timeout
        self.last_command_at: float | None = None
        self.last_state_at: float | None = None
        self.motors_enabled = False
        self.stopped = False

        channel: str | int
        if transport == "libcanbus":
            configured_library = str(
                self.get_parameter("libcanbus_path").value
            ).strip()
            library_path = (
                Path(configured_library).expanduser().resolve()
                if configured_library
                else bundled_libcanbus_path()
            )
            if not library_path.is_file():
                raise FileNotFoundError(
                    f"O30i libcanbus runtime not found: {library_path}"
                )
            o30i_control.CANFDCommunication = make_libcanbus_communication(
                o30i_control,
                library_path,
            )
            channel = int(self.get_parameter("canfd_channel").value)
        else:
            channel = str(self.get_parameter("can").value)
        self.controller = o30i_control.LinkerHandO30IController(
            hand_type=side,
            canfd_device=int(self.get_parameter("canfd_device").value),
            frame_id=int(self.get_parameter("frame_id").value),
            comm_type=transport,
            channel=channel,
            bitrate=int(self.get_parameter("bitrate").value),
            dbitrate=int(self.get_parameter("data_bitrate").value),
            auto_setup=bool(self.get_parameter("auto_setup_can").value),
        )
        try:
            if not self.controller.is_connected:
                raise RuntimeError("O30i SocketCAN transport did not connect")
            model = str(self.controller.get_product_model() or "").upper()
            device_uid = str(self.controller.get_device_uid() or "")
            protocol_version = str(
                self.controller.get_protocol_version() or ""
            )
            reported_side = str(self.controller.get_hand_side() or "").upper()
            if "O30" not in model:
                raise RuntimeError(f"connected device is not O30i: {model!r}")
            if reported_side != "RIGHT":
                raise RuntimeError(
                    f"connected O30i reports {reported_side}, expected RIGHT"
                )
        except BaseException:
            self.controller.close()
            raise
        self.get_logger().info(
            f"Verified {model} {reported_side}; uid={device_uid!r}; "
            f"protocol={protocol_version!r}; transport={transport}"
        )

        self.command_subscription = (
            self.create_subscription(
                JointState,
                "/cb_right_hand_control_cmd",
                self._command,
                10,
            )
            if self.output_enabled
            else None
        )
        self.state_publisher = self.create_publisher(
            JointState, "/cb_right_hand_state", 10
        )
        self.raw_state_publisher = self.create_publisher(
            JointState, "/linker_hand_o30i/raw_state", 10
        )
        self.state_timer = self.create_timer(1.0 / publish_rate, self._publish_state)
        self.watchdog_timer = self.create_timer(
            min(0.05, self.command_timeout / 2.0), self._watchdog
        )
        if self.output_enabled:
            self.get_logger().warn(
                "O30i transport connected; motors remain disabled until the "
                "first complete, finite, in-limit radians command and fresh "
                "calibrated position feedback"
            )
        else:
            self.get_logger().info(
                "O30i transport connected in READ-ONLY mode; no command "
                "subscription exists and motors will not be enabled"
            )

    def _calibration_vector(self, parameter: str) -> tuple[float, ...]:
        values = tuple(float(value) for value in self.get_parameter(parameter).value)
        if len(values) != 20 or not all(
            math.isfinite(value) and 0.0 <= value <= 255.0 for value in values
        ):
            raise ValueError(f"{parameter} must contain 20 finite values in [0, 255]")
        return values

    def _ordered_radians(self, message: JointState) -> tuple[float, ...]:
        if len(message.name) != 20 or len(message.position) != 20:
            raise ValueError("O30i commands require all 20 named URDF joints")
        if len(set(message.name)) != 20:
            raise ValueError("O30i command contains duplicate joint names")
        by_name = dict(zip(message.name, message.position, strict=True))
        if set(by_name) != set(O30I_URDF_JOINT_NAMES):
            missing = sorted(set(O30I_URDF_JOINT_NAMES) - set(by_name))
            unknown = sorted(set(by_name) - set(O30I_URDF_JOINT_NAMES))
            raise ValueError(
                f"O30i command joint contract differs: missing={missing}, "
                f"unknown={unknown}"
            )
        values = tuple(float(by_name[name]) for name in O30I_URDF_JOINT_NAMES)
        violations = [
            name
            for name, value, lower, upper in zip(
                O30I_URDF_JOINT_NAMES,
                values,
                O30I_RIGHT_LOWER,
                O30I_RIGHT_UPPER,
                strict=True,
            )
            if not math.isfinite(value) or value < lower or value > upper
        ]
        if violations:
            raise ValueError("O30i radians limits exceeded: " + ", ".join(violations))
        return values

    def _to_driver_ticks(self, radians: tuple[float, ...]) -> list[int]:
        by_name = dict(zip(O30I_URDF_JOINT_NAMES, radians, strict=True))
        calibration = {
            name: (lower, upper, tick_lower, tick_upper)
            for name, lower, upper, tick_lower, tick_upper in zip(
                O30I_URDF_JOINT_NAMES,
                O30I_RIGHT_LOWER,
                O30I_RIGHT_UPPER,
                self.tick_at_lower,
                self.tick_at_upper,
                strict=True,
            )
        }
        return [
            radians_to_ticks(by_name[urdf_name], *calibration[urdf_name])
            for driver_name in O30I_DRIVER_JOINT_NAMES
            for urdf_name in (O30I_DRIVER_TO_URDF[driver_name],)
        ]

    def _command(self, message: JointState) -> None:
        if self.stopped:
            self.get_logger().error(
                "rejecting O30i command after terminal watchdog stop; restart node"
            )
            return
        try:
            radians = self._ordered_radians(message)
            ticks = self._to_driver_ticks(radians)
            now = time.monotonic()
            if (
                self.last_state_at is None
                or now - self.last_state_at > self.state_timeout
            ):
                raise RuntimeError(
                    "no fresh, complete, in-calibration O30i position feedback"
                )
            if not self.motors_enabled:
                if not self.controller.setup():
                    raise RuntimeError("O30i position-mode/enable setup was rejected")
                self.motors_enabled = True
            if not self.controller.set_target_position(ticks):
                raise RuntimeError("O30i position command was rejected")
            self.last_command_at = now
        except Exception as error:  # noqa: BLE001 - reject malformed hardware input
            if self.motors_enabled:
                self._disable_terminal(
                    f"O30i command failed after enable: {error}"
                )
                return
            self.get_logger().warning(f"rejecting O30i command: {error}")

    def _publish_state(self) -> None:
        try:
            ticks = self.controller.get_current_position()
            if ticks is None or len(ticks) != 20:
                raise RuntimeError("O30i returned no complete position state")
            raw_message = JointState()
            raw_message.header.stamp = self.get_clock().now().to_msg()
            raw_message.name = list(O30I_DRIVER_JOINT_NAMES)
            raw_message.position = [float(value) for value in ticks]
            self.raw_state_publisher.publish(raw_message)
            if not self.calibration_verified:
                return
            driver_ticks = dict(zip(O30I_DRIVER_JOINT_NAMES, ticks, strict=True))
            radians_by_name = {}
            for index, name in enumerate(O30I_URDF_JOINT_NAMES):
                driver_name = next(
                    candidate
                    for candidate, urdf_name in O30I_DRIVER_TO_URDF.items()
                    if urdf_name == name
                )
                tick = float(driver_ticks[driver_name])
                tick_min = min(
                    self.tick_at_lower[index], self.tick_at_upper[index]
                )
                tick_max = max(
                    self.tick_at_lower[index], self.tick_at_upper[index]
                )
                if not math.isfinite(tick) or not tick_min <= tick <= tick_max:
                    raise ValueError(
                        f"{driver_name} feedback tick {tick} is outside its "
                        f"calibrated interval [{tick_min}, {tick_max}]"
                    )
                radians_by_name[name] = ticks_to_radians(
                    tick,
                    O30I_RIGHT_LOWER[index],
                    O30I_RIGHT_UPPER[index],
                    self.tick_at_lower[index],
                    self.tick_at_upper[index],
                )
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(O30I_URDF_JOINT_NAMES)
            message.position = [
                radians_by_name[name] for name in O30I_URDF_JOINT_NAMES
            ]
            self.state_publisher.publish(message)
            self.last_state_at = time.monotonic()
        except Exception as error:  # noqa: BLE001 - keep driver alive for diagnostics
            self.get_logger().warning(
                f"O30i state read failed: {error}",
                throttle_duration_sec=2.0,
            )

    def _watchdog(self) -> None:
        if not self.motors_enabled:
            return
        now = time.monotonic()
        command_stale = (
            self.last_command_at is None
            or now - self.last_command_at > self.command_timeout
        )
        state_stale = (
            self.last_state_at is None
            or now - self.last_state_at > self.state_timeout
        )
        if not command_stale and not state_stale:
            return
        reason = (
            "command watchdog expired"
            if command_stale
            else "position feedback became stale"
        )
        self._disable_terminal(f"O30i {reason}")

    def _disable_terminal(self, reason: str) -> None:
        accepted = self.controller.set_joint_enable([0] * 20)
        self.motors_enabled = False
        self.stopped = True
        message = f"{reason}; all joints disabled and node requires restart"
        if accepted is False:
            message += " (disable command was rejected)"
        self.get_logger().error(message)

    def destroy_node(self) -> bool:
        try:
            if self.motors_enabled:
                self.controller.set_joint_enable([0] * 20)
        finally:
            self.controller.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: O30IDriver | None = None
    try:
        node = O30IDriver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
