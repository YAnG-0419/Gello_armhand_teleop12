import math
import os
import termios
import time
import tty

import numpy as np
import pinocchio as pin

from .config import InputConfig
from .pose_mapping import is_valid_xr_pose, xr_pose_to_world
from .types import Pose, SIDES, TeleopSample


class ControllerInput:
    def __init__(
        self,
        grip_threshold: float,
        ready_timeout: float,
        stale_timeout: float,
    ) -> None:
        import xrobotoolkit_sdk as xrt

        if not 0 < grip_threshold <= 1:
            raise ValueError("Controller grip threshold must be in (0, 1]")
        if ready_timeout <= 0 or stale_timeout <= 0:
            raise ValueError("Controller ready and stale timeouts must be positive")
        self.xrt = xrt
        self.grip_threshold = float(grip_threshold)
        self.stale_timeout = float(stale_timeout)
        self.blocked = {side: True for side in SIDES}
        self.last_timestamp: int | None = None
        self.last_update_at: float | None = None
        try:
            self.xrt.init()
            self._wait_until_ready(float(ready_timeout))
        except BaseException:
            self.close()
            raise

    def _snapshot(self):
        for _ in range(3):
            timestamp_before = int(self.xrt.get_time_stamp_ns())
            poses = {
                "left": np.asarray(self.xrt.get_left_controller_pose(), dtype=float),
                "right": np.asarray(
                    self.xrt.get_right_controller_pose(), dtype=float
                ),
            }
            grips = {
                "left": float(self.xrt.get_left_grip()),
                "right": float(self.xrt.get_right_grip()),
            }
            timestamp_after = int(self.xrt.get_time_stamp_ns())
            if timestamp_before > 0 and timestamp_before == timestamp_after:
                break
        else:
            return None
        if not all(is_valid_xr_pose(pose) for pose in poses.values()):
            return None
        if any(not np.isfinite(grip) or not 0 <= grip <= 1 for grip in grips.values()):
            return None
        return timestamp_after, poses, grips

    def _wait_until_ready(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            snapshot = self._snapshot()
            if snapshot is not None:
                timestamp, _, _ = snapshot
                self.last_timestamp = timestamp
                self.last_update_at = time.monotonic()
                return
            time.sleep(0.05)
        raise TimeoutError("Timed out waiting for valid PICO controller data")

    def sample(self) -> TeleopSample | None:
        snapshot = self._snapshot()
        now = time.monotonic()
        if snapshot is not None:
            timestamp, raw_poses, grips = snapshot
            if timestamp != self.last_timestamp:
                self.last_timestamp = timestamp
                self.last_update_at = now
        if (
            snapshot is None
            or self.last_update_at is None
            or now - self.last_update_at > self.stale_timeout
        ):
            self.disable_all("controller data missing or stale")
            return None
        _, raw_poses, grips = snapshot
        activations = {}
        for side in SIDES:
            if grips[side] < self.grip_threshold:
                self.blocked[side] = False
            activations[side] = (
                not self.blocked[side] and grips[side] >= self.grip_threshold
            )
        return TeleopSample(
            poses={
                side: xr_pose_to_world(raw_poses[side])
                for side in SIDES
            },
            activations=activations,
            timestamp=now,
        )

    def disable_all(self, _reason: str) -> None:
        self.blocked = {side: True for side in SIDES}

    def close(self) -> None:
        if getattr(self, "xrt", None) is not None:
            xrt = self.xrt
            self.xrt = None
            xrt.close()


class KeyboardActivation:
    def __init__(self, device: str) -> None:
        self.device = device
        self.active = {side: False for side in SIDES}
        self.fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        if not os.isatty(self.fd):
            os.close(self.fd)
            raise ValueError(f"Keyboard activation device is not a TTY: {device}")
        self.saved_attributes = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        self._show(
            "Keyboard: [space] toggle both, [l]/[r] toggle one arm, "
            "[x] disable all, [q] quit"
        )
        self._show_state()

    def _show(self, message: str) -> None:
        os.write(self.fd, f"\r{message}\n".encode())

    def _show_state(self) -> None:
        left = "ON" if self.active["left"] else "off"
        right = "ON" if self.active["right"] else "off"
        self._show(f"Teleop activation: left={left} right={right}")

    def poll(self) -> dict[str, bool]:
        changed = False
        while True:
            try:
                keys = os.read(self.fd, 64)
            except BlockingIOError:
                break
            if not keys:
                break
            for key in keys.decode(errors="ignore").lower():
                if key == " ":
                    activate = not any(self.active.values())
                    self.active = {side: activate for side in SIDES}
                    changed = True
                elif key in ("l", "r"):
                    side = "left" if key == "l" else "right"
                    self.active[side] = not self.active[side]
                    changed = True
                elif key == "x":
                    self.active = {side: False for side in SIDES}
                    changed = True
                elif key == "q":
                    self.active = {side: False for side in SIDES}
                    self._show_state()
                    raise KeyboardInterrupt
        if changed:
            self._show_state()
        return dict(self.active)

    def disable_all(self, reason: str) -> None:
        if any(self.active.values()):
            self.active = {side: False for side in SIDES}
            self._show(f"Teleop disabled: {reason}")
            self._show_state()

    def close(self) -> None:
        if self.fd is None:
            return
        self.active = {side: False for side in SIDES}
        fd = self.fd
        self.fd = None
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, self.saved_attributes)
        finally:
            os.close(fd)


def _local_transform(raw: dict, label: str) -> Pose:
    if not isinstance(raw, dict) or set(raw) != {
        "translation_xyz",
        "quaternion_xyzw",
    }:
        raise ValueError(
            f"{label} must contain exactly translation_xyz and quaternion_xyzw"
        )
    translation = np.asarray(raw["translation_xyz"], dtype=float)
    quaternion = np.asarray(raw["quaternion_xyzw"], dtype=float)
    if translation.shape != (3,) or quaternion.shape != (4,):
        raise ValueError(f"{label} transform dimensions must be 3 and 4")
    if not np.all(np.isfinite(translation)) or not np.all(np.isfinite(quaternion)):
        raise ValueError(f"{label} transform must be finite")
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-8:
        raise ValueError(f"{label} quaternion must be non-zero")
    quaternion /= norm
    rotation = pin.Quaternion(
        quaternion[3], quaternion[0], quaternion[1], quaternion[2]
    ).toRotationMatrix()
    return Pose(translation, rotation)


def _apply_local_transform(pose: Pose, transform: Pose) -> Pose:
    return Pose(
        pose.position + pose.rotation @ transform.position,
        pose.rotation @ transform.rotation,
    )


class MotionTrackerInput:
    def __init__(
        self,
        serials: dict[str, str],
        tracker_to_control: dict[str, dict],
        ready_timeout: float,
        stale_timeout: float,
        frozen_timeout: float,
        max_position_jump: float,
        max_rotation_jump: float,
        max_linear_speed: float,
        max_angular_speed: float,
        keyboard_device: str,
    ) -> None:
        import xrobotoolkit_sdk as xrt

        if set(serials) != set(SIDES) or any(not value for value in serials.values()):
            raise ValueError("Both motion tracker serial numbers are required")
        if serials["left"] == serials["right"]:
            raise ValueError("Left and right motion tracker serials must differ")
        if set(tracker_to_control) != set(SIDES):
            raise ValueError("Both tracker-to-control transforms are required")
        limits = (
            ready_timeout,
            stale_timeout,
            frozen_timeout,
            max_position_jump,
            max_rotation_jump,
            max_linear_speed,
            max_angular_speed,
        )
        if any(not np.isfinite(value) or value <= 0 for value in limits):
            raise ValueError("Tracker timeouts and motion limits must be positive")

        self.xrt = xrt
        self.serials = dict(serials)
        self.transforms = {
            side: _local_transform(tracker_to_control[side], side)
            for side in SIDES
        }
        self.stale_timeout = float(stale_timeout)
        self.frozen_timeout = float(frozen_timeout)
        self.max_position_jump = float(max_position_jump)
        self.max_rotation_jump = float(max_rotation_jump)
        self.max_linear_speed = float(max_linear_speed)
        self.max_angular_speed = float(max_angular_speed)
        self.last_motion_timestamp: int | None = None
        self.last_motion_update_at: float | None = None
        self.last_poses: dict[str, Pose | None] = {side: None for side in SIDES}
        self.last_position_changed_at = {side: None for side in SIDES}
        self.last_rotation_changed_at = {side: None for side in SIDES}
        self.last_activations = {side: False for side in SIDES}
        self.keyboard = KeyboardActivation(keyboard_device)
        try:
            self.xrt.init()
            self._wait_until_ready(float(ready_timeout))
        except BaseException:
            self.close()
            raise

    def _snapshot(self):
        for _ in range(3):
            timestamp_before = int(self.xrt.get_motion_timestamp_ns())
            count = int(self.xrt.num_motion_data_available())
            serials = list(self.xrt.get_motion_tracker_serial_numbers())
            poses = list(self.xrt.get_motion_tracker_pose())
            timestamp_after = int(self.xrt.get_motion_timestamp_ns())
            if timestamp_before > 0 and timestamp_before == timestamp_after:
                break
        else:
            return None
        if (
            count != len(serials)
            or count != len(poses)
            or len(set(serials)) != len(serials)
        ):
            return None
        by_serial = {
            serial: np.asarray(pose, dtype=float)
            for serial, pose in zip(serials, poses)
        }
        if any(serial not in by_serial for serial in self.serials.values()):
            return None
        selected = {
            side: by_serial[self.serials[side]]
            for side in SIDES
        }
        if not all(is_valid_xr_pose(pose) for pose in selected.values()):
            return None
        return timestamp_after, selected, serials

    def _wait_until_ready(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        detected: list[str] = []
        while time.monotonic() < deadline:
            self.keyboard.poll()
            snapshot = self._snapshot()
            detected = list(self.xrt.get_motion_tracker_serial_numbers())
            if snapshot is not None:
                timestamp, raw_poses, _ = snapshot
                now = time.monotonic()
                self.last_motion_timestamp = timestamp
                self.last_motion_update_at = now
                self.last_poses = {
                    side: _apply_local_transform(
                        xr_pose_to_world(raw_poses[side]),
                        self.transforms[side],
                    )
                    for side in SIDES
                }
                self.last_position_changed_at = {side: now for side in SIDES}
                self.last_rotation_changed_at = {side: now for side in SIDES}
                self.keyboard.disable_all("motion trackers initialized")
                return
            time.sleep(0.05)
        raise TimeoutError(
            "Timed out waiting for configured motion trackers "
            f"{self.serials}; detected serials={detected}"
        )

    def _motion_fault(
        self,
        poses: dict[str, Pose],
        activations: dict[str, bool],
        timestamp: int,
        now: float,
    ) -> str | None:
        previous_timestamp = self.last_motion_timestamp
        if previous_timestamp is None:
            return None
        if timestamp < previous_timestamp:
            return "motion tracker timestamp moved backwards"
        if timestamp == previous_timestamp:
            return None

        elapsed = (timestamp - previous_timestamp) * 1e-9
        fault = None
        for side in SIDES:
            previous_pose = self.last_poses[side]
            if (
                not activations[side]
                or not self.last_activations[side]
                or previous_pose is None
            ):
                self.last_poses[side] = poses[side]
                self.last_position_changed_at[side] = now
                self.last_rotation_changed_at[side] = now
                continue
            position_delta = float(
                np.linalg.norm(poses[side].position - previous_pose.position)
            )
            rotation_delta = float(
                np.linalg.norm(
                    pin.log3(poses[side].rotation @ previous_pose.rotation.T)
                )
            )
            linear_speed = position_delta / elapsed
            angular_speed = rotation_delta / elapsed
            if position_delta > self.max_position_jump:
                fault = f"{side} tracker position jumped {position_delta:.3f} m"
            elif rotation_delta > self.max_rotation_jump:
                fault = f"{side} tracker rotation jumped {rotation_delta:.3f} rad"
            elif linear_speed > self.max_linear_speed:
                fault = f"{side} tracker linear speed {linear_speed:.3f} m/s"
            elif angular_speed > self.max_angular_speed:
                fault = f"{side} tracker angular speed {angular_speed:.3f} rad/s"

            # Position and rotation freeze independently, because they come
            # from different sensors: position from the headset's optical view
            # of the tracker, rotation from the tracker's own IMU. Losing the
            # optical fix freezes position while the IMU keeps streaming, and a
            # combined liveness check is then blind to it. Measured on a real
            # failure, position updated 37 times in 45 s while rotation moved
            # on 84% of ticks, and a single alive-if-anything-moves clock let
            # the arms track a sub-hertz position stream for half a minute.
            if position_delta > 1e-5:
                self.last_position_changed_at[side] = now
            if rotation_delta > 1e-4:
                self.last_rotation_changed_at[side] = now
            position_changed_at = self.last_position_changed_at[side]
            rotation_changed_at = self.last_rotation_changed_at[side]
            if (
                position_changed_at is None
                or now - position_changed_at > self.frozen_timeout
            ):
                fault = f"{side} tracker position is frozen"
            elif (
                rotation_changed_at is None
                or now - rotation_changed_at > self.frozen_timeout
            ):
                fault = f"{side} tracker rotation is frozen"
            self.last_poses[side] = poses[side]
        return fault

    def sample(self) -> TeleopSample | None:
        activations = self.keyboard.poll()
        snapshot = self._snapshot()
        now = time.monotonic()
        if snapshot is not None:
            timestamp, raw_poses, _ = snapshot
            if timestamp != self.last_motion_timestamp:
                self.last_motion_update_at = now
        if (
            snapshot is None
            or self.last_motion_update_at is None
            or now - self.last_motion_update_at > self.stale_timeout
        ):
            self.disable_all("motion tracker data missing or stale")
            return None
        poses = {
            side: _apply_local_transform(
                xr_pose_to_world(raw_poses[side]),
                self.transforms[side],
            )
            for side in SIDES
        }
        motion_fault = self._motion_fault(poses, activations, timestamp, now)
        self.last_motion_timestamp = timestamp
        if motion_fault is not None:
            self.disable_all(motion_fault)
            return None
        self.last_activations = dict(activations)
        return TeleopSample(poses, activations, now)

    def disable_all(self, reason: str) -> None:
        self.keyboard.disable_all(reason)
        self.last_activations = {side: False for side in SIDES}

    def close(self) -> None:
        try:
            if getattr(self, "xrt", None) is not None:
                xrt = self.xrt
                self.xrt = None
                xrt.close()
        finally:
            if getattr(self, "keyboard", None) is not None:
                keyboard = self.keyboard
                self.keyboard = None
                keyboard.close()


def create_pico_input(config: InputConfig, input_type: str):
    if input_type == "controllers":
        controllers = config.controllers
        return ControllerInput(
            grip_threshold=controllers.grip_threshold,
            ready_timeout=controllers.ready_timeout,
            stale_timeout=controllers.stale_timeout,
        )
    if input_type == "motion-trackers":
        trackers = config.motion_trackers
        if any(
            serial.startswith("REPLACE_WITH_")
            for serial in trackers.serials.values()
        ):
            raise ValueError(
                "Motion tracker serials are not configured; set "
                "input.motion_trackers.serials in config/pico.yaml."
            )
        return MotionTrackerInput(
            serials=trackers.serials,
            tracker_to_control=trackers.tracker_to_control,
            ready_timeout=trackers.ready_timeout,
            stale_timeout=trackers.stale_timeout,
            frozen_timeout=trackers.frozen_timeout,
            max_position_jump=trackers.max_position_jump,
            max_rotation_jump=trackers.max_rotation_jump,
            max_linear_speed=trackers.max_linear_speed,
            max_angular_speed=trackers.max_angular_speed,
            keyboard_device=trackers.keyboard_device,
        )
    raise ValueError(f"Unsupported PICO input type: {input_type}")


class MockTeleopInput:
    def __init__(self) -> None:
        self.started_at = time.monotonic()

    def sample(self) -> TeleopSample:
        elapsed = time.monotonic() - self.started_at
        angle = 0.12 * math.sin(elapsed)
        rotation = pin.exp3(np.array([0.0, angle, 0.0]))
        offset = 0.04 * math.sin(0.8 * elapsed)
        return TeleopSample(
            poses={
                "left": Pose(np.array([-0.2, 0.0, 1.2 + offset]), rotation),
                "right": Pose(np.array([0.2, 0.0, 1.2 + offset]), rotation.T),
            },
            activations={"left": True, "right": True},
            timestamp=time.monotonic(),
        )

    def close(self) -> None:
        return None
