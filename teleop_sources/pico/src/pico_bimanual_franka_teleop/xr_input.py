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
        keyboard_device: str,
    ) -> None:
        import xrobotoolkit_sdk as xrt

        if set(serials) != set(SIDES) or any(not value for value in serials.values()):
            raise ValueError("Both motion tracker serial numbers are required")
        if serials["left"] == serials["right"]:
            raise ValueError("Left and right motion tracker serials must differ")
        if set(tracker_to_control) != set(SIDES):
            raise ValueError("Both tracker-to-control transforms are required")
        if ready_timeout <= 0 or stale_timeout <= 0:
            raise ValueError("Tracker ready and stale timeouts must be positive")

        self.xrt = xrt
        self.serials = dict(serials)
        self.transforms = {
            side: _local_transform(tracker_to_control[side], side)
            for side in SIDES
        }
        self.stale_timeout = float(stale_timeout)
        self.last_motion_timestamp: int | None = None
        self.last_motion_update_at: float | None = None
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
            serials = list(self.xrt.get_motion_tracker_serial_numbers())
            poses = list(self.xrt.get_motion_tracker_pose())
            timestamp_after = int(self.xrt.get_motion_timestamp_ns())
            if timestamp_before > 0 and timestamp_before == timestamp_after:
                break
        else:
            return None
        if len(serials) != len(poses) or len(set(serials)) != len(serials):
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
                timestamp, _, _ = snapshot
                self.last_motion_timestamp = timestamp
                self.last_motion_update_at = time.monotonic()
                self.keyboard.disable_all("motion trackers initialized")
                return
            time.sleep(0.05)
        raise TimeoutError(
            "Timed out waiting for configured motion trackers "
            f"{self.serials}; detected serials={detected}"
        )

    def sample(self) -> TeleopSample | None:
        activations = self.keyboard.poll()
        snapshot = self._snapshot()
        now = time.monotonic()
        if snapshot is not None:
            timestamp, raw_poses, _ = snapshot
            if timestamp != self.last_motion_timestamp:
                self.last_motion_timestamp = timestamp
                self.last_motion_update_at = now
        if (
            snapshot is None
            or self.last_motion_update_at is None
            or now - self.last_motion_update_at > self.stale_timeout
        ):
            self.keyboard.disable_all("motion tracker data missing or stale")
            return None
        _, raw_poses, _ = snapshot
        poses = {
            side: _apply_local_transform(
                xr_pose_to_world(raw_poses[side]),
                self.transforms[side],
            )
            for side in SIDES
        }
        return TeleopSample(poses, activations, now)

    def disable_all(self, reason: str) -> None:
        self.keyboard.disable_all(reason)

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


def create_pico_input(config: InputConfig):
    if config.type == "controllers":
        controllers = config.controllers
        return ControllerInput(
            grip_threshold=controllers.grip_threshold,
            ready_timeout=controllers.ready_timeout,
            stale_timeout=controllers.stale_timeout,
        )
    if config.type == "motion_trackers":
        trackers = config.motion_trackers
        return MotionTrackerInput(
            serials=trackers.serials,
            tracker_to_control=trackers.tracker_to_control,
            ready_timeout=trackers.ready_timeout,
            stale_timeout=trackers.stale_timeout,
            keyboard_device=trackers.keyboard_device,
        )
    raise ValueError(f"Unsupported PICO input type: {config.type}")


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
