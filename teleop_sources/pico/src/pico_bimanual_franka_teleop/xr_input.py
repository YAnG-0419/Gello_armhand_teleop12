import math
import os
import termios
import time
import tty
from pathlib import Path

import numpy as np
import pinocchio as pin

from .config import InputConfig
from .hand_input import SkeletonLiveness
from .hand_landmarks import OPENXR_WRIST
from .pose_mapping import is_valid_xr_pose, xr_pose_to_world
from .types import Pose, SIDES, TeleopSample


def desktop_gui_pids() -> list[int]:
    """PIDs of the running XRoboToolkit desktop GUI, if any."""
    pids = []
    for process in Path("/proc").iterdir():
        if not process.name.isdigit():
            continue
        try:
            command = (process / "cmdline").read_bytes().replace(b"\0", b" ")
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if b"RobotLinuxDemo.x86_64" in command:
            pids.append(int(process.name))
    return sorted(pids)


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
    def __init__(self, device: str, sides: tuple[str, ...] = SIDES) -> None:
        if not sides or set(sides).difference(SIDES):
            raise ValueError(f"Invalid keyboard sides: {sides}")
        self.device = device
        self.sides = tuple(sides)
        self.active = {side: False for side in SIDES}
        self.requests = {"open_hands": False, "reset": False}
        self.fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        if not os.isatty(self.fd):
            os.close(self.fd)
            raise ValueError(f"Keyboard activation device is not a TTY: {device}")
        self.saved_attributes = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        self._show(
            "Keyboard: [space] toggle configured sides, [l]/[r] toggle one side, "
            "[x] disable all, [o] open hands, [h] reset to initial pose, "
            "[q] quit"
        )
        self._show_state()

    def _show(self, message: str) -> None:
        os.write(self.fd, f"\r{message}\n".encode())

    def _show_state(self) -> None:
        def state(side: str) -> str:
            if side not in self.sides:
                return "not-configured"
            return "ON" if self.active[side] else "off"

        self._show(
            f"Teleop activation: left={state('left')} right={state('right')}"
        )

    def show(self, message: str) -> None:
        self._show(message)

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
                    activate = not any(self.active[side] for side in self.sides)
                    self.active = {
                        side: activate if side in self.sides else False
                        for side in SIDES
                    }
                    changed = True
                elif key in ("l", "r"):
                    side = "left" if key == "l" else "right"
                    if side not in self.sides:
                        self._show(f"{side}: not configured for this run")
                        continue
                    self.active[side] = not self.active[side]
                    changed = True
                elif key == "x":
                    self.active = {side: False for side in SIDES}
                    changed = True
                elif key == "o":
                    self.requests["open_hands"] = True
                elif key == "h":
                    self.requests["reset"] = True
                elif key == "q":
                    self.active = {side: False for side in SIDES}
                    self._show_state()
                    raise KeyboardInterrupt
        if changed:
            self._show_state()
        return dict(self.active)

    def take_requests(self) -> dict[str, bool]:
        """Return and clear the one-shot requests collected by poll()."""
        taken = self.requests
        self.requests = {name: False for name in taken}
        return taken

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
        sides: tuple[str, ...] = SIDES,
    ) -> None:
        import xrobotoolkit_sdk as xrt

        if not sides or set(sides).difference(SIDES):
            raise ValueError(f"Invalid motion tracker sides: {sides}")
        if set(serials) != set(SIDES) or any(not value for value in serials.values()):
            raise ValueError("Both motion tracker serial numbers are required")
        if len(sides) > 1 and serials["left"] == serials["right"]:
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
        self.sides = tuple(sides)
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
        self.detected_serials: list[str] = []
        self.readiness = {
            side: "waiting" if side in self.sides else "not required"
            for side in SIDES
        }
        self.keyboard = KeyboardActivation(keyboard_device, sides=self.sides)
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
            self.detected_serials = serials
            for side in SIDES:
                if side not in self.sides:
                    self.readiness[side] = "not required"
                elif self.serials[side] not in serials:
                    self.readiness[side] = (
                        f"missing ({self.serials[side]}); detected={serials}"
                    )
                else:
                    self.readiness[side] = (
                        "detected but SDK motion timestamp is unavailable "
                        f"or unstable ({timestamp_before}->{timestamp_after})"
                    )
            return None
        if (
            count != len(serials)
            or count != len(poses)
            or len(set(serials)) != len(serials)
        ):
            self.detected_serials = serials
            for side in self.sides:
                self.readiness[side] = (
                    f"inconsistent SDK frame count={count}, "
                    f"serials={len(serials)}, poses={len(poses)}"
                )
            return None
        self.detected_serials = serials
        by_serial = {
            serial: np.asarray(pose, dtype=float)
            for serial, pose in zip(serials, poses)
        }
        missing = [
            side for side in self.sides if self.serials[side] not in by_serial
        ]
        for side in SIDES:
            if side not in self.sides:
                self.readiness[side] = "not required"
            elif side in missing:
                self.readiness[side] = (
                    f"missing ({self.serials[side]}); detected={serials}"
                )
            else:
                raw_pose = by_serial[self.serials[side]]
                self.readiness[side] = (
                    "ready" if is_valid_xr_pose(raw_pose) else "invalid pose"
                )
        if missing:
            return None
        selected = {
            side: by_serial[self.serials[side]]
            for side in self.sides
        }
        if not all(is_valid_xr_pose(pose) for pose in selected.values()):
            return None
        return timestamp_after, selected, serials

    def _wait_until_ready(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        next_report = 0.0
        while time.monotonic() < deadline:
            self.keyboard.poll()
            snapshot = self._snapshot()
            if snapshot is not None:
                timestamp, raw_poses, _ = snapshot
                now = time.monotonic()
                self.last_motion_timestamp = timestamp
                self.last_motion_update_at = now
                for side in self.sides:
                    self.last_poses[side] = _apply_local_transform(
                        xr_pose_to_world(raw_poses[side]), self.transforms[side]
                    )
                    self.last_position_changed_at[side] = now
                    self.last_rotation_changed_at[side] = now
                self.keyboard.disable_all("motion trackers initialized")
                self.keyboard.show("Tracker state: " + self.status_summary())
                return
            now = time.monotonic()
            if now >= next_report:
                self.keyboard.show(
                    "Waiting for trackers: "
                    + self.status_summary()
                    + f" | timeout in {max(0.0, deadline - now):.0f}s"
                )
                next_report = now + 1.0
            time.sleep(0.05)
        raise TimeoutError(
            "Timed out waiting for motion trackers; per-side state: "
            + self.status_summary()
        )

    def status_summary(self) -> str:
        parts = []
        for side in SIDES:
            configured = self.serials[side]
            state = self.readiness[side]
            control = "ON" if self.last_activations[side] else "off"
            parts.append(
                f"{side}={state}, configured={configured}, control={control}"
            )
        return " | ".join(parts)

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
            for side in self.sides:
                self.readiness[side] = "SDK motion timestamp moved backwards"
            return "motion tracker timestamp moved backwards"
        if timestamp == previous_timestamp:
            return None

        elapsed = (timestamp - previous_timestamp) * 1e-9
        faults = []
        for side in self.sides:
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
            side_fault = None
            if position_delta > self.max_position_jump:
                side_fault = (
                    f"{side} tracker position jumped {position_delta:.3f} m"
                )
            elif rotation_delta > self.max_rotation_jump:
                side_fault = (
                    f"{side} tracker rotation jumped {rotation_delta:.3f} rad"
                )
            elif linear_speed > self.max_linear_speed:
                side_fault = (
                    f"{side} tracker linear speed {linear_speed:.3f} m/s"
                )
            elif angular_speed > self.max_angular_speed:
                side_fault = (
                    f"{side} tracker angular speed {angular_speed:.3f} rad/s"
                )

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
                side_fault = f"{side} tracker position is frozen"
            elif (
                rotation_changed_at is None
                or now - rotation_changed_at > self.frozen_timeout
            ):
                side_fault = f"{side} tracker rotation is frozen"
            self.readiness[side] = side_fault or "ready"
            if side_fault is not None:
                faults.append(side_fault)
            self.last_poses[side] = poses[side]
        return "; ".join(faults) if faults else None

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
            if snapshot is not None and self.last_motion_update_at is not None:
                age = now - self.last_motion_update_at
                for side in self.sides:
                    self.readiness[side] = (
                        f"stale: no new SDK motion frame for {age:.2f}s"
                    )
            self.disable_all("motion tracker data missing or stale")
            return None
        poses = {
            side: _apply_local_transform(
                xr_pose_to_world(raw_poses[side]),
                self.transforms[side],
            )
            for side in self.sides
        }
        for side in SIDES:
            if side not in poses:
                poses[side] = Pose(np.zeros(3), np.eye(3))
                activations[side] = False
        motion_fault = self._motion_fault(poses, activations, timestamp, now)
        self.last_motion_timestamp = timestamp
        if motion_fault is not None:
            self.disable_all(motion_fault)
            return None
        self.last_activations = dict(activations)
        return TeleopSample(poses, activations, now)

    def take_requests(self) -> dict[str, bool]:
        return self.keyboard.take_requests()

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


class PoseEma:
    """Low-pass a pose, with optional motion-adaptive rotation smoothing.

    Position uses a fixed EMA time constant. Rotation can use a slow time
    constant near the current output, suppressing optical wrist jitter, and
    continuously approach a fast time constant as tracking error grows during
    an intentional turn. Setting no adaptive parameters preserves the original
    fixed-time-constant behavior.
    """

    def __init__(
        self,
        time_constant: float,
        *,
        rotation_slow_time_constant: float | None = None,
        rotation_fast_time_constant: float | None = None,
        rotation_error_low: float | None = None,
        rotation_error_high: float | None = None,
    ) -> None:
        if not np.isfinite(time_constant) or time_constant <= 0:
            raise ValueError("Smoothing time constant must be positive")
        self.time_constant = float(time_constant)
        adaptive = (
            rotation_slow_time_constant,
            rotation_fast_time_constant,
            rotation_error_low,
            rotation_error_high,
        )
        if any(value is not None for value in adaptive):
            if any(value is None for value in adaptive):
                raise ValueError(
                    "Adaptive rotation smoothing requires all four parameters"
                )
            slow, fast, low, high = (float(value) for value in adaptive)
            if not all(np.isfinite(value) and value > 0 for value in (slow, fast, low, high)):
                raise ValueError(
                    "Adaptive rotation smoothing parameters must be positive"
                )
            if slow < fast:
                raise ValueError(
                    "Rotation slow time constant must be at least the fast one"
                )
            if high <= low:
                raise ValueError(
                    "Rotation high error must be greater than low error"
                )
            self.rotation_adaptive = (slow, fast, low, high)
        else:
            self.rotation_adaptive = None
        self.value: Pose | None = None

    def reset(self) -> None:
        self.value = None

    def update(self, pose: Pose, elapsed: float | None) -> Pose:
        if self.value is None or elapsed is None or elapsed <= 0.0:
            self.value = pose
            return pose
        position_alpha = 1.0 - math.exp(
            -float(elapsed) / self.time_constant
        )
        rotation_delta = pin.log3(pose.rotation @ self.value.rotation.T)
        rotation_time_constant = self.time_constant
        if self.rotation_adaptive is not None:
            slow, fast, low, high = self.rotation_adaptive
            activation = np.clip(
                (np.linalg.norm(rotation_delta) - low) / (high - low),
                0.0,
                1.0,
            )
            rotation_time_constant = slow + activation * (fast - slow)
        rotation_alpha = 1.0 - math.exp(
            -float(elapsed) / rotation_time_constant
        )
        rotation = (
            pin.exp3(rotation_alpha * rotation_delta)
            @ self.value.rotation
        )
        self.value = Pose(
            self.value.position
            + position_alpha * (pose.position - self.value.position),
            rotation,
        )
        return self.value


class HandRootInput:
    """Arm teleoperation from the wrist joint of the optical hand skeleton.

    Why this exists: the motion tracker's position comes from the headset
    cameras seeing the tracker and freezes, then jumps, when the wrist flips
    into a side-grasp pose, while optical hand tracking of the same hand keeps
    updating through those exact poses (observed on hardware, 2026-07-25). The
    skeleton's wrist joint is a full pose in the same XR frame as the trackers,
    so it can stand in for them, at a measured cost: against
    hand_coexistence.jsonl the wrist position carries 18-37 mm rms
    high-frequency noise versus sub-millimetre for a tracker with optical fix,
    rotation jitters by ~0.1 rad between samples, and the whole skeleton
    occasionally teleports (0.906 m in a single step in that recording). The
    EMA absorbs the noise; the per-sample jump guard turns a teleport into a
    disengage instead of an arm lunge.

    Liveness comes from `SkeletonLiveness`, the same rules the hand pipeline
    uses: `isActive` must be 1 and the array must keep changing, because
    plausible pose arrays keep being served after tracking loss. A fault on an
    ENGAGED side disengages everything, exactly like a tracker fault; a fault
    on a disengaged side is ignored so one hand leaving the cameras does not
    stop the other arm.
    """

    def __init__(
        self,
        ready_timeout: float,
        stale_timeout: float,
        frozen_timeout: float,
        max_position_jump: float,
        max_rotation_jump: float,
        smoothing_time_constant: float,
        keyboard_device: str,
        rotation_slow_time_constant: float | None = None,
        rotation_fast_time_constant: float | None = None,
        rotation_error_low: float | None = None,
        rotation_error_high: float | None = None,
    ) -> None:
        import xrobotoolkit_sdk as xrt

        limits = (
            ready_timeout,
            stale_timeout,
            frozen_timeout,
            max_position_jump,
            max_rotation_jump,
            smoothing_time_constant,
        )
        if any(not np.isfinite(value) or value <= 0 for value in limits):
            raise ValueError("Hand root timeouts and limits must be positive")

        self.xrt = xrt
        self.max_position_jump = float(max_position_jump)
        self.max_rotation_jump = float(max_rotation_jump)
        self.liveness = {
            side: SkeletonLiveness(stale_timeout, frozen_timeout) for side in SIDES
        }
        self.filters = {
            side: PoseEma(
                smoothing_time_constant,
                rotation_slow_time_constant=rotation_slow_time_constant,
                rotation_fast_time_constant=rotation_fast_time_constant,
                rotation_error_low=rotation_error_low,
                rotation_error_high=rotation_error_high,
            )
            for side in SIDES
        }
        self.last_raw_wrist: dict[str, np.ndarray | None] = {
            side: None for side in SIDES
        }
        self.last_raw_pose: dict[str, Pose | None] = {side: None for side in SIDES}
        self.last_observed_at: dict[str, float | None] = {
            side: None for side in SIDES
        }
        self.smoothed: dict[str, Pose | None] = {side: None for side in SIDES}
        self.last_activations = {side: False for side in SIDES}
        self.keyboard = KeyboardActivation(keyboard_device)
        try:
            self.xrt.init()
            self._wait_until_ready(float(ready_timeout))
        except BaseException:
            self.close()
            raise

    def _read_raw(self, side: str):
        if side == "left":
            return (
                self.xrt.get_left_hand_tracking_state(),
                int(self.xrt.get_left_hand_is_active()),
            )
        return (
            self.xrt.get_right_hand_tracking_state(),
            int(self.xrt.get_right_hand_is_active()),
        )

    def _forget_raw(self, side: str) -> None:
        # The smoothed pose is deliberately kept: a disengaged side still needs
        # a pose in the TeleopSample, and the mapper ignores it while inactive.
        self.last_raw_wrist[side] = None
        self.last_raw_pose[side] = None
        self.last_observed_at[side] = None
        self.filters[side].reset()

    def _observe(self, side: str, engaged: bool, now: float) -> str | None:
        """Ingest one poll for one side; return a fault description or None."""
        state = self.liveness[side]
        try:
            raw, is_active = self._read_raw(side)
        except Exception as error:  # noqa: BLE001 - SDK raises bare exceptions
            state.fault = f"hand SDK read failed: {error}"
            state.forget()
            self._forget_raw(side)
            return f"{side} {state.fault}"
        if state.accept(raw, is_active, now) is None:
            self._forget_raw(side)
            return f"{side} hand: {state.fault}"
        wrist = np.asarray(raw, dtype=float)[OPENXR_WRIST]
        previous_wrist = self.last_raw_wrist[side]
        if previous_wrist is not None and np.array_equal(previous_wrist, wrist):
            # No new optical frame this poll; the smoothed pose stands. The
            # loop polls at 100 Hz but the skeleton updates at ~52 Hz, so this
            # is the common case, and advancing the EMA on repeats would let
            # the filter converge onto stale data.
            return None
        pose = xr_pose_to_world(wrist)
        fault = None
        previous_pose = self.last_raw_pose[side]
        if engaged and self.last_activations[side] and previous_pose is not None:
            position_delta = float(
                np.linalg.norm(pose.position - previous_pose.position)
            )
            rotation_delta = float(
                np.linalg.norm(pin.log3(pose.rotation @ previous_pose.rotation.T))
            )
            if position_delta > self.max_position_jump:
                fault = f"{side} hand root position jumped {position_delta:.3f} m"
            elif rotation_delta > self.max_rotation_jump:
                fault = f"{side} hand root rotation jumped {rotation_delta:.3f} rad"
        previous_at = self.last_observed_at[side]
        self.last_raw_wrist[side] = np.asarray(wrist, dtype=float).copy()
        self.last_raw_pose[side] = pose
        self.last_observed_at[side] = now
        if fault is not None:
            # The raw baseline advanced so a persistent re-lock does not trip
            # the guard forever, but the teleported sample stays out of the
            # filter.
            return fault
        elapsed = None if previous_at is None else now - previous_at
        self.smoothed[side] = self.filters[side].update(pose, elapsed)
        return None

    def _wait_until_ready(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.keyboard.poll()
            now = time.monotonic()
            for side in SIDES:
                self._observe(side, False, now)
            if all(self.smoothed[side] is not None for side in SIDES):
                self.keyboard.disable_all("hand tracking initialized")
                return
            time.sleep(0.05)
        faults = {side: state.fault for side, state in self.liveness.items()}
        raise TimeoutError(
            "Timed out waiting for both optical hand skeletons; per-side "
            f"status: {faults}"
        )

    def sample(self) -> TeleopSample | None:
        activations = self.keyboard.poll()
        now = time.monotonic()
        fault = None
        for side in SIDES:
            side_fault = self._observe(side, activations[side], now)
            if side_fault is not None and activations[side]:
                fault = side_fault
        if fault is not None:
            self.disable_all(fault)
            return None
        if any(self.smoothed[side] is None for side in SIDES):
            # Only reachable before _wait_until_ready seeded both sides.
            self.disable_all("hand root pose not yet available")
            return None
        self.last_activations = dict(activations)
        return TeleopSample(
            poses={side: self.smoothed[side] for side in SIDES},
            activations=activations,
            timestamp=now,
        )

    def debug_raw_poses(self) -> dict[str, Pose]:
        """Return the latest accepted pre-EMA wrist poses for instrumentation.

        These are snapshots of state already read by :meth:`sample`; this
        method deliberately performs no SDK I/O so enabling a debug log cannot
        alter control-loop timing or skeleton liveness.
        """
        return {
            side: pose
            for side, pose in self.last_raw_pose.items()
            if pose is not None
        }

    def take_requests(self) -> dict[str, bool]:
        return self.keyboard.take_requests()

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


def create_pico_input(
    config: InputConfig,
    input_type: str,
    sides: tuple[str, ...] = SIDES,
):
    # The desktop GUI and the Python SDK compete for the PC Service feedback
    # stream; whichever connects last can leave the other client open but no
    # longer receiving fresh poses. A recorded teleop session with the GUI in
    # use showed exactly that: tracker positions updating at sub-hertz in our
    # client while the GUI displayed them moving accurately. Every diagnostic
    # script already refuses to start next to the GUI; teleoperation, the one
    # place where degraded input moves hardware, must refuse too.
    gui_pids = desktop_gui_pids()
    if gui_pids:
        raise RuntimeError(
            "The desktop RobotLinuxDemo GUI is running "
            f"(PID(s): {gui_pids}) and would compete for the PC Service "
            "stream. Close only the desktop GUI; keep RoboticsService and "
            "the headset app running."
        )
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
            sides=sides,
        )
    if input_type == "hand-roots":
        hand_roots = config.hand_roots
        return HandRootInput(
            ready_timeout=hand_roots.ready_timeout,
            stale_timeout=hand_roots.stale_timeout,
            frozen_timeout=hand_roots.frozen_timeout,
            max_position_jump=hand_roots.max_position_jump,
            max_rotation_jump=hand_roots.max_rotation_jump,
            smoothing_time_constant=hand_roots.smoothing_time_constant,
            keyboard_device=hand_roots.keyboard_device,
            rotation_slow_time_constant=(
                hand_roots.rotation_slow_time_constant
            ),
            rotation_fast_time_constant=(
                hand_roots.rotation_fast_time_constant
            ),
            rotation_error_low=hand_roots.rotation_error_low,
            rotation_error_high=hand_roots.rotation_error_high,
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
