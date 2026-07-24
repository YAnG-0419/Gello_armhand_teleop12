import numpy as np
import pinocchio as pin

from .types import Pose


# PICO/OpenXR: +X right, +Y up, -Z forward.
# Robot world: +X forward, +Y left, +Z up.
R_HEADSET_TO_WORLD = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ]
)


def is_valid_xr_pose(raw_pose: np.ndarray) -> bool:
    raw = np.asarray(raw_pose, dtype=float)
    return (
        raw.shape == (7,)
        and np.all(np.isfinite(raw))
        and float(np.linalg.norm(raw[3:])) >= 1e-8
    )


def xr_pose_to_world(raw_pose: np.ndarray) -> Pose:
    raw = np.asarray(raw_pose, dtype=float)
    if not is_valid_xr_pose(raw):
        raise ValueError("XR pose must contain seven finite values with a non-zero quaternion")
    quaternion_xyzw = raw[3:]
    quaternion_xyzw = quaternion_xyzw / np.linalg.norm(quaternion_xyzw)
    rotation_xr = pin.Quaternion(
        quaternion_xyzw[3],
        quaternion_xyzw[0],
        quaternion_xyzw[1],
        quaternion_xyzw[2],
    ).toRotationMatrix()
    return Pose(
        R_HEADSET_TO_WORLD @ raw[:3],
        R_HEADSET_TO_WORLD @ rotation_xr @ R_HEADSET_TO_WORLD.T,
    )


class RelativePoseMapper:
    def __init__(
        self,
        translation_scale: float,
        rotation_scale: float,
        grip_threshold: float,
    ) -> None:
        if translation_scale <= 0.0 or rotation_scale <= 0.0:
            raise ValueError("Pose scales must be positive")
        if not 0.0 < grip_threshold <= 1.0:
            raise ValueError("Grip threshold must be in (0, 1]")
        self.translation_scale = float(translation_scale)
        self.rotation_scale = float(rotation_scale)
        self.grip_threshold = float(grip_threshold)
        self.controller_anchor: Pose | None = None
        self.robot_anchor: Pose | None = None

    @property
    def active(self) -> bool:
        return self.controller_anchor is not None

    def update(self, controller: Pose, grip: float, robot_pose: Pose) -> Pose | None:
        engaged = grip >= self.grip_threshold
        if not engaged:
            self.controller_anchor = None
            self.robot_anchor = None
            return None
        if self.controller_anchor is None:
            self.controller_anchor = controller
            self.robot_anchor = robot_pose
            return robot_pose

        assert self.robot_anchor is not None
        delta_position = controller.position - self.controller_anchor.position
        delta_rotation = controller.rotation @ self.controller_anchor.rotation.T
        if self.rotation_scale != 1.0:
            rotation_vector = pin.log3(delta_rotation) * self.rotation_scale
            delta_rotation = pin.exp3(rotation_vector)
        return Pose(
            self.robot_anchor.position + self.translation_scale * delta_position,
            delta_rotation @ self.robot_anchor.rotation,
        )
