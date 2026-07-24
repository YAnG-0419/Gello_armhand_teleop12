import math
import time

import numpy as np
import pinocchio as pin

from .pose_mapping import is_valid_xr_pose, xr_pose_to_world
from .types import Pose, XrSample


class XrInput:
    def __init__(self, ready_timeout: float = 30.0) -> None:
        import xrobotoolkit_sdk as xrt

        if ready_timeout <= 0.0:
            raise ValueError("ready_timeout must be positive")
        self.xrt = xrt
        self.xrt.init()
        self._wait_until_controllers_ready(ready_timeout)

    def _raw_poses(self) -> tuple[np.ndarray, np.ndarray]:
        left = np.asarray(self.xrt.get_left_controller_pose(), dtype=float)
        right = np.asarray(self.xrt.get_right_controller_pose(), dtype=float)
        return left, right

    def _wait_until_controllers_ready(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            left, right = self._raw_poses()
            if is_valid_xr_pose(left) and is_valid_xr_pose(right):
                return
            time.sleep(0.05)
        raise TimeoutError(
            "Timed out waiting for valid PICO controller poses from XRoboToolkit"
        )

    def sample(self) -> XrSample | None:
        left, right = self._raw_poses()
        if not (is_valid_xr_pose(left) and is_valid_xr_pose(right)):
            return None
        return XrSample(
            poses={
                "left": xr_pose_to_world(left),
                "right": xr_pose_to_world(right),
            },
            grips={
                "left": float(self.xrt.get_left_grip()),
                "right": float(self.xrt.get_right_grip()),
            },
            timestamp=time.monotonic(),
        )

    def close(self) -> None:
        self.xrt.close()


class MockXrInput:
    def __init__(self) -> None:
        self.started_at = time.monotonic()

    def sample(self) -> XrSample | None:
        elapsed = time.monotonic() - self.started_at
        angle = 0.12 * math.sin(elapsed)
        rotation = pin.exp3(np.array([0.0, angle, 0.0]))
        offset = 0.04 * math.sin(0.8 * elapsed)
        return XrSample(
            poses={
                "left": Pose(np.array([-0.2, 0.0, 1.2 + offset]), rotation),
                "right": Pose(np.array([0.2, 0.0, 1.2 + offset]), rotation.T),
            },
            grips={"left": 1.0, "right": 1.0},
            timestamp=time.monotonic(),
        )

    def close(self) -> None:
        return None
