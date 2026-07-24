from dataclasses import dataclass

import numpy as np


SIDES = ("left", "right")


@dataclass(frozen=True)
class Pose:
    position: np.ndarray
    rotation: np.ndarray

    def __post_init__(self) -> None:
        position = np.asarray(self.position, dtype=float)
        rotation = np.asarray(self.rotation, dtype=float)
        if position.shape != (3,) or rotation.shape != (3, 3):
            raise ValueError("Pose requires a 3-vector and a 3x3 rotation")
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(rotation)):
            raise ValueError("Pose contains a non-finite value")
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
            raise ValueError("Pose rotation is not orthonormal")
        object.__setattr__(self, "position", position.copy())
        object.__setattr__(self, "rotation", rotation.copy())


@dataclass(frozen=True)
class XrSample:
    poses: dict[str, Pose]
    grips: dict[str, float]
    timestamp: float

    def __post_init__(self) -> None:
        if set(self.poses) != set(SIDES) or set(self.grips) != set(SIDES):
            raise ValueError("XR sample must contain left and right controllers")
        if not np.isfinite(self.timestamp):
            raise ValueError("XR timestamp is not finite")
        for grip in self.grips.values():
            if not np.isfinite(grip) or grip < 0.0 or grip > 1.0:
                raise ValueError("Grip values must be in [0, 1]")
