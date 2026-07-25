"""Retarget canonical hand landmarks to a LinkerHand L20 URDF pose.

Ported from the sibling WiLoR repository's `wilor/retargeting/l20.py`, which
drove this hand from monocular MANO reconstructions. The optimizer, palm-frame
canonicalization, landmark weights, smoothing and filter constants are kept
deliberately unchanged, because that combination is the part already validated
in simulation there. Only the input representation differs: landmarks now
arrive from `hand_landmarks.to_canonical_landmarks`, which converts PICO's
26-joint OpenXR skeleton into the same canonical 21-landmark layout.

The objective matches human and robot finger landmarks in their own palm-local
frames, so it is invariant to the global XR origin, to overall hand size, and to
the operator's wrist orientation. That invariance is what lets the optical
skeleton drive the fingers while the wrist trackers independently drive the arm.

The output is 21 URDF joint values in radians, clipped to the URDF limits. It is
not a hardware command; converting to the vendor's 0..255 slots is the ROS
bridge's job, so that robot-side code never acquires a PICO dependency.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import pybullet as pb
from scipy.optimize import minimize

from .hand_landmarks import (
    CANONICAL_LANDMARK_COUNT,
    finger_base_centroid,
    palm_scale,
)

# Canonical landmark indices per finger, ordered base, middle, distal, tip.
CANONICAL_FINGERS = {
    "thumb": (1, 2, 3, 4),
    "index": (5, 6, 7, 8),
    "middle": (9, 10, 11, 12),
    "ring": (13, 14, 15, 16),
    "pinky": (17, 18, 19, 20),
}

FINGER_LINKS = {
    "thumb": ("thumb_metacarpals", "thumb_proximal", "thumb_distal"),
    "index": ("index_proximal", "index_middle", "index_distal"),
    "middle": ("middle_proximal", "middle_middle", "middle_distal"),
    "ring": ("ring_proximal", "ring_middle", "ring_distal"),
    "pinky": ("pinky_proximal", "pinky_middle", "pinky_distal"),
}

DISTAL_TIP_OFFSETS = {
    "thumb": np.array([0.0, 0.0, 0.032], dtype=np.float64),
    "index": np.array([0.0, 0.0, 0.023], dtype=np.float64),
    "middle": np.array([0.0, 0.0, 0.023], dtype=np.float64),
    "ring": np.array([0.0, 0.0, 0.023], dtype=np.float64),
    "pinky": np.array([0.0, 0.0, 0.023], dtype=np.float64),
}

# Canonical landmark indices used by the palm frame.
_INDEX_BASE = 5
_MIDDLE_BASE = 9
_RING_BASE = 13
_LITTLE_BASE = 17
_WRIST = 0


@dataclass(frozen=True)
class TargetPoint:
    landmark_index: int
    link_index: int
    local_position: np.ndarray
    weight: float


def _normalize(vector: np.ndarray, name: str) -> np.ndarray:
    norm = np.linalg.norm(vector)
    if norm < 1e-8:
        raise ValueError(f"Cannot construct palm frame: {name} has near-zero length")
    return vector / norm


def orthonormal_palm_frame(points: np.ndarray) -> np.ndarray:
    """Build an articulation-invariant palm frame from canonical landmarks."""
    lateral = _normalize(
        points[_INDEX_BASE] - points[_LITTLE_BASE], "index-to-little vector"
    )
    # The palm frame must be articulation-invariant. Fingertips move back toward
    # the wrist during a fist, so using them here rotates the reference frame and
    # cancels part of the very flexion that retargeting is supposed to preserve.
    forward_raw = (
        points[[_INDEX_BASE, _MIDDLE_BASE, _RING_BASE, _LITTLE_BASE]].mean(axis=0)
        - points[_WRIST]
    )
    forward = _normalize(
        forward_raw - lateral * np.dot(forward_raw, lateral),
        "wrist-to-finger-bases vector",
    )
    normal = _normalize(np.cross(lateral, forward), "palm normal")
    return np.column_stack((normal, lateral, forward))


class L20Retargeter:
    """Solve L20 URDF joint angles matching a canonical hand landmark set."""

    def __init__(
        self,
        urdf_path: str | Path,
        side: str,
        *,
        gui: bool = False,
        smooth_weight: float = 2.5e-3,
        filter_alpha: float = 0.35,
        max_iterations: int = 25,
        normalize_finger_length: bool = True,
    ) -> None:
        if side not in {"left", "right"}:
            raise ValueError(f"side must be 'left' or 'right', got {side!r}")
        if not 0.0 < filter_alpha <= 1.0:
            raise ValueError("filter_alpha must be in (0, 1]")

        self.side = side
        self.normalize_finger_length = bool(normalize_finger_length)
        self.urdf_path = Path(urdf_path).resolve()
        if not self.urdf_path.is_file():
            raise FileNotFoundError(f"L20 URDF not found: {self.urdf_path}")

        self.client = pb.connect(pb.GUI if gui else pb.DIRECT)
        if self.client < 0:
            raise RuntimeError("Failed to connect to PyBullet")
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0, physicsClientId=self.client)
        self.body = pb.loadURDF(
            str(self.urdf_path),
            useFixedBase=True,
            flags=pb.URDF_USE_INERTIA_FROM_FILE,
            physicsClientId=self.client,
        )

        self.joint_names: list[str] = []
        lower, upper = [], []
        self.link_indices: dict[str, int] = {}
        for joint_index in range(
            pb.getNumJoints(self.body, physicsClientId=self.client)
        ):
            info = pb.getJointInfo(self.body, joint_index, physicsClientId=self.client)
            if info[2] != pb.JOINT_REVOLUTE:
                raise ValueError(
                    "Expected only revolute L20 joints, got type "
                    f"{info[2]} at index {joint_index}"
                )
            self.joint_names.append(info[1].decode("utf-8"))
            self.link_indices[info[12].decode("utf-8")] = joint_index
            lower.append(info[8])
            upper.append(info[9])

        self.lower = np.asarray(lower, dtype=np.float64)
        self.upper = np.asarray(upper, dtype=np.float64)
        self.bounds = list(zip(self.lower, self.upper))
        self.smooth_weight = float(smooth_weight)
        self.filter_alpha = float(filter_alpha)
        self.max_iterations = int(max_iterations)
        self.last_qpos = np.clip(
            np.zeros(len(self.joint_names)), self.lower, self.upper
        )
        self.filtered_qpos: np.ndarray | None = None
        self._zeros = [0.0] * len(self.joint_names)

        self.targets = self._make_target_points()
        self._reset_joints(self.last_qpos)
        self.robot_finger_lengths = self._robot_finger_lengths()
        (
            self.robot_frame,
            self.robot_centroid,
            self.robot_scale,
        ) = self._robot_reference()

    @property
    def dof(self) -> int:
        return len(self.joint_names)

    def _make_target_points(self) -> list[TargetPoint]:
        targets: list[TargetPoint] = []
        for finger, landmark_indices in CANONICAL_FINGERS.items():
            links = FINGER_LINKS[finger]
            targets.extend(
                [
                    TargetPoint(
                        landmark_indices[0], self.link_indices[links[0]], np.zeros(3), 1.0
                    ),
                    TargetPoint(
                        landmark_indices[1], self.link_indices[links[1]], np.zeros(3), 1.0
                    ),
                    TargetPoint(
                        landmark_indices[2], self.link_indices[links[2]], np.zeros(3), 1.25
                    ),
                    TargetPoint(
                        landmark_indices[3],
                        self.link_indices[links[2]],
                        DISTAL_TIP_OFFSETS[finger],
                        2.5,
                    ),
                ]
            )
        return targets

    def _reset_joints(self, qpos: Iterable[float]) -> None:
        for index, value in enumerate(qpos):
            pb.resetJointState(
                self.body, index, float(value), physicsClientId=self.client
            )

    def _point_position(self, target: TargetPoint) -> np.ndarray:
        state = pb.getLinkState(
            self.body,
            target.link_index,
            computeForwardKinematics=True,
            physicsClientId=self.client,
        )
        position = np.asarray(state[4], dtype=np.float64)
        if np.any(target.local_position):
            rotation = np.asarray(pb.getMatrixFromQuaternion(state[5])).reshape(3, 3)
            position = position + rotation @ target.local_position
        return position

    def robot_landmarks(self) -> np.ndarray:
        """Canonical landmark positions from FK at the current joint pose.

        Landmark 0, the wrist, is left at the URDF base origin because the robot
        has no wrist target. It is used only to give the palm frame a proximal
        reference direction, never as a scale or position correspondence.
        """
        by_landmark = {
            target.landmark_index: self._point_position(target)
            for target in self.targets
        }
        points = np.zeros((CANONICAL_LANDMARK_COUNT, 3), dtype=np.float64)
        for index, position in by_landmark.items():
            points[index] = position
        return points

    def _robot_finger_lengths(self) -> dict[str, float]:
        points = self.robot_landmarks()
        return {
            finger: sum(
                float(np.linalg.norm(points[chain[i + 1]] - points[chain[i]]))
                for i in range(len(chain) - 1)
            )
            for finger, chain in CANONICAL_FINGERS.items()
        }

    def _normalize_finger_lengths(self, points: np.ndarray) -> np.ndarray:
        """Rescale each finger's chain to the robot's own finger length.

        The L20's four non-thumb fingers are all one length, while a human's are
        not. Measured against one operator, the robot's pinky is about 34 mm
        longer than the scaled human pinky and its index about 15 mm longer, with
        only the middle finger matching. Asking an over-long finger to place its
        tip on a shorter human tip forces extra curl, and the optimizer recruits
        abduction to help, driving mcp_roll into its narrow +/-0.17 rad limit.
        Because all four abduction axes share one sign, that tilts every finger
        the same way at differing curl depths and the finger bodies interfere.

        Segment directions, and so the curl shape the operator actually made, are
        preserved exactly; only segment lengths change. Each finger base stays
        where the palm-width scaling put it.
        """
        out = np.array(points, dtype=np.float64, copy=True)
        for finger, chain in CANONICAL_FINGERS.items():
            segments = [
                out[chain[i + 1]] - out[chain[i]] for i in range(len(chain) - 1)
            ]
            length = sum(float(np.linalg.norm(segment)) for segment in segments)
            if length < 1e-9:
                continue
            ratio = self.robot_finger_lengths[finger] / length
            position = out[chain[0]].copy()
            for index, segment in enumerate(segments):
                position = position + segment * ratio
                out[chain[index + 1]] = position
        return out

    def _robot_reference(self) -> tuple[np.ndarray, np.ndarray, float]:
        points = self.robot_landmarks()
        frame = orthonormal_palm_frame(points)
        centroid = finger_base_centroid(points)
        scale = palm_scale(points)
        if scale < 1e-6:
            raise ValueError("Invalid L20 URDF: finger bases are coincident")
        return frame, centroid, scale

    def target_positions(self, landmarks: np.ndarray) -> np.ndarray:
        """Canonicalize landmarks into the robot's palm frame and scale."""
        points = np.asarray(landmarks, dtype=np.float64)
        if points.shape != (CANONICAL_LANDMARK_COUNT, 3):
            raise ValueError(
                f"Expected canonical landmarks with shape "
                f"{(CANONICAL_LANDMARK_COUNT, 3)}, got {points.shape}"
            )
        if not np.isfinite(points).all():
            raise ValueError("Canonical landmarks contain NaN or infinity")

        # The palm frame needs the wrist only as a proximal direction, so it is
        # built before re-origining.
        human_frame = orthonormal_palm_frame(points - points[_WRIST])
        human_scale = palm_scale(points)
        if human_scale < 1e-6:
            raise ValueError("Invalid landmarks: finger bases are coincident")

        # Correspond the finger-base centroid, not the wrist. The URDF base
        # origin is not a wrist, so equating the two would bake its mount offset
        # into the scale and place targets beyond the hand's reach.
        centered = points - finger_base_centroid(points)
        canonical = centered @ human_frame
        canonical *= self.robot_scale / human_scale
        robot_points = canonical @ self.robot_frame.T + self.robot_centroid
        if self.normalize_finger_length:
            robot_points = self._normalize_finger_lengths(robot_points)
        return np.stack(
            [robot_points[target.landmark_index] for target in self.targets]
        )

    def _positions_and_jacobians(
        self, qpos: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        self._reset_joints(qpos)
        positions, jacobians = [], []
        q_list = qpos.tolist()
        for target in self.targets:
            positions.append(self._point_position(target))
            jacobian, _ = pb.calculateJacobian(
                self.body,
                target.link_index,
                target.local_position.tolist(),
                q_list,
                self._zeros,
                self._zeros,
                physicsClientId=self.client,
            )[:2]
            jacobians.append(np.asarray(jacobian, dtype=np.float64))
        return np.stack(positions), np.stack(jacobians)

    def retarget(
        self, landmarks: np.ndarray
    ) -> tuple[np.ndarray, dict[str, float | int | bool]]:
        """Solve for the joint pose matching one canonical landmark frame."""
        target_positions = self.target_positions(landmarks)
        weights = np.asarray(
            [target.weight for target in self.targets], dtype=np.float64
        )
        start = np.clip(self.last_qpos, self.lower, self.upper)

        def objective(qpos: np.ndarray) -> tuple[float, np.ndarray]:
            positions, jacobians = self._positions_and_jacobians(qpos)
            residual = positions - target_positions
            weighted = weights[:, None] * residual
            loss = float(np.sum(weighted * residual))
            gradient = 2.0 * np.einsum("ni,nij->j", weighted, jacobians)
            delta = qpos - self.last_qpos
            loss += self.smooth_weight * float(np.dot(delta, delta))
            gradient += 2.0 * self.smooth_weight * delta
            return loss, gradient

        result = minimize(
            objective,
            start,
            method="L-BFGS-B",
            jac=True,
            bounds=self.bounds,
            options={"maxiter": self.max_iterations, "ftol": 1e-9, "gtol": 1e-6},
        )
        qpos = np.clip(result.x, self.lower, self.upper)
        # The regularizer anchors on the raw solution while the emitted command
        # is filtered, so smoothing never fights the optimizer's own history.
        self.last_qpos = qpos
        if self.filtered_qpos is None:
            self.filtered_qpos = qpos.copy()
        else:
            self.filtered_qpos += self.filter_alpha * (qpos - self.filtered_qpos)
        self._reset_joints(self.filtered_qpos)

        stats: dict[str, float | int | bool] = {
            "success": bool(result.success),
            "loss": float(result.fun),
            "iterations": int(result.nit),
            "function_evaluations": int(result.nfev),
        }
        return self.filtered_qpos.copy(), stats

    def set_qpos(self, qpos: np.ndarray) -> None:
        values = np.asarray(qpos, dtype=np.float64)
        if values.shape != (self.dof,):
            raise ValueError(f"Expected qpos shape {(self.dof,)}, got {values.shape}")
        values = np.clip(values, self.lower, self.upper)
        self.last_qpos = values.copy()
        self.filtered_qpos = values.copy()
        self._reset_joints(values)

    def reset(self) -> None:
        """Forget filter and warm-start history after a tracking dropout."""
        self.last_qpos = np.clip(
            np.zeros(len(self.joint_names)), self.lower, self.upper
        )
        self.filtered_qpos = None
        self._reset_joints(self.last_qpos)

    def close(self) -> None:
        if self.client >= 0 and pb.isConnected(self.client):
            pb.disconnect(self.client)
            self.client = -1

    def __enter__(self) -> "L20Retargeter":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
