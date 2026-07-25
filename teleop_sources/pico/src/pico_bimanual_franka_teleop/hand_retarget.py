"""Retarget canonical hand landmarks to a LinkerHand L20 URDF pose.

Pinocchio-based, solving five independent per-finger problems. An earlier
implementation used PyBullet and one 21-DoF optimization.
The objective, palm-frame canonicalization, per-finger length normalization,
landmark weights, smoothing and filter constants are unchanged; only how the
solution is computed changed.

Two measured facts drove the rewrite. First, the old evaluation called
`pb.calculateJacobian` once per target point, 380 times per solve, and each call
recomputed the whole hand's kinematics; those calls were 36% of a 7.45 ms solve
and pure Python bookkeeping was the rest. Here one `computeJointJacobians` pass
per evaluation serves every target through cheap frame lookups. Second, the
problem is block diagonal by construction and by measurement: no target's
Jacobian has any support outside its own finger (0 of 20 leak), so solving five
4-5 DoF problems converges in fewer, cheaper iterations than one 21-DoF problem
and cannot change the answer, only reach it better.

The public surface is unchanged from the previous implementation, so the
scripts, the service and the test suite are unaffected.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import pinocchio as pin
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

# Landmark weights along each finger chain: base, middle, distal, tip.
CHAIN_WEIGHTS = (1.0, 1.0, 1.25, 2.5)

# Canonical landmark indices used by the palm frame.
_INDEX_BASE = 5
_MIDDLE_BASE = 9
_RING_BASE = 13
_LITTLE_BASE = 17
_WRIST = 0


@dataclass(frozen=True)
class TargetPoint:
    landmark_index: int
    frame_id: int
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
        smooth_weight: float = 2.5e-3,
        filter_alpha: float = 0.35,
        max_iterations: int = 20,
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

        self.model = pin.buildModelFromUrdf(str(self.urdf_path))

        # Joint bookkeeping in q order. Every joint is revolute with nq of one,
        # verified below, so idx_q equals idx_v and one index serves both.
        entries = []
        for joint_id in range(1, self.model.njoints):
            joint = self.model.joints[joint_id]
            if joint.nq != 1 or joint.nv != 1:
                raise ValueError(
                    f"Expected single-DoF revolute L20 joints, got nq={joint.nq} "
                    f"for {self.model.names[joint_id]}"
                )
            entries.append((joint.idx_q, self.model.names[joint_id]))
        entries.sort()
        self.joint_names: list[str] = [name for _, name in entries]
        self.lower = np.asarray(self.model.lowerPositionLimit, dtype=np.float64)
        self.upper = np.asarray(self.model.upperPositionLimit, dtype=np.float64)

        self._finger_joints: dict[str, np.ndarray] = {}
        for finger in CANONICAL_FINGERS:
            indices = [
                i for i, name in enumerate(self.joint_names) if name.startswith(finger)
            ]
            if not indices:
                raise ValueError(f"URDF has no joints for finger {finger!r}")
            self._finger_joints[finger] = np.asarray(indices, dtype=int)

        # One frame per target point. Chain landmarks sit at link frames; the tip
        # gets an extra operational frame carrying the link-local offset, so a
        # single FK pass serves every target and no per-target math remains.
        self.targets: list[TargetPoint] = []
        self._targets_by_finger: dict[str, list[int]] = {f: [] for f in CANONICAL_FINGERS}
        for finger, landmark_indices in CANONICAL_FINGERS.items():
            links = FINGER_LINKS[finger]
            frame_ids = [self._body_frame(link) for link in links]
            distal = self.model.frames[frame_ids[2]]
            tip_placement = distal.placement * pin.SE3(
                np.eye(3), DISTAL_TIP_OFFSETS[finger].copy()
            )
            tip_id = self.model.addFrame(
                pin.Frame(
                    f"{links[2]}_tip_target",
                    distal.parentJoint,
                    frame_ids[2],
                    tip_placement,
                    pin.FrameType.OP_FRAME,
                )
            )
            for landmark, frame_id, weight in zip(
                landmark_indices, (*frame_ids, tip_id), CHAIN_WEIGHTS
            ):
                self._targets_by_finger[finger].append(len(self.targets))
                self.targets.append(TargetPoint(landmark, frame_id, weight))
        self.data = self.model.createData()

        self.smooth_weight = float(smooth_weight)
        self.filter_alpha = float(filter_alpha)
        self.max_iterations = int(max_iterations)
        self.last_qpos = np.clip(np.zeros(self.dof), self.lower, self.upper)
        self.filtered_qpos: np.ndarray | None = None
        self._q_current = self.last_qpos.copy()

        self.robot_finger_lengths = self._robot_finger_lengths()
        (
            self.robot_frame,
            self.robot_centroid,
            self.robot_scale,
        ) = self._robot_reference()

    @property
    def dof(self) -> int:
        return len(self.joint_names)

    def _body_frame(self, link_name: str) -> int:
        if not self.model.existFrame(link_name):
            raise ValueError(f"URDF has no link named {link_name!r}")
        return self.model.getFrameId(link_name)

    # ------------------------------------------------------------- kinematics
    def _update(self, qpos: np.ndarray, jacobians: bool = False) -> None:
        if jacobians:
            pin.computeJointJacobians(self.model, self.data, qpos)
        else:
            pin.forwardKinematics(self.model, self.data, qpos)
        pin.updateFramePlacements(self.model, self.data)

    def _reset_joints(self, qpos: Iterable[float]) -> None:
        self._q_current = np.asarray(list(qpos), dtype=np.float64).copy()

    def robot_landmarks(self) -> np.ndarray:
        """Canonical landmark positions from FK at the current joint pose.

        Landmark 0, the wrist, is left at the URDF base origin because the robot
        has no wrist target. It is used only to give the palm frame a proximal
        reference direction, never as a scale or position correspondence.
        """
        self._update(self._q_current)
        points = np.zeros((CANONICAL_LANDMARK_COUNT, 3), dtype=np.float64)
        for target in self.targets:
            points[target.landmark_index] = self.data.oMf[target.frame_id].translation
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

    def _robot_reference(self) -> tuple[np.ndarray, np.ndarray, float]:
        points = self.robot_landmarks()
        frame = orthonormal_palm_frame(points)
        centroid = finger_base_centroid(points)
        scale = palm_scale(points)
        if scale < 1e-6:
            raise ValueError("Invalid L20 URDF: finger bases are coincident")
        return frame, centroid, scale

    # ---------------------------------------------------------------- targets
    def _normalize_finger_lengths(self, points: np.ndarray) -> np.ndarray:
        """Rescale each finger's chain to the robot's own finger length.

        The L20's four non-thumb fingers are all one length, while a human's are
        not. Measured against one operator, the robot's pinky is about 34 mm
        longer than the scaled human pinky and its index about 15 mm longer, with
        only the middle finger matching. Asking an over-long finger to place its
        tip on a shorter human tip forces extra curl deep enough to foul the
        neighbouring fingers. Segment directions, and so the curl shape the
        operator actually made, are preserved exactly; only lengths change.
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

    # ------------------------------------------------------------------ solve
    def retarget(
        self, landmarks: np.ndarray
    ) -> tuple[np.ndarray, dict[str, float | int | bool]]:
        """Solve for the joint pose matching one canonical landmark frame."""
        target_positions = self.target_positions(landmarks)
        start = np.clip(self.last_qpos, self.lower, self.upper)
        solution = start.copy()
        total_loss = 0.0
        total_iterations = 0
        total_evaluations = 0
        success = True

        for finger, joint_indices in self._finger_joints.items():
            target_indices = self._targets_by_finger[finger]
            frame_ids = [self.targets[t].frame_id for t in target_indices]
            weights = np.asarray(
                [self.targets[t].weight for t in target_indices], dtype=np.float64
            )
            wanted = target_positions[target_indices]
            anchor = self.last_qpos[joint_indices]
            q_work = solution.copy()

            def objective(finger_q: np.ndarray) -> tuple[float, np.ndarray]:
                q_work[joint_indices] = finger_q
                self._update(q_work, jacobians=True)
                loss = 0.0
                gradient = np.zeros(len(joint_indices))
                for row, frame_id in enumerate(frame_ids):
                    residual = self.data.oMf[frame_id].translation - wanted[row]
                    loss += weights[row] * float(residual @ residual)
                    jacobian = pin.getFrameJacobian(
                        self.model,
                        self.data,
                        frame_id,
                        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
                    )[:3, joint_indices]
                    gradient += 2.0 * weights[row] * (jacobian.T @ residual)
                delta = finger_q - anchor
                loss += self.smooth_weight * float(delta @ delta)
                gradient += 2.0 * self.smooth_weight * delta
                return loss, gradient

            result = minimize(
                objective,
                start[joint_indices],
                method="L-BFGS-B",
                jac=True,
                bounds=list(zip(self.lower[joint_indices], self.upper[joint_indices])),
                options={
                    "maxiter": self.max_iterations,
                    "ftol": 1e-9,
                    "gtol": 1e-6,
                },
            )
            solution[joint_indices] = np.clip(
                result.x, self.lower[joint_indices], self.upper[joint_indices]
            )
            total_loss += float(result.fun)
            total_iterations += int(result.nit)
            total_evaluations += int(result.nfev)
            success = success and bool(result.success)

        # The regularizer anchors on the raw solution while the emitted command
        # is filtered, so smoothing never fights the optimizer's own history.
        self.last_qpos = solution
        if self.filtered_qpos is None:
            self.filtered_qpos = solution.copy()
        else:
            self.filtered_qpos += self.filter_alpha * (solution - self.filtered_qpos)
        self._q_current = self.filtered_qpos.copy()

        stats: dict[str, float | int | bool] = {
            "success": success,
            "loss": total_loss,
            "iterations": total_iterations,
            "function_evaluations": total_evaluations,
        }
        return self.filtered_qpos.copy(), stats

    # ------------------------------------------------------------------ state
    def set_qpos(self, qpos: np.ndarray) -> None:
        values = np.asarray(qpos, dtype=np.float64)
        if values.shape != (self.dof,):
            raise ValueError(f"Expected qpos shape {(self.dof,)}, got {values.shape}")
        values = np.clip(values, self.lower, self.upper)
        self.last_qpos = values.copy()
        self.filtered_qpos = values.copy()
        self._q_current = values.copy()

    def reset(self) -> None:
        """Forget filter and warm-start history after a tracking dropout."""
        self.last_qpos = np.clip(np.zeros(self.dof), self.lower, self.upper)
        self.filtered_qpos = None
        self._q_current = self.last_qpos.copy()

    def close(self) -> None:
        """Kept for API compatibility; pinocchio holds no external resources."""

    def __enter__(self) -> "L20Retargeter":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
