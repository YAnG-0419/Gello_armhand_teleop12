"""MANUS canonical-landmark retargeting for the right Linker O30i."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pinocchio as pin
from scipy.optimize import minimize

from pico_bimanual_franka_teleop.hand_landmarks import (
    CANONICAL_LANDMARK_COUNT,
    finger_base_centroid,
    palm_scale,
)
from pico_bimanual_franka_teleop.hand_retarget import (
    CANONICAL_FINGERS,
    CHAIN_WEIGHTS,
    orthonormal_palm_frame,
)

_TARGET_LINKS = {
    "thumb": (
        "thumb_metacarpals_base1",
        "thumb_metacarpals_base2",
        "thumb_proximal",
    ),
    "index": (
        "index_proximal",
        "index_middle",
        "index_distal",
    ),
    "middle": (
        "middle_proximal",
        "middle_middle",
        "middle_distal",
    ),
    "ring": (
        "ring_proximal",
        "ring_middle",
        "ring_distal",
    ),
    "pinky": (
        "pinky_proximal",
        "pinky_middle",
        "pinky_distal",
    ),
}
_TIP_LINKS = {
    "thumb": "thumb_distal",
    "index": "index_distal",
    "middle": "middle_distal",
    "ring": "ring_distal",
    "pinky": "pinky_distal",
}
_TIP_OFFSETS = {
    "thumb": np.array([0.0, 0.02329, 0.0]),
    "index": np.array([0.0, 0.0, 0.02346]),
    "middle": np.array([0.0, 0.0, 0.02346]),
    "ring": np.array([0.0, 0.0, 0.02346]),
    "pinky": np.array([0.0, 0.0, 0.02346]),
}


class O30IRetargeter:
    """Solve all 20 independent O30i URDF joints in canonical radians."""

    def __init__(
        self,
        urdf_path: str | Path,
        side: str,
        *,
        smooth_weight: float = 2.5e-3,
        filter_alpha: float = 0.7,
        max_iterations: int = 30,
    ) -> None:
        if side != "right":
            raise ValueError("the checked-in O30i retargeter currently supports right only")
        if not 0.0 < filter_alpha <= 1.0:
            raise ValueError("filter_alpha must be in (0, 1]")
        self.side = side
        self.urdf_path = Path(urdf_path).resolve()
        if not self.urdf_path.is_file():
            raise FileNotFoundError(f"O30i URDF not found: {self.urdf_path}")
        self.model = pin.buildModelFromUrdf(str(self.urdf_path))
        entries = sorted(
            (self.model.joints[joint_id].idx_q, self.model.names[joint_id])
            for joint_id in range(1, self.model.njoints)
        )
        self.joint_names = [name for _, name in entries]
        if self.model.nq != 20 or any(
            self.model.joints[joint_id].nq != 1
            for joint_id in range(1, self.model.njoints)
        ):
            raise ValueError("O30i URDF must contain 20 independent single-DoF joints")
        self.lower = np.asarray(self.model.lowerPositionLimit, dtype=np.float64)
        self.upper = np.asarray(self.model.upperPositionLimit, dtype=np.float64)
        self.smooth_weight = float(smooth_weight)
        self.filter_alpha = float(filter_alpha)
        self.max_iterations = int(max_iterations)
        self.last_qpos = np.clip(np.zeros(self.model.nq), self.lower, self.upper)
        self.filtered_qpos: np.ndarray | None = None

        self._joint_indices = {
            finger: np.asarray(
                [
                    index
                    for index, name in enumerate(self.joint_names)
                    if name.startswith(finger)
                ],
                dtype=int,
            )
            for finger in CANONICAL_FINGERS
        }
        self._frames: dict[str, list[int]] = {}
        for finger, links in _TARGET_LINKS.items():
            frame_ids = [self._body_frame(name) for name in links]
            distal_id = self._body_frame(_TIP_LINKS[finger])
            distal = self.model.frames[distal_id]
            tip_id = self.model.addFrame(
                pin.Frame(
                    f"{finger}_tip_target",
                    distal.parentJoint,
                    distal_id,
                    distal.placement
                    * pin.SE3(np.eye(3), _TIP_OFFSETS[finger].copy()),
                    pin.FrameType.OP_FRAME,
                )
            )
            self._frames[finger] = [*frame_ids, tip_id]
        self.data = self.model.createData()

        neutral = self.robot_landmarks(self.last_qpos)
        self.robot_frame = orthonormal_palm_frame(neutral)
        self.robot_centroid = finger_base_centroid(neutral)
        self.robot_scale = palm_scale(neutral)
        self.robot_finger_lengths = {
            finger: self._chain_length(neutral, chain)
            for finger, chain in CANONICAL_FINGERS.items()
        }
        if self.robot_scale < 1e-6:
            raise ValueError("invalid O30i URDF palm geometry")

    def _body_frame(self, link_name: str) -> int:
        if not self.model.existFrame(link_name):
            raise ValueError(f"O30i URDF has no link named {link_name!r}")
        return self.model.getFrameId(link_name)

    @staticmethod
    def _chain_length(points: np.ndarray, chain: tuple[int, ...]) -> float:
        return sum(
            float(np.linalg.norm(points[chain[index + 1]] - points[chain[index]]))
            for index in range(3)
        )

    def _update(self, qpos: np.ndarray, *, jacobians: bool = False) -> None:
        if jacobians:
            pin.computeJointJacobians(self.model, self.data, qpos)
        else:
            pin.forwardKinematics(self.model, self.data, qpos)
        pin.updateFramePlacements(self.model, self.data)

    def robot_landmarks(self, qpos: np.ndarray | None = None) -> np.ndarray:
        values = self.last_qpos if qpos is None else np.asarray(qpos, dtype=np.float64)
        self._update(values)
        points = np.zeros((CANONICAL_LANDMARK_COUNT, 3), dtype=np.float64)
        for finger, chain in CANONICAL_FINGERS.items():
            for landmark, frame_id in zip(chain, self._frames[finger], strict=True):
                points[landmark] = self.data.oMf[frame_id].translation
        return points

    def target_positions(self, landmarks: np.ndarray) -> np.ndarray:
        points = np.asarray(landmarks, dtype=np.float64)
        if points.shape != (CANONICAL_LANDMARK_COUNT, 3):
            raise ValueError(
                f"expected canonical landmarks shape {(CANONICAL_LANDMARK_COUNT, 3)}, "
                f"got {points.shape}"
            )
        if not np.isfinite(points).all():
            raise ValueError("canonical landmarks contain NaN or infinity")
        human_scale = palm_scale(points)
        if human_scale < 1e-6:
            raise ValueError("invalid MANUS palm geometry")
        human_frame = orthonormal_palm_frame(points - points[0])
        centered = points - finger_base_centroid(points)
        transformed = (
            (centered @ human_frame)
            * (self.robot_scale / human_scale)
            @ self.robot_frame.T
            + self.robot_centroid
        )
        for finger, chain in CANONICAL_FINGERS.items():
            segments = [
                transformed[chain[index + 1]] - transformed[chain[index]]
                for index in range(3)
            ]
            human_length = sum(
                float(np.linalg.norm(segment)) for segment in segments
            )
            if human_length < 1e-8:
                continue
            ratio = self.robot_finger_lengths[finger] / human_length
            cursor = transformed[chain[0]].copy()
            for index, segment in enumerate(segments):
                cursor = cursor + segment * ratio
                transformed[chain[index + 1]] = cursor
        return transformed

    def retarget(
        self, landmarks: np.ndarray
    ) -> tuple[np.ndarray, dict[str, float | int | bool]]:
        wanted = self.target_positions(landmarks)
        start = np.clip(self.last_qpos, self.lower, self.upper)
        solution = start.copy()
        total_loss = 0.0
        iterations = 0
        evaluations = 0
        success = True
        for finger in ("index", "middle", "ring", "pinky", "thumb"):
            indices = self._joint_indices[finger]
            frames = self._frames[finger]
            targets = wanted[list(CANONICAL_FINGERS[finger])]
            anchor = start[indices]
            q_work = solution.copy()

            def objective(finger_q: np.ndarray) -> tuple[float, np.ndarray]:
                q_work[indices] = finger_q
                self._update(q_work, jacobians=True)
                loss = 0.0
                gradient = np.zeros(len(indices), dtype=np.float64)
                for frame_id, target, weight in zip(
                    frames, targets, CHAIN_WEIGHTS, strict=True
                ):
                    residual = self.data.oMf[frame_id].translation - target
                    jacobian = pin.getFrameJacobian(
                        self.model,
                        self.data,
                        frame_id,
                        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
                    )[:3, indices]
                    loss += weight * float(residual @ residual)
                    gradient += 2.0 * weight * (jacobian.T @ residual)
                delta = finger_q - anchor
                loss += self.smooth_weight * float(delta @ delta)
                gradient += 2.0 * self.smooth_weight * delta
                return loss, gradient

            result = minimize(
                objective,
                start[indices],
                method="L-BFGS-B",
                jac=True,
                bounds=list(zip(self.lower[indices], self.upper[indices])),
                options={
                    "maxiter": self.max_iterations,
                    "ftol": 1e-15,
                    "gtol": 1e-10,
                },
            )
            solution[indices] = np.clip(
                result.x, self.lower[indices], self.upper[indices]
            )
            total_loss += float(result.fun)
            iterations += int(result.nit)
            evaluations += int(result.nfev)
            success = success and bool(result.success)
        self.last_qpos = solution
        if self.filtered_qpos is None:
            self.filtered_qpos = solution.copy()
        else:
            self.filtered_qpos += self.filter_alpha * (
                solution - self.filtered_qpos
            )
        return self.filtered_qpos.copy(), {
            "success": success,
            "loss": total_loss,
            "iterations": iterations,
            "function_evaluations": evaluations,
        }

    def reset(self) -> None:
        self.last_qpos = np.clip(np.zeros(self.model.nq), self.lower, self.upper)
        self.filtered_qpos = None

    def close(self) -> None:
        """Pinocchio owns no external resource."""
