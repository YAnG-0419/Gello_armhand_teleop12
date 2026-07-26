"""Retarget canonical hand landmarks to a LinkerHand L20 URDF pose.

Pinocchio solves the 16 physical actuators and expands the five URDF mimic
joints into the vendor-facing 21-name packet. Ordinary fingers use independent
Cartesian landmark objectives. The heterogeneous thumb fixes its coupled flex
actuator first, then solves CMC orientation from position, segment direction,
local-frame, and activated fingertip-distance terms.

Each optimization changes one finger block. The thumb runs last so proximity
terms use the final ordinary-finger positions.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree

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

# Constraint weights follow somehand's L20 vector-retargeting configuration.
# They are applied only to the thumb solve; the four ordinary fingers retain
# the faster Cartesian landmark objective.
THUMB_VECTOR_WEIGHTS = (1.0, 1.0, 0.9)
THUMB_FRAME_WEIGHTS = (2.0, 1.8)
THUMB_DISTANCE_WEIGHTS = (2000.0, 1500.0, 1000.0, 800.0)
THUMB_DISTANCE_THRESHOLD = 0.04

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


def _orthonormal_axes(
    primary_vector: np.ndarray,
    secondary_vector: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    primary = _normalize(primary_vector, "thumb-frame primary vector")
    rejected = secondary_vector - primary * np.dot(secondary_vector, primary)
    secondary = _normalize(rejected, "thumb-frame secondary vector")
    return primary, secondary


def chain_bend_angle(points: np.ndarray) -> float:
    """Return total unsigned bend across a four-landmark finger chain.

    A straight chain is zero. For the thumb this is the sum of its anatomical
    MCP and IP bend. Angles are invariant to hand pose, scale, and segment
    length, which makes this a better correspondence for the G20's single
    coupled thumb-flexion actuator than forcing unlike human and robot thumb
    linkages to reach identical Cartesian points.
    """
    chain = np.asarray(points, dtype=np.float64)
    if chain.shape != (4, 3):
        raise ValueError(f"Expected a four-point finger chain, got {chain.shape}")
    segments = np.diff(chain, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    if np.any(lengths < 1e-8):
        raise ValueError("Cannot measure finger bend from a zero-length segment")
    directions = segments / lengths[:, None]
    cosines = np.clip(
        np.sum(directions[:-1] * directions[1:], axis=1), -1.0, 1.0
    )
    return float(np.sum(np.arccos(cosines)))


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
        filter_alpha: float = 0.7,
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

        # Keep the established 21-name packet contract and its limits, but solve
        # the actual 16-DoF mechanism. Without mimic=True Pinocchio treats each
        # declared follower as independent. In practice that let the optimizer
        # put nearly all thumb curl into the distal joint while leaving MCP
        # almost straight; the G20 has only one motor for that pair, so the
        # mapper's normalized average then cut the physical curl roughly in half.
        packet_model = pin.buildModelFromUrdf(str(self.urdf_path))
        packet_entries = sorted(
            (packet_model.joints[joint_id].idx_q, packet_model.names[joint_id])
            for joint_id in range(1, packet_model.njoints)
        )
        self.joint_names: list[str] = [name for _, name in packet_entries]
        self.lower = np.asarray(
            packet_model.lowerPositionLimit, dtype=np.float64
        ).copy()
        self.upper = np.asarray(
            packet_model.upperPositionLimit, dtype=np.float64
        ).copy()

        self.model = pin.buildModelFromUrdf(str(self.urdf_path), True)
        active_entries = []
        for joint_id in range(1, self.model.njoints):
            joint = self.model.joints[joint_id]
            if joint.nq == 0 and joint.nv == 0:
                continue
            if joint.nq != 1 or joint.nv != 1:
                raise ValueError(
                    f"Expected single-DoF revolute L20 joints, got nq={joint.nq} "
                    f"for {self.model.names[joint_id]}"
                )
            active_entries.append((joint.idx_q, self.model.names[joint_id]))
        active_entries.sort()
        self._active_joint_names = [name for _, name in active_entries]
        self._active_output_indices = np.asarray(
            [self.joint_names.index(name) for name in self._active_joint_names],
            dtype=int,
        )
        self._active_lower = np.asarray(
            self.model.lowerPositionLimit, dtype=np.float64
        )
        self._active_upper = np.asarray(
            self.model.upperPositionLimit, dtype=np.float64
        )

        root = ElementTree.parse(self.urdf_path).getroot()
        self._mimics: dict[str, tuple[str, float, float]] = {}
        for element in root.findall("joint"):
            mimic = element.find("mimic")
            if mimic is None:
                continue
            self._mimics[element.attrib["name"]] = (
                mimic.attrib["joint"],
                float(mimic.attrib.get("multiplier", "1")),
                float(mimic.attrib.get("offset", "0")),
            )
        thumb_distal = "thumb_ip" if side == "left" else "thumb_dip"
        thumb_source, self._thumb_mimic_multiplier, thumb_offset = self._mimics[
            thumb_distal
        ]
        if thumb_source != "thumb_mcp" or thumb_offset != 0.0:
            raise ValueError(
                "Expected the distal thumb joint to mimic thumb_mcp with zero offset"
            )
        self._thumb_mcp_index = self._active_joint_names.index("thumb_mcp")

        self._finger_joints: dict[str, np.ndarray] = {}
        for finger in CANONICAL_FINGERS:
            indices = [
                i
                for i, name in enumerate(self._active_joint_names)
                if name.startswith(finger)
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
        self._frame_by_landmark = {
            target.landmark_index: target.frame_id for target in self.targets
        }

        self.smooth_weight = float(smooth_weight)
        self.filter_alpha = float(filter_alpha)
        self.max_iterations = int(max_iterations)
        self.last_qpos = np.clip(
            np.zeros(self.model.nq), self._active_lower, self._active_upper
        )
        self.filtered_qpos: np.ndarray | None = None
        self._q_current = self.last_qpos.copy()

        self.robot_finger_lengths = self._robot_finger_lengths()
        (
            self._thumb_bend_samples,
            self._thumb_q_samples,
        ) = self._build_thumb_bend_lookup()
        (
            self._thumb_frame_id,
            self._thumb_local_primary_axis,
            self._thumb_local_secondary_axis,
        ) = self._build_thumb_frame_reference()
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
        values = np.asarray(list(qpos), dtype=np.float64)
        if values.shape == (self.dof,):
            values = values[self._active_output_indices]
        elif values.shape != (self.model.nq,):
            raise ValueError(
                f"Expected qpos shape {(self.dof,)} or {(self.model.nq,)}, "
                f"got {values.shape}"
            )
        self._q_current = values.copy()

    def _expand_qpos(self, active_qpos: np.ndarray) -> np.ndarray:
        """Expand the 16 actuated coordinates into the stable 21-name packet."""
        output = np.zeros(self.dof, dtype=np.float64)
        output[self._active_output_indices] = active_qpos
        by_name = dict(zip(self.joint_names, output))
        for name, (source, multiplier, offset) in self._mimics.items():
            value = multiplier * by_name[source] + offset
            output[self.joint_names.index(name)] = value
            by_name[name] = value
        return np.clip(output, self.lower, self.upper)

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

    def _build_thumb_bend_lookup(self) -> tuple[np.ndarray, np.ndarray]:
        """Tabulate the robot thumb's observable bend versus its one actuator."""
        q_samples = np.linspace(
            self._active_lower[self._thumb_mcp_index],
            self._active_upper[self._thumb_mcp_index],
            257,
        )
        bends = []
        qpos = np.zeros(self.model.nq, dtype=np.float64)
        chain = list(CANONICAL_FINGERS["thumb"])
        for value in q_samples:
            qpos[self._thumb_mcp_index] = value
            self._reset_joints(qpos)
            bends.append(chain_bend_angle(self.robot_landmarks()[chain]))

        # Near the straight mechanical stop, fixed link-frame offsets can make
        # the unsigned angle dip slightly before it rises. The physical command
        # still has one direction, so use its monotonic envelope for inversion.
        monotonic = np.maximum.accumulate(np.asarray(bends, dtype=np.float64))
        keep = np.concatenate(([True], np.diff(monotonic) > 1e-10))
        self._reset_joints(self.last_qpos)
        return monotonic[keep], q_samples[keep]

    def _thumb_q_for_bend(self, bend: float) -> float:
        """Invert the robot-specific thumb bend curve with endpoint clipping."""
        return float(
            np.interp(
                bend,
                self._thumb_bend_samples,
                self._thumb_q_samples,
                left=self._thumb_q_samples[0],
                right=self._thumb_q_samples[-1],
            )
        )

    def _build_thumb_frame_reference(
        self,
    ) -> tuple[int, np.ndarray, np.ndarray]:
        """Build the neutral robot-local thumb frame used by somehand."""
        self._reset_joints(np.zeros(self.model.nq, dtype=np.float64))
        points = self.robot_landmarks()
        frame_id = self._frame_by_landmark[1]
        rotation = self.data.oMf[frame_id].rotation
        primary, secondary = _orthonormal_axes(
            points[2] - points[1],
            points[5] - points[1],
        )
        self._reset_joints(self.last_qpos)
        return (
            frame_id,
            rotation.T @ primary,
            rotation.T @ secondary,
        )

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
        raw_landmarks = np.asarray(landmarks, dtype=np.float64)
        wanted_by_landmark = {
            target.landmark_index: target_positions[index]
            for index, target in enumerate(self.targets)
        }
        start = np.clip(self.last_qpos, self._active_lower, self._active_upper)
        solution = start.copy()
        total_loss = 0.0
        total_iterations = 0
        total_evaluations = 0
        success = True

        # Determine and fix the physical flex actuator before solving CMC
        # orientation, so the CMC solution remains consistent with emitted MCP.
        thumb_chain = np.asarray(landmarks, dtype=np.float64)[
            list(CANONICAL_FINGERS["thumb"])
        ]
        thumb_bend = chain_bend_angle(thumb_chain)
        thumb_flex = np.clip(
            self._thumb_q_for_bend(thumb_bend),
            self._active_lower[self._thumb_mcp_index],
            self._active_upper[self._thumb_mcp_index],
        )
        thumb_flex_fraction = float(
            (thumb_flex - self._active_lower[self._thumb_mcp_index])
            / (
                self._active_upper[self._thumb_mcp_index]
                - self._active_lower[self._thumb_mcp_index]
            )
        )
        solution[self._thumb_mcp_index] = thumb_flex

        # Solve the ordinary fingers first. The thumb then sees their final tip
        # locations for the activated thumb-to-fingertip distance constraints.
        solve_order = ("index", "middle", "ring", "pinky", "thumb")
        for finger in solve_order:
            all_joint_indices = self._finger_joints[finger]
            joint_indices = (
                all_joint_indices[all_joint_indices != self._thumb_mcp_index]
                if finger == "thumb"
                else all_joint_indices
            )
            target_indices = self._targets_by_finger[finger]
            frame_ids = [self.targets[t].frame_id for t in target_indices]
            weights = np.asarray(
                [self.targets[t].weight for t in target_indices], dtype=np.float64
            )
            wanted = target_positions[target_indices]
            anchor = self.last_qpos[joint_indices]
            q_work = solution.copy()

            thumb_direction_constraints = []
            thumb_frame_targets = None
            thumb_distance_constraints = []
            thumb_orientation_activation = 0.0
            if finger == "thumb":
                thumb_landmarks = CANONICAL_FINGERS["thumb"]
                for pair, weight in zip(
                    zip(thumb_landmarks[:-1], thumb_landmarks[1:]),
                    THUMB_VECTOR_WEIGHTS,
                ):
                    origin_landmark, target_landmark = pair
                    desired = _normalize(
                        wanted_by_landmark[target_landmark]
                        - wanted_by_landmark[origin_landmark],
                        f"thumb segment {origin_landmark}->{target_landmark}",
                    )
                    thumb_direction_constraints.append(
                        (
                            self._frame_by_landmark[origin_landmark],
                            self._frame_by_landmark[target_landmark],
                            desired,
                            weight,
                        )
                    )

                thumb_frame_targets = _orthonormal_axes(
                    wanted_by_landmark[2] - wanted_by_landmark[1],
                    wanted_by_landmark[5] - wanted_by_landmark[1],
                )
                middle_chain = CANONICAL_FINGERS["middle"]
                human_middle_length = sum(
                    float(
                        np.linalg.norm(
                            raw_landmarks[middle_chain[index + 1]]
                            - raw_landmarks[middle_chain[index]]
                        )
                    )
                    for index in range(3)
                )
                thumb_distance_scale = (
                    self.robot_finger_lengths["middle"]
                    / max(human_middle_length, 1e-8)
                )
                proximity_activation = 0.0
                for tip_landmark, distance_weight in zip(
                    (8, 12, 16, 20),
                    THUMB_DISTANCE_WEIGHTS,
                ):
                    raw_distance = float(
                        np.linalg.norm(
                            raw_landmarks[4] - raw_landmarks[tip_landmark]
                        )
                    )
                    desired_distance = raw_distance * thumb_distance_scale
                    activation = max(
                        0.0,
                        1.0 - raw_distance / THUMB_DISTANCE_THRESHOLD,
                    )
                    proximity_activation = max(proximity_activation, activation)
                    thumb_distance_constraints.append(
                        (
                            self._frame_by_landmark[4],
                            self._frame_by_landmark[tip_landmark],
                            desired_distance,
                            distance_weight * activation,
                        )
                    )
                # The somehand direction/frame terms are useful during curl and
                # opposition, but applying them at full weight to an extended
                # heterogeneous thumb makes CMC pitch compensate for morphology
                # and visibly bends the physical thumb at rest. Fade them in
                # with actual flexion, while a close fingertip activates them
                # immediately for pinch.
                thumb_orientation_activation = max(
                    thumb_flex_fraction * thumb_flex_fraction,
                    proximity_activation,
                )

            def objective(finger_q: np.ndarray) -> tuple[float, np.ndarray]:
                q_work[joint_indices] = finger_q
                if finger == "thumb":
                    q_work[self._thumb_mcp_index] = thumb_flex
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

                if finger == "thumb":
                    # Match the three segment directions. Direction objectives
                    # transfer articulation without assuming that human and
                    # robot thumb link lengths or joint locations coincide.
                    for (
                        origin_id,
                        target_id,
                        desired_direction,
                        direction_weight,
                    ) in thumb_direction_constraints:
                        vector = (
                            self.data.oMf[target_id].translation
                            - self.data.oMf[origin_id].translation
                        )
                        length = float(np.linalg.norm(vector))
                        if length < 1e-8:
                            continue
                        direction = vector / length
                        cosine = float(np.dot(direction, desired_direction))
                        effective_weight = (
                            direction_weight * thumb_orientation_activation
                        )
                        loss += effective_weight * (1.0 - cosine)
                        jac_target = pin.getFrameJacobian(
                            self.model,
                            self.data,
                            target_id,
                            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
                        )[:3, joint_indices]
                        jac_origin = pin.getFrameJacobian(
                            self.model,
                            self.data,
                            origin_id,
                            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
                        )[:3, joint_indices]
                        derivative = -(
                            desired_direction - cosine * direction
                        ) / length
                        gradient += effective_weight * (
                            derivative @ (jac_target - jac_origin)
                        )

                    # Match the thumb-metacarpal local frame using the same
                    # primary/secondary construction as somehand.
                    desired_primary, desired_secondary = thumb_frame_targets
                    rotation = self.data.oMf[self._thumb_frame_id].rotation
                    angular_jacobian = pin.getFrameJacobian(
                        self.model,
                        self.data,
                        self._thumb_frame_id,
                        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
                    )[3:, joint_indices]
                    for local_axis, desired_axis, frame_weight in (
                        (
                            self._thumb_local_primary_axis,
                            desired_primary,
                            THUMB_FRAME_WEIGHTS[0],
                        ),
                        (
                            self._thumb_local_secondary_axis,
                            desired_secondary,
                            THUMB_FRAME_WEIGHTS[1],
                        ),
                    ):
                        axis = rotation @ local_axis
                        cosine = float(np.dot(axis, desired_axis))
                        effective_weight = (
                            frame_weight * thumb_orientation_activation
                        )
                        loss += effective_weight * (1.0 - cosine)
                        axis_jacobian = np.cross(angular_jacobian.T, axis).T
                        gradient += -effective_weight * (
                            desired_axis @ axis_jacobian
                        )

                    # When the human thumb approaches a fingertip, penalize
                    # only robot excess distance. Open-hand gestures remain
                    # unaffected, while pinch/opposition receives a strong cue.
                    for (
                        thumb_tip_id,
                        finger_tip_id,
                        desired_distance,
                        distance_weight,
                    ) in thumb_distance_constraints:
                        if distance_weight <= 0.0:
                            continue
                        vector = (
                            self.data.oMf[finger_tip_id].translation
                            - self.data.oMf[thumb_tip_id].translation
                        )
                        distance = float(np.linalg.norm(vector))
                        if distance < 1e-8:
                            continue
                        excess = distance - desired_distance
                        if excess <= 0.0:
                            continue
                        loss += distance_weight * excess * excess
                        jac_thumb = pin.getFrameJacobian(
                            self.model,
                            self.data,
                            thumb_tip_id,
                            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
                        )[:3, joint_indices]
                        jac_finger = pin.getFrameJacobian(
                            self.model,
                            self.data,
                            finger_tip_id,
                            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
                        )[:3, joint_indices]
                        gradient += (
                            2.0
                            * distance_weight
                            * excess
                            * ((vector / distance) @ (jac_finger - jac_thumb))
                        )
                delta = finger_q - anchor
                loss += self.smooth_weight * float(delta @ delta)
                gradient += 2.0 * self.smooth_weight * delta
                return loss, gradient

            result = minimize(
                objective,
                start[joint_indices],
                method="L-BFGS-B",
                jac=True,
                bounds=list(
                    zip(
                        self._active_lower[joint_indices],
                        self._active_upper[joint_indices],
                    )
                ),
                options={
                    "maxiter": (
                        max(40, self.max_iterations)
                        if finger == "thumb"
                        else self.max_iterations
                    ),
                    "ftol": 1e-9,
                    "gtol": 1e-6,
                },
            )
            solution[joint_indices] = np.clip(
                result.x,
                self._active_lower[joint_indices],
                self._active_upper[joint_indices],
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

        # Report physical, interpretable thumb fidelity at the pose that will
        # actually be emitted (after gesture filtering). Aggregate optimizer
        # loss mixes unlike weighted objectives and cannot say whether an
        # observed mismatch is flexion, CMC orientation, reach, or opposition.
        robot_points = self.robot_landmarks()
        thumb_indices = CANONICAL_FINGERS["thumb"]
        thumb_position_errors = np.asarray(
            [
                np.linalg.norm(
                    robot_points[index] - wanted_by_landmark[index]
                )
                for index in thumb_indices
            ],
            dtype=np.float64,
        )
        thumb_direction_errors = []
        for origin, target in zip(thumb_indices[:-1], thumb_indices[1:]):
            actual = _normalize(
                robot_points[target] - robot_points[origin],
                f"robot thumb segment {origin}->{target}",
            )
            desired = _normalize(
                wanted_by_landmark[target] - wanted_by_landmark[origin],
                f"target thumb segment {origin}->{target}",
            )
            thumb_direction_errors.append(
                np.degrees(np.arccos(np.clip(np.dot(actual, desired), -1.0, 1.0)))
            )
        robot_thumb_bend = chain_bend_angle(
            robot_points[list(thumb_indices)]
        )

        stats: dict[str, float | int | bool] = {
            "success": success,
            "loss": total_loss,
            "iterations": total_iterations,
            "function_evaluations": total_evaluations,
            "thumb_bend": thumb_bend,
            "thumb_robot_bend": robot_thumb_bend,
            "thumb_bend_error": robot_thumb_bend - thumb_bend,
            "thumb_flex_target": float(thumb_flex),
            "thumb_flex_emitted": float(
                self.filtered_qpos[self._thumb_mcp_index]
            ),
            "thumb_position_rmse": float(
                np.sqrt(np.mean(thumb_position_errors**2))
            ),
            "thumb_tip_error": float(thumb_position_errors[-1]),
            "thumb_direction_error_deg": float(
                np.mean(thumb_direction_errors)
            ),
            "thumb_orientation_activation": thumb_orientation_activation,
        }
        for finger, tip_landmark in zip(
            ("index", "middle", "ring", "pinky"), (8, 12, 16, 20)
        ):
            desired_distance = float(
                np.linalg.norm(
                    wanted_by_landmark[4] - wanted_by_landmark[tip_landmark]
                )
            )
            robot_distance = float(
                np.linalg.norm(
                    robot_points[4] - robot_points[tip_landmark]
                )
            )
            stats[f"thumb_{finger}_distance_error"] = (
                robot_distance - desired_distance
            )
        return self._expand_qpos(self.filtered_qpos), stats

    # ------------------------------------------------------------------ state
    def set_qpos(self, qpos: np.ndarray) -> None:
        values = np.asarray(qpos, dtype=np.float64)
        if values.shape != (self.dof,):
            raise ValueError(f"Expected qpos shape {(self.dof,)}, got {values.shape}")
        values = np.clip(values, self.lower, self.upper)
        active = values[self._active_output_indices]
        self.last_qpos = active.copy()
        self.filtered_qpos = active.copy()
        self._q_current = active.copy()

    def reset(self) -> None:
        """Forget filter and warm-start history after a tracking dropout."""
        self.last_qpos = np.clip(
            np.zeros(self.model.nq), self._active_lower, self._active_upper
        )
        self.filtered_qpos = None
        self._q_current = self.last_qpos.copy()

    def close(self) -> None:
        """Kept for API compatibility; pinocchio holds no external resources."""

    def __enter__(self) -> "L20Retargeter":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
