from collections.abc import Mapping

import numpy as np
import pinocchio as pin
import pink
from pink import solve_ik
from pink.tasks import DampingTask, FrameTask, PostureTask
from qpsolvers.exceptions import QPError

from .paths import URDF_PATH
from .types import Pose, SIDES


END_EFFECTOR_FRAMES = {
    "left": "left_fr3v2_link7",
    "right": "right_fr3v2_link7",
}


class IKError(RuntimeError):
    pass


class BimanualPinkIK:
    def __init__(self, dt: float, max_joint_speed: float) -> None:
        if dt <= 0.0 or max_joint_speed <= 0.0:
            raise ValueError("IK timestep and joint speed must be positive")
        self.dt = float(dt)
        self.max_joint_speed = float(max_joint_speed)
        self.model = pin.buildModelFromUrdf(str(URDF_PATH))
        self.data = self.model.createData()
        self.configuration = pink.Configuration(
            self.model,
            self.data,
            pin.neutral(self.model),
        )
        self.frame_tasks = {
            side: FrameTask(
                frame,
                position_cost=100.0,
                orientation_cost=20.0,
            )
            for side, frame in END_EFFECTOR_FRAMES.items()
        }
        # A fixed posture reference gives the null space somewhere to go. With a
        # 7-DoF arm every end-effector pose has a one-parameter family of elbow
        # configurations, and with no attractor the elbow random-walks: measured
        # on closed end-effector loops, q drifted 0.88 rad in one loop while the
        # end effector returned to within 0.00 mm. Drifted configurations end up
        # near joint limits where some directions stop responding, which the
        # operator experiences as ambiguity. The cost is far below the frame
        # tasks' so tracking stays practically exact; the reference defaults to
        # the first configuration seen and is normally the captured hardware
        # home. The cost was swept: 0.2 is too weak to pull the elbow back and
        # drift reached 2.3 rad, while 1.0 returned the configuration to within
        # 0.000 rad after ten adversarial loops at a worst-case tracking cost of
        # 1.5 mm at a 30 cm displacement; 3.0 already costs 12 mm. The same
        # equilibrium holds a 90-degree orientation step to within 0.875 deg,
        # measured, against 0.232 deg at cost 0.5 whose drift protection fails.
        # The attractor is a constant pull, so unlike the speed limit and the
        # damping task it does trade a sub-perceptual amount of steady-state
        # accuracy for a bounded elbow.
        self.posture_task = PostureTask(cost=1.0)
        self.posture_reference: np.ndarray | None = None
        self.damping_task = DampingTask(cost=10.0)
        self.joint_names = tuple(str(name) for name in self.model.names[1:])
        expected = tuple(
            f"{side}_fr3v2_joint{index}"
            for side in SIDES
            for index in range(1, 8)
        )
        if self.joint_names != expected:
            raise ValueError(f"Unexpected URDF joint order: {self.joint_names}")

    def set_posture_reference(self, q: np.ndarray) -> None:
        """Anchor the null-space attractor, normally at the hardware home."""
        values = np.asarray(q, dtype=float)
        if values.shape != (self.model.nq,) or not np.all(np.isfinite(values)):
            raise ValueError(f"Expected {self.model.nq} finite joint positions")
        self.posture_reference = np.clip(
            values,
            self.model.lowerPositionLimit,
            self.model.upperPositionLimit,
        )

    def update(self, q: np.ndarray) -> None:
        values = np.asarray(q, dtype=float)
        if values.shape != (self.model.nq,) or not np.all(np.isfinite(values)):
            raise ValueError(f"Expected {self.model.nq} finite joint positions")
        # MuJoCo/physics can drift a few ulps past joint limits; Pink rejects that.
        values = np.clip(
            values,
            self.model.lowerPositionLimit,
            self.model.upperPositionLimit,
        )
        self.configuration.update(values)

    def frame_pose(self, q: np.ndarray, side: str) -> Pose:
        if side not in END_EFFECTOR_FRAMES:
            raise ValueError(f"Unknown side: {side}")
        self.update(q)
        transform = self.configuration.get_transform_frame_to_world(
            END_EFFECTOR_FRAMES[side]
        )
        return Pose(transform.translation, transform.rotation)

    def step(self, q: np.ndarray, targets: Mapping[str, Pose]) -> np.ndarray:
        unknown = set(targets).difference(SIDES)
        if unknown:
            raise ValueError(f"Unknown target sides: {sorted(unknown)}")
        self.update(q)
        if not targets:
            return self.configuration.q.copy()

        if self.posture_reference is None:
            self.posture_reference = self.configuration.q.copy()
        self.posture_task.set_target(self.posture_reference)
        tasks = [self.posture_task, self.damping_task]
        for side, target in targets.items():
            self.frame_tasks[side].set_target(
                pin.SE3(target.rotation, target.position)
            )
            tasks.append(self.frame_tasks[side])
        try:
            velocity = solve_ik(
                self.configuration,
                tasks,
                self.dt,
                solver="quadprog",
                safety_break=True,
            )
        except (QPError, AssertionError) as exc:
            raise IKError(f"Pink failed to solve the bimanual target: {exc}") from exc
        velocity = np.clip(velocity, -self.max_joint_speed, self.max_joint_speed)
        result = pin.integrate(self.model, self.configuration.q, velocity * self.dt)
        return np.clip(
            result,
            self.model.lowerPositionLimit,
            self.model.upperPositionLimit,
        )
