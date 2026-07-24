import time

import numpy as np

from .ik import BimanualPinkIK, IKError
from .pose_mapping import RelativePoseMapper
from .robot_udp import UdpRobotBackend
from .types import SIDES
from .xr_input import XrInput


class DualFr3HardwareTeleop:
    def __init__(
        self,
        command_host: str,
        command_port: int,
        state_host: str,
        state_port: int,
        state_timeout: float,
        translation_scale: float,
        rotation_scale: float,
        grip_threshold: float,
        control_rate: float,
        max_joint_speed: float,
        xr_ready_timeout: float,
        robot_state_wait_timeout: float,
    ) -> None:
        self.dt = 1.0 / control_rate
        self.xr = XrInput(ready_timeout=xr_ready_timeout)
        self.robot = UdpRobotBackend(
            command_host=command_host,
            command_port=command_port,
            state_host=state_host,
            state_port=state_port,
            state_timeout=state_timeout,
        )
        self.ik = BimanualPinkIK(dt=self.dt, max_joint_speed=max_joint_speed)
        self.mappers = {
            side: RelativePoseMapper(
                translation_scale=translation_scale,
                rotation_scale=rotation_scale,
                grip_threshold=grip_threshold,
            )
            for side in SIDES
        }
        self.hold_q: np.ndarray | None = None

    def run(self) -> None:
        self.robot.wait_for_state(timeout=robot_state_wait_timeout)
        try:
            while True:
                started_at = time.monotonic()
                q = self.robot.receive_state()
                if q is None:
                    self.robot.send_command(
                        self.hold_q if self.hold_q is not None else self.ik.configuration.q,
                        (),
                    )
                    time.sleep(self.dt)
                    continue
                if self.hold_q is None:
                    self.hold_q = np.asarray(q, dtype=float).copy()
                sample = self.xr.sample()
                targets = {}
                for side in SIDES:
                    current = self.ik.frame_pose(self.hold_q, side)
                    if sample is None:
                        self.mappers[side].update(current, 0.0, current)
                        continue
                    target = self.mappers[side].update(
                        sample.poses[side],
                        sample.grips[side],
                        current,
                    )
                    if target is not None:
                        targets[side] = target
                try:
                    if targets:
                        self.hold_q = self.ik.step(self.hold_q, targets)
                except IKError:
                    self.robot.send_command(q, ())
                    raise
                active_sides = tuple(side for side in SIDES if side in targets)
                self.robot.send_command(self.hold_q, active_sides)
                remaining = self.dt - (time.monotonic() - started_at)
                if remaining > 0.0:
                    time.sleep(remaining)
        finally:
            self.robot.close()
            self.xr.close()
