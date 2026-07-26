import threading
import time

import numpy as np

from .config import InputConfig
from .ik import BimanualPinkIK, IKError
from .pose_mapping import RelativePoseMapper
from .robot_udp import UdpRobotBackend
from .types import SIDES
from .xr_input import create_pico_input


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
        control_rate: float,
        max_joint_speed: float,
        robot_state_wait_timeout: float,
        input_config: InputConfig,
        input_type: str,
        hand_sender_factory=None,
        debug_logger=None,
        reset_invoker=None,
    ) -> None:
        self.dt = 1.0 / control_rate
        self.robot_state_wait_timeout = robot_state_wait_timeout
        self.robot = UdpRobotBackend(
            command_host=command_host,
            command_port=command_port,
            state_host=state_host,
            state_port=state_port,
            state_timeout=state_timeout,
        )
        try:
            self.teleop_input = create_pico_input(input_config, input_type)
        except BaseException:
            self.robot.close()
            raise
        self.ik = BimanualPinkIK(dt=self.dt, max_joint_speed=max_joint_speed)
        self.mappers = {
            side: RelativePoseMapper(
                translation_scale=translation_scale,
                rotation_scale=rotation_scale,
            )
            for side in SIDES
        }
        self.hold_q: np.ndarray | None = None

        # The hand pipeline shares this process's single SDK client and is ticked
        # synchronously from the loop below: a solve costs about 1.5 ms and the
        # pipeline runs at most one per tick, so it fits the arm's 10 ms budget.
        # Constructing it must never prevent the arms from running.
        self.hands = None
        if hand_sender_factory is not None:
            self.hands = hand_sender_factory(self.teleop_input.xrt)
        self.debug_logger = debug_logger
        # Reset to the captured initial pose, requested from the keyboard. The
        # operator process is deliberately ROS-free, so the reset is delegated
        # to a blocking callable (a `ros2 service call` in the container) run on
        # a worker thread; the worker only appends to `reset_outcome`, and every
        # other state change stays on the control thread.
        self.reset_invoker = reset_invoker
        self.reset_thread: threading.Thread | None = None
        self.reset_outcome: list[tuple[bool, str]] = []

    def _start_reset(self) -> None:
        if self.reset_invoker is None:
            print("reset requested, but no reset command is configured")
            return
        if self.reset_thread is not None:
            print("reset already in progress")
            return
        self.teleop_input.disable_all("resetting to initial pose")
        for mapper in self.mappers.values():
            mapper.reset()
        if self.hands is not None:
            self.hands.request_open()
        print(
            "reset: moving arms to the initial pose"
            + ("; opening hands" if self.hands is not None else "")
        )

        def worker() -> None:
            try:
                outcome = self.reset_invoker()
            except Exception as error:  # noqa: BLE001 - report, never crash the loop
                outcome = (False, str(error))
            self.reset_outcome.append(outcome)

        self.reset_thread = threading.Thread(target=worker, daemon=True)
        self.reset_thread.start()

    def _service_reset(self) -> None:
        """Fold a finished reset back into the loop, on the control thread."""
        if self.reset_thread is None or self.reset_thread.is_alive():
            return
        self.reset_thread.join()
        self.reset_thread = None
        succeeded, message = (
            self.reset_outcome.pop() if self.reset_outcome else (False, "no result")
        )
        print(f"reset {'done' if succeeded else 'FAILED'}: {message}")
        # The arms are wherever the reset left them, so the pre-reset hold_q is
        # a lie. Dropping it makes the loop re-seed from measured state and
        # re-anchor the IK posture reference before anything can re-engage.
        self.hold_q = None

    def run(self) -> None:
        try:
            self.robot.wait_for_state(timeout=self.robot_state_wait_timeout)
            while True:
                started_at = time.monotonic()
                self._service_reset()
                q = self.robot.receive_state()
                if q is None:
                    self.teleop_input.disable_all(
                        "robot state missing or stale"
                    )
                    for mapper in self.mappers.values():
                        mapper.reset()
                    self.robot.send_command(
                        self.hold_q if self.hold_q is not None else self.ik.configuration.q,
                        (),
                    )
                    # The arms are disengaged, so the hands stop following too;
                    # a pending open request still streams.
                    if self.hands is not None:
                        self.hands.tick(active={side: False for side in SIDES})
                    time.sleep(self.dt)
                    continue
                if self.hold_q is None:
                    self.hold_q = np.asarray(q, dtype=float).copy()
                    # Anchor the IK null-space attractor at the pose the session
                    # started from, normally the captured hardware home.
                    self.ik.set_posture_reference(self.hold_q)
                sample = self.teleop_input.sample()
                take_requests = getattr(self.teleop_input, "take_requests", None)
                requests = take_requests() if take_requests is not None else {}
                if requests.get("open_hands"):
                    if self.hands is not None:
                        self.hands.request_open()
                        print("hands: opening (sides not currently following)")
                    else:
                        print("hands: not running, start with --hands")
                if requests.get("reset"):
                    self._start_reset()
                if self.reset_thread is not None:
                    # The reset trajectory owns the arms; nothing may engage,
                    # and this tick's sample must not act on stale activations.
                    self.teleop_input.disable_all("reset in progress")
                    sample = None
                targets = {}
                current_poses = self.ik.frame_poses(self.hold_q)
                for side in SIDES:
                    current = current_poses[side]
                    if sample is None:
                        self.mappers[side].update(current, False, current)
                        continue
                    target = self.mappers[side].update(
                        sample.poses[side],
                        sample.activations[side],
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
                if self.debug_logger is not None:
                    raw_pose_reader = getattr(
                        self.teleop_input, "debug_raw_poses", None
                    )
                    raw_poses = (
                        raw_pose_reader() if raw_pose_reader is not None else {}
                    )
                    self.debug_logger.record(
                        time.monotonic(),
                        q,
                        self.hold_q,
                        {} if sample is None else sample.poses,
                        {} if sample is None else sample.activations,
                        targets,
                        self.ik.frame_poses(self.hold_q),
                        raw_tracker_poses=raw_poses,
                        measured_ee_poses=self.ik.frame_poses(q),
                    )
                # Hands go after the arm command so the deadline-critical work
                # is never queued behind a hand solve. Each hand follows only
                # while its arm is engaged: one keyboard, one on/off per side.
                if self.hands is not None:
                    self.hands.tick(
                        active=(
                            {side: False for side in SIDES}
                            if sample is None
                            else sample.activations
                        )
                    )
                remaining = self.dt - (time.monotonic() - started_at)
                if remaining > 0.0:
                    time.sleep(remaining)
        finally:
            if self.debug_logger is not None:
                self.debug_logger.close()
            # Close the hand pipeline before the SDK client it reads from.
            try:
                if self.hands is not None:
                    self.hands.close()
            finally:
                try:
                    self.robot.close()
                finally:
                    self.teleop_input.close()
