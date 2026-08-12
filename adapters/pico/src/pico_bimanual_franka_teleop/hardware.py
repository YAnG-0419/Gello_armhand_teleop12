import threading
import time
from dataclasses import replace

import numpy as np
from teleop_core.safety import LOWER_LIMITS, UPPER_LIMITS

from .ik import BimanualPinkIK, IKError, classify_step
from .interfaces import ArmPoseSource, HandController, OperatorState
from .joint_mapping import RelativeJointMapper
from .pose_mapping import RelativePoseMapper
from .robot_udp import UdpRobotBackend
from .types import ArmSample, SIDES, TeleopSample


def reseed_inactive_joints(held, measured, activations, mapper_active):
    """Keep every inactive or newly engaging side aligned to hardware."""
    result = np.asarray(held, dtype=float).copy()
    measured = np.asarray(measured, dtype=float)
    for side, joints in (("left", slice(0, 7)), ("right", slice(7, 14))):
        if not activations.get(side, False) or not mapper_active.get(side, False):
            result[joints] = measured[joints]
    return result


def disengage_sample_sides(sample: ArmSample | None, sides) -> ArmSample | None:
    if sample is None:
        return None
    activations = dict(sample.activations)
    for side in sides:
        activations[side] = False
    return replace(sample, activations=activations)


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
        arm_source: ArmPoseSource,
        operator: OperatorState,
        hands: HandController | None = None,
        debug_logger=None,
        reset_invoker=None,
    ) -> None:
        self.arm_source = arm_source
        self.operator = operator
        self._notify = operator.show
        self.dt = 1.0 / control_rate
        self.robot_state_wait_timeout = robot_state_wait_timeout
        try:
            self.robot = UdpRobotBackend(
                command_host=command_host,
                command_port=command_port,
                state_host=state_host,
                state_port=state_port,
                state_timeout=state_timeout,
            )
        except BaseException:
            if hands is not None:
                hands.close()
            arm_source.close()
            raise
        self.ik = BimanualPinkIK(dt=self.dt, max_joint_speed=max_joint_speed)
        self.joint_input = getattr(arm_source, "output_kind", "pose") == "joint"
        if self.joint_input:
            max_delta = float(getattr(arm_source, "max_relative_delta", 0.25))
            max_target_velocity = getattr(
                arm_source, "max_target_velocity", None
            )
            sensitivity_by_side = getattr(arm_source, "joint_sensitivity", {})
            self.mappers = {
                side: RelativeJointMapper(
                    LOWER_LIMITS[index : index + 7],
                    UPPER_LIMITS[index : index + 7],
                    max_delta,
                    joint_sensitivity=sensitivity_by_side.get(
                        side, np.ones(7, dtype=float)
                    ),
                    max_target_velocity=max_target_velocity,
                    nominal_dt=self.dt,
                )
                for side, index in (("left", 0), ("right", 7))
            }
        else:
            self.mappers = {
                side: RelativePoseMapper(
                    translation_scale=translation_scale,
                    rotation_scale=rotation_scale,
                )
                for side in SIDES
            }
        self.hold_q: np.ndarray | None = None

        self.hands = hands
        self.debug_logger = debug_logger
        # Reset to the captured initial pose, requested from the keyboard. The
        # operator process is deliberately ROS-free, so the reset is delegated
        # to a blocking callable (a `ros2 service call` in the container) run on
        # a worker thread; the worker only appends to `reset_outcome`, and every
        # other state change stays on the control thread.
        self.reset_invoker = reset_invoker
        self.reset_thread: threading.Thread | None = None
        self.reset_outcome: list[tuple[bool, str]] = []

    def _start_reset(self, side: str | None = None) -> None:
        """Home both arms, or only `side`. Either way the whole session
        disengages for the duration: the reset trajectory owns the command
        bus (the gateway blocks while it is active), so the other arm simply
        holds where it is."""
        if self.reset_invoker is None:
            self._notify("reset requested, but no reset command is configured")
            return
        if self.reset_thread is not None:
            self._notify("reset already in progress")
            return
        self.operator.disable_all("resetting to initial pose")
        for mapper in self.mappers.values():
            mapper.reset()
        scope = f"{side} arm" if side else "arms"
        self._notify(f"reset: moving {scope} to the initial pose")

        def worker() -> None:
            try:
                outcome = self.reset_invoker(side)
            except Exception as error:  # noqa: BLE001 - report, never crash the loop
                outcome = (False, str(error))
            self.reset_outcome.append(outcome)

        self.reset_thread = threading.Thread(target=worker, daemon=True)
        self.reset_thread.start()

    def _open_hands(self, sides: tuple[str, ...]) -> None:
        """Stop selected sides following, then stream their open pose."""
        selected = tuple(side for side in SIDES if side in sides)
        if self.hands is None:
            self._notify("hands: not running, start with --hand-source")
            return
        for side in selected:
            self.operator.deny_hand(side, "opening hand")
        self.hands.request_open(sides=selected)
        self._notify("hands: opening " + "/".join(selected))

    def _service_reset(self) -> None:
        """Fold a finished reset back into the loop, on the control thread."""
        if self.reset_thread is None or self.reset_thread.is_alive():
            return
        self.reset_thread.join()
        self.reset_thread = None
        succeeded, message = (
            self.reset_outcome.pop() if self.reset_outcome else (False, "no result")
        )
        self._notify(f"reset {'done' if succeeded else 'FAILED'}: {message}")
        # The arms are wherever the reset left them, so the pre-reset hold_q is
        # a lie. Dropping it makes the loop re-seed from measured state and
        # re-anchor the IK posture reference before anything can re-engage.
        self.hold_q = None

    @staticmethod
    def _ik_status(worst: dict) -> str:
        """One operator-readable clause per side, worst tick since the last
        report: the IK failure cause, or 'ok' while tracking is transparent."""
        parts = []
        for side in SIDES:
            diagnostics = worst.get(side)
            if diagnostics is None:
                continue
            cause = classify_step(diagnostics)
            if cause == "ok":
                parts.append(f"{side} ok")
                continue
            detail = (
                f"{diagnostics['position_error'] * 1e3:.0f}mm/"
                f"{np.degrees(diagnostics['orientation_error']):.0f}deg"
            )
            if cause == "joint-limit":
                joints = ",".join(
                    f"j{joint}" for joint, _ in diagnostics["limit_joints"]
                )
                parts.append(f"{side} LIMIT {joints} off {detail}")
            elif cause == "speed-clamp":
                parts.append(f"{side} clamped off {detail}")
            else:
                parts.append(f"{side} UNREACHABLE off {detail}")
        return " | ".join(parts)

    def run(self) -> None:
        next_status_report = 0.0
        state_missing_since: float | None = None
        # Worst IK step per side since the last status report; a transient
        # at the report instant must not hide a limit hit seconds earlier.
        ik_worst: dict[str, dict] = {}
        previous_hand_sent = (
            {} if self.hands is None else {side: 0 for side in self.hands.sides}
        )
        status_summary = getattr(self.arm_source, "status_summary", None)
        try:
            self.robot.wait_for_state(timeout=self.robot_state_wait_timeout)
            if self.hands is not None:
                start_hands = getattr(self.hands, "start", None)
                if start_hands is not None:
                    start_hands()
            while True:
                started_at = time.monotonic()
                self._service_reset()
                q = self.robot.receive_state()
                if q is None:
                    reason = "robot state missing or stale"
                    if state_missing_since is None:
                        state_missing_since = started_at
                        self.operator.disable_all(reason)
                    missing_for = started_at - state_missing_since
                    self.operator.set_status(
                        f"FAULT | {reason} for {missing_for:.1f}s"
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
                        self.hands.set_active(
                            {side: False for side in SIDES}
                        )
                    if missing_for >= self.robot_state_wait_timeout:
                        raise TimeoutError(
                            "Lost fresh dual-FR3 state for "
                            f"{missing_for:.1f}s"
                        )
                    time.sleep(self.dt)
                    continue
                state_missing_since = None
                if self.hold_q is None:
                    self.hold_q = np.asarray(q, dtype=float).copy()
                    # Anchor the IK null-space attractor at the pose the session
                    # started from, normally the captured hardware home.
                    self.ik.set_posture_reference(self.hold_q)
                for fault in self.robot.take_gateway_faults():
                    rejected = tuple(
                        side for side in SIDES if fault.startswith(f"{side} ")
                    )
                    if len(rejected) == 1:
                        self.operator.deny(
                            rejected[0], f"safety gateway: {fault}"
                        )
                    else:
                        self.operator.disable_all(
                            f"safety gateway: {fault}"
                        )
                sample = self.arm_source.sample()
                requests = self.operator.take_requests()
                open_sides = {
                    side
                    for side in SIDES
                    if requests.get("open_hands")
                    or requests.get(f"open_{side}_hand")
                }
                if open_sides:
                    self._open_hands(tuple(open_sides))
                if requests.get("reset"):
                    self._start_reset()
                elif requests.get("reset_left"):
                    self._start_reset("left")
                elif requests.get("reset_right"):
                    self._start_reset("right")
                if self.reset_thread is not None:
                    # The reset trajectory owns the arms; nothing may engage,
                    # and this tick's sample must not act on stale activations.
                    self.operator.disable_all("reset in progress")
                    sample = None
                if self.reset_thread is None:
                    activations = (
                        {side: False for side in SIDES}
                        if sample is None
                        else sample.activations
                    )
                    self.hold_q = reseed_inactive_joints(
                        self.hold_q,
                        q,
                        activations,
                        {
                            side: self.mappers[side].active
                            for side in SIDES
                        },
                    )
                targets = {}
                if self.joint_input:
                    for side, joints in (
                        ("left", slice(0, 7)),
                        ("right", slice(7, 14)),
                    ):
                        leader = (
                            np.zeros(7)
                            if sample is None
                            else sample.positions[side]
                        )
                        target = self.mappers[side].update(
                            leader,
                            sample is not None and sample.activations[side],
                            q[joints],
                        )
                        if target is not None:
                            self.hold_q[joints] = target
                            targets[side] = target
                    self.ik.last_diagnostics = {}
                else:
                    current_poses = self.ik.frame_poses(self.hold_q)
                    for side in SIDES:
                        current = current_poses[side]
                        if sample is None:
                            self.mappers[side].update(current, False, current)
                            continue
                        target = self.mappers[side].update(
                            sample.poses[side], sample.activations[side], current
                        )
                        if target is not None:
                            targets[side] = target
                    try:
                        if targets:
                            self.hold_q = self.ik.step(self.hold_q, targets)
                    except IKError:
                        self.robot.send_command(q, ())
                        raise
                    for side, diagnostics in self.ik.last_diagnostics.items():
                        worst = ik_worst.get(side)
                        if worst is None or diagnostics["position_error"] > worst[
                            "position_error"
                        ]:
                            ik_worst[side] = diagnostics
                active_sides = tuple(side for side in SIDES if side in targets)
                self.robot.send_command(self.hold_q, active_sides)
                if self.debug_logger is not None:
                    raw_pose_reader = getattr(
                        self.arm_source, "debug_raw_poses", None
                    )
                    raw_poses = (
                        raw_pose_reader() if raw_pose_reader is not None else {}
                    )
                    feed_reader = getattr(
                        self.arm_source, "debug_feed_state", None
                    )
                    pose_sample = sample if isinstance(sample, TeleopSample) else None
                    pose_targets = {} if self.joint_input else targets
                    self.debug_logger.record(
                        time.monotonic(),
                        q,
                        self.hold_q,
                        {} if pose_sample is None else pose_sample.poses,
                        {} if sample is None else sample.activations,
                        pose_targets,
                        self.ik.frame_poses(self.hold_q),
                        raw_tracker_poses=raw_poses,
                        measured_ee_poses=self.ik.frame_poses(q),
                        ik_diagnostics=self.ik.last_diagnostics,
                        feed_state=(
                            feed_reader() if feed_reader is not None else None
                        ),
                    )
                # Only copy engagement into the independent hand worker here;
                # arm timing never waits for hand I/O or retargeting.
                if self.hands is not None:
                    hand_activation_reader = getattr(
                        self.operator, "poll_hands", self.operator.poll
                    )
                    self.hands.set_active(hand_activation_reader())
                now = time.monotonic()
                if now >= next_status_report:
                    input_summary = (
                        status_summary() if status_summary is not None else ""
                    )
                    source_name = getattr(self.arm_source, "source_name", "trackers")
                    parts = (
                        [f"{source_name}: {input_summary}"] if input_summary else []
                    )
                    ik_summary = self._ik_status(ik_worst)
                    if ik_summary:
                        parts.append(f"ik: {ik_summary}")
                    ik_worst = {}
                    if self.hands is not None:
                        hand_parts = []
                        for side in self.hands.sides:
                            status = self.hands.status.sides[side]
                            sent = status.sent - previous_hand_sent[side]
                            previous_hand_sent[side] = status.sent
                            state = (
                                f"sending {sent:.0f}Hz"
                                if status.sending
                                else f"stopped ({status.fault})"
                            )
                            hand_parts.append(f"{side}={state}")
                        parts.append("hands: " + " | ".join(hand_parts))
                    if parts:
                        line = "STATE | " + " | ".join(parts)
                        self.operator.set_status(line)
                    next_status_report = now + 1.0
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
                    self.arm_source.close()
