"""Single-owner hardware runtime behind the Wuji browser UI."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Mapping, Sequence

import numpy as np

from adapters.wuji.pipeline import WujiHandPipeline, canonical_landmarks

from .models import SIDES, manus_gesture_features


@dataclass(frozen=True)
class HandSnapshot:
    side: str
    qpos: tuple[float, ...]
    received_at: float


@dataclass(frozen=True)
class ManusGapSnapshot:
    side: str
    gaps_m: Mapping[str, float]
    features: Mapping[str, float]
    sequence: int
    received_at: float


@dataclass(frozen=True)
class PoseSequenceStatus:
    running: bool
    step_index: int
    step_count: int
    pose_name: str


class WujiUiRuntime:
    """Own both hands, MANUS retargeting, feedback, and bounded pose motion."""

    def __init__(
        self,
        *,
        addresses: Mapping[str, str],
        kp: float = 4.0,
        kd: float = 0.1,
        current_limit: float = 1.0,
        rate: float = 30.0,
        stale_timeout: float = 0.25,
        pose_speed_rad_s: float = 1.0,
        feedback_timeout: float = 0.5,
    ) -> None:
        if set(addresses) != set(SIDES) or any(not addresses[side] for side in SIDES):
            raise ValueError("Wuji UI需要左右两只Hand 2的明确地址")
        if not np.isfinite(pose_speed_rad_s) or pose_speed_rad_s <= 0.0:
            raise ValueError("姿态移动速度必须为正数")
        self.pose_speed_rad_s = float(pose_speed_rad_s)
        self.feedback_timeout = float(feedback_timeout)
        self.pipeline = WujiHandPipeline(
            sides=SIDES,
            models={side: "wuji_hand_2" for side in SIDES},
            addresses=dict(addresses),
            rate=rate,
            stale_timeout=stale_timeout,
            kp=kp,
            kd=kd,
            current_limit=current_limit,
            auto_enable=False,
        )
        self.joint_names = {
            side: tuple(self.pipeline.joint_names[side]) for side in SIDES
        }
        self.joint_limits = {
            side: tuple(self.pipeline.joint_limits[side]) for side in SIDES
        }
        self._lock = threading.RLock()
        self._hardware_lock = threading.RLock()
        self._stop = threading.Event()
        self._modes = {side: "manual" for side in SIDES}
        self._snapshots: dict[str, HandSnapshot] = {}
        self._manus_gaps: dict[str, ManusGapSnapshot] = {}
        self._manus_sequences = {side: -1 for side in SIDES}
        self._pose_commands: dict[str, np.ndarray | None] = {
            side: None for side in SIDES
        }
        self._pose_targets: dict[str, np.ndarray | None] = {
            side: None for side in SIDES
        }
        self._errors = {side: "等待真实关节反馈" for side in SIDES}
        self._sequence_stop = {side: threading.Event() for side in SIDES}
        self._sequence_status = {
            side: PoseSequenceStatus(False, 0, 0, "") for side in SIDES
        }
        self._thread = threading.Thread(
            target=self._loop, name="wuji-ui-hardware", daemon=True
        )
        self._thread.start()

    def mode(self, side: str) -> str:
        with self._lock:
            return self._modes[side]

    def error(self, side: str) -> str:
        with self._lock:
            return self._errors[side]

    def sequence_status(self, side: str) -> PoseSequenceStatus:
        with self._lock:
            return self._sequence_status[side]

    def snapshot(self, side: str, *, require_fresh: bool = True) -> HandSnapshot:
        with self._lock:
            snapshot = self._snapshots.get(side)
        if snapshot is None:
            raise RuntimeError(f"尚未收到{side} Wuji的完整关节反馈")
        age = time.monotonic() - snapshot.received_at
        if require_fresh and age > self.feedback_timeout:
            raise RuntimeError(f"{side} Wuji关节反馈已过期（{age:.2f}秒）")
        return snapshot

    def manus_gaps(
        self, side: str, *, require_fresh: bool = True
    ) -> ManusGapSnapshot:
        with self._lock:
            snapshot = self._manus_gaps.get(side)
        if snapshot is None:
            raise RuntimeError(f"尚未收到{side} MANUS骨骼数据")
        age = time.monotonic() - snapshot.received_at
        if require_fresh and age > self.feedback_timeout:
            raise RuntimeError(f"{side} MANUS骨骼数据已过期（{age:.2f}秒）")
        return snapshot

    def set_manual(self, side: str) -> None:
        self._sequence_stop[side].set()
        with self._hardware_lock:
            with self._lock:
                self._modes[side] = "manual"
                self._pose_commands[side] = None
                self._pose_targets[side] = None
            self.pipeline.set_enabled(side, False)

    def start_teleop(self, side: str) -> None:
        if self.sequence_status(side).running:
            raise RuntimeError(f"{side}姿态序列正在运行")
        snapshot = self.snapshot(side)
        with self._hardware_lock:
            self.pipeline.set_enabled(side, True)
            # Hold the measured pose until the next fresh MANUS solve arrives.
            self.pipeline.backends[side].send(np.asarray(snapshot.qpos))
            with self._lock:
                self._modes[side] = "teleop"
                self._pose_commands[side] = None
                self._pose_targets[side] = None

    def move_to_pose(self, side: str, qpos: Sequence[float]) -> None:
        if self.sequence_status(side).running:
            raise RuntimeError(f"{side}姿态序列正在运行")
        self._set_pose_target(side, qpos, mode="pose")

    def _set_pose_target(
        self, side: str, qpos: Sequence[float], *, mode: str
    ) -> None:
        target = np.asarray(qpos, dtype=np.float64)
        if target.shape != (20,) or not np.isfinite(target).all():
            raise ValueError("目标姿态必须包含20个有限关节角")
        limits = np.asarray(self.joint_limits[side], dtype=np.float64)
        if np.any(target < limits[:, 0] - 1e-6) or np.any(
            target > limits[:, 1] + 1e-6
        ):
            raise ValueError("目标姿态超出Wuji Hand 2模型关节范围")
        snapshot = self.snapshot(side)
        current = np.asarray(snapshot.qpos, dtype=np.float64)
        with self._hardware_lock:
            self.pipeline.set_enabled(side, True)
            self.pipeline.backends[side].send(current)
            with self._lock:
                self._pose_commands[side] = current
                self._pose_targets[side] = target.copy()
                self._modes[side] = mode

    def execute_pose_sequence(
        self,
        side: str,
        poses: Sequence[tuple[str, Sequence[float]]],
        *,
        hold_seconds: float = 0.5,
        position_tolerance_rad: float = 0.08,
    ) -> None:
        """Execute named poses serially and wait for measured settling per step."""
        if not poses:
            raise ValueError("姿态序列不能为空")
        hold = float(hold_seconds)
        tolerance = float(position_tolerance_rad)
        if not np.isfinite(hold) or hold < 0.0:
            raise ValueError("姿态停留时间不能为负数")
        if not np.isfinite(tolerance) or tolerance <= 0.0:
            raise ValueError("姿态到位容差必须为正数")
        validated = []
        limits = np.asarray(self.joint_limits[side], dtype=np.float64)
        for name, values in poses:
            target = np.asarray(values, dtype=np.float64)
            if target.shape != (20,) or not np.isfinite(target).all():
                raise ValueError(f"姿态{name}必须包含20个有限关节角")
            if np.any(target < limits[:, 0] - 1e-6) or np.any(
                target > limits[:, 1] + 1e-6
            ):
                raise ValueError(f"姿态{name}超出Wuji Hand 2模型关节范围")
            validated.append((str(name), target.copy()))

        with self._lock:
            if self._sequence_status[side].running:
                raise RuntimeError(f"{side}姿态序列已经在运行")
            self._sequence_status[side] = PoseSequenceStatus(
                True, 0, len(validated), ""
            )
        stop = self._sequence_stop[side]
        stop.clear()
        try:
            for index, (name, target) in enumerate(validated, start=1):
                if stop.is_set() or self._stop.is_set():
                    raise InterruptedError("姿态序列已停止")
                with self._lock:
                    self._sequence_status[side] = PoseSequenceStatus(
                        True, index, len(validated), name
                    )
                start = np.asarray(self.snapshot(side).qpos, dtype=np.float64)
                expected_motion = float(np.max(np.abs(target - start))) / (
                    self.pose_speed_rad_s
                )
                deadline = time.monotonic() + expected_motion + 5.0
                self._set_pose_target(side, target, mode="sequence")
                while True:
                    if stop.wait(0.02) or self._stop.is_set():
                        raise InterruptedError("姿态序列已停止")
                    with self._lock:
                        command_done = self._pose_targets[side] is None
                    if command_done:
                        measured = np.asarray(self.snapshot(side).qpos)
                        if np.max(np.abs(target - measured)) <= tolerance:
                            break
                    if time.monotonic() >= deadline:
                        raise TimeoutError(f"姿态{name}未在限定时间内到位")
                if hold > 0.0 and stop.wait(hold):
                    raise InterruptedError("姿态序列已停止")
            with self._lock:
                self._modes[side] = "hold"
        finally:
            with self._lock:
                self._sequence_status[side] = PoseSequenceStatus(
                    False, 0, 0, ""
                )
                if self._modes[side] == "sequence":
                    self._modes[side] = "hold"
                self._pose_targets[side] = None

    def stop_pose_sequence(self, side: str) -> bool:
        running = self.sequence_status(side).running
        self._sequence_stop[side].set()
        return running

    def _read_feedback(self, now: float) -> None:
        for side in SIDES:
            try:
                position = self.pipeline.feedback_position(side)
                if position is None:
                    continue
                snapshot = HandSnapshot(
                    side, tuple(float(value) for value in position), now
                )
                with self._lock:
                    self._snapshots[side] = snapshot
                    self._errors[side] = ""
            except Exception as error:
                with self._lock:
                    self._errors[side] = str(error)

    def _advance_poses(self, dt: float) -> None:
        for side in SIDES:
            with self._lock:
                if self._modes[side] not in {"pose", "sequence"}:
                    continue
                command = self._pose_commands[side]
                target = self._pose_targets[side]
            if command is None or target is None:
                continue
            step = self.pose_speed_rad_s * max(0.0, min(dt, 0.1))
            next_command = command + np.clip(target - command, -step, step)
            self.pipeline.backends[side].send(next_command)
            reached = bool(np.max(np.abs(target - next_command)) < 1e-6)
            with self._lock:
                self._pose_commands[side] = next_command
                if reached:
                    if self._modes[side] == "pose":
                        self._modes[side] = "hold"
                    self._pose_targets[side] = None

    def _read_manus_gaps(self, now: float) -> None:
        frames = getattr(self.pipeline, "last_frames", {})
        for side in SIDES:
            frame = frames.get(side)
            if frame is None:
                continue
            sequence = int(frame.sequence)
            if sequence == self._manus_sequences[side]:
                continue
            points = canonical_landmarks(frame)
            features = manus_gesture_features(points)
            thumb = points[4]
            gaps = {
                finger: float(np.linalg.norm(points[index] - thumb))
                for finger, index in {
                    "index": 8,
                    "middle": 12,
                    "ring": 16,
                    "pinky": 20,
                }.items()
            }
            with self._lock:
                self._manus_sequences[side] = sequence
                self._manus_gaps[side] = ManusGapSnapshot(
                    side, gaps, features, sequence, now
                )

    def _loop(self) -> None:
        previous = time.monotonic()
        while not self._stop.wait(0.01):
            now = time.monotonic()
            try:
                with self._hardware_lock:
                    self._read_feedback(now)
                    with self._lock:
                        active = {
                            side: self._modes[side] == "teleop" for side in SIDES
                        }
                    self.pipeline.tick(now, active=active)
                    self._read_manus_gaps(now)
                    with self._lock:
                        for side in SIDES:
                            if self._modes[side] == "teleop":
                                fault = self.pipeline.status.sides[side].fault
                                if fault:
                                    self._errors[side] = str(fault)
                    self._advance_poses(now - previous)
            except Exception as error:
                with self._lock:
                    for side in SIDES:
                        if self._modes[side] != "manual":
                            self._errors[side] = str(error)
            previous = now

    def close(self) -> None:
        self._stop.set()
        for stop in self._sequence_stop.values():
            stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        with self._hardware_lock:
            self.pipeline.close()
