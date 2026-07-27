"""MANUS right-glove source with the same lifecycle as the PICO hand pipeline."""

from __future__ import annotations

import ctypes
import time
from pathlib import Path

import numpy as np

from pico_bimanual_franka_teleop.hand_sender import HandCommandSender, HandStatus

KEYPOINT_COUNT = 25
RIGHT = 2
CANONICAL_FROM_MANUS = (
    0,
    1, 2, 3, 4,
    6, 7, 8, 9,
    11, 12, 13, 14,
    16, 17, 18, 19,
    21, 22, 23, 24,
)
REPO_ROOT = Path(__file__).resolve().parents[4]


class ManusPose(ctypes.Structure):
    _fields_ = [
        ("position_x", ctypes.c_float),
        ("position_y", ctypes.c_float),
        ("position_z", ctypes.c_float),
        ("orientation_x", ctypes.c_float),
        ("orientation_y", ctypes.c_float),
        ("orientation_z", ctypes.c_float),
        ("orientation_w", ctypes.c_float),
    ]


class ManusFrame(ctypes.Structure):
    _fields_ = [
        ("side", ctypes.c_int32),
        ("keypoint_count", ctypes.c_uint32),
        ("timestamp_ns", ctypes.c_uint64),
        ("sequence", ctypes.c_uint64),
        ("keypoints", ManusPose * KEYPOINT_COUNT),
    ]


class ManusBridge:
    """Small ctypes owner for the vendored native MANUS bridge."""

    def __init__(self, library: Path) -> None:
        self.library = ctypes.CDLL(str(library))
        self.library.litchi_manus_bridge_abi_version.restype = ctypes.c_uint32
        self.library.litchi_manus_frame_size.restype = ctypes.c_uint32
        self.library.litchi_manus_connect.argtypes = [
            ctypes.c_uint32,
            ctypes.c_int32,
            ctypes.c_char_p,
        ]
        self.library.litchi_manus_connect.restype = ctypes.c_int32
        self.library.litchi_manus_disconnect.restype = ctypes.c_int32
        self.library.litchi_manus_read_frame.argtypes = [
            ctypes.c_int32,
            ctypes.c_uint32,
            ctypes.POINTER(ManusFrame),
        ]
        self.library.litchi_manus_read_frame.restype = ctypes.c_int32
        self.library.litchi_manus_last_error.restype = ctypes.c_char_p
        if self.library.litchi_manus_bridge_abi_version() != 1:
            raise RuntimeError("unsupported MANUS skeleton bridge ABI")
        if self.library.litchi_manus_frame_size() != ctypes.sizeof(ManusFrame):
            raise RuntimeError("MANUS frame ABI size mismatch")
        self.connected = False

    def error(self) -> str:
        message = self.library.litchi_manus_last_error()
        return message.decode("utf-8", errors="replace") if message else "unknown error"

    def connect(self, calibration_dir: Path) -> None:
        result = self.library.litchi_manus_connect(
            1, 1, str(calibration_dir).encode("utf-8")
        )
        if result != 0:
            raise RuntimeError(self.error())
        self.connected = True

    def read(self, timeout_s: float = 0.0) -> ManusFrame | None:
        frame = ManusFrame()
        result = self.library.litchi_manus_read_frame(
            RIGHT,
            max(0, round(float(timeout_s) * 1000)),
            ctypes.byref(frame),
        )
        if result < 0:
            raise RuntimeError(self.error())
        if result == 0:
            return None
        if frame.side != RIGHT or frame.keypoint_count != KEYPOINT_COUNT:
            raise RuntimeError("invalid right MANUS skeleton frame")
        return frame

    def close(self) -> None:
        if self.connected:
            self.library.litchi_manus_disconnect()
            self.connected = False


def canonical_landmarks(frame: ManusFrame) -> np.ndarray:
    points = np.asarray(
        [
            (point.position_x, point.position_y, point.position_z)
            for point in frame.keypoints
        ],
        dtype=np.float64,
    )
    selected = points[list(CANONICAL_FROM_MANUS)]
    if selected.shape != (21, 3) or not np.isfinite(selected).all():
        raise ValueError("MANUS skeleton contains invalid landmarks")
    return selected


class RightOnlyManusHandPipeline:
    """Bimanual source: default left hand plus activation-gated right MANUS."""

    sides = ("left", "right")

    def __init__(
        self,
        *,
        host: str = "127.0.0.1",
        port: int = 5570,
        rate: float = 30.0,
        stale_timeout: float = 0.25,
        library: Path | None = None,
        calibration_dir: Path | None = None,
        debug_log: str | Path | None = None,
        bridge_factory=ManusBridge,
    ) -> None:
        if stale_timeout <= 0.0:
            raise ValueError("MANUS stale timeout must be positive")

        from pico_bimanual_franka_teleop.hand_retarget import L20Retargeter

        self.stale_timeout = float(stale_timeout)
        self.status = HandStatus()
        self.sender = HandCommandSender(
            host=host, port=port, rate=rate, sides=self.sides, status=self.status
        )
        try:
            self.retargeter = L20Retargeter(
                REPO_ROOT
                / "assets"
                / "linkerhand_l20"
                / "right"
                / "linkerhand_l20_right.urdf",
                "right",
                thumb_opposition_fixed=None,
            )
        except BaseException:
            self.sender.close()
            raise
        library_path = (
            Path(library)
            if library is not None
            else REPO_ROOT
            / "teleop_sources"
            / "manus"
            / "build"
            / "libmanus_skeleton_bridge.so"
        )
        calibration_path = (
            Path(calibration_dir)
            if calibration_dir is not None
            else REPO_ROOT / "teleop_sources" / "manus" / "config"
        )
        try:
            self.bridge = bridge_factory(library_path.resolve())
            self.bridge.connect(calibration_path.resolve())
        except BaseException:
            self.retargeter.close()
            self.sender.close()
            raise
        self.left_joint_names = tuple(
            "thumb_ip" if name == "thumb_dip" else name
            for name in self.retargeter.joint_names
        )
        self.open_until = 0.0
        self.last_frame: ManusFrame | None = None
        self.last_frame_at: float | None = None
        self.was_following = False
        self.debug_logger = None
        if debug_log is not None:
            from pico_bimanual_franka_teleop.debug_log import HandRetargetDebugLogger

            self.debug_logger = HandRetargetDebugLogger(debug_log)

    def request_open(self, now: float | None = None, duration: float = 2.0) -> None:
        if duration <= 0.0:
            raise ValueError("Open duration must be positive")
        moment = time.monotonic() if now is None else float(now)
        self.open_until = moment + float(duration)

    def tick(
        self,
        now: float | None = None,
        active: dict[str, bool] | None = None,
    ) -> None:
        moment = time.monotonic() if now is None else float(now)
        if self.sender.due("left", moment):
            self.sender.emit(
                "manus-left-default",
                "left",
                self.left_joint_names,
                [0.0] * len(self.left_joint_names),
                moment,
                "left default command failed",
            )

        status = self.status.sides["right"]
        try:
            frame = self.bridge.read(0.0)
            if frame is not None:
                self.last_frame = frame
                self.last_frame_at = moment
        except Exception as error:  # noqa: BLE001 - contain source failure
            self.status.errors += 1
            self.status.last_error = f"MANUS read failed: {error}"
            status.sending = False
            status.fault = self.status.last_error
            return

        allowed = active is None or bool(active.get("right", False))
        fresh = (
            self.last_frame is not None
            and self.last_frame_at is not None
            and moment - self.last_frame_at <= self.stale_timeout
        )
        following = allowed and fresh
        if following:
            self.open_until = 0.0
        elif self.was_following:
            self.retargeter.reset()
        self.was_following = following

        if not following:
            if (
                not allowed
                and moment < self.open_until
                and self.sender.due("right", moment)
            ):
                self.sender.emit(
                    "manus-right-open",
                    "right",
                    self.retargeter.joint_names,
                    [0.0] * len(self.retargeter.joint_names),
                    moment,
                    "MANUS open command failed",
                )
                return
            status.sending = False
            if not allowed:
                status.fault = "disengaged by operator"
            elif self.last_frame is None:
                status.fault = "waiting for calibrated right MANUS glove"
            else:
                status.fault = "right MANUS frame is stale"
            return

        if not self.sender.due("right", moment):
            return
        try:
            started = time.monotonic()
            landmarks = canonical_landmarks(self.last_frame)
            qpos, stats = self.retargeter.retarget(landmarks)
            status.solve_seconds = time.monotonic() - started
            if self.debug_logger is not None:
                self.debug_logger.record(moment, "right", landmarks, qpos, stats)
        except Exception as error:  # noqa: BLE001 - never break arm control
            self.status.errors += 1
            self.status.last_error = f"MANUS retargeting failed: {error}"
            status.sending = False
            status.fault = str(error)
            return
        self.sender.emit(
            "manus-right-full-thumb",
            "right",
            self.retargeter.joint_names,
            qpos,
            moment,
            "MANUS send failed",
        )

    def close(self) -> None:
        try:
            if self.debug_logger is not None:
                self.debug_logger.close()
        finally:
            try:
                self.sender.close()
            finally:
                try:
                    self.retargeter.close()
                finally:
                    self.bridge.close()
