"""MANUS glove source with the same lifecycle as the PICO hand pipeline.

Bimanual: each side in ``dynamic_sides`` follows its glove through the
native skeleton bridge (left is retargeted through the same L20 profile as
the validated PICO left-G20 path, right through the O30i solver); every
other side streams its default pose. A glove only delivers frames once its
calibration file exists next to the bridge's calibration directory
(``Calibration_left.mcal`` / ``Calibration_right.mcal``) - an uncalibrated
side simply reports waiting and the rest keeps working.
"""

from __future__ import annotations

import ctypes
import time
from pathlib import Path
from xml.etree import ElementTree

import numpy as np

from pico_bimanual_franka_teleop.hand_sender import HandCommandSender, HandStatus

KEYPOINT_COUNT = 25
SIDE_CODES = {"left": 1, "right": 2}
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
        self.library.litchi_manus_available_sides.restype = ctypes.c_uint32
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

    def available_sides(self) -> tuple[str, ...]:
        mask = int(self.library.litchi_manus_available_sides())
        return tuple(
            side for side, code in SIDE_CODES.items() if mask & (1 << (code - 1))
        )

    def read(self, side: str, timeout_s: float = 0.0) -> ManusFrame | None:
        code = SIDE_CODES[side]
        frame = ManusFrame()
        result = self.library.litchi_manus_read_frame(
            code,
            max(0, round(float(timeout_s) * 1000)),
            ctypes.byref(frame),
        )
        if result < 0:
            raise RuntimeError(self.error())
        if result == 0:
            return None
        if frame.side != code or frame.keypoint_count != KEYPOINT_COUNT:
            raise RuntimeError(f"invalid {side} MANUS skeleton frame")
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


def _create_retargeter(side: str, model: str, filter_alpha: float):
    if model == "o30i":
        from .o30i_retarget import O30IRetargeter

        return O30IRetargeter(
            REPO_ROOT
            / "assets"
            / "linkerhand_o30i"
            / side
            / f"linkerhand_o30i_{side}.urdf",
            side,
            filter_alpha=filter_alpha,
        )
    # The G20 profile mirrors the hardware-validated PICO path: the L20
    # model with the fixed-opposition thumb (hand_profiles.py).
    from pico_bimanual_franka_teleop.hand_retarget import (
        L20Retargeter,
        THUMB_OPPOSITION_YAW_ROLL,
    )

    return L20Retargeter(
        REPO_ROOT
        / "assets"
        / "linkerhand_l20"
        / side
        / f"linkerhand_l20_{side}.urdf",
        side,
        filter_alpha=filter_alpha,
        thumb_opposition_fixed=THUMB_OPPOSITION_YAW_ROLL[side],
    )


def _default_joint_names(side: str) -> tuple[str, ...]:
    urdf = (
        REPO_ROOT
        / "assets"
        / "linkerhand_l20"
        / side
        / f"linkerhand_l20_{side}.urdf"
    )
    root = ElementTree.parse(urdf).getroot()
    return tuple(
        element.attrib["name"]
        for element in root.findall("joint")
        if element.attrib.get("type") in {"revolute", "continuous"}
    )


class ManusHandPipeline:
    """Bimanual MANUS source: dynamic sides follow their gloves, the rest
    stream their default pose. At most one retargeting solve per tick."""

    sides = ("left", "right")

    def __init__(
        self,
        *,
        host: str = "127.0.0.1",
        port: int = 5570,
        rate: float = 30.0,
        stale_timeout: float = 0.25,
        filter_alpha: float = 0.85,
        library: Path | None = None,
        calibration_dir: Path | None = None,
        debug_log: str | Path | None = None,
        models: dict[str, str] | None = None,
        dynamic_sides: tuple[str, ...] = ("left", "right"),
        bridge_factory=ManusBridge,
    ) -> None:
        if stale_timeout <= 0.0:
            raise ValueError("MANUS stale timeout must be positive")
        if not dynamic_sides or set(dynamic_sides).difference(self.sides):
            raise ValueError(f"Invalid dynamic sides: {dynamic_sides}")
        self.dynamic_sides = tuple(
            side for side in self.sides if side in dynamic_sides
        )
        if models is not None and set(models) != set(self.sides):
            raise ValueError("models must define exactly left and right")
        models = (
            {side: "g20" for side in self.sides}
            if models is None
            else {
                side: str(models[side]).strip().lower() for side in self.sides
            }
        )
        if models["left"] != "g20" or models["right"] not in {"g20", "o30i"}:
            raise ValueError(
                "MANUS supports left=g20 and right in {g20, o30i}"
            )

        self.stale_timeout = float(stale_timeout)
        self.status = HandStatus()
        self.sender = HandCommandSender(
            host=host,
            port=port,
            rate=rate,
            sides=self.sides,
            models=models,
            status=self.status,
        )
        self.retargeters = {}
        try:
            for side in self.dynamic_sides:
                self.retargeters[side] = _create_retargeter(
                    side, models[side], filter_alpha
                )
        except BaseException:
            for retargeter in self.retargeters.values():
                retargeter.close()
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
            for retargeter in self.retargeters.values():
                retargeter.close()
            self.sender.close()
            raise
        self.default_joint_names = {
            side: _default_joint_names(side)
            for side in self.sides
            if side not in self.dynamic_sides
        }
        self.open_until = {side: 0.0 for side in self.dynamic_sides}
        self.last_frame: dict[str, ManusFrame | None] = {
            side: None for side in self.dynamic_sides
        }
        self.last_frame_at: dict[str, float | None] = {
            side: None for side in self.dynamic_sides
        }
        self.was_following = {side: False for side in self.dynamic_sides}
        # Round-robin start point, so one side cannot starve the other when
        # both come due on the same tick (same rule as the PICO pipeline).
        self._preferred = 0
        self.debug_logger = None
        if debug_log is not None:
            from pico_bimanual_franka_teleop.debug_log import HandRetargetDebugLogger

            self.debug_logger = HandRetargetDebugLogger(debug_log)

    def request_open(
        self,
        now: float | None = None,
        duration: float = 2.0,
        sides: tuple[str, ...] | None = None,
    ) -> None:
        if duration <= 0.0:
            raise ValueError("Open duration must be positive")
        moment = time.monotonic() if now is None else float(now)
        selected = self.dynamic_sides if sides is None else sides
        for side in selected:
            if side in self.open_until:
                self.open_until[side] = moment + float(duration)

    def _read_side(self, side: str, moment: float) -> bool:
        """Pull the freshest frame for one side; False when the source broke."""
        status = self.status.sides[side]
        try:
            frame = self.bridge.read(side, 0.0)
        except Exception as error:  # noqa: BLE001 - contain source failure
            self.status.errors += 1
            self.status.last_error = f"{side} MANUS read failed: {error}"
            status.sending = False
            status.fault = self.status.last_error
            return False
        if frame is not None:
            self.last_frame[side] = frame
            self.last_frame_at[side] = moment
        return True

    def tick(
        self,
        now: float | None = None,
        active: dict[str, bool] | None = None,
    ) -> None:
        moment = time.monotonic() if now is None else float(now)
        for side, joint_names in self.default_joint_names.items():
            if self.sender.due(side, moment):
                self.sender.emit(
                    f"manus-{side}-default",
                    side,
                    joint_names,
                    [0.0] * len(joint_names),
                    moment,
                    f"{side} default command failed",
                )

        following = {}
        for side in self.dynamic_sides:
            if not self._read_side(side, moment):
                following[side] = False
                self.was_following[side] = False
                continue
            allowed = active is None or bool(active.get(side, False))
            fresh = (
                self.last_frame[side] is not None
                and self.last_frame_at[side] is not None
                and moment - self.last_frame_at[side] <= self.stale_timeout
            )
            follows = allowed and fresh
            if follows:
                self.open_until[side] = 0.0
            elif self.was_following[side]:
                self.retargeters[side].reset()
            following[side] = follows
            self.was_following[side] = follows

            if follows:
                continue
            status = self.status.sides[side]
            if (
                not allowed
                and moment < self.open_until[side]
                and self.sender.due(side, moment)
            ):
                names = self.retargeters[side].joint_names
                self.sender.emit(
                    f"manus-{side}-open",
                    side,
                    names,
                    [0.0] * len(names),
                    moment,
                    f"{side} MANUS open command failed",
                )
                continue
            status.sending = False
            if not allowed:
                status.fault = "disengaged by operator"
            elif self.last_frame[side] is None:
                status.fault = (
                    f"waiting for calibrated {side} MANUS glove "
                    f"(Calibration_{side}.mcal present?)"
                )
            else:
                status.fault = f"{side} MANUS frame is stale"

        # Solve at most one side per tick so a tick never costs two solves.
        order = [
            self.dynamic_sides[
                (self._preferred + offset) % len(self.dynamic_sides)
            ]
            for offset in range(len(self.dynamic_sides))
        ]
        for side in order:
            if not following.get(side) or not self.sender.due(side, moment):
                continue
            status = self.status.sides[side]
            try:
                started = time.monotonic()
                landmarks = canonical_landmarks(self.last_frame[side])
                qpos, stats = self.retargeters[side].retarget(landmarks)
                elapsed = time.monotonic() - started
                if self.debug_logger is not None:
                    self.debug_logger.record(moment, side, landmarks, qpos, stats)
            except Exception as error:  # noqa: BLE001 - never break arm control
                self.status.errors += 1
                self.status.last_error = f"{side} MANUS retargeting failed: {error}"
                status.sending = False
                status.fault = str(error)
                return
            if self.sender.emit(
                f"manus-{side}-{self.sender.models[side]}",
                side,
                self.retargeters[side].joint_names,
                qpos,
                moment,
                f"{side} MANUS send failed",
            ):
                self._preferred = (
                    self.dynamic_sides.index(side) + 1
                ) % len(self.dynamic_sides)
                status.solve_seconds = elapsed
            return

    def close(self) -> None:
        try:
            if self.debug_logger is not None:
                self.debug_logger.close()
        finally:
            try:
                self.sender.close()
            finally:
                try:
                    for retargeter in self.retargeters.values():
                        retargeter.close()
                finally:
                    self.bridge.close()
