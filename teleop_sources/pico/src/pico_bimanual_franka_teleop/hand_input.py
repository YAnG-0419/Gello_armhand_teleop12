"""Acquire PICO optical hand skeletons from an already-open SDK client.

This reader deliberately does not call `xrt.init()`. The XRoboToolkit PC Service
has not been shown to serve multiple simultaneous Python clients safely, and the
existing controller and motion-tracker inputs already own a client. Whoever owns
that client passes the module in here, so arm poses and hand skeletons arrive
through one connection.

Verified on real hardware: optical hand tracking and Object Motion Tracking do
coexist in one client. Motion timestamp advance was statistically identical
whether hands were active or not, and motion never went stale while a hand was
live. Hand tracking runs at about 52 Hz against an XR frame rate of about 69 Hz,
which is the native optical rate rather than contention.

Liveness rules, all established from measured data rather than assumed:

`isActive` is load-bearing. Complete, entirely plausible pose arrays continue to
be served after tracking is lost. In a 45 s recording, fully valid poses appeared
in 434 of 440 samples while `isActive` was 1 in only 321. Liveness therefore
cannot be inferred from array contents, and `isActive != 1` is treated as no
data at all.

There is no per-hand timestamp in the binding, so freshness is detected by the
pose array changing. That is sound here because optical jitter means a hand held
deliberately still still changes: zero consecutive active samples were bitwise
identical across the recording. A genuinely frozen cache would appear as exactly
constant data and is reported as a fault.

Loss of a hand skeleton is independent of loss of a wrist tracker and must never
be allowed to disengage the arms, nor the reverse.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from .hand_landmarks import (
    OPENXR_JOINT_COUNT,
    to_canonical_landmarks,
    validate_skeleton,
)
from .types import SIDES


@dataclass(frozen=True)
class HandSample:
    """One usable optical hand observation for a single side."""

    side: str
    landmarks: np.ndarray
    timestamp: float

    def __post_init__(self) -> None:
        if self.side not in SIDES:
            raise ValueError(f"side must be left or right, got {self.side!r}")
        array = np.asarray(self.landmarks, dtype=float)
        if array.shape != (21, 3):
            raise ValueError(
                f"Hand sample requires canonical (21, 3) landmarks, got {array.shape}"
            )
        if not np.all(np.isfinite(array)):
            raise ValueError("Hand sample contains a non-finite value")
        object.__setattr__(self, "landmarks", array.copy())


class HandSkeletonReader:
    """Read and validate both hand skeletons from a shared SDK client."""

    def __init__(
        self,
        xrt,
        *,
        stale_timeout: float = 0.25,
        frozen_timeout: float = 1.0,
    ) -> None:
        if xrt is None:
            raise ValueError("HandSkeletonReader requires an initialized SDK module")
        if stale_timeout <= 0 or frozen_timeout <= 0:
            raise ValueError("Hand stale and frozen timeouts must be positive")
        self.xrt = xrt
        self.stale_timeout = float(stale_timeout)
        self.frozen_timeout = float(frozen_timeout)
        self._previous: dict[str, np.ndarray | None] = {s: None for s in SIDES}
        self._changed_at: dict[str, float | None] = {s: None for s in SIDES}
        self._active_since: dict[str, float | None] = {s: None for s in SIDES}
        self.faults: dict[str, str | None] = {s: None for s in SIDES}

    def _raw(self, side: str) -> tuple[np.ndarray, int]:
        if side == "left":
            poses = self.xrt.get_left_hand_tracking_state()
            active = self.xrt.get_left_hand_is_active()
        else:
            poses = self.xrt.get_right_hand_tracking_state()
            active = self.xrt.get_right_hand_is_active()
        return np.asarray(poses, dtype=float), int(active)

    def _forget(self, side: str) -> None:
        self._previous[side] = None
        self._changed_at[side] = None
        self._active_since[side] = None

    def sample(self, now: float | None = None) -> dict[str, HandSample | None]:
        """Return a usable sample per side, or None where the hand is unusable.

        Each side is evaluated independently. `self.faults` carries a short
        reason for every side that returned None.
        """
        moment = time.monotonic() if now is None else float(now)
        result: dict[str, HandSample | None] = {}

        for side in SIDES:
            try:
                raw, is_active = self._raw(side)
            except Exception as error:  # noqa: BLE001 - SDK raises bare exceptions
                self.faults[side] = f"hand SDK read failed: {error}"
                self._forget(side)
                result[side] = None
                continue

            if raw.shape != (OPENXR_JOINT_COUNT, 7):
                self.faults[side] = (
                    f"unexpected hand array shape {raw.shape}; "
                    f"expected {(OPENXR_JOINT_COUNT, 7)}"
                )
                self._forget(side)
                result[side] = None
                continue

            # Anything other than 1 means the pose array, however plausible it
            # looks, is not a current measurement.
            if is_active != 1:
                self.faults[side] = f"hand tracking inactive (isActive={is_active})"
                self._forget(side)
                result[side] = None
                continue

            previous = self._previous[side]
            self._previous[side] = raw
            if previous is None:
                # First active frame after a dropout. Start the freshness clock
                # rather than trusting a single sample.
                self._changed_at[side] = moment
                self._active_since[side] = moment
                self.faults[side] = "hand tracking reacquiring"
                result[side] = None
                continue
            if not np.array_equal(previous, raw):
                self._changed_at[side] = moment

            changed_at = self._changed_at[side]
            if changed_at is None or moment - changed_at > self.frozen_timeout:
                self.faults[side] = "hand skeleton is frozen"
                result[side] = None
                continue
            if moment - changed_at > self.stale_timeout:
                self.faults[side] = "hand skeleton is stale"
                result[side] = None
                continue

            try:
                validate_skeleton(raw)
                landmarks = to_canonical_landmarks(raw)
            except ValueError as error:
                self.faults[side] = str(error)
                result[side] = None
                continue

            self.faults[side] = None
            result[side] = HandSample(side, landmarks, moment)

        return result
