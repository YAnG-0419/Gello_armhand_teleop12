"""Retarget PICO hand skeletons inline and send hand commands to the bridge.

One synchronous object, ticked from whichever loop owns the SDK client. No thread
and no second process: since the pinocchio rewrite a one-hand solve costs about
1.5 ms, and the pipeline solves at most one side per tick, so the worst a tick
can cost is one solve. That fits inside the arm loop's 10 ms budget, which the
jitter benchmark in the deployment notes verifies. The earlier design forwarded
skeletons to a separate retargeting process because a solve then cost 8-11 ms
while holding the GIL; that constraint is gone and the process with it.

Failure is contained in both directions. `tick` never raises, and losing an
optical skeleton never disengages an arm: they are independent signals, and a
wrist tracker can be perfectly healthy while the cameras lose sight of the
fingers. When a side becomes unusable this simply stops sending for it, the
bridge's watchdog stops publishing, the hand holds position, and the retargeter's
filter history is dropped so reacquisition cannot jump.
"""

from __future__ import annotations

import socket
import time
from dataclasses import dataclass, field
from pathlib import Path

from .hand_input import HandSkeletonReader
from .hand_stream import build_hand_packet
from .types import SIDES


@dataclass
class HandSideStatus:
    sending: bool = False
    fault: str | None = None
    sent: int = 0
    solve_seconds: float = 0.0


@dataclass
class HandStatus:
    sides: dict[str, HandSideStatus] = field(
        default_factory=lambda: {side: HandSideStatus() for side in SIDES}
    )
    errors: int = 0
    last_error: str | None = None


class HandPipeline:
    """Read, retarget and send both hands, at most one solve per tick."""

    def __init__(
        self,
        xrt,
        *,
        assets_dir: Path,
        host: str,
        port: int,
        rate: float = 30.0,
        sides: tuple[str, ...] = SIDES,
        stale_timeout: float = 0.25,
        frozen_timeout: float = 1.0,
        max_iterations: int = 20,
        debug_log: str | Path | None = None,
    ) -> None:
        if xrt is None:
            raise ValueError("HandPipeline requires an initialized SDK module")
        if not 0.0 < rate <= 60.0:
            raise ValueError("Hand send rate must be in (0, 60] Hz")
        if not sides or set(sides).difference(SIDES):
            raise ValueError(f"Invalid hand sides: {sides}")

        # Import here so arm-only runs never pay for pinocchio.
        from .hand_retarget import L20Retargeter, THUMB_OPPOSITION_YAW_ROLL

        assets = Path(assets_dir)
        self.retargeters = {}
        for side in sides:
            urdf = assets / side / f"linkerhand_l20_{side}.urdf"
            if not urdf.is_file():
                raise FileNotFoundError(f"Hand URDF not found: {urdf}")
            self.retargeters[side] = L20Retargeter(
                urdf,
                side,
                max_iterations=max_iterations,
                thumb_opposition_fixed=THUMB_OPPOSITION_YAW_ROLL[side],
            )

        self.reader = HandSkeletonReader(
            xrt, stale_timeout=stale_timeout, frozen_timeout=frozen_timeout
        )
        self.address = (str(host), int(port))
        self.interval = 1.0 / float(rate)
        self.sides = tuple(sides)
        self.status = HandStatus()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sequence = {side: 0 for side in sides}
        self._next_due = {side: 0.0 for side in sides}
        self._open_until = {side: 0.0 for side in sides}
        self._was_following = {side: False for side in sides}
        # Round-robin start point, so one side cannot starve the other when both
        # come due on the same tick.
        self._preferred = 0
        self.debug_logger = None
        if debug_log is not None:
            from .debug_log import HandRetargetDebugLogger

            self.debug_logger = HandRetargetDebugLogger(debug_log)

    def _advance_deadline(self, side: str, moment: float) -> None:
        """Advance a periodic deadline without drifting down to the loop grid."""
        deadline = self._next_due[side]
        if deadline <= 0.0 or moment - deadline >= self.interval:
            # First send or a long pause: do not create a catch-up burst.
            self._next_due[side] = moment + self.interval
        else:
            # Preserve fractional deadlines. With a 100 Hz owner loop, setting
            # `moment + interval` made nominal 30 Hz become 25 Hz because every
            # 33.3 ms period rounded up to four 10 ms ticks.
            self._next_due[side] = deadline + self.interval

    def request_open(self, now: float | None = None, duration: float = 2.0) -> None:
        """Stream the open-hand pose to every side that is not following.

        URDF zeros are the bridge's own home() pose: fingers straight, abduction
        centred. The stream lasts `duration` seconds because the bridge's 250 ms
        watchdog needs a continuous feed, not one packet. A side that is
        actively following the operator's hand ignores the request: live
        tracking always supersedes a parked-hand command.
        """
        if duration <= 0.0:
            raise ValueError("Open duration must be positive")
        moment = time.monotonic() if now is None else float(now)
        for side in self.sides:
            self._open_until[side] = moment + float(duration)

    def tick(
        self,
        now: float | None = None,
        active: dict[str, bool] | None = None,
    ) -> None:
        """Advance the hand pipeline by at most one solve. Never raises.

        `active` optionally gates following per side: a side whose flag is
        False stops sending, exactly as if its skeleton were lost, so the
        bridge watchdog holds that hand. The arm loop passes its keyboard
        activations here, making one keyboard control arm and hand together.
        Passing None (the standalone hands-only path) keeps every side
        following whenever its skeleton is live.
        """
        moment = time.monotonic() if now is None else float(now)
        try:
            samples = self.reader.sample(now=moment)
        except Exception as error:  # noqa: BLE001 - must not reach the arm loop
            self.status.errors += 1
            self.status.last_error = f"hand sample failed: {error}"
            return

        following = {}
        for side in self.sides:
            allowed = active is None or bool(active.get(side, False))
            follows = allowed and samples.get(side) is not None
            if follows:
                # Live tracking on an engaged side supersedes a pending open.
                self._open_until[side] = 0.0
            elif self._was_following[side]:
                # Dropping filter history keeps reacquisition smooth, whether
                # the side went on to stream the open pose or to hold.
                self.retargeters[side].reset()
            following[side] = follows
            self._was_following[side] = follows

        # Open-pose sends carry a fixed qpos and no solve, so they run outside
        # the one-solve-per-tick rotation; both sides may send in one tick.
        for side in self.sides:
            if (
                following[side]
                or moment >= self._open_until[side]
                or moment < self._next_due[side]
            ):
                continue
            status = self.status.sides[side]
            names = self.retargeters[side].joint_names
            try:
                self._socket.sendto(
                    build_hand_packet(
                        f"pico-hand-{side}-open",
                        self._sequence[side],
                        time.time(),
                        side,
                        names,
                        [0.0] * len(names),
                    ),
                    self.address,
                )
            except Exception as error:  # noqa: BLE001 - contain per side
                self.status.errors += 1
                self.status.last_error = f"{side} open command failed: {error}"
                continue
            self._sequence[side] += 1
            self._advance_deadline(side, moment)
            status.sending = True
            status.fault = None
            status.sent += 1

        for side in self.sides:
            if following[side] or moment < self._open_until[side]:
                continue
            # Send nothing; the bridge watchdog holds the hand.
            status = self.status.sides[side]
            status.sending = False
            status.fault = (
                self.reader.faults[side]
                if samples.get(side) is None
                else "disengaged by operator"
            )

        # Solve at most one side per tick so a tick never costs two solves.
        order = [
            self.sides[(self._preferred + offset) % len(self.sides)]
            for offset in range(len(self.sides))
        ]
        for side in order:
            sample = samples.get(side)
            if not following[side] or sample is None or moment < self._next_due[side]:
                continue
            status = self.status.sides[side]
            try:
                started = time.monotonic()
                qpos, stats = self.retargeters[side].retarget(sample.landmarks)
                elapsed = time.monotonic() - started
                if self.debug_logger is not None:
                    self.debug_logger.record(
                        moment, side, sample.landmarks, qpos, stats
                    )
                self._socket.sendto(
                    build_hand_packet(
                        f"pico-hand-{side}",
                        self._sequence[side],
                        time.time(),
                        side,
                        self.retargeters[side].joint_names,
                        qpos,
                    ),
                    self.address,
                )
            except Exception as error:  # noqa: BLE001 - contain per side
                self.status.errors += 1
                self.status.last_error = f"{side} hand retargeting failed: {error}"
                status.sending = False
                status.fault = str(error)
                return
            self._sequence[side] += 1
            self._advance_deadline(side, moment)
            self._preferred = (self.sides.index(side) + 1) % len(self.sides)
            status.sending = True
            status.fault = None
            status.sent += 1
            status.solve_seconds = elapsed
            return

    def close(self) -> None:
        try:
            if self.debug_logger is not None:
                self.debug_logger.close()
        finally:
            self._socket.close()
            for retargeter in self.retargeters.values():
                retargeter.close()
