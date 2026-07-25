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
    ) -> None:
        if xrt is None:
            raise ValueError("HandPipeline requires an initialized SDK module")
        if not 0.0 < rate <= 60.0:
            raise ValueError("Hand send rate must be in (0, 60] Hz")
        if not sides or set(sides).difference(SIDES):
            raise ValueError(f"Invalid hand sides: {sides}")

        # Import here so arm-only runs never pay for pinocchio.
        from .hand_retarget import L20Retargeter

        assets = Path(assets_dir)
        self.retargeters = {}
        for side in sides:
            urdf = assets / side / f"linkerhand_l20_{side}.urdf"
            if not urdf.is_file():
                raise FileNotFoundError(f"Hand URDF not found: {urdf}")
            self.retargeters[side] = L20Retargeter(
                urdf, side, max_iterations=max_iterations
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
        # Round-robin start point, so one side cannot starve the other when both
        # come due on the same tick.
        self._preferred = 0

    def tick(self, now: float | None = None) -> None:
        """Advance the hand pipeline by at most one solve. Never raises."""
        moment = time.monotonic() if now is None else float(now)
        try:
            samples = self.reader.sample(now=moment)
        except Exception as error:  # noqa: BLE001 - must not reach the arm loop
            self.status.errors += 1
            self.status.last_error = f"hand sample failed: {error}"
            return

        for side in self.sides:
            status = self.status.sides[side]
            if samples.get(side) is None:
                if status.sending:
                    # Send nothing; the bridge watchdog holds the hand, and
                    # dropping filter history keeps reacquisition smooth.
                    self.retargeters[side].reset()
                status.sending = False
                status.fault = self.reader.faults[side]

        # Solve at most one side per tick so a tick never costs two solves.
        order = [
            self.sides[(self._preferred + offset) % len(self.sides)]
            for offset in range(len(self.sides))
        ]
        for side in order:
            sample = samples.get(side)
            if sample is None or moment < self._next_due[side]:
                continue
            status = self.status.sides[side]
            try:
                started = time.monotonic()
                qpos, _ = self.retargeters[side].retarget(sample.landmarks)
                elapsed = time.monotonic() - started
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
            self._next_due[side] = moment + self.interval
            self._preferred = (self.sides.index(side) + 1) % len(self.sides)
            status.sending = True
            status.fault = None
            status.sent += 1
            status.solve_seconds = elapsed
            return

    def close(self) -> None:
        self._socket.close()
        for retargeter in self.retargeters.values():
            retargeter.close()
