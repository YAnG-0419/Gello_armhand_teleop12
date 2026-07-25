"""Forward PICO hand skeletons out of the arm process.

The arm process owns the only XRoboToolkit client, so it is the only place the hand
skeletons can be read. It is also a 100 Hz real-time loop with a 10 ms budget, and
retargeting a hand costs 8 to 11 ms while holding the GIL. Measured here, doing that
work on a thread inside this process pushed the arm loop from a p99 of 10.18 ms to
29 ms, late on 23% of ticks. Retargeting therefore happens in another process; this
only reads and forwards, which costs well under a millisecond.

Forwarding runs on its own thread anyway, so the arm loop never waits on a socket,
and every failure is contained: nothing here raises into the arm loop, and losing an
optical skeleton never disengages an arm. The two are independent signals, and a
wrist tracker can be perfectly healthy while the cameras lose sight of the fingers.

Skeletons are forwarded raw rather than as landmarks, including `isActive`, so the
receiver applies the same validation and liveness rules it would apply to a live
client. Inactive frames are forwarded too: the receiver needs to see tracking drop
in order to stop commanding, and silence alone cannot be distinguished from a dead
sender.
"""

from __future__ import annotations

import socket
import threading
import time
from dataclasses import dataclass, field

from .hand_skeleton_stream import encode_skeleton_packet
from .types import SIDES


@dataclass
class HandSideStatus:
    forwarded: int = 0
    active: bool = False
    last_active_at: float | None = None


@dataclass
class HandForwardStatus:
    sides: dict[str, HandSideStatus] = field(
        default_factory=lambda: {side: HandSideStatus() for side in SIDES}
    )
    errors: int = 0
    last_error: str | None = None


class HandSkeletonForwarder:
    """Read both hand skeletons from the shared SDK client and send them on."""

    def __init__(
        self,
        xrt,
        *,
        host: str,
        port: int,
        rate: float,
        sides: tuple[str, ...] = SIDES,
        stream_id: str = "pico-skeleton",
    ) -> None:
        if xrt is None:
            raise ValueError("HandSkeletonForwarder requires an initialized SDK module")
        if not 0.0 < rate <= 120.0:
            raise ValueError("Hand forward rate must be in (0, 120] Hz")
        if not sides or set(sides).difference(SIDES):
            raise ValueError(f"Invalid hand sides: {sides}")

        self.xrt = xrt
        self.address = (str(host), int(port))
        self.interval = 1.0 / float(rate)
        self.sides = tuple(sides)
        self.stream_id = str(stream_id)

        self.status = HandForwardStatus()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("Hand forwarder is already running")
        self._thread = threading.Thread(
            target=self._run, name="hand-forward", daemon=True
        )
        self._thread.start()

    def stop(self, timeout: float = 3.0) -> None:
        self._stop.set()
        thread = self._thread
        self._thread = None
        if thread is not None:
            thread.join(timeout=timeout)

    def snapshot(self) -> HandForwardStatus:
        with self._lock:
            return HandForwardStatus(
                sides={
                    side: HandSideStatus(
                        forwarded=status.forwarded,
                        active=status.active,
                        last_active_at=status.last_active_at,
                    )
                    for side, status in self.status.sides.items()
                },
                errors=self.status.errors,
                last_error=self.status.last_error,
            )

    def _note_error(self, message: str) -> None:
        with self._lock:
            self.status.errors += 1
            self.status.last_error = message

    def _read(self, side: str):
        if side == "left":
            return (
                self.xrt.get_left_hand_tracking_state(),
                int(self.xrt.get_left_hand_is_active()),
            )
        return (
            self.xrt.get_right_hand_tracking_state(),
            int(self.xrt.get_right_hand_is_active()),
        )

    def _run(self) -> None:
        sock: socket.socket | None = None
        sequence = {side: 0 for side in self.sides}
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            while not self._stop.is_set():
                started = time.monotonic()
                for side in self.sides:
                    try:
                        joints, is_active = self._read(side)
                        payload = encode_skeleton_packet(
                            self.stream_id,
                            sequence[side],
                            time.time(),
                            side,
                            is_active,
                            joints,
                        )
                        sock.sendto(payload, self.address)
                    except Exception as error:  # noqa: BLE001 - contain per side
                        self._note_error(f"{side} skeleton forward failed: {error}")
                        continue
                    sequence[side] += 1
                    with self._lock:
                        status = self.status.sides[side]
                        status.forwarded += 1
                        status.active = is_active == 1
                        if is_active == 1:
                            status.last_active_at = started
                remaining = self.interval - (time.monotonic() - started)
                if remaining > 0.0:
                    self._stop.wait(remaining)
        except Exception as error:  # noqa: BLE001 - the thread must not kill the arm
            self._note_error(f"hand forward thread stopped: {error}")
        finally:
            if sock is not None:
                sock.close()
