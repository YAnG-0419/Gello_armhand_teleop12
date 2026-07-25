import time

import numpy as np
import pytest
from hand_fixtures import synthetic_skeleton

from pico_bimanual_franka_teleop.hand_skeleton_stream import (
    MAX_DATAGRAM_BYTES,
    decode_skeleton_packet,
    encode_skeleton_packet,
)
from pico_bimanual_franka_teleop.hand_teleop import HandSkeletonForwarder


class FakeXrt:
    def __init__(self) -> None:
        self.left = synthetic_skeleton(mirror=True)
        self.right = synthetic_skeleton(mirror=False)
        self.left_active = 1
        self.right_active = 1
        self.raise_on_read = False
        self.reads = 0

    def get_left_hand_tracking_state(self):
        self.reads += 1
        if self.raise_on_read:
            raise RuntimeError("SDK exploded")
        return self.left

    def get_right_hand_tracking_state(self):
        self.reads += 1
        if self.raise_on_read:
            raise RuntimeError("SDK exploded")
        return self.right

    def get_left_hand_is_active(self):
        return self.left_active

    def get_right_hand_is_active(self):
        return self.right_active


def wait_until(predicate, timeout=4.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def test_skeleton_packet_round_trip():
    skeleton = synthetic_skeleton()
    payload = encode_skeleton_packet("s", 3, 1234.5, "left", 1, skeleton)
    assert len(payload) <= MAX_DATAGRAM_BYTES
    packet = decode_skeleton_packet(payload)
    assert packet.side == "left"
    assert packet.sequence == 3
    assert packet.is_active == 1
    assert packet.joints.shape == (26, 7)
    # Rounding must not lose anything the tracker could resolve.
    assert np.allclose(packet.joints, skeleton, atol=1e-6)


def test_skeleton_packet_preserves_inactive_frames():
    # The receiver has to see tracking drop in order to stop commanding, so an
    # inactive frame is data, not something to suppress.
    packet = decode_skeleton_packet(
        encode_skeleton_packet("s", 0, 1.0, "right", 0, synthetic_skeleton())
    )
    assert packet.is_active == 0


@pytest.mark.parametrize(
    "kwargs",
    [
        {"side": "middle"},
        {"stream_id": ""},
        {"sequence": -1},
        {"timestamp": float("nan")},
    ],
)
def test_skeleton_packet_rejects_invalid_headers(kwargs):
    base = {
        "stream_id": "s",
        "sequence": 0,
        "timestamp": 1.0,
        "side": "left",
        "is_active": 1,
        "joints": synthetic_skeleton(),
    }
    base.update(kwargs)
    with pytest.raises(ValueError):
        encode_skeleton_packet(**base)


def test_skeleton_packet_rejects_bad_geometry():
    with pytest.raises(ValueError):
        encode_skeleton_packet("s", 0, 1.0, "left", 1, np.zeros((21, 3)))
    bad = synthetic_skeleton()
    bad[4, 2] = np.inf
    with pytest.raises(ValueError):
        encode_skeleton_packet("s", 0, 1.0, "left", 1, bad)


def test_decode_rejects_junk_and_oversize():
    with pytest.raises(ValueError):
        decode_skeleton_packet(b"not json")
    with pytest.raises(ValueError):
        decode_skeleton_packet(b"x" * (MAX_DATAGRAM_BYTES + 1))


def test_forwarder_validates_its_arguments():
    with pytest.raises(ValueError):
        HandSkeletonForwarder(None, host="127.0.0.1", port=1, rate=30.0)
    with pytest.raises(ValueError):
        HandSkeletonForwarder(FakeXrt(), host="127.0.0.1", port=1, rate=0.0)
    with pytest.raises(ValueError):
        HandSkeletonForwarder(FakeXrt(), host="127.0.0.1", port=1, rate=500.0)
    with pytest.raises(ValueError):
        HandSkeletonForwarder(
            FakeXrt(), host="127.0.0.1", port=1, rate=30.0, sides=("middle",)
        )


def test_forwarder_sends_decodable_packets_for_both_sides():
    import socket

    sink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sink.bind(("127.0.0.1", 0))
    sink.settimeout(2.0)
    port = sink.getsockname()[1]
    xrt = FakeXrt()
    forwarder = HandSkeletonForwarder(
        xrt, host="127.0.0.1", port=port, rate=60.0
    )
    seen = {}
    try:
        forwarder.start()
        deadline = time.monotonic() + 4.0
        while len(seen) < 2 and time.monotonic() < deadline:
            payload, _ = sink.recvfrom(MAX_DATAGRAM_BYTES + 1)
            packet = decode_skeleton_packet(payload)
            seen[packet.side] = packet
    finally:
        forwarder.stop()
        sink.close()
    assert set(seen) == {"left", "right"}
    assert all(p.joints.shape == (26, 7) for p in seen.values())
    status = forwarder.snapshot()
    assert status.errors == 0
    assert all(s.forwarded > 0 for s in status.sides.values())


def test_forwarder_contains_sdk_failures():
    # A thrown SDK read must be recorded and survived, never raised into the arm
    # loop that owns this object.
    xrt = FakeXrt()
    xrt.raise_on_read = True
    forwarder = HandSkeletonForwarder(
        xrt, host="127.0.0.1", port=1, rate=60.0
    )
    try:
        forwarder.start()
        assert wait_until(lambda: forwarder.snapshot().errors > 0)
    finally:
        forwarder.stop()
    status = forwarder.snapshot()
    assert "failed" in (status.last_error or "")
    assert all(s.forwarded == 0 for s in status.sides.values())


def test_forwarder_reports_inactive_without_stopping():
    import socket

    sink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sink.bind(("127.0.0.1", 0))
    sink.settimeout(2.0)
    port = sink.getsockname()[1]
    xrt = FakeXrt()
    xrt.left_active = 0
    forwarder = HandSkeletonForwarder(xrt, host="127.0.0.1", port=port, rate=60.0)
    try:
        forwarder.start()
        assert wait_until(
            lambda: forwarder.snapshot().sides["left"].forwarded > 2
        )
    finally:
        forwarder.stop()
        sink.close()
    status = forwarder.snapshot()
    # Still forwarded, but flagged inactive so the receiver can stop commanding.
    assert status.sides["left"].forwarded > 0
    assert status.sides["left"].active is False
    assert status.sides["right"].active is True
    assert status.errors == 0


def test_forwarder_cannot_be_started_twice():
    forwarder = HandSkeletonForwarder(
        FakeXrt(), host="127.0.0.1", port=1, rate=60.0
    )
    try:
        forwarder.start()
        with pytest.raises(RuntimeError):
            forwarder.start()
    finally:
        forwarder.stop()
