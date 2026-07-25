"""Host-side encoder for hand qpos datagrams.

The hand stream is deliberately separate from the arm's joint protocol in
`teleop_core.protocol`. The arm protocol validates against FR3 joint names and
is owned by the arm safety gateway; hand commands travel their own path to their
own bridge, on their own port, so that a fault in one stream cannot be mistaken
for a fault in the other. Losing the optical skeleton must not look like losing
a wrist tracker.

Values are URDF joint angles in radians, keyed by name. The 0..255 vendor
conversion happens in the ROS bridge, so nothing on the robot side of the wire
needs to know about PICO and nothing on this side needs to know about the
vendor's slot layout.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Sequence

PROTOCOL_VERSION = 1
MAX_DATAGRAM_BYTES = 16_384
SIDES = ("left", "right")


@dataclass(frozen=True)
class HandQposPacket:
    stream_id: str
    sequence: int
    timestamp: float
    side: str
    joint_names: tuple[str, ...]
    qpos: tuple[float, ...]


def encode_hand_packet(packet: HandQposPacket) -> bytes:
    """Serialize one side's URDF pose into a datagram."""
    if packet.side not in SIDES:
        raise ValueError(f"side must be left or right, got {packet.side!r}")
    if not packet.stream_id:
        raise ValueError("stream_id must be non-empty")
    if packet.sequence < 0:
        raise ValueError("sequence must be non-negative")
    if len(packet.joint_names) != len(packet.qpos):
        raise ValueError("joint_names and qpos lengths differ")
    if len(set(packet.joint_names)) != len(packet.joint_names):
        raise ValueError("joint_names contains duplicates")
    if not math.isfinite(packet.timestamp):
        raise ValueError("timestamp must be finite")
    if not all(math.isfinite(value) for value in packet.qpos):
        raise ValueError("qpos contains a non-finite value")

    payload = {
        "version": PROTOCOL_VERSION,
        "stream_id": packet.stream_id,
        "sequence": int(packet.sequence),
        "timestamp": float(packet.timestamp),
        "side": packet.side,
        "joint_names": list(packet.joint_names),
        "qpos": [float(value) for value in packet.qpos],
    }
    encoded = json.dumps(payload, separators=(",", ":")).encode("utf-8")
    if len(encoded) > MAX_DATAGRAM_BYTES:
        raise ValueError("Hand qpos packet exceeds the datagram limit.")
    return encoded


def build_hand_packet(
    stream_id: str,
    sequence: int,
    timestamp: float,
    side: str,
    joint_names: Sequence[str],
    qpos: Sequence[float],
) -> bytes:
    """Convenience wrapper returning an encoded datagram."""
    return encode_hand_packet(
        HandQposPacket(
            stream_id=str(stream_id),
            sequence=int(sequence),
            timestamp=float(timestamp),
            side=str(side),
            joint_names=tuple(str(name) for name in joint_names),
            qpos=tuple(float(value) for value in qpos),
        )
    )
