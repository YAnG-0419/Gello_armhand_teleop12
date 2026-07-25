"""Datagram carrying a raw PICO hand skeleton between processes.

Retargeting and arm control cannot share a process. Measured on this workstation,
solving both hands costs about 17 ms and holds the GIL while doing it, which turned
a rock-steady 100 Hz arm loop (median 10.07 ms, p99 10.18 ms) into one that missed
its deadline on 23% of ticks and was late by up to 43 ms. Fewer optimizer
iterations only halved that.

So the split is here, at the skeleton, rather than at the SDK. The arm process owns
the single SDK client and forwards skeletons, which costs well under a millisecond;
a separate process does the retargeting with its own interpreter and its own GIL.
The rule that only one process may open an SDK client is preserved, because the
retargeting process never touches the SDK.

This carries the skeleton as delivered by the binding, not landmarks, so that the
receiving process applies the same validation and liveness rules it would apply to
a live client and nothing is silently normalized on the way across.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass

import numpy as np

PROTOCOL_VERSION = 1
MAX_DATAGRAM_BYTES = 65_507  # the practical maximum for a single UDP payload
OPENXR_JOINT_COUNT = 26
POSE_WIDTH = 7
SIDES = ("left", "right")


@dataclass(frozen=True)
class SkeletonPacket:
    stream_id: str
    sequence: int
    timestamp: float
    side: str
    is_active: int
    joints: np.ndarray


def encode_skeleton_packet(
    stream_id: str,
    sequence: int,
    timestamp: float,
    side: str,
    is_active: int,
    joints: np.ndarray,
) -> bytes:
    if side not in SIDES:
        raise ValueError(f"side must be left or right, got {side!r}")
    if not stream_id:
        raise ValueError("stream_id must be non-empty")
    if sequence < 0:
        raise ValueError("sequence must be non-negative")
    if not math.isfinite(timestamp):
        raise ValueError("timestamp must be finite")
    array = np.asarray(joints, dtype=float)
    if array.shape != (OPENXR_JOINT_COUNT, POSE_WIDTH):
        raise ValueError(
            f"joints must have shape {(OPENXR_JOINT_COUNT, POSE_WIDTH)}, "
            f"got {array.shape}"
        )
    if not np.isfinite(array).all():
        raise ValueError("joints contain NaN or infinity")

    payload = {
        "version": PROTOCOL_VERSION,
        "stream_id": str(stream_id),
        "sequence": int(sequence),
        "timestamp": float(timestamp),
        "side": side,
        "is_active": int(is_active),
        # Rounded to a tenth of a millimetre, far finer than the tracker resolves,
        # to keep the datagram small without affecting retargeting.
        "joints": [[round(float(v), 7) for v in row] for row in array],
    }
    encoded = json.dumps(payload, separators=(",", ":")).encode("utf-8")
    if len(encoded) > MAX_DATAGRAM_BYTES:
        raise ValueError("Skeleton packet exceeds the datagram limit.")
    return encoded


def decode_skeleton_packet(payload: bytes) -> SkeletonPacket:
    if len(payload) > MAX_DATAGRAM_BYTES:
        raise ValueError(f"packet is too large: {len(payload)} bytes")
    try:
        message = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("invalid skeleton packet JSON") from error
    if not isinstance(message, dict):
        raise ValueError("skeleton packet JSON must be an object")

    required = {
        "version",
        "stream_id",
        "sequence",
        "timestamp",
        "side",
        "is_active",
        "joints",
    }
    missing = required.difference(message)
    if missing:
        raise ValueError(f"skeleton packet is missing fields: {sorted(missing)}")
    if message["version"] != PROTOCOL_VERSION:
        raise ValueError(f"unsupported protocol version: {message['version']}")
    if message["side"] not in SIDES:
        raise ValueError(f"invalid hand side: {message['side']!r}")
    try:
        sequence = int(message["sequence"])
        timestamp = float(message["timestamp"])
        is_active = int(message["is_active"])
        joints = np.asarray(message["joints"], dtype=float)
    except (TypeError, ValueError) as error:
        raise ValueError("skeleton packet contains a non-numeric field") from error
    if sequence < 0 or not math.isfinite(timestamp):
        raise ValueError("skeleton packet sequence or timestamp is invalid")
    if joints.shape != (OPENXR_JOINT_COUNT, POSE_WIDTH):
        raise ValueError(f"skeleton packet joints have shape {joints.shape}")
    if not np.isfinite(joints).all():
        raise ValueError("skeleton packet joints contain NaN or infinity")
    return SkeletonPacket(
        stream_id=str(message["stream_id"]),
        sequence=sequence,
        timestamp=timestamp,
        side=str(message["side"]),
        is_active=is_active,
        joints=joints,
    )
