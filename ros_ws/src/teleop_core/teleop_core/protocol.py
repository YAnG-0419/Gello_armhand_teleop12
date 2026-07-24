import json
import math
from dataclasses import dataclass
from typing import Literal

from .contract import COMMAND_JOINT_NAMES, SIDES, command_names


PROTOCOL_VERSION = 1
MAX_PACKET_BYTES = 16_384


@dataclass(frozen=True)
class JointPacket:
    kind: Literal["command", "state"]
    stream_id: str
    sequence: int
    timestamp: float
    active_sides: tuple[str, ...]
    names: tuple[str, ...]
    positions: tuple[float, ...]


def encode_packet(packet: JointPacket) -> bytes:
    payload = {
        "version": PROTOCOL_VERSION,
        "kind": packet.kind,
        "stream_id": packet.stream_id,
        "sequence": packet.sequence,
        "timestamp": packet.timestamp,
        "active_sides": list(packet.active_sides),
        "names": list(packet.names),
        "positions": list(packet.positions),
    }
    encoded = json.dumps(payload, separators=(",", ":")).encode("utf-8")
    if len(encoded) > MAX_PACKET_BYTES:
        raise ValueError("Joint packet exceeds the datagram limit.")
    return encoded


def decode_packet(payload: bytes) -> JointPacket:
    if len(payload) > MAX_PACKET_BYTES:
        raise ValueError("Joint packet exceeds the datagram limit.")
    try:
        message = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("Joint packet is not valid JSON.") from exc
    required = {
        "version",
        "kind",
        "stream_id",
        "sequence",
        "timestamp",
        "active_sides",
        "names",
        "positions",
    }
    if not isinstance(message, dict) or set(message) != required:
        raise ValueError("Joint packet has an invalid schema.")
    if message["version"] != PROTOCOL_VERSION:
        raise ValueError("Unsupported joint protocol version.")
    if message["kind"] not in {"command", "state"}:
        raise ValueError("Joint packet kind must be command or state.")
    if not isinstance(message["stream_id"], str) or not message["stream_id"]:
        raise ValueError("Joint packet stream ID is invalid.")
    try:
        sequence = int(message["sequence"])
        timestamp = float(message["timestamp"])
        positions = tuple(float(value) for value in message["positions"])
    except (TypeError, ValueError) as exc:
        raise ValueError("Joint packet contains a non-numeric field.") from exc
    names = tuple(str(name) for name in message["names"])
    active_sides = tuple(str(side) for side in message["active_sides"])
    if sequence < 0 or not math.isfinite(timestamp):
        raise ValueError("Joint packet sequence or timestamp is invalid.")
    if set(active_sides).difference(SIDES):
        raise ValueError("Joint packet contains an invalid active side.")
    if len(active_sides) != len(set(active_sides)):
        raise ValueError("Joint packet contains duplicate active sides.")
    if len(names) != len(positions) or len(names) != len(set(names)):
        raise ValueError("Joint packet names and positions do not match.")
    if not all(math.isfinite(value) for value in positions):
        raise ValueError("Joint packet contains a non-finite position.")
    if set(names).difference(COMMAND_JOINT_NAMES):
        raise ValueError("Joint packet contains an unknown joint name.")
    expected = COMMAND_JOINT_NAMES if message["kind"] == "state" else command_names(active_sides)
    if names != expected:
        raise ValueError("Joint packet names are not in canonical order.")
    return JointPacket(
        kind=message["kind"],
        stream_id=message["stream_id"],
        sequence=sequence,
        timestamp=timestamp,
        active_sides=active_sides,
        names=names,
        positions=positions,
    )
