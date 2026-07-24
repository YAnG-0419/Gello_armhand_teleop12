import pytest

from teleop_core.contract import LEFT_COMMAND_JOINT_NAMES
from teleop_core.protocol import JointPacket, decode_packet, encode_packet


def test_packet_round_trip():
    packet = JointPacket(
        "command",
        "session",
        4,
        2.5,
        ("left",),
        LEFT_COMMAND_JOINT_NAMES,
        tuple(0.0 for _ in range(7)),
    )
    assert decode_packet(encode_packet(packet)) == packet


def test_noncanonical_names_are_rejected():
    packet = JointPacket(
        "command",
        "session",
        4,
        2.5,
        ("left",),
        tuple(reversed(LEFT_COMMAND_JOINT_NAMES)),
        tuple(0.0 for _ in range(7)),
    )
    with pytest.raises(ValueError, match="canonical"):
        decode_packet(encode_packet(packet))
