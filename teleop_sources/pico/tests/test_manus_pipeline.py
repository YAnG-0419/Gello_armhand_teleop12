import json
import socket
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
# linker_hand_bridge is a ROS package, not pip-installed in this env;
# manus_teleop is an editable install (see scripts/setup_pico_env.sh).
sys.path.insert(0, str(REPO_ROOT / "ros_ws" / "src" / "linker_hand_bridge"))

from linker_hand_bridge.core import G20Mapper
from manus_teleop.pipeline import ManusFrame, ManusHandPipeline, SIDE_CODES
from manus_teleop import o30i_retarget
from pico_bimanual_franka_teleop import hand_retarget


class FakeBridge:
    """Per-side frame source; sides outside `serving` never deliver."""

    serving = ("left", "right")

    def __init__(self, _library: Path) -> None:
        self.connected = False
        self.closed = False
        self.sequence = 0

    def connect(self, _calibration_dir: Path) -> None:
        self.connected = True

    def available_sides(self) -> tuple[str, ...]:
        return tuple(self.serving)

    def read(self, side: str, _timeout_s: float = 0.0) -> ManusFrame | None:
        if side not in self.serving:
            return None
        frame = ManusFrame()
        frame.side = SIDE_CODES[side]
        frame.keypoint_count = 25
        frame.sequence = self.sequence
        self.sequence += 1
        return frame

    def close(self) -> None:
        self.closed = True


class RightOnlyFakeBridge(FakeBridge):
    serving = ("right",)


class FakeRetargeter:
    joint_names = [
        "index_mcp_roll",
        "index_mcp_pitch",
        "index_pip",
        "index_dip",
        "middle_mcp_roll",
        "middle_mcp_pitch",
        "middle_pip",
        "middle_dip",
        "pinky_mcp_roll",
        "pinky_mcp_pitch",
        "pinky_pip",
        "pinky_dip",
        "ring_mcp_roll",
        "ring_mcp_pitch",
        "ring_pip",
        "ring_dip",
        "thumb_cmc_yaw",
        "thumb_cmc_roll",
        "thumb_cmc_pitch",
        "thumb_mcp",
        "thumb_dip",
    ]

    def __init__(self, *_args, **_kwargs) -> None:
        self.closed = False
        self.reset_count = 0

    def retarget(self, landmarks):
        assert np.asarray(landmarks).shape == (21, 3)
        return np.zeros(len(self.joint_names)), {}

    def reset(self) -> None:
        self.reset_count += 1

    def close(self) -> None:
        self.closed = True


class FakeO30IRetargeter(FakeRetargeter):
    joint_names = [
        "thumb_cmc_roll",
        "thumb_cmc_yaw",
        "thumb_mcp",
        "thumb_ip",
        "index_mcp_roll",
        "index_mcp_pitch",
        "index_pip",
        "index_dip",
        "middle_mcp_roll",
        "middle_mcp_pitch",
        "middle_pip",
        "middle_dip",
        "ring_mcp_roll",
        "ring_mcp_pitch",
        "ring_pip",
        "ring_dip",
        "pinky_mcp_roll",
        "pinky_mcp_pitch",
        "pinky_pip",
        "pinky_dip",
    ]


def _drain(sink: socket.socket) -> list[dict]:
    messages = []
    sink.settimeout(0.02)
    while True:
        try:
            payload, _ = sink.recvfrom(65536)
        except TimeoutError:
            return messages
        messages.append(json.loads(payload))


def _sink() -> socket.socket:
    sink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sink.bind(("127.0.0.1", 0))
    return sink


def test_manus_right_only_keeps_left_default_and_shared_activation(
    monkeypatch,
) -> None:
    monkeypatch.setattr(hand_retarget, "L20Retargeter", FakeRetargeter)
    sink = _sink()
    pipeline = ManusHandPipeline(
        host="127.0.0.1",
        port=sink.getsockname()[1],
        bridge_factory=RightOnlyFakeBridge,
        dynamic_sides=("right",),
    )
    try:
        for step in range(20):
            pipeline.tick(
                100.0 + step * 0.01,
                active={"left": False, "right": False},
            )
        inactive_messages = _drain(sink)
        assert inactive_messages
        assert all(
            message["side"] == "left"
            and message["stream_id"] == "manus-left-default"
            and all(value == 0.0 for value in message["qpos"])
            for message in inactive_messages
        )
        assert pipeline.status.sides["right"].fault == "disengaged by operator"

        for step in range(100):
            pipeline.tick(
                101.0 + step * 0.01,
                active={"left": False, "right": True},
            )
        messages = _drain(sink)
        by_side = {
            side: [message for message in messages if message["side"] == side]
            for side in ("left", "right")
        }
        assert 29 <= len(by_side["left"]) <= 31
        assert 29 <= len(by_side["right"]) <= 31
        assert all(
            message["stream_id"] == "manus-left-default"
            for message in by_side["left"]
        )
        left_default = by_side["left"][0]
        mapped_left = G20Mapper(abduction_invert=True).map_qpos(
            "left",
            left_default["joint_names"],
            left_default["qpos"],
        )
        assert len(mapped_left) == 20
        assert all(
            message["stream_id"] == "manus-right-g20"
            for message in by_side["right"]
        )

        for step in range(20):
            pipeline.tick(
                103.0 + step * 0.01,
                active={"left": False, "right": False},
            )
        stopped_messages = _drain(sink)
        assert stopped_messages
        assert all(message["side"] == "left" for message in stopped_messages)
        assert pipeline.status.sides["right"].fault == "disengaged by operator"
        assert pipeline.retargeters["right"].reset_count == 1
    finally:
        bridge = pipeline.bridge
        retargeter = pipeline.retargeters["right"]
        pipeline.close()
        sink.close()
    assert bridge.closed
    assert retargeter.closed


def test_manus_pipeline_selects_o30i_only_for_right(monkeypatch) -> None:
    monkeypatch.setattr(o30i_retarget, "O30IRetargeter", FakeO30IRetargeter)
    sink = _sink()
    pipeline = ManusHandPipeline(
        host="127.0.0.1",
        port=sink.getsockname()[1],
        bridge_factory=RightOnlyFakeBridge,
        dynamic_sides=("right",),
        models={"left": "g20", "right": "o30i"},
    )
    try:
        for step in range(10):
            pipeline.tick(
                200.0 + step * 0.04,
                active={"left": False, "right": True},
            )
        messages = _drain(sink)
        right = [message for message in messages if message["side"] == "right"]
        assert right
        assert all(message["model"] == "o30i" for message in right)
        assert all(
            message["joint_names"] == FakeO30IRetargeter.joint_names
            for message in right
        )
        left = [message for message in messages if message["side"] == "left"]
        assert left
        assert all(message["model"] == "g20" for message in left)
        assert all(len(message["joint_names"]) == 21 for message in left)
    finally:
        pipeline.close()
        sink.close()


def test_manus_bimanual_follows_both_sides(monkeypatch) -> None:
    monkeypatch.setattr(hand_retarget, "L20Retargeter", FakeRetargeter)
    monkeypatch.setattr(o30i_retarget, "O30IRetargeter", FakeO30IRetargeter)
    sink = _sink()
    pipeline = ManusHandPipeline(
        host="127.0.0.1",
        port=sink.getsockname()[1],
        bridge_factory=FakeBridge,
        models={"left": "g20", "right": "o30i"},
    )
    try:
        for step in range(100):
            pipeline.tick(
                300.0 + step * 0.01,
                active={"left": True, "right": True},
            )
        messages = _drain(sink)
        by_side = {
            side: [message for message in messages if message["side"] == side]
            for side in ("left", "right")
        }
        # Both sides stream at the configured 30 Hz; the one-solve-per-tick
        # round-robin must not starve either.
        assert 25 <= len(by_side["left"]) <= 31
        assert 25 <= len(by_side["right"]) <= 31
        assert all(
            message["stream_id"] == "manus-left-g20"
            and message["model"] == "g20"
            and len(message["joint_names"]) == 21
            for message in by_side["left"]
        )
        assert all(
            message["stream_id"] == "manus-right-o30i"
            and message["model"] == "o30i"
            for message in by_side["right"]
        )

        # Disengaging one side must reset only that side's filter history.
        for step in range(20):
            pipeline.tick(
                302.0 + step * 0.01,
                active={"left": False, "right": True},
            )
        assert pipeline.retargeters["left"].reset_count == 1
        assert pipeline.retargeters["right"].reset_count == 0
        assert pipeline.status.sides["left"].fault == "disengaged by operator"
    finally:
        pipeline.close()
        sink.close()


def test_manus_bimanual_reports_a_missing_left_glove(monkeypatch) -> None:
    monkeypatch.setattr(hand_retarget, "L20Retargeter", FakeRetargeter)
    monkeypatch.setattr(o30i_retarget, "O30IRetargeter", FakeO30IRetargeter)
    sink = _sink()
    pipeline = ManusHandPipeline(
        host="127.0.0.1",
        port=sink.getsockname()[1],
        bridge_factory=RightOnlyFakeBridge,
        models={"left": "g20", "right": "o30i"},
    )
    try:
        for step in range(50):
            pipeline.tick(
                400.0 + step * 0.01,
                active={"left": True, "right": True},
            )
        messages = _drain(sink)
        # The uncalibrated left glove never blocks the right hand.
        assert all(message["side"] == "right" for message in messages)
        assert messages
        assert "Calibration_left.mcal" in pipeline.status.sides["left"].fault
        assert pipeline.status.sides["right"].sending
    finally:
        pipeline.close()
        sink.close()
