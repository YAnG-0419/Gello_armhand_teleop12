import time
from types import SimpleNamespace

import numpy as np
import pytest

import apps.wuji_ui.runtime as runtime_module
from apps.wuji_ui.runtime import WujiUiRuntime


class FakeBackend:
    def __init__(self) -> None:
        self.enabled = False
        self.sent = []

    def send(self, command) -> None:
        assert self.enabled
        self.sent.append(np.asarray(command).copy())


class FakePipeline:
    def __init__(self, **kwargs) -> None:
        assert kwargs["auto_enable"] is False
        self.joint_names = {
            side: tuple(f"{side}_j{i}" for i in range(20))
            for side in ("left", "right")
        }
        self.joint_limits = {
            side: tuple((-1.0, 1.5) for _ in range(20))
            for side in ("left", "right")
        }
        self.backends = {side: FakeBackend() for side in ("left", "right")}
        points = [
            SimpleNamespace(
                position_x=index * 0.001,
                position_y=0.0,
                position_z=0.0,
            )
            for index in range(25)
        ]
        self.last_frames = {
            side: SimpleNamespace(sequence=1, keypoints=points)
            for side in ("left", "right")
        }
        self.closed = False
        self.status = SimpleNamespace(
            sides={
                side: SimpleNamespace(fault=None, sending=False)
                for side in ("left", "right")
            }
        )

    def set_enabled(self, side, enabled) -> None:
        self.backends[side].enabled = enabled

    def feedback_position(self, side):
        return np.full(20, 0.1 if side == "left" else 0.2)

    def tick(self, _now, active) -> None:
        for side, enabled in active.items():
            if enabled:
                self.backends[side].send(np.full(20, 0.3))

    def close(self) -> None:
        self.closed = True


def test_runtime_starts_manual_and_switches_each_side_independently(monkeypatch) -> None:
    monkeypatch.setattr(runtime_module, "WujiHandPipeline", FakePipeline)
    runtime = WujiUiRuntime(addresses={"left": "left:1", "right": "right:2"})
    try:
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            try:
                runtime.snapshot("left")
                break
            except RuntimeError:
                time.sleep(0.01)
        assert runtime.mode("left") == "manual"
        assert runtime.mode("right") == "manual"
        deadline = time.monotonic() + 0.5
        while True:
            try:
                gaps = runtime.manus_gaps("left")
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.01)
        assert gaps.gaps_m["index"] == pytest.approx(0.005)

        runtime.start_teleop("left")
        assert runtime.mode("left") == "teleop"
        assert runtime.mode("right") == "manual"
        assert runtime.pipeline.backends["left"].enabled
        assert not runtime.pipeline.backends["right"].enabled

        runtime.move_to_pose("right", [0.4] * 20)
        assert runtime.mode("right") == "pose"
        assert runtime.pipeline.backends["right"].enabled

        runtime.set_manual("left")
        assert runtime.mode("left") == "manual"
        assert not runtime.pipeline.backends["left"].enabled
    finally:
        pipeline = runtime.pipeline
        runtime.close()
    assert pipeline.closed
