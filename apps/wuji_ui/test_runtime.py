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
        canonical = np.asarray(
            [
                [0.00, 0.00, 0.00],
                [0.02, 0.00, 0.00], [0.04, 0.00, 0.00],
                [0.06, 0.00, 0.00], [0.08, 0.00, 0.00],
                [0.03, 0.04, 0.00], [0.03, 0.06, 0.00],
                [0.03, 0.08, 0.00], [0.03, 0.10, 0.00],
                [0.00, 0.05, 0.00], [0.00, 0.075, 0.00],
                [0.00, 0.10, 0.00], [0.00, 0.125, 0.00],
                [-0.03, 0.04, 0.00], [-0.03, 0.06, 0.00],
                [-0.03, 0.08, 0.00], [-0.03, 0.10, 0.00],
                [-0.06, 0.03, 0.00], [-0.06, 0.05, 0.00],
                [-0.06, 0.07, 0.00], [-0.06, 0.09, 0.00],
            ]
        )
        raw = np.zeros((25, 3), dtype=float)
        raw[[0, 1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14,
             16, 17, 18, 19, 21, 22, 23, 24]] = canonical
        points = [
            SimpleNamespace(
                position_x=point[0], position_y=point[1], position_z=point[2]
            )
            for point in raw
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
        assert gaps.gaps_m["index"] == pytest.approx(np.hypot(0.05, 0.10))
        assert len(gaps.features) == 12
        assert "index_curl_rad" in gaps.features

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


def test_runtime_jog_follows_slider_target_with_rate_limit(monkeypatch) -> None:
    monkeypatch.setattr(runtime_module, "WujiHandPipeline", FakePipeline)
    runtime = WujiUiRuntime(
        addresses={"left": "left:1", "right": "right:2"},
        pose_speed_rad_s=10.0,
    )
    try:
        deadline = time.monotonic() + 0.5
        while True:
            try:
                runtime.snapshot("right")
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.01)

        with pytest.raises(RuntimeError, match="不在滑块控制模式"):
            runtime.set_jog_target("right", [0.5] * 20)

        runtime.start_jog("right")
        assert runtime.mode("right") == "jog"
        assert runtime.pipeline.backends["right"].enabled
        runtime.set_jog_target("right", [0.5] * 20)

        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            sent = runtime.pipeline.backends["right"].sent
            if sent and np.allclose(sent[-1], 0.5):
                break
            time.sleep(0.01)
        else:
            raise AssertionError("jog target was not reached")
        assert runtime.mode("right") == "jog"

        runtime.set_manual("right")
        assert runtime.mode("right") == "manual"
        assert not runtime.pipeline.backends["right"].enabled
    finally:
        runtime.close()


def test_runtime_jog_rejects_out_of_limit_target(monkeypatch) -> None:
    monkeypatch.setattr(runtime_module, "WujiHandPipeline", FakePipeline)
    runtime = WujiUiRuntime(addresses={"left": "left:1", "right": "right:2"})
    try:
        deadline = time.monotonic() + 0.5
        while True:
            try:
                runtime.snapshot("left")
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.01)
        runtime.start_jog("left")
        with pytest.raises(ValueError, match="关节范围"):
            runtime.set_jog_target("left", [9.0] * 20)
    finally:
        runtime.close()
