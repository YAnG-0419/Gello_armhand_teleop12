"""Atomic task Home storage for measured Wuji Hand 2 joint positions."""

from __future__ import annotations

import math
import os
from pathlib import Path
import tempfile
import threading
from typing import Iterable

import yaml

from operator_tasks import OPERATOR_TASKS, require_operator_task

SIDES = ("left", "right")
WUJI_HAND_2_JOINT_COUNT = 20


def _positions(values: Iterable[object]) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != WUJI_HAND_2_JOINT_COUNT or not all(
        math.isfinite(value) for value in result
    ):
        raise ValueError("Wuji Hand 2 Home must contain 20 finite joint positions")
    return result


class HandHomeStore:
    def __init__(self, data_root: str | Path) -> None:
        self.path = Path(data_root) / "operator_gui" / "hand_poses.yaml"
        self._lock = threading.RLock()

    def load_side(self, task: object, side: str) -> tuple[float, ...]:
        selected = require_operator_task(task)
        if side not in SIDES:
            raise ValueError(f"invalid hand side: {side}")
        with self._lock:
            document = self._read()
            value = document["tasks"][selected]["hands"].get(side)
        if value is None:
            raise RuntimeError(f"{OPERATOR_TASKS[selected]} {side} hand Home is not recorded")
        return _positions(value)

    def load(self, task: object, sides=SIDES) -> dict[str, tuple[float, ...]]:
        return {side: self.load_side(task, side) for side in sides}

    def save_side(self, task: object, side: str, positions: Iterable[object]) -> Path:
        selected = require_operator_task(task)
        if side not in SIDES:
            raise ValueError(f"invalid hand side: {side}")
        values = _positions(positions)
        with self._lock:
            document = self._read()
            document["tasks"][selected]["hands"][side] = list(values)
            self._write(document)
        return self.path

    def _empty(self) -> dict:
        return {
            "version": 1,
            "model": "wuji_hand_2",
            "tasks": {
                task: {
                    "label": label,
                    "hands": {side: None for side in SIDES},
                }
                for task, label in OPERATOR_TASKS.items()
            },
        }

    def _read(self) -> dict:
        if not self.path.exists():
            return self._empty()
        with self.path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        if (
            not isinstance(document, dict)
            or document.get("version") != 1
            or document.get("model") != "wuji_hand_2"
        ):
            raise ValueError("invalid Wuji Hand 2 Home document")
        tasks = document.get("tasks")
        if not isinstance(tasks, dict) or set(tasks) != set(OPERATOR_TASKS):
            raise ValueError("hand Home document must contain all operator tasks")
        for task in OPERATOR_TASKS:
            entry = tasks[task]
            hands = entry.get("hands") if isinstance(entry, dict) else None
            if not isinstance(hands, dict) or set(hands) != set(SIDES):
                raise ValueError(f"invalid hand Home task: {task}")
            for side in SIDES:
                if hands[side] is not None:
                    _positions(hands[side])
        return document

    def _write(self, document: dict) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(
            prefix=f".{self.path.name}.", suffix=".tmp", dir=self.path.parent
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                yaml.safe_dump(document, stream, allow_unicode=True, sort_keys=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
        except BaseException:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass
            raise
