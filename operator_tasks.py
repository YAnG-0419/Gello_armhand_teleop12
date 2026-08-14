"""Stable task identifiers shared by the operator and recording UIs."""

from __future__ import annotations


OPERATOR_TASKS = {
    "powder_weighing": "粉末称量",
    "assembly": "装配",
    "bean_picking": "夹豆",
}
DEFAULT_OPERATOR_TASK = "powder_weighing"


def require_operator_task(value: object) -> str:
    task = str(value)
    if task not in OPERATOR_TASKS:
        raise ValueError(f"unknown operator task: {task!r}")
    return task
