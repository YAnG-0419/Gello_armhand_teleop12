"""The teleop operator's control backend: console state + JSON-TCP server.

One unified backend, swappable frontends. `OperatorConsole` is the single
activation/request/status object the input sources and the control loop
share; `OperatorControlServer` exposes it over line-delimited JSON
({"id", "command", "arguments"} answered by {"id", "ok",
"result"|"error"}, gluon's demonstration-server protocol). The PySide6
GUI in apps/operator_gui is a shell over this protocol and is the
operator frontend; the process itself is headless.

The server thread never touches robot state directly. Commands mutate the
console's activation and one-shot request flags; the 100 Hz loop reads
them at its own pace, exactly as it read the old keyboard, and `status`
returns the loop's latest published snapshot.
"""

from __future__ import annotations

import json
from pathlib import Path
import socketserver
import threading
import time

import yaml

from .relative_action import PresetAction, save_preset_task
from .types import SIDES


class OperatorConsole:
    """Thread-safe operator state shared by the GUI and coordinator.

    State arrives from the control server, and feedback accumulates in a ring
    the `status` command serves to frontends.
    """

    def __init__(
        self,
        preset_actions: dict[str, PresetAction | None] | None = None,
        *,
        preset_config: str | Path | None = None,
        preset_data_root: str | Path | None = None,
        selected_task: str | None = None,
    ) -> None:
        self.sides = SIDES
        # `active` remains the arm activation map for compatibility with every
        # existing arm input source. Hands have independent ownership so a
        # pedal can start/stop MANUS following without moving the FR3.
        self.active = {side: False for side in SIDES}
        self.hand_active = {side: False for side in SIDES}
        self.requests = {
            "open_hands": False,
            "open_left_hand": False,
            "open_right_hand": False,
            "reset": False,
            "reset_left": False,
            "reset_right": False,
            "capture_home_left": False,
            "capture_home_right": False,
            "preset_q": False,
            "preset_w": False,
            "preset_e": False,
            "preset_task": None,
            "stop_action": False,
            "abort_action": False,
        }
        self._status_line = "starting..."
        self._feedback: list[str] = []
        self._lock = threading.Lock()
        self._preset_actions = preset_actions if preset_actions is not None else {}
        self._preset_config = Path(preset_config) if preset_config is not None else None
        self._preset_data_root = (
            Path(preset_data_root) if preset_data_root is not None else None
        )
        self._available_actions = self._scan_actions()
        named = [key for key in self._preset_actions if key not in {"q", "w", "e"}]
        self._selected_task = (
            selected_task if selected_task in named else (named[0] if named else None)
        )

    # -- operator-state interface ----------------------------------------
    # Every mutation happens under the lock: the server thread writes while
    # the 100 Hz control loop reads, and the keyboard-era code was safe only
    # because both sides ran on one thread. Without the lock a request set
    # between take_requests' read and swap lands in the discarded dict and
    # the operator's click is silently lost.

    def poll(self) -> dict[str, bool]:
        with self._lock:
            return dict(self.active)

    def poll_hands(self) -> dict[str, bool]:
        with self._lock:
            return dict(self.hand_active)

    def take_requests(self) -> dict[str, object]:
        with self._lock:
            taken = self.requests
            self.requests = {
                name: None if name == "preset_task" else False for name in taken
            }
        return taken

    def set_active(self, side: str, engaged: bool, *, target: str = "both") -> None:
        if side not in self.sides:
            raise ValueError(f"{side} is not configured for this run")
        if target not in {"arm", "hand", "both"}:
            raise ValueError(f"unknown activation target: {target}")
        with self._lock:
            if target in {"arm", "both"}:
                self.active[side] = engaged
            if target in {"hand", "both"}:
                self.hand_active[side] = engaged

    def request(self, name: str) -> None:
        with self._lock:
            if name not in self.requests:
                raise ValueError(f"Unknown request: {name}")
            self.requests[name] = True

    def request_task(self, task_id: str | None = None) -> None:
        with self._lock:
            selected = str(task_id or self._selected_task or "")
            if selected not in self._preset_actions or selected in {"q", "w", "e"}:
                raise ValueError("select a configured task first")
            self.requests["preset_task"] = selected

    def select_task(self, task_id: str) -> None:
        with self._lock:
            if task_id not in self._preset_actions or task_id in {"q", "w", "e"}:
                raise ValueError(f"unknown task: {task_id}")
            self._selected_task = task_id

    def preset_action(self, task_id: str) -> PresetAction | None:
        with self._lock:
            return self._preset_actions.get(task_id)

    def configure_task(self, arguments: dict) -> dict:
        if self._preset_config is None or self._preset_data_root is None:
            raise RuntimeError("task editing is not configured on this backend")
        task = save_preset_task(
            self._preset_config,
            self._preset_data_root,
            task_id=str(arguments.get("task_id", "")),
            label=str(arguments.get("label", "")),
            side=str(arguments.get("side", "")),
            action_name=str(arguments.get("action", "")),
            speed_scale=float(arguments.get("speed_scale", 0.15)),
        )
        with self._lock:
            self._preset_actions[task.key] = task
            self._selected_task = task.key
        self.show(f"task configured: {task.label}")
        self.refresh_actions()
        return {"task_id": task.key}

    def _scan_actions(self) -> list[dict]:
        if self._preset_data_root is None:
            return []
        root = self._preset_data_root / "arm_ui" / "actions"
        result = []
        for path in sorted(root.glob("*.yaml")):
            try:
                with path.open("r", encoding="utf-8") as stream:
                    data = yaml.safe_load(stream) or {}
                side = str(data.get("side", ""))
                name = str(data.get("name", "")).strip()
                if side in SIDES and name:
                    result.append({"side": side, "name": name})
            except (OSError, TypeError, ValueError, yaml.YAMLError):
                continue
        return result

    def refresh_actions(self) -> dict:
        actions = self._scan_actions()
        with self._lock:
            self._available_actions = actions
        return {"count": len(actions)}

    def disable_all(self, reason: str) -> None:
        with self._lock:
            self.active = {side: False for side in SIDES}
            self.hand_active = {side: False for side in SIDES}
        self.show(f"all sides disengaged: {reason}")

    def deny(self, side: str, reason: str) -> None:
        """Deny the selected arm path without changing independent hand state."""
        with self._lock:
            self.active[side] = False
        self.show(f"{side} arm: {reason}")

    def deny_hand(self, side: str, reason: str) -> None:
        with self._lock:
            self.hand_active[side] = False
        self.show(f"{side} hand: {reason}")

    def show(self, message: str) -> None:
        with self._lock:
            self._feedback.append(time.strftime("%H:%M:%S ") + str(message))
            del self._feedback[:-50]

    def set_status(self, text: str) -> None:
        with self._lock:
            self._status_line = str(text)

    def close(self) -> None:
        pass

    # -- frontend-facing snapshot ----------------------------------------

    def snapshot(self) -> dict:
        with self._lock:
            available = {
                (item["side"], item["name"]) for item in self._available_actions
            }
            snapshot = {
                "status_line": self._status_line,
                "feedback": list(self._feedback),
                "active": dict(self.active),
                "hand_active": dict(self.hand_active),
                "sides": list(self.sides),
                "selected_task": self._selected_task,
                "tasks": [
                    {
                        "id": key,
                        "label": action.label,
                        "side": action.side,
                        "action": action.action_name,
                        "speed_scale": action.speed_scale,
                        "available": (action.side, action.action_name) in available,
                    }
                    for key, action in self._preset_actions.items()
                    if key not in {"q", "w", "e"} and action is not None
                ],
                "available_actions": list(self._available_actions),
            }
        return snapshot


class _RequestHandler(socketserver.StreamRequestHandler):
    def setup(self) -> None:
        super().setup()
        self.server.client_connected()

    def finish(self) -> None:
        try:
            super().finish()
        finally:
            self.server.client_disconnected()

    def handle(self) -> None:
        while True:
            line = self.rfile.readline()
            if not line:
                return
            request_id = None
            try:
                request = json.loads(line)
                if not isinstance(request, dict):
                    raise ValueError("Request must be a JSON object.")
                request_id = request.get("id")
                command = str(request.get("command", ""))
                arguments = request.get("arguments", {})
                if not isinstance(arguments, dict):
                    raise ValueError("arguments must be an object.")
                result = self.server.dispatch(command, arguments)
                response = {"id": request_id, "ok": True, "result": result}
            except Exception as exception:  # noqa: BLE001 - report to client
                response = {"id": request_id, "ok": False, "error": str(exception)}
            self.wfile.write((json.dumps(response) + "\n").encode("utf-8"))
            self.wfile.flush()


def _require_side(arguments) -> str:
    side = str(arguments.get("side", ""))
    if side not in ("left", "right"):
        raise ValueError("side must be left or right")
    return side


class OperatorControlServer(socketserver.ThreadingTCPServer):
    """Serves engage/disengage, arm-home, and hand-open commands."""

    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address, console: OperatorConsole):
        super().__init__(address, _RequestHandler)
        self.keyboard = console
        self.snapshot = console.snapshot
        self._thread: threading.Thread | None = None
        self._clients = 0
        self._clients_lock = threading.Lock()

    def start(self) -> None:
        self._thread = threading.Thread(
            target=self.serve_forever, kwargs={"poll_interval": 0.2}, daemon=True
        )
        self._thread.start()

    def close(self) -> None:
        self.shutdown()
        self.server_close()
        self.keyboard.disable_all("operator server stopped")
        self.keyboard.request("abort_action")

    def client_connected(self) -> None:
        with self._clients_lock:
            self._clients += 1

    def client_disconnected(self) -> None:
        with self._clients_lock:
            self._clients = max(0, self._clients - 1)
            last_client = self._clients == 0
        if last_client:
            self.keyboard.disable_all("operator frontend disconnected")
            self.keyboard.request("abort_action")

    # -- commands ---------------------------------------------------------

    def _disengage_all(self) -> None:
        self.keyboard.disable_all("operator frontend")
        self.keyboard.request("abort_action")

    def _scoped_request(
        self,
        arguments,
        *,
        both: str,
        left: str,
        right: str,
    ) -> None:
        scope = str(arguments.get("side", "both"))
        if scope == "both":
            self.keyboard.request(both)
        elif scope == "left":
            self.keyboard.request(left)
        elif scope == "right":
            self.keyboard.request(right)
        else:
            raise ValueError("side must be left, right, or both")

    def dispatch(self, command: str, arguments: dict):
        commands = {
            "status": self.snapshot,
            "engage": lambda: self.keyboard.set_active(
                _require_side(arguments), True, target="both"
            ),
            "disengage": lambda: self.keyboard.set_active(
                _require_side(arguments), False, target="both"
            ),
            "engage_arm": lambda: self.keyboard.set_active(
                _require_side(arguments), True, target="arm"
            ),
            "disengage_arm": lambda: self.keyboard.set_active(
                _require_side(arguments), False, target="arm"
            ),
            "engage_hand": lambda: self.keyboard.set_active(
                _require_side(arguments), True, target="hand"
            ),
            "disengage_hand": lambda: self.keyboard.set_active(
                _require_side(arguments), False, target="hand"
            ),
            "disengage_all": self._disengage_all,
            "open_hand": lambda: self._scoped_request(
                arguments,
                both="open_hands",
                left="open_left_hand",
                right="open_right_hand",
            ),
            "home_arm": lambda: self._scoped_request(
                arguments,
                both="reset",
                left="reset_left",
                right="reset_right",
            ),
            "capture_home": lambda: self.keyboard.request(
                f"capture_home_{_require_side(arguments)}"
            ),
            "run_preset": lambda: self.keyboard.request(
                f"preset_{_require_preset(arguments)}"
            ),
            "select_task": lambda: self.keyboard.select_task(
                str(arguments.get("task_id", ""))
            ),
            "run_selected_task": lambda: self.keyboard.request_task(),
            "configure_task": lambda: self.keyboard.configure_task(arguments),
            "refresh_actions": self.keyboard.refresh_actions,
            "stop_action": lambda: self.keyboard.request("stop_action"),
        }
        handler = commands.get(command)
        if handler is None:
            raise ValueError(f"Unknown command: {command}")
        result = handler()
        return {} if result is None else result


def _require_preset(arguments) -> str:
    key = str(arguments.get("key", "")).lower()
    if key not in ("q", "w", "e"):
        raise ValueError("preset key must be q, w, or e")
    return key
