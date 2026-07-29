"""The teleop operator's control backend: console state + JSON-TCP server.

One unified backend, swappable frontends. `OperatorConsole` is the single
activation/request/status object the input sources and the control loop
share; `OperatorControlServer` exposes it over line-delimited JSON
({"id", "command", "arguments"} answered by {"id", "ok",
"result"|"error"}, gluon's demonstration-server protocol). The PySide6
GUI in teleop_sources/gui is a shell over this protocol and is the
operator frontend; the process itself is headless.

The server thread never touches robot state directly. Commands mutate the
console's activation and one-shot request flags; the 100 Hz loop reads
them at its own pace, exactly as it read the old keyboard, and `status`
returns the loop's latest published snapshot.
"""

from __future__ import annotations

import json
import socketserver
import threading
import time

from .types import SIDES


class OperatorConsole:
    """Headless drop-in for the keyboard object the input sources own.

    Same interface (poll / take_requests / disable_all / deny / show /
    set_status / close), but state arrives from the control server instead
    of a tty, and feedback accumulates in a ring the `status` command
    serves to frontends.
    """

    def __init__(self, sides: tuple[str, ...] = SIDES) -> None:
        if not sides or set(sides).difference(SIDES):
            raise ValueError(f"Invalid console sides: {sides}")
        self.sides = tuple(sides)
        self.active = {side: False for side in SIDES}
        self.requests = {
            "open_hands": False,
            "reset": False,
            "reset_left": False,
            "reset_right": False,
        }
        self._status_line = "starting..."
        self._feedback: list[str] = []
        self._lock = threading.Lock()

    # -- input-source interface ------------------------------------------

    def poll(self) -> dict[str, bool]:
        return dict(self.active)

    def take_requests(self) -> dict[str, bool]:
        taken = self.requests
        self.requests = {name: False for name in taken}
        return taken

    def disable_all(self, reason: str) -> None:
        self.active = {side: False for side in SIDES}
        self.show(f"all sides disengaged: {reason}")

    def deny(self, side: str, reason: str) -> None:
        self.active[side] = False
        self.show(f"{side}: {reason}")

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
            return {
                "status_line": self._status_line,
                "feedback": list(self._feedback),
                "active": dict(self.active),
                "sides": list(self.sides),
            }


class _RequestHandler(socketserver.StreamRequestHandler):
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
    """Serves engage/disengage/open/home commands into the console."""

    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address, console: OperatorConsole):
        super().__init__(address, _RequestHandler)
        self.keyboard = console
        self.snapshot = console.snapshot
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(
            target=self.serve_forever, kwargs={"poll_interval": 0.2}, daemon=True
        )
        self._thread.start()

    def close(self) -> None:
        self.shutdown()
        self.server_close()

    # -- commands ---------------------------------------------------------

    def _set_side(self, side: str, engaged: bool) -> None:
        if side not in self.keyboard.sides:
            raise ValueError(f"{side} is not configured for this run")
        self.keyboard.active[side] = engaged

    def _disengage_all(self) -> None:
        for side in tuple(self.keyboard.active):
            self.keyboard.active[side] = False

    def _request(self, name: str) -> None:
        if name not in self.keyboard.requests:
            raise ValueError(f"Unknown request: {name}")
        self.keyboard.requests[name] = True

    def _reset(self, arguments) -> None:
        scope = str(arguments.get("side", "both"))
        if scope == "both":
            self._request("reset")
        elif scope in ("left", "right"):
            self._request(f"reset_{scope}")
        else:
            raise ValueError("side must be left, right, or both")

    def dispatch(self, command: str, arguments: dict):
        commands = {
            "status": self.snapshot,
            "engage": lambda: self._set_side(_require_side(arguments), True),
            "disengage": lambda: self._set_side(_require_side(arguments), False),
            "disengage_all": self._disengage_all,
            "open_hands": lambda: self._request("open_hands"),
            "reset": lambda: self._reset(arguments),
        }
        handler = commands.get(command)
        if handler is None:
            raise ValueError(f"Unknown command: {command}")
        result = handler()
        return {} if result is None else result
