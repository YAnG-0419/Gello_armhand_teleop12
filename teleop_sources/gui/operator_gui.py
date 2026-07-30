"""Operator GUI: a shell over the teleop operator's JSON-TCP control server.

Every button sends one command from the server's dispatch table; a poll
timer refreshes the status panel. No teleop logic lives here - the
backend (pico_bimanual_franka_teleop.control_server) is the single
authority, and this window can disconnect and reconnect at any time
without affecting the session.

    conda activate base && python teleop_sources/gui/operator_gui.py
"""

from __future__ import annotations

import argparse
import json
import sys
import time

from PySide6.QtCore import QTimer
from PySide6.QtNetwork import QAbstractSocket, QTcpSocket
from PySide6.QtWidgets import (
    QApplication,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

POLL_INTERVAL_MS = 500
RECONNECT_INTERVAL_MS = 2000
STATUS_TIMEOUT_SECONDS = 3.0
SIDES = ("left", "right")


class OperatorWindow(QMainWindow):
    def __init__(self, host: str, port: int) -> None:
        super().__init__()
        self.host = host
        self.port = port
        self.setWindowTitle(f"Teleop operator - {host}:{port}")
        self.next_request_id = 1
        self.pending: dict[int, str] = {}
        self.buffer = b""
        self.connection_state = "disconnected"
        self.connected_at: float | None = None
        self.last_status_at: float | None = None
        self._handling_disconnect = False

        self.socket = QTcpSocket(self)
        self.socket.readyRead.connect(self._read_responses)
        self.socket.connected.connect(self._connected)
        self.socket.disconnected.connect(self._socket_disconnected)
        self.socket.errorOccurred.connect(self._socket_error)

        self.poll_timer = QTimer(self)
        self.poll_timer.setInterval(POLL_INTERVAL_MS)
        self.poll_timer.timeout.connect(self._poll_status)
        self.reconnect_timer = QTimer(self)
        self.reconnect_timer.setInterval(RECONNECT_INTERVAL_MS)
        self.reconnect_timer.timeout.connect(self._connect)
        self.health_timer = QTimer(self)
        self.health_timer.setInterval(POLL_INTERVAL_MS)
        self.health_timer.timeout.connect(self._check_connection_health)

        self._build_ui()
        self._set_connection_state("disconnected", "backend is not connected")
        self._connect()

    # ------------------------------------------------------------------ ui
    def _build_ui(self) -> None:
        root = QWidget(self)
        layout = QVBoxLayout(root)

        connection_row = QHBoxLayout()
        self.connection_label = QLabel()
        connection_row.addWidget(self.connection_label, stretch=1)
        self.reconnect_button = QPushButton("Reconnect now")
        self.reconnect_button.clicked.connect(self._reconnect_now)
        connection_row.addWidget(self.reconnect_button)
        layout.addLayout(connection_row)
        self.status_label = QLabel("-")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet("font-family: monospace;")
        layout.addWidget(self.status_label)

        sides_row = QHBoxLayout()
        self.engage_buttons: dict[str, QPushButton] = {}
        for side in SIDES:
            box = QGroupBox(side.capitalize())
            grid = QGridLayout(box)
            engage = QPushButton("Engage")
            engage.setCheckable(True)
            engage.setMinimumHeight(56)
            engage.clicked.connect(
                lambda checked, side=side: self._send(
                    "engage" if checked else "disengage", {"side": side}
                )
            )
            self.engage_buttons[side] = engage
            grid.addWidget(engage, 0, 0)
            grid.addWidget(
                self._button("Home arm", "home_arm", {"side": side}), 1, 0
            )
            grid.addWidget(
                self._button("Open hand", "open_hand", {"side": side}), 2, 0
            )
            sides_row.addWidget(box)
        layout.addLayout(sides_row)

        actions = QHBoxLayout()
        disengage_all = self._button("DISENGAGE ALL", "disengage_all")
        disengage_all.setMinimumHeight(64)
        disengage_all.setStyleSheet(
            "background-color: #a83232; color: white; font-weight: bold;"
        )
        actions.addWidget(disengage_all, stretch=2)
        actions.addWidget(
            self._button("Open both hands", "open_hand", {"side": "both"})
        )
        actions.addWidget(
            self._button("Home both arms", "home_arm", {"side": "both"})
        )
        layout.addLayout(actions)

        self.feedback = QPlainTextEdit()
        self.feedback.setReadOnly(True)
        self.feedback.setMaximumBlockCount(200)
        layout.addWidget(self.feedback, stretch=1)

        self.setCentralWidget(root)
        self.resize(640, 480)

    def _button(self, text: str, command: str, arguments=None) -> QPushButton:
        button = QPushButton(text)
        button.clicked.connect(
            lambda: self._send(command, dict(arguments or {}))
        )
        self.action_buttons = getattr(self, "action_buttons", [])
        self.action_buttons.append(button)
        return button

    # -------------------------------------------------------------- socket
    def _connect(self) -> None:
        if self.socket.state() == QAbstractSocket.UnconnectedState:
            self._set_connection_state("connecting")
            self.socket.connectToHost(self.host, self.port)

    def _connected(self) -> None:
        self.reconnect_timer.stop()
        self.connected_at = time.monotonic()
        self.last_status_at = None
        self.pending.clear()
        self.buffer = b""
        self._set_connection_state("syncing")
        self.poll_timer.start()
        self.health_timer.start()
        self._poll_status()

    def _socket_error(self, _error) -> None:
        self._disconnected(self.socket.errorString())

    def _socket_disconnected(self) -> None:
        self._disconnected(self.socket.errorString())

    def _disconnected(self, reason: str = "") -> None:
        if self._handling_disconnect:
            return
        self._handling_disconnect = True
        self.poll_timer.stop()
        self.health_timer.stop()
        self.pending.clear()
        self.buffer = b""
        self.connected_at = None
        self.last_status_at = None
        detail = reason.strip() or "backend connection closed"
        changed = self.connection_state != "disconnected"
        self._set_connection_state("disconnected", detail)
        if changed:
            self.feedback.appendPlainText(f"[connection] {detail}")
        if self.socket.state() != QAbstractSocket.UnconnectedState:
            self.socket.abort()
        self.reconnect_timer.start()
        self._handling_disconnect = False

    def _set_connection_state(self, state: str, detail: str = "") -> None:
        self.connection_state = state
        if state == "connected":
            text = f"CONNECTED — {self.host}:{self.port}"
            style = "color: #248a3d; font-weight: bold;"
        elif state == "syncing":
            text = f"SYNCING — {self.host}:{self.port}"
            style = "color: #9a6b00; font-weight: bold;"
        elif state == "connecting":
            text = f"CONNECTING — {self.host}:{self.port}"
            style = "color: #9a6b00; font-weight: bold;"
        else:
            suffix = f" — {detail}" if detail else ""
            text = f"DISCONNECTED — {self.host}:{self.port}{suffix}"
            style = "color: #b02020; font-weight: bold;"
        self.connection_label.setText(text)
        self.connection_label.setStyleSheet(style)
        ready = state == "connected"
        for button in getattr(self, "action_buttons", []):
            button.setEnabled(ready)
        for button in self.engage_buttons.values():
            button.setEnabled(ready)
            if not ready:
                button.blockSignals(True)
                button.setChecked(False)
                button.setText("Engage")
                button.blockSignals(False)
        self.reconnect_button.setEnabled(state == "disconnected")
        if state == "disconnected":
            self.status_label.setText("DISCONNECTED — backend status unavailable")

    def _reconnect_now(self) -> None:
        self.reconnect_timer.stop()
        if self.socket.state() != QAbstractSocket.UnconnectedState:
            self.socket.abort()
        self._connect()

    def _check_connection_health(self) -> None:
        if self.socket.state() != QAbstractSocket.ConnectedState:
            return
        baseline = self.last_status_at or self.connected_at
        if baseline is None:
            return
        if time.monotonic() - baseline <= STATUS_TIMEOUT_SECONDS:
            return
        self._disconnected(
            f"no status response for {STATUS_TIMEOUT_SECONDS:.0f}s"
        )

    def _send(self, command: str, arguments=None) -> None:
        if self.socket.state() != QAbstractSocket.ConnectedState:
            return
        request_id = self.next_request_id
        self.next_request_id += 1
        self.pending[request_id] = command
        payload = {
            "id": request_id,
            "command": command,
            "arguments": arguments or {},
        }
        self.socket.write((json.dumps(payload) + "\n").encode("utf-8"))

    def _poll_status(self) -> None:
        if "status" in self.pending.values():
            return
        self._send("status")

    def _read_responses(self) -> None:
        self.buffer += bytes(self.socket.readAll())
        while b"\n" in self.buffer:
            line, self.buffer = self.buffer.split(b"\n", 1)
            try:
                response = json.loads(line)
            except json.JSONDecodeError:
                continue
            command = self.pending.pop(response.get("id"), "")
            if not response.get("ok"):
                self.feedback.appendPlainText(
                    f"[{command}] rejected: {response.get('error')}"
                )
            elif command == "status":
                self._apply_status(response.get("result", {}))

    # -------------------------------------------------------------- status
    def _apply_status(self, status: dict) -> None:
        self.last_status_at = time.monotonic()
        if self.connection_state != "connected":
            self._set_connection_state("connected")
        self.status_label.setText(str(status.get("status_line", "-")))
        active = status.get("active", {})
        for side, button in self.engage_buttons.items():
            engaged = bool(active.get(side))
            button.blockSignals(True)
            button.setChecked(engaged)
            button.setText("Engaged" if engaged else "Engage")
            button.blockSignals(False)
        feedback = status.get("feedback", [])
        # Re-render the ring wholesale: the server caps it at 50 lines, so
        # replacing the text is the simplest correct display.
        if feedback and self.feedback.toPlainText().splitlines() != feedback:
            self.feedback.setPlainText("\n".join(feedback))
            self.feedback.verticalScrollBar().setValue(
                self.feedback.verticalScrollBar().maximum()
            )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5590)
    args = parser.parse_args()
    application = QApplication(sys.argv)
    window = OperatorWindow(args.host, args.port)
    window.show()
    return application.exec()


if __name__ == "__main__":
    raise SystemExit(main())
