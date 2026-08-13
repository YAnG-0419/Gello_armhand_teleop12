"""Operator GUI: a shell over the teleop operator's JSON-TCP control server.

Every button sends one command from the server's dispatch table; a poll
timer refreshes the status panel. No teleop logic lives here - the
backend (pico_bimanual_franka_teleop.control_server) is the single
authority, and this window can disconnect and reconnect at any time
without affecting the session.

    conda activate base && python apps/operator_gui/operator_gui.py
"""

from __future__ import annotations

import argparse
import json
import sys
import time

from PySide6.QtCore import QSettings, Qt, QTimer, QUrl
from PySide6.QtGui import QAction, QDesktopServices, QFontDatabase, QKeySequence, QShortcut
from PySide6.QtNetwork import QAbstractSocket, QTcpSocket
from PySide6.QtWidgets import (
    QApplication,
    QDialog,
    QDialogButtonBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from apps.operator_gui.camera_view import CameraView

POLL_INTERVAL_MS = 500
RECONNECT_INTERVAL_MS = 2000
STATUS_TIMEOUT_SECONDS = 3.0
SIDES = ("left", "right")
PEDAL_BINDINGS = {
    "L": ("toggle", "arm", "left"),
    "R": ("toggle", "hand", "left"),
    "Space": ("home", "arm", "both"),
    "A": ("toggle", "arm", "right"),
    "B": ("toggle", "hand", "right"),
}
PRESET_KEYS = ("Q",)
HAND_KEY_HINTS = {"left": "R", "right": "B"}


class ConnectionDialog(QDialog):
    def __init__(self, host: str, port: int, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Connect to teleop backend")
        self.setModal(True)
        self.setMinimumWidth(380)

        self.host_field = QLineEdit(host)
        self.host_field.setPlaceholderText("Host name or IP address")
        self.port_field = QSpinBox()
        self.port_field.setRange(1, 65535)
        self.port_field.setValue(port)

        form = QFormLayout()
        form.addRow("Host", self.host_field)
        form.addRow("Port", self.port_field)
        buttons = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel
        )
        buttons.button(QDialogButtonBox.Ok).setText("Connect")
        buttons.accepted.connect(self._accept_if_valid)
        buttons.rejected.connect(self.reject)

        layout = QVBoxLayout(self)
        layout.addLayout(form)
        layout.addWidget(buttons)

    def _accept_if_valid(self) -> None:
        if self.host_field.text().strip():
            self.accept()
            return
        QMessageBox.warning(self, "Invalid host", "Enter a host name or IP address.")

    def endpoint(self) -> tuple[str, int]:
        return self.host_field.text().strip(), self.port_field.value()


class TaskDialog(QDialog):
    def __init__(self, actions: list[dict], parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("新增或更新轨迹任务")
        self.setMinimumWidth(440)
        self.task_id = QLineEdit()
        self.task_id.setPlaceholderText("例如 powder_scoop（保存后不建议修改）")
        self.label = QLineEdit()
        self.label.setPlaceholderText("例如 粉末称量")
        self.action = QComboBox()
        for item in actions:
            side, name = str(item["side"]), str(item["name"])
            self.action.addItem(f"{side} · {name}", (side, name))
        self.action.setEditable(not actions)
        if not actions:
            self.action.setPlaceholderText("请先在轨迹示教 UI 中录制动作")
        self.speed = QDoubleSpinBox()
        self.speed.setRange(5.0, 100.0)
        self.speed.setValue(15.0)
        self.speed.setSuffix(" %")
        form = QFormLayout()
        form.addRow("任务 ID", self.task_id)
        form.addRow("显示名称", self.label)
        form.addRow("动作轨迹", self.action)
        form.addRow("执行速度", self.speed)
        buttons = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
        buttons.button(QDialogButtonBox.Save).setText("保存任务")
        buttons.accepted.connect(self._accept_if_valid)
        buttons.rejected.connect(self.reject)
        layout = QVBoxLayout(self)
        layout.addLayout(form)
        layout.addWidget(buttons)

    def _accept_if_valid(self) -> None:
        if not self.task_id.text().strip() or not self.label.text().strip():
            QMessageBox.warning(self, "信息不完整", "请填写任务 ID 和显示名称。")
            return
        if self.action.currentData() is None:
            QMessageBox.warning(self, "没有轨迹", "请先在轨迹示教 UI 中录制动作。")
            return
        self.accept()

    def payload(self) -> dict:
        side, action = self.action.currentData()
        return {
            "task_id": self.task_id.text().strip(),
            "label": self.label.text().strip(),
            "side": side,
            "action": action,
            "speed_scale": self.speed.value() / 100.0,
        }


class OperatorWindow(QMainWindow):
    def __init__(self, host: str | None, port: int | None) -> None:
        super().__init__()
        self.settings = QSettings("HSC", "FrankaUpperBodyTeleop")
        self.host = host or str(self.settings.value("host", "127.0.0.1"))
        self.port = int(
            port if port is not None else self.settings.value("port", 5590)
        )
        self.setWindowTitle(f"Teleop operator - {self.host}:{self.port}")
        self.next_request_id = 1
        self.pending: dict[int, str] = {}
        self.buffer = b""
        self.connection_state = "disconnected"
        self.connected_at: float | None = None
        self.last_status_at: float | None = None
        self._handling_disconnect = False
        self.was_connected = False
        self.connection_dialog: ConnectionDialog | None = None
        self.disconnect_message: QMessageBox | None = None
        self.capture_dialogs: dict[str, QMessageBox] = {}

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
        self._install_shortcuts()
        self._set_connection_state("disconnected", "backend is not connected")
        self._connect()

    # ------------------------------------------------------------------ ui
    def _build_ui(self) -> None:
        connection_menu = self.menuBar().addMenu("&Connection")
        self.connect_action = QAction("Connect...", self)
        self.connect_action.setShortcut("Ctrl+K")
        self.connect_action.triggered.connect(self._open_connection_dialog)
        connection_menu.addAction(self.connect_action)
        connection_menu.addSeparator()
        quit_action = QAction("Quit", self)
        quit_action.setShortcut("Ctrl+Q")
        quit_action.triggered.connect(self.close)
        connection_menu.addAction(quit_action)

        root = QWidget(self)
        root_layout = QVBoxLayout(root)
        tabs = QTabWidget()
        control_tab = QWidget()
        layout = QVBoxLayout(control_tab)
        tabs.addTab(control_tab, "遥操作与任务")

        layout.addWidget(QLabel("System status"))
        self.status_label = QPlainTextEdit("-")
        self.status_label.setReadOnly(True)
        self.status_label.setMinimumHeight(240)
        self.status_label.setFont(
            QFontDatabase.systemFont(QFontDatabase.FixedFont)
        )
        layout.addWidget(self.status_label, stretch=2)

        sides_row = QHBoxLayout()
        self.arm_engage_buttons: dict[str, QPushButton] = {}
        self.hand_engage_buttons: dict[str, QPushButton] = {}
        self.capture_home_buttons: dict[str, QPushButton] = {}
        # Compatibility alias used by existing integrations and tests.
        self.engage_buttons = self.arm_engage_buttons
        pedal_keys = {
            "left": {"arm": "L", "hand": HAND_KEY_HINTS["left"]},
            "right": {"arm": "A", "hand": HAND_KEY_HINTS["right"]},
        }
        for side in SIDES:
            box = QGroupBox(side.capitalize())
            grid = QGridLayout(box)
            keys = pedal_keys[side]
            arm_engage = QPushButton(f"Start arm ({keys['arm']})")
            arm_engage.setCheckable(True)
            arm_engage.setFocusPolicy(Qt.NoFocus)
            arm_engage.setMinimumHeight(56)
            arm_engage.setToolTip(
                f"Pedal/shortcut: {keys['arm']} toggles arm following"
            )
            arm_engage.clicked.connect(
                lambda checked, side=side: self._send(
                    "engage_arm" if checked else "disengage_arm", {"side": side}
                )
            )
            self.arm_engage_buttons[side] = arm_engage
            grid.addWidget(arm_engage, 0, 0)

            hand_engage = QPushButton(f"Start hand ({keys['hand']})")
            hand_engage.setCheckable(True)
            hand_engage.setFocusPolicy(Qt.NoFocus)
            hand_engage.setMinimumHeight(56)
            hand_engage.setToolTip(
                f"Pedal/shortcut: {keys['hand']} toggles MANUS hand following"
            )
            hand_engage.clicked.connect(
                lambda checked, side=side: self._send(
                    "engage_hand" if checked else "disengage_hand",
                    {"side": side},
                )
            )
            self.hand_engage_buttons[side] = hand_engage
            grid.addWidget(hand_engage, 1, 0)

            capture_home = QPushButton("Record current as Home")
            capture_home.setFocusPolicy(Qt.NoFocus)
            capture_home.setToolTip(
                "Save this arm's current measured joint angles as its new Home. "
                "This does not move the robot."
            )
            capture_home.clicked.connect(
                lambda _checked=False, side=side: self._confirm_capture_home(side)
            )
            self.capture_home_buttons[side] = capture_home
            self.action_buttons = getattr(self, "action_buttons", [])
            self.action_buttons.append(capture_home)
            grid.addWidget(capture_home, 2, 0)

            grid.addWidget(
                self._button("Open hand", "open_hand", {"side": side}), 3, 0
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
        home_both = self._button(
            "Home both arms (Space)", "home_arm", {"side": "both"}
        )
        home_both.setToolTip(
            "Pedal/shortcut: Space homes both arms and disengages followers"
        )
        actions.addWidget(home_both)
        layout.addLayout(actions)

        preset_box = QGroupBox("轨迹任务 · Q 脚踏触发当前选择")
        preset_layout = QHBoxLayout(preset_box)
        self.task_select = QComboBox()
        self.task_select.setMinimumHeight(48)
        self.task_select.setMinimumWidth(280)
        self.task_select.currentIndexChanged.connect(self._select_task)
        preset_layout.addWidget(self.task_select, stretch=2)
        run_task = self._button("执行当前任务 (Q)", "run_selected_task")
        run_task.setMinimumHeight(48)
        run_task.setToolTip("从真实当前位置重定位轨迹，完整 IK 预检通过后执行一次")
        preset_layout.addWidget(run_task)
        add_task = QPushButton("新增/更新任务")
        add_task.setFocusPolicy(Qt.NoFocus)
        add_task.clicked.connect(self._open_task_dialog)
        preset_layout.addWidget(add_task)
        refresh_actions = self._button("刷新轨迹", "refresh_actions")
        preset_layout.addWidget(refresh_actions)
        teach_action = QPushButton("打开轨迹示教")
        teach_action.setFocusPolicy(Qt.NoFocus)
        teach_action.clicked.connect(self._open_arm_ui)
        preset_layout.addWidget(teach_action)
        stop_action = self._button("STOP PRESET", "stop_action")
        stop_action.setMinimumHeight(48)
        stop_action.setStyleSheet(
            "background-color: #d47b22; color: white; font-weight: bold;"
        )
        stop_action.setToolTip("Interrupt preset execution and re-anchor GELLO")
        preset_layout.addWidget(stop_action)
        layout.addWidget(preset_box)

        shortcut_hint = QLabel(
            "Foot pedals: L left arm · R left hand · Space home both  |  "
            "A right arm · B right hand"
        )
        shortcut_hint.setStyleSheet("color: #666;")
        layout.addWidget(shortcut_hint)

        layout.addWidget(QLabel("Event log"))
        self.feedback = QPlainTextEdit()
        self.feedback.setReadOnly(True)
        self.feedback.setMaximumBlockCount(200)
        self.feedback.setPlaceholderText("Backend messages will appear here.")
        layout.addWidget(self.feedback, stretch=1)

        camera_tab = QWidget()
        camera_layout = QVBoxLayout(camera_tab)
        camera_layout.addWidget(QLabel("相机监看（画面与机器人控制完全解耦）"))
        self.camera_view = CameraView(f"http://{self.host}:8091")
        camera_layout.addWidget(self.camera_view, stretch=1)
        tabs.addTab(camera_tab, "相机")
        root_layout.addWidget(tabs)

        self.setCentralWidget(root)
        self.connection_indicator = QLabel()
        self.connection_indicator.setContentsMargins(4, 0, 4, 0)
        self.statusBar().addPermanentWidget(self.connection_indicator)
        self.resize(980, 900)

    def _open_arm_ui(self) -> None:
        QDesktopServices.openUrl(QUrl(f"http://{self.host}:8081"))

    def _open_task_dialog(self) -> None:
        dialog = TaskDialog(getattr(self, "available_actions", []), self)
        if dialog.exec() == QDialog.Accepted:
            self._send("configure_task", dialog.payload())

    def _select_task(self) -> None:
        task_id = self.task_select.currentData()
        if task_id and self.connection_state == "connected":
            self._send("select_task", {"task_id": task_id})

    def _confirm_capture_home(self, side: str) -> None:
        """Confirm before overwriting one arm's persisted Home pose."""
        current = self.capture_dialogs.get(side)
        if current is not None:
            current.raise_()
            current.activateWindow()
            return
        message = QMessageBox(self)
        self.capture_dialogs[side] = message
        message.setIcon(QMessageBox.Warning)
        message.setWindowTitle(f"Replace {side} arm Home?")
        message.setText(
            f"Record the {side} arm's current measured joint angles as its new Home?"
        )
        message.setInformativeText(
            "The arm must be stopped. Recording does not move the robot; the "
            "next Home command will move to this saved posture."
        )
        message.setStandardButtons(QMessageBox.Save | QMessageBox.Cancel)
        message.setDefaultButton(QMessageBox.Cancel)
        message.button(QMessageBox.Save).setText("Record Home")
        message.finished.connect(
            lambda result, selected=side, current=message: (
                self._capture_home_dialog_finished(selected, current, result)
            )
        )
        message.open()

    def _capture_home_dialog_finished(
        self, side: str, message: QMessageBox, result: int
    ) -> None:
        if self.capture_dialogs.get(side) is message:
            del self.capture_dialogs[side]
        if result == QMessageBox.Save:
            self._record_current_home(side)
        message.deleteLater()

    def _record_current_home(self, side: str) -> None:
        if self.connection_state != "connected":
            return
        if self.arm_engage_buttons[side].isChecked():
            self.feedback.appendPlainText(
                f"[capture_home] rejected: stop the {side} arm first"
            )
            return
        self._send("capture_home", {"side": side})

    def _button(self, text: str, command: str, arguments=None) -> QPushButton:
        button = QPushButton(text)
        # Space is a physical pedal binding. Buttons must not also consume it
        # as Qt's default "activate focused button" key.
        button.setFocusPolicy(Qt.NoFocus)
        button.clicked.connect(
            lambda: self._send(command, dict(arguments or {}))
        )
        self.action_buttons = getattr(self, "action_buttons", [])
        self.action_buttons.append(button)
        return button

    def _install_shortcuts(self) -> None:
        # Foot pedals appear as ordinary keyboard keys. Each press toggles one
        # follower or homes both arms; auto-repeat is disabled so holding a
        # pedal cannot retrigger an action.
        self.shortcuts = []
        for key, (action, target, side) in PEDAL_BINDINGS.items():
            if action == "toggle":
                slot = lambda side=side, target=target: (
                    self._shortcut_toggle_engage(side, target)
                )
            else:
                slot = lambda side=side: self._shortcut_home(side)
            shortcut = QShortcut(QKeySequence(key), self)
            shortcut.setContext(Qt.WindowShortcut)
            shortcut.setAutoRepeat(False)
            shortcut.activated.connect(slot)
            self.shortcuts.append(shortcut)
        self.preset_shortcuts = []
        shortcut = QShortcut(QKeySequence("Q"), self)
        shortcut.setContext(Qt.WindowShortcut)
        shortcut.setAutoRepeat(False)
        shortcut.activated.connect(self._shortcut_task)
        self.preset_shortcuts.append(shortcut)

    def _shortcut_preset(self, key: str) -> None:
        if self.connection_state == "connected":
            self._send("run_preset", {"key": key})

    def _shortcut_task(self) -> None:
        if self.connection_state == "connected":
            self._send("run_selected_task")

    def _shortcut_toggle_engage(self, side: str, target: str = "arm") -> None:
        if self.connection_state != "connected":
            return
        buttons = (
            self.arm_engage_buttons
            if target == "arm"
            else self.hand_engage_buttons
        )
        button = buttons[side]
        if button.isChecked():
            self._send(f"disengage_{target}", {"side": side})
        else:
            self._send(f"engage_{target}", {"side": side})

    def _shortcut_home(self, side: str) -> None:
        if self.connection_state == "connected":
            self._send("home_arm", {"side": side})

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
        if self.was_connected:
            message = QMessageBox(self)
            self.disconnect_message = message
            message.setIcon(QMessageBox.Warning)
            message.setWindowTitle("Connection lost")
            message.setText("The teleoperation backend connection was lost.")
            message.finished.connect(
                lambda _result, current=message: self._forget_message(current)
            )
            message.open()
        self.was_connected = False
        if self.socket.state() != QAbstractSocket.UnconnectedState:
            self.socket.abort()
        if self.connection_dialog is None:
            self.reconnect_timer.start()
        self._handling_disconnect = False

    def _set_connection_state(self, state: str, detail: str = "") -> None:
        self.connection_state = state
        if state == "connected":
            text = f"CONNECTED  {self.host}:{self.port}"
            style = "color: #248a3d; font-weight: bold;"
        elif state == "syncing":
            text = f"SYNCING  {self.host}:{self.port}"
            style = "color: #9a6b00; font-weight: bold;"
        elif state == "connecting":
            text = f"CONNECTING  {self.host}:{self.port}"
            style = "color: #9a6b00; font-weight: bold;"
        else:
            text = f"DISCONNECTED  {self.host}:{self.port}"
            style = "color: #b02020; font-weight: bold;"
        self.connection_indicator.setText(text)
        self.connection_indicator.setStyleSheet(style)
        self.statusBar().showMessage(detail if state == "disconnected" else "")
        ready = state == "connected"
        for button in getattr(self, "action_buttons", []):
            button.setEnabled(ready)
        for side, button in self.arm_engage_buttons.items():
            button.setEnabled(ready)
            if not ready:
                button.blockSignals(True)
                button.setChecked(False)
                key_hint = "L" if side == "left" else "A"
                button.setText(f"Start arm ({key_hint})")
                button.blockSignals(False)
        for button in getattr(self, "capture_home_buttons", {}).values():
            button.setEnabled(ready)
        for side, button in self.hand_engage_buttons.items():
            button.setEnabled(ready)
            if not ready:
                button.blockSignals(True)
                button.setChecked(False)
                key_hint = HAND_KEY_HINTS[side]
                button.setText(f"Start hand ({key_hint})")
                button.blockSignals(False)
        self.connect_action.setEnabled(state != "connected")
        if state == "disconnected":
            self.status_label.setPlainText(
                "DISCONNECTED — backend status unavailable"
            )
        elif state == "connected":
            self.was_connected = True

    def _forget_message(self, message: QMessageBox) -> None:
        if self.disconnect_message is message:
            self.disconnect_message = None

    def _open_connection_dialog(self) -> None:
        if self.connection_dialog is not None:
            self.connection_dialog.raise_()
            self.connection_dialog.activateWindow()
            return
        dialog = ConnectionDialog(self.host, self.port, self)
        self.connection_dialog = dialog
        self.reconnect_timer.stop()
        if self.socket.state() != QAbstractSocket.UnconnectedState:
            self.socket.abort()
        dialog.finished.connect(self._connection_dialog_finished)
        dialog.open()

    def _connection_dialog_finished(self, result: int) -> None:
        dialog = self.connection_dialog
        self.connection_dialog = None
        if dialog is None:
            return
        if result == QDialog.Accepted:
            self.host, self.port = dialog.endpoint()
            self.settings.setValue("host", self.host)
            self.settings.setValue("port", self.port)
            self.setWindowTitle(f"Teleop operator - {self.host}:{self.port}")
            self.camera_view.set_base_url(f"http://{self.host}:8091")
            self._connect()
        elif self.connection_state == "disconnected":
            self.reconnect_timer.start()
        dialog.deleteLater()

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
        self.status_label.setPlainText(str(status.get("status_line", "-")))
        tasks = status.get("tasks", [])
        selected_task = status.get("selected_task")
        signature = tuple(
            (
                item.get("id"),
                item.get("label"),
                item.get("side"),
                bool(item.get("available")),
            )
            for item in tasks
        )
        if getattr(self, "_task_signature", None) != signature:
            self._task_signature = signature
            self.task_select.blockSignals(True)
            self.task_select.clear()
            for item in tasks:
                suffix = "" if item.get("available") else " · 轨迹缺失"
                self.task_select.addItem(
                    f"{item.get('label')} · {item.get('side')}{suffix}", item.get("id")
                )
            self.task_select.blockSignals(False)
        if selected_task:
            index = self.task_select.findData(selected_task)
            if index >= 0 and index != self.task_select.currentIndex():
                self.task_select.blockSignals(True)
                self.task_select.setCurrentIndex(index)
                self.task_select.blockSignals(False)
        self.available_actions = list(status.get("available_actions", []))
        active = status.get("active", {})
        for side, button in self.arm_engage_buttons.items():
            engaged = bool(active.get(side))
            button.blockSignals(True)
            button.setChecked(engaged)
            key_hint = "L" if side == "left" else "A"
            button.setText(
                f"Arm running ({key_hint})"
                if engaged
                else f"Start arm ({key_hint})"
            )
            button.blockSignals(False)
            self.capture_home_buttons[side].setEnabled(not engaged)
        hand_active = status.get("hand_active", {})
        for side, button in self.hand_engage_buttons.items():
            engaged = bool(hand_active.get(side))
            button.blockSignals(True)
            button.setChecked(engaged)
            key_hint = HAND_KEY_HINTS[side]
            button.setText(
                f"Hand running ({key_hint})"
                if engaged
                else f"Start hand ({key_hint})"
            )
            button.blockSignals(False)
        feedback = status.get("feedback", [])
        # Re-render the ring wholesale: the server caps it at 50 lines, so
        # replacing the text is the simplest correct display.
        if feedback and self.feedback.toPlainText().splitlines() != feedback:
            self.feedback.setPlainText("\n".join(feedback))
            self.feedback.verticalScrollBar().setValue(
                self.feedback.verticalScrollBar().maximum()
            )

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt API
        self.camera_view.stop()
        super().closeEvent(event)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--host",
        default=None,
        help="initial backend host (default: saved value or 127.0.0.1)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=None,
        help="initial backend port (default: saved value or 5590)",
    )
    args = parser.parse_args()
    application = QApplication(sys.argv)
    window = OperatorWindow(args.host, args.port)
    window.show()
    return application.exec()


if __name__ == "__main__":
    raise SystemExit(main())
