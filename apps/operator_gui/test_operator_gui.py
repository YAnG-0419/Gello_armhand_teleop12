import multiprocessing
import os
import socket
import sys
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(
    0, str(REPO_ROOT / "adapters" / "pico" / "src")
)

from PySide6.QtCore import QSettings
from PySide6.QtWidgets import QApplication

from pico_bimanual_franka_teleop.control_server import (
    OperatorConsole,
    OperatorControlServer,
)
from apps.operator_gui.operator_gui import (
    COLLECTION_SHORTCUTS,
    PEDAL_BINDINGS,
    PRESET_KEYS,
    OperatorWindow,
)


def test_shortcut_bindings_match_the_workcell_layout():
    assert PEDAL_BINDINGS == {
        "R": ("toggle", "hand", "left"),
        "Space": ("toggle_hold", "arm", "left"),
        "B": ("toggle", "hand", "right"),
        "Q": ("toggle_hold", "arm", "right"),
    }
    assert COLLECTION_SHORTCUTS == {"L": "toggle_recording", "A": "discard"}
    assert PRESET_KEYS == ("W", "E")


def test_teleop_shortcuts_no_longer_toggle_arm_following(tmp_path):
    QSettings.setPath(
        QSettings.NativeFormat, QSettings.UserScope, str(tmp_path)
    )
    application = QApplication.instance() or QApplication([])
    window = OperatorWindow("127.0.0.1", _unused_port())
    sent = []
    try:
        window.socket.abort()
        window.reconnect_timer.stop()
        window.connection_state = "connected"
        window._send = lambda command, arguments=None: sent.append(
            (command, arguments or {})
        )
        for shortcut in window.shortcuts:
            shortcut.activated.emit()
            application.processEvents()
    finally:
        window.poll_timer.stop()
        window.health_timer.stop()
        window.reconnect_timer.stop()
        window.socket.abort()
        window.close()

    assert sent == [
        ("engage_hand", {"side": "left"}),
        ("hold_arm", {"side": "left", "enabled": True}),
        ("engage_hand", {"side": "right"}),
        ("hold_arm", {"side": "right", "enabled": True}),
    ]


def test_hold_button_sends_latched_publish_command(tmp_path):
    QSettings.setPath(QSettings.NativeFormat, QSettings.UserScope, str(tmp_path))
    application = QApplication.instance() or QApplication([])
    window = OperatorWindow("127.0.0.1", _unused_port())
    sent = []
    try:
        window.socket.abort()
        window.reconnect_timer.stop()
        window.connection_state = "connected"
        window._send = lambda command, arguments=None: sent.append(
            (command, arguments or {})
        )
        window.arm_hold_buttons["left"].setEnabled(True)
        window.arm_hold_buttons["left"].click()
        window.arm_hold_buttons["left"].click()
        application.processEvents()
    finally:
        window.poll_timer.stop()
        window.health_timer.stop()
        window.reconnect_timer.stop()
        window.socket.abort()
        window.close()

    assert sent == [
        ("hold_arm", {"side": "left", "enabled": True}),
        ("hold_arm", {"side": "left", "enabled": False}),
    ]


def test_record_home_sends_per_side_command_only_while_arm_is_stopped(tmp_path):
    QSettings.setPath(
        QSettings.NativeFormat, QSettings.UserScope, str(tmp_path)
    )
    application = QApplication.instance() or QApplication([])
    window = OperatorWindow("127.0.0.1", _unused_port())
    sent = []
    try:
        window.socket.abort()
        window.reconnect_timer.stop()
        window.connection_state = "connected"
        window._send = lambda command, arguments=None: sent.append(
            (command, arguments or {})
        )

        window._record_current_home("left")
        window.arm_engage_buttons["right"].setChecked(True)
        window._record_current_home("right")
        application.processEvents()
    finally:
        window.poll_timer.stop()
        window.health_timer.stop()
        window.reconnect_timer.stop()
        window.socket.abort()
        window.close()

    assert sent == [
        ("capture_home", {"side": "left", "task": "powder_weighing"})
    ]
    assert "stop the right arm first" in window.feedback.toPlainText()


def test_task_selection_drives_home_and_ready_to_home(tmp_path):
    QSettings.setPath(QSettings.NativeFormat, QSettings.UserScope, str(tmp_path))
    application = QApplication.instance() or QApplication([])
    window = OperatorWindow("127.0.0.1", _unused_port())
    sent = []
    try:
        window.socket.abort()
        window.reconnect_timer.stop()
        window.connection_state = "connected"
        window._send = lambda command, arguments=None: sent.append(
            (command, arguments or {})
        )
        window.task_selector.setCurrentIndex(
            window.task_selector.findData("bean_picking")
        )
        window._shortcut_home("both")
        window.ready_to_home_button.setEnabled(True)
        window.ready_to_home_button.click()
        application.processEvents()
    finally:
        window.poll_timer.stop()
        window.health_timer.stop()
        window.reconnect_timer.stop()
        window.socket.abort()
        window.close()

    assert sent == [
        ("home_arm", {"side": "both", "task": "bean_picking"}),
        ("ready_to_home", {"task": "bean_picking"}),
    ]


def test_w_e_shortcuts_run_presets(tmp_path):
    QSettings.setPath(
        QSettings.NativeFormat, QSettings.UserScope, str(tmp_path)
    )
    application = QApplication.instance() or QApplication([])
    window = OperatorWindow("127.0.0.1", _unused_port())
    sent = []
    try:
        window.socket.abort()
        window.reconnect_timer.stop()
        window.connection_state = "connected"
        window._send = lambda command, arguments=None: sent.append(
            (command, arguments or {})
        )
        for shortcut in window.preset_shortcuts:
            shortcut.activated.emit()
            application.processEvents()
    finally:
        window.poll_timer.stop()
        window.health_timer.stop()
        window.reconnect_timer.stop()
        window.socket.abort()
        window.close()

    assert sent == [
        ("run_preset", {"key": "w"}),
        ("run_preset", {"key": "e"}),
    ]


def test_collection_panel_enables_only_valid_episode_actions(tmp_path):
    QSettings.setPath(QSettings.NativeFormat, QSettings.UserScope, str(tmp_path))
    application = QApplication.instance() or QApplication([])
    window = OperatorWindow("127.0.0.1", _unused_port())
    try:
        window.collection_socket.abort()
        window.collection_reconnect_timer.stop()
        window._apply_collection_status(
            {
                "state": "READY",
                "can_start": True,
                "can_stop": False,
                "can_discard": False,
            }
        )
        assert window.collection_record_button.isEnabled()
        assert window.collection_record_button.text() == "开始录制 (L)"

        window._apply_collection_status(
            {
                "state": "RECORDING",
                "current_episode": "episode5",
                "elapsed_sec": 2.5,
                "can_start": False,
                "can_stop": True,
                "can_discard": False,
            }
        )
        assert "episode5" in window.collection_status_label.text()
        assert "2.5s" in window.collection_status_label.text()
        assert window.collection_record_button.isEnabled()
        assert window.collection_record_button.text() == "停止并校验 (L)"

        window._apply_collection_status(
            {
                "state": "FINALIZED",
                "last_episode": "episode5",
                "transport_warnings": ["bag gap"],
                "can_start": True,
                "can_stop": False,
                "can_discard": True,
            }
        )
        assert "1 warning(s)" in window.collection_status_label.text()
        assert window.collection_record_button.isEnabled()
        assert window.collection_record_button.text() == "开始录制 (L)"
        assert window.collection_discard_button.isEnabled()
    finally:
        window.close()
        application.processEvents()


def test_collection_shortcuts_toggle_recording_and_discard(tmp_path):
    QSettings.setPath(QSettings.NativeFormat, QSettings.UserScope, str(tmp_path))
    application = QApplication.instance() or QApplication([])
    window = OperatorWindow("127.0.0.1", _unused_port())
    sent = []
    try:
        window.collection_socket.abort()
        window.collection_reconnect_timer.stop()
        window._send_collection = lambda command: sent.append(command)
        shortcuts = {
            shortcut.key().toString(): shortcut
            for shortcut in window.collection_shortcuts
        }

        window._apply_collection_status(
            {"state": "READY", "can_start": True, "can_stop": False}
        )
        shortcuts["L"].activated.emit()
        window._apply_collection_status(
            {"state": "RECORDING", "can_start": False, "can_stop": True}
        )
        shortcuts["L"].activated.emit()
        window._apply_collection_status(
            {"state": "FINALIZED", "can_discard": True}
        )
        shortcuts["A"].activated.emit()
    finally:
        window.close()
        application.processEvents()

    assert sent == ["start", "stop", "discard"]


def _unused_port() -> int:
    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


def _serve(port: int) -> None:
    server = OperatorControlServer(("127.0.0.1", port), OperatorConsole())
    server.serve_forever(poll_interval=0.05)


def _wait(application, predicate, timeout: float = 5.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        application.processEvents()
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("GUI state did not converge before timeout")


def test_backend_crash_displays_disconnect_and_reconnects_safely(tmp_path):
    QSettings.setPath(
        QSettings.NativeFormat, QSettings.UserScope, str(tmp_path)
    )
    application = QApplication.instance() or QApplication([])
    port = _unused_port()
    context = multiprocessing.get_context("spawn")
    backend = context.Process(target=_serve, args=(port,), daemon=True)
    backend.start()
    window = OperatorWindow("127.0.0.1", port)
    try:
        _wait(application, lambda: window.connection_state == "connected")
        status_before_engage = window.last_status_at
        window.engage_buttons["left"].click()
        _wait(
            application,
            lambda: (
                window.last_status_at is not None
                and window.last_status_at != status_before_engage
                and window.engage_buttons["left"].isChecked()
            ),
        )
        status_before_hand = window.last_status_at
        window.hand_engage_buttons["left"].click()
        _wait(
            application,
            lambda: (
                window.last_status_at is not None
                and window.last_status_at != status_before_hand
                and window.hand_engage_buttons["left"].isChecked()
                and window.arm_engage_buttons["left"].isChecked()
            ),
        )

        backend.terminate()
        backend.join(timeout=2.0)
        _wait(application, lambda: window.connection_state == "disconnected")
        assert window.connection_indicator.text().startswith("DISCONNECTED")
        assert "backend status unavailable" in window.status_label.toPlainText()
        assert window.disconnect_message is not None
        assert window.disconnect_message.isVisible()
        assert window.disconnect_message.text() == (
            "The teleoperation backend connection was lost."
        )
        assert window.connect_action.isEnabled()
        assert not window.engage_buttons["left"].isChecked()
        assert not window.engage_buttons["left"].isEnabled()
        assert not window.hand_engage_buttons["left"].isChecked()
        assert not window.hand_engage_buttons["left"].isEnabled()

        reconnect_port = _unused_port()
        backend = context.Process(
            target=_serve, args=(reconnect_port,), daemon=True
        )
        backend.start()
        window.disconnect_message.accept()
        window.connect_action.trigger()
        _wait(application, lambda: window.connection_dialog is not None)
        assert window.connection_dialog.host_field.text() == "127.0.0.1"
        assert window.connection_dialog.port_field.value() == port
        window.connection_dialog.port_field.setValue(reconnect_port)
        window.connection_dialog._accept_if_valid()
        _wait(application, lambda: window.connection_state == "connected")
        assert window.port == reconnect_port
        assert window.engage_buttons["left"].isEnabled()
        assert not window.engage_buttons["left"].isChecked()
        assert window.disconnect_message is None
    finally:
        window.poll_timer.stop()
        window.health_timer.stop()
        window.reconnect_timer.stop()
        window.socket.abort()
        window.close()
        if backend.is_alive():
            backend.terminate()
        backend.join(timeout=2.0)
