import multiprocessing
import os
import socket
import sys
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(
    0, str(REPO_ROOT / "teleop_sources" / "pico" / "src")
)

from PySide6.QtWidgets import QApplication

from pico_bimanual_franka_teleop.control_server import (
    OperatorConsole,
    OperatorControlServer,
)
from teleop_sources.gui.operator_gui import OperatorWindow


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


def test_backend_crash_displays_disconnect_and_reconnects_safely():
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

        backend.terminate()
        backend.join(timeout=2.0)
        _wait(application, lambda: window.connection_state == "disconnected")
        assert window.connection_indicator.text().startswith("DISCONNECTED")
        assert "backend status unavailable" in window.status_label.text()
        assert window.disconnect_message is not None
        assert window.disconnect_message.isVisible()
        assert window.reconnect_action.isEnabled()
        assert not window.engage_buttons["left"].isChecked()
        assert not window.engage_buttons["left"].isEnabled()

        backend = context.Process(target=_serve, args=(port,), daemon=True)
        backend.start()
        window.reconnect_action.trigger()
        _wait(application, lambda: window.connection_state == "connected")
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
