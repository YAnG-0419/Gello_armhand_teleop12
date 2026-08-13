"""Non-blocking HTTP camera snapshot widget for the operator GUI."""

from __future__ import annotations

import time

from PySide6.QtCore import Qt, QTimer, QUrl
from PySide6.QtGui import QPixmap
from PySide6.QtNetwork import QNetworkAccessManager, QNetworkReply, QNetworkRequest
from PySide6.QtWidgets import (
    QComboBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


class CameraView(QWidget):
    """Poll one latest-frame endpoint with at most one request in flight."""

    def __init__(self, base_url: str, parent=None) -> None:
        super().__init__(parent)
        self.base_url = base_url.rstrip("/")
        self.manager = QNetworkAccessManager(self)
        self.reply: QNetworkReply | None = None
        self.last_frame_at = 0.0
        self.last_pixmap = QPixmap()

        row = QHBoxLayout()
        self.camera = QComboBox()
        self.camera.addItem("Gemini 435Le", "gemini435le")
        self.camera.addItem("305 相机", "camera305")
        self.camera.currentIndexChanged.connect(self._camera_changed)
        row.addWidget(self.camera, stretch=1)
        self.toggle = QPushButton("暂停")
        self.toggle.setCheckable(True)
        self.toggle.setFocusPolicy(Qt.NoFocus)
        self.toggle.toggled.connect(self._toggle)
        row.addWidget(self.toggle)

        self.image = QLabel("等待相机画面…")
        self.image.setAlignment(Qt.AlignCenter)
        self.image.setMinimumSize(420, 260)
        self.image.setStyleSheet(
            "background:#111827; color:#94a3b8; border-radius:8px;"
        )
        self.status = QLabel(f"快照服务：{self.base_url}")
        self.status.setStyleSheet("color:#64748b;")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addLayout(row)
        layout.addWidget(self.image, stretch=1)
        layout.addWidget(self.status)

        self.timer = QTimer(self)
        self.timer.setInterval(125)  # cap UI/network load at 8 FPS
        self.timer.timeout.connect(self._request_frame)
        self.timer.start()

    def set_base_url(self, base_url: str) -> None:
        self.base_url = base_url.rstrip("/")
        self._camera_changed()

    def _toggle(self, paused: bool) -> None:
        self.toggle.setText("继续" if paused else "暂停")
        if paused:
            self.timer.stop()
        else:
            self.timer.start()

    def _camera_changed(self) -> None:
        if self.reply is not None:
            self.reply.abort()
        self.last_frame_at = 0.0
        self.image.setText("正在连接相机…")

    def _request_frame(self) -> None:
        if self.reply is not None:
            return
        name = str(self.camera.currentData())
        request = QNetworkRequest(QUrl(f"{self.base_url}/camera/{name}.jpg"))
        request.setTransferTimeout(1000)
        self.reply = self.manager.get(request)
        self.reply.finished.connect(self._frame_finished)

    def _frame_finished(self) -> None:
        reply = self.reply
        self.reply = None
        if reply is None:
            return
        try:
            if reply.error() != QNetworkReply.NoError:
                if time.monotonic() - self.last_frame_at > 2.0:
                    self.image.setText("相机离线 · 自动重连中")
                    self.status.setText(reply.errorString())
                return
            pixmap = QPixmap()
            if not pixmap.loadFromData(bytes(reply.readAll()), "JPG"):
                self.status.setText("收到的相机帧不是有效 JPEG")
                return
            self.last_pixmap = pixmap
            self.last_frame_at = time.monotonic()
            self._render_pixmap()
            self.status.setText(
                f"{self.camera.currentText()} · 8 FPS 上限 · 仅保留最新帧"
            )
        finally:
            reply.deleteLater()

    def _render_pixmap(self) -> None:
        if self.last_pixmap.isNull():
            return
        self.image.setPixmap(
            self.last_pixmap.scaled(
                self.image.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
        )

    def resizeEvent(self, event) -> None:  # noqa: N802 - Qt API
        super().resizeEvent(event)
        self._render_pixmap()

    def stop(self) -> None:
        self.timer.stop()
        if self.reply is not None:
            self.reply.abort()
