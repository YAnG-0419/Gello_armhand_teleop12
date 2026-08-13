"""ROS compressed-image to HTTP snapshot bridge for the operator GUI.

Run this inside the ROS container.  The Qt operator stays ROS-free and polls
only the most recent JPEG, so slow remote links cannot build an image backlog
or interfere with the teleoperation control loop.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import time
from urllib.parse import urlparse

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


@dataclass
class LatestFrame:
    jpeg: bytes = b""
    received_at: float = 0.0
    source_stamp_ns: int = 0


class CameraSnapshotNode(Node):
    def __init__(self, cameras: dict[str, str]) -> None:
        super().__init__("operator_camera_snapshot")
        self.frames = {name: LatestFrame() for name in cameras}
        self.lock = threading.Lock()
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._camera_subscriptions = []
        for name, topic in cameras.items():
            self._camera_subscriptions.append(
                self.create_subscription(
                    CompressedImage,
                    topic,
                    lambda message, selected=name: self._receive(selected, message),
                    qos,
                )
            )
            self.get_logger().info(f"{name}: {topic}")

    def _receive(self, name: str, message: CompressedImage) -> None:
        if (
            "jpeg" not in message.format.lower()
            and "jpg" not in message.format.lower()
        ):
            return
        frame = LatestFrame(
            jpeg=bytes(message.data),
            received_at=time.monotonic(),
            source_stamp_ns=(
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec)
            ),
        )
        with self.lock:
            self.frames[name] = frame

    def snapshot(self, name: str) -> LatestFrame | None:
        with self.lock:
            return self.frames.get(name)

    def status(self) -> dict:
        now = time.monotonic()
        with self.lock:
            return {
                name: {
                    "ready": bool(frame.jpeg),
                    "age_ms": round((now - frame.received_at) * 1000.0, 1)
                    if frame.received_at
                    else None,
                }
                for name, frame in self.frames.items()
            }


def handler_for(node: CameraSnapshotNode):
    class SnapshotHandler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
            path = urlparse(self.path).path
            if path == "/status":
                payload = json.dumps(node.status()).encode("utf-8")
                self._send(200, "application/json", payload)
                return
            if path.startswith("/camera/") and path.endswith(".jpg"):
                name = path[len("/camera/") : -len(".jpg")]
                frame = node.snapshot(name)
                if frame is None:
                    self._send(404, "text/plain", b"unknown camera")
                elif not frame.jpeg:
                    self._send(503, "text/plain", b"camera frame unavailable")
                else:
                    self._send(200, "image/jpeg", frame.jpeg)
                return
            self._send(404, "text/plain", b"not found")

        def _send(self, status: int, content_type: str, payload: bytes) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store, max-age=0")
            self.end_headers()
            self.wfile.write(payload)

        def log_message(self, _format: str, *_args) -> None:
            return

    return SnapshotHandler


def _camera(value: str) -> tuple[str, str]:
    name, separator, topic = value.partition("=")
    if not separator or not name.strip() or not topic.startswith("/"):
        raise argparse.ArgumentTypeError(
            "camera must be NAME=/absolute/compressed/topic"
        )
    return name.strip(), topic


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8091)
    parser.add_argument(
        "--camera",
        action="append",
        type=_camera,
        default=[],
        help="repeatable NAME=/topic; defaults to Gemini 435Le color",
    )
    args = parser.parse_args()
    cameras = dict(args.camera) or {
        "gemini435le": "/camera/color/image_raw/compressed"
    }
    rclpy.init()
    node = CameraSnapshotNode(cameras)
    server = ThreadingHTTPServer((args.host, args.port), handler_for(node))
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    node.get_logger().info(f"camera snapshots listening on {args.host}:{args.port}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
