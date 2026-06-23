"""Embedded HTTP server for Jetson Nano dashboard.

Serves:
  /                     → runtime/dashboard/index.html
  /dashboard/static/*   → runtime/dashboard/ (CSS, JS)
  /stream               → MJPEG live stream
  /api/status           → JSON telemetry
  /api/base/<cmd>       → base motor command
  /api/relay/<state>    → relay ON/OFF
  /api/power/<state>    → power ON/OFF
"""

from __future__ import annotations

import json
import logging
import mimetypes
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path
from socketserver import ThreadingMixIn
from typing import Any, Callable

import cv2
import numpy as np

logger = logging.getLogger(__name__)

_DASHBOARD_DIR = Path(__file__).resolve().parent / "dashboard"


class _RequestHandler(BaseHTTPRequestHandler):
    """Minimal request handler with CORS support."""

    def log_message(self, fmt: str, *args: Any) -> None:
        logger.debug("HTTP %s", fmt % args)

    def _cors(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")

    def _text(self, code: int, body: str) -> None:
        self.send_response(code)
        self._cors()
        self.send_header("Content-Type", "text/plain")
        self.end_headers()
        self.wfile.write(body.encode())

    def _json(self, data: dict[str, Any]) -> None:
        body = json.dumps(data, default=str)
        self.send_response(200)
        self._cors()
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(body.encode())

    def _file(self, path: Path) -> None:
        if not path.is_file():
            self._text(404, "not found")
            return
        content_type, _ = mimetypes.guess_type(str(path))
        self.send_response(200)
        self._cors()
        self.send_header("Content-Type", content_type or "application/octet-stream")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(path.read_bytes())

    def do_GET(self) -> None:
        # ---- Dashboard HTML ----
        if self.path == "/" or self.path == "/dashboard":
            self._file(_DASHBOARD_DIR / "index.html")
            return

        # ---- Static files ----
        if self.path.startswith("/dashboard/static/"):
            rel = self.path[len("/dashboard/static/"):]
            safe = rel.lstrip("/").replace("\\", "/")
            if ".." in safe:
                self._text(403, "forbidden")
                return
            self._file(_DASHBOARD_DIR / safe)
            return

        # ---- MJPEG stream ----
        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            getter = self.server.frame_getter
            while True:
                frame = getter() if callable(getter) else None
                if frame is None:
                    time.sleep(0.03)
                    continue
                _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                self.wfile.write(jpeg.tobytes())
                self.wfile.write(b"\r\n")
            return

        # ---- API /status ----
        if self.path == "/api/status":
            getter = getattr(self.server, "status_getter", None)
            data = getter() if callable(getter) else {}
            self._json(data)
            return

        # ---- API /base/<cmd> ----
        if self.path.startswith("/api/base/"):
            cmd = self.path.split("/api/base/")[-1]
            handler = getattr(self.server, "base_handler", None)
            if callable(handler):
                handler(cmd)
            self._text(200, f"base:{cmd}")
            return

        # ---- API /relay/<state> ----
        if self.path.startswith("/api/relay/"):
            state = self.path.split("/api/relay/")[-1].upper()
            handler = getattr(self.server, "relay_handler", None)
            if callable(handler):
                handler(state)
            self._text(200, f"relay:{state}")
            return

        # ---- API /power/<state> ----
        if self.path.startswith("/api/power/"):
            state = self.path.split("/api/power/")[-1].upper()
            handler = getattr(self.server, "power_handler", None)
            if callable(handler):
                handler(state)
            self._text(200, f"power:{state}")
            return

        self._text(404, "not found")


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """HTTP server with threading support."""
    allow_reuse_address = True
    daemon_threads = True


class JetsonHttpServer:
    """Embedded HTTP server for the Jetson Nano dashboard."""

    def __init__(self, host: str = "0.0.0.0", port: int = 8080) -> None:
        self._host = host
        self._port = port
        self._server: ThreadedHTTPServer | None = None
        self._frame_getter: Callable[[], np.ndarray | None] | None = None
        self._status_getter: Callable[[], dict[str, Any]] | None = None
        self._base_handler: Callable[[str], None] | None = None
        self._relay_handler: Callable[[str], None] | None = None
        self._power_handler: Callable[[str], None] | None = None
        self._thread: threading.Thread | None = None

    def set_frame_getter(self, fn: Callable[[], np.ndarray | None]) -> None:
        self._frame_getter = fn

    def set_status_getter(self, fn: Callable[[], dict[str, Any]]) -> None:
        self._status_getter = fn

    def set_base_handler(self, fn: Callable[[str], None]) -> None:
        self._base_handler = fn

    def set_relay_handler(self, fn: Callable[[str], None]) -> None:
        self._relay_handler = fn

    def set_power_handler(self, fn: Callable[[str], None]) -> None:
        self._power_handler = fn

    def start(self) -> None:
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        logger.info("Dashboard: http://%s:%d", self._host, self._port)

    def stop(self) -> None:
        if self._server is not None:
            self._server.shutdown()
        logger.info("Dashboard: stopped")

    def _serve(self) -> None:
        self._server = ThreadedHTTPServer((self._host, self._port), _RequestHandler)
        self._server.frame_getter = self._frame_getter
        self._server.status_getter = self._status_getter
        self._server.base_handler = self._base_handler
        self._server.relay_handler = self._relay_handler
        self._server.power_handler = self._power_handler
        self._server.serve_forever()
