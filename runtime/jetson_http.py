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
import os
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path
from socketserver import ThreadingMixIn
from typing import Any, Callable
from urllib.parse import parse_qs, unquote, urlparse

import cv2
import numpy as np

from runtime.calib_tuning import TuneBlockedError, TuneValidationError
from runtime.manual_override import ManualOverrideError

logger = logging.getLogger(__name__)

_DASHBOARD_DIR = Path(__file__).resolve().parent / "dashboard"


class _RequestHandler(BaseHTTPRequestHandler):
    """Minimal request handler with CORS support."""

    def log_message(self, fmt: str, *args: Any) -> None:
        logger.debug("HTTP %s", fmt % args)

    def _cors(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def _text(self, code: int, body: str) -> None:
        self.send_response(code)
        self._cors()
        self.send_header("Content-Type", "text/plain")
        self.end_headers()
        self.wfile.write(body.encode())

    def _json(self, data: dict[str, Any], code: int = 200) -> None:
        body = json.dumps(data, default=str)
        self.send_response(code)
        self._cors()
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(body.encode())

    def _request_path(self) -> str:
        return urlparse(self.path).path

    def _query(self) -> dict[str, list[str]]:
        return parse_qs(urlparse(self.path).query)

    def _json_body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length") or "0")
        if length <= 0:
            return {}
        raw = self.rfile.read(length).decode("utf-8")
        return json.loads(raw) if raw else {}

    def _script_status(self) -> dict[str, Any]:
        runner = getattr(self.server, "script_runner", None)
        status = runner() if callable(runner) else {"running": False, "steps": [], "current": None}
        return {"status": status}

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
        if path.name == "index.html":
            body = path.read_text(encoding="utf-8").replace("__STREAM_PATH__", "/stream")
            self.wfile.write(body.encode())
            return
        self.wfile.write(path.read_bytes())

    def do_OPTIONS(self) -> None:
        self.send_response(204)
        self._cors()
        self.end_headers()

    def do_GET(self) -> None:
        path = self._request_path()
        # ---- Dashboard HTML ----
        if path == "/" or path == "/dashboard":
            self._file(_DASHBOARD_DIR / "index.html")
            return

        # ---- Static files ----
        if path.startswith("/dashboard/static/"):
            rel = path[len("/dashboard/static/"):]
            safe = rel.lstrip("/").replace("\\", "/")
            if ".." in safe:
                self._text(403, "forbidden")
                return
            self._file(_DASHBOARD_DIR / safe)
            return

        # ---- MJPEG stream ----
        if path == "/stream":
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
        if path == "/api/status":
            getter = getattr(self.server, "status_getter", None)
            data = getter() if callable(getter) else {}
            self._json(data)
            return

        if path == "/api/tune":
            getter = getattr(self.server, "tune_getter", None)
            if not callable(getter):
                self._json({"detail": "tune unavailable"}, 404)
                return
            self._json(getter())
            return

        # ---- API /base/<cmd> ----
        if path.startswith("/api/base/"):
            cmd = path.split("/api/base/")[-1]
            handler = getattr(self.server, "base_handler", None)
            if callable(handler):
                handler(cmd)
            self._text(200, f"base:{cmd}")
            return

        # ---- API /relay/<state> ----
        if path.startswith("/api/relay/"):
            state = path.split("/api/relay/")[-1].upper()
            handler = getattr(self.server, "relay_handler", None)
            if callable(handler):
                handler(state)
            self._text(200, f"relay:{state}")
            return

        # ---- API /power/<state> ----
        if path.startswith("/api/power/"):
            state = path.split("/api/power/")[-1].upper()
            handler = getattr(self.server, "power_handler", None)
            if callable(handler):
                handler(state)
            self._text(200, f"power:{state}")
            return

        # ---- Route script builder ----
        if path == "/route/script/status":
            self._json(self._script_status())
            return
        # ---- Presets ----
        if path == "/presets":
            pg = getattr(self.server, "presets_getter", None)
            self._json({"presets": pg() if callable(pg) else []})
            return
        if path.startswith("/presets/"):
            name = unquote(path.split("/presets/", 1)[1])
            pg = getattr(self.server, "presets_getter", None)
            presets = pg() if callable(pg) else []
            preset = next((p for p in presets if p.get("name") == name), None)
            self._json({"preset": preset} if preset else {"detail": "preset not found"}, 200 if preset else 404)
            return
        # ---- Routes list ----
        if path.startswith("/routes/list"):
            rg = getattr(self.server, "routes_getter", None)
            self._json({"routes": rg() if callable(rg) else []})
            return
        if path.startswith("/routes/") and path.endswith("/summary"):
            route_id = unquote(path[len("/routes/"):-len("/summary")])
            root = Path(os.getenv("ROUTE_LOG_ROOT", "/data/routes")).resolve()
            summary_path = (root / route_id / "route_summary.json").resolve()
            if root not in summary_path.parents or not summary_path.is_file():
                self._json({"summary": {"route_id": route_id, "status": "not_recorded"}}, 404)
                return
            self._json({"summary": json.loads(summary_path.read_text(encoding="utf-8"))})
            return
        if path.startswith("/routes/download/"):
            route_id = unquote(path[len("/routes/download/"):])
            root = Path(os.getenv("ROUTE_LOG_ROOT", "/data/routes")).resolve()
            zip_path = (root / f"{route_id}.zip").resolve()
            if root not in zip_path.parents:
                self._text(400, "bad route")
                return
            self._file(zip_path)
            return

        self._text(404, "not found")

    def do_POST(self) -> None:
        path = self._request_path()
        if path == "/api/manual_override":
            handler = getattr(self.server, "manual_override_handler", None)
            if not callable(handler):
                self._json({"detail": "manual override unavailable"}, 404)
                return
            try:
                self._json(handler(self._json_body()))
            except (json.JSONDecodeError, ManualOverrideError) as exc:
                self._json({"detail": str(exc)}, 400)
            return
        if path == "/route/script":
            submitter = getattr(self.server, "script_submitter", None)
            body = self._json_body()
            ok = submitter(json.dumps(body)) if callable(submitter) else False
            self._json({"ok": bool(ok)}, 200 if ok else 400)
            return
        if path == "/route/script/step":
            submitter = getattr(self.server, "script_submitter", None)
            step = self._json_body()
            ok = submitter(json.dumps({"steps": [step]})) if callable(submitter) else False
            self._json({"ok": bool(ok)}, 200 if ok else 400)
            return
        if path == "/route/script/stop":
            stopper = getattr(self.server, "script_stopper", None)
            if callable(stopper):
                stopper()
            self._json({"ok": True})
            return
        if path == "/route/relay":
            on = (self._query().get("on") or ["0"])[0] == "1"
            handler = getattr(self.server, "relay_handler", None)
            if callable(handler):
                handler("ON" if on else "OFF")
            self._json({"ok": True, "on": on})
            return
        if path == "/control/power":
            on = (self._query().get("on") or ["0"])[0] == "1"
            handler = getattr(self.server, "power_handler", None)
            if callable(handler):
                handler("ON" if on else "OFF")
            self._json({"ok": True, "on": on})
            return
        if path == "/control/estop_reset":
            self._json({"ok": True})
            return
        if path == "/routes/delete_all":
            self._json({"removed": 0, "errors": []})
            return
        if path == "/api/tune/save":
            saver = getattr(self.server, "tune_saver", None)
            if not callable(saver):
                self._json({"detail": "tune unavailable"}, 404)
                return
            try:
                self._json(saver())
            except TuneValidationError as exc:
                self._json({"detail": str(exc)}, 400)
            return
        if path == "/api/tune/reset":
            resetter = getattr(self.server, "tune_resetter", None)
            if not callable(resetter):
                self._json({"detail": "tune unavailable"}, 404)
                return
            try:
                self._json(resetter(str(self._json_body().get("target", ""))))
            except TuneBlockedError as exc:
                self._json({"detail": str(exc)}, 409)
            except TuneValidationError as exc:
                self._json({"detail": str(exc)}, 400)
            return
        self._text(404, "not found")

    def do_PUT(self) -> None:
        path = self._request_path()
        if path == "/api/tune":
            applier = getattr(self.server, "tune_applier", None)
            if not callable(applier):
                self._json({"detail": "tune unavailable"}, 404)
                return
            try:
                self._json(applier(self._json_body().get("values", {})))
            except TuneBlockedError as exc:
                self._json({"detail": str(exc)}, 409)
            except TuneValidationError as exc:
                self._json({"detail": str(exc)}, 400)
            return
        if path.startswith("/presets/"):
            name = unquote(path.split("/presets/", 1)[1])
            setter = getattr(self.server, "presets_setter", None)
            body = self._json_body()
            body["name"] = name
            if callable(setter):
                setter(json.dumps(body))
            self._json({"ok": True, "preset": body})
            return
        self._text(404, "not found")

    def do_DELETE(self) -> None:
        path = self._request_path()
        if path.startswith("/presets/"):
            name = unquote(path.split("/presets/", 1)[1])
            deleter = getattr(self.server, "preset_deleter", None)
            if callable(deleter):
                deleter(name)
            self._json({"ok": True})
            return
        if path.startswith("/routes/"):
            self._json({"ok": True})
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
        self._script_runner: Callable[[], dict[str, Any]] | None = None
        self._script_stopper: Callable[[], None] | None = None
        self._script_submitter: Callable[[str], None] | None = None
        self._steps_getter: Callable[[], list[dict[str, Any]]] | None = None
        self._steps_setter: Callable[[str], None] | None = None
        self._presets_getter: Callable[[], list[dict[str, Any]]] | None = None
        self._presets_setter: Callable[[str], None] | None = None
        self._preset_deleter: Callable[[str], None] | None = None
        self._routes_getter: Callable[[], list[dict[str, Any]]] | None = None
        self._tune_getter: Callable[[], dict[str, Any]] | None = None
        self._tune_applier: Callable[[dict[str, Any]], dict[str, Any]] | None = None
        self._tune_saver: Callable[[], dict[str, Any]] | None = None
        self._tune_resetter: Callable[[str], dict[str, Any]] | None = None
        self._manual_override_handler: Callable[[dict[str, Any]], dict[str, Any]] | None = None
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

    def set_manual_override_handler(self, fn: Callable[[dict[str, Any]], dict[str, Any]]) -> None:
        self._manual_override_handler = fn

    def set_script_runner(self, fn: Callable[[], dict[str, Any]]) -> None:
        self._script_runner = fn

    def set_script_stopper(self, fn: Callable[[], None]) -> None:
        self._script_stopper = fn

    def set_script_submitter(self, fn: Callable[[str], None]) -> None:
        self._script_submitter = fn

    def set_steps_getter(self, fn: Callable[[], list[dict[str, Any]]]) -> None:
        self._steps_getter = fn

    def set_steps_setter(self, fn: Callable[[str], None]) -> None:
        self._steps_setter = fn

    def set_presets_getter(self, fn: Callable[[], list[dict[str, Any]]]) -> None:
        self._presets_getter = fn

    def set_presets_setter(self, fn: Callable[[str], None]) -> None:
        self._presets_setter = fn

    def set_preset_deleter(self, fn: Callable[[str], None]) -> None:
        self._preset_deleter = fn

    def set_routes_getter(self, fn: Callable[[], list[dict[str, Any]]]) -> None:
        self._routes_getter = fn

    def set_tune_handlers(
        self,
        *,
        getter: Callable[[], dict[str, Any]],
        applier: Callable[[dict[str, Any]], dict[str, Any]],
        saver: Callable[[], dict[str, Any]],
        resetter: Callable[[str], dict[str, Any]],
    ) -> None:
        self._tune_getter = getter
        self._tune_applier = applier
        self._tune_saver = saver
        self._tune_resetter = resetter

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
        self._server.manual_override_handler = self._manual_override_handler
        self._server.script_runner = self._script_runner
        self._server.script_stopper = self._script_stopper
        self._server.script_submitter = self._script_submitter
        self._server.steps_getter = self._steps_getter
        self._server.steps_setter = self._steps_setter
        self._server.presets_getter = self._presets_getter
        self._server.presets_setter = self._presets_setter
        self._server.preset_deleter = self._preset_deleter
        self._server.routes_getter = self._routes_getter
        self._server.tune_getter = self._tune_getter
        self._server.tune_applier = self._tune_applier
        self._server.tune_saver = self._tune_saver
        self._server.tune_resetter = self._tune_resetter
        self._server.serve_forever()
