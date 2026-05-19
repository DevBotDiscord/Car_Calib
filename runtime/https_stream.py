"""HTTPS MJPEG streaming helpers for realtime camera output."""

from __future__ import annotations

import ipaddress
import importlib
import ssl
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Optional

import cv2

try:  # Optional dependency: only needed when stream server is started.
    from fastapi import Request as _FastAPIRequest
except Exception:  # noqa: BLE001
    _FastAPIRequest = Any  # type: ignore[assignment,misc]


@dataclass
class SharedFrameStore:
    """Thread-safe holder for the latest JPEG frame and telemetry."""

    jpeg_bytes: Optional[bytes] = None
    timestamp_unix: Optional[float] = None
    telemetry: dict[str, Any] | None = None

    def __post_init__(self) -> None:
        self._lock = threading.Lock()

    def set_frame(self, frame_bgr: Any, telemetry: dict[str, Any]) -> None:
        ok, encoded = cv2.imencode(".jpg", frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            return
        now = time.time()
        with self._lock:
            self.jpeg_bytes = encoded.tobytes()
            self.timestamp_unix = now
            self.telemetry = telemetry

    def snapshot(self) -> tuple[Optional[bytes], Optional[float], dict[str, Any] | None]:
        with self._lock:
            return self.jpeg_bytes, self.timestamp_unix, self.telemetry


class HttpsMjpegServer:
    """Owns FastAPI app and uvicorn server lifecycle."""

    def __init__(
        self,
        host: str,
        port: int,
        stream_path: str,
        snapshot_path: str,
        status_path: str,
        token: str,
        cert_file: str,
        key_file: str,
        frame_store: SharedFrameStore,
        script_runner: Any | None = None,
    ) -> None:
        self._host = host
        self._port = port
        self._stream_path = _normalize_path(stream_path)
        self._snapshot_path = _normalize_path(snapshot_path)
        self._status_path = _normalize_path(status_path)
        self._token = token.strip()
        self._cert_file = cert_file
        self._key_file = key_file
        self._frame_store = frame_store
        self._script_runner = script_runner

        self._server: Any = None
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        app = self._build_app()
        uvicorn_module = importlib.import_module("uvicorn")
        config = uvicorn_module.Config(
            app,
            host=self._host,
            port=self._port,
            log_level="warning",
            ssl_certfile=self._cert_file,
            ssl_keyfile=self._key_file,
        )
        self._server = uvicorn_module.Server(config)
        if self._server is None:
            raise RuntimeError("Failed to create HTTPS stream server.")
        server = self._server
        self._thread = threading.Thread(target=server.run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if self._server is not None:
            self._server.should_exit = True
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def stream_url(self) -> str:
        return f"https://{self._host}:{self._port}{self._stream_path}"

    def status_url(self) -> str:
        return f"https://{self._host}:{self._port}{self._status_path}"

    def snapshot_url(self) -> str:
        return f"https://{self._host}:{self._port}{self._snapshot_path}"

    def _build_app(self) -> Any:
        fastapi_module = importlib.import_module("fastapi")
        responses_module = importlib.import_module("fastapi.responses")
        FastAPI = fastapi_module.FastAPI
        HTTPException = fastapi_module.HTTPException
        Request = fastapi_module.Request
        JSONResponse = responses_module.JSONResponse
        Response = responses_module.Response
        StreamingResponse = responses_module.StreamingResponse

        app = FastAPI(title="Robot Debug Stream", docs_url=None, redoc_url=None)

        def _check_token(candidate: str) -> None:
            if not self._token:
                return
            if candidate != self._token:
                raise HTTPException(status_code=401, detail="Unauthorized")

        @app.get(self._status_path)
        def status(token: str = "") -> Any:
            _check_token(token)
            _, ts, telemetry = self._frame_store.snapshot()
            return JSONResponse(
                {
                    "ok": True,
                    "has_frame": ts is not None,
                    "last_frame_unix": ts,
                    "telemetry": telemetry,
                }
            )

        @app.get(self._snapshot_path)
        def snapshot(token: str = "") -> Any:
            _check_token(token)
            jpeg, _, _ = self._frame_store.snapshot()
            if jpeg is None:
                return Response(status_code=503, content=b"No frame available")
            return Response(content=jpeg, media_type="image/jpeg")

        @app.get(self._stream_path)
        def stream(token: str = "") -> Any:
            _check_token(token)

            def generate() -> Any:
                boundary = b"--frame\r\n"
                while True:
                    jpeg, _, _ = self._frame_store.snapshot()
                    if jpeg is None:
                        time.sleep(0.05)
                        continue
                    yield boundary
                    yield b"Content-Type: image/jpeg\r\n"
                    yield f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii")
                    yield jpeg
                    yield b"\r\n"
                    time.sleep(0.04)

            return StreamingResponse(generate(), media_type="multipart/x-mixed-replace; boundary=frame")

        @app.get("/dashboard")
        def dashboard(token: str = "") -> Any:
            _check_token(token)
            return Response(
                content=_DASHBOARD_HTML.replace(
                    "__STREAM_PATH__", self._stream_path
                ).replace("__STATUS_PATH__", self._status_path).replace(
                    "__TOKEN__", self._token
                ),
                media_type="text/html; charset=utf-8",
            )

        @app.get("/route/script/status")
        def route_script_status(token: str = "") -> Any:
            _check_token(token)
            if self._script_runner is None:
                return JSONResponse({"ok": False, "error": "script_runner_disabled"}, status_code=503)
            return JSONResponse({"ok": True, "status": self._script_runner.status()})

        @app.post("/route/script")
        async def route_script_submit(request: _FastAPIRequest, token: str = "") -> Any:
            _check_token(token)
            if self._script_runner is None:
                raise HTTPException(status_code=503, detail="script_runner_disabled")
            try:
                payload = await request.json()
            except Exception as exc:  # noqa: BLE001
                raise HTTPException(status_code=400, detail=f"invalid_json: {exc}") from exc

            from runtime.route_script import validate_steps  # local import to avoid cycle

            try:
                steps = validate_steps(payload.get("steps") if isinstance(payload, dict) else None)
            except ValueError as exc:
                raise HTTPException(status_code=400, detail=str(exc)) from exc

            if not self._script_runner.submit(steps):
                raise HTTPException(status_code=409, detail="script_already_running")
            return JSONResponse({"ok": True, "status": self._script_runner.status()})

        @app.post("/route/script/stop")
        def route_script_stop(token: str = "") -> Any:
            _check_token(token)
            if self._script_runner is None:
                raise HTTPException(status_code=503, detail="script_runner_disabled")
            self._script_runner.stop()
            return JSONResponse({"ok": True, "status": self._script_runner.status()})

        return app


def ensure_self_signed_cert(
    cert_file: str,
    key_file: str,
    host: str,
    valid_days: int,
) -> None:
    """Create self-signed cert/key pair if they are missing."""
    cert_path = Path(cert_file)
    key_path = Path(key_file)
    cert_path.parent.mkdir(parents=True, exist_ok=True)
    key_path.parent.mkdir(parents=True, exist_ok=True)

    if cert_path.exists() and key_path.exists():
        _validate_cert_pair(str(cert_path), str(key_path))
        return

    x509 = importlib.import_module("cryptography.x509")
    hashes = importlib.import_module("cryptography.hazmat.primitives.hashes")
    serialization = importlib.import_module("cryptography.hazmat.primitives.serialization")
    rsa = importlib.import_module("cryptography.hazmat.primitives.asymmetric.rsa")
    oid_module = importlib.import_module("cryptography.x509.oid")
    NameOID = oid_module.NameOID

    key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    subject = issuer = x509.Name([
        x509.NameAttribute(NameOID.COUNTRY_NAME, "US"),
        x509.NameAttribute(NameOID.ORGANIZATION_NAME, "UOG AIS AUTOBOT"),
        x509.NameAttribute(NameOID.COMMON_NAME, host),
    ])

    san_entries: list[Any] = [x509.DNSName("localhost")]
    try:
        san_entries.append(x509.IPAddress(ipaddress.ip_address("127.0.0.1")))
        host_ip = ipaddress.ip_address(host)
        san_entries.append(x509.IPAddress(host_ip))
    except ValueError:
        san_entries.append(x509.DNSName(host))

    now = datetime.now(timezone.utc)
    cert = (
        x509.CertificateBuilder()
        .subject_name(subject)
        .issuer_name(issuer)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now - timedelta(minutes=1))
        .not_valid_after(now + timedelta(days=max(1, valid_days)))
        .add_extension(x509.SubjectAlternativeName(san_entries), critical=False)
        .sign(key, hashes.SHA256())
    )

    key_bytes = key.private_bytes(
        encoding=serialization.Encoding.PEM,
        format=serialization.PrivateFormat.TraditionalOpenSSL,
        encryption_algorithm=serialization.NoEncryption(),
    )
    cert_bytes = cert.public_bytes(serialization.Encoding.PEM)

    key_path.write_bytes(key_bytes)
    cert_path.write_bytes(cert_bytes)


def _validate_cert_pair(cert_file: str, key_file: str) -> None:
    """Verify cert/key files can be loaded by ssl stack."""
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(certfile=cert_file, keyfile=key_file)


def _normalize_path(path: str) -> str:
    if not path:
        return "/"
    return path if path.startswith("/") else f"/{path}"


_DASHBOARD_HTML = """<!doctype html>
<html lang=\"en\">
<head>
  <meta charset=\"utf-8\">
  <title>car-calib route dashboard</title>
  <style>
    body { font-family: system-ui, sans-serif; background: #111; color: #eee; margin: 0; padding: 16px; }
    h1 { margin: 0 0 12px; font-size: 18px; }
    .layout { display: grid; grid-template-columns: minmax(360px, 60vw) 1fr; gap: 16px; }
    .panel { background: #1c1c1c; border: 1px solid #333; border-radius: 8px; padding: 12px; }
    img.stream { width: 100%; max-width: 960px; border: 1px solid #333; background: #000; }
    .row { display: flex; gap: 8px; align-items: center; margin: 6px 0; flex-wrap: wrap; }
    select, input, button { background: #222; color: #eee; border: 1px solid #555; padding: 6px 8px; border-radius: 4px; }
    button { cursor: pointer; }
    button.primary { background: #285; border-color: #4a8; color: #001; font-weight: 600; }
    button.danger { background: #722; border-color: #a44; color: #fee; }
    table { width: 100%; border-collapse: collapse; margin-top: 8px; }
    th, td { text-align: left; padding: 4px 6px; border-bottom: 1px solid #333; font-size: 13px; }
    pre { background: #0a0a0a; padding: 8px; border-radius: 4px; max-height: 200px; overflow: auto; font-size: 12px; }
    .muted { color: #888; font-size: 12px; }
  </style>
</head>
<body>
  <h1>car-calib · route dashboard</h1>
  <div class=\"layout\">
    <div class=\"panel\">
      <img id=\"stream\" class=\"stream\" alt=\"camera stream\">
      <div class=\"muted\" id=\"telemetry\">telemetry: …</div>
    </div>
    <div class=\"panel\">
      <h2 style=\"font-size:15px;margin:0 0 8px\">Route script builder</h2>
      <div class=\"row\">
        <select id=\"action\">
          <option value=\"forward\">forward (FORWARD + center)</option>
          <option value=\"backward\">backward (BACKWARD + center)</option>
          <option value=\"left\">left (FORWARD + max left)</option>
          <option value=\"right\">right (FORWARD + max right)</option>
          <option value=\"straight\">straight (alias)</option>
          <option value=\"stop\">stop / pause</option>
        </select>
        <input id=\"duration\" type=\"number\" min=\"0\" step=\"0.5\" value=\"2\" style=\"width: 90px\">
        <span class=\"muted\">seconds</span>
        <button id=\"add\">+ add step</button>
        <button id=\"clear\">clear</button>
      </div>
      <table id=\"steps\">
        <thead><tr><th>#</th><th>action</th><th>duration_s</th><th></th></tr></thead>
        <tbody></tbody>
      </table>
      <div class=\"row\" style=\"margin-top:12px\">
        <button id=\"run\" class=\"primary\">▶ run script (auto-record)</button>
        <button id=\"stop\" class=\"danger\">■ stop</button>
        <span id=\"runState\" class=\"muted\">idle</span>
      </div>
      <h3 style=\"font-size:13px;margin:14px 0 4px\">JSON preview</h3>
      <pre id=\"preview\">[]</pre>
    </div>
  </div>
<script>
const TOKEN = \"__TOKEN__\";
const STREAM = \"__STREAM_PATH__\";
const STATUS = \"__STATUS_PATH__\";
const qp = TOKEN ? (\"?token=\" + encodeURIComponent(TOKEN)) : \"\";
document.getElementById(\"stream\").src = STREAM + qp;
const steps = [];
const tbody = document.querySelector(\"#steps tbody\");
const preview = document.getElementById(\"preview\");
const runState = document.getElementById(\"runState\");
const telemetry = document.getElementById(\"telemetry\");

function render() {
  tbody.innerHTML = \"\";
  steps.forEach((s, i) => {
    const tr = document.createElement(\"tr\");
    tr.innerHTML = `<td>${i+1}</td><td>${s.action}</td><td>${s.duration_s}</td><td><button data-i=\"${i}\" class=\"rm\">×</button></td>`;
    tbody.appendChild(tr);
  });
  preview.textContent = JSON.stringify({steps}, null, 2);
}

document.getElementById(\"add\").onclick = () => {
  const action = document.getElementById(\"action\").value;
  const duration_s = parseFloat(document.getElementById(\"duration\").value || \"0\");
  if (!isFinite(duration_s) || duration_s < 0) return;
  steps.push({action, duration_s});
  render();
};
document.getElementById(\"clear\").onclick = () => { steps.length = 0; render(); };
tbody.onclick = (e) => {
  const t = e.target;
  if (t.classList.contains(\"rm\")) { steps.splice(parseInt(t.dataset.i, 10), 1); render(); }
};

async function postJSON(url, body) {
  const r = await fetch(url + qp, {method: \"POST\", headers: {\"Content-Type\": \"application/json\"}, body: JSON.stringify(body)});
  return await r.json().catch(() => ({}));
}

document.getElementById(\"run\").onclick = async () => {
  if (steps.length === 0) { runState.textContent = \"add steps first\"; return; }
  runState.textContent = \"submitting…\";
  const r = await fetch(\"/route/script\" + qp, {method: \"POST\", headers: {\"Content-Type\": \"application/json\"}, body: JSON.stringify({steps})});
  const j = await r.json().catch(() => ({}));
  if (!r.ok) { runState.textContent = \"error: \" + (j.detail || r.status); return; }
  runState.textContent = \"running\";
};
document.getElementById(\"stop\").onclick = async () => {
  await fetch(\"/route/script/stop\" + qp, {method: \"POST\"});
  runState.textContent = \"stopping\";
};

async function pollStatus() {
  try {
    const r = await fetch(\"/route/script/status\" + qp);
    if (r.ok) {
      const j = await r.json();
      const st = j.status || {};
      const cur = st.step ? `${st.step.action} ${st.step.duration_s}s` : \"-\";
      runState.textContent = st.running ? `running step ${st.current_step}/${st.total} (${cur})` : (st.last_error ? (\"error: \" + st.last_error) : \"idle\");
    }
  } catch (e) {}
  try {
    const r2 = await fetch(STATUS + qp);
    if (r2.ok) {
      const j2 = await r2.json();
      const t = j2.telemetry || {};
      telemetry.textContent = `route_id=${t.route_id||'-'} mode=${t.route_mode||'-'} fsm=${t.fsm_state||'-'} servo=${t.servo_angle||'-'} theta=${t.theta||'-'}`;
    }
  } catch (e) {}
}
setInterval(pollStatus, 800);
render();
</script>
</body>
</html>
"""
