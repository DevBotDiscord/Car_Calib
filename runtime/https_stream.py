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

        @app.get("/routes/list")
        def routes_list(token: str = "", limit: int = 30) -> Any:
            _check_token(token)
            from config.settings import ROUTE_LOG_ROOT
            root = Path(ROUTE_LOG_ROOT)
            if not root.exists():
                root = Path("logs/routes")
            if not root.exists():
                return JSONResponse({"ok": True, "routes": []})
            entries: list[dict[str, Any]] = []
            for child in sorted(root.iterdir(), reverse=True):
                if not child.is_dir():
                    continue
                summary_path = child / "route_summary.json"
                zip_path = child.parent / (child.name + ".zip")
                meta = {"route_id": child.name, "has_zip": zip_path.exists()}
                if zip_path.exists():
                    try:
                        meta["zip_size"] = zip_path.stat().st_size
                    except OSError:
                        meta["zip_size"] = None
                if summary_path.exists():
                    try:
                        meta.update(_load_summary_brief(summary_path))
                    except Exception:  # noqa: BLE001
                        pass
                entries.append(meta)
                if len(entries) >= max(1, min(200, limit)):
                    break
            return JSONResponse({"ok": True, "routes": entries})

        @app.get("/routes/download/{name}")
        def routes_download(name: str, token: str = "") -> Any:
            _check_token(token)
            from config.settings import ROUTE_LOG_ROOT
            # Reject path traversal: only basename, no slashes/backslashes/..
            if "/" in name or "\\" in name or ".." in name or name.startswith("."):
                raise HTTPException(status_code=400, detail="invalid_name")
            root = Path(ROUTE_LOG_ROOT)
            if not root.exists():
                root = Path("logs/routes")
            zip_path = (root / f"{name}.zip").resolve()
            try:
                root_resolved = root.resolve()
            except OSError as exc:
                raise HTTPException(status_code=500, detail=f"root_resolve: {exc}") from exc
            if not str(zip_path).startswith(str(root_resolved) + "/") and zip_path.parent != root_resolved:
                raise HTTPException(status_code=400, detail="path_outside_root")
            if not zip_path.exists():
                raise HTTPException(status_code=404, detail="zip_not_found")
            return Response(
                content=zip_path.read_bytes(),
                media_type="application/zip",
                headers={"Content-Disposition": f'attachment; filename=\"{name}.zip\"'},
            )

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


def _load_summary_brief(path: Path) -> dict[str, Any]:
    import json as _json
    raw = _json.loads(path.read_text(encoding="utf-8"))
    return {
        "route_mode": raw.get("route_mode"),
        "status": raw.get("status"),
        "accepted": raw.get("accepted"),
        "total_frames": raw.get("total_frames"),
        "elapsed_s": raw.get("total_elapsed_seconds"),
        "end_timestamp_utc": raw.get("end_timestamp_utc"),
    }


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
    * { box-sizing: border-box; }
    body { font-family: system-ui, sans-serif; background: #0d0d0f; color: #eee; margin: 0; padding: 20px; font-size: 15px; }
    h1 { margin: 0 0 16px; font-size: 22px; letter-spacing: 0.3px; }
    h2 { font-size: 17px; margin: 0 0 10px; }
    h3 { font-size: 14px; margin: 16px 0 6px; color: #aaa; text-transform: uppercase; letter-spacing: 0.6px; }
    .layout { display: grid; grid-template-columns: minmax(420px, 55vw) 1fr; gap: 20px; }
    .panel { background: #16171a; border: 1px solid #2a2c30; border-radius: 10px; padding: 16px; }
    img.stream { width: 85%; max-width: 800px; border: 1px solid #2a2c30; border-radius: 6px; background: #000; display: block; }
    .row { display: flex; gap: 10px; align-items: center; margin: 8px 0; flex-wrap: wrap; }
    select, input, button { background: #1f2125; color: #eee; border: 1px solid #3a3d42; padding: 9px 12px; border-radius: 6px; font-size: 14px; }
    select { min-width: 220px; }
    input[type=number] { width: 110px; }
    button { cursor: pointer; transition: background 0.12s; }
    button:hover { background: #2a2d33; }
    button.primary { background: #2a8c5a; border-color: #3fb37a; color: #fff; font-weight: 600; padding: 11px 18px; font-size: 15px; }
    button.primary:hover { background: #34a36b; }
    button.danger { background: #8c2a2a; border-color: #b34040; color: #fff; padding: 11px 18px; font-size: 15px; }
    button.danger:hover { background: #a33434; }
    button.rm { background: #2a2c30; border-color: #4a4c50; color: #f88; padding: 4px 10px; font-size: 13px; }
    table { width: 100%; border-collapse: collapse; margin-top: 10px; }
    th { text-align: left; padding: 8px 10px; border-bottom: 2px solid #2a2c30; font-size: 12px; color: #888; text-transform: uppercase; letter-spacing: 0.5px; }
    td { padding: 9px 10px; border-bottom: 1px solid #232427; font-size: 14px; }
    tr.step-row { transition: background 0.15s; }
    tr.done { color: #6a8; background: rgba(60, 140, 90, 0.08); }
    tr.done td.idx::before { content: \"✓ \"; color: #4cb37a; }
    tr.active { background: rgba(255, 200, 80, 0.18); color: #ffd870; font-weight: 600; box-shadow: inset 4px 0 0 #ffb840; }
    tr.active td.idx::before { content: \"▶ \"; color: #ffb840; }
    tr.pending { color: #888; }
    pre { background: #0a0a0c; padding: 10px; border-radius: 6px; max-height: 220px; overflow: auto; font-size: 12px; border: 1px solid #2a2c30; }
    .muted { color: #888; font-size: 13px; }
    .pill { display: inline-block; padding: 3px 9px; border-radius: 12px; font-size: 12px; font-weight: 600; letter-spacing: 0.3px; }
    .pill-idle { background: #2a2c30; color: #888; }
    .pill-running { background: #2a8c5a; color: #fff; }
    .pill-error { background: #8c2a2a; color: #fff; }
    .telemetry-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(140px, 1fr)); gap: 8px; margin-top: 12px; }
    .tile { background: #1a1c20; border: 1px solid #2a2c30; border-radius: 6px; padding: 8px 10px; }
    .tile .k { font-size: 11px; color: #888; text-transform: uppercase; letter-spacing: 0.5px; }
    .tile .v { font-size: 14px; font-weight: 600; margin-top: 2px; word-break: break-all; }
    .progress { height: 6px; background: #1a1c20; border-radius: 3px; overflow: hidden; margin-top: 8px; }
    .progress > div { height: 100%; background: linear-gradient(90deg, #2a8c5a, #4cb37a); transition: width 0.2s; }
  </style>
</head>
<body>
  <h1>car-calib · route dashboard</h1>
  <div class=\"layout\">
    <div class=\"panel\">
      <h2>Live stream</h2>
      <img id=\"stream\" class=\"stream\" alt=\"camera stream\">
      <h3>Telemetry</h3>
      <div class=\"telemetry-grid\" id=\"telemetryGrid\"></div>
    </div>
    <div class=\"panel\">
      <h2>Route script builder</h2>
      <div class=\"row\">
        <select id=\"action\">
          <option value=\"forward\">forward (FORWARD + center)</option>
          <option value=\"backward\">backward (BACKWARD + center)</option>
          <option value=\"left\">left (FORWARD + max left)</option>
          <option value=\"right\">right (FORWARD + max right)</option>
          <option value=\"straight\">straight (alias)</option>
          <option value=\"stop\">stop / pause</option>
        </select>
        <input id=\"duration\" type=\"number\" min=\"0\" step=\"0.5\" value=\"2\">
        <span class=\"muted\">seconds</span>
        <button id=\"add\">+ add step</button>
        <button id=\"clear\">clear all</button>
      </div>

      <h3>Steps</h3>
      <table id=\"steps\">
        <thead><tr><th style=\"width:50px\">#</th><th>Action</th><th style=\"width:120px\">Duration</th><th style=\"width:70px\"></th></tr></thead>
        <tbody></tbody>
      </table>

      <div class=\"row\" style=\"margin-top:18px\">
        <button id=\"run\" class=\"primary\">▶ run script (auto-record)</button>
        <button id=\"stop\" class=\"danger\">■ stop</button>
        <span id=\"runPill\" class=\"pill pill-idle\">idle</span>
        <span id=\"runDetail\" class=\"muted\"></span>
      </div>
      <div class=\"progress\"><div id=\"progressBar\" style=\"width:0%\"></div></div>

      <h3>JSON preview</h3>
      <pre id=\"preview\">{\n  \"steps\": []\n}</pre>
    </div>
  </div>
  <div class=\"panel\" style=\"margin-top:20px\">
    <h2>Recent routes</h2>
    <div class=\"row\"><button id=\"refreshRoutes\">↻ refresh</button><span class=\"muted\" id=\"routesMuted\">auto-refresh every 5s</span></div>
    <table id=\"routesTable\">
      <thead><tr><th>Route ID</th><th>Mode</th><th>Status</th><th>Frames</th><th>Elapsed</th><th>Zip size</th><th>Ended (UTC)</th><th></th></tr></thead>
      <tbody></tbody>
    </table>
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
const runPill = document.getElementById(\"runPill\");
const runDetail = document.getElementById(\"runDetail\");
const progressBar = document.getElementById(\"progressBar\");
const telemetryGrid = document.getElementById(\"telemetryGrid\");
let currentRunningStep = 0;
let isRunning = false;

function render() {
  tbody.innerHTML = \"\";
  steps.forEach((s, i) => {
    const tr = document.createElement(\"tr\");
    tr.className = \"step-row\";
    tr.dataset.idx = i;
    if (isRunning) {
      if (i + 1 < currentRunningStep) tr.classList.add(\"done\");
      else if (i + 1 === currentRunningStep) tr.classList.add(\"active\");
      else tr.classList.add(\"pending\");
    }
    tr.innerHTML = `<td class=\"idx\">${i+1}</td><td>${s.action}</td><td>${s.duration_s.toFixed(1)} s</td><td>${isRunning ? \"\" : `<button data-i=\"${i}\" class=\"rm\">×</button>`}</td>`;
    tbody.appendChild(tr);
  });
  preview.textContent = JSON.stringify({steps}, null, 2);
}

document.getElementById(\"add\").onclick = () => {
  if (isRunning) return;
  const action = document.getElementById(\"action\").value;
  const duration_s = parseFloat(document.getElementById(\"duration\").value || \"0\");
  if (!isFinite(duration_s) || duration_s < 0) return;
  steps.push({action, duration_s});
  render();
};
document.getElementById(\"clear\").onclick = () => {
  if (isRunning) return;
  steps.length = 0;
  render();
};
tbody.onclick = (e) => {
  if (isRunning) return;
  const t = e.target;
  if (t.classList.contains(\"rm\")) { steps.splice(parseInt(t.dataset.i, 10), 1); render(); }
};

document.getElementById(\"run\").onclick = async () => {
  if (steps.length === 0) { runDetail.textContent = \"add steps first\"; return; }
  runDetail.textContent = \"submitting…\";
  const r = await fetch(\"/route/script\" + qp, {method: \"POST\", headers: {\"Content-Type\": \"application/json\"}, body: JSON.stringify({steps})});
  const j = await r.json().catch(() => ({}));
  if (!r.ok) { runDetail.textContent = \"error: \" + JSON.stringify(j.detail || r.status); return; }
  runDetail.textContent = \"\";
};
document.getElementById(\"stop\").onclick = async () => {
  await fetch(\"/route/script/stop\" + qp, {method: \"POST\"});
  runDetail.textContent = \"stopping…\";
};

function setPill(klass, text) {
  runPill.className = \"pill \" + klass;
  runPill.textContent = text;
}

function buildTile(k, v) {
  const el = document.createElement(\"div\");
  el.className = \"tile\";
  el.innerHTML = `<div class=\"k\">${k}</div><div class=\"v\">${v ?? '-'}</div>`;
  return el;
}

function renderTelemetry(t) {
  telemetryGrid.innerHTML = \"\";
  const fields = [
    [\"route_id\", t.route_id],
    [\"mode\", t.route_mode],
    [\"fsm\", t.fsm_state],
    [\"theta\", t.theta != null ? Number(t.theta).toFixed(2) + \"°\" : null],
    [\"servo\", t.servo_angle != null ? Number(t.servo_angle).toFixed(2) + \"°\" : null],
    [\"frame\", t.frame_num],
  ];
  fields.forEach(([k, v]) => telemetryGrid.appendChild(buildTile(k, v)));
}

async function pollStatus() {
  try {
    const r = await fetch(\"/route/script/status\" + qp);
    if (r.ok) {
      const j = await r.json();
      const st = j.status || {};
      const wasRunning = isRunning;
      isRunning = !!st.running;
      currentRunningStep = st.current_step || 0;
      if (st.running) {
        setPill(\"pill-running\", `running ${currentRunningStep}/${st.total}`);
        const cur = st.step ? `${st.step.action} ${st.step.duration_s}s` : \"\";
        runDetail.textContent = cur;
        progressBar.style.width = (st.total ? (currentRunningStep / st.total * 100) : 0) + \"%\";
      } else if (st.last_error) {
        setPill(\"pill-error\", \"error\");
        runDetail.textContent = st.last_error;
        progressBar.style.width = \"0%\";
      } else {
        setPill(\"pill-idle\", \"idle\");
        if (wasRunning) runDetail.textContent = \"finished\";
        progressBar.style.width = \"0%\";
        currentRunningStep = 0;
      }
      if (wasRunning !== isRunning || st.running) render();
    }
  } catch (e) {}
  try {
    const r2 = await fetch(STATUS + qp);
    if (r2.ok) {
      const j2 = await r2.json();
      renderTelemetry(j2.telemetry || {});
    }
  } catch (e) {}
}
setInterval(pollStatus, 500);
render();
renderTelemetry({});

const routesTbody = document.querySelector(\"#routesTable tbody\");
function fmtBytes(n) {
  if (n == null) return \"-\";
  if (n < 1024) return n + \" B\";
  if (n < 1024*1024) return (n/1024).toFixed(1) + \" KB\";
  return (n/1024/1024).toFixed(1) + \" MB\";
}
function fmtElapsed(s) {
  if (s == null) return \"-\";
  return Number(s).toFixed(1) + \" s\";
}
function fmtTs(t) {
  if (!t) return \"-\";
  return t.replace(\"T\", \" \").replace(/\\.[0-9]+/, \"\").replace(\"+00:00\", \"Z\");
}
async function refreshRoutes() {
  try {
    const r = await fetch(\"/routes/list?limit=50\" + (TOKEN ? (\"&token=\" + encodeURIComponent(TOKEN)) : \"\"));
    if (!r.ok) return;
    const j = await r.json();
    const list = j.routes || [];
    routesTbody.innerHTML = \"\";
    list.forEach(r => {
      const tr = document.createElement(\"tr\");
      const dl = r.has_zip ? `<a class=\"pill pill-running\" style=\"text-decoration:none;padding:4px 10px\" href=\"/routes/download/${encodeURIComponent(r.route_id)}${qp}\">⬇ download</a>` : `<span class=\"muted\">no zip</span>`;
      tr.innerHTML = `<td>${r.route_id}</td><td>${r.route_mode||'-'}</td><td>${r.status||'-'}${r.accepted===false?' ✗':''}${r.accepted===true?' ✓':''}</td><td>${r.total_frames??'-'}</td><td>${fmtElapsed(r.elapsed_s)}</td><td>${fmtBytes(r.zip_size)}</td><td>${fmtTs(r.end_timestamp_utc)}</td><td>${dl}</td>`;
      routesTbody.appendChild(tr);
    });
  } catch (e) {}
}
document.getElementById(\"refreshRoutes\").onclick = refreshRoutes;
setInterval(refreshRoutes, 5000);
refreshRoutes();
</script>
</body>
</html>
"""
