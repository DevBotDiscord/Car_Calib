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
        rpi_status_provider: Any | None = None,
        steering_controller: Any | None = None,
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
        self._rpi_status_provider = rpi_status_provider
        self._steering_controller = steering_controller

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
        staticfiles_module = importlib.import_module("fastapi.staticfiles")
        FastAPI = fastapi_module.FastAPI
        HTTPException = fastapi_module.HTTPException
        Request = fastapi_module.Request
        JSONResponse = responses_module.JSONResponse
        Response = responses_module.Response
        StreamingResponse = responses_module.StreamingResponse
        StaticFiles = staticfiles_module.StaticFiles

        app = FastAPI(title="Robot Debug Stream", docs_url=None, redoc_url=None)

        dashboard_dir = Path(__file__).parent / "dashboard"
        app.mount(
            "/dashboard/static",
            StaticFiles(directory=str(dashboard_dir)),
            name="dashboard_static",
        )

        def _check_token(candidate: str) -> None:
            if not self._token:
                return
            if candidate != self._token:
                raise HTTPException(status_code=401, detail="Unauthorized")

        @app.get(self._status_path)
        def status(token: str = "") -> Any:
            _check_token(token)
            _, ts, telemetry = self._frame_store.snapshot()
            rpi_status: dict[str, Any] | None = None
            provider = self._rpi_status_provider
            if provider is not None:
                try:
                    rpi_status = provider()
                except Exception:  # noqa: BLE001
                    rpi_status = None
            return JSONResponse(
                {
                    "ok": True,
                    "has_frame": ts is not None,
                    "last_frame_unix": ts,
                    "telemetry": telemetry,
                    "rpi_status": rpi_status,
                }
            )

        @app.get(self._snapshot_path)
        def snapshot(token: str = "") -> Any:
            _check_token(token)
            jpeg, _, _ = self._frame_store.snapshot()
            if jpeg is None:
                return Response(status_code=503, content=b"No frame available")
            return Response(content=jpeg, media_type="image/jpeg")

        @app.get("/control/params")
        def get_control_params(token: str = "") -> Any:
            _check_token(token)
            ctrl = self._steering_controller
            if ctrl is None:
                return JSONResponse({"available": False, "params": None, "bounds": None})
            return JSONResponse({
                "available": True,
                "params": ctrl.get_params(),
                "bounds": ctrl.PARAM_BOUNDS,
            })

        @app.post("/control/params")
        async def post_control_params(request: Request, token: str = "") -> Any:
            _check_token(token)
            ctrl = self._steering_controller
            if ctrl is None:
                raise HTTPException(status_code=503, detail="steering controller not available")
            try:
                body = await request.json()
            except Exception as exc:  # noqa: BLE001
                raise HTTPException(status_code=400, detail=f"invalid JSON: {exc}")
            try:
                new_params = ctrl.update_params(body)
            except ValueError as exc:
                raise HTTPException(status_code=400, detail=str(exc))
            return JSONResponse({"params": new_params, "bounds": ctrl.PARAM_BOUNDS})

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
            template_path = dashboard_dir / "index.html"
            html = template_path.read_text(encoding="utf-8")
            html = (
                html.replace("__STREAM_PATH__", self._stream_path)
                .replace("__STATUS_PATH__", self._status_path)
                .replace("__TOKEN__", self._token)
            )
            return Response(
                content=html,
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

            preset_name = (payload.get("preset_name") if isinstance(payload, dict) else None) or None
            description = (payload.get("description") if isinstance(payload, dict) else None) or None
            if preset_name is not None:
                preset_name = str(preset_name)[:_PRESET_NAME_MAX]
            if description is not None:
                description = str(description)[:512]

            if not self._script_runner.submit(steps, preset_name=preset_name, description=description):
                raise HTTPException(status_code=409, detail="script_already_running")
            return JSONResponse({"ok": True, "status": self._script_runner.status()})

        @app.post("/route/script/stop")
        def route_script_stop(token: str = "") -> Any:
            _check_token(token)
            if self._script_runner is None:
                raise HTTPException(status_code=503, detail="script_runner_disabled")
            self._script_runner.stop()
            return JSONResponse({"ok": True, "status": self._script_runner.status()})

        @app.post("/route/relay")
        def route_relay(on: int = 0, token: str = "") -> Any:
            _check_token(token)
            if self._script_runner is None:
                raise HTTPException(status_code=503, detail="script_runner_disabled")
            self._script_runner.publish_relay(bool(on))
            return JSONResponse({"ok": True, "relay": "ON" if on else "OFF"})

        @app.get("/routes/list")
        def routes_list(token: str = "", limit: int = 30) -> Any:
            _check_token(token)
            from config.settings import ROUTE_LOG_ROOT
            root = _resolve_route_root(ROUTE_LOG_ROOT)
            if not root.exists():
                return JSONResponse({"ok": True, "routes": []})
            entries: list[dict[str, Any]] = []
            for child in sorted(root.iterdir(), reverse=True):
                if not child.is_dir():
                    continue
                if child.name.startswith("_"):
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
            zip_path = _resolve_route_zip(name)
            return Response(
                content=zip_path.read_bytes(),
                media_type="application/zip",
                headers={"Content-Disposition": f'attachment; filename=\"{name}.zip\"'},
            )

        @app.get("/routes/{name}/summary")
        def routes_summary(name: str, token: str = "") -> Any:
            _check_token(token)
            from config.settings import ROUTE_LOG_ROOT
            safe = _safe_basename(name)
            root = _resolve_route_root(ROUTE_LOG_ROOT)
            summary_path = (root / safe / "route_summary.json").resolve()
            try:
                root_resolved = root.resolve()
            except OSError as exc:
                raise HTTPException(status_code=500, detail=f"root_resolve: {exc}") from exc
            if root_resolved not in summary_path.parents:
                raise HTTPException(status_code=400, detail="path_outside_root")
            if not summary_path.exists():
                raise HTTPException(status_code=404, detail="summary_not_found")
            try:
                return JSONResponse({"ok": True, "summary": _load_summary_full(summary_path)})
            except Exception as exc:  # noqa: BLE001
                raise HTTPException(status_code=500, detail=f"read_failed: {exc}") from exc

        @app.delete("/routes/{name}")
        def routes_delete(name: str, token: str = "") -> Any:
            _check_token(token)
            removed = _delete_route_dir_and_zip(name)
            return JSONResponse({"ok": True, "removed": removed})

        @app.post("/routes/delete_all")
        def routes_delete_all(token: str = "") -> Any:
            _check_token(token)
            from config.settings import ROUTE_LOG_ROOT
            root = _resolve_route_root(ROUTE_LOG_ROOT)
            count = 0
            errors: list[str] = []
            if root.exists():
                import shutil as _shutil
                for child in list(root.iterdir()):
                    if child.name.startswith("_"):
                        continue  # keep _presets and similar metadata
                    try:
                        if child.is_dir():
                            _shutil.rmtree(child)
                            count += 1
                        elif child.suffix == ".zip":
                            child.unlink()
                            count += 1
                    except Exception as exc:  # noqa: BLE001
                        errors.append(f"{child.name}: {exc}")
            return JSONResponse({"ok": not errors, "removed": count, "errors": errors})

        @app.get("/presets")
        def presets_list(token: str = "") -> Any:
            _check_token(token)
            return JSONResponse({"ok": True, "presets": _presets_list()})

        @app.get("/presets/{name}")
        def presets_get(name: str, token: str = "") -> Any:
            _check_token(token)
            data = _preset_load(name)
            if data is None:
                raise HTTPException(status_code=404, detail="preset_not_found")
            return JSONResponse({"ok": True, "preset": data})

        @app.put("/presets/{name}")
        async def presets_put(name: str, request: _FastAPIRequest, token: str = "") -> Any:
            _check_token(token)
            try:
                payload = await request.json()
            except Exception as exc:  # noqa: BLE001
                raise HTTPException(status_code=400, detail=f"invalid_json: {exc}") from exc
            if not isinstance(payload, dict):
                raise HTTPException(status_code=400, detail="payload_must_be_object")
            from runtime.route_script import validate_steps
            try:
                steps = validate_steps(payload.get("steps"))
            except ValueError as exc:
                raise HTTPException(status_code=400, detail=str(exc)) from exc
            try:
                _preset_save(name, steps, payload.get("description"))
            except ValueError as exc:
                raise HTTPException(status_code=400, detail=str(exc)) from exc
            return JSONResponse({"ok": True, "preset": _preset_load(name)})

        @app.delete("/presets/{name}")
        def presets_delete(name: str, token: str = "") -> Any:
            _check_token(token)
            removed = _preset_delete(name)
            if not removed:
                raise HTTPException(status_code=404, detail="preset_not_found")
            return JSONResponse({"ok": True})

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
    extra = raw.get("extra_meta") or {}
    script = (extra.get("script") if isinstance(extra, dict) else None) or {}
    return {
        "route_mode": raw.get("route_mode"),
        "status": raw.get("status"),
        "accepted": raw.get("accepted"),
        "total_frames": raw.get("total_frames"),
        "elapsed_s": raw.get("total_elapsed_seconds"),
        "end_timestamp_utc": raw.get("end_timestamp_utc"),
        "preset_name": script.get("preset_name") if isinstance(script, dict) else None,
        "script_source": script.get("source") if isinstance(script, dict) else None,
    }


def _load_summary_full(path: Path) -> dict[str, Any]:
    import json as _json
    return _json.loads(path.read_text(encoding="utf-8"))


def _resolve_route_root(route_log_root: str) -> Path:
    root = Path(route_log_root)
    if not root.exists():
        fallback = Path("logs/routes")
        if fallback.exists():
            return fallback
    return root


def _safe_basename(name: str) -> str:
    if "/" in name or "\\" in name or ".." in name or name.startswith("."):
        raise _http_invalid_name()
    return name


def _http_invalid_name() -> Exception:
    fastapi_module = importlib.import_module("fastapi")
    return fastapi_module.HTTPException(status_code=400, detail="invalid_name")


def _resolve_route_zip(name: str) -> Path:
    from config.settings import ROUTE_LOG_ROOT
    name = _safe_basename(name)
    root = _resolve_route_root(ROUTE_LOG_ROOT)
    zip_path = (root / f"{name}.zip").resolve()
    try:
        root_resolved = root.resolve()
    except OSError as exc:
        fastapi_module = importlib.import_module("fastapi")
        raise fastapi_module.HTTPException(status_code=500, detail=f"root_resolve: {exc}") from exc
    if zip_path.parent != root_resolved:
        fastapi_module = importlib.import_module("fastapi")
        raise fastapi_module.HTTPException(status_code=400, detail="path_outside_root")
    if not zip_path.exists():
        fastapi_module = importlib.import_module("fastapi")
        raise fastapi_module.HTTPException(status_code=404, detail="zip_not_found")
    return zip_path


def _delete_route_dir_and_zip(name: str) -> dict[str, bool]:
    import shutil as _shutil
    from config.settings import ROUTE_LOG_ROOT
    name = _safe_basename(name)
    root = _resolve_route_root(ROUTE_LOG_ROOT)
    try:
        root_resolved = root.resolve()
    except OSError as exc:
        fastapi_module = importlib.import_module("fastapi")
        raise fastapi_module.HTTPException(status_code=500, detail=f"root_resolve: {exc}") from exc
    out = {"dir": False, "zip": False}
    dir_path = (root / name).resolve()
    if dir_path.parent == root_resolved and dir_path.exists() and dir_path.is_dir():
        _shutil.rmtree(dir_path)
        out["dir"] = True
    zip_path = (root / f"{name}.zip").resolve()
    if zip_path.parent == root_resolved and zip_path.exists():
        zip_path.unlink()
        out["zip"] = True
    return out


# ----- preset CRUD ---------------------------------------------------------

_PRESET_NAME_MAX = 64


def _preset_dir() -> Path:
    from config.settings import ROUTE_LOG_ROOT
    root = _resolve_route_root(ROUTE_LOG_ROOT)
    p = root / "_presets"
    p.mkdir(parents=True, exist_ok=True)
    return p


def _preset_path(name: str) -> Path:
    if not name or len(name) > _PRESET_NAME_MAX:
        raise ValueError(f"preset name length must be 1..{_PRESET_NAME_MAX}")
    safe = _safe_basename(name)
    # restrict charset to ASCII alnum + dash/underscore/space
    for ch in safe:
        if not (ch.isalnum() or ch in "-_ "):
            raise ValueError(f"preset name char not allowed: {ch!r}")
    return _preset_dir() / f"{safe}.json"


def _preset_save(name: str, steps: list[dict[str, Any]], description: Any | None) -> None:
    import json as _json
    path = _preset_path(name)
    payload = {
        "name": name,
        "description": str(description) if description is not None else "",
        "steps": steps,
        "updated_at": datetime.now(timezone.utc).isoformat(),
    }
    path.write_text(_json.dumps(payload, indent=2), encoding="utf-8")


def _preset_load(name: str) -> dict[str, Any] | None:
    import json as _json
    try:
        path = _preset_path(name)
    except ValueError:
        return None
    if not path.exists():
        return None
    try:
        return _json.loads(path.read_text(encoding="utf-8"))
    except Exception:  # noqa: BLE001
        return None


def _preset_delete(name: str) -> bool:
    try:
        path = _preset_path(name)
    except ValueError:
        return False
    if not path.exists():
        return False
    path.unlink()
    return True


def _presets_list() -> list[dict[str, Any]]:
    import json as _json
    out: list[dict[str, Any]] = []
    pdir = _preset_dir()
    if not pdir.exists():
        return out
    for child in sorted(pdir.glob("*.json")):
        try:
            data = _json.loads(child.read_text(encoding="utf-8"))
            out.append({
                "name": data.get("name", child.stem),
                "description": data.get("description", ""),
                "steps_count": len(data.get("steps", []) or []),
                "updated_at": data.get("updated_at"),
            })
        except Exception:  # noqa: BLE001
            continue
    return out


def _normalize_path(path: str) -> str:
    if not path:
        return "/"
    return path if path.startswith("/") else f"/{path}"


