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

        def _check_token(request: Any) -> None:
            if not self._token:
                return
            candidate = request.query_params.get("token", "")
            if candidate != self._token:
                raise HTTPException(status_code=401, detail="Unauthorized")

        @app.get(self._status_path)
        def status(request: Any) -> Any:
            _check_token(request)
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
        def snapshot(request: Any) -> Any:
            _check_token(request)
            jpeg, _, _ = self._frame_store.snapshot()
            if jpeg is None:
                return Response(status_code=503, content=b"No frame available")
            return Response(content=jpeg, media_type="image/jpeg")

        @app.get(self._stream_path)
        def stream(request: Any) -> Any:
            _check_token(request)

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
