"""ESP32 firmware flasher — compile the board-matched sketch and flash over USB.

The dashboard asks this module to update firmware; the module detects the
attached Arduino board, selects the matching local sketch folder/FQBN, compiles,
and uploads. Everything runs in a background thread; failures land in status().
"""

from __future__ import annotations

import json
import logging
import shutil
import subprocess
import tempfile
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class BoardProfile:
    key: str
    label: str
    fqbn: str
    sketch_dir: Path
    match_tokens: tuple[str, ...]


_REPO_ROOT = Path(__file__).resolve().parents[1]
_DEFAULT_PROFILES = (
    BoardProfile(
        key="esp32",
        label="ESP32 Dev Module",
        fqbn="esp32:esp32:esp32",
        sketch_dir=_REPO_ROOT / "firmware" / "esp32_mqtt_bridge_esp32",
        match_tokens=("esp32:esp32:esp32", "esp32 dev module", "esp32"),
    ),
    BoardProfile(
        key="esp32s3",
        label="ESP32-S3 Dev Module",
        fqbn="esp32:esp32:esp32s3",
        sketch_dir=_REPO_ROOT / "firmware" / "esp32_mqtt_bridge_esp32s3",
        match_tokens=("esp32:esp32:esp32s3", "esp32-s3", "esp32s3", "s3"),
    ),
    BoardProfile(
        key="esp8266",
        label="ESP8266 NodeMCU 1.0",
        fqbn="esp8266:esp8266:nodemcuv2",
        sketch_dir=_REPO_ROOT / "firmware" / "esp8266_serial_bridge",
        match_tokens=("esp8266:esp8266:nodemcuv2", "nodemcu", "esp8266"),
    ),
)


class ESP32Flasher:
    """Detect board, compile matching sketch, and flash in a background thread."""

    def __init__(
        self,
        *,
        fqbn: str = "esp32:esp32:esp32",
        fqbn_esp8266: str = "esp8266:esp8266:nodemcuv2",
        bridge: Any | None = None,
        port_globs: tuple[str, ...] = ("/dev/ttyUSB*", "/dev/ttyACM*"),
        arduino_cli: str = "arduino-cli",
        board_profiles: tuple[BoardProfile, ...] = _DEFAULT_PROFILES,
    ) -> None:
        self._fallback_fqbn = fqbn
        self._fqbn_esp8266 = fqbn_esp8266
        self._bridge = bridge
        self._port_globs = port_globs
        self._arduino_cli = arduino_cli
        self._board_profiles = board_profiles
        self._lock = threading.Lock()
        self._state: dict[str, Any] = {
            "phase": "idle",       # idle|saved|detecting|compiling|flashing|done|error
            "message": "",
            "log": "",
            "updated_at": None,
            "board": None,
            "port": None,
            "fqbn": None,
            "sketch": None,
            "profiles": self._profile_summaries(),
        }
        self._uploaded_text: str | None = None
        self._thread: threading.Thread | None = None

    # ------------------------------------------------------------------ #
    # status
    # ------------------------------------------------------------------ #
    def status(self) -> dict[str, Any]:
        with self._lock:
            out = dict(self._state)
            out["profiles"] = self._profile_summaries()
            return out

    def _set(self, phase: str, message: str = "", log_append: str = "", **updates: Any) -> None:
        with self._lock:
            self._state["phase"] = phase
            if message:
                self._state["message"] = message
            if log_append:
                self._state["log"] = (self._state["log"] + log_append)[-8192:]
            self._state.update(updates)
            self._state["updated_at"] = time.time()

    def _profile_summaries(self) -> list[dict[str, str]]:
        return [
            {"key": p.key, "label": p.label, "fqbn": p.fqbn, "sketch": str(p.sketch_dir)}
            for p in self._board_profiles
        ]

    def is_busy(self) -> bool:
        with self._lock:
            return self._state["phase"] in ("saved", "detecting", "compiling", "flashing")

    # ------------------------------------------------------------------ #
    # upload optional sketch
    # ------------------------------------------------------------------ #
    def save_sketch(self, ino_text: str) -> None:
        """Store an optional uploaded .ino override for the next flash."""
        if self.is_busy():
            raise RuntimeError("flash already in progress")
        self._uploaded_text = ino_text
        with self._lock:
            self._state = {
                "phase": "saved",
                "message": f"uploaded sketch saved ({len(ino_text)} bytes)",
                "log": "",
                "updated_at": time.time(),
                "board": None,
                "port": None,
                "fqbn": None,
                "sketch": "uploaded",
                "profiles": self._profile_summaries(),
            }

    # ------------------------------------------------------------------ #
    # flash (background)
    # ------------------------------------------------------------------ #
    def start_flash(self) -> None:
        if self.is_busy() and self._state["phase"] != "saved":
            raise RuntimeError("flash already in progress")
        self._thread = threading.Thread(target=self._flash_worker, daemon=True, name="esp32-flash")
        self._thread.start()

    def _run(self, args: list[str], timeout: float) -> tuple[int, str]:
        self._set(self._state["phase"], log_append=f"$ {' '.join(args)}\n")
        try:
            proc = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
        except FileNotFoundError as exc:
            self._set("error", f"{args[0]} not found", f"{exc}\n")
            return 127, str(exc)
        except subprocess.TimeoutExpired as exc:
            self._set("error", "command timed out", f"timeout after {timeout}s\n")
            return 124, str(exc)
        out = (proc.stdout or "") + (proc.stderr or "")
        self._set(self._state["phase"], log_append=out + "\n")
        return proc.returncode, out

    def _flash_worker(self) -> None:
        paused_port: str | None = None
        temp_root: Path | None = None
        try:
            self._set("detecting", "detecting board…")
            port = self._resolve_port()
            if not port:
                self._set("error", "no ESP32 serial port detected")
                return

            detected = self._detect_profile(port)
            profile = self._with_configured_fqbn(detected) if detected else self._fallback_profile()
            if detected is None:
                self._set(
                    "detecting",
                    f"board unknown on {port}; using fallback {profile.label}",
                    log_append=f"board detect fallback: {profile.fqbn}\n",
                )

            sketch = profile.sketch_dir
            if self._uploaded_text is not None:
                temp_root = Path(tempfile.mkdtemp(prefix="esp32_sketch_"))
                sketch_name = f"uploaded_{profile.key}"
                sketch = temp_root / sketch_name
                sketch.mkdir(parents=True, exist_ok=True)
                (sketch / f"{sketch_name}.ino").write_text(self._uploaded_text, encoding="utf-8")

            if not sketch.exists():
                self._set("error", f"sketch folder missing: {sketch}")
                return

            self._set(
                "compiling",
                f"compiling {profile.label}…",
                board={"key": profile.key, "label": profile.label},
                port=port,
                fqbn=profile.fqbn,
                sketch=str(sketch),
            )
            rc, _ = self._run([self._arduino_cli, "compile", "--fqbn", profile.fqbn, str(sketch)], timeout=300.0)
            if rc != 0:
                self._set("error", f"compile failed (rc={rc})")
                return

            if self._bridge is not None:
                try:
                    paused_port = self._bridge.pause()
                    time.sleep(1.0)
                except Exception as exc:  # noqa: BLE001
                    self._set("flashing", log_append=f"bridge pause warning: {exc}\n")

            self._set("flashing", f"flashing {profile.label} on {port}…")
            rc, _ = self._run(
                [self._arduino_cli, "upload", "-p", port, "--fqbn", profile.fqbn, str(sketch)],
                timeout=180.0,
            )
            if rc != 0:
                self._set("error", f"flash failed (rc={rc})")
                return
            self._set("done", f"firmware updated for {profile.label}")
        except Exception as exc:  # noqa: BLE001
            self._set("error", f"unexpected: {exc}")
        finally:
            if self._bridge is not None and paused_port is not None:
                try:
                    self._bridge.resume()
                except Exception:  # noqa: BLE001
                    pass
            if temp_root is not None:
                shutil.rmtree(temp_root, ignore_errors=True)

    def _resolve_port(self) -> str | None:
        if self._bridge is not None:
            try:
                port = self._bridge.current_port()
                if port:
                    return port
            except Exception:  # noqa: BLE001
                pass
        return self._first_port()

    def _detect_profile(self, port: str) -> BoardProfile | None:
        rc, out = self._run([self._arduino_cli, "board", "list", "--format", "json"], timeout=30.0)
        if rc != 0:
            return None
        text = out.strip()
        try:
            boards = json.loads(text or "[]")
        except json.JSONDecodeError:
            boards = []
        port_rows = [row for row in boards if str(row.get("port", {}).get("address", "")) == port]
        haystack = json.dumps(port_rows or boards).lower() if boards else out.lower()
        for profile in sorted(self._board_profiles, key=lambda p: len(p.key), reverse=True):
            if any(token.lower() in haystack for token in profile.match_tokens):
                return profile
        return None

    def _with_configured_fqbn(self, profile: BoardProfile) -> BoardProfile:
        if profile.key == "esp8266" and profile.fqbn != self._fqbn_esp8266:
            return BoardProfile(profile.key, profile.label, self._fqbn_esp8266, profile.sketch_dir, profile.match_tokens)
        return profile

    def _fallback_profile(self) -> BoardProfile:
        for profile in self._board_profiles:
            if profile.fqbn == self._fallback_fqbn:
                return self._with_configured_fqbn(profile)
        return self._with_configured_fqbn(self._board_profiles[0])

    def _first_port(self) -> str | None:
        import glob
        for pattern in self._port_globs:
            hits = sorted(glob.glob(pattern))
            if hits:
                return hits[0]
        return None
