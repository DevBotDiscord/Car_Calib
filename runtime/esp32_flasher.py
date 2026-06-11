"""ESP32 firmware flasher — compile an uploaded .ino and flash it over USB.

A .ino is Arduino source, not a flashable image, so this module:

1. Saves the uploaded sketch into a temp Arduino sketch folder.
2. Compiles it with ``arduino-cli compile`` for the configured FQBN.
3. Pauses the running ESP32 serial bridge so the port is free.
4. Flashes with ``arduino-cli upload`` to the detected port.
5. Resumes the bridge.

Everything runs in a background thread; the dashboard polls ``status()``.
The flasher never raises out of its thread — failures land in the status
object with the captured log tail.
"""

from __future__ import annotations

import logging
import shutil
import subprocess
import tempfile
import threading
import time
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


class ESP32Flasher:
    """Compile + flash an uploaded .ino to the ESP32 in a background thread."""

    def __init__(
        self,
        *,
        fqbn: str = "esp32:esp32:esp32",
        bridge: Any | None = None,
        port_globs: tuple[str, ...] = ("/dev/ttyUSB*", "/dev/ttyACM*"),
        arduino_cli: str = "arduino-cli",
    ) -> None:
        self._fqbn = fqbn
        self._bridge = bridge
        self._port_globs = port_globs
        self._arduino_cli = arduino_cli
        self._lock = threading.Lock()
        self._state: dict[str, Any] = {
            "phase": "idle",       # idle|saved|compiling|flashing|done|error
            "message": "",
            "log": "",
            "updated_at": None,
        }
        self._sketch_dir: Path | None = None
        self._thread: threading.Thread | None = None

    # ------------------------------------------------------------------ #
    # status
    # ------------------------------------------------------------------ #
    def status(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._state)

    def _set(self, phase: str, message: str = "", log_append: str = "") -> None:
        with self._lock:
            self._state["phase"] = phase
            if message:
                self._state["message"] = message
            if log_append:
                # keep only the last ~8KB of log so status stays small
                self._state["log"] = (self._state["log"] + log_append)[-8192:]
            self._state["updated_at"] = time.time()

    def is_busy(self) -> bool:
        with self._lock:
            return self._state["phase"] in ("saved", "compiling", "flashing")

    # ------------------------------------------------------------------ #
    # upload sketch
    # ------------------------------------------------------------------ #
    def save_sketch(self, ino_text: str) -> None:
        """Write the uploaded .ino into a fresh temp sketch dir.

        Arduino requires the .ino basename to match the folder name.
        """
        if self.is_busy():
            raise RuntimeError("flash already in progress")
        # clean previous
        if self._sketch_dir is not None:
            shutil.rmtree(self._sketch_dir, ignore_errors=True)
        base = Path(tempfile.mkdtemp(prefix="esp32_sketch_"))
        sketch_name = "uploaded_sketch"
        sketch_folder = base / sketch_name
        sketch_folder.mkdir(parents=True, exist_ok=True)
        (sketch_folder / f"{sketch_name}.ino").write_text(ino_text, encoding="utf-8")
        self._sketch_dir = sketch_folder
        with self._lock:
            self._state = {
                "phase": "saved",
                "message": f"sketch saved ({len(ino_text)} bytes)",
                "log": "",
                "updated_at": time.time(),
            }

    # ------------------------------------------------------------------ #
    # flash (background)
    # ------------------------------------------------------------------ #
    def start_flash(self) -> None:
        if self.is_busy() and self._state["phase"] != "saved":
            raise RuntimeError("flash already in progress")
        if self._sketch_dir is None or not self._sketch_dir.exists():
            raise RuntimeError("no sketch uploaded")
        self._thread = threading.Thread(target=self._flash_worker, daemon=True, name="esp32-flash")
        self._thread.start()

    def _run(self, args: list[str], timeout: float) -> tuple[int, str]:
        """Run a subprocess, capture combined output, append to log."""
        self._set(self._state["phase"], log_append=f"$ {' '.join(args)}\n")
        try:
            proc = subprocess.run(
                args, capture_output=True, text=True, timeout=timeout,
            )
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
        sketch = self._sketch_dir
        paused_port: str | None = None
        try:
            # 1. compile
            self._set("compiling", "compiling sketch…")
            rc, _ = self._run(
                [self._arduino_cli, "compile", "--fqbn", self._fqbn, str(sketch)],
                timeout=300.0,
            )
            if rc != 0:
                self._set("error", f"compile failed (rc={rc})")
                return

            # 2. resolve port (pause bridge to free it)
            port = None
            if self._bridge is not None:
                try:
                    port = self._bridge.current_port()
                except Exception:  # noqa: BLE001
                    port = None
            if not port:
                port = self._first_port()
            if not port:
                self._set("error", "no ESP32 serial port detected")
                return

            if self._bridge is not None:
                try:
                    paused_port = self._bridge.pause()
                    time.sleep(1.0)  # let the OS release the handle
                except Exception as exc:  # noqa: BLE001
                    self._set("flashing", log_append=f"bridge pause warning: {exc}\n")

            # 3. flash
            self._set("flashing", f"flashing {port}…")
            rc, _ = self._run(
                [self._arduino_cli, "upload", "-p", port, "--fqbn", self._fqbn, str(sketch)],
                timeout=180.0,
            )
            if rc != 0:
                self._set("error", f"flash failed (rc={rc})")
                return
            self._set("done", "firmware updated successfully")
        except Exception as exc:  # noqa: BLE001 — never let the thread die
            self._set("error", f"unexpected: {exc}")
        finally:
            # always resume the bridge so the actuator path recovers
            if self._bridge is not None and paused_port is not None:
                try:
                    self._bridge.resume()
                except Exception:  # noqa: BLE001
                    pass

    def _first_port(self) -> str | None:
        import glob
        for pattern in self._port_globs:
            hits = sorted(glob.glob(pattern))
            if hits:
                return hits[0]
        return None
