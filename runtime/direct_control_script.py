"""Route script runner for direct GPIO control."""

from __future__ import annotations

import logging
import threading
import time
from typing import Any

from config.settings import SERVO_CENTER_ANGLE
from runtime import script_lock
from runtime.route_script import (
    SCRIPT_LEFT_ANGLE,
    SCRIPT_REPUBLISH_HZ,
    SCRIPT_RIGHT_ANGLE,
    validate_steps,
)

logger = logging.getLogger(__name__)


class DirectControlScriptRunner:
    """Runs dashboard route steps against local drivers."""

    def __init__(
        self,
        servo: Any,
        base: Any,
        relay: Any | None = None,
        on_route: Any | None = None,
    ) -> None:
        self._servo = servo
        self._base = base
        self._relay = relay
        self._on_route = on_route
        self._lock = threading.Lock()
        self._meta_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._pending_meta: dict[str, Any] | None = None
        self._state: dict[str, Any] = {
            "running": False,
            "current_step": 0,
            "total": 0,
            "step": None,
            "started_at": None,
            "last_error": None,
            "transport": "direct",
        }

    def consume_pending_meta(self) -> dict[str, Any] | None:
        with self._meta_lock:
            meta = self._pending_meta
            self._pending_meta = None
            return meta

    def is_running(self) -> bool:
        with self._lock:
            return bool(self._state["running"])

    def is_servo_pinned(self) -> bool:
        return script_lock.is_pinned()

    def status(self) -> dict[str, Any]:
        with self._lock:
            status = dict(self._state)
        status["servo_pinned"] = self.is_servo_pinned()
        status["relay"] = getattr(self._relay, "relay_state", None)
        return status

    def submit(
        self,
        steps: list[dict[str, Any]],
        preset_name: str | None = None,
        description: str | None = None,
    ) -> bool:
        steps = validate_steps(steps)
        with self._lock:
            if self._state["running"]:
                return False
            self._state.update(
                running=True,
                current_step=0,
                total=len(steps),
                step=None,
                started_at=time.time(),
                last_error=None,
            )
        with self._meta_lock:
            self._pending_meta = {
                "source": "dashboard_direct_runner",
                "preset_name": preset_name,
                "description": description,
                "steps": list(steps),
                "submitted_at_unix": time.time(),
            }
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, args=(steps,), daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        self._stop_event.set()
        self._base.stop()

    def close(self) -> None:
        self.stop()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def publish_relay(self, on: bool) -> None:
        if self._relay is None:
            return
        self._relay.set(bool(on))

    def _run(self, steps: list[dict[str, Any]]) -> None:
        logger.info("Direct route script start (%d steps)", len(steps))
        self._publish_route("START")
        try:
            for idx, step in enumerate(steps):
                if self._stop_event.is_set():
                    break
                with self._lock:
                    self._state["current_step"] = idx + 1
                    self._state["step"] = dict(step)
                self._execute_step(step)
            self._base.stop()
            self._servo.center()
        except Exception as exc:  # noqa: BLE001
            logger.exception("Direct route script crashed: %s", exc)
            with self._lock:
                self._state["last_error"] = str(exc)
            self._base.stop()
            self._servo.center()
        finally:
            self._publish_route("STOP")
            script_lock.set_pinned(False)
            with self._lock:
                self._state["running"] = False
                self._state["step"] = None
            logger.info("Direct route script finished")

    def _publish_route(self, command: str) -> None:
        if self._on_route is None:
            return
        try:
            self._on_route(command)
        except Exception as exc:  # noqa: BLE001
            logger.error("Direct route callback failed: %s", exc)

    def _execute_step(self, step: dict[str, Any]) -> None:
        action = step["action"]
        duration_s = float(step["duration_s"])
        if action in ("forward", "straight"):
            base_cmd = "FORWARD"
            angle: float | None = None
        elif action == "backward":
            base_cmd = "BACKWARD"
            angle = SERVO_CENTER_ANGLE
        elif action == "left":
            base_cmd = "FORWARD"
            angle = SCRIPT_LEFT_ANGLE
        elif action == "right":
            base_cmd = "FORWARD"
            angle = SCRIPT_RIGHT_ANGLE
        elif action == "turn_left":
            base_cmd = "TURN_LEFT"
            angle = None
        elif action == "turn_right":
            base_cmd = "TURN_RIGHT"
            angle = None
        else:
            base_cmd = "STOP"
            angle = None

        script_lock.set_pinned(angle is not None)
        self._base.command(base_cmd)
        if angle is not None:
            self._servo.send_angle(angle)

        period = 1.0 / max(1.0, SCRIPT_REPUBLISH_HZ)
        deadline = time.monotonic() + duration_s
        try:
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0 or self._stop_event.is_set():
                    break
                time.sleep(min(period, remaining))
                if angle is not None:
                    self._servo.send_angle(angle)
        finally:
            if action in ("left", "right"):
                self._servo.center()
            script_lock.set_pinned(False)
