"""Route script runner for Jetson direct control — drives base, servo, relay."""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable

logger = logging.getLogger(__name__)

BASE_MAP: dict[str, tuple[int, int, int]] = {
    "STOP": (0, 0, 0),
    "FORWARD": (0, 1, 0),
    "BACKWARD": (0, 0, 1),
    "LOCK": (1, 0, 1),
    "UNLOCK": (1, 1, 0),
    "TURN_LEFT": (1, 0, 0),
    "TURN_RIGHT": (0, 1, 1),
}

_VALID_ACTIONS = {"forward", "backward", "straight", "left", "right", "turn_left", "turn_right", "stop", "pause"}


class JetsonScriptRunner:
    """Runs route scripts in a background thread using direct callbacks."""

    def __init__(self) -> None:
        self._thread: threading.Thread | None = None
        self._running = False
        self._steps: list[dict[str, Any]] = []
        self._current_step: dict[str, Any] | None = None
        self._base_cb: Callable[[str], None] | None = None
        self._servo_cb: Callable[[float], None] | None = None
        self._relay_cb: Callable[[str], None] | None = None
        self._center_angle: float = -8.0
        self._max_steer: float = 60.0

    def set_handlers(
        self,
        base_cb: Callable[[str], None],
        servo_cb: Callable[[float], None],
        relay_cb: Callable[[str], None],
    ) -> None:
        self._base_cb = base_cb
        self._servo_cb = servo_cb
        self._relay_cb = relay_cb

    def is_running(self) -> bool:
        return self._running

    def status(self) -> dict[str, Any]:
        return {
            "running": self._running,
            "current_step": self._current_step,
            "total": len(self._steps),
        }

    def submit(self, steps: list[dict[str, Any]]) -> bool:
        if self._running:
            return False
        self._running = True
        self._steps = steps
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        self._running = False

    # ------------------------------------------------------------------ #
    # Worker
    # ------------------------------------------------------------------ #

    def _run(self) -> None:
        logger.info("Route script start (%d steps)", len(self._steps))
        try:
            for step in self._steps:
                if not self._running:
                    break
                self._current_step = step
                self._execute_step(step)
        except Exception as exc:
            logger.exception("Route script crashed: %s", exc)
        finally:
            self._current_step = None
            self._running = False
            if self._base_cb is not None:
                self._base_cb("STOP")
            logger.info("Route script finished")

    def _execute_step(self, step: dict[str, Any]) -> None:
        action = step.get("action", "stop")
        duration_s = float(step.get("duration_s", 1.0))

        if action in ("forward", "straight"):
            base_cmd = "FORWARD"
            angle: float | None = None
        elif action == "backward":
            base_cmd = "BACKWARD"
            angle = self._center_angle
        elif action == "left":
            base_cmd = "FORWARD"
            angle = self._center_angle + self._max_steer
        elif action == "right":
            base_cmd = "FORWARD"
            angle = self._center_angle - self._max_steer
        elif action == "turn_left":
            base_cmd = "TURN_LEFT"
            angle = None
        elif action == "turn_right":
            base_cmd = "TURN_RIGHT"
            angle = None
        else:  # stop / pause
            base_cmd = "STOP"
            angle = None

        if self._base_cb is not None:
            self._base_cb(base_cmd)
        if angle is not None and self._servo_cb is not None:
            self._servo_cb(angle)

        deadline = time.monotonic() + duration_s
        while self._running and time.monotonic() < deadline:
            time.sleep(0.05)
