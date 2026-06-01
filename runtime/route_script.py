"""Route script runner — manual route builder backend.

Drives the car through a list of ``(action, duration_s)`` steps by publishing
MQTT commands to ``car/base/command`` and ``car/servo/angle``. Wraps each run
in a route session via ``car/control/route`` so the MiniPC vision loop logs
the matching CSV + MP4 automatically.

Action types (servo angles follow signed-angle convention used by gamepad
control: more negative = LEFT, more positive = RIGHT):

* ``forward``  / ``straight`` — base FORWARD, servo CENTER
* ``backward``                 — base BACKWARD, servo CENTER
* ``left``                     — base FORWARD, servo LEFT  (max permissible)
* ``right``                    — base FORWARD, servo RIGHT (max permissible)
* ``turn_left``                — base TURN_LEFT (GPIO 1 0 0), no servo pin
* ``turn_right``               — base TURN_RIGHT (GPIO 0 1 1), no servo pin
* ``stop`` / ``pause``         — base STOP

Re-publishes the servo angle at ``REPUBLISH_HZ`` so the vision PID stream does
not steal the steering target during the step window.
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time
from typing import Any

from runtime import script_lock

try:
    import paho.mqtt.client as mqtt
except ImportError:  # pragma: no cover - optional dependency
    mqtt = None  # type: ignore[assignment]

from config.settings import (
    MQTT_BASE_COMMAND_TOPIC,
    MQTT_BROKER_HOST,
    MQTT_BROKER_PORT,
    MQTT_CLIENT_ID_PREFIX,
    MQTT_KEEPALIVE_S,
    MQTT_PASSWORD,
    MQTT_RELAY_TOPIC,
    MQTT_SERVO_TOPIC,
    MQTT_USERNAME,
)

logger = logging.getLogger(__name__)


_VALID_ACTIONS = {"forward", "backward", "straight", "left", "right", "turn_left", "turn_right", "stop", "pause"}


def _env_float(name: str, default: float) -> float:
    raw = os.getenv(name)
    if raw is None:
        return default
    try:
        return float(raw)
    except ValueError:
        return default


SCRIPT_CENTER_ANGLE = _env_float("ROUTE_SCRIPT_CENTER_ANGLE", _env_float("SERVO_CENTER_ANGLE", -8.0))
SCRIPT_MAX_STEER = _env_float("ROUTE_SCRIPT_MAX_STEER", _env_float("SERVO_MAX_ANGLE_DEG", 45.0))
SCRIPT_LEFT_ANGLE = _env_float("ROUTE_SCRIPT_LEFT_ANGLE", SCRIPT_CENTER_ANGLE + SCRIPT_MAX_STEER)
SCRIPT_RIGHT_ANGLE = _env_float("ROUTE_SCRIPT_RIGHT_ANGLE", SCRIPT_CENTER_ANGLE - SCRIPT_MAX_STEER)
SCRIPT_REPUBLISH_HZ = _env_float("ROUTE_SCRIPT_REPUBLISH_HZ", 10.0)
SCRIPT_MAX_DURATION_S = _env_float("ROUTE_SCRIPT_MAX_DURATION_S", 30.0)
SCRIPT_MAX_STEPS = int(_env_float("ROUTE_SCRIPT_MAX_STEPS", 64))


def validate_steps(raw_steps: Any) -> list[dict[str, Any]]:
    """Validate a payload list of steps and return normalized dicts."""
    if not isinstance(raw_steps, list):
        raise ValueError("steps must be a list")
    if len(raw_steps) == 0:
        raise ValueError("steps must not be empty")
    if len(raw_steps) > SCRIPT_MAX_STEPS:
        raise ValueError(f"steps exceeds maximum {SCRIPT_MAX_STEPS}")

    normalized: list[dict[str, Any]] = []
    for idx, raw in enumerate(raw_steps):
        if not isinstance(raw, dict):
            raise ValueError(f"step #{idx} must be an object")
        action = str(raw.get("action", "")).strip().lower()
        if action not in _VALID_ACTIONS:
            raise ValueError(f"step #{idx} action {action!r} not in {sorted(_VALID_ACTIONS)}")
        try:
            duration = float(raw.get("duration_s", 0.0))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"step #{idx} duration_s invalid: {exc}") from exc
        if duration < 0.0:
            raise ValueError(f"step #{idx} duration_s must be >= 0")
        if duration > SCRIPT_MAX_DURATION_S:
            raise ValueError(
                f"step #{idx} duration_s exceeds {SCRIPT_MAX_DURATION_S}s safety cap"
            )
        normalized.append({"action": action, "duration_s": duration})
    return normalized


class RouteScriptRunner:
    """Runs route scripts in a background thread and reports status."""

    def __init__(self) -> None:
        if mqtt is None:
            raise RuntimeError("paho-mqtt is not available; cannot run route scripts")

        self._lock = threading.Lock()
        self._state: dict[str, Any] = {
            "running": False,
            "current_step": 0,
            "total": 0,
            "step": None,
            "started_at": None,
            "last_error": None,
        }
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._mqtt = self._build_client()
        self._pending_meta: dict[str, Any] | None = None
        self._meta_lock = threading.Lock()

    def consume_pending_meta(self) -> dict[str, Any] | None:
        """Atomic getter for the most recent script-meta blob (and clear it)."""
        with self._meta_lock:
            meta = self._pending_meta
            self._pending_meta = None
            return meta

    # ------------------------------------------------------------------ #
    # MQTT setup
    # ------------------------------------------------------------------ #

    def _build_client(self) -> Any:
        client_id = f"{MQTT_CLIENT_ID_PREFIX}-route-script-{os.getpid()}"
        callback_api_version = getattr(mqtt, "CallbackAPIVersion", None)
        if callback_api_version is not None:
            client = mqtt.Client(
                callback_api_version=callback_api_version.VERSION1,
                client_id=client_id,
            )
        else:
            client = mqtt.Client(client_id=client_id)
        if MQTT_USERNAME:
            client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        client.reconnect_delay_set(min_delay=1, max_delay=5)
        client.connect_async(MQTT_BROKER_HOST, MQTT_BROKER_PORT, keepalive=MQTT_KEEPALIVE_S)
        client.loop_start()
        logger.info(
            "RouteScriptRunner MQTT client connecting to %s:%d", MQTT_BROKER_HOST, MQTT_BROKER_PORT
        )
        return client

    def close(self) -> None:
        self.stop()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        try:
            self._mqtt.disconnect()
        except Exception:  # noqa: BLE001
            pass
        try:
            self._mqtt.loop_stop()
        except Exception:  # noqa: BLE001
            pass

    # ------------------------------------------------------------------ #
    # Public API used by HTTPS server
    # ------------------------------------------------------------------ #

    def is_running(self) -> bool:
        with self._lock:
            return bool(self._state["running"])

    def status(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._state)

    def submit(self, steps: list[dict[str, Any]], preset_name: str | None = None, description: str | None = None) -> bool:
        with self._lock:
            if self._state["running"]:
                return False
            self._state["running"] = True
            self._state["current_step"] = 0
            self._state["total"] = len(steps)
            self._state["step"] = None
            self._state["started_at"] = time.time()
            self._state["last_error"] = None
        with self._meta_lock:
            self._pending_meta = {
                "source": "dashboard_script_runner",
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

    # ------------------------------------------------------------------ #
    # Worker
    # ------------------------------------------------------------------ #

    def _run(self, steps: list[dict[str, Any]]) -> None:
        logger.info("Route script start (%d steps)", len(steps))
        self._publish_script_active("ON")
        self._publish_route("START")
        try:
            for idx, step in enumerate(steps):
                if self._stop_event.is_set():
                    logger.info("Route script stop requested at step %d", idx)
                    break
                with self._lock:
                    self._state["current_step"] = idx + 1
                    self._state["step"] = dict(step)
                self._execute_step(step)
            self._publish_neutral()
        except Exception as exc:  # noqa: BLE001
            logger.exception("Route script crashed: %s", exc)
            with self._lock:
                self._state["last_error"] = str(exc)
            self._publish_neutral()
        finally:
            self._publish_route("STOP")
            self._publish_script_active("OFF")
            with self._lock:
                self._state["running"] = False
                self._state["step"] = None
            logger.info("Route script finished")

    def _execute_step(self, step: dict[str, Any]) -> None:
        action = step["action"]
        duration_s = float(step["duration_s"])

        # forward/straight: defer steering to vision PID (do not publish
        # servo angle so the PID stream wins). All other actions pin the
        # servo to a fixed angle while the step is active.
        if action in ("forward", "straight"):
            base_cmd = "FORWARD"
            angle: float | None = None
        elif action == "backward":
            base_cmd = "BACKWARD"
            angle = SCRIPT_CENTER_ANGLE
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
        else:  # stop / pause
            base_cmd = "STOP"
            angle = None

        # Tell the local servo driver (vision PID side) to back off while
        # this step pins the servo, so its 30Hz stream does not fight our
        # 10Hz republish on the same MQTT topic.
        script_lock.set_pinned(angle is not None)

        self._publish_base(base_cmd)
        if angle is not None:
            self._publish_angle(angle)

        # Re-publish pinned servo target at REPUBLISH_HZ to outvote the
        # vision PID stream while the step window is active. Base command
        # is sticky on the RPi side so we only publish it once at start.
        # When angle is None (forward/straight, stop) the script lets the
        # PID stream drive the servo unchanged.
        period = 1.0 / max(1.0, SCRIPT_REPUBLISH_HZ)
        deadline = time.monotonic() + duration_s
        try:
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0 or self._stop_event.is_set():
                    break
                time.sleep(min(period, remaining))
                if angle is not None:
                    self._publish_angle(angle)
        finally:
            # Recenter the servo to home angle after a steering step so the
            # wheels do not stay locked at full lock between steps. Publish
            # while still pinned (vision PID backed off) so the center target
            # lands before control is released.
            if action in ("left", "right"):
                self._publish_angle(SCRIPT_CENTER_ANGLE)
            script_lock.set_pinned(False)

    def _publish_neutral(self) -> None:
        # Stop the base and recenter the servo before releasing control.
        # Script_active is still ON here so the center command lands on
        # the RPi servo path; the caller flips it OFF afterwards.
        self._publish_base("STOP")
        self._publish_angle(SCRIPT_CENTER_ANGLE)

    # ------------------------------------------------------------------ #
    # MQTT publishes
    # ------------------------------------------------------------------ #

    def _publish_base(self, command: str) -> None:
        try:
            self._mqtt.publish(MQTT_BASE_COMMAND_TOPIC, command, qos=1)
        except Exception as exc:  # noqa: BLE001
            logger.error("RouteScriptRunner base publish failed: %s", exc)

    def publish_relay(self, on: bool) -> None:
        """Publish a relay ON/OFF command (dashboard light toggle)."""
        command = "ON" if on else "OFF"
        try:
            self._mqtt.publish(MQTT_RELAY_TOPIC, command, qos=1)
        except Exception as exc:  # noqa: BLE001
            logger.error("RouteScriptRunner relay publish failed: %s", exc)

    def _publish_angle(self, angle: float) -> None:
        payload = json.dumps({"angle": int(round(angle))})
        try:
            self._mqtt.publish(MQTT_SERVO_TOPIC, payload, qos=0)
        except Exception as exc:  # noqa: BLE001
            logger.error("RouteScriptRunner servo publish failed: %s", exc)

    def _publish_route(self, command: str) -> None:
        try:
            self._mqtt.publish("car/control/route", command, qos=1)
        except Exception as exc:  # noqa: BLE001
            logger.error("RouteScriptRunner route publish failed: %s", exc)

    def _publish_script_active(self, command: str) -> None:
        try:
            self._mqtt.publish("car/control/script_active", command, qos=1, retain=True)
        except Exception as exc:  # noqa: BLE001
            logger.error("RouteScriptRunner script_active publish failed: %s", exc)
