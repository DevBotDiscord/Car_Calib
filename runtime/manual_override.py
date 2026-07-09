"""Manual joystick override state and command mapping."""

from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Any


class ManualOverrideError(ValueError):
    """Raised when a joystick override payload is invalid."""


@dataclass(frozen=True)
class ManualOverrideConfig:
    enabled: bool = True
    timeout_s: float = 0.3
    deadzone: float = 0.15
    center_angle: float = 90.0
    max_steer: float = 60.0


@dataclass(frozen=True)
class ManualOverrideDecision:
    active: bool
    timed_out: bool
    drive: float
    steer: float
    base_command: str
    servo_angle: float | None
    release_servo: bool
    pause_script: bool
    blocked_reason: str
    age_s: float | None

    def telemetry(self) -> dict[str, Any]:
        return {
            "manual_override_active": self.active,
            "manual_override_age_s": round(self.age_s, 3) if self.age_s is not None else None,
            "manual_drive": round(self.drive, 3),
            "manual_steer": round(self.steer, 3),
            "manual_blocked_reason": self.blocked_reason,
        }


class ManualOverrideController:
    """Hold-to-drive joystick override with heartbeat timeout."""

    def __init__(self, config: ManualOverrideConfig) -> None:
        self.config = config
        self._lock = threading.Lock()
        self._active = False
        self._drive = 0.0
        self._steer = 0.0
        self._last_seen: float | None = None
        self._release_pending = False

    @classmethod
    def from_env(cls, *, center_angle: float, max_steer: float) -> "ManualOverrideController":
        return cls(
            ManualOverrideConfig(
                enabled=_env_bool("MANUAL_OVERRIDE_ENABLED", True),
                timeout_s=_env_float("MANUAL_OVERRIDE_TIMEOUT_S", 0.3),
                deadzone=_env_float("MANUAL_OVERRIDE_DEADZONE", 0.15),
                center_angle=float(center_angle),
                max_steer=abs(float(max_steer)),
            )
        )

    def submit(self, payload: dict[str, Any], *, now: float | None = None) -> dict[str, Any]:
        if not isinstance(payload, dict):
            raise ManualOverrideError("payload must be an object")
        if "active" not in payload:
            raise ManualOverrideError("active is required")
        active = payload["active"]
        if not isinstance(active, bool):
            raise ManualOverrideError("active must be boolean")
        drive = _read_axis(payload, "drive", required=active)
        steer = _read_axis(payload, "steer", required=active)
        if "ts" in payload:
            _read_number(payload["ts"], "ts")

        ts = time.monotonic() if now is None else float(now)
        with self._lock:
            if not self.config.enabled:
                self._active = False
                self._drive = 0.0
                self._steer = 0.0
                self._last_seen = ts
            else:
                self._active = active
                self._drive = drive if active else 0.0
                self._steer = steer if active else 0.0
                self._last_seen = ts
                if not active:
                    self._release_pending = True
        return self.status(now=ts)

    def wants_control(self, *, now: float | None = None) -> bool:
        ts = time.monotonic() if now is None else float(now)
        with self._lock:
            return self._active and not self._is_expired_locked(ts)

    def evaluate(
        self,
        *,
        now: float | None = None,
        object_near: bool = False,
        estop_active: bool = False,
    ) -> ManualOverrideDecision:
        ts = time.monotonic() if now is None else float(now)
        with self._lock:
            timed_out = False
            released = False
            if self._active and self._is_expired_locked(ts):
                timed_out = True
                self._active = False
                self._drive = 0.0
                self._steer = 0.0
            if self._release_pending:
                released = True
                self._release_pending = False
            active = self.config.enabled and self._active
            drive = self._drive if active else 0.0
            steer = self._steer if active else 0.0
            age_s = None if self._last_seen is None else max(0.0, ts - self._last_seen)

        if timed_out:
            return ManualOverrideDecision(False, True, 0.0, 0.0, "STOP", None, True, False, "timeout", age_s)
        if released:
            return ManualOverrideDecision(False, False, 0.0, 0.0, "STOP", None, True, False, "", age_s)
        if not active:
            return ManualOverrideDecision(False, False, 0.0, 0.0, "STOP", None, False, False, "", age_s)
        if estop_active:
            return ManualOverrideDecision(True, False, drive, steer, "STOP", None, True, True, "estop", age_s)

        base_command = _base_command_for_drive(drive, self.config.deadzone)
        blocked_reason = ""
        if object_near and base_command == "FORWARD":
            base_command = "STOP"
            blocked_reason = "object_near"
        servo_angle = self._servo_angle_for_steer(steer)
        return ManualOverrideDecision(
            True,
            False,
            drive,
            steer,
            base_command,
            servo_angle,
            False,
            True,
            blocked_reason,
            age_s,
        )

    def status(self, *, now: float | None = None) -> dict[str, Any]:
        ts = time.monotonic() if now is None else float(now)
        with self._lock:
            expired = self._active and self._is_expired_locked(ts)
            active = self.config.enabled and self._active and not expired
            age_s = None if self._last_seen is None else max(0.0, ts - self._last_seen)
            drive = self._drive if active else 0.0
            steer = self._steer if active else 0.0
        return {
            "manual_override_active": active,
            "manual_override_age_s": round(age_s, 3) if age_s is not None else None,
            "manual_drive": round(drive, 3),
            "manual_steer": round(steer, 3),
            "manual_blocked_reason": "timeout" if expired else "",
        }

    def _is_expired_locked(self, now: float) -> bool:
        return self._last_seen is None or now - self._last_seen > max(0.0, self.config.timeout_s)

    def _servo_angle_for_steer(self, steer: float) -> float:
        if abs(steer) < self.config.deadzone:
            steer = 0.0
        return max(0.0, min(180.0, self.config.center_angle - steer * self.config.max_steer))


def _base_command_for_drive(drive: float, deadzone: float) -> str:
    if drive > deadzone:
        return "FORWARD"
    if drive < -deadzone:
        return "BACKWARD"
    return "STOP"


def _read_axis(payload: dict[str, Any], key: str, *, required: bool) -> float:
    if key not in payload:
        if required:
            raise ManualOverrideError(f"{key} is required")
        return 0.0
    return max(-1.0, min(1.0, _read_number(payload[key], key)))


def _read_number(value: Any, key: str) -> float:
    if isinstance(value, bool):
        raise ManualOverrideError(f"{key} must be a number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ManualOverrideError(f"{key} must be a number") from exc
    if not math.isfinite(number):
        raise ManualOverrideError(f"{key} must be finite")
    return number


def _env_bool(name: str, default: bool) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _env_float(name: str, default: float) -> float:
    value = os.getenv(name)
    if value is None:
        return default
    try:
        return float(value)
    except ValueError:
        return default
