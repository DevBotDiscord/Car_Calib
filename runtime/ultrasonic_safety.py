"""Non-blocking HC-SR04 safety gate for a Raspberry Pi pigpio runtime.

The echo pin is measured by pigpiod callbacks.  ``update`` only schedules a
10-us trigger pulse and checks deadlines, so it never waits for an echo in the
vision/control loop.
"""

from __future__ import annotations

import math
import os
import threading
import time
import logging
from dataclasses import dataclass
from typing import Any, Callable

try:
    import pigpio  # type: ignore[import-untyped]
    _PIGPIO_AVAILABLE = True
except ImportError:  # pragma: no cover - exercised on development hosts
    pigpio = None  # type: ignore[assignment]
    _PIGPIO_AVAILABLE = False

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class SonarConfig:
    enabled: bool = False
    trig_pin: int = 23
    echo_pin: int = 24
    interval_s: float = 0.10
    echo_timeout_s: float = 0.040
    stop_cm: float = 25.0
    clear_cm: float = 35.0
    stop_samples: int = 3
    clear_samples: int = 5
    fail_safe: bool = True
    fail_safe_timeout_s: float = 0.0
    min_distance_cm: float = 2.0
    max_distance_cm: float = 400.0

    def __post_init__(self) -> None:
        if self.trig_pin < 0 or self.echo_pin < 0 or self.trig_pin == self.echo_pin:
            raise ValueError("SONAR_TRIG_PIN and SONAR_ECHO_PIN must be distinct BCM pins")
        if not (0.02 <= self.interval_s <= 1.0):
            raise ValueError("SONAR_INTERVAL_S must be between 0.02 and 1.0")
        if not (0.005 <= self.echo_timeout_s <= self.interval_s):
            raise ValueError("SONAR_ECHO_TIMEOUT_S must be between 0.005 and SONAR_INTERVAL_S")
        if not (0.0 < self.stop_cm < self.clear_cm <= self.max_distance_cm):
            raise ValueError("SONAR_CLEAR_CM must be greater than SONAR_STOP_CM")
        if self.stop_samples < 1 or self.clear_samples < 1:
            raise ValueError("SONAR stop/clear sample counts must be at least 1")
        if self.fail_safe_timeout_s < 0.0:
            raise ValueError("SONAR_FAIL_SAFE_TIMEOUT_S must not be negative")


@dataclass(frozen=True)
class SonarStatus:
    enabled: bool
    available: bool
    blocked: bool
    distance_cm: float | None
    age_s: float | None
    valid_samples: int
    invalid_samples: int
    reason: str

    def telemetry(self) -> dict[str, Any]:
        return {
            "sonar_enabled": self.enabled,
            "sonar_available": self.available,
            "sonar_blocked": self.blocked,
            "sonar_distance_cm": (
                round(self.distance_cm, 1) if self.distance_cm is not None else None
            ),
            "sonar_age_s": round(self.age_s, 3) if self.age_s is not None else None,
            "sonar_valid_samples": self.valid_samples,
            "sonar_invalid_samples": self.invalid_samples,
            "sonar_reason": self.reason,
        }


class UltrasonicSafety:
    """HC-SR04 distance monitor with hysteresis and optional fail-safe stop."""

    def __init__(
        self,
        config: SonarConfig,
        *,
        host: str = "127.0.0.1",
        port: int = 8888,
        pi: Any | None = None,
        pigpio_module: Any | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.config = config
        self._clock = clock
        self._pigpio = pigpio if pigpio_module is None else pigpio_module
        self._pi = pi
        self._owns_pi = pi is None
        self._callback: Any | None = None
        self._lock = threading.Lock()
        self._available = False
        self._closed = False
        self._pending = False
        self._triggered_at: float | None = None
        self._last_trigger_at: float | None = None
        self._rise_tick: int | None = None
        self._last_sample_at: float | None = None
        self._distance_cm: float | None = None
        self._invalid_since: float | None = None
        self._valid_samples = 0
        self._invalid_samples = 0
        self._stop_count = 0
        self._clear_count = 0
        self._blocked = bool(config.enabled and config.fail_safe)
        self._reason = "starting" if self._blocked else ""

        if not config.enabled:
            return
        if self._pigpio is None:
            self._reason = "pigpio_unavailable"
            return
        try:
            if self._pi is None:
                self._pi = self._pigpio.pi(host, int(port))
            if self._pi is None or not self._pi.connected:
                self._pi = None
                self._reason = "pigpiod_unavailable"
                return
            self._pi.set_mode(config.trig_pin, self._pigpio.OUTPUT)
            self._pi.write(config.trig_pin, 0)
            self._pi.set_mode(config.echo_pin, self._pigpio.INPUT)
            if hasattr(self._pi, "set_pull_up_down"):
                self._pi.set_pull_up_down(config.echo_pin, self._pigpio.PUD_DOWN)
            self._callback = self._pi.callback(
                config.echo_pin,
                self._pigpio.EITHER_EDGE,
                self._on_echo,
            )
            self._available = True
        except Exception as exc:  # hardware must never abort the control loop
            self._pi = None
            self._reason = f"setup_error:{type(exc).__name__}"

    @classmethod
    def from_env(cls, *, host: str = "127.0.0.1", port: int = 8888) -> "UltrasonicSafety":
        enabled = _env_bool("SONAR_ENABLED", False)
        try:
            config = SonarConfig(
                enabled=enabled,
                trig_pin=_env_int("SONAR_TRIG_PIN", 23),
                echo_pin=_env_int("SONAR_ECHO_PIN", 24),
                interval_s=_env_float("SONAR_INTERVAL_S", 0.10),
                echo_timeout_s=_env_float("SONAR_ECHO_TIMEOUT_S", 0.040),
                stop_cm=_env_float("SONAR_STOP_CM", 25.0),
                clear_cm=_env_float("SONAR_CLEAR_CM", 35.0),
                stop_samples=_env_int("SONAR_STOP_SAMPLES", 3),
                clear_samples=_env_int("SONAR_CLEAR_SAMPLES", 5),
                fail_safe=_env_bool("SONAR_FAIL_SAFE", True),
                fail_safe_timeout_s=_env_float("SONAR_FAIL_SAFE_TIMEOUT_S", 0.0),
            )
        except ValueError as exc:
            # A malformed optional sensor config must not abort steering.
            # If the sensor was enabled, retain a fail-safe default instead.
            logger.error("Invalid sonar configuration: %s; using safe defaults", exc)
            config = SonarConfig(enabled=enabled, fail_safe=True)
        return cls(config, host=host, port=port)

    def update(self, *, now: float | None = None) -> SonarStatus:
        """Schedule one measurement when due; this method never blocks for Echo."""
        ts = self._clock() if now is None else float(now)
        with self._lock:
            if not self.config.enabled or self._closed:
                return self._status_locked(ts)
            if not self._available or self._pi is None:
                self._apply_fail_safe_locked(ts, self._reason or "unavailable")
                return self._status_locked(ts)
            if self._pending:
                if self._triggered_at is not None and ts - self._triggered_at >= self.config.echo_timeout_s:
                    self._pending = False
                    self._rise_tick = None
                    self._record_invalid_locked("echo_timeout", ts)
                return self._status_locked(ts)
            if self._last_trigger_at is not None and ts - self._last_trigger_at < self.config.interval_s:
                return self._status_locked(ts)
            self._pending = True
            self._triggered_at = ts
            self._last_trigger_at = ts
            self._rise_tick = None
            try:
                self._pi.gpio_trigger(self.config.trig_pin, 10, 1)
            except Exception as exc:
                self._pending = False
                self._record_invalid_locked(f"trigger_error:{type(exc).__name__}", ts)
            return self._status_locked(ts)

    def status(self, *, now: float | None = None) -> SonarStatus:
        ts = self._clock() if now is None else float(now)
        with self._lock:
            return self._status_locked(ts)

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            callback = self._callback
            pi_client = self._pi
            owns_pi = self._owns_pi
            self._callback = None
            self._pi = None
            self._available = False

        # Cancel outside the lock: pigpio may wait for an in-flight callback,
        # which itself takes this lock.
        if callback is not None:
            try:
                callback.cancel()
            except Exception:
                pass
        if pi_client is not None:
            try:
                pi_client.write(self.config.trig_pin, 0)
            except Exception:
                pass
            if owns_pi:
                try:
                    pi_client.stop()
                except Exception:
                    pass

    def _on_echo(self, _gpio: int, level: int, tick: int) -> None:
        ts = self._clock()
        with self._lock:
            if self._closed or not self._pending:
                return
            if level == 1:  # rising edge
                self._rise_tick = int(tick)
                return
            if level != 0 or self._rise_tick is None:  # watchdog/noise
                return
            pulse_us = self._tick_diff(self._rise_tick, int(tick))
            self._pending = False
            self._rise_tick = None
            distance_cm = pulse_us * 0.01715
            if not math.isfinite(distance_cm) or not (
                self.config.min_distance_cm <= distance_cm <= self.config.max_distance_cm
            ):
                self._record_invalid_locked("invalid_echo", ts)
                return
            self._record_distance_locked(distance_cm, ts)

    def _tick_diff(self, start_tick: int, end_tick: int) -> int:
        if self._pigpio is not None and hasattr(self._pigpio, "tickDiff"):
            return int(self._pigpio.tickDiff(start_tick, end_tick))
        return (end_tick - start_tick) & 0xFFFFFFFF

    def _record_distance_locked(self, distance_cm: float, ts: float) -> None:
        self._distance_cm = distance_cm
        self._last_sample_at = ts
        self._invalid_since = None
        self._valid_samples += 1
        if distance_cm <= self.config.stop_cm:
            self._stop_count += 1
            self._clear_count = 0
            if self._stop_count >= self.config.stop_samples:
                self._blocked = True
                self._reason = "distance"
        elif distance_cm >= self.config.clear_cm:
            self._clear_count += 1
            self._stop_count = 0
            if self._clear_count >= self.config.clear_samples:
                self._blocked = False
                self._reason = ""
        else:
            self._stop_count = 0
            self._clear_count = 0

    def _record_invalid_locked(self, reason: str, ts: float) -> None:
        self._invalid_samples += 1
        if self._invalid_since is None:
            self._invalid_since = ts
        self._apply_fail_safe_locked(ts, reason)

    def _apply_fail_safe_locked(self, ts: float, reason: str) -> None:
        if not self.config.fail_safe:
            return
        if self._invalid_since is None:
            self._invalid_since = ts
        if ts - self._invalid_since >= self.config.fail_safe_timeout_s:
            self._blocked = True
            self._reason = reason

    def _status_locked(self, ts: float) -> SonarStatus:
        age_s = None if self._last_sample_at is None else max(0.0, ts - self._last_sample_at)
        return SonarStatus(
            enabled=self.config.enabled,
            available=self._available,
            blocked=self._blocked,
            distance_cm=self._distance_cm,
            age_s=age_s,
            valid_samples=self._valid_samples,
            invalid_samples=self._invalid_samples,
            reason=self._reason,
        )


def _env_bool(name: str, default: bool) -> bool:
    value = os.getenv(name)
    return default if value is None else value.strip().lower() in {"1", "true", "yes", "on"}


def _env_int(name: str, default: int) -> int:
    value = os.getenv(name)
    try:
        return default if value is None else int(value)
    except ValueError:
        return default


def _env_float(name: str, default: float) -> float:
    value = os.getenv(name)
    try:
        return default if value is None else float(value)
    except ValueError:
        return default
