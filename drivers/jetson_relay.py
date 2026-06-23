"""Jetson Nano relay + power control via GPIO.

Mimics ESP8266 relay/power behavior:
  - relay: ON/OFF toggle via single GPIO pin
  - power: pulse-to-toggle (100ms ON, 3000ms OFF) via second GPIO pin

Default pins (BOARD numbering):
  RELAY_PIN       → pin 18 (BCM 24)
  POWER_RELAY_PIN → pin 16 (BCM 23)
"""

from __future__ import annotations

import logging
import time
import threading
from typing import Any

logger = logging.getLogger(__name__)

try:
    import Jetson.GPIO as GPIO  # type: ignore[import-untyped]
    _GPIO_AVAILABLE = True
except ImportError:
    GPIO = None  # type: ignore[assignment]
    _GPIO_AVAILABLE = False

DEFAULT_RELAY_PIN = 18        # BOARD 18
DEFAULT_POWER_RELAY_PIN = 16  # BOARD 16


class JetsonRelayDriver:
    """Relay + ignition power control for Jetson Nano."""

    def __init__(
        self,
        relay_pin: int = DEFAULT_RELAY_PIN,
        power_relay_pin: int = DEFAULT_POWER_RELAY_PIN,
        relay_active_low: bool = True,
        power_active_low: bool = True,
        power_on_pulse_ms: int = 100,
        power_off_pulse_ms: int = 3000,
    ) -> None:
        self._relay_pin = int(relay_pin)
        self._power_pin = int(power_relay_pin)
        self._relay_active_low = bool(relay_active_low)
        self._power_active_low = bool(power_active_low)
        self._power_on_ms = max(50, int(power_on_pulse_ms))
        self._power_off_ms = max(100, int(power_off_pulse_ms))
        self._relay_on = False
        self._power_pulsing = False
        self._lock = threading.Lock()

        if not _GPIO_AVAILABLE:
            logger.warning("Jetson.GPIO not installed — relay/power disabled")
            return

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self._relay_pin, GPIO.OUT, initial=self._relay_level(False))
        GPIO.setup(self._power_pin, GPIO.OUT, initial=self._power_level(False))
        logger.info(
            "Jetson relay: relay=pin%d power=pin%d active_low=%s,%s",
            self._relay_pin, self._power_pin,
            self._relay_active_low, self._power_active_low,
        )

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #

    def relay_on(self) -> None:
        with self._lock:
            self._relay_on = True
            self._set_relay(True)
            logger.info("Relay: ON")

    def relay_off(self) -> None:
        with self._lock:
            self._relay_on = False
            self._set_relay(False)
            logger.info("Relay: OFF")

    def relay_toggle(self) -> bool:
        if self._relay_on:
            self.relay_off()
        else:
            self.relay_on()
        return self._relay_on

    def power_on(self) -> None:
        """Pulse the power relay to turn ignition ON (short pulse)."""
        self._pulse(self._power_on_ms, "ON")

    def power_off(self) -> None:
        """Pulse the power relay to turn ignition OFF (long pulse)."""
        self._pulse(self._power_off_ms, "OFF")

    @property
    def relay_state(self) -> bool:
        return self._relay_on

    @property
    def power_pulsing(self) -> bool:
        return self._power_pulsing

    def close(self) -> None:
        self._set_relay(False)
        self._set_power(False)

    # ------------------------------------------------------------------ #
    # Internal
    # ------------------------------------------------------------------ #

    def _relay_level(self, on: bool) -> int:
        return GPIO.LOW if (on != self._relay_active_low) else GPIO.HIGH

    def _power_level(self, on: bool) -> int:
        return GPIO.LOW if (on != self._power_active_low) else GPIO.HIGH

    def _set_relay(self, on: bool) -> None:
        if _GPIO_AVAILABLE:
            GPIO.output(self._relay_pin, self._relay_level(on))

    def _set_power(self, on: bool) -> None:
        if _GPIO_AVAILABLE:
            GPIO.output(self._power_pin, self._power_level(on))

    def _pulse(self, duration_ms: int, label: str) -> None:
        logger.info("Power: pulsing %s (%d ms)", label, duration_ms)
        self._power_pulsing = True
        try:
            self._set_power(True)
            time.sleep(duration_ms / 1000.0)
            self._set_power(False)
        finally:
            self._power_pulsing = False
        logger.info("Power: pulse %s complete", label)
