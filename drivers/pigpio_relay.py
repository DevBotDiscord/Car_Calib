"""Direct relay + power pulse control through pigpio on Raspberry Pi."""

from __future__ import annotations

import logging
import threading
import time
from typing import Any

logger = logging.getLogger(__name__)

try:
    import pigpio  # type: ignore[import-untyped]
    _PIGPIO_AVAILABLE = True
except ImportError:  # pragma: no cover
    pigpio = None  # type: ignore[assignment]
    _PIGPIO_AVAILABLE = False


class PigpioRelayDriver:
    """Relay + power pulse using BCM pins."""

    def __init__(
        self,
        relay_pin: int = 13,
        power_relay_pin: int = 5,
        relay_active_low: bool = False,
        power_active_low: bool = False,
        power_on_pulse_ms: int = 100,
        power_off_pulse_ms: int = 3000,
        host: str = "127.0.0.1",
        port: int = 8888,
        pi: Any | None = None,
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
        self._owns_pi = pi is None
        self._pi = pi

        if not _PIGPIO_AVAILABLE:
            logger.warning("pigpio module not installed - relay/power disabled")
            return
        if self._pi is None:
            self._pi = pigpio.pi(host, int(port))
        if not self._pi.connected:
            logger.warning("pigpiod not connected at %s:%s - relay/power disabled", host, port)
            self._pi = None
            return

        for pin in (self._relay_pin, self._power_pin):
            self._pi.set_mode(pin, pigpio.OUTPUT)
        self._set_relay(False)
        self._set_power(False)
        logger.info(
            "pigpio relay: relay=BCM%d power=BCM%d active_low=%s,%s",
            self._relay_pin,
            self._power_pin,
            self._relay_active_low,
            self._power_active_low,
        )

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
        self._pulse(self._power_on_ms, "ON")

    def power_off(self) -> None:
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
        if self._pi is not None and self._owns_pi:
            self._pi.stop()

    def _relay_level(self, on: bool) -> int:
        return 0 if (on != self._relay_active_low) else 1

    def _power_level(self, on: bool) -> int:
        return 0 if (on != self._power_active_low) else 1

    def _set_relay(self, on: bool) -> None:
        if self._pi is not None:
            self._pi.write(self._relay_pin, self._relay_level(on))

    def _set_power(self, on: bool) -> None:
        if self._pi is not None:
            self._pi.write(self._power_pin, self._power_level(on))

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
