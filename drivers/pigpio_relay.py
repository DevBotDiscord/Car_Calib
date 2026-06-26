"""Direct relay control through pigpio on Raspberry Pi."""

from __future__ import annotations

import logging
from typing import Any

logger = logging.getLogger(__name__)

try:
    import pigpio  # type: ignore[import-untyped]
except ImportError:  # pragma: no cover
    pigpio = None  # type: ignore[assignment]


class PigpioRelayDriver:
    """Single relay output using BCM pin."""

    def __init__(
        self,
        relay_pin: int = 13,
        active_low: bool = False,
        host: str = "127.0.0.1",
        port: int = 8888,
        pi: Any | None = None,
    ) -> None:
        self._pin = int(relay_pin)
        self._active_low = bool(active_low)
        self._on = False
        self._owns_pi = pi is None
        self._pi = pi

        if pigpio is None:
            logger.warning("pigpio module not installed - relay disabled")
            return
        if self._pi is None:
            self._pi = pigpio.pi(host, int(port))
        if not self._pi.connected:
            logger.warning("pigpiod not connected at %s:%s - relay disabled", host, port)
            self._pi = None
            return

        self._pi.set_mode(self._pin, pigpio.OUTPUT)
        self.set(False)
        logger.info("pigpio relay: BCM%d active_low=%s", self._pin, self._active_low)

    @property
    def relay_state(self) -> bool:
        return self._on

    def set(self, on: bool) -> None:
        self._on = bool(on)
        if self._pi is not None:
            self._pi.write(self._pin, self._level(self._on))
        logger.info("Relay: %s", "ON" if self._on else "OFF")

    def close(self) -> None:
        self.set(False)
        if self._pi is not None and self._owns_pi:
            self._pi.stop()

    def _level(self, on: bool) -> int:
        return 0 if (on == self._active_low) else 1
