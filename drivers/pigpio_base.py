"""Direct base motor GPIO control through pigpio on Raspberry Pi."""

from __future__ import annotations

import logging
from typing import Any

logger = logging.getLogger(__name__)

try:
    import pigpio  # type: ignore[import-untyped]
except ImportError:  # pragma: no cover
    pigpio = None  # type: ignore[assignment]

_BASE_MAP: dict[str, tuple[int, int, int]] = {
    "STOP": (0, 0, 0),
    "FORWARD": (0, 1, 0),
    "BACKWARD": (0, 0, 1),
    "LOCK": (1, 0, 1),
    "UNLOCK": (1, 1, 0),
    "TURN_LEFT": (1, 0, 0),
    "TURN_RIGHT": (0, 1, 1),
}


class PigpioBaseDriver:
    """3-bit base control using BCM pins OUT1/OUT2/OUT3."""

    def __init__(
        self,
        out1: int = 17,
        out2: int = 27,
        out3: int = 22,
        host: str = "127.0.0.1",
        port: int = 8888,
        pi: Any | None = None,
    ) -> None:
        self._pins = (int(out1), int(out2), int(out3))
        self._owns_pi = pi is None
        self._pi = pi

        if pigpio is None:
            logger.warning("pigpio module not installed - base control disabled")
            return
        if self._pi is None:
            self._pi = pigpio.pi(host, int(port))
        if not self._pi.connected:
            logger.warning("pigpiod not connected at %s:%s - base control disabled", host, port)
            self._pi = None
            return

        for pin in self._pins:
            self._pi.set_mode(pin, pigpio.OUTPUT)
            self._pi.write(pin, 0)
        logger.info("pigpio base: OUT1=BCM%d OUT2=BCM%d OUT3=BCM%d", *self._pins)

    def command(self, cmd: str) -> None:
        cmd_upper = cmd.strip().upper()
        if cmd_upper not in _BASE_MAP:
            logger.warning("Unknown base command: %s", cmd)
            return
        self._write(*_BASE_MAP[cmd_upper])
        logger.info("Base: %s -> %s", cmd_upper, _BASE_MAP[cmd_upper])

    def stop(self) -> None:
        self.command("STOP")

    def close(self) -> None:
        self.stop()
        if self._pi is not None:
            for pin in self._pins:
                self._pi.write(pin, 0)
            if self._owns_pi:
                self._pi.stop()
        logger.info("pigpio base: released")

    def _write(self, out1: int, out2: int, out3: int) -> None:
        if self._pi is None:
            return
        for pin, level in zip(self._pins, (out1, out2, out3), strict=True):
            self._pi.write(pin, 1 if level else 0)
