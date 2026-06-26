"""Direct base motor GPIO control through pigpio on Raspberry Pi."""

from __future__ import annotations

import logging
from typing import Any

logger = logging.getLogger(__name__)

try:
    import pigpio  # type: ignore[import-untyped]
    _PIGPIO_AVAILABLE = True
except ImportError:  # pragma: no cover
    pigpio = None  # type: ignore[assignment]
    _PIGPIO_AVAILABLE = False

DEFAULT_PINS = {
    "OUT1": 17,
    "OUT2": 27,
    "OUT3": 22,
}

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
        out1: int = DEFAULT_PINS["OUT1"],
        out2: int = DEFAULT_PINS["OUT2"],
        out3: int = DEFAULT_PINS["OUT3"],
        host: str = "127.0.0.1",
        port: int = 8888,
        pi: Any | None = None,
    ) -> None:
        self._pins = {"OUT1": int(out1), "OUT2": int(out2), "OUT3": int(out3)}
        self._owns_pi = pi is None
        self._pi = pi

        if not _PIGPIO_AVAILABLE:
            logger.warning("pigpio module not installed - base control disabled")
            return
        if self._pi is None:
            self._pi = pigpio.pi(host, int(port))
        if not self._pi.connected:
            logger.warning("pigpiod not connected at %s:%s - base control disabled", host, port)
            self._pi = None
            return

        for pin in self._pins.values():
            self._pi.set_mode(pin, pigpio.OUTPUT)
            self._pi.write(pin, 0)
        self.command("STOP")
        logger.info("pigpio base: OUT1=BCM%d OUT2=BCM%d OUT3=BCM%d", out1, out2, out3)

    def command(self, cmd: str) -> None:
        cmd_upper = cmd.strip().upper()
        if cmd_upper not in _BASE_MAP:
            logger.warning("Unknown base command: %s", cmd)
            return
        out1, out2, out3 = _BASE_MAP[cmd_upper]
        self._write(out1, out2, out3)
        logger.info("Base: %s -> OUT1=%d OUT2=%d OUT3=%d", cmd_upper, out1, out2, out3)

    def stop(self) -> None:
        self.command("STOP")

    def close(self) -> None:
        self.stop()
        if self._pi is not None:
            for pin in self._pins.values():
                self._pi.write(pin, 0)
            if self._owns_pi:
                self._pi.stop()
        logger.info("pigpio base: released")

    def _write(self, out1: int, out2: int, out3: int) -> None:
        if self._pi is None:
            return
        self._pi.write(self._pins["OUT1"], 1 if out1 else 0)
        self._pi.write(self._pins["OUT2"], 1 if out2 else 0)
        self._pi.write(self._pins["OUT3"], 1 if out3 else 0)
