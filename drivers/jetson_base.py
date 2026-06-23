"""Jetson Nano direct base (motor) control via 3-bit GPIO.

Mimics ESP8266 `baseCommand()` firmware: FORWARD / BACKWARD / STOP
using three GPIO output bits (bit2 / bit1 / bit0).

Default pins (BOARD numbering):
  BIT2  → pin 15  (BCM 22)  – high bit
  BIT1  → pin 13  (BCM 27)  – middle bit
  BIT0  → pin 11  (BCM 17)  – low bit

Commands:
  FORWARD    → 001 (BIT2=0, BIT1=0, BIT0=1)
  BACKWARD   → 010 (BIT2=0, BIT1=1, BIT0=0)
  STOP       → 000 (all 0)
  LOCK       → 011 (maintain lock)
  UNLOCK     → 111 (release)
"""

from __future__ import annotations

import logging
from typing import Any

logger = logging.getLogger(__name__)

try:
    import Jetson.GPIO as GPIO  # type: ignore[import-untyped]
    _GPIO_AVAILABLE = True
except ImportError:  # pragma: no cover
    GPIO = None  # type: ignore[assignment]
    _GPIO_AVAILABLE = False

# Default pin mapping (BOARD numbering)
DEFAULT_PINS = {
    "BIT2": 15,  # GPIO 22
    "BIT1": 13,  # GPIO 27
    "BIT0": 11,  # GPIO 17
}

# 3-bit command table
_BASE_MAP: dict[str, tuple[int, int, int]] = {
    "STOP":      (0, 0, 0),
    "FORWARD":   (0, 0, 1),
    "BACKWARD":  (0, 1, 0),
    "LOCK":      (0, 1, 1),
    "UNLOCK":    (1, 1, 1),
    "TURN_LEFT": (1, 0, 0),
    "TURN_RIGHT":(1, 0, 1),
}


class JetsonBaseDriver:
    """3-bit GPIO base control for Jetson Nano."""

    def __init__(
        self,
        pin_bit2: int = DEFAULT_PINS["BIT2"],
        pin_bit1: int = DEFAULT_PINS["BIT1"],
        pin_bit0: int = DEFAULT_PINS["BIT0"],
    ) -> None:
        self._pins = {"BIT2": pin_bit2, "BIT1": pin_bit1, "BIT0": pin_bit0}
        self._initialized = False

        if not _GPIO_AVAILABLE:
            logger.warning("Jetson.GPIO not installed — base control disabled")
            return

        GPIO.setmode(GPIO.BOARD)
        for name, pin in self._pins.items():
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        self._initialized = True
        self.command("STOP")
        logger.info(
            "Jetson base: BIT2=pin%d BIT1=pin%d BIT0=pin%d",
            pin_bit2, pin_bit1, pin_bit0,
        )

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #

    def command(self, cmd: str) -> None:
        """Send base command."""
        cmd_upper = cmd.strip().upper()
        if cmd_upper not in _BASE_MAP:
            logger.warning("Unknown base command: %s", cmd)
            return
        b2, b1, b0 = _BASE_MAP[cmd_upper]
        self._write(b2, b1, b0)
        logger.debug("Base: %s → BIT2=%d BIT1=%d BIT0=%d", cmd_upper, b2, b1, b0)

    def stop(self) -> None:
        self.command("STOP")

    def close(self) -> None:
        self.stop()
        if self._initialized:
            for pin in self._pins.values():
                GPIO.output(pin, GPIO.LOW)
        logger.info("Jetson base: released")

    # ------------------------------------------------------------------ #
    # Internal
    # ------------------------------------------------------------------ #

    def _write(self, bit2: int, bit1: int, bit0: int) -> None:
        if not self._initialized:
            return
        GPIO.output(self._pins["BIT2"], GPIO.HIGH if bit2 else GPIO.LOW)
        GPIO.output(self._pins["BIT1"], GPIO.HIGH if bit1 else GPIO.LOW)
        GPIO.output(self._pins["BIT0"], GPIO.HIGH if bit0 else GPIO.LOW)
