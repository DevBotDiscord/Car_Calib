"""Direct servo control through pigpio on Raspberry Pi.

Uses BCM numbering. The angle is written as-is from the calibrator/controller;
only configured clamp + pulse conversion happens here.
"""

from __future__ import annotations

import logging
from typing import Any

from config.settings import (
    DRIVER_SERVO_ANGLE_MAX,
    DRIVER_SERVO_ANGLE_MIN,
    DRIVER_SERVO_PULSE_MAX_US,
    DRIVER_SERVO_PULSE_MIN_US,
    SERVO_CENTER_ANGLE,
)

logger = logging.getLogger(__name__)

try:
    import pigpio  # type: ignore[import-untyped]
    _PIGPIO_AVAILABLE = True
except ImportError:  # pragma: no cover
    pigpio = None  # type: ignore[assignment]
    _PIGPIO_AVAILABLE = False


class PigpioServoDriver:
    """50Hz servo pulse writer via pigpiod."""

    def __init__(
        self,
        pin: int = 12,
        center_angle: float = 90.0 + SERVO_CENTER_ANGLE,
        pulse_min_us: int = DRIVER_SERVO_PULSE_MIN_US,
        pulse_max_us: int = DRIVER_SERVO_PULSE_MAX_US,
        host: str = "127.0.0.1",
        port: int = 8888,
        pi: Any | None = None,
    ) -> None:
        self._pin = int(pin)
        self._center_angle = float(center_angle)
        self._pulse_min_us = int(pulse_min_us)
        self._pulse_max_us = int(pulse_max_us)
        self._owns_pi = pi is None
        self._pi = pi

        if not _PIGPIO_AVAILABLE:
            logger.warning("pigpio module not installed - servo PWM disabled")
            return
        if self._pi is None:
            self._pi = pigpio.pi(host, int(port))
        if not self._pi.connected:
            logger.warning("pigpiod not connected at %s:%s - servo PWM disabled", host, port)
            self._pi = None
            return

        self.center()
        logger.info(
            "pigpio servo: BCM%d center=%.2f deg pulse=[%d..%d] us",
            self._pin,
            self._center_angle,
            self._pulse_min_us,
            self._pulse_max_us,
        )

    def send_angle(self, angle: float) -> None:
        clamped = max(DRIVER_SERVO_ANGLE_MIN, min(DRIVER_SERVO_ANGLE_MAX, float(angle)))
        pulse_us = self._angle_to_pulse_us(clamped)
        if self._pi is not None:
            rc = self._pi.set_servo_pulsewidth(self._pin, pulse_us)
            if rc != 0:
                logger.error("PigpioServo: set_servo_pulsewidth BCM%d failed rc=%s", self._pin, rc)
        logger.info("PigpioServo: %.2f deg -> %dus", clamped, pulse_us)
        return clamped

    def center(self) -> None:
        logger.info("pigpio servo: centering to %.2f deg", self._center_angle)
        self.send_angle(self._center_angle)

    def close(self) -> None:
        if self._pi is not None:
            self._pi.set_servo_pulsewidth(self._pin, 0)
            if self._owns_pi:
                self._pi.stop()
        logger.info("pigpio servo: released")

    def _angle_to_pulse_us(self, angle: float) -> int:
        span = DRIVER_SERVO_ANGLE_MAX - DRIVER_SERVO_ANGLE_MIN
        if span == 0:
            return self._pulse_min_us
        ratio = (angle - DRIVER_SERVO_ANGLE_MIN) / span
        ratio = max(0.0, min(1.0, ratio))
        return int(round(self._pulse_min_us + ratio * (self._pulse_max_us - self._pulse_min_us)))
