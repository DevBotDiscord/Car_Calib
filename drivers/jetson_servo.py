"""Direct PWM servo driver for NVIDIA Jetson Nano (Jetson.GPIO).

No MQTT, no network dependency — writes hardware PWM directly.

Requires: Jetson.GPIO (pip install Jetson.GPIO)
Pin: GPIO 33 (PWM-capable, J41 header pin 33 on Jetson Nano)
PWM freq: 50 Hz (standard servo)
"""

from __future__ import annotations

import logging
import time
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
    import Jetson.GPIO as GPIO  # type: ignore[import-untyped]
    _GPIO_AVAILABLE = True
except ImportError:  # pragma: no cover
    GPIO = None  # type: ignore[assignment]
    _GPIO_AVAILABLE = False


class JetsonServoDriver:
    """Direct PWM servo control for Jetson Nano.

    Maps [-90, +90] deg → [500, 2500] µs pulse on 50Hz PWM.
    """

    def __init__(
        self,
        pin: int = 33,
        center_angle: float = SERVO_CENTER_ANGLE,
        pulse_min_us: int = DRIVER_SERVO_PULSE_MIN_US,
        pulse_max_us: int = DRIVER_SERVO_PULSE_MAX_US,
    ) -> None:
        self._pin = int(pin)
        self._center_angle = float(center_angle)
        self._pulse_min_us = int(pulse_min_us)
        self._pulse_max_us = int(pulse_max_us)
        self._pwm = None

        if not _GPIO_AVAILABLE:
            logger.warning(
                "Jetson.GPIO not installed — servo PWM disabled. "
                "Install via: pip install Jetson.GPIO"
            )
            return

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self._pin, GPIO.OUT, initial=GPIO.LOW)
        self._pwm = GPIO.PWM(self._pin, 50)  # 50 Hz
        self._pwm.start(self._angle_to_duty(self._center_angle))
        logger.info(
            "Jetson servo: pin=%d center=%.2f deg pulse=[%d..%d] µs",
            self._pin,
            self._center_angle,
            self._pulse_min_us,
            self._pulse_max_us,
        )

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #

    def send_angle(self, angle: float) -> None:
        """Write *angle* directly to servo PWM."""
        clamped = max(DRIVER_SERVO_ANGLE_MIN, min(DRIVER_SERVO_ANGLE_MAX, angle))
        duty = self._angle_to_duty(clamped)
        if self._pwm is not None:
            self._pwm.ChangeDutyCycle(duty)
        logger.debug("JetsonServo: %.2f deg → duty=%.2f%%", clamped, duty)

    def center(self) -> None:
        self.send_angle(self._center_angle)

    def close(self) -> None:
        if self._pwm is not None:
            self._pwm.stop()
        if _GPIO_AVAILABLE:
            GPIO.cleanup(self._pin)
        logger.info("Jetson servo: released")

    # ------------------------------------------------------------------ #
    # Internal
    # ------------------------------------------------------------------ #

    def _angle_to_duty(self, angle: float) -> float:
        """Map angle → PWM duty cycle (0–100%)."""
        clamped = max(DRIVER_SERVO_ANGLE_MIN, min(DRIVER_SERVO_ANGLE_MAX, angle))
        if DRIVER_SERVO_ANGLE_MAX == DRIVER_SERVO_ANGLE_MIN:
            return self._pulse_min_us / 20000.0 * 100.0
        ratio = (clamped - DRIVER_SERVO_ANGLE_MIN) / (DRIVER_SERVO_ANGLE_MAX - DRIVER_SERVO_ANGLE_MIN)
        pulse_us = self._pulse_min_us + ratio * (self._pulse_max_us - self._pulse_min_us)
        return pulse_us / 20000.0 * 100.0  # 20ms period at 50Hz
