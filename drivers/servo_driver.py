"""Drivers module: servo angle translation layer.

Provides an abstracted interface for sending angle commands to a servo
motor connected via a PCA9685 I²C PWM board or directly to Jetson Nano
PWM pins.

Usage example (PCA9685 with adafruit-circuitpython-pca9685)::

    from drivers.servo_driver import ServoDriver

    driver = ServoDriver(channel=0)
    driver.send_angle(90.0)   # centre
    driver.send_angle(120.0)  # 30° right
    driver.center()           # returns to neutral
"""

import logging

logger = logging.getLogger(__name__)

# Default servo PWM pulse widths in microseconds (standard hobby servo)
_PULSE_MIN_US: int = 1000   # 0°
_PULSE_MAX_US: int = 2000   # 180°

# Servo travel limits (degrees)
_ANGLE_MIN: float = 0.0
_ANGLE_MAX: float = 180.0


class ServoDriver:
    """Translates angle commands into PWM signals for a servo motor.

    The driver maps a degree value to a PWM pulse width using linear
    interpolation between *pulse_min_us* and *pulse_max_us*.

    Args:
        channel: PWM channel index on the PCA9685 (0–15) or Jetson Nano
            PWM pin number.
        center_angle: Neutral (straight-ahead) servo angle in degrees.
            Defaults to ``90.0``.
        pulse_min_us: Pulse width in µs corresponding to 0°.
            Defaults to ``1000``.
        pulse_max_us: Pulse width in µs corresponding to 180°.
            Defaults to ``2000``.
    """

    def __init__(
        self,
        channel: int = 0,
        center_angle: float = 90.0,
        pulse_min_us: int = _PULSE_MIN_US,
        pulse_max_us: int = _PULSE_MAX_US,
    ) -> None:
        self._channel = channel
        self._center_angle = center_angle
        self._pulse_min_us = pulse_min_us
        self._pulse_max_us = pulse_max_us

    def _angle_to_pulse(self, angle: float) -> int:
        """Convert *angle* to a PWM pulse width in microseconds.

        Args:
            angle: Servo angle in degrees.

        Returns:
            Pulse width in µs, clamped to [*pulse_min_us*, *pulse_max_us*].
        """
        clamped = max(_ANGLE_MIN, min(_ANGLE_MAX, angle))
        pulse = (
            self._pulse_min_us
            + (clamped / _ANGLE_MAX) * (self._pulse_max_us - self._pulse_min_us)
        )
        return int(round(pulse))

    def send_angle(self, angle: float) -> None:
        """Send *angle* to the servo hardware.

        Args:
            angle: Target servo angle in degrees.
        """
        pulse_us = self._angle_to_pulse(angle)
        logger.debug(
            "ServoDriver: channel=%d  angle=%.2f°  pulse=%d µs",
            self._channel,
            angle,
            pulse_us,
        )
        self._write_angle(angle, pulse_us)

    def center(self) -> None:
        """Return the servo to the neutral center position.

        Used for safe-state initialisation and emergency-stop procedures.
        """
        logger.info(
            "ServoDriver: centering servo (channel=%d, angle=%.2f°).",
            self._channel,
            self._center_angle,
        )
        self.send_angle(self._center_angle)

    def _write_angle(self, angle: float, pulse_us: int) -> None:
        """Send the PWM pulse to the hardware interface.

        This is a stub implementation intended to be overridden by a
        platform-specific subclass.  Replace the body with your hardware
        library calls, for example:

        **PCA9685 (adafruit-circuitpython-pca9685)**::

            import board, busio
            from adafruit_pca9685 import PCA9685
            i2c = busio.I2C(board.SCL, board.SDA)
            pca = PCA9685(i2c)
            pca.frequency = 50  # 50 Hz for standard servos
            # duty_cycle is 16-bit (0–65535); period = 20 000 µs at 50 Hz
            duty = int(pulse_us / 20_000 * 65_535)
            pca.channels[self._channel].duty_cycle = duty

        **Jetson Nano GPIO PWM**::

            import Jetson.GPIO as GPIO
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self._channel, GPIO.OUT)
            pwm = GPIO.PWM(self._channel, 50)
            pwm.start(pulse_us / 20_000 * 100)  # duty in percent

        Args:
            angle: Servo angle in degrees (informational).
            pulse_us: Computed pulse width in microseconds.
        """
        # Stub – replace with real hardware calls in a platform subclass.
        logger.debug(
            "ServoDriver._write_angle stub called (channel=%d, angle=%.2f°, "
            "pulse=%d µs). Override _write_angle for real hardware.",
            self._channel,
            angle,
            pulse_us,
        )
