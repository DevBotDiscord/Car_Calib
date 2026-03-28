"""Drivers module: motor PWM translation layer.

Provides a generic interface for converting PID output values into PWM
signals suitable for differential-drive or servo-based motor hardware on
the NVIDIA Jetson Nano.
"""

import logging

from config.settings import (
    DRIVER_MOTOR_PWM_CENTRE,
    DRIVER_MOTOR_PWM_MAX,
    DRIVER_MOTOR_PWM_MIN,
)

logger = logging.getLogger(__name__)

# PWM range limits
_PWM_MIN: int = DRIVER_MOTOR_PWM_MIN
_PWM_MAX: int = DRIVER_MOTOR_PWM_MAX

# Baseline forward speed (centre of PWM range)
_PWM_CENTRE: int = DRIVER_MOTOR_PWM_CENTRE


class MotorDriver:
    """Translates PID controller output into motor PWM commands.

    The driver maps a signed PID output to differential PWM values:

    * ``pwm_left  = clamp(centre + output)``
    * ``pwm_right = clamp(centre - output)``

    This produces a gentle turn proportional to the heading error while
    maintaining forward motion.

    Args:
        pwm_centre: Baseline PWM value representing straight-ahead motion.
            Defaults to ``128`` (mid-range for 0–255 PWM).
        pwm_min: Minimum allowable PWM value.
        pwm_max: Maximum allowable PWM value.
    """

    def __init__(
        self,
        pwm_centre: int = _PWM_CENTRE,
        pwm_min: int = _PWM_MIN,
        pwm_max: int = _PWM_MAX,
    ) -> None:
        self._centre = pwm_centre
        self._min = pwm_min
        self._max = pwm_max

    @staticmethod
    def _clamp(value: float, low: int, high: int) -> int:
        """Clamp *value* to the range [*low*, *high*].

        Args:
            value: Raw floating-point value.
            low: Minimum integer output.
            high: Maximum integer output.

        Returns:
            Clamped integer PWM value.
        """
        return int(max(low, min(high, round(value))))

    def set_pwm(self, pid_output: float) -> tuple[int, int]:
        """Convert *pid_output* to a (left, right) PWM pair and apply it.

        Args:
            pid_output: Signed PID controller output.  Positive values
                steer right; negative values steer left.

        Returns:
            Tuple of ``(pwm_left, pwm_right)`` integer PWM values.

        Raises:
            OSError: Raised if the underlying hardware interface fails.
        """
        pwm_left = self._clamp(self._centre + pid_output, self._min, self._max)
        pwm_right = self._clamp(self._centre - pid_output, self._min, self._max)

        logger.debug(
            "Motor PWM  left=%d  right=%d  (pid_output=%.4f)",
            pwm_left,
            pwm_right,
            pid_output,
        )

        # Hardware write – subclasses override this for real Jetson GPIO/PWM
        self._write_pwm(pwm_left, pwm_right)
        return pwm_left, pwm_right

    def _write_pwm(self, pwm_left: int, pwm_right: int) -> None:
        """Send PWM values to the hardware interface.

        This is a stub implementation intended to be overridden by
        platform-specific subclasses (e.g., using ``Jetson.GPIO`` or
        ``smbus`` for an H-bridge driver).

        Args:
            pwm_left: PWM duty cycle for the left motor (0–255).
            pwm_right: PWM duty cycle for the right motor (0–255).
        """
        # In a real deployment replace this with Jetson.GPIO or similar:
        # GPIO.output(LEFT_PIN, pwm_left)
        # GPIO.output(RIGHT_PIN, pwm_right)
        pass  # noqa: WPS420

    def stop(self) -> None:
        """Command both motors to stop (PWM = 0).

        Used for emergency stop procedures.
        """
        logger.warning("MotorDriver: EMERGENCY STOP issued.")
        self._write_pwm(0, 0)
