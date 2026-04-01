"""Control module: servo PID controller for the heading-hold system.

Behaviour:
- **Hysteresis gating**: calibration starts only when ``|e|`` exceeds a
    start threshold and stops when ``|e|`` goes below a stop threshold.
- **LOCKED + active calibration**: Compute ``e = θ - 90°``, apply PID,
    clamp steering offset to ±``state.max_steering_offset``°, and output
    ``servo_center_angle + steering_offset``.
- **LOCKED + inactive calibration**: Hold servo at ``servo_center_angle``
    and avoid PID accumulation.
- **Hold logic**: If vision returns ``None``, bypass PID and return
    ``last_valid_servo_angle`` (GAPPING state).
- **Integral reset**: On transition from GAPPING back to LOCKED and on
    calibration deactivation, the integral term is zeroed.
"""

import logging
import time
from typing import Optional

from models.robot_state import FSMState, RobotState
from config.settings import (
    CTRL_HYSTERESIS_HIGH,
    CTRL_HYSTERESIS_LOW,
    CTRL_RELOCK_VALID_FRAMES,
)

logger = logging.getLogger(__name__)


class ServoPID:
    """PID controller that outputs servo angle commands.

    Args:
        state: Shared :class:`~models.robot_state.RobotState` instance.
    """

    def __init__(
        self,
        state: RobotState,
        start_calib_threshold_deg: float = CTRL_HYSTERESIS_HIGH,
        stop_calib_threshold_deg: float = CTRL_HYSTERESIS_LOW,
    ) -> None:
        if start_calib_threshold_deg <= 0 or stop_calib_threshold_deg <= 0:
            raise ValueError("Calibration thresholds must be positive.")
        if stop_calib_threshold_deg > start_calib_threshold_deg:
            raise ValueError(
                "stop_calib_threshold_deg must be <= start_calib_threshold_deg."
            )

        self._state = state
        self._start_calib_threshold_deg = start_calib_threshold_deg
        self._stop_calib_threshold_deg = stop_calib_threshold_deg
        self._relock_valid_frames_required = max(1, int(CTRL_RELOCK_VALID_FRAMES))
        self._relock_valid_count = 0
        self._last_time: float = time.monotonic()

    # ---------------------------------------------------------------------- #
    # Private helpers
    # ---------------------------------------------------------------------- #

    def _compute_pid(self, error: float) -> tuple[float, float, float, float]:
        """Calculate PID output for *error*.

        Args:
            error: ``θ - 90°`` in degrees.

        Returns:
            Tuple of ``(p_term, i_term, d_term, total_output)``.
        """
        now = time.monotonic()
        dt = now - self._last_time
        if dt <= 0:
            dt = 1e-6
        self._last_time = now

        pid = self._state.pid
        max_offset = self._state.max_steering_offset

        p_term = pid.kp * error

        self._state.pid_integral += error * dt
        # Anti-windup: clamp the integral accumulator so the i_term
        # contribution cannot exceed the steering clamp range.
        if pid.ki != 0.0:
            max_integral = max_offset / abs(pid.ki)
            self._state.pid_integral = max(
                -max_integral, min(max_integral, self._state.pid_integral)
            )
        i_term = pid.ki * self._state.pid_integral

        d_term = pid.kd * (error - self._state.pid_last_error) / dt
        self._state.pid_last_error = error

        output = p_term + i_term + d_term
        return p_term, i_term, d_term, output

    # ---------------------------------------------------------------------- #
    # Public API
    # ---------------------------------------------------------------------- #

    def update(self, theta: Optional[float]) -> float:
        """Compute and return the servo angle command for this control cycle.

        When *theta* is ``None`` (vision lost), the controller enters
        GAPPING state and returns ``state.last_valid_servo_angle``.

        When *theta* is valid and the previous state was GAPPING, the PID
        integral is reset before transitioning to LOCKED.

        Args:
            theta: Tile-gap angle from the vision module (degrees, relative
                to x-axis), or ``None`` if no line was detected.

        Returns:
            Servo angle command in degrees.
        """
        state = self._state

        if theta is None:
            # ---------------------------------------------------------------- #
            # Hold logic – vision lost
            # ---------------------------------------------------------------- #
            self._relock_valid_count = 0
            if state.fsm_state == FSMState.LOCKED:
                state.transition_to(FSMState.GAPPING)

            logger.info(
                "state=%s  theta=None  servo=%.2f°  (holding last valid)",
                state.fsm_state.name,
                state.last_valid_servo_angle,
            )
            return state.last_valid_servo_angle

        # -------------------------------------------------------------------- #
        # Valid signal
        # -------------------------------------------------------------------- #
        if state.fsm_state == FSMState.GAPPING:
            self._relock_valid_count += 1
            if self._relock_valid_count < self._relock_valid_frames_required:
                logger.info(
                    "state=%s  theta=%.2f°  relock=%d/%d  servo=%.2f°  (debouncing)",
                    state.fsm_state.name,
                    theta,
                    self._relock_valid_count,
                    self._relock_valid_frames_required,
                    state.last_valid_servo_angle,
                )
                return state.last_valid_servo_angle

            self._relock_valid_count = 0
            # Re-entry after blind gap – reset integral to prevent windup
            state.reset_pid_integral()
            state.pid_last_error = 0.0
            state.calibration_active = False
        else:
            self._relock_valid_count = 0

        state.transition_to(FSMState.LOCKED)

        error = theta - 90.0
        abs_error = abs(error)

        # Hysteresis gating for calibration stage.
        if (not state.calibration_active) and (abs_error >= self._start_calib_threshold_deg):
            state.calibration_active = True
            logger.info(
                "Calibration stage activated (theta=%.2f°, error=%.2f°, start=±%.2f°)",
                theta,
                error,
                self._start_calib_threshold_deg,
            )

        if state.calibration_active and abs_error <= self._stop_calib_threshold_deg:
            state.calibration_active = False
            state.reset_pid_integral()
            state.pid_last_error = 0.0
            logger.info(
                "Calibration stage cleared (theta=%.2f°, error=%.2f°, stop=±%.2f°)",
                theta,
                error,
                self._stop_calib_threshold_deg,
            )

        # If calibration is inactive, keep servo centered and avoid PID updates.
        if not state.calibration_active:
            servo_angle = state.servo_center_angle
            state.last_valid_servo_angle = servo_angle
            logger.info(
                "state=%s  theta=%.2f°  error=%.2f°  calibration=inactive "
                "(start=±%.2f°, stop=±%.2f°)  servo=%.2f°",
                state.fsm_state.name,
                theta,
                error,
                self._start_calib_threshold_deg,
                self._stop_calib_threshold_deg,
                servo_angle,
            )
            return servo_angle

        p_term, i_term, d_term, raw_offset = self._compute_pid(error)

        # Clamp steering offset to ±max_steering_offset
        max_offset = state.max_steering_offset
        steering_offset = max(-max_offset, min(max_offset, raw_offset))
        servo_angle = state.servo_center_angle + steering_offset
        state.last_valid_servo_angle = servo_angle

        logger.info(
            "state=%s  theta=%.2f°  error=%.2f°  "
            "P=%.4f  I=%.4f  D=%.4f  offset=%.2f°  servo=%.2f°",
            state.fsm_state.name,
            theta,
            error,
            p_term,
            i_term,
            d_term,
            steering_offset,
            servo_angle,
        )
        return servo_angle
