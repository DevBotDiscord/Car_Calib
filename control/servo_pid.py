"""Control module: servo PID controller for the heading-hold system.

Behaviour:
- Hysteresis gating: calibration starts only when |e| exceeds a start
  threshold and stops when |e| goes below a stop threshold.
- LOCKED + active calibration: compute e = theta - 90 deg, apply PID,
  clamp steering offset to +/-state.max_steering_offset deg, and output
  servo_center_angle + steering_offset.
- LOCKED + inactive calibration: steer back toward servo_center_angle and
  avoid PID accumulation.
- Hold logic: if vision returns None, bypass PID and return
  state.last_valid_servo_angle (GAPPING state).
- Integral reset: on transition from GAPPING back to LOCKED and on
  calibration deactivation, the integral term is zeroed.
- Output shaping: deadband and slew limiting smooth small command chatter
  before commands are sent to the servo transport.
"""

from __future__ import annotations

import logging
import time
from typing import Optional

from config.settings import (
    CTRL_HYSTERESIS_HIGH,
    CTRL_HYSTERESIS_LOW,
    CTRL_RELOCK_VALID_FRAMES,
    CTRL_SERVO_OUTPUT_DEADBAND_DEG,
    CTRL_SERVO_OUTPUT_SLEW_RATE_DEG_PER_S,
)
from models.robot_state import FSMState, RobotState

logger = logging.getLogger(__name__)


class ServoPID:
    """PID controller that outputs servo angle commands."""

    def __init__(
        self,
        state: RobotState,
        start_calib_threshold_deg: float = CTRL_HYSTERESIS_HIGH,
        stop_calib_threshold_deg: float = CTRL_HYSTERESIS_LOW,
        servo_output_deadband_deg: float = CTRL_SERVO_OUTPUT_DEADBAND_DEG,
        servo_output_slew_rate_deg_per_s: float = CTRL_SERVO_OUTPUT_SLEW_RATE_DEG_PER_S,
    ) -> None:
        if start_calib_threshold_deg <= 0 or stop_calib_threshold_deg <= 0:
            raise ValueError("Calibration thresholds must be positive.")
        if stop_calib_threshold_deg > start_calib_threshold_deg:
            raise ValueError(
                "stop_calib_threshold_deg must be <= start_calib_threshold_deg."
            )
        if servo_output_deadband_deg < 0:
            raise ValueError("servo_output_deadband_deg must be >= 0.")
        if servo_output_slew_rate_deg_per_s < 0:
            raise ValueError("servo_output_slew_rate_deg_per_s must be >= 0.")

        self._state = state
        self._start_calib_threshold_deg = start_calib_threshold_deg
        self._stop_calib_threshold_deg = stop_calib_threshold_deg
        self._servo_output_deadband_deg = servo_output_deadband_deg
        self._servo_output_slew_rate_deg_per_s = servo_output_slew_rate_deg_per_s
        self._relock_valid_frames_required = max(1, int(CTRL_RELOCK_VALID_FRAMES))
        self._relock_valid_count = 0
        now = time.monotonic()
        self._last_time: float = now
        self._last_output_time: float = now

    def _compute_pid(self, error: float, now: float) -> tuple[float, float, float, float]:
        """Calculate PID output for *error*."""
        dt = now - self._last_time
        if dt <= 0:
            dt = 1e-6
        self._last_time = now

        pid = self._state.pid
        max_offset = self._state.max_steering_offset

        p_term = pid.kp * error

        self._state.pid_integral += error * dt
        if pid.ki != 0.0:
            max_integral = max_offset / abs(pid.ki)
            self._state.pid_integral = max(
                -max_integral,
                min(max_integral, self._state.pid_integral),
            )
        i_term = pid.ki * self._state.pid_integral

        d_term = pid.kd * (error - self._state.pid_last_error) / dt
        self._state.pid_last_error = error

        output = p_term + i_term + d_term
        return p_term, i_term, d_term, output

    def _mark_output_hold(self, now: float) -> float:
        self._last_output_time = now
        return self._state.last_valid_servo_angle

    def _shape_servo_output(self, target_angle: float, now: float) -> float:
        current_angle = self._state.last_valid_servo_angle
        delta = target_angle - current_angle

        if abs(delta) <= self._servo_output_deadband_deg:
            self._last_output_time = now
            return current_angle

        if self._servo_output_slew_rate_deg_per_s > 0.0:
            dt = now - self._last_output_time
            if dt <= 0.0:
                dt = 1e-6
            # Cap dt so a pause does not allow one oversized steering jump.
            dt = min(dt, 0.1)
            max_step = self._servo_output_slew_rate_deg_per_s * dt
            if abs(delta) > max_step:
                target_angle = current_angle + (max_step if delta > 0 else -max_step)

        self._last_output_time = now
        return target_angle

    def update(self, theta: Optional[float]) -> float:
        """Compute and return the servo angle command for this control cycle."""
        state = self._state
        now = time.monotonic()

        if theta is None:
            self._relock_valid_count = 0
            if state.fsm_state == FSMState.LOCKED:
                state.transition_to(FSMState.GAPPING)

            logger.info(
                "state=%s theta=None servo=%.2f deg (holding last valid)",
                state.fsm_state.name,
                state.last_valid_servo_angle,
            )
            return self._mark_output_hold(now)

        if state.fsm_state == FSMState.GAPPING:
            self._relock_valid_count += 1
            if self._relock_valid_count < self._relock_valid_frames_required:
                logger.info(
                    "state=%s theta=%.2f deg relock=%d/%d servo=%.2f deg (debouncing)",
                    state.fsm_state.name,
                    theta,
                    self._relock_valid_count,
                    self._relock_valid_frames_required,
                    state.last_valid_servo_angle,
                )
                return self._mark_output_hold(now)

            self._relock_valid_count = 0
            state.reset_pid_integral()
            state.pid_last_error = 0.0
            state.calibration_active = False
        else:
            self._relock_valid_count = 0

        state.transition_to(FSMState.LOCKED)

        error = theta - 90.0
        abs_error = abs(error)

        if (not state.calibration_active) and (abs_error >= self._start_calib_threshold_deg):
            state.calibration_active = True
            logger.info(
                "Calibration stage activated (theta=%.2f deg, error=%.2f deg, start=+-%.2f deg)",
                theta,
                error,
                self._start_calib_threshold_deg,
            )

        if state.calibration_active and abs_error <= self._stop_calib_threshold_deg:
            state.calibration_active = False
            state.reset_pid_integral()
            state.pid_last_error = 0.0
            logger.info(
                "Calibration stage cleared (theta=%.2f deg, error=%.2f deg, stop=+-%.2f deg)",
                theta,
                error,
                self._stop_calib_threshold_deg,
            )

        if not state.calibration_active:
            target_angle = state.servo_center_angle
            servo_angle = self._shape_servo_output(target_angle, now)
            state.last_valid_servo_angle = servo_angle
            logger.info(
                "state=%s theta=%.2f deg error=%.2f deg calibration=inactive "
                "(start=+-%.2f deg, stop=+-%.2f deg) target=%.2f deg servo=%.2f deg",
                state.fsm_state.name,
                theta,
                error,
                self._start_calib_threshold_deg,
                self._stop_calib_threshold_deg,
                target_angle,
                servo_angle,
            )
            return servo_angle

        p_term, i_term, d_term, raw_offset = self._compute_pid(error, now)

        max_offset = state.max_steering_offset
        steering_offset = max(-max_offset, min(max_offset, raw_offset))
        target_angle = state.servo_center_angle + steering_offset
        servo_angle = self._shape_servo_output(target_angle, now)
        state.last_valid_servo_angle = servo_angle

        logger.info(
            "state=%s theta=%.2f deg error=%.2f deg "
            "P=%.4f I=%.4f D=%.4f offset=%.2f deg target=%.2f deg servo=%.2f deg",
            state.fsm_state.name,
            theta,
            error,
            p_term,
            i_term,
            d_term,
            steering_offset,
            target_angle,
            servo_angle,
        )
        return servo_angle
