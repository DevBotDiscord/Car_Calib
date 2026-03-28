"""Control module: PID heading controller with hysteresis and sparse-signal support.

Behaviour:
- **Hysteresis filter**: correction activates when ``e > 5°`` and stops
  once ``e < 3°``.
- **Sparse signal**: if vision returns ``None``, the last valid command
  is re-applied instead of resetting the PID.
- **Integral wind-up guard**: when the FSM re-enters LOCKED after a
  GAPPING gap, the integral term is zeroed.
"""

import logging
import time
from typing import Optional

from models.robot_state import FSMState, RobotState
from settings import CTRL_HYSTERESIS_HIGH, CTRL_HYSTERESIS_LOW

logger = logging.getLogger(__name__)

# Hysteresis thresholds (degrees)
_HYSTERESIS_HIGH = CTRL_HYSTERESIS_HIGH
_HYSTERESIS_LOW = CTRL_HYSTERESIS_LOW


class HeadingController:
    """PID controller for robot heading stabilisation.

    Args:
        state: Shared :class:`~models.robot_state.RobotState` instance.
    """

    def __init__(self, state: RobotState) -> None:
        self._state = state
        self._correcting: bool = False
        self._last_time: float = time.monotonic()

    def _compute_pid(self, error: float) -> float:
        """Calculate PID output for the given *error*.

        Args:
            error: Current heading error in degrees.

        Returns:
            PID output value (unbounded float).
        """
        now = time.monotonic()
        dt = now - self._last_time
        if dt <= 0:
            dt = 1e-6  # guard against zero division
        self._last_time = now

        pid = self._state.pid

        # Proportional
        p_term = pid.kp * error

        # Integral (accumulated in state for persistence across calls)
        self._state.pid_integral += error * dt
        i_term = pid.ki * self._state.pid_integral

        # Derivative
        d_term = pid.kd * (error - self._state.pid_last_error) / dt
        self._state.pid_last_error = error

        output = p_term + i_term + d_term
        logger.debug(
            "PID  P=%.4f  I=%.4f  D=%.4f  output=%.4f",
            p_term,
            i_term,
            d_term,
            output,
        )
        return output

    def _should_correct(self, error: float) -> bool:
        """Apply hysteresis logic to decide whether to issue a correction.

        Args:
            error: Current heading error in degrees.

        Returns:
            ``True`` if a correction command should be generated.
        """
        if not self._correcting and error > _HYSTERESIS_HIGH:
            self._correcting = True
            logger.debug(
                "Hysteresis: correction ACTIVATED (e=%.2f° > %.1f°)",
                error,
                _HYSTERESIS_HIGH,
            )
        elif self._correcting and error < _HYSTERESIS_LOW:
            self._correcting = False
            logger.debug(
                "Hysteresis: correction DEACTIVATED (e=%.2f° < %.1f°)",
                error,
                _HYSTERESIS_LOW,
            )
        return self._correcting

    def update(self, heading_error: Optional[float]) -> float:
        """Compute and return the motor command for this control cycle.

        When *heading_error* is ``None`` (vision lost), the method enters
        GAPPING and re-applies ``state.last_valid_command``.

        When *heading_error* is a valid value and the previous state was
        GAPPING, the integral term is reset before switching back to LOCKED.

        Args:
            heading_error: Heading error from the vision module, or
                ``None`` if no lines were detected.

        Returns:
            Motor command value derived from the PID output.
        """
        state = self._state

        if heading_error is None:
            # --- Sparse signal path ---
            if state.fsm_state != FSMState.GAPPING:
                state.transition_to(FSMState.GAPPING)

            logger.info(
                "Vision Lost: Applying Last Known Correction (%.4f)",
                state.last_valid_command,
            )
            return state.last_valid_command

        # --- Valid signal path ---
        # On re-entry from GAPPING, reset integral to avoid windup
        if state.fsm_state == FSMState.GAPPING:
            state.reset_pid_integral()

        if state.fsm_state != FSMState.LOCKED:
            state.transition_to(FSMState.LOCKED)

        if not self._should_correct(heading_error):
            # Within hysteresis dead-band – no correction needed
            return state.last_valid_command

        command = self._compute_pid(heading_error)
        state.last_valid_command = command
        return command
