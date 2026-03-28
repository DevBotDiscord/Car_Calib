"""Model module: robot state for the heading-hold system.

Manages:
- PID constants (Kp, Ki, Kd)
- Servo config: ``servo_center_angle`` (neutral position, default 90°) and
  ``max_steering_offset`` (maximum steering deviation, default 30°)
- ``last_valid_servo_angle`` (fallback during GAPPING) and
  ``last_valid_command`` (motor-command fallback for heading controller)
- ROI parameters (relative): ``roi_height_pct``, ``roi_top_width_pct``,
  ``roi_bottom_width_pct``
- ``debug_mode`` flag and FSM states: SEARCHING, LOCKED, GAPPING
"""

import logging
from dataclasses import dataclass, field
from enum import Enum, auto

from config.settings import (
    MAX_STEERING_OFFSET,
    PID_KD,
    PID_KI,
    PID_KP,
    ROBOT_DEBUG_MODE,
    ROI_BOTTOM_WIDTH_PCT,
    ROI_HEIGHT_PCT,
    ROI_TOP_WIDTH_PCT,
    SERVO_CENTER_ANGLE,
)

logger = logging.getLogger(__name__)


class FSMState(Enum):
    """Finite State Machine states for the heading-hold system."""

    SEARCHING = auto()
    """No valid tile-gap line detected; robot is looking for a reference."""

    LOCKED = auto()
    """Vision is active; robot is tracking a detected tile-gap line."""

    GAPPING = auto()
    """Vision lost; robot coasts on the last known servo angle (~2 s gap)."""


@dataclass
class PIDConstants:
    """Container for PID gain constants.

    Attributes:
        kp: Proportional gain.
        ki: Integral gain.
        kd: Derivative gain.
    """

    kp: float = PID_KP
    ki: float = PID_KI
    kd: float = PID_KD


@dataclass
class RobotState:
    """Mutable state shared across MVC layers.

    Attributes:
        pid: PID gain constants.
        servo_center_angle: Neutral servo angle in degrees (default 90°).
        max_steering_offset: Maximum steering deviation from centre in
            degrees (default 30°).
        last_valid_servo_angle: Most recent servo angle issued while LOCKED.
            Used as a fallback during GAPPING.
        last_valid_command: Most recent motor-command output.  Used as a
            fallback by the heading controller during GAPPING.
        roi_height_pct: Fraction of frame height covered by the ROI
            trapezoid (bottom portion, default 0.4).
        roi_top_width_pct: Width of the trapezoid top edge as a fraction
            of frame width (default 0.6).
        roi_bottom_width_pct: Width of the trapezoid bottom edge as a
            fraction of frame width (default 1.0).
        debug_mode: When ``True``, save ``debug_mask.jpg`` once on
            first :meth:`~vision.detector.LineDetector.get_reference_angle`
            call to verify the trapezoid ROI.
        fsm_state: Current FSM state.
        calibration_active: Whether PID is actively calibrating back to
            center heading (90°).
        pid_integral: Accumulated integral term for the PID controller.
        pid_last_error: Previous error for the derivative calculation.
    """

    pid: PIDConstants = field(default_factory=PIDConstants)
    servo_center_angle: float = SERVO_CENTER_ANGLE
    max_steering_offset: float = MAX_STEERING_OFFSET
    last_valid_servo_angle: float = SERVO_CENTER_ANGLE
    last_valid_command: float = 0.0
    roi_height_pct: float = ROI_HEIGHT_PCT
    roi_top_width_pct: float = ROI_TOP_WIDTH_PCT
    roi_bottom_width_pct: float = ROI_BOTTOM_WIDTH_PCT
    debug_mode: bool = ROBOT_DEBUG_MODE
    fsm_state: FSMState = FSMState.SEARCHING
    calibration_active: bool = False
    pid_integral: float = 0.0
    pid_last_error: float = 0.0

    def transition_to(self, new_state: FSMState) -> None:
        """Transition the FSM to *new_state* and log the event.

        Args:
            new_state: The FSMState to transition to.
        """
        if new_state != self.fsm_state:
            logger.info(
                "FSM transition: %s -> %s",
                self.fsm_state.name,
                new_state.name,
            )
            self.fsm_state = new_state

    def reset_pid_integral(self) -> None:
        """Reset the PID integral term to zero (prevents windup on re-entry)."""
        logger.debug("PID integral reset to 0 (was %.4f)", self.pid_integral)
        self.pid_integral = 0.0
