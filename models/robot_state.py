"""Robot state model contracts."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

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


class FSMState(str, Enum):
    """Calibration/steering states emitted by SteeringController.compute_steering."""

    GAPPING = "GAPPING"
    DANGER_LEFT = "DANGER_LEFT"
    DANGER_RIGHT = "DANGER_RIGHT"
    AMBIGUOUS_DANGER = "AMBIGUOUS_DANGER"
    TRACKING_COAST = "TRACKING_COAST"
    TRACKING_PD = "TRACKING_PD"


@dataclass
class PIDConstants:
    """Container for PID gains."""

    kp: float = PID_KP
    ki: float = PID_KI
    kd: float = PID_KD


@dataclass
class RobotState:
    """Runtime robot configuration and mutable control state."""

    pid: PIDConstants = field(default_factory=PIDConstants)
    servo_center_angle: float = SERVO_CENTER_ANGLE
    max_steering_offset: float = MAX_STEERING_OFFSET
    last_valid_servo_angle: float = SERVO_CENTER_ANGLE
    last_valid_command: float = SERVO_CENTER_ANGLE
    roi_height_pct: float = ROI_HEIGHT_PCT
    roi_top_width_pct: float = ROI_TOP_WIDTH_PCT
    roi_bottom_width_pct: float = ROI_BOTTOM_WIDTH_PCT
    debug_mode: bool = ROBOT_DEBUG_MODE
    fsm_state: FSMState = FSMState.GAPPING
    calibration_active: bool = False
    pid_integral: float = 0.0
    pid_last_error: float = 0.0

    def transition_to(self, new_state: FSMState) -> None:
        """Transition finite state machine to a new state."""
        self.fsm_state = FSMState(new_state)

    def reset_pid_integral(self) -> None:
        """Reset accumulated PID integral term."""
        self.pid_integral = 0.0
