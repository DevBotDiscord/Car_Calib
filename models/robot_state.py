"""Model module: robot state for the heading-hold system.

Manages:
- PID constants (Kp, Ki, Kd)
- ``servo_center_angle`` (neutral position, default 90°) and
  ``last_valid_servo_angle`` (fallback during GAPPING)
- FSM states: SEARCHING, LOCKED, GAPPING
"""

import logging
from dataclasses import dataclass, field
from enum import Enum, auto

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

    kp: float = 1.0
    ki: float = 0.05
    kd: float = 0.1


@dataclass
class RobotState:
    """Mutable state shared across MVC layers.

    Attributes:
        pid: PID gain constants.
        servo_center_angle: Neutral servo angle in degrees (default 90°).
        last_valid_servo_angle: Most recent servo angle issued while LOCKED.
            Used as a fallback during GAPPING.
        fsm_state: Current FSM state.
        pid_integral: Accumulated integral term for the PID controller.
        pid_last_error: Previous error for the derivative calculation.
    """

    pid: PIDConstants = field(default_factory=PIDConstants)
    servo_center_angle: float = 90.0
    last_valid_servo_angle: float = 90.0
    fsm_state: FSMState = FSMState.SEARCHING
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
