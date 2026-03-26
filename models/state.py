"""Model module: robot state management and Finite State Machine (FSM).

Manages:
- Current heading error ``e``
- PID constants (Kp, Ki, Kd)
- ``last_valid_command`` for sparse-signal fallback
- FSM states: IDLE, CALIBRATING, DEAD_RECKONING
"""

import logging
from dataclasses import dataclass, field
from enum import Enum, auto

logger = logging.getLogger(__name__)


class FSMState(Enum):
    """Finite State Machine states for the robot heading-stability system."""

    IDLE = auto()
    """Robot is stationary, awaiting a start command."""

    CALIBRATING = auto()
    """Robot is actively detecting floor tiles and computing heading error."""

    DEAD_RECKONING = auto()
    """Vision signal lost; robot coasts on the last valid command."""


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
        heading_error: Current heading error ``e`` in degrees
            (``|theta_robot - 90|``).
        pid: PID gain constants.
        last_valid_command: Most recent non-None PID output sent to the
            motors.  Used as a fallback during DEAD_RECKONING.
        fsm_state: Current FSM state.
        pid_integral: Accumulated integral term for the PID controller.
        pid_last_error: Previous heading error for derivative calculation.
    """

    heading_error: float = 0.0
    pid: PIDConstants = field(default_factory=PIDConstants)
    last_valid_command: float = 0.0
    fsm_state: FSMState = FSMState.IDLE
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
