"""Unit tests for control/heading_controller.py."""

import time

import pytest

from control.heading_controller import HeadingController, _HYSTERESIS_HIGH, _HYSTERESIS_LOW
from models.robot_state import FSMState, RobotState


@pytest.fixture()
def state():
    return RobotState()


@pytest.fixture()
def controller(state):
    return HeadingController(state)


class TestSparseSignal:
    def test_none_input_enters_gapping(self, controller, state):
        state.transition_to(FSMState.LOCKED)
        controller.update(None)
        assert state.fsm_state == FSMState.GAPPING

    def test_none_input_returns_last_valid_command(self, controller, state):
        state.last_valid_command = 7.5
        result = controller.update(None)
        assert result == pytest.approx(7.5)

    def test_none_does_not_reset_pid_integral(self, controller, state):
        state.pid_integral = 3.0
        controller.update(None)
        assert state.pid_integral == pytest.approx(3.0)


class TestHysteresis:
    def test_no_correction_below_high_threshold(self, controller, state):
        """Error below 5° should not trigger a new correction."""
        state.last_valid_command = 0.0
        result = controller.update(4.9)
        # Should stay at last_valid_command (no correction activated)
        assert result == pytest.approx(0.0)

    def test_correction_activates_above_high_threshold(self, controller, state):
        """Error above 5° should trigger PID computation."""
        result = controller.update(6.0)
        # PID output for e=6 with default gains should be non-zero
        assert result != 0.0

    def test_correction_deactivates_below_low_threshold(self, controller, state):
        """Once correcting, error below 3° should deactivate correction."""
        # Activate correction
        controller.update(6.0)
        assert controller._correcting is True
        # Now drop below low threshold
        state.last_valid_command = 5.0
        result = controller.update(2.0)
        assert controller._correcting is False
        assert result == pytest.approx(5.0)

    def test_hysteresis_stays_active_between_thresholds(self, controller, state):
        """Error in (3°, 5°) should keep existing correction state."""
        # Activate first
        controller.update(6.0)
        assert controller._correcting is True
        # Drop to mid-band: still correcting
        controller.update(4.0)
        assert controller._correcting is True


class TestIntegralReset:
    def test_integral_reset_on_reentry_from_gapping(self, controller, state):
        """Returning vision after a gap should zero the integral term."""
        state.transition_to(FSMState.GAPPING)
        state.pid_integral = 99.9
        controller.update(6.0)  # valid signal after gap
        assert state.pid_integral == pytest.approx(0.0, abs=1.0)

    def test_no_integral_reset_if_already_locked(self, controller, state):
        """Consecutive valid frames should NOT reset the integral."""
        state.transition_to(FSMState.LOCKED)
        controller.update(6.0)  # first valid frame
        state.pid_integral = 5.0
        controller.update(6.0)  # second valid frame – no reset
        assert state.pid_integral > 0.0


class TestFSMTransitions:
    def test_locked_on_valid_signal(self, controller, state):
        controller.update(6.0)
        assert state.fsm_state == FSMState.LOCKED

    def test_gapping_on_none(self, controller, state):
        controller.update(None)
        assert state.fsm_state == FSMState.GAPPING

    def test_returns_to_locked_after_gapping(self, controller, state):
        controller.update(None)
        assert state.fsm_state == FSMState.GAPPING
        controller.update(6.0)
        assert state.fsm_state == FSMState.LOCKED
