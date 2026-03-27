"""Unit tests for control/servo_pid.py."""

import pytest

from control.servo_pid import ServoPID, _STEERING_CLAMP
from models.robot_state import FSMState, RobotState


@pytest.fixture()
def state():
    return RobotState()


@pytest.fixture()
def controller(state):
    return ServoPID(state)


class TestHoldLogic:
    def test_none_when_locked_enters_gapping(self, controller, state):
        """Losing vision while LOCKED should transition to GAPPING."""
        state.transition_to(FSMState.LOCKED)
        controller.update(None)
        assert state.fsm_state == FSMState.GAPPING

    def test_none_when_searching_stays_searching(self, controller, state):
        """Receiving None while SEARCHING should not transition to GAPPING."""
        assert state.fsm_state == FSMState.SEARCHING
        controller.update(None)
        assert state.fsm_state == FSMState.SEARCHING

    def test_none_returns_last_valid_servo_angle(self, controller, state):
        """Hold logic must return last_valid_servo_angle when vision is lost."""
        state.last_valid_servo_angle = 105.0
        result = controller.update(None)
        assert result == pytest.approx(105.0)

    def test_none_does_not_update_pid_state(self, controller, state):
        """Integral and last_error must not change on a None update."""
        state.pid_integral = 2.0
        state.pid_last_error = 1.0
        controller.update(None)
        assert state.pid_integral == pytest.approx(2.0)
        assert state.pid_last_error == pytest.approx(1.0)


class TestValidSignal:
    def test_valid_signal_transitions_to_locked(self, controller, state):
        """Any valid theta should move the FSM to LOCKED."""
        controller.update(90.0)
        assert state.fsm_state == FSMState.LOCKED

    def test_zero_error_gives_center_angle(self, controller, state):
        """θ=90° → error=0 → steering_offset=0 → servo_angle=center."""
        result = controller.update(90.0)
        assert result == pytest.approx(state.servo_center_angle, abs=0.5)

    def test_positive_error_steers_above_center(self, controller, state):
        """θ > 90° → positive error → servo angle > center."""
        result = controller.update(100.0)
        assert result > state.servo_center_angle

    def test_negative_error_steers_below_center(self, controller, state):
        """θ < 90° → negative error → servo angle < center."""
        result = controller.update(80.0)
        assert result < state.servo_center_angle


class TestSteeringClamp:
    def test_large_positive_error_clamped(self, controller, state):
        """Very large positive error must not exceed center + 30°."""
        result = controller.update(200.0)  # error = 110°
        assert result <= state.servo_center_angle + _STEERING_CLAMP

    def test_large_negative_error_clamped(self, controller, state):
        """Very large negative error must not go below center - 30°."""
        result = controller.update(0.0)  # error = -90°
        assert result >= state.servo_center_angle - _STEERING_CLAMP


class TestIntegralReset:
    def test_integral_reset_on_reentry_from_gapping(self, controller, state):
        """Returning vision after GAPPING should zero the integral term."""
        state.transition_to(FSMState.LOCKED)
        controller.update(None)  # → GAPPING
        assert state.fsm_state == FSMState.GAPPING
        state.pid_integral = 99.9
        controller.update(90.0)  # valid signal → back to LOCKED
        assert state.pid_integral == pytest.approx(0.0, abs=1.0)

    def test_no_integral_reset_in_consecutive_locked_frames(self, controller, state):
        """Consecutive valid frames while LOCKED must not reset the integral."""
        state.transition_to(FSMState.LOCKED)
        controller.update(95.0)
        state.pid_integral = 5.0
        controller.update(95.0)
        assert state.pid_integral > 0.0


class TestFSMTransitions:
    def test_searching_plus_valid_goes_locked(self, controller, state):
        assert state.fsm_state == FSMState.SEARCHING
        controller.update(90.0)
        assert state.fsm_state == FSMState.LOCKED

    def test_locked_plus_none_goes_gapping(self, controller, state):
        state.transition_to(FSMState.LOCKED)
        controller.update(None)
        assert state.fsm_state == FSMState.GAPPING

    def test_gapping_plus_valid_goes_locked(self, controller, state):
        state.transition_to(FSMState.LOCKED)
        controller.update(None)  # → GAPPING
        controller.update(90.0)  # valid → LOCKED
        assert state.fsm_state == FSMState.LOCKED

    def test_last_valid_servo_angle_updated_on_valid_signal(self, controller, state):
        result = controller.update(90.0)
        assert state.last_valid_servo_angle == pytest.approx(result)
