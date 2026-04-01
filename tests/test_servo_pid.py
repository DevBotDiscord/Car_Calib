"""Unit tests for control/servo_pid.py."""

import pytest

from control.servo_pid import ServoPID
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
        """Very large positive error must not exceed center + max_steering_offset."""
        result = controller.update(200.0)  # error = 110°
        assert result <= state.servo_center_angle + state.max_steering_offset

    def test_large_negative_error_clamped(self, controller, state):
        """Very large negative error must not go below center - max_steering_offset."""
        result = controller.update(0.0)  # error = -90°
        assert result >= state.servo_center_angle - state.max_steering_offset

    def test_custom_max_steering_offset_respected(self, controller, state):
        """Setting a custom max_steering_offset should change the clamp range."""
        state.max_steering_offset = 10.0
        result = controller.update(200.0)  # very large error
        assert result <= state.servo_center_angle + 10.0


class TestIntegralReset:
    def test_integral_reset_on_reentry_from_gapping(self, controller, state):
        """Returning vision after GAPPING should zero the integral term."""
        state.transition_to(FSMState.LOCKED)
        controller.update(None)  # → GAPPING
        assert state.fsm_state == FSMState.GAPPING
        state.pid_integral = 99.9
        controller.update(90.0)  # relock 1/3
        controller.update(90.0)  # relock 2/3
        controller.update(90.0)  # relock 3/3 -> LOCKED
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
        controller.update(90.0)  # relock 1/3
        controller.update(90.0)  # relock 2/3
        controller.update(90.0)  # relock 3/3 -> LOCKED
        assert state.fsm_state == FSMState.LOCKED

    def test_gapping_relock_debounce_holds_before_required_frames(self, controller, state):
        """During relock debounce, FSM should remain GAPPING and hold last servo."""
        state.transition_to(FSMState.LOCKED)
        state.last_valid_servo_angle = 103.0
        controller.update(None)  # -> GAPPING

        hold_1 = controller.update(90.0)
        hold_2 = controller.update(90.0)

        assert hold_1 == pytest.approx(103.0)
        assert hold_2 == pytest.approx(103.0)
        assert state.fsm_state == FSMState.GAPPING

    def test_last_valid_servo_angle_updated_on_valid_signal(self, controller, state):
        result = controller.update(90.0)
        assert state.last_valid_servo_angle == pytest.approx(result)


class TestCalibrationStage:
    def test_calibration_stage_activates_when_error_large(self, controller, state):
        """Calibration stage should become active when theta is away from 90°."""
        assert state.calibration_active is False
        controller.update(100.0)
        assert state.calibration_active is True

    def test_calibration_stage_clears_near_90_and_resets_pid_memory(self, controller, state):
        """Returning near 90° should clear calibration stage and PID memory."""
        controller.update(100.0)
        state.pid_integral = 5.0
        state.pid_last_error = 3.0

        controller.update(90.0)

        assert state.calibration_active is False
        assert state.pid_integral == pytest.approx(0.0)
        assert state.pid_last_error == pytest.approx(0.0)

    def test_small_error_below_start_threshold_keeps_servo_centered(self, state):
        """Sub-threshold theta fluctuation must not start calibration."""
        controller = ServoPID(state, start_calib_threshold_deg=5.0, stop_calib_threshold_deg=3.0)

        result = controller.update(90.5)  # error=0.5° < start threshold

        assert state.calibration_active is False
        assert result == pytest.approx(state.servo_center_angle)
        assert state.pid_integral == pytest.approx(0.0)
        assert state.pid_last_error == pytest.approx(0.0)

    def test_threshold_hysteresis_controls_start_and_stop(self, state):
        """Calibration starts at high threshold and stops at low threshold."""
        controller = ServoPID(state, start_calib_threshold_deg=5.0, stop_calib_threshold_deg=3.0)

        # Below start threshold: should not calibrate.
        result1 = controller.update(94.0)  # error=4.0°
        assert state.calibration_active is False
        assert result1 == pytest.approx(state.servo_center_angle)

        # Cross start threshold: calibration should activate and move servo.
        result2 = controller.update(96.0)  # error=6.0°
        assert state.calibration_active is True
        assert result2 > state.servo_center_angle

        # Between stop and start: keep calibrating (hysteresis hold).
        result3 = controller.update(94.0)  # error=4.0°
        assert state.calibration_active is True
        assert result3 != pytest.approx(state.servo_center_angle)

        # Go below stop threshold: calibration should stop and recenter.
        result4 = controller.update(92.5)  # error=2.5°
        assert state.calibration_active is False
        assert result4 == pytest.approx(state.servo_center_angle)
        assert state.pid_integral == pytest.approx(0.0)
        assert state.pid_last_error == pytest.approx(0.0)
