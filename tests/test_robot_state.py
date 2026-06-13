"""Unit tests for models/robot_state.py."""

import pytest

from models.robot_state import FSMState, PIDConstants, RobotState
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


class TestFSMState:
    def test_enum_members_exist(self):
        assert FSMState.GAPPING
        assert FSMState.DANGER_LEFT
        assert FSMState.DANGER_RIGHT
        assert FSMState.AMBIGUOUS_DANGER
        assert FSMState.TRACKING_COAST
        assert FSMState.TRACKING_PD

    def test_distinct_values(self):
        states = {
            FSMState.GAPPING,
            FSMState.DANGER_LEFT,
            FSMState.DANGER_RIGHT,
            FSMState.AMBIGUOUS_DANGER,
            FSMState.TRACKING_COAST,
            FSMState.TRACKING_PD,
        }
        assert len(states) == 6


class TestPIDConstants:
    def test_defaults(self):
        pid = PIDConstants()
        assert pid.kp == pytest.approx(PID_KP)
        assert pid.ki == pytest.approx(PID_KI)
        assert pid.kd == pytest.approx(PID_KD)

    def test_custom_values(self):
        pid = PIDConstants(kp=2.0, ki=0.2, kd=0.5)
        assert pid.kp == pytest.approx(2.0)
        assert pid.ki == pytest.approx(0.2)
        assert pid.kd == pytest.approx(0.5)


class TestRobotState:
    def test_defaults(self):
        state = RobotState()
        assert state.servo_center_angle == pytest.approx(SERVO_CENTER_ANGLE)
        assert state.max_steering_offset == pytest.approx(MAX_STEERING_OFFSET)
        assert state.last_valid_servo_angle == pytest.approx(SERVO_CENTER_ANGLE)
        assert state.last_valid_command == pytest.approx(SERVO_CENTER_ANGLE)
        assert state.roi_height_pct == pytest.approx(ROI_HEIGHT_PCT)
        assert state.roi_top_width_pct == pytest.approx(ROI_TOP_WIDTH_PCT)
        assert state.roi_bottom_width_pct == pytest.approx(ROI_BOTTOM_WIDTH_PCT)
        assert state.debug_mode is ROBOT_DEBUG_MODE
        assert state.fsm_state == FSMState.GAPPING
        assert state.pid_integral == pytest.approx(0.0)
        assert state.pid_last_error == pytest.approx(0.0)

    def test_transition_changes_state(self):
        state = RobotState()
        state.transition_to(FSMState.TRACKING_PD)
        assert state.fsm_state == FSMState.TRACKING_PD

    def test_transition_same_state_is_noop(self):
        state = RobotState()
        state.transition_to(FSMState.GAPPING)
        assert state.fsm_state == FSMState.GAPPING

    def test_transition_to_danger(self):
        state = RobotState()
        state.transition_to(FSMState.TRACKING_COAST)
        state.transition_to(FSMState.DANGER_LEFT)
        assert state.fsm_state == FSMState.DANGER_LEFT

    def test_reset_pid_integral(self):
        state = RobotState()
        state.pid_integral = 42.0
        state.reset_pid_integral()
        assert state.pid_integral == pytest.approx(0.0)

    def test_custom_max_steering_offset(self):
        state = RobotState(max_steering_offset=45.0)
        assert state.max_steering_offset == pytest.approx(45.0)

    def test_custom_roi_parameters(self):
        state = RobotState(roi_height_pct=0.5, roi_top_width_pct=0.4,
                           roi_bottom_width_pct=0.8)
        assert state.roi_height_pct == pytest.approx(0.5)
        assert state.roi_top_width_pct == pytest.approx(0.4)
        assert state.roi_bottom_width_pct == pytest.approx(0.8)

    def test_debug_mode_default_false(self):
        assert RobotState().debug_mode is ROBOT_DEBUG_MODE

    def test_debug_mode_can_be_enabled(self):
        state = RobotState(debug_mode=True)
        assert state.debug_mode is True
