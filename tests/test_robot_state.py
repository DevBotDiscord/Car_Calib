"""Unit tests for models/robot_state.py."""

import pytest

from models.robot_state import FSMState, PIDConstants, RobotState


class TestFSMState:
    def test_enum_members_exist(self):
        assert FSMState.SEARCHING
        assert FSMState.LOCKED
        assert FSMState.GAPPING

    def test_distinct_values(self):
        states = {FSMState.SEARCHING, FSMState.LOCKED, FSMState.GAPPING}
        assert len(states) == 3


class TestPIDConstants:
    def test_defaults(self):
        pid = PIDConstants()
        assert pid.kp == pytest.approx(1.0)
        assert pid.ki == pytest.approx(0.05)
        assert pid.kd == pytest.approx(0.1)

    def test_custom_values(self):
        pid = PIDConstants(kp=2.0, ki=0.2, kd=0.5)
        assert pid.kp == pytest.approx(2.0)
        assert pid.ki == pytest.approx(0.2)
        assert pid.kd == pytest.approx(0.5)


class TestRobotState:
    def test_defaults(self):
        state = RobotState()
        assert state.servo_center_angle == pytest.approx(90.0)
        assert state.last_valid_servo_angle == pytest.approx(90.0)
        assert state.fsm_state == FSMState.SEARCHING
        assert state.pid_integral == pytest.approx(0.0)
        assert state.pid_last_error == pytest.approx(0.0)

    def test_transition_changes_state(self):
        state = RobotState()
        state.transition_to(FSMState.LOCKED)
        assert state.fsm_state == FSMState.LOCKED

    def test_transition_same_state_is_noop(self):
        state = RobotState()
        state.transition_to(FSMState.SEARCHING)
        assert state.fsm_state == FSMState.SEARCHING

    def test_transition_to_gapping(self):
        state = RobotState()
        state.transition_to(FSMState.LOCKED)
        state.transition_to(FSMState.GAPPING)
        assert state.fsm_state == FSMState.GAPPING

    def test_reset_pid_integral(self):
        state = RobotState()
        state.pid_integral = 42.0
        state.reset_pid_integral()
        assert state.pid_integral == pytest.approx(0.0)
