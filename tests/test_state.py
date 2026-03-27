"""Unit tests for models/state.py."""

import pytest

from models.state import FSMState, PIDConstants, RobotState


class TestFSMState:
    def test_enum_members_exist(self):
        assert FSMState.IDLE
        assert FSMState.CALIBRATING
        assert FSMState.DEAD_RECKONING

    def test_distinct_values(self):
        states = {FSMState.IDLE, FSMState.CALIBRATING, FSMState.DEAD_RECKONING}
        assert len(states) == 3


class TestPIDConstants:
    def test_defaults(self):
        pid = PIDConstants()
        assert pid.kp == pytest.approx(1.0)
        assert pid.ki == pytest.approx(0.05)
        assert pid.kd == pytest.approx(0.1)

    def test_custom_values(self):
        pid = PIDConstants(kp=2.0, ki=0.1, kd=0.5)
        assert pid.kp == pytest.approx(2.0)
        assert pid.ki == pytest.approx(0.1)
        assert pid.kd == pytest.approx(0.5)


class TestRobotState:
    def test_defaults(self):
        state = RobotState()
        assert state.heading_error == pytest.approx(0.0)
        assert state.last_valid_command == pytest.approx(0.0)
        assert state.fsm_state == FSMState.IDLE
        assert state.pid_integral == pytest.approx(0.0)
        assert state.pid_last_error == pytest.approx(0.0)

    def test_transition_changes_state(self):
        state = RobotState()
        state.transition_to(FSMState.CALIBRATING)
        assert state.fsm_state == FSMState.CALIBRATING

    def test_transition_same_state_is_noop(self):
        state = RobotState()
        state.transition_to(FSMState.IDLE)
        assert state.fsm_state == FSMState.IDLE

    def test_reset_pid_integral(self):
        state = RobotState()
        state.pid_integral = 42.0
        state.reset_pid_integral()
        assert state.pid_integral == pytest.approx(0.0)

    def test_transition_to_dead_reckoning(self):
        state = RobotState()
        state.transition_to(FSMState.CALIBRATING)
        state.transition_to(FSMState.DEAD_RECKONING)
        assert state.fsm_state == FSMState.DEAD_RECKONING
