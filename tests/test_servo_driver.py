"""Unit tests for drivers/servo_driver.py."""

import pytest

from drivers.servo_driver import ServoDriver, _ANGLE_MAX, _ANGLE_MIN, _PULSE_MAX_US, _PULSE_MIN_US


@pytest.fixture()
def driver():
    return ServoDriver(channel=0)


class TestAngleToPulse:
    def test_center_angle(self, driver):
        """90° should map to the midpoint pulse width."""
        pulse = driver._angle_to_pulse(90.0)
        expected = _PULSE_MIN_US + (90.0 / 180.0) * (_PULSE_MAX_US - _PULSE_MIN_US)
        assert pulse == int(round(expected))

    def test_zero_degrees(self, driver):
        assert driver._angle_to_pulse(0.0) == _PULSE_MIN_US

    def test_180_degrees(self, driver):
        assert driver._angle_to_pulse(180.0) == _PULSE_MAX_US

    def test_clamps_below_min(self, driver):
        assert driver._angle_to_pulse(-10.0) == _PULSE_MIN_US

    def test_clamps_above_max(self, driver):
        assert driver._angle_to_pulse(200.0) == _PULSE_MAX_US

    def test_returns_int(self, driver):
        assert isinstance(driver._angle_to_pulse(90.0), int)


class TestSendAngle:
    def test_send_angle_does_not_raise(self, driver):
        """send_angle must not raise with the stub _write_angle."""
        driver.send_angle(90.0)

    def test_send_angle_clamped_values_do_not_raise(self, driver):
        driver.send_angle(-999.0)
        driver.send_angle(999.0)


class TestCenter:
    def test_center_does_not_raise(self, driver):
        driver.center()

    def test_custom_center_angle(self):
        d = ServoDriver(center_angle=75.0)
        d.center()  # should not raise


class TestCustomPulseRange:
    def test_custom_pulse_min_max(self):
        d = ServoDriver(pulse_min_us=500, pulse_max_us=2500)
        assert d._angle_to_pulse(0.0) == 500
        assert d._angle_to_pulse(180.0) == 2500
