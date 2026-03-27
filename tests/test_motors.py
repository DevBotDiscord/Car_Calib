"""Unit tests for drivers/motors.py."""

import pytest

from drivers.motors import MotorDriver


@pytest.fixture()
def driver():
    return MotorDriver()


class TestClamp:
    def test_clamp_within_range(self):
        assert MotorDriver._clamp(100.0, 0, 255) == 100

    def test_clamp_below_min(self):
        assert MotorDriver._clamp(-10.0, 0, 255) == 0

    def test_clamp_above_max(self):
        assert MotorDriver._clamp(300.0, 0, 255) == 255

    def test_clamp_at_boundary(self):
        assert MotorDriver._clamp(0.0, 0, 255) == 0
        assert MotorDriver._clamp(255.0, 0, 255) == 255

    def test_clamp_rounds(self):
        assert MotorDriver._clamp(127.6, 0, 255) == 128


class TestSetPWM:
    def test_zero_output_gives_symmetric_pwm(self, driver):
        left, right = driver.set_pwm(0.0)
        assert left == right == 128

    def test_positive_output_steers_right(self, driver):
        left, right = driver.set_pwm(20.0)
        assert left > right

    def test_negative_output_steers_left(self, driver):
        left, right = driver.set_pwm(-20.0)
        assert left < right

    def test_pwm_values_within_range(self, driver):
        for output in [-200.0, -10.0, 0.0, 10.0, 200.0]:
            left, right = driver.set_pwm(output)
            assert 0 <= left <= 255
            assert 0 <= right <= 255

    def test_custom_centre(self):
        driver = MotorDriver(pwm_centre=100)
        left, right = driver.set_pwm(0.0)
        assert left == right == 100

    def test_returns_integers(self, driver):
        left, right = driver.set_pwm(10.5)
        assert isinstance(left, int)
        assert isinstance(right, int)


class TestStop:
    def test_stop_does_not_raise(self, driver):
        """stop() must not raise even with the stub _write_pwm."""
        driver.stop()  # should complete without exception
