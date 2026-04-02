"""Unit tests for drivers/servo_driver.py."""

import json

import pytest

from drivers.servo_driver import ServoDriver, _ANGLE_MAX, _ANGLE_MIN, _PULSE_MAX_US, _PULSE_MIN_US


@pytest.fixture()
def driver():
    return ServoDriver(channel=0)


class TestAngleToPulse:
    def test_center_angle(self, driver):
        pulse = driver._angle_to_pulse(90.0)
        expected = _PULSE_MIN_US + ((90.0 - _ANGLE_MIN) / (_ANGLE_MAX - _ANGLE_MIN)) * (
            _PULSE_MAX_US - _PULSE_MIN_US
        )
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

    def test_negative_angle_range_supported(self):
        from drivers import servo_driver as servo_module

        driver = ServoDriver(pulse_min_us=500, pulse_max_us=2500)
        original_min = servo_module._ANGLE_MIN
        original_max = servo_module._ANGLE_MAX
        servo_module._ANGLE_MIN = -90.0
        servo_module._ANGLE_MAX = 90.0
        try:
            assert driver._angle_to_pulse(-90.0) == 500
            assert driver._angle_to_pulse(0.0) == 1500
            assert driver._angle_to_pulse(90.0) == 2500
        finally:
            servo_module._ANGLE_MIN = original_min
            servo_module._ANGLE_MAX = original_max


class TestSendAngle:
    def test_send_angle_does_not_raise(self, driver):
        driver.send_angle(90.0)

    def test_send_angle_clamped_values_do_not_raise(self, driver):
        driver.send_angle(-999.0)
        driver.send_angle(999.0)


class TestCenter:
    def test_center_does_not_raise(self, driver):
        driver.center()

    def test_custom_center_angle(self):
        driver = ServoDriver(center_angle=75.0)
        driver.center()


class TestCustomPulseRange:
    def test_custom_pulse_min_max(self):
        driver = ServoDriver(pulse_min_us=500, pulse_max_us=2500)
        assert driver._angle_to_pulse(0.0) == 500
        assert driver._angle_to_pulse(180.0) == 2500


class FakeSocket:
    def __init__(self):
        self.sent = []
        self.closed = False

    def setsockopt(self, level, option, value):
        del level, option, value

    def sendall(self, data):
        self.sent.append(data.decode("utf-8"))

    def close(self):
        self.closed = True


class TestBridgeMode:
    def test_bridge_send_angle_sends_json(self, monkeypatch):
        fake_socket = FakeSocket()
        moments = iter([0.0])

        monkeypatch.setattr("drivers.servo_driver.socket.create_connection", lambda *args, **kwargs: fake_socket)
        monkeypatch.setattr("drivers.servo_driver.time.monotonic", lambda: next(moments))

        driver = ServoDriver(bridge_enabled=True, bridge_min_send_interval_s=0.0)
        driver.send_angle(90.0)

        assert len(fake_socket.sent) == 1
        payload = json.loads(fake_socket.sent[0].strip())
        assert payload["type"] == "angle"
        assert payload["angle"] == 90.0

    def test_bridge_throttles_and_sends_latest_angle(self, monkeypatch):
        fake_socket = FakeSocket()
        moments = iter([0.00, 0.01, 0.06])

        monkeypatch.setattr("drivers.servo_driver.socket.create_connection", lambda *args, **kwargs: fake_socket)
        monkeypatch.setattr("drivers.servo_driver.time.monotonic", lambda: next(moments))

        driver = ServoDriver(
            bridge_enabled=True,
            bridge_min_send_interval_s=0.05,
            bridge_min_angle_delta=0.0,
        )
        driver.send_angle(90.0)
        driver.send_angle(91.0)
        driver.send_angle(95.0)

        assert len(fake_socket.sent) == 2
        first_payload = json.loads(fake_socket.sent[0].strip())
        second_payload = json.loads(fake_socket.sent[1].strip())
        assert first_payload["angle"] == 90.0
        assert second_payload["angle"] == 95.0

    def test_bridge_skips_small_angle_delta(self, monkeypatch):
        fake_socket = FakeSocket()
        moments = iter([0.00, 0.10])

        monkeypatch.setattr("drivers.servo_driver.socket.create_connection", lambda *args, **kwargs: fake_socket)
        monkeypatch.setattr("drivers.servo_driver.time.monotonic", lambda: next(moments))

        driver = ServoDriver(
            bridge_enabled=True,
            bridge_min_send_interval_s=0.0,
            bridge_min_angle_delta=2.0,
        )
        driver.send_angle(90.0)
        driver.send_angle(91.0)

        assert len(fake_socket.sent) == 1
