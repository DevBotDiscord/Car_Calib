"""Unit tests for drivers/servo_driver.py."""

import json

import pytest

from config.settings import MQTT_SERVO_TOPIC
from drivers import servo_driver as servo_module
from drivers.servo_driver import ServoDriver


@pytest.fixture(autouse=True)
def reset_angle_range(monkeypatch):
    # _angle_to_pulse clamps against module-level _ANGLE_MIN/_ANGLE_MAX.
    monkeypatch.setattr(servo_module, "_ANGLE_MIN", 0.0)
    monkeypatch.setattr(servo_module, "_ANGLE_MAX", 180.0)
    # Never let an optional route-script lock suppress publishes in tests.
    monkeypatch.setattr(servo_module, "_script_lock", None)


@pytest.fixture()
def driver():
    return ServoDriver(
        channel=0,
        mqtt_enabled=False,
        pulse_min_us=1000,
        pulse_max_us=2000,
    )


class TestAngleToPulse:
    def test_center_angle(self, driver):
        # 90 deg is the midpoint of [0,180] → midpoint of [1000,2000].
        assert driver._angle_to_pulse(90.0) == 1500

    def test_zero_degrees(self, driver):
        assert driver._angle_to_pulse(0.0) == 1000

    def test_180_degrees(self, driver):
        assert driver._angle_to_pulse(180.0) == 2000

    def test_clamps_below_min(self, driver):
        assert driver._angle_to_pulse(-10.0) == 1000

    def test_clamps_above_max(self, driver):
        assert driver._angle_to_pulse(200.0) == 2000

    def test_returns_int(self, driver):
        assert isinstance(driver._angle_to_pulse(90.0), int)

    def test_negative_angle_range_supported(self, monkeypatch):
        monkeypatch.setattr(servo_module, "_ANGLE_MIN", -90.0)
        monkeypatch.setattr(servo_module, "_ANGLE_MAX", 90.0)
        driver = ServoDriver(pulse_min_us=500, pulse_max_us=2500)
        assert driver._angle_to_pulse(-90.0) == 500
        assert driver._angle_to_pulse(0.0) == 1500
        assert driver._angle_to_pulse(90.0) == 2500


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
        driver = ServoDriver(center_angle=75.0, mqtt_enabled=False)
        driver.center()


class TestCustomPulseRange:
    def test_custom_pulse_min_max(self):
        driver = ServoDriver(pulse_min_us=500, pulse_max_us=2500, mqtt_enabled=False)
        assert driver._angle_to_pulse(0.0) == 500
        assert driver._angle_to_pulse(180.0) == 2500


class _FakeMQTTClient:
    def __init__(self):
        self.connected_async = None
        self.started = False
        self.stopped = False
        self.disconnected = False
        self.published: list[tuple[str, str, int]] = []

    def username_pw_set(self, username, password):
        self._user = (username, password)

    def reconnect_delay_set(self, min_delay, max_delay):
        self._reconnect = (min_delay, max_delay)

    def connect_async(self, host, port, keepalive=60):
        self.connected_async = (host, port, keepalive)

    def loop_start(self):
        self.started = True

    def loop_stop(self):
        self.stopped = True

    def disconnect(self):
        self.disconnected = True

    def publish(self, topic, payload, qos=0):
        self.published.append((topic, payload, qos))


class _FakeMQTTModule:
    class CallbackAPIVersion:
        VERSION1 = object()

    def __init__(self):
        self.clients: list[_FakeMQTTClient] = []

    def Client(self, *args, **kwargs):
        client = _FakeMQTTClient()
        self.clients.append(client)
        return client


class TestMqttMode:
    def test_mqtt_send_angle_publishes_int_angle_payload(self, monkeypatch):
        fake = _FakeMQTTModule()
        monkeypatch.setattr(servo_module, "mqtt", fake)

        driver = ServoDriver(mqtt_enabled=True)
        driver.send_angle(95.0)

        client = fake.clients[0]
        assert client.started is True
        assert client.published == [(MQTT_SERVO_TOPIC, json.dumps({"angle": 95}), 0)]

    def test_mqtt_rounds_to_nearest_int(self, monkeypatch):
        fake = _FakeMQTTModule()
        monkeypatch.setattr(servo_module, "mqtt", fake)

        driver = ServoDriver(mqtt_enabled=True)
        driver.send_angle(95.4)

        client = fake.clients[0]
        assert client.published == [(MQTT_SERVO_TOPIC, json.dumps({"angle": 95}), 0)]

    def test_mqtt_center_uses_center_angle(self, monkeypatch):
        fake = _FakeMQTTModule()
        monkeypatch.setattr(servo_module, "mqtt", fake)

        driver = ServoDriver(mqtt_enabled=True, center_angle=75.0)
        driver.center()

        client = fake.clients[0]
        assert client.published == [(MQTT_SERVO_TOPIC, json.dumps({"angle": 75}), 0)]

    def test_mqtt_close_disconnects_client(self, monkeypatch):
        fake = _FakeMQTTModule()
        monkeypatch.setattr(servo_module, "mqtt", fake)

        driver = ServoDriver(mqtt_enabled=True)
        driver.send_angle(90.0)
        client = fake.clients[0]

        driver.close()

        assert client.disconnected is True
        assert client.stopped is True
