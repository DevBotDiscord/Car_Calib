"""Unit tests for drivers/servo_driver.py."""

import importlib
import json

import pytest

from drivers import servo_driver as servo_module
from drivers.servo_driver import ServoDriver


@pytest.fixture(autouse=True)
def reset_servo_module_defaults(monkeypatch):
    monkeypatch.setattr(servo_module, "_ANGLE_MIN", 0.0)
    monkeypatch.setattr(servo_module, "_ANGLE_MAX", 180.0)
    monkeypatch.setattr(servo_module, "_PULSE_MIN_US", 1000)
    monkeypatch.setattr(servo_module, "_PULSE_MAX_US", 2000)


@pytest.fixture()
def driver():
    return ServoDriver(channel=0, mqtt_enabled=False, bridge_enabled=False)


class TestAngleToPulse:
    def test_center_angle(self, driver):
        pulse = driver._angle_to_pulse(90.0)
        expected = servo_module._PULSE_MIN_US + (
            (90.0 - servo_module._ANGLE_MIN) / (servo_module._ANGLE_MAX - servo_module._ANGLE_MIN)
        ) * (
            servo_module._PULSE_MAX_US - servo_module._PULSE_MIN_US
        )
        assert pulse == int(round(expected))

    def test_zero_degrees(self, driver):
        assert driver._angle_to_pulse(0.0) == servo_module._PULSE_MIN_US

    def test_180_degrees(self, driver):
        assert driver._angle_to_pulse(180.0) == servo_module._PULSE_MAX_US

    def test_clamps_below_min(self, driver):
        assert driver._angle_to_pulse(-10.0) == servo_module._PULSE_MIN_US

    def test_clamps_above_max(self, driver):
        assert driver._angle_to_pulse(200.0) == servo_module._PULSE_MAX_US

    def test_returns_int(self, driver):
        assert isinstance(driver._angle_to_pulse(90.0), int)

    def test_negative_angle_range_supported(self):
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
        driver = ServoDriver(center_angle=75.0, mqtt_enabled=False, bridge_enabled=False)
        driver.center()


class TestCustomPulseRange:
    def test_custom_pulse_min_max(self):
        driver = ServoDriver(
            pulse_min_us=500,
            pulse_max_us=2500,
            mqtt_enabled=False,
            bridge_enabled=False,
        )
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


class FakeMQTTPublishResult:
    def __init__(self, rc=0):
        self.rc = rc
        self.waited = False

    def wait_for_publish(self):
        self.waited = True


class FakeMQTTClient:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        self.connected = None
        self.started = False
        self.stopped = False
        self.disconnected = False
        self.username = None
        self.published = []
        self.publish_results = []
        self.retains = []

    def username_pw_set(self, username, password):
        self.username = (username, password)

    def connect(self, host, port, keepalive=60):
        self.connected = (host, port, keepalive)

    def loop_start(self):
        self.started = True

    def loop_stop(self):
        self.stopped = True

    def disconnect(self):
        self.disconnected = True

    def publish(self, topic, payload, retain=False):
        self.published.append((topic, payload))
        self.retains.append(retain)
        result = FakeMQTTPublishResult()
        self.publish_results.append(result)
        return result


class FakeMQTTModule:
    class CallbackAPIVersion:
        VERSION1 = object()

    def __init__(self):
        self.clients = []

    def Client(self, *args, **kwargs):
        client = FakeMQTTClient(*args, **kwargs)
        self.clients.append(client)
        return client


class TestBridgeMode:
    def test_bridge_send_angle_sends_json(self, monkeypatch):
        fake_socket = FakeSocket()
        moments = iter([0.0])

        monkeypatch.setattr("drivers.servo_driver.socket.create_connection", lambda *args, **kwargs: fake_socket)
        monkeypatch.setattr("drivers.servo_driver.time.monotonic", lambda: next(moments))

        driver = ServoDriver(
            bridge_enabled=True,
            bridge_min_send_interval_s=0.0,
            mqtt_enabled=False,
        )
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
            mqtt_enabled=False,
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
            mqtt_enabled=False,
        )
        driver.send_angle(90.0)
        driver.send_angle(91.0)

        assert len(fake_socket.sent) == 1


class TestMqttMode:
    def test_mqtt_send_angle_publishes_angle_payload(self, monkeypatch):
        fake_mqtt = FakeMQTTModule()

        monkeypatch.setattr(
            "drivers.servo_driver.importlib.import_module",
            lambda name: fake_mqtt if name == "paho.mqtt.client" else importlib.import_module(name),
        )

        driver = ServoDriver(
            mqtt_enabled=True,
            mqtt_host="broker.local",
            mqtt_port=1883,
            mqtt_topic="car/servo/angle",
        )
        driver.send_angle(95.5)

        client = fake_mqtt.clients[0]
        assert client.connected == ("broker.local", 1883, 60)
        assert client.published == [("car/servo/angle", "95.5000")]
        assert client.retains == [True]
        assert client.publish_results[0].waited is False

    def test_mqtt_publishes_every_send(self, monkeypatch):
        fake_mqtt = FakeMQTTModule()

        monkeypatch.setattr(
            "drivers.servo_driver.importlib.import_module",
            lambda name: fake_mqtt if name == "paho.mqtt.client" else importlib.import_module(name),
        )

        driver = ServoDriver(
            mqtt_enabled=True,
            mqtt_host="broker.local",
            mqtt_port=1883,
            mqtt_topic="car/servo/angle",
        )
        driver.send_angle(95.5)
        driver.send_angle(95.9)
        driver.send_angle(96.1)

        client = fake_mqtt.clients[0]
        assert client.published == [
            ("car/servo/angle", "95.5000"),
            ("car/servo/angle", "95.9000"),
            ("car/servo/angle", "96.1000"),
        ]
        assert client.retains == [True, True, True]

    def test_mqtt_center_uses_center_angle_payload(self, monkeypatch):
        fake_mqtt = FakeMQTTModule()

        monkeypatch.setattr(
            "drivers.servo_driver.importlib.import_module",
            lambda name: fake_mqtt if name == "paho.mqtt.client" else importlib.import_module(name),
        )

        driver = ServoDriver(
            mqtt_enabled=True,
            center_angle=75.0,
            mqtt_topic="car/servo/angle",
        )
        driver.center()

        client = fake_mqtt.clients[0]
        assert client.published == [("car/servo/angle", "75.0000")]
        assert client.retains == [True]

    def test_mqtt_close_disconnects_client(self, monkeypatch):
        fake_mqtt = FakeMQTTModule()

        monkeypatch.setattr(
            "drivers.servo_driver.importlib.import_module",
            lambda name: fake_mqtt if name == "paho.mqtt.client" else importlib.import_module(name),
        )

        driver = ServoDriver(mqtt_enabled=True)
        driver.send_angle(90.0)
        client = fake_mqtt.clients[0]

        driver.close()

        assert client.stopped is True
        assert client.disconnected is True
