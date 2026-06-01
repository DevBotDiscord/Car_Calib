"""Tests for compact RPi MQTT command logging."""

from __future__ import annotations

import importlib
from types import SimpleNamespace

import pytest


@pytest.fixture()
def mqtt_mod(monkeypatch):
    import scripts.rpi.config as config
    import scripts.rpi.logging_utils as logging_utils
    import scripts.rpi.mqtt_client as mqtt_client

    logging_utils = importlib.reload(logging_utils)
    logging_utils.setup_rpi_logging()
    mqtt_client = importlib.reload(mqtt_client)

    class FakeGPIO:
        def __init__(self) -> None:
            self.writes: list[tuple[int, int]] = []

        def write(self, pin: int, value: int) -> None:
            self.writes.append((pin, value))

        def set_servo_pulsewidth(self, pin: int, width: int) -> None:
            self.writes.append((pin, width))

    fake_gpio = FakeGPIO()
    monkeypatch.setattr(config, "gpio", fake_gpio)
    monkeypatch.setattr(config, "OUT1", 17)
    monkeypatch.setattr(config, "OUT2", 27)
    monkeypatch.setattr(config, "OUT3", 22)
    monkeypatch.setattr(config, "RELAY_PIN", 5)
    monkeypatch.setattr(config, "last_base_state", None)
    monkeypatch.setattr(config, "relay_on", False)
    monkeypatch.setattr(config, "script_active", False)
    monkeypatch.setattr(config, "CENTER_ANGLE", -8.0)
    monkeypatch.setattr(config, "LEFT_LIMIT", -53.0)
    monkeypatch.setattr(config, "RIGHT_LIMIT", 37.0)
    monkeypatch.setattr(config, "SERVO_PIN", 12)
    monkeypatch.setattr(config, "SERVO_MIN_PULSE_US", 500)
    monkeypatch.setattr(config, "SERVO_MAX_PULSE_US", 2500)
    monkeypatch.setattr(config, "STEER_DEADBAND_DEG", 0.0)
    monkeypatch.setattr(config, "last_steer_angle", None)
    monkeypatch.setattr(config, "last_steer_source", None)
    monkeypatch.setattr(config, "steer_angle", -8.0)
    monkeypatch.setattr(config, "REMOTE_INPUT_MIN_ANGLE", 60.0)
    monkeypatch.setattr(config, "REMOTE_INPUT_CENTER_ANGLE", 90.0)
    monkeypatch.setattr(config, "REMOTE_INPUT_MAX_ANGLE", 120.0)
    monkeypatch.setattr(config, "MQTT_BROKER_HOST", "127.0.0.1")
    monkeypatch.setattr(config, "MQTT_BROKER_PORT", 1883)
    monkeypatch.setattr(config, "MQTT_SERVO_TOPIC", "car/servo/angle")
    monkeypatch.setattr(config, "MQTT_BASE_COMMAND_TOPIC", "car/base/command")
    monkeypatch.setattr(config, "MQTT_RELAY_TOPIC", "car/relay")
    monkeypatch.setattr(config, "MQTT_STATUS_TOPIC", "car/status")
    monkeypatch.setattr(config, "running", True)

    return SimpleNamespace(
        module=mqtt_client,
        config=config,
        gpio=fake_gpio,
        logging_utils=logging_utils,
    )


def test_base_command_logs_compact_info(mqtt_mod, capsys) -> None:
    mqtt_mod.logging_utils.setup_rpi_logging()
    mqtt_mod.module.handle_base_message("TURN_LEFT")
    captured = capsys.readouterr()

    assert mqtt_mod.config.last_base_state == (1, 0, 0)
    assert "[MQTT][RX][BASE] TURN_LEFT state=1,0,0" in captured.out


def test_unknown_base_command_logs_warning(mqtt_mod, capsys) -> None:
    mqtt_mod.logging_utils.setup_rpi_logging()
    mqtt_mod.module.handle_base_message("SPIN")
    captured = capsys.readouterr()

    assert "WARNING [MQTT][RX][BASE] unknown command=SPIN" in captured.out


def test_relay_command_logs_compact_info(mqtt_mod, capsys) -> None:
    mqtt_mod.logging_utils.setup_rpi_logging()
    mqtt_mod.module.handle_relay_message("ON")
    captured = capsys.readouterr()

    assert mqtt_mod.config.relay_on is True
    assert (5, 1) in mqtt_mod.gpio.writes
    assert "[MQTT][RX][RELAY] ON pin=5" in captured.out


def test_script_active_logs_compact_info(mqtt_mod, capsys) -> None:
    mqtt_mod.logging_utils.setup_rpi_logging()
    mqtt_mod.module.handle_script_active_message("START")
    captured = capsys.readouterr()

    assert mqtt_mod.config.script_active is True
    assert "[MQTT][RX][SCRIPT] script_active=ON" in captured.out


def test_servo_ignored_when_script_inactive_logs_warning(mqtt_mod, capsys) -> None:
    mqtt_mod.logging_utils.setup_rpi_logging()
    mqtt_mod.module.handle_servo_message('{"angle": -12}')
    captured = capsys.readouterr()

    assert "WARNING [MQTT][RX][SERVO] ignored script_active=OFF raw={\"angle\": -12}" in captured.out


def test_servo_apply_logs_compact_info(mqtt_mod, capsys) -> None:
    mqtt_mod.logging_utils.setup_rpi_logging()
    mqtt_mod.config.script_active = True

    mqtt_mod.module.handle_servo_message('{"angle": -12}')
    captured = capsys.readouterr()

    assert mqtt_mod.config.steer_angle == -12.0
    assert "[MQTT][RX][SERVO] apply angle=-12.00 raw={\"angle\": -12}" in captured.out


def test_invalid_payload_logs_warning_from_callback(mqtt_mod, capsys) -> None:
    mqtt_mod.logging_utils.setup_rpi_logging()
    message = SimpleNamespace(topic="car/unknown", payload=b"PING")

    mqtt_mod.module.on_mqtt_message(None, None, message)
    captured = capsys.readouterr()

    assert "WARNING [MQTT][RX] unhandled topic=car/unknown" in captured.out
