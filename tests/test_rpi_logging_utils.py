"""Tests for RPi logging setup helper."""

from __future__ import annotations

import importlib
import logging


def _reload_logging_utils(monkeypatch):
    monkeypatch.delenv("RPI_LOG_LEVEL", raising=False)
    import scripts.rpi.logging_utils as logging_utils

    return importlib.reload(logging_utils)


def test_setup_rpi_logging_uses_compact_stdout_format(monkeypatch, capsys) -> None:
    logging_utils = _reload_logging_utils(monkeypatch)
    logger = logging_utils.setup_rpi_logging()

    logger.info("[MQTT][RX][BASE] TURN_LEFT state=1,0,0")
    captured = capsys.readouterr()

    assert "INFO [MQTT][RX][BASE] TURN_LEFT state=1,0,0" in captured.out
    assert "car_calib.rpi" not in captured.out


def test_setup_rpi_logging_is_idempotent(monkeypatch, capsys) -> None:
    logging_utils = _reload_logging_utils(monkeypatch)
    logger = logging_utils.setup_rpi_logging()
    logging_utils.setup_rpi_logging()

    logger.info("[MQTT][CONN] connected host=127.0.0.1 port=1883")
    captured = capsys.readouterr()

    assert captured.out.count("[MQTT][CONN] connected") == 1


def test_get_logger_returns_child_logger(monkeypatch) -> None:
    logging_utils = _reload_logging_utils(monkeypatch)
    logger = logging_utils.get_logger("mqtt")

    assert logger.name == "car_calib.rpi.mqtt"


def test_setup_rpi_logging_honors_env_level(monkeypatch, capsys) -> None:
    monkeypatch.setenv("RPI_LOG_LEVEL", "WARNING")
    import scripts.rpi.logging_utils as logging_utils

    logging_utils = importlib.reload(logging_utils)
    logger = logging_utils.setup_rpi_logging()

    logger.info("[MQTT][RX][BASE] hidden")
    logger.warning("[MQTT][RX][BASE] visible")
    captured = capsys.readouterr()

    assert "hidden" not in captured.out
    assert "WARNING [MQTT][RX][BASE] visible" in captured.out
