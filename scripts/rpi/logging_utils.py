"""Compact stdout logging for Raspberry Pi bridge modules."""

from __future__ import annotations

import logging
import os
import sys

_LOGGER_ROOT = "car_calib.rpi"
_HANDLER_MARKER = "_car_calib_rpi_handler"
_DEFAULT_FORMAT = "%(asctime)s %(levelname)s %(message)s"
_DEFAULT_DATEFMT = "%H:%M:%S"


def _resolve_level() -> int:
    level_name = os.getenv("RPI_LOG_LEVEL", "INFO").strip().upper()
    level = getattr(logging, level_name, None)
    if isinstance(level, int):
        return level
    return logging.INFO


def setup_rpi_logging() -> logging.Logger:
    """Configure compact stdout logging for all RPi bridge loggers."""
    logger = logging.getLogger(_LOGGER_ROOT)
    level = _resolve_level()
    logger.setLevel(level)
    logger.propagate = False

    formatter = logging.Formatter(_DEFAULT_FORMAT, datefmt=_DEFAULT_DATEFMT)
    for handler in logger.handlers:
        if getattr(handler, _HANDLER_MARKER, False):
            handler.setLevel(level)
            handler.setFormatter(formatter)
            handler.stream = sys.stdout
            return logger

    handler = logging.StreamHandler(sys.stdout)
    setattr(handler, _HANDLER_MARKER, True)
    handler.setLevel(level)
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    return logger


def get_logger(name: str | None = None) -> logging.Logger:
    """Return RPi logger or named child logger."""
    if not name:
        return logging.getLogger(_LOGGER_ROOT)
    return logging.getLogger(f"{_LOGGER_ROOT}.{name}")
