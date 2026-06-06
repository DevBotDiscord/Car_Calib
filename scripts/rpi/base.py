"""Base motor control via direct pigpio GPIO writes."""

from __future__ import annotations

from . import config
from .logging_utils import get_logger

logger = get_logger("base")

FORWARD_STATE = (0, 1, 0)
BACKWARD_STATE = (0, 0, 1)
STOP_STATE = (0, 0, 0)
LOCK_STATE = (1, 0, 1)
UNLOCK_STATE = (1, 1, 0)
TURN_LEFT_STATE = (1, 0, 0)
TURN_RIGHT_STATE = (0, 1, 1)


def set_base(b1: int, b2: int, b3: int, label: str | None = None, force: bool = False) -> None:
    state = (b1, b2, b3)

    if config.gpio is None:
        raise RuntimeError("pigpio is not initialized.")

    # E-stop gate: only STOP_STATE may pass through while latched, and
    # only when the caller explicitly requests force (e.g. estop._safe_outputs).
    if config.estop_active and not force:
        if state != STOP_STATE:
            logger.warning("[BASE][ESTOP_BLOCKED] %s state=%d,%d,%d", label or "?", *state)
            return

    if state == config.last_base_state:
        return

    config.gpio.write(config.OUT1, 1 if b1 else 0)
    config.gpio.write(config.OUT2, 1 if b2 else 0)
    config.gpio.write(config.OUT3, 1 if b3 else 0)

    if label:
        logger.debug("[BASE] %s state=%d,%d,%d", label, *state)
    else:
        logger.debug("[BASE] state=%d,%d,%d", *state)
    config.last_base_state = state
    if label:
        config.last_base_label = label


def stop_base(force: bool = False) -> None:
    set_base(*STOP_STATE, "STOP", force=force)


def forward() -> None:
    set_base(*FORWARD_STATE, "FORWARD")


def backward() -> None:
    set_base(*BACKWARD_STATE, "BACKWARD")


def lock_base() -> None:
    set_base(*LOCK_STATE, "LOCK")


def unlock_base() -> None:
    set_base(*UNLOCK_STATE, "UNLOCK")


def turn_left() -> None:
    set_base(*TURN_LEFT_STATE, "TURN_LEFT")


def turn_right() -> None:
    set_base(*TURN_RIGHT_STATE, "TURN_RIGHT")


def toggle_relay() -> None:
    if config.gpio is None:
        raise RuntimeError("pigpio is not initialized.")
    config.relay_on = not config.relay_on
    config.gpio.write(config.RELAY_PIN, 1 if config.relay_on else 0)
    logger.info("[RELAY] %s pin=%d", "ON" if config.relay_on else "OFF", config.RELAY_PIN)
