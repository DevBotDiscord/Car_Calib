"""Base motor control via direct pigpio GPIO writes."""

from __future__ import annotations

from . import config

FORWARD_STATE = (0, 1, 0)
BACKWARD_STATE = (0, 0, 1)
STOP_STATE = (0, 0, 0)
LOCK_STATE = (1, 0, 1)
UNLOCK_STATE = (1, 1, 0)


def set_base(b1: int, b2: int, b3: int, label: str | None = None) -> None:
    state = (b1, b2, b3)

    if config.gpio is None:
        raise RuntimeError("pigpio is not initialized.")

    config.gpio.write(config.OUT1, 1 if b1 else 0)
    config.gpio.write(config.OUT2, 1 if b2 else 0)
    config.gpio.write(config.OUT3, 1 if b3 else 0)

    if state != config.last_base_state:
        if label:
            print(f"BASE: {label} -> {state}")
        else:
            print(f"BASE: {state}")
        config.last_base_state = state


def stop_base() -> None:
    set_base(*STOP_STATE, "STOP")


def forward() -> None:
    set_base(*FORWARD_STATE, "FORWARD")


def backward() -> None:
    set_base(*BACKWARD_STATE, "BACKWARD")


def lock_base() -> None:
    set_base(*LOCK_STATE, "LOCK")


def unlock_base() -> None:
    set_base(*UNLOCK_STATE, "UNLOCK")
