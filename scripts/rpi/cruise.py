"""Auto-cruise mode: timed forward drive with MQTT steer."""

from __future__ import annotations

from . import config
from .base import forward, stop_base
from .config import _clamp as clamp


def start_cruise(now: float) -> None:
    if config.cruise_active:
        return

    config.cruise_active = True
    config.cruise_phase = "vision"
    config.cruise_straight_count = 0
    config.cruise_start_time = now
    config.cruise_prev_remote_steer_only = config.controller_remote_steer_only
    config.controller_remote_steer_only = True
    forward()
    print(f"CRUISE: started (phase=vision, {config.CRUISE_STRAIGHT_FRAMES}f settle -> MQTT steer)")


def cancel_cruise() -> None:
    if not config.cruise_active:
        return
    config.cruise_active = False
    config.cruise_phase = "vision"
    config.controller_remote_steer_only = config.cruise_prev_remote_steer_only
    stop_base()
    print("CRUISE: cancelled")
