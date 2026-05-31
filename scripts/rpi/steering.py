"""Servo steering: direct pigpio pulse, deadband, remote-angle mapping (RPi actuator side)."""

from __future__ import annotations

import json

from . import config
from .config import _angle_within_limits, _clamp as _clamp_angle
from .logging_utils import get_logger

logger = get_logger("steering")


# Absolute angle scale used to map degrees -> servo pulse, matching the
# standalone controlv8 reference: [-90 deg, +90 deg] -> [SERVO_MIN_PULSE_US,
# SERVO_MAX_PULSE_US]. The logical [LEFT_LIMIT, RIGHT_LIMIT] window only
# clamps the input; the pulse mapping itself is absolute so the same
# numeric angle produces the same physical servo position regardless of
# how the trim limits are configured.
_SERVO_ABS_MIN_DEG = -90.0
_SERVO_ABS_MAX_DEG = 90.0


def _angle_to_pulse_us(angle: float) -> int:
    clamped = _clamp_angle(angle, config.LEFT_LIMIT, config.RIGHT_LIMIT)
    span = _SERVO_ABS_MAX_DEG - _SERVO_ABS_MIN_DEG
    if span == 0:
        return config.SERVO_MIN_PULSE_US
    ratio = (clamped - _SERVO_ABS_MIN_DEG) / span
    if ratio < 0.0:
        ratio = 0.0
    elif ratio > 1.0:
        ratio = 1.0
    return int(config.SERVO_MIN_PULSE_US + ratio * (config.SERVO_MAX_PULSE_US - config.SERVO_MIN_PULSE_US))


def _write_servo(angle: float) -> None:
    if config.gpio is None:
        raise RuntimeError("pigpio is not initialized.")
    config.gpio.set_servo_pulsewidth(config.SERVO_PIN, _angle_to_pulse_us(angle))


def _servo_off() -> None:
    if config.gpio is not None:
        config.gpio.set_servo_pulsewidth(config.SERVO_PIN, 0)


def apply_steering(target_angle: float, source: str) -> None:
    target = _clamp_angle(target_angle, config.LEFT_LIMIT, config.RIGHT_LIMIT)

    if (
        config.last_steer_angle is not None
        and abs(target - config.last_steer_angle) < config.STEER_DEADBAND_DEG
        and source == config.last_steer_source
    ):
        return

    config.steer_angle = target
    _write_servo(target)
    logger.debug(
        "[STEER] %s angle=%.1f center=%.1f",
        source,
        config.steer_angle,
        config.CENTER_ANGLE,
    )
    config.last_steer_angle = config.steer_angle
    config.last_steer_source = source


def release_servo(reason: str = "IDLE") -> None:
    _servo_off()
    logger.info("[STEER][RELEASE] reason=%s", reason)


def steer_center(source: str) -> None:
    apply_steering(config.CENTER_ANGLE, source)


# ---------------------------------------------------------------------------
# remote angle mapping (MQTT → physical)
# ---------------------------------------------------------------------------

def map_remote_angle(angle: float) -> float:
    angle = _clamp_angle(angle, config.REMOTE_INPUT_MIN_ANGLE, config.REMOTE_INPUT_MAX_ANGLE)

    if angle <= config.REMOTE_INPUT_CENTER_ANGLE:
        remote_span = config.REMOTE_INPUT_CENTER_ANGLE - config.REMOTE_INPUT_MIN_ANGLE
        if remote_span <= 0:
            return config.CENTER_ANGLE
        ratio = (angle - config.REMOTE_INPUT_CENTER_ANGLE) / remote_span
        return config.CENTER_ANGLE + ratio * (config.CENTER_ANGLE - config.LEFT_LIMIT)

    remote_span = config.REMOTE_INPUT_MAX_ANGLE - config.REMOTE_INPUT_CENTER_ANGLE
    if remote_span <= 0:
        return config.CENTER_ANGLE
    ratio = (angle - config.REMOTE_INPUT_CENTER_ANGLE) / remote_span
    return config.CENTER_ANGLE + ratio * (config.RIGHT_LIMIT - config.CENTER_ANGLE)


def resolve_remote_servo_angle(payload_text: str) -> float:
    payload_text = payload_text.strip()
    if not payload_text:
        raise ValueError("Empty MQTT servo payload")

    if payload_text.startswith("{"):
        payload = json.loads(payload_text)
        command = payload.get("type", "angle")
        if command == "center":
            raw_angle = config.CENTER_ANGLE
        elif command == "angle":
            raw_angle = float(payload.get("angle", config.REMOTE_INPUT_CENTER_ANGLE))
        else:
            raise ValueError(f"Unsupported command: {command}")
    else:
        raw_angle = float(payload_text)

    if _angle_within_limits(raw_angle, config.LEFT_LIMIT, config.RIGHT_LIMIT):
        return _clamp_angle(raw_angle, config.LEFT_LIMIT, config.RIGHT_LIMIT)

    # Backward-compatible mapping path: only apply when payload is in the
    # legacy remote-input range (typically 60..120 around 90 center).
    if _angle_within_limits(
        raw_angle,
        config.REMOTE_INPUT_MIN_ANGLE,
        config.REMOTE_INPUT_MAX_ANGLE,
    ):
        return _clamp_angle(map_remote_angle(raw_angle), config.LEFT_LIMIT, config.RIGHT_LIMIT)

    # For signed-angle publishers, prefer direct clamp to physical range
    # instead of forcing legacy map that can bias toward one side.
    return _clamp_angle(raw_angle, config.LEFT_LIMIT, config.RIGHT_LIMIT)
