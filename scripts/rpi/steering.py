"""Servo steering: direct pigpio pulse, deadband, remote-angle mapping."""

from __future__ import annotations

import json
import time

from . import config
from .config import clamp_angle as _clamp_angle, _angle_within_limits


def _angle_to_pulse_us(angle: float) -> int:
    clamped = _clamp_angle(angle, config.LEFT_LIMIT, config.RIGHT_LIMIT)
    span = config.RIGHT_LIMIT - config.LEFT_LIMIT
    if span == 0:
        return config.SERVO_MIN_PULSE_US
    ratio = (clamped - config.LEFT_LIMIT) / span
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
    print(f"STEER[{source}]: {config.steer_angle:.1f} deg | CENTER: {config.CENTER_ANGLE:.1f} deg")
    config.last_steer_angle = config.steer_angle
    config.last_steer_source = source


def release_servo(reason: str = "IDLE") -> None:
    _servo_off()
    print(f"STEER[RELEASE]: servo PWM off ({reason})")


def steer_center(source: str) -> None:
    apply_steering(config.CENTER_ANGLE, source)


def steer_right_step(source: str) -> None:
    apply_steering(config.steer_angle - config.STEP, source)


def steer_left_step(source: str) -> None:
    apply_steering(config.steer_angle + config.STEP, source)


def steer_from_gamepad_axis(axis_value: float, source: str) -> bool:
    if config.INVERT_STEER_AXIS:
        axis_value = -axis_value

    axis_value = _apply_deadzone(axis_value, config.GAMEPAD_STEER_DEADZONE)
    if axis_value == 0.0:
        return False

    if axis_value < 0:
        target = config.CENTER_ANGLE - axis_value * (config.RIGHT_LIMIT - config.CENTER_ANGLE)
    else:
        target = config.CENTER_ANGLE - axis_value * (config.CENTER_ANGLE - config.LEFT_LIMIT)
    apply_steering(target, source)
    return True


def adjust_center(delta: float, source: str) -> None:
    config.CENTER_ANGLE = _clamp_angle(config.CENTER_ANGLE + delta, config.LEFT_LIMIT, config.RIGHT_LIMIT)
    print(f"CENTER_ANGLE: {config.CENTER_ANGLE}")

    if source == "GAMEPAD":
        if config.controller_remote_steer_only:
            return
        steer_value = _apply_deadzone(config.axis_state.get(config.STEER_AXIS, 0.0), config.GAMEPAD_STEER_DEADZONE)
        if steer_value == 0.0:
            from .controls import activate_manual_override
            activate_manual_override(source, time.monotonic())
            steer_center(source)


def remote_control_active(now: float) -> bool:
    if config.REMOTE_SERVO_HOLD_LAST:
        return config.remote_servo_angle is not None
    return config.remote_servo_angle is not None and (now - config.remote_servo_updated_at) <= config.REMOTE_SERVO_TIMEOUT


def _apply_deadzone(value: float, deadzone: float) -> float:
    return 0.0 if abs(value) < deadzone else value


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

    return _clamp_angle(map_remote_angle(raw_angle), config.LEFT_LIMIT, config.RIGHT_LIMIT)
