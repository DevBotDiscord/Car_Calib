"""Shared angle helpers for Raspberry Pi servo bridge scripts."""

from __future__ import annotations


def angle_bounds(left_limit: float, right_limit: float) -> tuple[float, float]:
    return (left_limit, right_limit) if left_limit <= right_limit else (right_limit, left_limit)


def clamp_angle(value: float, left_limit: float, right_limit: float) -> float:
    lower, upper = angle_bounds(left_limit, right_limit)
    return max(lower, min(upper, value))


def angle_within_limits(value: float, left_limit: float, right_limit: float) -> bool:
    lower, upper = angle_bounds(left_limit, right_limit)
    return lower <= value <= upper


def angle_to_pulse_us(
    angle: float,
    left_limit: float,
    right_limit: float,
    min_pulse_us: int,
    max_pulse_us: int,
) -> int:
    """Map angle between limits to servo pulse width in microseconds."""
    lower, upper = angle_bounds(left_limit, right_limit)
    clamped = clamp_angle(angle, left_limit, right_limit)
    span = upper - lower
    if span == 0:
        return int(min_pulse_us)

    ratio = (clamped - lower) / span
    if left_limit > right_limit:
        ratio = 1.0 - ratio
    pulse = min_pulse_us + ratio * (max_pulse_us - min_pulse_us)
    return int(round(pulse))


