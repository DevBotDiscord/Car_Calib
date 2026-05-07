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


