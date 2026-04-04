"""Unit tests for shared Raspberry Pi servo bridge helpers."""

from scripts.servo_bridge_common import angle_to_pulse_us, angle_within_limits, clamp_angle


def test_clamp_angle_preserves_center_with_descending_limits():
    assert clamp_angle(-8.0, 60.0, -30.0) == -8.0


def test_angle_within_limits_accepts_descending_limit_order():
    assert angle_within_limits(-8.0, 60.0, -30.0) is True
    assert angle_within_limits(-40.0, 60.0, -30.0) is False


def test_angle_to_pulse_us_supports_descending_limit_order():
    assert angle_to_pulse_us(60.0, 60.0, -30.0, 500, 2500) == 500
    assert angle_to_pulse_us(-30.0, 60.0, -30.0, 500, 2500) == 2500
    assert angle_to_pulse_us(-8.0, 60.0, -30.0, 500, 2500) == 2011
