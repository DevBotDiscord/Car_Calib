"""Tests for dashboard manual override joystick mapping."""

from __future__ import annotations

from runtime.manual_override import (
    ManualOverrideConfig,
    ManualOverrideController,
    ManualOverrideError,
)

def _assert_raises(exc_type: type[BaseException], fn) -> None:
    try:
        fn()
    except exc_type:
        return
    raise AssertionError(f"expected {exc_type.__name__}")


def test_manual_override_rejects_bad_payloads():
    controller = ManualOverrideController(ManualOverrideConfig())

    _assert_raises(ManualOverrideError, lambda: controller.submit({}, now=1.0))
    _assert_raises(
        ManualOverrideError,
        lambda: controller.submit({"active": True, "drive": "nan", "steer": 0}, now=1.0),
    )
    _assert_raises(
        ManualOverrideError,
        lambda: controller.submit({"active": True, "drive": 0, "steer": True}, now=1.0),
    )


def test_manual_override_clamps_and_maps_axes_to_commands():
    controller = ManualOverrideController(
        ManualOverrideConfig(center_angle=62.0, max_steer=45.0, deadzone=0.15)
    )

    controller.submit({"active": True, "drive": 2.0, "steer": -2.0}, now=1.0)
    decision = controller.evaluate(now=1.1)

    assert decision.active is True
    assert decision.drive == 1.0
    assert decision.steer == -1.0
    assert decision.base_command == "FORWARD"
    assert decision.servo_angle == 107.0

    controller.submit({"active": True, "drive": -0.8, "steer": 1.0}, now=2.0)
    decision = controller.evaluate(now=2.1, object_near=True)
    assert decision.base_command == "BACKWARD"
    assert decision.servo_angle == 17.0

    controller.submit({"active": True, "drive": 0.8, "steer": 0.0}, now=3.0)
    decision = controller.evaluate(now=3.1, object_near=True)
    assert decision.base_command == "STOP"
    assert decision.blocked_reason == "object_near"


def test_manual_override_timeout_stops_and_releases():
    controller = ManualOverrideController(
        ManualOverrideConfig(timeout_s=0.3, center_angle=62.0, max_steer=45.0)
    )
    controller.submit({"active": True, "drive": 0.8, "steer": 0.3}, now=1.0)

    decision = controller.evaluate(now=1.31)

    assert decision.active is False
    assert decision.timed_out is True
    assert decision.base_command == "STOP"
    assert decision.release_servo is True
    assert decision.blocked_reason == "timeout"


def test_manual_override_release_stops_and_releases_without_timeout():
    controller = ManualOverrideController(
        ManualOverrideConfig(timeout_s=0.3, center_angle=62.0, max_steer=45.0)
    )
    controller.submit({"active": True, "drive": 0.8, "steer": 0.3}, now=1.0)
    controller.submit({"active": False, "drive": 0.0, "steer": 0.0}, now=1.1)

    decision = controller.evaluate(now=1.1)

    assert decision.active is False
    assert decision.timed_out is False
    assert decision.base_command == "STOP"
    assert decision.release_servo is True
