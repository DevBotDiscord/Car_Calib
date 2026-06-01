"""Unit tests for InputController steering mode arbitration."""

from __future__ import annotations

from scripts.rpi.controls import InputController


class _FakeHandler:
    def __init__(self) -> None:
        self.axis_state: dict[int, float] = {}
        self.pressed_buttons: set[int] = set()
        self.pressed_keys: set[str] = set()


def test_manual_mode_auto_centers_when_stick_neutral() -> None:
    handler = _FakeHandler()
    controller = InputController(handler)
    controller.controller_remote_steer_only = False

    # Simulate a prior manual steer command followed by stick release.
    controller._steer_angle = controller.right_limit  # type: ignore[attr-defined]
    controller._steering_active = True  # type: ignore[attr-defined]

    # First tick with stick neutral: gamepad path marks recenter pending,
    # then the recenter branch fires and snaps angle back to center.
    decision = controller.process(now=1.0)

    assert decision.manual_steer is True
    assert decision.steer_angle == controller._center_angle  # type: ignore[attr-defined]


def test_remote_mode_yields_vision_path() -> None:
    handler = _FakeHandler()
    controller = InputController(handler)
    controller.controller_remote_steer_only = True

    decision = controller.process(now=1.0)

    # Remote mode → controller does NOT take manual authority; MQTT vision
    # path drives the servo on subsequent ticks.
    assert decision.manual_steer is False
