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

    # Simulate a prior manual steer command.
    controller._steer_angle = controller.right_limit  # type: ignore[attr-defined]
    decision = controller.process(now=1.0)

    assert decision.steer_source == "MANUAL-CENTER"
    assert decision.steer_angle == controller._center_angle  # type: ignore[attr-defined]


def test_remote_mode_allows_vision_steer() -> None:
    handler = _FakeHandler()
    controller = InputController(handler)
    controller.controller_remote_steer_only = True

    decision = controller.process(now=1.0)

    assert decision.steer_source == "VISION"
    assert decision.steer_angle is None
