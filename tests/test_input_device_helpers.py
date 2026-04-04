"""Unit tests for optional input-device helpers."""

from scripts.input_device_helpers import (
    find_optional_abs_input_device,
    open_optional_input_device,
)


def test_open_optional_input_device_returns_none_when_missing() -> None:
    logs: list[str] = []

    def missing_device_factory(path: str) -> object:
        raise FileNotFoundError(path)

    device = open_optional_input_device(
        "/dev/input/by-id/missing-kbd",
        log=logs.append,
        device_factory=missing_device_factory,
    )

    assert device is None
    assert logs == [
        "INPUT: device not found, skipping local input: /dev/input/by-id/missing-kbd"
    ]


def test_open_optional_input_device_returns_device_when_available() -> None:
    logs: list[str] = []
    fake_device = object()

    def ok_device_factory(path: str) -> object:
        assert path == "/dev/input/by-id/ok-kbd"
        return fake_device

    device = open_optional_input_device(
        "/dev/input/by-id/ok-kbd",
        log=logs.append,
        device_factory=ok_device_factory,
    )

    assert device is fake_device
    assert logs == []


class FakeAbsDevice:
    def __init__(self, name: str, caps: dict[int, object]) -> None:
        self.name = name
        self._caps = caps
        self.closed = False

    def capabilities(self) -> dict[int, object]:
        return self._caps

    def close(self) -> None:
        self.closed = True


def test_find_optional_abs_input_device_matches_name_hint() -> None:
    logs: list[str] = []
    keyboard = FakeAbsDevice("USB Keyboard", {3: object()})
    gamepad = FakeAbsDevice("Edra Controller", {3: object()})
    devices = {
        "/dev/input/event0": keyboard,
        "/dev/input/event1": gamepad,
    }

    found = find_optional_abs_input_device(
        "",
        log=logs.append,
        device_factory=lambda path: devices[path],
        list_devices_fn=lambda: list(devices.keys()),
        name_hints=("edra", "controller"),
        ev_abs_code=3,
    )

    assert found is gamepad
    assert keyboard.closed is True
    assert gamepad.closed is False
    assert logs == []


def test_find_optional_abs_input_device_logs_when_no_match() -> None:
    logs: list[str] = []
    joystick = FakeAbsDevice("Unknown Joystick", {3: object()})

    found = find_optional_abs_input_device(
        "",
        log=logs.append,
        device_factory=lambda path: joystick,
        list_devices_fn=lambda: ["/dev/input/event2"],
        name_hints=("edra",),
        ev_abs_code=3,
    )

    assert found is None
    assert joystick.closed is True
    assert logs == [
        "INPUT: no matching controller found, skipping local controller. "
        "Detected EV_ABS devices: Unknown Joystick (/dev/input/event2)"
    ]
