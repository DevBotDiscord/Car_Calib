"""Unit tests for optional input-device helpers."""

from scripts.input_device_helpers import open_optional_input_device


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
