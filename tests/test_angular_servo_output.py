"""Unit tests for AngularServo-based bridge output helpers."""

from scripts.angular_servo_output import (
    AngularServoOutput,
    apply_boot_servo_behavior,
    apply_idle_servo_behavior,
)


class FakePinFactory:
    def __init__(self) -> None:
        self.closed = False

    def close(self) -> None:
        self.closed = True


class FakePinFactoryBuilder:
    def __init__(self) -> None:
        self.calls: list[tuple[str, int]] = []
        self.instance = FakePinFactory()

    def __call__(self, *, host: str, port: int) -> FakePinFactory:
        self.calls.append((host, port))
        return self.instance


class FakeServo:
    def __init__(self) -> None:
        self.angle_writes: list[float] = []
        self.detach_calls = 0
        self.close_calls = 0

    @property
    def angle(self) -> float | None:
        return self.angle_writes[-1] if self.angle_writes else None

    @angle.setter
    def angle(self, value: float) -> None:
        self.angle_writes.append(value)

    def detach(self) -> None:
        self.detach_calls += 1

    def close(self) -> None:
        self.close_calls += 1


class FakeServoBuilder:
    def __init__(self) -> None:
        self.calls: list[tuple[int, dict[str, object]]] = []
        self.instance = FakeServo()

    def __call__(self, pin: int, **kwargs: object) -> FakeServo:
        self.calls.append((pin, kwargs))
        return self.instance


class FakeServoOutput:
    def __init__(self) -> None:
        self.attached = True
        self.calls: list[tuple[str, object]] = []

    def set_servo(self, angle: float, source: str) -> float:
        self.calls.append(("set", angle, source))
        self.attached = True
        return angle

    def detach_servo(self, reason: str = "IDLE") -> None:
        self.calls.append(("detach", reason))
        self.attached = False

    def set_center_angle(self, center_angle: float) -> None:
        self.calls.append(("center", center_angle))


def build_output(logs: list[str]) -> tuple[AngularServoOutput, FakePinFactoryBuilder, FakeServoBuilder]:
    pin_factory_builder = FakePinFactoryBuilder()
    servo_builder = FakeServoBuilder()
    output = AngularServoOutput(
        servo_pin=19,
        left_limit=-65.0,
        right_limit=60.0,
        servo_min_pulse_us=1000,
        servo_max_pulse_us=2000,
        pigpio_host="127.0.0.1",
        pigpio_port=8888,
        center_angle=-8.0,
        log=logs.append,
        servo_factory=servo_builder,
        pin_factory_factory=pin_factory_builder,
    )
    return output, pin_factory_builder, servo_builder


def test_set_servo_initializes_factories_and_clamps_angle() -> None:
    logs: list[str] = []
    output, pin_factory_builder, servo_builder = build_output(logs)

    result = output.set_servo(120.0, "REMOTE")

    assert result == 60.0
    assert pin_factory_builder.calls == [("127.0.0.1", 8888)]
    assert servo_builder.calls[0][0] == 19
    assert servo_builder.calls[0][1]["min_angle"] == -90
    assert servo_builder.calls[0][1]["max_angle"] == 90
    assert servo_builder.calls[0][1]["min_pulse_width"] == 0.001
    assert servo_builder.calls[0][1]["max_pulse_width"] == 0.002
    assert servo_builder.instance.angle_writes == [60.0]
    assert logs == ["STEER[REMOTE]: 60.0 deg | CENTER: -8.0 deg | PULSE: 2000 us"]


def test_set_servo_skips_duplicate_angle_and_source() -> None:
    logs: list[str] = []
    output, _, servo_builder = build_output(logs)

    output.set_servo(-8.0, "REMOTE")
    output.set_servo(-8.0, "REMOTE")
    output.set_servo(-8.0, "KEYBOARD")

    assert servo_builder.instance.angle_writes == [-8.0, -8.0]
    assert logs == [
        "STEER[REMOTE]: -8.0 deg | CENTER: -8.0 deg | PULSE: 1456 us",
        "STEER[KEYBOARD]: -8.0 deg | CENTER: -8.0 deg | PULSE: 1456 us",
    ]


def test_detach_servo_and_center_updates_are_tracked() -> None:
    logs: list[str] = []
    output, pin_factory_builder, servo_builder = build_output(logs)

    output.set_center_angle(-5.0)
    output.set_servo(-5.0, "BOOT")
    output.detach_servo("IDLE")
    output.detach_servo("IDLE")
    output.close()

    assert servo_builder.instance.detach_calls == 1
    assert servo_builder.instance.close_calls == 1
    assert pin_factory_builder.instance.closed is True
    assert logs == [
        "STEER[BOOT]: -5.0 deg | CENTER: -5.0 deg | PULSE: 1480 us",
        "STEER[RELEASE]: servo PWM off (IDLE)",
    ]


def test_boot_and_idle_behaviors_respect_release_idle_flag() -> None:
    output = FakeServoOutput()

    apply_boot_servo_behavior(output, release_idle=False, center_angle=-8.0)
    apply_idle_servo_behavior(output, release_idle=True, center_angle=-8.0)

    assert output.calls == [
        ("set", -8.0, "BOOT"),
        ("detach", "IDLE"),
    ]
