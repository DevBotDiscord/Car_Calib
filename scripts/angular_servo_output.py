"""AngularServo-based output wrapper for Raspberry Pi servo bridges."""

from __future__ import annotations

from typing import Any, Callable, Protocol

try:
    from .servo_bridge_common import angle_to_pulse_us, clamp_angle
except ImportError:  # pragma: no cover - direct script execution on Raspberry Pi
    from servo_bridge_common import angle_to_pulse_us, clamp_angle  # type: ignore


class ServoOutput(Protocol):
    """Small protocol for servo output helpers used by the bridge."""

    attached: bool

    def set_servo(self, angle: float, source: str) -> float: ...

    def detach_servo(self, reason: str = "IDLE") -> None: ...

    def set_center_angle(self, center_angle: float) -> None: ...


def _default_pin_factory_factory(*, host: str, port: int) -> Any:
    try:
        from gpiozero.pins.pigpio import PiGPIOFactory
    except ImportError as exc:  # pragma: no cover - runtime dependency on Raspberry Pi
        raise RuntimeError(
            "gpiozero pigpio support is required for TCP servo bridge output. "
            "Install gpiozero and use PiGPIOFactory."
        ) from exc

    try:
        return PiGPIOFactory(host=host, port=port)
    except Exception as exc:  # pragma: no cover - runtime dependency on Raspberry Pi
        raise RuntimeError(
            f"Cannot connect PiGPIOFactory to pigpio at {host}:{port}. "
            "Start pigpiod first, for example: sudo systemctl enable --now pigpiod"
        ) from exc


def _default_servo_factory(pin: int, **kwargs: Any) -> Any:
    try:
        from gpiozero import AngularServo
    except ImportError as exc:  # pragma: no cover - runtime dependency on Raspberry Pi
        raise RuntimeError(
            "gpiozero is required for TCP servo bridge output. Install gpiozero first."
        ) from exc

    return AngularServo(pin, **kwargs)


class AngularServoOutput:
    """Wrap AngularServo with bridge-specific logging and duplicate filtering."""

    def __init__(
        self,
        *,
        servo_pin: int,
        left_limit: float,
        right_limit: float,
        servo_min_pulse_us: int,
        servo_max_pulse_us: int,
        pigpio_host: str,
        pigpio_port: int,
        center_angle: float,
        log: Callable[[str], None],
        servo_factory: Callable[..., Any] | None = None,
        pin_factory_factory: Callable[..., Any] | None = None,
    ) -> None:
        self._servo_pin = servo_pin
        self._left_limit = left_limit
        self._right_limit = right_limit
        self._servo_min_pulse_us = servo_min_pulse_us
        self._servo_max_pulse_us = servo_max_pulse_us
        self._center_angle = center_angle
        self._log = log
        self._attached = False
        self._last_angle: float | None = None
        self._last_source: str | None = None

        pin_factory_ctor = pin_factory_factory or _default_pin_factory_factory
        servo_ctor = servo_factory or _default_servo_factory

        self._pin_factory = pin_factory_ctor(host=pigpio_host, port=pigpio_port)
        self._servo = servo_ctor(
            servo_pin,
            min_angle=-90,
            max_angle=90,
            min_pulse_width=servo_min_pulse_us / 1_000_000,
            max_pulse_width=servo_max_pulse_us / 1_000_000,
            pin_factory=self._pin_factory,
        )

    @property
    def attached(self) -> bool:
        return self._attached

    def set_center_angle(self, center_angle: float) -> None:
        self._center_angle = center_angle

    def set_servo(self, angle: float, source: str) -> float:
        target = clamp_angle(angle, self._left_limit, self._right_limit)

        if self._attached and target == self._last_angle and source == self._last_source:
            return target

        self._servo.angle = target
        self._attached = True
        self._last_angle = target
        self._last_source = source

        pulse_us = angle_to_pulse_us(
            target,
            self._left_limit,
            self._right_limit,
            self._servo_min_pulse_us,
            self._servo_max_pulse_us,
        )
        self._log(
            f"STEER[{source}]: {target:.1f} deg | CENTER: {self._center_angle:.1f} deg | PULSE: {pulse_us} us"
        )
        return target

    def detach_servo(self, reason: str = "IDLE") -> None:
        if not self._attached:
            return

        self._servo.detach()
        self._attached = False
        self._log(f"STEER[RELEASE]: servo PWM off ({reason})")

    def close(self) -> None:
        try:
            self.detach_servo("CLOSE")
        except Exception:
            pass

        servo_close = getattr(self._servo, "close", None)
        if callable(servo_close):
            try:
                servo_close()
            except Exception:
                pass

        pin_factory_close = getattr(self._pin_factory, "close", None)
        if callable(pin_factory_close):
            try:
                pin_factory_close()
            except Exception:
                pass


def apply_boot_servo_behavior(
    servo_output: ServoOutput,
    *,
    release_idle: bool,
    center_angle: float,
) -> None:
    if release_idle:
        servo_output.detach_servo("BOOT")
    else:
        servo_output.set_servo(center_angle, "BOOT")


def apply_idle_servo_behavior(
    servo_output: ServoOutput,
    *,
    release_idle: bool,
    center_angle: float,
) -> None:
    if release_idle:
        servo_output.detach_servo("IDLE")
    else:
        servo_output.set_servo(center_angle, "IDLE")
