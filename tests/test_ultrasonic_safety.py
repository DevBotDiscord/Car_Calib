from __future__ import annotations

from runtime.ultrasonic_safety import SonarConfig, UltrasonicSafety


class _Callback:
    def __init__(self, fn):
        self.fn = fn
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class _Pi:
    connected = True

    def __init__(self):
        self.callback_handle = None
        self.triggers = []
        self.writes = []
        self.stopped = False

    def set_mode(self, *_args):
        pass

    def set_pull_up_down(self, *_args):
        pass

    def write(self, pin, value):
        self.writes.append((pin, value))

    def callback(self, _pin, _edge, fn):
        self.callback_handle = _Callback(fn)
        return self.callback_handle

    def gpio_trigger(self, pin, length_us, level):
        self.triggers.append((pin, length_us, level))

    def stop(self):
        self.stopped = True


class _Pigpio:
    OUTPUT = 1
    INPUT = 0
    PUD_DOWN = 0
    EITHER_EDGE = 2

    @staticmethod
    def tickDiff(start, end):
        return (end - start) & 0xFFFFFFFF


def _measure(monitor, pi, clock, start, pulse_us):
    monitor.update(now=start)
    clock[0] = start + 0.001
    pi.callback_handle.fn(24, 1, 1000)
    pi.callback_handle.fn(24, 0, 1000 + pulse_us)


def test_hysteresis_blocks_near_and_clears_after_stable_distance():
    clock = [0.0]
    pi = _Pi()
    monitor = UltrasonicSafety(
        SonarConfig(enabled=True, stop_samples=2, clear_samples=2),
        pi=pi,
        pigpio_module=_Pigpio,
        clock=lambda: clock[0],
    )

    assert monitor.status(now=0.0).blocked  # fail-safe until consecutive clear samples
    _measure(monitor, pi, clock, 0.0, 2500)  # 42.9 cm
    _measure(monitor, pi, clock, 0.11, 2500)
    assert not monitor.status(now=0.12).blocked

    _measure(monitor, pi, clock, 0.22, 1200)  # 20.6 cm
    assert not monitor.status(now=0.23).blocked
    _measure(monitor, pi, clock, 0.33, 1200)
    status = monitor.status(now=0.34)
    assert status.blocked
    assert status.reason == "distance"
    assert round(status.distance_cm or 0.0, 1) == 20.6


def test_echo_timeout_is_fail_safe_and_close_releases_callback():
    clock = [0.0]
    pi = _Pi()
    monitor = UltrasonicSafety(
        SonarConfig(enabled=True, fail_safe_timeout_s=0.0),
        pi=pi,
        pigpio_module=_Pigpio,
        clock=lambda: clock[0],
    )

    monitor.update(now=0.0)
    status = monitor.update(now=0.05)
    assert status.blocked
    assert status.reason == "echo_timeout"
    assert status.invalid_samples == 1
    assert pi.triggers == [(23, 10, 1)]

    monitor.close()
    assert pi.callback_handle.cancelled
    assert not pi.stopped  # injected pigpio client remains owned by its caller


def test_disabled_sonar_never_opens_pigpio_or_blocks_control():
    pi = _Pi()
    monitor = UltrasonicSafety(
        SonarConfig(enabled=False),
        pi=pi,
        pigpio_module=_Pigpio,
    )

    status = monitor.update(now=0.0)
    assert not status.enabled
    assert not status.blocked
    assert pi.callback_handle is None
