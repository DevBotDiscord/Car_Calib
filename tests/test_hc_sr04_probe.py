from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


_SCRIPT = Path(__file__).parents[1] / "scripts" / "test_hc_sr04.py"
_SPEC = importlib.util.spec_from_file_location("test_hc_sr04_script", _SCRIPT)
assert _SPEC is not None and _SPEC.loader is not None
_MODULE = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = _MODULE
_SPEC.loader.exec_module(_MODULE)
HcSr04Probe = _MODULE.HcSr04Probe


class _Callback:
    def __init__(self, fn):
        self.fn = fn
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class _Pi:
    def __init__(self, pulse_us: int):
        self.pulse_us = pulse_us
        self.callback_handle = None
        self.writes = []

    def set_mode(self, *_args):
        pass

    def set_pull_up_down(self, *_args):
        pass

    def write(self, pin, value):
        self.writes.append((pin, value))

    def callback(self, _pin, _edge, fn):
        self.callback_handle = _Callback(fn)
        return self.callback_handle

    def gpio_trigger(self, _pin, _length_us, _level):
        self.callback_handle.fn(24, 1, 1000)
        self.callback_handle.fn(24, 0, 1000 + self.pulse_us)


class _Pigpio:
    OUTPUT = 1
    INPUT = 0
    PUD_DOWN = 0
    EITHER_EDGE = 2

    @staticmethod
    def tickDiff(start, end):
        return (end - start) & 0xFFFFFFFF


def test_probe_returns_distance_and_cleans_up_callback():
    pi = _Pi(pulse_us=2000)
    probe = HcSr04Probe(pi, _Pigpio, trig_pin=23, echo_pin=24)

    measurement = probe.measure(timeout_s=0.01)

    assert measurement is not None
    assert round(measurement.distance_cm, 1) == 34.3
    assert measurement.pulse_us == 2000
    probe.close()
    assert pi.callback_handle.cancelled
    assert pi.writes[-1] == (23, 0)


def test_probe_rejects_out_of_range_echo():
    pi = _Pi(pulse_us=100)
    probe = HcSr04Probe(pi, _Pigpio, trig_pin=23, echo_pin=24)

    assert probe.measure(timeout_s=0.01) is None
