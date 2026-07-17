#!/usr/bin/env python3
"""Standalone HC-SR04 wiring test for a Raspberry Pi running pigpiod.

Default BCM wiring:
    HC-SR04 VCC   -> 5V (physical pin 2 or 4)
    HC-SR04 GND   -> GND (physical pin 6)
    HC-SR04 TRIG  -> BCM23 (physical pin 16)
    HC-SR04 ECHO  -> 1k resistor -> BCM24 (physical pin 18)
                         BCM24 -> 2k resistor -> GND

Never connect the 5V ECHO signal directly to a Raspberry Pi GPIO.

Run on the Raspberry Pi (or in the control-direct container):
    python3 scripts/test_hc_sr04.py
    python3 scripts/test_hc_sr04.py --count 0
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class Measurement:
    distance_cm: float
    pulse_us: int


class HcSr04Probe:
    """One-at-a-time HC-SR04 probe; no motor, servo, or safety control."""

    def __init__(self, pi: Any, pigpio_module: Any, *, trig_pin: int, echo_pin: int) -> None:
        self._pi = pi
        self._pigpio = pigpio_module
        self._trig_pin = trig_pin
        self._echo_pin = echo_pin
        self._lock = threading.Lock()
        self._echo_event = threading.Event()
        self._rise_tick: int | None = None
        self._measurement: Measurement | None = None

        self._pi.set_mode(self._trig_pin, self._pigpio.OUTPUT)
        self._pi.write(self._trig_pin, 0)
        self._pi.set_mode(self._echo_pin, self._pigpio.INPUT)
        self._pi.set_pull_up_down(self._echo_pin, self._pigpio.PUD_DOWN)
        self._callback = self._pi.callback(
            self._echo_pin,
            self._pigpio.EITHER_EDGE,
            self._on_echo,
        )

    def measure(self, timeout_s: float) -> Measurement | None:
        """Trigger once and wait only in this dedicated test script for Echo."""
        with self._lock:
            self._rise_tick = None
            self._measurement = None
            self._echo_event.clear()
        self._pi.gpio_trigger(self._trig_pin, 10, 1)
        if not self._echo_event.wait(timeout_s):
            return None
        with self._lock:
            return self._measurement

    def close(self) -> None:
        try:
            self._callback.cancel()
        finally:
            self._pi.write(self._trig_pin, 0)

    def _on_echo(self, _gpio: int, level: int, tick: int) -> None:
        with self._lock:
            if level == 1:
                self._rise_tick = int(tick)
                return
            if level != 0 or self._rise_tick is None:
                return
            pulse_us = int(self._pigpio.tickDiff(self._rise_tick, int(tick)))
            self._rise_tick = None
            # Distance = pulse duration × speed of sound / 2.
            # 0.01715 cm/us corresponds to 343 m/s at approximately 20 °C.
            distance_cm = pulse_us * 0.01715
            if 2.0 <= distance_cm <= 400.0:
                self._measurement = Measurement(distance_cm, pulse_us)
            self._echo_event.set()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Test HC-SR04 wiring through pigpiod")
    parser.add_argument("--trig", type=int, default=23, help="TRIG BCM pin (default: 23)")
    parser.add_argument("--echo", type=int, default=24, help="ECHO BCM pin (default: 24)")
    parser.add_argument("--host", default="127.0.0.1", help="pigpiod host")
    parser.add_argument("--port", type=int, default=8888, help="pigpiod port")
    parser.add_argument("--interval", type=float, default=0.10, help="seconds between pulses")
    parser.add_argument("--timeout", type=float, default=0.040, help="seconds to wait for Echo")
    parser.add_argument("--count", type=int, default=20, help="sample count; 0 runs until Ctrl+C")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.trig < 0 or args.echo < 0 or args.trig == args.echo:
        print("TRIG and ECHO must be distinct non-negative BCM pins", file=sys.stderr)
        return 2
    if args.interval < 0.06 or args.timeout <= 0.0 or args.timeout >= args.interval:
        print("Use --interval >= 0.06 and 0 < --timeout < --interval", file=sys.stderr)
        return 2
    if args.count < 0:
        print("--count must be zero or positive", file=sys.stderr)
        return 2

    try:
        import pigpio  # type: ignore[import-untyped]
    except ImportError:
        print("pigpio is not installed. Install requirements-control-direct.txt first.", file=sys.stderr)
        return 2

    pi = pigpio.pi(args.host, args.port)
    if not pi.connected:
        print(
            f"Cannot connect to pigpiod at {args.host}:{args.port}. "
            "Start pigpiod, then retry.",
            file=sys.stderr,
        )
        return 2

    probe = HcSr04Probe(pi, pigpio, trig_pin=args.trig, echo_pin=args.echo)
    print(
        f"HC-SR04 test: TRIG=BCM{args.trig}, ECHO=BCM{args.echo}, "
        f"interval={args.interval:.2f}s. Press Ctrl+C to stop."
    )
    try:
        sample = 0
        while args.count == 0 or sample < args.count:
            sample += 1
            started = time.monotonic()
            measurement = probe.measure(args.timeout)
            if measurement is None:
                print(f"[{sample:03d}] timeout (> {args.timeout * 1000:.0f} ms) - check Echo/divider/wiring")
            else:
                print(
                    f"[{sample:03d}] {measurement.distance_cm:6.1f} cm "
                    f"({measurement.pulse_us} us)"
                )
            remaining = args.interval - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        probe.close()
        pi.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
