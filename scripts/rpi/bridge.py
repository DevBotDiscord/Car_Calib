#!/usr/bin/env python3
"""RPi MQTT actuator bridge — subscribes to MQTT, writes GPIO (servo, base, relay).

No gamepad, keyboard, IMU, or control logic. Pure MQTT → hardware executor.
"""

from __future__ import annotations

import signal
import time

from . import config
from .base import stop_base
from .steering import apply_steering, release_servo
from .mqtt_client import close_mqtt, publish_status, setup_mqtt


def setup_gpio() -> None:
    pi = config.pigpio
    config.gpio = pi.pi(config.PIGPIO_HOST, config.PIGPIO_PORT)
    if not config.gpio.connected:
        raise RuntimeError(
            f"Cannot connect to pigpio at {config.PIGPIO_HOST}:{config.PIGPIO_PORT}. "
            "Start pigpiod first: sudo systemctl enable --now pigpiod"
        )
    for pin in (config.OUT1, config.OUT2, config.OUT3, config.RELAY_PIN):
        config.gpio.set_mode(pin, pi.OUTPUT)
    config.gpio.write(config.RELAY_PIN, 0)  # relay off at startup


def setup_servo_output() -> None:
    pass  # servo driven directly via pigpio


def cleanup() -> None:
    try:
        stop_base()
    except Exception:
        pass
    try:
        release_servo("CLEANUP")
    except Exception:
        pass
    close_mqtt()
    try:
        if config.gpio is not None:
            config.gpio.stop()
    except Exception:
        pass


def _signal_handler(sig, frame) -> None:
    del sig, frame
    config.running = False


def _print_banner() -> None:
    print("=== RPI MQTT ACTUATOR BRIDGE ===")
    print(f"MQTT broker: {config.MQTT_BROKER_HOST}:{config.MQTT_BROKER_PORT}")
    print(f"  servo:  {config.MQTT_SERVO_TOPIC}")
    print(f"  base:   {config.MQTT_BASE_COMMAND_TOPIC}")
    print(f"  relay:  {config.MQTT_RELAY_TOPIC}")
    print(f"pigpio: {config.PIGPIO_HOST}:{config.PIGPIO_PORT}")
    print(f"Servo pin: {config.SERVO_PIN}  Base pins: {config.OUT1},{config.OUT2},{config.OUT3}  Relay pin: {config.RELAY_PIN}")
    print(f"Steering center={config.CENTER_ANGLE:.1f} deg  limits=[{config.LEFT_LIMIT:.1f}, {config.RIGHT_LIMIT:.1f}]")
    print(f"Release idle={config.SERVO_RELEASE_IDLE}  Hold last={config.REMOTE_SERVO_HOLD_LAST}")
    print("")


def main() -> None:
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    setup_gpio()
    setup_servo_output()
    setup_mqtt()

    _print_banner()

    if config.SERVO_RELEASE_IDLE:
        release_servo("BOOT")
    else:
        apply_steering(config.CENTER_ANGLE, "BOOT")
    stop_base()

    # MQTT network I/O runs in background thread (loop_start).
    # Main thread idles with periodic heartbeat.
    try:
        while config.running:
            time.sleep(10)
            publish_status("online")
    except KeyboardInterrupt:
        print("\nEXIT: Ctrl+C")
    finally:
        print("Cleaning up...")
        cleanup()
        print("Done.")


if __name__ == "__main__":
    main()
