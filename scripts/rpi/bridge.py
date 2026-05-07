#!/usr/bin/env python3
"""RPi MQTT bridge entrypoint — keyboard, gamepad, MQTT, IMU, cruise.

Orchestrates all input sources and steers servo + base via direct pigpio.
"""

from __future__ import annotations

import signal
import time

from evdev import InputDevice, ecodes, list_devices

try:
    from input_device_helpers import find_optional_abs_input_device, open_optional_input_device
except ImportError:
    from scripts.input_device_helpers import (  # type: ignore[no-redef]
        find_optional_abs_input_device,
        open_optional_input_device,
    )

from . import config
from .base import stop_base, forward, backward
from .steering import apply_steering, release_servo, steer_center
from .imu import setup_imu
from .mqtt_client import setup_mqtt, close_mqtt
from .controls import process_controls, update_key_state, update_gamepad_button_state, update_gamepad_axis_state


def setup_gpio() -> None:
    pi = config.pigpio
    config.gpio = pi.pi(config.PIGPIO_HOST, config.PIGPIO_PORT)
    if not config.gpio.connected:
        raise RuntimeError(
            f"Cannot connect to pigpio at {config.PIGPIO_HOST}:{config.PIGPIO_PORT}. "
            "Start pigpiod first: sudo systemctl enable --now pigpiod"
        )
    for pin in (config.OUT1, config.OUT2, config.OUT3):
        config.gpio.set_mode(pin, pi.OUTPUT)


def setup_servo_output() -> None:
    pass  # servo driven directly via pigpio (_write_servo / _servo_off)


def setup_keyboard() -> None:
    config.keyboard = open_optional_input_device(
        config.KEYBOARD_DEVICE, log=print, device_factory=InputDevice,
    )


def setup_gamepad() -> None:
    config.gamepad = find_optional_abs_input_device(
        config.GAMEPAD_DEVICE, log=print, device_factory=InputDevice,
        list_devices_fn=list_devices, name_hints=config.GAMEPAD_NAME_HINTS,
        ev_abs_code=ecodes.EV_ABS,
    )
    if config.gamepad is not None:
        print(f"INPUT: controller ready: {config.gamepad.name} ({config.gamepad.path})")


def cleanup() -> None:
    try:
        stop_base()
    except Exception:
        pass
    try:
        release_servo("CLEANUP")
    except Exception:
        pass
    try:
        if config.keyboard is not None:
            config.keyboard.ungrab()
    except Exception:
        pass
    try:
        if config.gamepad is not None:
            config.gamepad.ungrab()
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
    print("=== RPI KEYBOARD + MQTT SERVO BRIDGE MODE ===")
    if config.keyboard is not None:
        print(f"Keyboard: {config.keyboard.path}")
    else:
        print(f"Keyboard: disabled (missing {config.KEYBOARD_DEVICE})")
    if config.gamepad is not None:
        print(f"Controller: {config.gamepad.name} ({config.gamepad.path})")
    else:
        print("Controller: disabled (not detected)")
    print(f"MQTT broker: {config.MQTT_BROKER_HOST}:{config.MQTT_BROKER_PORT}")
    print(f"MQTT servo topic: {config.MQTT_SERVO_TOPIC}")
    print(f"pigpio: {config.PIGPIO_HOST}:{config.PIGPIO_PORT}")
    print(f"Servo GPIO pin: {config.SERVO_PIN}")
    print("W = forward  S = backward  X = stop")
    print("A = steer left  D = steer right  C = center")
    print("L = lock  U = unlock  1/2 = adj center  Q = quit")
    print("Right stick X = steering  Left stick Y = base drive")
    print("Y = toggle MQTT steer  B = toggle IMU mode")
    print("A = stop | LB = lock | RB = unlock | START = quit")
    print(f"SELECT/BACK or {config.CRUISE_KEY} = cruise ({config.CRUISE_DURATION_S:.0f}s)")
    print("X or D-pad = adj center | Ctrl+C = emergency exit")
    print("")
    print(
        f"Steering center={config.CENTER_ANGLE:.1f}, left={config.LEFT_LIMIT:.1f}, right={config.RIGHT_LIMIT:.1f}, "
        f"step={config.STEP:.1f}, deadband={config.STEER_DEADBAND_DEG:.1f}"
    )
    print(f"Remote hold last={config.REMOTE_SERVO_HOLD_LAST}, release_idle={config.SERVO_RELEASE_IDLE}")
    print(f"Servo pulse range={config.SERVO_MIN_PULSE_US}..{config.SERVO_MAX_PULSE_US} us")
    print(f"Remote mapped input range={config.REMOTE_INPUT_MIN_ANGLE:.1f}..{config.REMOTE_INPUT_CENTER_ANGLE:.1f}..{config.REMOTE_INPUT_MAX_ANGLE:.1f}")
    print("")


def main() -> None:
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    setup_gpio()
    setup_servo_output()
    setup_keyboard()
    setup_gamepad()
    setup_mqtt()
    setup_imu()

    _print_banner()

    if config.SERVO_RELEASE_IDLE:
        release_servo("BOOT")
    else:
        apply_steering(config.CENTER_ANGLE, "BOOT")
    stop_base()

    if config.keyboard is not None:
        config.keyboard.grab()
    if config.gamepad is not None:
        config.gamepad.grab()

    try:
        while config.running:
            if config.keyboard is not None:
                try:
                    for event in config.keyboard.read():
                        update_key_state(event)
                except (BlockingIOError, OSError):
                    pass
            if config.gamepad is not None:
                try:
                    for event in config.gamepad.read():
                        update_gamepad_button_state(event)
                        update_gamepad_axis_state(config.gamepad, event)
                except (BlockingIOError, OSError):
                    pass
            process_controls()
            time.sleep(config.LOOP_DELAY)
    except KeyboardInterrupt:
        print("\nEMERGENCY EXIT: Ctrl+C")
    finally:
        print("Cleaning up...")
        cleanup()
        print("Done.")


if __name__ == "__main__":
    main()
