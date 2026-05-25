#!/usr/bin/env python3
"""RPi MQTT actuator bridge — subscribes to MQTT, writes GPIO (servo, base, relay).

No gamepad, keyboard, IMU, or control logic. Pure MQTT → hardware executor.
"""

from __future__ import annotations

import errno
import signal
import time

from . import config
from .base import backward, forward, lock_base, stop_base, unlock_base
from .controls import InputController
from .input_handler import InputDeviceHandler
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

    # Setup input devices (optional, graceful fallback)
    input_handler: InputDeviceHandler | None = None
    input_controller: InputController | None = None
    try:
        input_handler = InputDeviceHandler(
            keyboard_device_path=config.KEYBOARD_DEVICE,
            gamepad_device_path=config.GAMEPAD_DEVICE,
            gamepad_name_hints=tuple(config.GAMEPAD_NAME_HINTS.split(",")),
            steer_axis=config.STEER_AXIS,
            drive_axis=config.DRIVE_AXIS,
            hat_y_axis=config.HAT_Y_AXIS,
        )
        input_handler.setup()
        if input_handler.has_gamepad or input_handler.has_keyboard:
            input_controller = InputController(input_handler)
            print(f"INPUT: gamepad={input_handler.has_gamepad} keyboard={input_handler.has_keyboard}")
        else:
            print("INPUT: no devices found, control disabled")
    except Exception as exc:
        print(f"INPUT: setup failed: {exc}")
        input_handler = None
        input_controller = None

    _print_banner()

    if config.SERVO_RELEASE_IDLE:
        release_servo("BOOT")
    else:
        apply_steering(config.CENTER_ANGLE, "BOOT")
    stop_base()

    # Main control loop: poll input, apply decisions, publish control signals
    heartbeat_interval = 10.0
    last_heartbeat = time.monotonic()

    try:
        while config.running:
            now = time.monotonic()

            # Process input if available
            if input_controller is not None and input_handler is not None:
                try:
                    input_handler.poll()
                    decision = input_controller.process(now)

                    # Apply base motor
                    if decision.base_command:
                        cmd = decision.base_command
                        if cmd == "FORWARD":
                            forward()
                        elif cmd == "BACKWARD":
                            backward()
                        elif cmd == "STOP":
                            stop_base()
                        elif cmd == "LOCK":
                            lock_base()
                        elif cmd == "UNLOCK":
                            unlock_base()

                    # Apply relay
                    if decision.relay_command:
                        cmd = decision.relay_command
                        if cmd == "ON":
                            config.gpio.write(config.RELAY_PIN, 1)
                            config.relay_on = True
                        elif cmd == "OFF":
                            config.gpio.write(config.RELAY_PIN, 0)
                            config.relay_on = False

                    # Manual servo override
                    if decision.manual_steer:
                        apply_steering(decision.steer_angle, "GAMEPAD")
                        config.manual_override_active = True
                    else:
                        config.manual_override_active = False
                        # MQTT callback will apply vision servo angle

                except BlockingIOError:
                    # No input event ready on non-blocking devices; skip silently.
                    pass
                except OSError as ctrl_exc:
                    if ctrl_exc.errno == errno.EAGAIN:
                        # Resource temporarily unavailable; retry next loop tick.
                        pass
                    else:
                        print(f"Control error (OSError): {ctrl_exc}")
                except Exception as ctrl_exc:
                    print(f"Control error: {ctrl_exc}")

            # Periodic heartbeat
            if (now - last_heartbeat) >= heartbeat_interval:
                publish_status("online")
                last_heartbeat = now

            time.sleep(0.01)  # ~100Hz loop

    except KeyboardInterrupt:
        print("\nEXIT: Ctrl+C")
    finally:
        print("Cleaning up...")
        if input_handler is not None:
            try:
                input_handler.close()
            except Exception:
                pass
        cleanup()
        print("Done.")


if __name__ == "__main__":
    main()
