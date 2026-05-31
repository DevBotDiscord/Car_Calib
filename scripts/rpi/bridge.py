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
from .logging_utils import get_logger, setup_rpi_logging
from .mqtt_client import close_mqtt, publish_status, setup_mqtt
from .steering import apply_steering, release_servo

logger = get_logger("bridge")


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
    logger.info("[BRIDGE][BOOT] RPI MQTT ACTUATOR BRIDGE")
    logger.info("[BRIDGE][BOOT] mqtt host=%s port=%s", config.MQTT_BROKER_HOST, config.MQTT_BROKER_PORT)
    logger.info(
        "[BRIDGE][BOOT] topics servo=%s base=%s relay=%s",
        config.MQTT_SERVO_TOPIC,
        config.MQTT_BASE_COMMAND_TOPIC,
        config.MQTT_RELAY_TOPIC,
    )
    logger.info("[BRIDGE][BOOT] pigpio host=%s port=%s", config.PIGPIO_HOST, config.PIGPIO_PORT)
    logger.info(
        "[BRIDGE][BOOT] pins servo=%s base=%s,%s,%s relay=%s",
        config.SERVO_PIN,
        config.OUT1,
        config.OUT2,
        config.OUT3,
        config.RELAY_PIN,
    )
    logger.info(
        "[BRIDGE][BOOT] steering center=%.1f limits=%.1f,%.1f",
        config.CENTER_ANGLE,
        config.LEFT_LIMIT,
        config.RIGHT_LIMIT,
    )
    logger.info(
        "[BRIDGE][BOOT] servo release_idle=%s hold_last=%s",
        config.SERVO_RELEASE_IDLE,
        config.REMOTE_SERVO_HOLD_LAST,
    )


def main() -> None:
    setup_rpi_logging()
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
            logger.info(
                "[BRIDGE][INPUT] gamepad=%s keyboard=%s",
                input_handler.has_gamepad,
                input_handler.has_keyboard,
            )
        else:
            logger.info("[BRIDGE][INPUT] no devices found control=disabled")
    except Exception as exc:
        logger.warning("[BRIDGE][INPUT] setup failed error=%s", exc)
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
                        logger.error("[BRIDGE][CONTROL] os_error=%s", ctrl_exc)
                except Exception as ctrl_exc:
                    logger.error("[BRIDGE][CONTROL] error=%s", ctrl_exc)

            # Periodic heartbeat
            if (now - last_heartbeat) >= heartbeat_interval:
                publish_status("online")
                last_heartbeat = now

            time.sleep(0.01)  # ~100Hz loop

    except KeyboardInterrupt:
        logger.info("[BRIDGE][EXIT] keyboard_interrupt")
    finally:
        logger.info("[BRIDGE][CLEANUP] start")
        if input_handler is not None:
            try:
                input_handler.close()
            except Exception:
                pass
        cleanup()
        logger.info("[BRIDGE][CLEANUP] done")


if __name__ == "__main__":
    main()
