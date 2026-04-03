#!/usr/bin/env python3
"""Raspberry Pi keyboard controller with MQTT servo bridge support.

The keyboard stays connected directly to the Raspberry Pi. Motor control
remains local, while remote servo angles received over MQTT are applied
only when the user is not actively steering with the keyboard.
"""

from __future__ import annotations

import json
import os
import signal
import time

try:
    from dotenv import load_dotenv
except ImportError:  # pragma: no cover - optional on Raspberry Pi
    load_dotenv = None

from evdev import InputDevice, ecodes

try:
    import paho.mqtt.client as mqtt
except ImportError as exc:  # pragma: no cover - runtime dependency on Raspberry Pi
    raise RuntimeError(
        "paho-mqtt is required for scripts/rpi_mqtt_bridge.py. Install requirements first."
    ) from exc

try:
    import pigpio
except ImportError as exc:  # pragma: no cover - runtime dependency on Raspberry Pi
    raise RuntimeError(
        "pigpio is required for scripts/rpi_mqtt_bridge.py. Install pigpio and start pigpiod."
    ) from exc

if load_dotenv is not None:
    load_dotenv()


# =========================================================
# CONFIG
# =========================================================
KEYBOARD_DEVICE = os.getenv(
    "KEYBOARD_DEVICE",
    "/dev/input/by-id/usb-YJX_CHIP_WirelessDevice-event-kbd",
)
MQTT_BROKER_HOST = os.getenv("MQTT_BROKER_HOST", "127.0.0.1")
MQTT_BROKER_PORT = int(os.getenv("MQTT_BROKER_PORT", "1883"))
MQTT_USERNAME = os.getenv("MQTT_USERNAME", "")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD", "")
MQTT_KEEPALIVE_S = int(os.getenv("MQTT_KEEPALIVE_S", "60"))
MQTT_SERVO_TOPIC = os.getenv("MQTT_SERVO_TOPIC", "car/servo/angle")
MQTT_STATUS_TOPIC = os.getenv("MQTT_STATUS_TOPIC", "car/status")
MQTT_CLIENT_ID = os.getenv(
    "RPI_MQTT_BRIDGE_CLIENT_ID",
    f"rpi-mqtt-bridge-{os.getpid()}",
)
PIGPIO_HOST = os.getenv("PIGPIO_HOST", "127.0.0.1")
PIGPIO_PORT = int(os.getenv("PIGPIO_PORT", "8888"))
SERVO_PIN = int(os.getenv("SERVO_PIN", "19"))

OUT1 = int(os.getenv("BASE_OUT1", "17"))
OUT2 = int(os.getenv("BASE_OUT2", "27"))
OUT3 = int(os.getenv("BASE_OUT3", "22"))

CENTER_ANGLE = float(os.getenv("SERVO_CENTER_ANGLE", "-8"))
LEFT_LIMIT = float(os.getenv("SERVO_LEFT_LIMIT", "-65"))
RIGHT_LIMIT = float(os.getenv("SERVO_RIGHT_LIMIT", "60"))
STEP = float(os.getenv("SERVO_STEP", "20"))
REMOTE_INPUT_MIN_ANGLE = float(os.getenv("REMOTE_INPUT_MIN_ANGLE", "60"))
REMOTE_INPUT_CENTER_ANGLE = float(os.getenv("REMOTE_INPUT_CENTER_ANGLE", "90"))
REMOTE_INPUT_MAX_ANGLE = float(os.getenv("REMOTE_INPUT_MAX_ANGLE", "120"))

SERVO_MIN_PULSE_US = int(round(float(os.getenv("SERVO_MIN_PULSE", "0.0005")) * 1_000_000))
SERVO_MAX_PULSE_US = int(round(float(os.getenv("SERVO_MAX_PULSE", "0.0025")) * 1_000_000))

REMOTE_SERVO_TIMEOUT = float(os.getenv("REMOTE_SERVO_TIMEOUT", "0.6"))
REMOTE_SERVO_HOLD_LAST = os.getenv("REMOTE_SERVO_HOLD_LAST", "true").strip().lower() in {
    "1",
    "true",
    "t",
    "yes",
    "y",
    "on",
}
MANUAL_STEER_HOLD = float(os.getenv("MANUAL_STEER_HOLD", "0.25"))
LOOP_DELAY = float(os.getenv("LOOP_DELAY", "0.01"))


# =========================================================
# GLOBAL STATE
# =========================================================
running = True
pressed_keys: set[str] = set()
steer_angle = CENTER_ANGLE
last_base_state: tuple[int, int, int] | None = None
last_steer_angle: float | None = None
last_steer_source: str | None = None
remote_servo_angle: float | None = None
remote_servo_updated_at = 0.0
manual_override_until = 0.0
mqtt_client: mqtt.Client | None = None
mqtt_connected = False
gpio: pigpio.pi | None = None
keyboard = InputDevice(KEYBOARD_DEVICE)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def log(message: str) -> None:
    print(message)


def setup_gpio() -> None:
    global gpio

    gpio = pigpio.pi(PIGPIO_HOST, PIGPIO_PORT)
    if not gpio.connected:
        raise RuntimeError(
            f"Cannot connect to pigpio at {PIGPIO_HOST}:{PIGPIO_PORT}. "
            "Start pigpiod first, for example: sudo systemctl enable --now pigpiod"
        )

    for pin in (OUT1, OUT2, OUT3, SERVO_PIN):
        gpio.set_mode(pin, pigpio.OUTPUT)


def angle_to_pulse_us(angle: float) -> int:
    clamped_angle = clamp(angle, LEFT_LIMIT, RIGHT_LIMIT)
    angle_span = RIGHT_LIMIT - LEFT_LIMIT
    if angle_span <= 0:
        return SERVO_MIN_PULSE_US

    ratio = (clamped_angle - LEFT_LIMIT) / angle_span
    return int(round(SERVO_MIN_PULSE_US + ratio * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)))


def set_base(b1: int, b2: int, b3: int, label: str | None = None) -> None:
    global last_base_state
    state = (b1, b2, b3)

    if gpio is None:
        raise RuntimeError("pigpio is not initialized.")

    gpio.write(OUT1, 1 if b1 else 0)
    gpio.write(OUT2, 1 if b2 else 0)
    gpio.write(OUT3, 1 if b3 else 0)

    if state != last_base_state:
        if label:
            log(f"BASE: {label} -> {state}")
        else:
            log(f"BASE: {state}")
        last_base_state = state


def stop_base() -> None:
    set_base(0, 0, 0, "STOP")


def forward() -> None:
    set_base(0, 0, 1, "FORWARD")


def backward() -> None:
    set_base(0, 1, 0, "BACKWARD")


def lock_base() -> None:
    set_base(1, 0, 1, "LOCK")


def unlock_base() -> None:
    set_base(1, 1, 0, "UNLOCK")


def apply_steering(target_angle: float, source: str) -> None:
    global steer_angle, last_steer_angle, last_steer_source

    steer_angle = clamp(target_angle, LEFT_LIMIT, RIGHT_LIMIT)

    if steer_angle != last_steer_angle or source != last_steer_source:
        if gpio is None:
            raise RuntimeError("pigpio is not initialized.")
        pulse_us = angle_to_pulse_us(steer_angle)
        gpio.set_servo_pulsewidth(SERVO_PIN, pulse_us)
        log(
            f"STEER[{source}]: {steer_angle:.1f} deg | CENTER: {CENTER_ANGLE:.1f} deg | PULSE: {pulse_us} us"
        )
        last_steer_angle = steer_angle
        last_steer_source = source


def steer_center(source: str) -> None:
    apply_steering(CENTER_ANGLE, source)


def steer_right_step(source: str) -> None:
    apply_steering(steer_angle - STEP, source)


def steer_left_step(source: str) -> None:
    apply_steering(steer_angle + STEP, source)


def remote_control_active(now: float) -> bool:
    if REMOTE_SERVO_HOLD_LAST:
        del now
        return remote_servo_angle is not None

    return (
        remote_servo_angle is not None
        and now - remote_servo_updated_at <= REMOTE_SERVO_TIMEOUT
    )


def map_remote_angle(angle: float) -> float:
    angle = clamp(angle, REMOTE_INPUT_MIN_ANGLE, REMOTE_INPUT_MAX_ANGLE)

    if angle <= REMOTE_INPUT_CENTER_ANGLE:
        remote_span = REMOTE_INPUT_CENTER_ANGLE - REMOTE_INPUT_MIN_ANGLE
        if remote_span <= 0:
            return CENTER_ANGLE
        ratio = (angle - REMOTE_INPUT_CENTER_ANGLE) / remote_span
        return CENTER_ANGLE + ratio * (CENTER_ANGLE - LEFT_LIMIT)

    remote_span = REMOTE_INPUT_MAX_ANGLE - REMOTE_INPUT_CENTER_ANGLE
    if remote_span <= 0:
        return CENTER_ANGLE
    ratio = (angle - REMOTE_INPUT_CENTER_ANGLE) / remote_span
    return CENTER_ANGLE + ratio * (RIGHT_LIMIT - CENTER_ANGLE)


def resolve_remote_servo_angle(payload_text: str) -> float:
    payload_text = payload_text.strip()
    if not payload_text:
        raise ValueError("Empty MQTT servo payload")

    if payload_text.startswith("{"):
        payload = json.loads(payload_text)
        command = payload.get("type", "angle")
        if command == "center":
            raw_angle = CENTER_ANGLE
        elif command == "angle":
            raw_angle = float(payload.get("angle", REMOTE_INPUT_CENTER_ANGLE))
        else:
            raise ValueError(f"Unsupported command: {command}")
    else:
        raw_angle = float(payload_text)

    # Accept already-normalized signed steering angles directly.
    if LEFT_LIMIT <= raw_angle <= RIGHT_LIMIT:
        return clamp(raw_angle, LEFT_LIMIT, RIGHT_LIMIT)

    return clamp(map_remote_angle(raw_angle), LEFT_LIMIT, RIGHT_LIMIT)


def publish_status(state: str) -> None:
    if mqtt_client is None or not mqtt_connected:
        return

    payload = json.dumps(
        {
            "source": "rpi-mqtt-bridge",
            "state": state,
            "steer_angle": round(steer_angle, 2),
            "center_angle": round(CENTER_ANGLE, 2),
            "servo_pin": SERVO_PIN,
            "pigpio_host": PIGPIO_HOST,
            "ts": time.time(),
        }
    )
    mqtt_client.publish(MQTT_STATUS_TOPIC, payload, retain=False)


def handle_mqtt_servo_message(payload_text: str) -> None:
    global remote_servo_angle, remote_servo_updated_at

    remote_servo_angle = resolve_remote_servo_angle(payload_text)
    remote_servo_updated_at = time.monotonic()


def on_mqtt_connect(client, userdata, flags, rc, properties=None) -> None:
    del userdata, flags, properties
    global mqtt_connected

    if rc != 0:
        log(f"MQTT: connect failed rc={rc}")
        mqtt_connected = False
        return

    mqtt_connected = True
    client.subscribe(MQTT_SERVO_TOPIC)
    log(f"MQTT: connected to {MQTT_BROKER_HOST}:{MQTT_BROKER_PORT} topic={MQTT_SERVO_TOPIC}")
    publish_status("online")


def on_mqtt_disconnect(client, userdata, rc, properties=None) -> None:
    del client, userdata, properties
    global mqtt_connected
    mqtt_connected = False

    if running:
        log(f"MQTT: disconnected rc={rc}")


def on_mqtt_message(client, userdata, message) -> None:
    del client, userdata

    try:
        payload_text = message.payload.decode("utf-8")
        handle_mqtt_servo_message(payload_text)
    except (UnicodeDecodeError, ValueError, json.JSONDecodeError) as exc:
        log(f"MQTT: invalid payload on {message.topic}: {exc}")


def setup_mqtt() -> None:
    global mqtt_client

    callback_api_version = getattr(mqtt, "CallbackAPIVersion", None)
    if callback_api_version is not None:
        client = mqtt.Client(
            callback_api_version=callback_api_version.VERSION1,
            client_id=MQTT_CLIENT_ID,
        )
    else:
        client = mqtt.Client(client_id=MQTT_CLIENT_ID)

    if MQTT_USERNAME:
        client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

    client.on_connect = on_mqtt_connect
    client.on_disconnect = on_mqtt_disconnect
    client.on_message = on_mqtt_message
    client.reconnect_delay_set(min_delay=1, max_delay=5)
    client.connect_async(MQTT_BROKER_HOST, MQTT_BROKER_PORT, keepalive=MQTT_KEEPALIVE_S)
    client.loop_start()
    mqtt_client = client


def close_mqtt() -> None:
    global mqtt_client, mqtt_connected

    if mqtt_client is None:
        return

    try:
        publish_status("offline")
    except Exception:
        pass

    try:
        mqtt_client.disconnect()
    except Exception:
        pass

    try:
        mqtt_client.loop_stop()
    except Exception:
        pass

    mqtt_client = None
    mqtt_connected = False


def cleanup() -> None:
    try:
        stop_base()
    except Exception:
        pass

    try:
        steer_center("CLEANUP")
    except Exception:
        pass

    try:
        keyboard.ungrab()
    except Exception:
        pass

    close_mqtt()

    try:
        if gpio is not None:
            gpio.set_servo_pulsewidth(SERVO_PIN, 0)
    except Exception:
        pass

    try:
        if gpio is not None:
            gpio.stop()
    except Exception:
        pass


def signal_handler(sig, frame) -> None:
    del sig, frame
    global running
    running = False


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


def update_key_state(event) -> None:
    global CENTER_ANGLE, running

    if event.type != ecodes.EV_KEY:
        return

    keycode = ecodes.KEY.get(event.code)
    if not keycode:
        return

    keys = keycode if isinstance(keycode, list) else [keycode]

    if event.value in (1, 2):
        for key in keys:
            pressed_keys.add(key)
            if event.value == 1:
                if key == "KEY_1":
                    CENTER_ANGLE = clamp(CENTER_ANGLE + 1, LEFT_LIMIT, RIGHT_LIMIT)
                    log(f"CENTER_ANGLE INCREASED: {CENTER_ANGLE}")
                elif key == "KEY_2":
                    CENTER_ANGLE = clamp(CENTER_ANGLE - 1, LEFT_LIMIT, RIGHT_LIMIT)
                    log(f"CENTER_ANGLE DECREASED: {CENTER_ANGLE}")
                elif key == "KEY_Q":
                    running = False
    elif event.value == 0:
        for key in keys:
            pressed_keys.discard(key)


def process_controls() -> None:
    global manual_override_until

    now = time.monotonic()

    if "KEY_L" in pressed_keys:
        lock_base()
        return

    if "KEY_U" in pressed_keys:
        unlock_base()
        return

    has_w = "KEY_W" in pressed_keys
    has_s = "KEY_S" in pressed_keys
    has_x = "KEY_X" in pressed_keys

    if has_x or (has_w and has_s):
        stop_base()
    elif has_w:
        backward()
    elif has_s:
        forward()
    else:
        stop_base()

    has_a = "KEY_A" in pressed_keys
    has_d = "KEY_D" in pressed_keys
    has_c = "KEY_C" in pressed_keys

    if has_c:
        manual_override_until = now + MANUAL_STEER_HOLD
        steer_center("KEYBOARD")
        return

    if has_a and not has_d:
        manual_override_until = now + MANUAL_STEER_HOLD
        steer_left_step("KEYBOARD")
        return

    if has_d and not has_a:
        manual_override_until = now + MANUAL_STEER_HOLD
        steer_right_step("KEYBOARD")
        return

    if now < manual_override_until:
        return

    if remote_control_active(now):
        apply_steering(remote_servo_angle if remote_servo_angle is not None else CENTER_ANGLE, "REMOTE")
    else:
        steer_center("IDLE")


def main() -> None:
    global running

    setup_gpio()
    setup_mqtt()

    print("=== RPI KEYBOARD + MQTT SERVO BRIDGE MODE ===")
    print(f"Keyboard: {keyboard.path}")
    print(f"MQTT broker: {MQTT_BROKER_HOST}:{MQTT_BROKER_PORT}")
    print(f"MQTT servo topic: {MQTT_SERVO_TOPIC}")
    print(f"pigpio: {PIGPIO_HOST}:{PIGPIO_PORT}")
    print(f"Servo GPIO pin: {SERVO_PIN}")
    print("W = forward")
    print("S = backward")
    print("A = steer left")
    print("D = steer right")
    print("C = center steering")
    print("X = stop")
    print("L = lock")
    print("U = unlock")
    print("1 = increase center angle")
    print("2 = decrease center angle")
    print("Q = quit")
    print("Ctrl+C = emergency exit")
    print("")
    print(
        f"Steering center={CENTER_ANGLE:.1f}, left={LEFT_LIMIT:.1f}, right={RIGHT_LIMIT:.1f}, "
        f"step={STEP:.1f}, remote_hold_last={REMOTE_SERVO_HOLD_LAST}"
    )
    if not REMOTE_SERVO_HOLD_LAST:
        print(f"Remote timeout={REMOTE_SERVO_TIMEOUT:.2f}s")
    print(
        f"Servo pulse range={SERVO_MIN_PULSE_US}..{SERVO_MAX_PULSE_US} us"
    )
    print(
        f"Remote mapped input range={REMOTE_INPUT_MIN_ANGLE:.1f}..{REMOTE_INPUT_CENTER_ANGLE:.1f}..{REMOTE_INPUT_MAX_ANGLE:.1f}"
    )
    print("Remote payload accepts plain float or JSON payload with type/angle.")
    print("")

    steer_center("BOOT")
    stop_base()

    try:
        keyboard.grab()

        while running:
            try:
                for event in keyboard.read():
                    update_key_state(event)
            except (BlockingIOError, OSError):
                pass

            process_controls()
            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        print("\nEMERGENCY EXIT: Ctrl+C")
    finally:
        print("Cleaning up...")
        cleanup()
        print("Done.")


if __name__ == "__main__":
    main()
