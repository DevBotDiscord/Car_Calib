#!/usr/bin/env python3
"""Raspberry Pi keyboard controller with remote servo bridge support.

The keyboard stays connected directly to the Raspberry Pi. Motor control
remains local, while remote servo angles received over TCP are applied
only when the user is not actively steering with the keyboard.
"""

from __future__ import annotations

import json
import os
import signal
import socket
import time

import RPi.GPIO as GPIO
from evdev import InputDevice, ecodes
from gpiozero import AngularServo

# =========================================================
# CONFIG
# =========================================================
KEYBOARD_DEVICE = os.getenv(
    "KEYBOARD_DEVICE",
    "/dev/input/by-id/usb-YJX_CHIP_WirelessDevice-event-kbd",
)
BRIDGE_HOST = os.getenv("SERVO_BRIDGE_HOST", "0.0.0.0")
BRIDGE_PORT = int(os.getenv("SERVO_BRIDGE_PORT", "8765"))
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

SERVO_MIN_PULSE = float(os.getenv("SERVO_MIN_PULSE", "0.0005"))
SERVO_MAX_PULSE = float(os.getenv("SERVO_MAX_PULSE", "0.0025"))

REMOTE_SERVO_TIMEOUT = float(os.getenv("REMOTE_SERVO_TIMEOUT", "0.6"))
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
bridge_server: socket.socket | None = None
bridge_client: socket.socket | None = None
bridge_client_addr: tuple[str, int] | None = None
bridge_buffer = ""

# =========================================================
# GPIO SETUP
# =========================================================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(OUT1, GPIO.OUT)
GPIO.setup(OUT2, GPIO.OUT)
GPIO.setup(OUT3, GPIO.OUT)

servo = AngularServo(
    SERVO_PIN,
    min_angle=-90,
    max_angle=90,
    min_pulse_width=SERVO_MIN_PULSE,
    max_pulse_width=SERVO_MAX_PULSE,
)
keyboard = InputDevice(KEYBOARD_DEVICE)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def log(message: str) -> None:
    print(message)


def setup_bridge_server() -> None:
    global bridge_server

    bridge_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    bridge_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    bridge_server.bind((BRIDGE_HOST, BRIDGE_PORT))
    bridge_server.listen(1)
    bridge_server.setblocking(False)


def close_bridge_client() -> None:
    global bridge_client, bridge_client_addr, bridge_buffer

    if bridge_client is not None:
        try:
            bridge_client.close()
        except OSError:
            pass

    bridge_client = None
    bridge_client_addr = None
    bridge_buffer = ""


def close_bridge_server() -> None:
    global bridge_server

    close_bridge_client()
    if bridge_server is not None:
        try:
            bridge_server.close()
        except OSError:
            pass
        bridge_server = None


def set_base(b1: int, b2: int, b3: int, label: str | None = None) -> None:
    global last_base_state
    state = (b1, b2, b3)

    GPIO.output(OUT1, GPIO.HIGH if b1 else GPIO.LOW)
    GPIO.output(OUT2, GPIO.HIGH if b2 else GPIO.LOW)
    GPIO.output(OUT3, GPIO.HIGH if b3 else GPIO.LOW)

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
        servo.angle = steer_angle
        log(f"STEER[{source}]: {steer_angle:.1f} deg | CENTER: {CENTER_ANGLE:.1f} deg")
        last_steer_angle = steer_angle
        last_steer_source = source


def steer_center(source: str) -> None:
    apply_steering(CENTER_ANGLE, source)


def steer_right_step(source: str) -> None:
    apply_steering(steer_angle - STEP, source)


def steer_left_step(source: str) -> None:
    apply_steering(steer_angle + STEP, source)


def remote_control_active(now: float) -> bool:
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


def handle_remote_payload(payload: dict[str, float | int | str]) -> None:
    global remote_servo_angle, remote_servo_updated_at

    command = payload.get("type")
    if command not in {"angle", "center"}:
        raise ValueError(f"Unsupported command: {command}")

    angle = float(payload.get("angle", REMOTE_INPUT_CENTER_ANGLE))
    remote_servo_angle = clamp(map_remote_angle(angle), LEFT_LIMIT, RIGHT_LIMIT)
    remote_servo_updated_at = time.monotonic()


def poll_bridge() -> None:
    global bridge_client, bridge_client_addr, bridge_buffer

    if bridge_server is None:
        return

    if bridge_client is None:
        try:
            conn, addr = bridge_server.accept()
            conn.setblocking(False)
            bridge_client = conn
            bridge_client_addr = addr
            bridge_buffer = ""
            log(f"BRIDGE: client connected {addr[0]}:{addr[1]}")
        except BlockingIOError:
            return

    while bridge_client is not None:
        try:
            data = bridge_client.recv(4096)
        except BlockingIOError:
            break
        except OSError as exc:
            log(f"BRIDGE: recv error {exc}")
            close_bridge_client()
            break

        if not data:
            if bridge_client_addr is not None:
                log(f"BRIDGE: client disconnected {bridge_client_addr[0]}:{bridge_client_addr[1]}")
            close_bridge_client()
            break

        bridge_buffer += data.decode("utf-8")
        while "\n" in bridge_buffer:
            line, bridge_buffer = bridge_buffer.split("\n", 1)
            if not line.strip():
                continue

            try:
                payload = json.loads(line)
                handle_remote_payload(payload)
            except (ValueError, json.JSONDecodeError) as exc:
                log(f"BRIDGE: invalid payload {exc}")


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

    close_bridge_server()

    try:
        servo.detach()
    except Exception:
        pass

    try:
        GPIO.cleanup()
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

    setup_bridge_server()

    print("=== RPI KEYBOARD + SERVO BRIDGE MODE ===")
    print(f"Keyboard: {keyboard.path}")
    print(f"Bridge listen: {BRIDGE_HOST}:{BRIDGE_PORT}")
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
        f"step={STEP:.1f}, remote_timeout={REMOTE_SERVO_TIMEOUT:.2f}s"
    )
    print(
        f"Remote input range={REMOTE_INPUT_MIN_ANGLE:.1f}..{REMOTE_INPUT_CENTER_ANGLE:.1f}..{REMOTE_INPUT_MAX_ANGLE:.1f}"
    )
    print("")

    steer_center("BOOT")
    stop_base()

    try:
        keyboard.grab()

        while running:
            poll_bridge()

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
