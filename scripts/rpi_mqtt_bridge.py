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
    from .servo_bridge_common import (
        angle_within_limits,
        clamp_angle,
    )
    from .input_device_helpers import (
        find_optional_abs_input_device,
        open_optional_input_device,
    )
    from .angular_servo_output import (
        AngularServoOutput,
        apply_boot_servo_behavior,
        apply_idle_servo_behavior,
    )
except ImportError:  # pragma: no cover - direct script execution on Raspberry Pi
    from servo_bridge_common import (  # type: ignore
        angle_within_limits,
        clamp_angle,
    )
    from input_device_helpers import (  # type: ignore
        find_optional_abs_input_device,
        open_optional_input_device,
    )
    from angular_servo_output import (  # type: ignore
        AngularServoOutput,
        apply_boot_servo_behavior,
        apply_idle_servo_behavior,
    )

try:
    from dotenv import load_dotenv
except ImportError:  # pragma: no cover - optional on Raspberry Pi
    load_dotenv = None

from evdev import InputDevice, ecodes, list_devices

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
GAMEPAD_DEVICE = os.getenv("GAMEPAD_DEVICE", "").strip()
GAMEPAD_NAME_HINTS = tuple(
    hint.strip().lower()
    for hint in os.getenv(
        "GAMEPAD_NAME_HINTS",
        "edra,joystick,gamepad,controller,pad",
    ).split(",")
    if hint.strip()
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

STEER_AXIS = ecodes.ABS_RX
DRIVE_AXIS = ecodes.ABS_Y
HAT_Y_AXIS = ecodes.ABS_HAT0Y
BUTTON_STOP = ecodes.BTN_SOUTH
BUTTON_CENTER = ecodes.BTN_EAST
BUTTON_LOCK = ecodes.BTN_TL
BUTTON_UNLOCK = ecodes.BTN_TR
BUTTON_REMOTE_STEER_ONLY = ecodes.BTN_NORTH
BUTTON_QUIT = ecodes.BTN_START
BUTTON_CENTER_PLUS = None
BUTTON_CENTER_MINUS = ecodes.BTN_WEST

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
STEER_DEADBAND_DEG = float(os.getenv("STEER_DEADBAND_DEG", "1.0"))
GAMEPAD_STEER_DEADZONE = float(os.getenv("GAMEPAD_STEER_DEADZONE", "0.12"))
GAMEPAD_DRIVE_DEADZONE = float(os.getenv("GAMEPAD_DRIVE_DEADZONE", "0.20"))
INVERT_STEER_AXIS = os.getenv("INVERT_STEER_AXIS", "false").strip().lower() in {
    "1",
    "true",
    "t",
    "yes",
    "y",
    "on",
}
INVERT_DRIVE_AXIS = os.getenv("INVERT_DRIVE_AXIS", "false").strip().lower() in {
    "1",
    "true",
    "t",
    "yes",
    "y",
    "on",
}
SERVO_RELEASE_IDLE = os.getenv("SERVO_RELEASE_IDLE", "true").strip().lower() in {
    "1",
    "true",
    "t",
    "yes",
    "y",
    "on",
}


# =========================================================
# GLOBAL STATE
# =========================================================
running = True
pressed_keys: set[str] = set()
pressed_buttons: set[int] = set()
axis_state = {
    STEER_AXIS: 0.0,
    DRIVE_AXIS: 0.0,
}
hat_state = {
    HAT_Y_AXIS: 0,
}
steer_angle = CENTER_ANGLE
last_base_state: tuple[int, int, int] | None = None
last_steer_angle: float | None = None
last_steer_source: str | None = None
remote_servo_angle: float | None = None
remote_servo_updated_at = 0.0
manual_override_until = 0.0
manual_override_source: str | None = None
mqtt_client: mqtt.Client | None = None
mqtt_connected = False
gpio: pigpio.pi | None = None
servo_output: AngularServoOutput | None = None
keyboard: InputDevice | None = None
gamepad: InputDevice | None = None


def clamp(value: float, low: float, high: float) -> float:
    return clamp_angle(value, low, high)


def log(message: str) -> None:
    print(message)


def apply_deadzone(value: float, deadzone: float) -> float:
    if abs(value) < deadzone:
        return 0.0
    return value


def normalize_axis(device: InputDevice, axis_code: int, raw_value: int) -> float:
    try:
        info = device.absinfo(axis_code)
    except Exception:
        return 0.0

    minimum = info.min
    maximum = info.max
    center = (minimum + maximum) / 2.0
    half_range = (maximum - minimum) / 2.0

    if half_range <= 0:
        return 0.0

    value = (raw_value - center) / half_range
    return clamp(value, -1.0, 1.0)


def controller_remote_steer_only_enabled() -> bool:
    return BUTTON_REMOTE_STEER_ONLY in pressed_buttons


def activate_manual_override(source: str, now: float) -> None:
    global manual_override_until, manual_override_source

    manual_override_until = now + MANUAL_STEER_HOLD
    manual_override_source = source


def clear_manual_override() -> None:
    global manual_override_until, manual_override_source

    manual_override_until = 0.0
    manual_override_source = None


def setup_gpio() -> None:
    global gpio

    gpio = pigpio.pi(PIGPIO_HOST, PIGPIO_PORT)
    if not gpio.connected:
        raise RuntimeError(
            f"Cannot connect to pigpio at {PIGPIO_HOST}:{PIGPIO_PORT}. "
            "Start pigpiod first, for example: sudo systemctl enable --now pigpiod"
        )

    for pin in (OUT1, OUT2, OUT3):
        gpio.set_mode(pin, pigpio.OUTPUT)


def setup_servo_output() -> None:
    global servo_output

    servo_output = AngularServoOutput(
        servo_pin=SERVO_PIN,
        left_limit=LEFT_LIMIT,
        right_limit=RIGHT_LIMIT,
        servo_min_pulse_us=SERVO_MIN_PULSE_US,
        servo_max_pulse_us=SERVO_MAX_PULSE_US,
        pigpio_host=PIGPIO_HOST,
        pigpio_port=PIGPIO_PORT,
        center_angle=CENTER_ANGLE,
        log=log,
    )


def setup_keyboard() -> None:
    global keyboard

    keyboard = open_optional_input_device(
        KEYBOARD_DEVICE,
        log=log,
        device_factory=InputDevice,
    )


def setup_gamepad() -> None:
    global gamepad

    gamepad = find_optional_abs_input_device(
        GAMEPAD_DEVICE,
        log=log,
        device_factory=InputDevice,
        list_devices_fn=list_devices,
        name_hints=GAMEPAD_NAME_HINTS,
        ev_abs_code=ecodes.EV_ABS,
    )
    if gamepad is not None:
        log(f"INPUT: controller ready: {gamepad.name} ({gamepad.path})")


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

    target = clamp(target_angle, LEFT_LIMIT, RIGHT_LIMIT)

    if (
        last_steer_angle is not None
        and abs(target - last_steer_angle) < STEER_DEADBAND_DEG
        and source == last_steer_source
        and servo_output is not None
        and servo_output.attached
    ):
        return

    steer_angle = target

    if servo_output is None:
        raise RuntimeError("AngularServo output is not initialized.")

    steer_angle = servo_output.set_servo(steer_angle, source)
    last_steer_angle = steer_angle
    last_steer_source = source


def release_servo(reason: str = "IDLE") -> None:
    if servo_output is None:
        raise RuntimeError("AngularServo output is not initialized.")

    servo_output.detach_servo(reason)


def steer_center(source: str) -> None:
    apply_steering(CENTER_ANGLE, source)


def steer_right_step(source: str) -> None:
    apply_steering(steer_angle - STEP, source)


def steer_left_step(source: str) -> None:
    apply_steering(steer_angle + STEP, source)


def steer_from_gamepad_axis(axis_value: float, source: str) -> bool:
    if INVERT_STEER_AXIS:
        axis_value = -axis_value

    axis_value = apply_deadzone(axis_value, GAMEPAD_STEER_DEADZONE)
    if axis_value == 0.0:
        return False

    if axis_value < 0:
        target_angle = CENTER_ANGLE + axis_value * (CENTER_ANGLE - LEFT_LIMIT)
    else:
        target_angle = CENTER_ANGLE + axis_value * (RIGHT_LIMIT - CENTER_ANGLE)

    apply_steering(target_angle, source)
    return True


def adjust_center(delta: float, source: str) -> None:
    global CENTER_ANGLE

    CENTER_ANGLE = clamp(CENTER_ANGLE + delta, LEFT_LIMIT, RIGHT_LIMIT)
    if servo_output is not None:
        servo_output.set_center_angle(CENTER_ANGLE)
    log(f"CENTER_ANGLE: {CENTER_ANGLE}")

    if source == "GAMEPAD":
        if controller_remote_steer_only_enabled():
            return
        steer_value = apply_deadzone(axis_state[STEER_AXIS], GAMEPAD_STEER_DEADZONE)
        if steer_value == 0.0:
            activate_manual_override(source, time.monotonic())
            steer_center(source)


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
    if angle_within_limits(raw_angle, LEFT_LIMIT, RIGHT_LIMIT):
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
        release_servo("CLEANUP")
    except Exception:
        pass

    try:
        if keyboard is not None:
            keyboard.ungrab()
    except Exception:
        pass

    try:
        if gamepad is not None:
            gamepad.ungrab()
    except Exception:
        pass

    close_mqtt()

    try:
        if servo_output is not None:
            servo_output.close()
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
    global running

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
                    adjust_center(1, "KEYBOARD")
                elif key == "KEY_2":
                    adjust_center(-1, "KEYBOARD")
                elif key == "KEY_Q":
                    running = False
    elif event.value == 0:
        for key in keys:
            pressed_keys.discard(key)


def update_gamepad_button_state(event) -> None:
    global running

    if event.type != ecodes.EV_KEY:
        return

    code = event.code
    if event.value == 1:
        pressed_buttons.add(code)
        if BUTTON_CENTER_PLUS is not None and code == BUTTON_CENTER_PLUS:
            adjust_center(1, "GAMEPAD")
        elif code == BUTTON_CENTER_MINUS:
            adjust_center(-1, "GAMEPAD")
        elif code == BUTTON_QUIT:
            running = False
    elif event.value == 0:
        pressed_buttons.discard(code)


def update_gamepad_axis_state(device: InputDevice, event) -> None:
    if event.type != ecodes.EV_ABS:
        return

    if event.code in axis_state:
        axis_state[event.code] = normalize_axis(device, event.code, event.value)
        return

    if event.code == HAT_Y_AXIS and event.value != hat_state[HAT_Y_AXIS]:
        hat_state[HAT_Y_AXIS] = event.value
        if event.value == -1:
            adjust_center(1, "GAMEPAD")
        elif event.value == 1:
            adjust_center(-1, "GAMEPAD")


def process_controls() -> None:
    global manual_override_source

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

    keyboard_base_active = has_x or has_w or has_s
    if has_x or (has_w and has_s):
        stop_base()
    elif has_w:
        backward()
    elif has_s:
        forward()
    else:
        drive_value = axis_state[DRIVE_AXIS]
        if INVERT_DRIVE_AXIS:
            drive_value = -drive_value
        drive_value = apply_deadzone(drive_value, GAMEPAD_DRIVE_DEADZONE)

        if BUTTON_LOCK in pressed_buttons:
            lock_base()
            return

        if BUTTON_UNLOCK in pressed_buttons:
            unlock_base()
            return

        if BUTTON_STOP in pressed_buttons:
            stop_base()
        elif drive_value < 0:
            set_base(0, 1, 0, "FORWARD")
        elif drive_value > 0:
            set_base(0, 0, 1, "BACKWARD")
        elif keyboard_base_active:
            stop_base()
        else:
            stop_base()

    has_a = "KEY_A" in pressed_keys
    has_d = "KEY_D" in pressed_keys
    has_c = "KEY_C" in pressed_keys
    gamepad_remote_steer_only = controller_remote_steer_only_enabled()

    if has_c:
        activate_manual_override("KEYBOARD", now)
        steer_center("KEYBOARD")
        return

    if has_a and not has_d:
        activate_manual_override("KEYBOARD", now)
        steer_left_step("KEYBOARD")
        return

    if has_d and not has_a:
        activate_manual_override("KEYBOARD", now)
        steer_right_step("KEYBOARD")
        return

    if gamepad_remote_steer_only and manual_override_source == "GAMEPAD":
        clear_manual_override()

    if now < manual_override_until:
        return

    if not gamepad_remote_steer_only:
        if BUTTON_CENTER in pressed_buttons:
            activate_manual_override("GAMEPAD", now)
            steer_center("GAMEPAD")
            return

        if steer_from_gamepad_axis(axis_state[STEER_AXIS], "GAMEPAD"):
            activate_manual_override("GAMEPAD", now)
            return

    if remote_control_active(now):
        apply_steering(remote_servo_angle if remote_servo_angle is not None else CENTER_ANGLE, "REMOTE")
    else:
        if servo_output is None:
            raise RuntimeError("AngularServo output is not initialized.")

        apply_idle_servo_behavior(
            servo_output,
            release_idle=SERVO_RELEASE_IDLE,
            center_angle=CENTER_ANGLE,
        )


def main() -> None:
    global running

    setup_gpio()
    setup_servo_output()
    setup_keyboard()
    setup_gamepad()
    setup_mqtt()

    print("=== RPI KEYBOARD + MQTT SERVO BRIDGE MODE ===")
    if keyboard is not None:
        print(f"Keyboard: {keyboard.path}")
    else:
        print(f"Keyboard: disabled (missing {KEYBOARD_DEVICE})")
    if gamepad is not None:
        print(f"Controller: {gamepad.name} ({gamepad.path})")
    else:
        print("Controller: disabled (not detected)")
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
    print("Right stick X = steering")
    print("Left stick Y = base drive")
    print("Y = keep base local, hand steering back to MQTT")
    print("A = stop | B = center steering | LB = lock | RB = unlock")
    print("X or D-pad down = center angle -1 | D-pad up = center angle +1 | START = quit")
    print("Ctrl+C = emergency exit")
    print("")
    print(
        f"Steering center={CENTER_ANGLE:.1f}, left={LEFT_LIMIT:.1f}, right={RIGHT_LIMIT:.1f}, "
        f"step={STEP:.1f}, deadband={STEER_DEADBAND_DEG:.1f}, "
        f"gamepad_steer_deadzone={GAMEPAD_STEER_DEADZONE:.2f}, "
        f"gamepad_drive_deadzone={GAMEPAD_DRIVE_DEADZONE:.2f}"
    )
    print(
        f"Remote hold last={REMOTE_SERVO_HOLD_LAST}, timeout={REMOTE_SERVO_TIMEOUT:.2f}s, "
        f"release_idle={SERVO_RELEASE_IDLE}"
    )
    print(
        f"Servo pulse range={SERVO_MIN_PULSE_US}..{SERVO_MAX_PULSE_US} us"
    )
    print(
        f"Remote mapped input range={REMOTE_INPUT_MIN_ANGLE:.1f}..{REMOTE_INPUT_CENTER_ANGLE:.1f}..{REMOTE_INPUT_MAX_ANGLE:.1f}"
    )
    print("Remote payload accepts plain float or JSON payload with type/angle.")
    print("")

    if servo_output is None:
        raise RuntimeError("AngularServo output is not initialized.")

    apply_boot_servo_behavior(
        servo_output,
        release_idle=SERVO_RELEASE_IDLE,
        center_angle=CENTER_ANGLE,
    )
    stop_base()

    try:
        if keyboard is not None:
            keyboard.grab()
        if gamepad is not None:
            gamepad.grab()

        while running:
            if keyboard is not None:
                try:
                    for event in keyboard.read():
                        update_key_state(event)
                except (BlockingIOError, OSError):
                    pass

            if gamepad is not None:
                try:
                    for event in gamepad.read():
                        update_gamepad_button_state(event)
                        update_gamepad_axis_state(gamepad, event)
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
