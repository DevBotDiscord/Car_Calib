#!/usr/bin/env python3
import signal
import time

import pigpio
from evdev import InputDevice, ecodes, list_devices

# =========================================================
# CONFIG
# =========================================================
GAMEPAD_DEVICE = None
GAMEPAD_NAME_HINTS = ("edra", "joystick", "gamepad", "controller","pad")

# Main joystick mapping
# Use second joystick (right stick X) for servo steering
STEER_AXIS = ecodes.ABS_RX
DRIVE_AXIS = ecodes.ABS_Y
HAT_Y_AXIS = ecodes.ABS_HAT0Y

# Common gamepad buttons
BUTTON_STOP = ecodes.BTN_SOUTH      # A
BUTTON_CENTER = ecodes.BTN_EAST     # B
BUTTON_LOCK = ecodes.BTN_TL         # LB
BUTTON_UNLOCK = ecodes.BTN_TR       # RB
BUTTON_QUIT = ecodes.BTN_START
BUTTON_CENTER_PLUS = ecodes.BTN_NORTH   # Y
BUTTON_CENTER_MINUS = ecodes.BTN_WEST   # X

INVERT_STEER_AXIS = False
INVERT_DRIVE_AXIS = False
STEER_DEADZONE = 0.12
DRIVE_DEADZONE = 0.20

# Base control pins
OUT1 = 17
OUT2 = 27
OUT3 = 22

# Servo pin
SERVO_PIN = 12

# Steering config
CENTER_ANGLE = -26
SERVO_MAX_ANGLE_DEG = 45
LEFT_LIMIT = CENTER_ANGLE - SERVO_MAX_ANGLE_DEG
RIGHT_LIMIT = CENTER_ANGLE + SERVO_MAX_ANGLE_DEG

# MG996R pulse
SERVO_MIN_PULSE = 0.0005
SERVO_MAX_PULSE = 0.0025

LOOP_DELAY = 0.01

# =========================================================
# GLOBAL STATE
# =========================================================
running = True
gamepad = None
pressed_buttons = set()
axis_state = {
    STEER_AXIS: 0.0,
    DRIVE_AXIS: 0.0,
}
hat_state = {
    HAT_Y_AXIS: 0,
}
steer_angle = CENTER_ANGLE
last_base_state = None

# =========================================================
# GPIO SETUP
# =========================================================
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError(
        "Could not connect to pigpio daemon. Start it with: sudo systemctl start pigpiod"
    )

for pin in (OUT1, OUT2, OUT3):
    pi.set_mode(pin, pigpio.OUTPUT)

# Servo angle-to-pulse helper (direct pigpio, no gpiozero caching)
SERVO_ANGLE_MIN = -90
SERVO_ANGLE_MAX = 90
SERVO_PULSE_MIN_US = int(SERVO_MIN_PULSE * 1_000_000)  # 500
SERVO_PULSE_MAX_US = int(SERVO_MAX_PULSE * 1_000_000)  # 2500


def _angle_to_pulse_us(angle: float) -> int:
    """Map angle in [-90, 90] to pulse width in microseconds."""
    ratio = (angle - SERVO_ANGLE_MIN) / (SERVO_ANGLE_MAX - SERVO_ANGLE_MIN)
    return int(SERVO_PULSE_MIN_US + ratio * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US))


def _write_servo(angle: float) -> None:
    """Send pulse directly via pigpio — no caching, no skipped writes."""
    pulse_us = _angle_to_pulse_us(angle)
    pi.set_servo_pulsewidth(SERVO_PIN, pulse_us)


def _servo_off() -> None:
    """Stop PWM pulses to servo (relaxes it)."""
    pi.set_servo_pulsewidth(SERVO_PIN, 0)


# =========================================================
# HELPERS
# =========================================================

def angle_bounds(left_limit: float, right_limit: float) -> tuple[float, float]:
    return (left_limit, right_limit) if left_limit <= right_limit else (right_limit, left_limit)


def clamp_angle(value: float, left_limit: float, right_limit: float) -> float:
    lower, upper = angle_bounds(left_limit, right_limit)
    return max(lower, min(upper, value))

    
def clamp(value, lo, hi):
    return clamp_angle(value, lo, hi)


def log(message):
    print(message)


def find_gamepad():
    if GAMEPAD_DEVICE:
        return InputDevice(GAMEPAD_DEVICE)

    candidates = []
    for path in list_devices():
        device = InputDevice(path)
        name = (device.name or "").lower()
        if ecodes.EV_ABS not in device.capabilities():
            continue
        candidates.append((path, device.name or "Unknown input"))
        if any(hint in name for hint in GAMEPAD_NAME_HINTS):
            return device

    if candidates:
        details = ", ".join(f"{name} ({path})" for path, name in candidates)
        raise RuntimeError(
            "Could not find Edra gamepad automatically. "
            f"Detected EV_ABS devices: {details}. Set GAMEPAD_DEVICE manually."
        )

    raise RuntimeError(
        "No joystick/gamepad device found. Connect the Edra controller or set GAMEPAD_DEVICE."
    )


def normalize_axis(device, axis_code, raw_value):
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


def apply_deadzone(value, deadzone):
    if abs(value) < deadzone:
        return 0.0
    return value


def adjust_center(delta):
    global CENTER_ANGLE
    CENTER_ANGLE = clamp(CENTER_ANGLE + delta, LEFT_LIMIT, RIGHT_LIMIT)
    log(f"CENTER_ANGLE: {CENTER_ANGLE}")
    steer_center()


# =========================================================
# BASE CONTROL
# =========================================================
def set_base(b1, b2, b3, label=None):
    global last_base_state
    state = (b1, b2, b3)

    pi.write(OUT1, 1 if b1 else 0)
    pi.write(OUT2, 1 if b2 else 0)
    pi.write(OUT3, 1 if b3 else 0)

    if state != last_base_state:
        if label:
            log(f"BASE: {label} -> {state}")
        else:
            log(f"BASE: {state}")
        last_base_state = state


def stop_base():
    set_base(0, 0, 0, "STOP")


def forward():
    set_base(0, 1, 0, "FORWARD")


def backward():
    set_base(0, 0, 1, "BACKWARD")


def lock_base():
    set_base(1, 0, 1, "LOCK")


def unlock_base():
    set_base(1, 1, 0, "UNLOCK")


# =========================================================
# STEERING
# =========================================================
def apply_steering(target_angle):
    global steer_angle

    steer_angle = clamp(target_angle, LEFT_LIMIT, RIGHT_LIMIT)
    _write_servo(steer_angle)
    log(f"STEER: {steer_angle:.1f} deg | CENTER: {CENTER_ANGLE} deg")


def steer_center():
    apply_steering(CENTER_ANGLE)


def steer_from_axis(axis_value):
    if INVERT_STEER_AXIS:
        axis_value = -axis_value

    axis_value = apply_deadzone(axis_value, STEER_DEADZONE)

    if axis_value == 0.0:
        steer_center()
        return

    if axis_value < 0:
        target_angle = CENTER_ANGLE + axis_value * (CENTER_ANGLE - LEFT_LIMIT)
    else:
        target_angle = CENTER_ANGLE + axis_value * (RIGHT_LIMIT - CENTER_ANGLE)

    apply_steering(target_angle)


# =========================================================
# CLEANUP
# =========================================================
def cleanup():
    global gamepad

    try:
        stop_base()
    except Exception:
        pass

    try:
        steer_center()
    except Exception:
        pass

    if gamepad is not None:
        try:
            gamepad.ungrab()
        except Exception:
            pass

    try:
        _servo_off()
    except Exception:
        pass

    try:
        pi.stop()
    except Exception:
        pass


def signal_handler(sig, frame):
    del sig, frame
    global running
    running = False


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


# =========================================================
# INPUT UPDATE
# =========================================================
def update_button_state(event):
    global running

    if event.type != ecodes.EV_KEY:
        return

    code = event.code

    if event.value == 1:
        pressed_buttons.add(code)

        if code == BUTTON_CENTER_PLUS:
            adjust_center(1)
        elif code == BUTTON_CENTER_MINUS:
            adjust_center(-1)
        elif code == BUTTON_QUIT:
            running = False
    elif event.value == 0:
        pressed_buttons.discard(code)


def update_axis_state(device, event):
    if event.type != ecodes.EV_ABS:
        return

    if event.code in axis_state:
        axis_state[event.code] = normalize_axis(device, event.code, event.value)
        return

    if event.code == HAT_Y_AXIS and event.value != hat_state[HAT_Y_AXIS]:
        hat_state[HAT_Y_AXIS] = event.value
        if event.value == -1:
            adjust_center(1)
        elif event.value == 1:
            adjust_center(-1)


def process_controls():
    drive_value = axis_state[DRIVE_AXIS]

    if INVERT_DRIVE_AXIS:
        drive_value = -drive_value

    drive_value = apply_deadzone(drive_value, DRIVE_DEADZONE)

    if BUTTON_LOCK in pressed_buttons:
        lock_base()
        return

    if BUTTON_UNLOCK in pressed_buttons:
        unlock_base()
        return

    if BUTTON_STOP in pressed_buttons:
        stop_base()
    elif drive_value < 0:
        forward()
    elif drive_value > 0:
        backward()
    else:
        stop_base()

    if BUTTON_CENTER in pressed_buttons:
        steer_center()
    else:
        steer_from_axis(axis_state[STEER_AXIS])


# =========================================================
# MAIN
# =========================================================
def main():
    global gamepad

    gamepad = find_gamepad()

    print("=== EDRA JOYSTICK DRIVE MODE ===")
    print(f"Gamepad: {gamepad.name} ({gamepad.path})")
    print("Right stick X = steering (servo)")
    print("Left stick Y = forward/backward (base)")
    print("A = stop")
    print("B = center steering")
    print("LB = lock")
    print("RB = unlock")
    print("Y = center angle +1")
    print("X = center angle -1")
    print("D-pad up/down = center angle +/-1")
    print("START = quit")
    print("Ctrl+C = emergency exit")
    print("")
    print(
        f"Steering center={CENTER_ANGLE}, left={LEFT_LIMIT}, right={RIGHT_LIMIT}, "
        f"steer_deadzone={STEER_DEADZONE}, drive_deadzone={DRIVE_DEADZONE}"
    )
    print("")

    steer_center()
    stop_base()

    try:
        gamepad.grab()

        while running:
            try:
                for event in gamepad.read():
                    update_button_state(event)
                    update_axis_state(gamepad, event)
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
