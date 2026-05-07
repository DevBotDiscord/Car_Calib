"""Main control loop: process_controls + gamepad/keyboard input handlers."""

from __future__ import annotations

import time

from evdev import InputDevice, ecodes

from . import config
from .base import backward, forward, lock_base, stop_base, unlock_base
from .config import _clamp as clamp, _apply_deadzone as apply_deadzone
from .steering import (
    adjust_center,
    apply_steering,
    release_servo,
    remote_control_active,
    steer_center,
    steer_from_gamepad_axis,
    steer_left_step,
    steer_right_step,
)
from .imu import imu_reset_for_cruise, poll_imu, set_home


# ---------------------------------------------------------------------------
# manual-override helpers
# ---------------------------------------------------------------------------
def controller_remote_steer_only_enabled() -> bool:
    return config.controller_remote_steer_only


def log_controller_mode_transition(remote_steer_only: bool) -> None:
    if remote_steer_only == config.last_controller_remote_steer_only:
        return
    if remote_steer_only:
        print("MODE: drive local only, steering source=MQTT")
    else:
        print("MODE: steer+drive local from controller")
    config.last_controller_remote_steer_only = remote_steer_only


def activate_manual_override(source: str, now: float) -> None:
    config.manual_override_until = now + config.MANUAL_STEER_HOLD
    config.manual_override_source = source


def clear_manual_override() -> None:
    config.manual_override_until = 0.0
    config.manual_override_source = None


# ---------------------------------------------------------------------------
# process_controls — main steering priority loop
# ---------------------------------------------------------------------------
def process_controls() -> None:
    now = time.monotonic()

    # --- cruise timeout ---
    if config.cruise_active and (now - config.cruise_start_time) >= config.CRUISE_DURATION_S:
        config.cruise_active = False
        config.controller_remote_steer_only = config.cruise_prev_remote_steer_only
        stop_base()
        print("CRUISE: finished (30s elapsed)")

    # --- IMU heading-hold (B toggle) ---
    if config.imu_steer_active and config.imu_active:
        _, error_deg = poll_imu()
        target = clamp(config.imu_home_steer_angle + error_deg, config.LEFT_LIMIT, config.RIGHT_LIMIT)
        state = "STRAIGHT" if abs(error_deg) <= 1.0 else ("RIGHT" if error_deg > 0 else "LEFT")
        apply_steering(target, f"IMU-{state}")
        return

    # --- cruise: pure MQTT steer ---
    if config.cruise_active:
        if remote_control_active(now) and config.remote_servo_angle is not None:
            apply_steering(config.remote_servo_angle, "REMOTE")
        else:
            apply_steering(config.CENTER_ANGLE, "CRUISE-IDLE")
        return

    # --- keyboard base ---
    if "KEY_L" in config.pressed_keys:
        lock_base()
        return
    if "KEY_U" in config.pressed_keys:
        unlock_base()
        return

    has_w = "KEY_W" in config.pressed_keys
    has_s = "KEY_S" in config.pressed_keys
    has_x = "KEY_X" in config.pressed_keys
    keyboard_base_active = has_x or has_w or has_s
    if has_x or (has_w and has_s):
        stop_base()
    elif has_w:
        forward()
    elif has_s:
        backward()
    else:
        drive_value = config.axis_state[config.DRIVE_AXIS]
        if config.INVERT_DRIVE_AXIS:
            drive_value = -drive_value
        drive_value = apply_deadzone(drive_value, config.GAMEPAD_DRIVE_DEADZONE)

        if config.BUTTON_LOCK in config.pressed_buttons:
            lock_base()
            return
        if config.BUTTON_UNLOCK in config.pressed_buttons:
            unlock_base()
            return
        if config.BUTTON_STOP in config.pressed_buttons:
            stop_base()
        elif drive_value < 0:
            forward()
        elif drive_value > 0:
            backward()
        elif keyboard_base_active:
            stop_base()
        else:
            stop_base()

    # --- keyboard steer ---
    has_a = "KEY_A" in config.pressed_keys
    has_d = "KEY_D" in config.pressed_keys
    has_c = "KEY_C" in config.pressed_keys
    remote_only = controller_remote_steer_only_enabled()
    log_controller_mode_transition(remote_only)

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

    if remote_only and config.manual_override_source == "GAMEPAD":
        clear_manual_override()

    if now < config.manual_override_until:
        return

    # --- gamepad steer ---
    if not remote_only:
        if steer_from_gamepad_axis(config.axis_state[config.STEER_AXIS], "GAMEPAD"):
            activate_manual_override("GAMEPAD", now)
            return

    # --- remote / idle steer ---
    if remote_only:
        if remote_control_active(now):
            apply_steering(config.remote_servo_angle if config.remote_servo_angle is not None else config.CENTER_ANGLE, "REMOTE")
        elif config.SERVO_RELEASE_IDLE:
            release_servo("IDLE")
        else:
            apply_steering(config.CENTER_ANGLE, "IDLE")
    elif config.SERVO_RELEASE_IDLE:
        release_servo("IDLE")
    else:
        apply_steering(config.CENTER_ANGLE, "IDLE")


# ---------------------------------------------------------------------------
# input handlers
# ---------------------------------------------------------------------------
def update_key_state(event) -> None:
    if event.type != ecodes.EV_KEY:
        return

    keycode = ecodes.KEY.get(event.code)
    if not keycode:
        return

    keys = keycode if isinstance(keycode, list) else [keycode]

    if event.value in (1, 2):
        for key in keys:
            config.pressed_keys.add(key)
            if event.value == 1:
                if key == "KEY_1":
                    adjust_center(1, "KEYBOARD")
                elif key == "KEY_2":
                    adjust_center(-1, "KEYBOARD")
                elif key == "KEY_Q":
                    config.running = False
                elif key == config.CRUISE_KEY:
                    from .cruise import cancel_cruise, start_cruise
                    if config.cruise_active:
                        cancel_cruise()
                    else:
                        start_cruise(time.monotonic())
    elif event.value == 0:
        for key in keys:
            config.pressed_keys.discard(key)


def update_gamepad_button_state(event) -> None:
    if event.type != ecodes.EV_KEY:
        return

    code = event.code
    if event.value == 1:
        config.pressed_buttons.add(code)
        if code == config.BUTTON_REMOTE_STEER_ONLY:
            config.controller_remote_steer_only = not config.controller_remote_steer_only
        elif code == config.BUTTON_CRUISE:
            from .cruise import cancel_cruise, start_cruise
            if config.cruise_active:
                cancel_cruise()
            else:
                start_cruise(time.monotonic())
        elif config.BUTTON_CENTER_PLUS is not None and code == config.BUTTON_CENTER_PLUS:
            adjust_center(1, "GAMEPAD")
        elif code == config.BUTTON_CENTER_MINUS:
            adjust_center(-1, "GAMEPAD")
        elif code == config.BUTTON_QUIT:
            config.running = False
    elif event.value == 0:
        config.pressed_buttons.discard(code)

    # IMU mode toggle (B button) — fires on press, after button state updated
    if event.type == ecodes.EV_KEY and event.value == 1 and code == config.BUTTON_IMU_MODE:
        config.imu_steer_active = not config.imu_steer_active
        if config.imu_steer_active:
            imu_reset_for_cruise()
            set_home()
            print("IMU-MODE: ON (heading-hold active, steer from IMU)")
        else:
            print("IMU-MODE: OFF (back to normal steer)")


def update_gamepad_axis_state(device: InputDevice, event) -> None:
    if event.type != ecodes.EV_ABS:
        return

    if event.code in config.axis_state:
        config.axis_state[event.code] = _normalize_axis(device, event.code, event.value)
        return

    if event.code == config.HAT_Y_AXIS and event.value != config.hat_state[config.HAT_Y_AXIS]:
        config.hat_state[config.HAT_Y_AXIS] = event.value
        if event.value == -1:
            adjust_center(1, "GAMEPAD")
        elif event.value == 1:
            adjust_center(-1, "GAMEPAD")


def _normalize_axis(device: InputDevice, axis_code: int, raw_value: int) -> float:
    try:
        info = device.absinfo(axis_code)
    except Exception:
        return 0.0
    minimum, maximum = info.min, info.max
    center = (minimum + maximum) / 2.0
    half_range = (maximum - minimum) / 2.0
    if half_range <= 0:
        return 0.0
    return clamp((raw_value - center) / half_range, -1.0, 1.0)
