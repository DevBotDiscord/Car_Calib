"""All env-based configuration and global state for the RPi MQTT bridge."""

from __future__ import annotations

import os
from typing import Any

from evdev import InputDevice, ecodes

# -- optional imports --------------------------------------------------
try:
    from dotenv import load_dotenv
except ImportError:
    load_dotenv = None

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None  # type: ignore[assignment]

try:
    import pigpio
except ImportError:
    pigpio = None  # type: ignore[assignment]

try:
    from mpu6050 import mpu6050 as _mpu6050_class
except ImportError:
    _mpu6050_class = None

if load_dotenv is not None:
    load_dotenv()

# -- helpers -----------------------------------------------------------
try:
    from servo_bridge_common import clamp_angle
except ImportError:
    from scripts.servo_bridge_common import clamp_angle  # type: ignore[no-redef]


def _clamp(value: float, low: float, high: float) -> float:
    return clamp_angle(value, high, low)


def _apply_deadzone(value: float, deadzone: float) -> float:
    return 0.0 if abs(value) < deadzone else value


try:
    from servo_bridge_common import angle_within_limits as _angle_within_limits
except ImportError:
    from scripts.servo_bridge_common import angle_within_limits as _angle_within_limits  # type: ignore[no-redef]


# ======================================================================
# CONFIG (env vars)
# ======================================================================
KEYBOARD_DEVICE = os.getenv("KEYBOARD_DEVICE", "/dev/input/by-id/usb-YJX_CHIP_WirelessDevice-event-kbd")
GAMEPAD_DEVICE = os.getenv("GAMEPAD_DEVICE", "").strip()
GAMEPAD_NAME_HINTS = tuple(
    hint.strip().lower()
    for hint in os.getenv("GAMEPAD_NAME_HINTS", "edra,joystick,gamepad,controller,pad").split(",")
    if hint.strip()
)

MQTT_BROKER_HOST = os.getenv("MQTT_BROKER_HOST", "127.0.0.1")
MQTT_BROKER_PORT = int(os.getenv("MQTT_BROKER_PORT", "1883"))
MQTT_USERNAME = os.getenv("MQTT_USERNAME", "")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD", "")
MQTT_KEEPALIVE_S = int(os.getenv("MQTT_KEEPALIVE_S", "60"))
MQTT_SERVO_TOPIC = os.getenv("MQTT_SERVO_TOPIC", "car/servo/angle")
MQTT_STATUS_TOPIC = os.getenv("MQTT_STATUS_TOPIC", "car/status")
MQTT_CLIENT_ID = os.getenv("RPI_MQTT_BRIDGE_CLIENT_ID", f"rpi-mqtt-bridge-{os.getpid()}")

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
BUTTON_IMU_MODE = ecodes.BTN_EAST
BUTTON_LOCK = ecodes.BTN_TL
BUTTON_UNLOCK = ecodes.BTN_TR
BUTTON_REMOTE_STEER_ONLY = ecodes.BTN_NORTH
BUTTON_QUIT = ecodes.BTN_START
BUTTON_CENTER_PLUS = None
BUTTON_CENTER_MINUS = ecodes.BTN_WEST
BUTTON_CRUISE = ecodes.BTN_SELECT

CENTER_ANGLE = float(os.getenv("SERVO_CENTER_ANGLE", "-8"))
SERVO_MAX_ANGLE_DEG = float(os.getenv("SERVO_MAX_ANGLE_DEG", "45"))
LEFT_LIMIT = CENTER_ANGLE - SERVO_MAX_ANGLE_DEG
RIGHT_LIMIT = CENTER_ANGLE + SERVO_MAX_ANGLE_DEG
STEP = float(os.getenv("SERVO_STEP", "20"))

REMOTE_INPUT_MIN_ANGLE = float(os.getenv("REMOTE_INPUT_MIN_ANGLE", "60"))
REMOTE_INPUT_CENTER_ANGLE = float(os.getenv("REMOTE_INPUT_CENTER_ANGLE", "90"))
REMOTE_INPUT_MAX_ANGLE = float(os.getenv("REMOTE_INPUT_MAX_ANGLE", "120"))

SERVO_MIN_PULSE_US = int(round(float(os.getenv("SERVO_MIN_PULSE", "0.0005")) * 1000000))
SERVO_MAX_PULSE_US = int(round(float(os.getenv("SERVO_MAX_PULSE", "0.0025")) * 1000000))

REMOTE_SERVO_TIMEOUT = float(os.getenv("REMOTE_SERVO_TIMEOUT", "0.6"))
REMOTE_SERVO_HOLD_LAST = os.getenv("REMOTE_SERVO_HOLD_LAST", "true").strip().lower() in {
    "1", "true", "t", "yes", "y", "on",
}

MANUAL_STEER_HOLD = float(os.getenv("MANUAL_STEER_HOLD", "0.25"))
LOOP_DELAY = float(os.getenv("LOOP_DELAY", "0.01"))
STEER_DEADBAND_DEG = float(os.getenv("STEER_DEADBAND_DEG", "1.0"))
GAMEPAD_STEER_DEADZONE = float(os.getenv("GAMEPAD_STEER_DEADZONE", "0.12"))
GAMEPAD_DRIVE_DEADZONE = float(os.getenv("GAMEPAD_DRIVE_DEADZONE", "0.20"))

INVERT_STEER_AXIS = os.getenv("INVERT_STEER_AXIS", "false").strip().lower() in {
    "1", "true", "t", "yes", "y", "on",
}
INVERT_DRIVE_AXIS = os.getenv("INVERT_DRIVE_AXIS", "false").strip().lower() in {
    "1", "true", "t", "yes", "y", "on",
}
SERVO_RELEASE_IDLE = os.getenv("SERVO_RELEASE_IDLE", "true").strip().lower() in {
    "1", "true", "t", "yes", "y", "on",
}

CRUISE_DURATION_S = float(os.getenv("CRUISE_DURATION_S", "30"))
CRUISE_KEY = os.getenv("CRUISE_KEY", "KEY_ENTER")
CRUISE_STRAIGHT_FRAMES = int(os.getenv("CRUISE_STRAIGHT_FRAMES", "5"))

IMU_ENABLED = os.getenv("IMU_ENABLED", "true").strip().lower() in {
    "1", "true", "t", "yes", "y", "on",
}
IMU_STRAIGHT_THRESHOLD_DEG = float(os.getenv("IMU_STRAIGHT_THRESHOLD_DEG", "3.0"))
IMU_KP = float(os.getenv("IMU_KP", "0.12"))
IMU_GYRO_BIAS_SAMPLES = int(os.getenv("IMU_GYRO_BIAS_SAMPLES", "500"))

# ======================================================================
# GLOBAL STATE
# ======================================================================
running = True
pressed_keys: set[str] = set()
pressed_buttons: set[int] = set()
axis_state: dict[int, float] = {STEER_AXIS: 0.0, DRIVE_AXIS: 0.0}
hat_state: dict[int, int] = {HAT_Y_AXIS: 0}

steer_angle = CENTER_ANGLE
last_base_state: tuple[int, int, int] | None = None
last_steer_angle: float | None = None
last_steer_source: str | None = None

remote_servo_angle: float | None = None
remote_servo_updated_at = 0.0

manual_override_until = 0.0
manual_override_source: str | None = None

controller_remote_steer_only = False
last_controller_remote_steer_only = False

cruise_active = False
cruise_phase = "vision"
cruise_straight_count = 0
cruise_start_time = 0.0
cruise_prev_remote_steer_only = False

imu_steer_active = False
imu: Any = None
imu_yaw = 0.0
imu_home_yaw = 0.0
imu_home_steer_angle = CENTER_ANGLE
imu_gyro_z_bias = 0.0
imu_last_time = 0.0
imu_active = False

mqtt_client: Any = None
mqtt_connected = False
gpio: Any = None
keyboard: InputDevice | None = None
gamepad: InputDevice | None = None
