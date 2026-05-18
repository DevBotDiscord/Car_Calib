"""Minimal env-based configuration for RPi MQTT actuator bridge.

No gamepad, keyboard, IMU, cruise, or control logic — only MQTT subscribers
and GPIO writes for servo, base motor, and relay.
"""

from __future__ import annotations

import os
from typing import Any

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

if load_dotenv is not None:
    load_dotenv()

# -- helpers -----------------------------------------------------------
try:
    from servo_bridge_common import clamp_angle
except ImportError:
    from scripts.servo_bridge_common import clamp_angle  # type: ignore[no-redef]


def _clamp(value: float, low: float, high: float) -> float:
    return clamp_angle(value, high, low)


try:
    from servo_bridge_common import angle_within_limits as _angle_within_limits
except ImportError:
    from scripts.servo_bridge_common import angle_within_limits as _angle_within_limits  # type: ignore[no-redef]


# ======================================================================
# MQTT
# ======================================================================
MQTT_BROKER_HOST = os.getenv("MQTT_BROKER_HOST", "127.0.0.1")
MQTT_BROKER_PORT = int(os.getenv("MQTT_BROKER_PORT", "1883"))
MQTT_USERNAME = os.getenv("MQTT_USERNAME", "")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD", "")
MQTT_KEEPALIVE_S = int(os.getenv("MQTT_KEEPALIVE_S", "60"))
MQTT_SERVO_TOPIC = os.getenv("MQTT_SERVO_TOPIC", "car/servo/angle")
MQTT_BASE_COMMAND_TOPIC = os.getenv("MQTT_BASE_COMMAND_TOPIC", "car/base/command")
MQTT_RELAY_TOPIC = os.getenv("MQTT_RELAY_TOPIC", "car/relay")
MQTT_STATUS_TOPIC = os.getenv("MQTT_STATUS_TOPIC", "car/status")
MQTT_CLIENT_ID = os.getenv("RPI_MQTT_BRIDGE_CLIENT_ID", f"rpi-mqtt-bridge-{os.getpid()}")

# ======================================================================
# pigpio
# ======================================================================
PIGPIO_HOST = os.getenv("PIGPIO_HOST", "127.0.0.1")
PIGPIO_PORT = int(os.getenv("PIGPIO_PORT", "8888"))

# ======================================================================
# Servo
# ======================================================================
SERVO_PIN = int(os.getenv("SERVO_PIN", "12"))
CENTER_ANGLE = float(os.getenv("SERVO_CENTER_ANGLE", "-8"))
SERVO_MAX_ANGLE_DEG = float(os.getenv("SERVO_MAX_ANGLE_DEG", "45"))
LEFT_LIMIT = CENTER_ANGLE - SERVO_MAX_ANGLE_DEG
RIGHT_LIMIT = CENTER_ANGLE + SERVO_MAX_ANGLE_DEG
SERVO_MIN_PULSE_US = int(round(float(os.getenv("SERVO_MIN_PULSE", "0.0005")) * 1000000))
SERVO_MAX_PULSE_US = int(round(float(os.getenv("SERVO_MAX_PULSE", "0.0025")) * 1000000))
STEER_DEADBAND_DEG = float(os.getenv("STEER_DEADBAND_DEG", "1.0"))
SERVO_RELEASE_IDLE = os.getenv("SERVO_RELEASE_IDLE", "true").strip().lower() in {
    "1", "true", "t", "yes", "y", "on",
}
REMOTE_SERVO_HOLD_LAST = os.getenv("REMOTE_SERVO_HOLD_LAST", "true").strip().lower() in {
    "1", "true", "t", "yes", "y", "on",
}
REMOTE_SERVO_TIMEOUT = float(os.getenv("REMOTE_SERVO_TIMEOUT", "0.6"))

# remote angle input range (for backward-compat angle mapping)
REMOTE_INPUT_MIN_ANGLE = float(os.getenv("REMOTE_INPUT_MIN_ANGLE", "60"))
REMOTE_INPUT_CENTER_ANGLE = float(os.getenv("REMOTE_INPUT_CENTER_ANGLE", "90"))
REMOTE_INPUT_MAX_ANGLE = float(os.getenv("REMOTE_INPUT_MAX_ANGLE", "120"))

# ======================================================================
# Input devices
# ======================================================================
KEYBOARD_DEVICE = os.getenv("KEYBOARD_DEVICE", "")
GAMEPAD_DEVICE = os.getenv("GAMEPAD_DEVICE", "")
GAMEPAD_NAME_HINTS = os.getenv("GAMEPAD_NAME_HINTS", "edra,joystick,gamepad,controller,pad")

# Axis codes (evdev ecodes)
STEER_AXIS = int(os.getenv("STEER_AXIS", "3"))  # ABS_RX
DRIVE_AXIS = int(os.getenv("DRIVE_AXIS", "1"))  # ABS_Y
HAT_Y_AXIS = int(os.getenv("HAT_Y_AXIS", "17"))  # ABS_HAT0Y

# Button codes (evdev ecodes)
BUTTON_STOP = int(os.getenv("BUTTON_STOP", "304"))  # BTN_SOUTH = A
BUTTON_SQUARE = int(os.getenv("BUTTON_SQUARE", "305"))  # BTN_EAST = B
BUTTON_LOCK = int(os.getenv("BUTTON_LOCK", "310"))  # BTN_TL = LB
BUTTON_UNLOCK = int(os.getenv("BUTTON_UNLOCK", "311"))  # BTN_TR = RB
BUTTON_REMOTE_STEER = int(os.getenv("BUTTON_REMOTE_STEER", "307"))  # BTN_NORTH = Y
BUTTON_QUIT = int(os.getenv("BUTTON_QUIT", "315"))  # BTN_START
BUTTON_CRUISE = int(os.getenv("BUTTON_CRUISE", "314"))  # BTN_SELECT
BUTTON_CENTER_MINUS = int(os.getenv("BUTTON_CENTER_MINUS", "306"))  # BTN_WEST = X

# Gamepad config
GAMEPAD_STEER_DEADZONE = float(os.getenv("GAMEPAD_STEER_DEADZONE", "0.12"))
GAMEPAD_DRIVE_DEADZONE = float(os.getenv("GAMEPAD_DRIVE_DEADZONE", "0.20"))
INVERT_STEER_AXIS = os.getenv("INVERT_STEER_AXIS", "false").strip().lower() in {"1", "true", "t", "yes", "y", "on"}
INVERT_DRIVE_AXIS = os.getenv("INVERT_DRIVE_AXIS", "false").strip().lower() in {"1", "true", "t", "yes", "y", "on"}

# Steering config
SERVO_STEP = float(os.getenv("SERVO_STEP", "20.0"))
MANUAL_STEER_HOLD = float(os.getenv("MANUAL_STEER_HOLD", "0.25"))

# Cruise control
CRUISE_DURATION_S = float(os.getenv("CRUISE_DURATION_S", "30.0"))

# Square pattern
SQUARE_STRAIGHT_DURATION = float(os.getenv("SQUARE_STRAIGHT_DURATION_S", "5.0"))
SQUARE_TURN_DURATION = float(os.getenv("SQUARE_TURN_DURATION_S", "1.0"))

# Relay blink
RELAY_BLINK_INTERVAL_S = float(os.getenv("RELAY_BLINK_INTERVAL_S", "0.12"))

# ======================================================================
# Base motor
# ======================================================================
OUT1 = int(os.getenv("BASE_OUT1", "17"))
OUT2 = int(os.getenv("BASE_OUT2", "27"))
OUT3 = int(os.getenv("BASE_OUT3", "22"))

# ======================================================================
# Relay
# ======================================================================
RELAY_PIN = int(os.getenv("RELAY_PIN", "5"))

# ======================================================================
# GLOBAL STATE (minimal)
# ======================================================================
running = True
gpio: Any = None

# Steering state (for deadband + status)
steer_angle = CENTER_ANGLE
last_steer_angle: float | None = None
last_steer_source: str | None = None

# Relay state
relay_on = False
last_base_state: tuple[int, int, int] | None = None

# MQTT state
mqtt_client: Any = None
mqtt_connected = False

# Control state (for gamepad override)
manual_override_active = False
current_route_mode = "AUTO"
