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
    import paho.mqtt as mqtt
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

# MQTT state
mqtt_client: Any = None
mqtt_connected = False
