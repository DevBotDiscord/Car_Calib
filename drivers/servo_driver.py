"""Drivers module: servo angle translation layer.

Provides an abstracted interface for sending angle commands to a servo
motor. Supports two transports:

* Local hardware stub (override ``_write_angle`` for PCA9685, GPIO, etc.).
* MQTT publish to the RPi bridge (angle is published as ``int`` degrees).

The MQTT transport is enabled with ``DRIVER_SERVO_MQTT_ENABLED=true``.
When enabled, every ``send_angle()`` call also publishes the rounded
integer angle to ``MQTT_SERVO_TOPIC``.
"""

from __future__ import annotations

import json
import logging
import os
from typing import Any

from config.settings import (
    DRIVER_SERVO_ANGLE_MAX,
    DRIVER_SERVO_ANGLE_MIN,
    DRIVER_SERVO_CHANNEL,
    DRIVER_SERVO_MQTT_ENABLED,
    DRIVER_SERVO_PULSE_MAX_US,
    DRIVER_SERVO_PULSE_MIN_US,
    MQTT_BROKER_HOST,
    MQTT_BROKER_PORT,
    MQTT_CLIENT_ID_PREFIX,
    MQTT_KEEPALIVE_S,
    MQTT_PASSWORD,
    MQTT_SERVO_TOPIC,
    MQTT_USERNAME,
    SERVO_CENTER_ANGLE,
)

try:
    import paho.mqtt.client as mqtt
except ImportError:  # pragma: no cover - optional dependency
    mqtt = None  # type: ignore[assignment]

try:
    from runtime import script_lock as _script_lock
except Exception:  # noqa: BLE001 - script_lock optional
    _script_lock = None  # type: ignore[assignment]

logger = logging.getLogger(__name__)

_PULSE_MIN_US: int = DRIVER_SERVO_PULSE_MIN_US
_PULSE_MAX_US: int = DRIVER_SERVO_PULSE_MAX_US
_ANGLE_MIN: float = DRIVER_SERVO_ANGLE_MIN
_ANGLE_MAX: float = DRIVER_SERVO_ANGLE_MAX


class ServoDriver:
    """Translates angle commands into PWM signals for a servo motor.

    When ``DRIVER_SERVO_MQTT_ENABLED`` is true, the driver also publishes
    the integer-rounded angle to MQTT for the RPi bridge to consume.
    """

    def __init__(
        self,
        channel: int = DRIVER_SERVO_CHANNEL,
        center_angle: float = SERVO_CENTER_ANGLE,
        pulse_min_us: int = _PULSE_MIN_US,
        pulse_max_us: int = _PULSE_MAX_US,
        mqtt_enabled: bool = DRIVER_SERVO_MQTT_ENABLED,
    ) -> None:
        self._channel = channel
        self._center_angle = center_angle
        self._pulse_min_us = pulse_min_us
        self._pulse_max_us = pulse_max_us
        self._mqtt_enabled = mqtt_enabled
        self._mqtt_client: Any | None = None

        if self._mqtt_enabled:
            self._setup_mqtt()

    def _setup_mqtt(self) -> None:
        if mqtt is None:
            logger.warning(
                "ServoDriver: paho-mqtt not installed, MQTT publish disabled."
            )
            self._mqtt_enabled = False
            return
        client_id = f"{MQTT_CLIENT_ID_PREFIX}-servo-{os.getpid()}"
        callback_api_version = getattr(mqtt, "CallbackAPIVersion", None)
        if callback_api_version is not None:
            client = mqtt.Client(
                callback_api_version=callback_api_version.VERSION1,
                client_id=client_id,
            )
        else:
            client = mqtt.Client(client_id=client_id)
        if MQTT_USERNAME:
            client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        client.reconnect_delay_set(min_delay=1, max_delay=5)
        client.connect_async(MQTT_BROKER_HOST, MQTT_BROKER_PORT, keepalive=MQTT_KEEPALIVE_S)
        client.loop_start()
        self._mqtt_client = client
        logger.info(
            "ServoDriver: MQTT publish to %s:%d topic=%s",
            MQTT_BROKER_HOST,
            MQTT_BROKER_PORT,
            MQTT_SERVO_TOPIC,
        )

    def _angle_to_pulse(self, angle: float) -> int:
        clamped = max(_ANGLE_MIN, min(_ANGLE_MAX, angle))
        if _ANGLE_MAX == _ANGLE_MIN:
            return self._pulse_min_us
        pulse = (
            self._pulse_min_us
            + ((clamped - _ANGLE_MIN) / (_ANGLE_MAX - _ANGLE_MIN))
            * (self._pulse_max_us - self._pulse_min_us)
        )
        return int(round(pulse))

    def send_angle(self, angle: float) -> None:
        """Send *angle* to the servo hardware and publish over MQTT (as int)."""
        clamped = max(_ANGLE_MIN, min(_ANGLE_MAX, angle))
        pulse_us = self._angle_to_pulse(clamped)
        logger.debug(
            "ServoDriver: channel=%d angle=%.2f deg pulse=%d us",
            self._channel,
            clamped,
            pulse_us,
        )

        # While a route-script step is pinning the servo, do not publish
        # the vision PID's target — it would fight the script's republish
        # on the same MQTT topic.
        if _script_lock is not None and _script_lock.is_pinned():
            return

        if self._mqtt_enabled and self._mqtt_client is not None:
            self._publish_mqtt_angle(clamped)
            return

        self._write_angle(clamped, pulse_us)

    def _publish_mqtt_angle(self, angle: float) -> None:
        """Publish the rounded integer angle to the MQTT bridge topic."""
        angle_int = int(round(angle))
        payload = json.dumps({"angle": angle_int})
        try:
            self._mqtt_client.publish(MQTT_SERVO_TOPIC, payload, qos=0)
        except Exception as exc:  # noqa: BLE001
            logger.error("ServoDriver: MQTT publish failed: %s", exc)

    def center(self) -> None:
        """Return the servo to the neutral center position."""
        logger.info(
            "ServoDriver: centering servo (channel=%d, angle=%.2f deg).",
            self._channel,
            self._center_angle,
        )
        self.send_angle(self._center_angle)

    def close(self) -> None:
        """Disconnect MQTT client (no-op when MQTT disabled)."""
        if self._mqtt_client is None:
            return
        try:
            self._mqtt_client.disconnect()
        except Exception:  # noqa: BLE001
            pass
        try:
            self._mqtt_client.loop_stop()
        except Exception:  # noqa: BLE001
            pass
        self._mqtt_client = None

    def _write_angle(self, angle: float, pulse_us: int) -> None:
        """Send the PWM pulse to the local hardware. Override for real HW."""
        logger.debug(
            "ServoDriver._write_angle stub (channel=%d, angle=%.2f deg, pulse=%d us).",
            self._channel,
            angle,
            pulse_us,
        )
