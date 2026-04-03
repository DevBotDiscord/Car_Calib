"""Drivers module: servo angle translation layer.

Provides an abstracted interface for sending angle commands to a servo
motor connected via a PCA9685 I2C PWM board, directly to Jetson GPIO,
or over a TCP bridge to a Raspberry Pi.
"""

from __future__ import annotations

import importlib
import json
import logging
import os
import socket
import time
from typing import Any

from config.settings import (
    DRIVER_SERVO_ANGLE_MAX,
    DRIVER_SERVO_ANGLE_MIN,
    DRIVER_SERVO_BRIDGE_CONNECT_TIMEOUT_S,
    DRIVER_SERVO_BRIDGE_ENABLED,
    DRIVER_SERVO_BRIDGE_HOST,
    DRIVER_SERVO_BRIDGE_MIN_ANGLE_DELTA,
    DRIVER_SERVO_BRIDGE_MIN_SEND_INTERVAL_S,
    DRIVER_SERVO_BRIDGE_PORT,
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

logger = logging.getLogger(__name__)

_PULSE_MIN_US: int = DRIVER_SERVO_PULSE_MIN_US
_PULSE_MAX_US: int = DRIVER_SERVO_PULSE_MAX_US
_ANGLE_MIN: float = DRIVER_SERVO_ANGLE_MIN
_ANGLE_MAX: float = DRIVER_SERVO_ANGLE_MAX


class ServoDriver:
    """Translates angle commands into PWM signals for a servo motor."""

    def __init__(
        self,
        channel: int = DRIVER_SERVO_CHANNEL,
        center_angle: float = SERVO_CENTER_ANGLE,
        pulse_min_us: int = _PULSE_MIN_US,
        pulse_max_us: int = _PULSE_MAX_US,
        mqtt_enabled: bool = DRIVER_SERVO_MQTT_ENABLED,
        mqtt_host: str = MQTT_BROKER_HOST,
        mqtt_port: int = MQTT_BROKER_PORT,
        mqtt_username: str = MQTT_USERNAME,
        mqtt_password: str = MQTT_PASSWORD,
        mqtt_keepalive_s: int = MQTT_KEEPALIVE_S,
        mqtt_topic: str = MQTT_SERVO_TOPIC,
        mqtt_client_id_prefix: str = MQTT_CLIENT_ID_PREFIX,
        bridge_enabled: bool = DRIVER_SERVO_BRIDGE_ENABLED,
        bridge_host: str = DRIVER_SERVO_BRIDGE_HOST,
        bridge_port: int = DRIVER_SERVO_BRIDGE_PORT,
        bridge_connect_timeout_s: float = DRIVER_SERVO_BRIDGE_CONNECT_TIMEOUT_S,
        bridge_min_send_interval_s: float = DRIVER_SERVO_BRIDGE_MIN_SEND_INTERVAL_S,
        bridge_min_angle_delta: float = DRIVER_SERVO_BRIDGE_MIN_ANGLE_DELTA,
    ) -> None:
        self._channel = channel
        self._center_angle = center_angle
        self._pulse_min_us = pulse_min_us
        self._pulse_max_us = pulse_max_us
        self._mqtt_enabled = mqtt_enabled
        self._mqtt_host = mqtt_host
        self._mqtt_port = mqtt_port
        self._mqtt_username = mqtt_username
        self._mqtt_password = mqtt_password
        self._mqtt_keepalive_s = mqtt_keepalive_s
        self._mqtt_topic = mqtt_topic
        self._mqtt_client_id = f"{mqtt_client_id_prefix}-servo-{os.getpid()}"
        self._mqtt_client: Any | None = None
        self._bridge_enabled = bridge_enabled
        self._bridge_host = bridge_host
        self._bridge_port = bridge_port
        self._bridge_connect_timeout_s = bridge_connect_timeout_s
        self._bridge_min_send_interval_s = max(0.0, bridge_min_send_interval_s)
        self._bridge_min_angle_delta = max(0.0, bridge_min_angle_delta)
        self._bridge_socket: socket.socket | None = None
        self._bridge_last_send_at: float | None = None
        self._bridge_last_angle: float | None = None
        self._bridge_pending: tuple[str, float, int] | None = None

    def _angle_to_pulse(self, angle: float) -> int:
        """Convert *angle* to a PWM pulse width in microseconds."""
        clamped = max(_ANGLE_MIN, min(_ANGLE_MAX, angle))
        if _ANGLE_MAX == _ANGLE_MIN:
            raise ValueError("Servo angle range cannot be zero.")

        pulse = (
            self._pulse_min_us
            + ((clamped - _ANGLE_MIN) / (_ANGLE_MAX - _ANGLE_MIN))
            * (self._pulse_max_us - self._pulse_min_us)
        )
        return int(round(pulse))

    def send_angle(self, angle: float, force: bool = False) -> None:
        """Send *angle* to the servo hardware.

        When bridge mode is enabled, commands are throttled so the sender
        does not flood the Raspberry Pi with near-identical updates.
        """
        clamped_angle = max(_ANGLE_MIN, min(_ANGLE_MAX, angle))
        pulse_us = self._angle_to_pulse(clamped_angle)
        logger.debug(
            "ServoDriver: channel=%d angle=%.2f deg pulse=%d us",
            self._channel,
            clamped_angle,
            pulse_us,
        )

        if self._mqtt_enabled:
            self._publish_mqtt_angle(clamped_angle)
            return

        if self._bridge_enabled:
            self._send_bridge_command("angle", clamped_angle, pulse_us, force=force)
            return

        self._write_angle(clamped_angle, pulse_us)

    def center(self) -> None:
        """Return the servo to the neutral center position."""
        logger.info(
            "ServoDriver: centering servo (channel=%d, angle=%.2f deg).",
            self._channel,
            self._center_angle,
        )

        if self._mqtt_enabled:
            self._publish_mqtt_angle(self._center_angle)
            return

        if self._bridge_enabled:
            pulse_us = self._angle_to_pulse(self._center_angle)
            self._send_bridge_command(
                "center",
                self._center_angle,
                pulse_us,
                force=True,
            )
            return

        self.send_angle(self._center_angle, force=True)

    def close(self) -> None:
        """Release any open bridge socket or MQTT client."""
        if self._mqtt_client is not None:
            try:
                loop_stop = getattr(self._mqtt_client, "loop_stop", None)
                if callable(loop_stop):
                    loop_stop()
                disconnect = getattr(self._mqtt_client, "disconnect", None)
                if callable(disconnect):
                    disconnect()
            finally:
                self._mqtt_client = None

        if self._bridge_socket is None:
            return

        try:
            self._bridge_socket.close()
        finally:
            self._bridge_socket = None
            self._bridge_last_send_at = None
            self._bridge_last_angle = None
            self._bridge_pending = None

    def _get_mqtt_client(self) -> Any:
        if self._mqtt_client is not None:
            return self._mqtt_client

        try:
            mqtt = importlib.import_module("paho.mqtt.client")
        except ImportError as exc:
            raise RuntimeError(
                "paho-mqtt is required for MQTT servo publishing. Install requirements.txt first."
            ) from exc

        client_ctor = getattr(mqtt, "Client")
        callback_api_version = getattr(mqtt, "CallbackAPIVersion", None)
        if callback_api_version is not None:
            client = client_ctor(
                callback_api_version=callback_api_version.VERSION1,
                client_id=self._mqtt_client_id,
            )
        else:
            client = client_ctor(client_id=self._mqtt_client_id)

        if self._mqtt_username:
            client.username_pw_set(self._mqtt_username, self._mqtt_password)

        client.connect(self._mqtt_host, self._mqtt_port, keepalive=self._mqtt_keepalive_s)
        loop_start = getattr(client, "loop_start", None)
        if callable(loop_start):
            loop_start()

        logger.info(
            "ServoDriver MQTT connected to %s:%d topic=%s.",
            self._mqtt_host,
            self._mqtt_port,
            self._mqtt_topic,
        )
        self._mqtt_client = client
        return client

    def _publish_mqtt_angle(self, angle: float) -> None:
        client = self._get_mqtt_client()
        payload = f"{angle:.4f}"

        try:
            result = client.publish(self._mqtt_topic, payload)
            wait_for_publish = getattr(result, "wait_for_publish", None)
            if callable(wait_for_publish):
                wait_for_publish()
            rc = getattr(result, "rc", 0)
            if rc not in (0, None):
                raise OSError(f"MQTT publish failed with rc={rc}")
        except Exception as exc:  # noqa: BLE001
            self.close()
            raise OSError(f"MQTT publish failed: {exc}") from exc

    def _connect_bridge(self) -> socket.socket:
        if self._bridge_socket is not None:
            return self._bridge_socket

        sock = socket.create_connection(
            (self._bridge_host, self._bridge_port),
            timeout=self._bridge_connect_timeout_s,
        )
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._bridge_socket = sock
        logger.info(
            "ServoDriver bridge connected to %s:%d.",
            self._bridge_host,
            self._bridge_port,
        )
        return sock

    def _send_bridge_command(
        self,
        command_type: str,
        angle: float,
        pulse_us: int,
        *,
        force: bool,
    ) -> None:
        now = time.monotonic()

        if not force and self._bridge_last_send_at is not None:
            elapsed = now - self._bridge_last_send_at
            if elapsed < self._bridge_min_send_interval_s:
                self._bridge_pending = (command_type, angle, pulse_us)
                logger.debug(
                    "ServoDriver bridge throttled: angle=%.2f deg elapsed=%.4fs min=%.4fs",
                    angle,
                    elapsed,
                    self._bridge_min_send_interval_s,
                )
                return

        if not force and self._bridge_pending is not None:
            pending_type, pending_angle, pending_pulse_us = self._bridge_pending
            self._bridge_pending = None
            if command_type == pending_type and angle == self._bridge_last_angle:
                command_type = pending_type
                angle = pending_angle
                pulse_us = pending_pulse_us

        if (
            not force
            and self._bridge_last_angle is not None
            and abs(angle - self._bridge_last_angle) < self._bridge_min_angle_delta
        ):
            logger.debug(
                "ServoDriver bridge skipped small delta: angle=%.2f deg last=%.2f deg min_delta=%.2f deg",
                angle,
                self._bridge_last_angle,
                self._bridge_min_angle_delta,
            )
            return

        payload: dict[str, Any] = {
            "type": command_type,
            "angle": angle,
            "pulse_us": pulse_us,
            "channel": self._channel,
        }

        try:
            sock = self._connect_bridge()
            sock.sendall((json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8"))
        except OSError:
            self.close()
            raise

        self._bridge_last_send_at = now
        self._bridge_last_angle = angle

    def _write_angle(self, angle: float, pulse_us: int) -> None:
        """Send the PWM pulse to the hardware interface.

        This stub is intended to be overridden by a platform-specific subclass.
        """
        logger.debug(
            "ServoDriver._write_angle stub called (channel=%d, angle=%.2f deg, pulse=%d us).",
            self._channel,
            angle,
            pulse_us,
        )
