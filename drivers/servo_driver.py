"""Drivers module: servo angle translation layer.

Provides an abstracted interface for sending angle commands to a servo
motor connected via a PCA9685 I2C PWM board, directly to Jetson GPIO,
or over a TCP bridge to a Raspberry Pi.
"""

from __future__ import annotations

import json
import logging
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
    DRIVER_SERVO_PULSE_MAX_US,
    DRIVER_SERVO_PULSE_MIN_US,
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
        """Release any open bridge socket."""
        if self._bridge_socket is None:
            return

        try:
            self._bridge_socket.close()
        finally:
            self._bridge_socket = None
            self._bridge_last_send_at = None
            self._bridge_last_angle = None
            self._bridge_pending = None

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
