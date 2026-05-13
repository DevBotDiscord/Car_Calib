"""MQTT relay driver — publishes ON/OFF to car/relay topic."""

from __future__ import annotations

import importlib
import logging
import os
from typing import Any

from config.settings import (
    MQTT_BROKER_HOST,
    MQTT_BROKER_PORT,
    MQTT_CLIENT_ID_PREFIX,
    MQTT_KEEPALIVE_S,
    MQTT_PASSWORD,
    MQTT_RELAY_TOPIC,
    MQTT_USERNAME,
)

logger = logging.getLogger(__name__)


class RelayDriver:
    """Publishes relay commands to MQTT (MiniPC side)."""

    def __init__(
        self,
        mqtt_host: str = MQTT_BROKER_HOST,
        mqtt_port: int = MQTT_BROKER_PORT,
        mqtt_username: str = MQTT_USERNAME,
        mqtt_password: str = MQTT_PASSWORD,
        mqtt_keepalive_s: int = MQTT_KEEPALIVE_S,
        mqtt_topic: str = MQTT_RELAY_TOPIC,
        mqtt_client_id_prefix: str = MQTT_CLIENT_ID_PREFIX,
    ) -> None:
        self._mqtt_host = mqtt_host
        self._mqtt_port = mqtt_port
        self._mqtt_username = mqtt_username
        self._mqtt_password = mqtt_password
        self._mqtt_keepalive_s = mqtt_keepalive_s
        self._mqtt_topic = mqtt_topic
        self._mqtt_client_id = f"{mqtt_client_id_prefix}-relay-{os.getpid()}"
        self._mqtt_client: Any | None = None
        self._last_state: str | None = None

    def _get_mqtt_client(self) -> Any:
        if self._mqtt_client is not None:
            return self._mqtt_client

        try:
            mqtt = importlib.import_module("paho.mqtt.client")
        except ImportError as exc:
            raise RuntimeError(
                "paho-mqtt is required for relay publishing."
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
            "RelayDriver MQTT connected to %s:%d topic=%s.",
            self._mqtt_host,
            self._mqtt_port,
            self._mqtt_topic,
        )
        self._mqtt_client = client
        return client

    def _publish(self, state: str) -> None:
        if state not in ("ON", "OFF"):
            raise ValueError(f"Invalid relay command: {state}")
        if state == self._last_state:
            return
        self._last_state = state
        client = self._get_mqtt_client()
        try:
            client.publish(self._mqtt_topic, state, retain=False)
        except Exception as exc:
            self.close()
            raise OSError(f"RelayDriver MQTT publish failed: {exc}") from exc

    def on(self) -> None:
        self._publish("ON")

    def off(self) -> None:
        self._publish("OFF")

    def close(self) -> None:
        if self._mqtt_client is not None:
            try:
                self._mqtt_client.disconnect()
            except Exception:
                pass
            self._mqtt_client = None
            self._last_state = None
