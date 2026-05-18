"""MiniPC MQTT control subscriber — receives route/mode commands from RPi."""

from __future__ import annotations

import logging
from typing import Callable

import paho.mqtt.client as mqtt

from config.settings import (
    MQTT_BROKER_HOST,
    MQTT_BROKER_PORT,
    MQTT_KEEPALIVE_S,
    MQTT_PASSWORD,
    MQTT_USERNAME,
)

logger = logging.getLogger(__name__)

RouteCallback = Callable[[str], None]
ModeCallback = Callable[[str], None]


class MQTTControlClient:
    """Subscribes to car/control/route and car/control/mode topics."""

    def __init__(
        self,
        on_route: RouteCallback | None = None,
        on_mode: ModeCallback | None = None,
        host: str = MQTT_BROKER_HOST,
        port: int = MQTT_BROKER_PORT,
        username: str = MQTT_USERNAME,
        password: str = MQTT_PASSWORD,
        keepalive: int = MQTT_KEEPALIVE_S,
    ) -> None:
        self._on_route = on_route
        self._on_mode = on_mode
        self._host = host
        self._port = port
        self._username = username
        self._password = password
        self._keepalive = keepalive
        self._client: mqtt.Client | None = None
        self._current_mode = "AUTO"

    @property
    def current_mode(self) -> str:
        return self._current_mode

    def setup(self) -> None:
        callback_api_version = getattr(mqtt, "CallbackAPIVersion", None)
        if callback_api_version is not None:
            client = mqtt.Client(callback_api_version=callback_api_version.VERSION1, client_id="minipc-control-sub")
        else:
            client = mqtt.Client(client_id="minipc-control-sub")

        if self._username:
            client.username_pw_set(self._username, self._password)

        client.on_connect = self._on_connect
        client.on_disconnect = self._on_disconnect
        client.on_message = self._on_message

        client.reconnect_delay_set(min_delay=1, max_delay=5)
        client.connect_async(self._host, self._port, keepalive=self._keepalive)
        client.loop_start()
        self._client = client
        logger.info("MQTT control client connecting to %s:%d", self._host, self._port)

    def close(self) -> None:
        if self._client is None:
            return
        try:
            self._client.disconnect()
        except Exception:
            pass
        try:
            self._client.loop_stop()
        except Exception:
            pass
        self._client = None

    def _on_connect(self, client, userdata, flags, rc, properties=None) -> None:
        del userdata, flags, properties
        if rc != 0:
            logger.error("MQTT control client connect failed rc=%d", rc)
            return
        client.subscribe("car/control/route", qos=1)
        client.subscribe("car/control/mode", qos=1)
        logger.info("MQTT control client connected, subscribed to car/control/#")

    def _on_disconnect(self, client, userdata, rc, properties=None) -> None:
        del client, userdata, properties
        logger.warning("MQTT control client disconnected rc=%d", rc)

    def _on_message(self, client, userdata, message) -> None:
        del client, userdata
        try:
            payload = message.payload.decode("utf-8").strip()
        except UnicodeDecodeError:
            logger.warning("MQTT control client invalid payload encoding")
            return

        topic = message.topic
        logger.debug("MQTT control client received topic=%s payload=%s", topic, payload)

        try:
            if topic == "car/control/route":
                if self._on_route is not None:
                    self._on_route(payload)
            elif topic == "car/control/mode":
                self._current_mode = payload
                if self._on_mode is not None:
                    self._on_mode(payload)
            else:
                logger.warning("MQTT control client unhandled topic: %s", topic)
        except Exception as exc:
            logger.error("MQTT control client callback error: %s", exc)
