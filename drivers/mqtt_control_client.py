"""MiniPC MQTT control subscriber — receives route/mode commands from RPi."""

from __future__ import annotations

import json
import logging
import threading
import time
from typing import Any, Callable

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
StatusCallback = Callable[[dict], None]
EstopCallback = Callable[[dict], None]

MQTT_STATUS_TOPIC = "car/status"
MQTT_ESTOP_TOPIC = "ugv/rpi/estop"


class MQTTControlClient:
    """Subscribes to car/control/route and car/control/mode topics."""

    def __init__(
        self,
        on_route: RouteCallback | None = None,
        on_mode: ModeCallback | None = None,
        on_status: StatusCallback | None = None,
        on_estop: EstopCallback | None = None,
        host: str = MQTT_BROKER_HOST,
        port: int = MQTT_BROKER_PORT,
        username: str = MQTT_USERNAME,
        password: str = MQTT_PASSWORD,
        keepalive: int = MQTT_KEEPALIVE_S,
        stale_after_s: float = 5.0,
    ) -> None:
        self._on_route = on_route
        self._on_mode = on_mode
        self._on_status = on_status
        self._on_estop = on_estop
        self._host = host
        self._port = port
        self._username = username
        self._password = password
        self._keepalive = keepalive
        self._client: mqtt.Client | None = None
        self._current_mode = "AUTO"
        self._stale_after_s = float(stale_after_s)
        self._lock = threading.Lock()
        self._last_status_payload: dict[str, Any] | None = None
        self._last_status_ts: float | None = None
        self._last_estop_payload: dict[str, Any] | None = None

    @property
    def current_mode(self) -> str:
        return self._current_mode

    def get_rpi_status(self) -> dict[str, Any]:
        """Return a snapshot of last RPi telemetry + freshness for the dashboard."""
        with self._lock:
            payload = dict(self._last_status_payload) if self._last_status_payload else None
            ts = self._last_status_ts
            estop_payload = dict(self._last_estop_payload) if self._last_estop_payload else None

        now = time.time()
        if ts is None:
            return {
                "online": False,
                "stale": True,
                "age_s": None,
                "payload": estop_payload or {},
            }
        age = max(0.0, now - ts)
        stale = age > self._stale_after_s
        rpi_online_flag = bool(payload.get("rpi_online")) if payload else False
        online = rpi_online_flag and not stale
        merged = dict(payload or {})
        if estop_payload is not None and "estop_active" not in merged:
            merged["estop_active"] = bool(estop_payload.get("active"))
        return {
            "online": online,
            "stale": stale,
            "age_s": round(age, 2),
            "payload": merged,
        }

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
        client.subscribe(MQTT_STATUS_TOPIC, qos=1)
        client.subscribe(MQTT_ESTOP_TOPIC, qos=1)
        logger.info(
            "MQTT control client connected, subscribed to car/control/#, %s, %s",
            MQTT_STATUS_TOPIC,
            MQTT_ESTOP_TOPIC,
        )

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
            elif topic == MQTT_STATUS_TOPIC:
                self._handle_status(payload)
            elif topic == MQTT_ESTOP_TOPIC:
                self._handle_estop(payload)
            else:
                logger.warning("MQTT control client unhandled topic: %s", topic)
        except Exception as exc:
            logger.error("MQTT control client callback error: %s", exc)

    def _handle_status(self, payload_text: str) -> None:
        try:
            data = json.loads(payload_text) if payload_text else {}
        except json.JSONDecodeError as exc:
            logger.warning("MQTT control client bad status JSON: %s", exc)
            return
        if not isinstance(data, dict):
            return
        with self._lock:
            self._last_status_payload = data
            self._last_status_ts = time.time()
        if self._on_status is not None:
            try:
                self._on_status(data)
            except Exception as exc:
                logger.error("on_status callback error: %s", exc)

    def _handle_estop(self, payload_text: str) -> None:
        try:
            data = json.loads(payload_text) if payload_text else {}
        except json.JSONDecodeError as exc:
            logger.warning("MQTT control client bad estop JSON: %s", exc)
            return
        if not isinstance(data, dict):
            return
        with self._lock:
            self._last_estop_payload = data
        if self._on_estop is not None:
            try:
                self._on_estop(data)
            except Exception as exc:
                logger.error("on_estop callback error: %s", exc)
