"""MQTT client setup, callbacks, and status publishing."""

from __future__ import annotations

import json
import time

from . import config
from .steering import resolve_remote_servo_angle


def publish_status(state: str) -> None:
    if config.mqtt_client is None or not config.mqtt_connected:
        return
    payload = json.dumps({
        "source": "rpi-mqtt-bridge",
        "state": state,
        "steer_angle": round(config.steer_angle, 2),
        "center_angle": round(config.CENTER_ANGLE, 2),
        "servo_pin": config.SERVO_PIN,
        "pigpio_host": config.PIGPIO_HOST,
        "ts": time.time(),
    })
    config.mqtt_client.publish(config.MQTT_STATUS_TOPIC, payload, retain=False)


def handle_mqtt_servo_message(payload_text: str) -> None:
    config.remote_servo_angle = resolve_remote_servo_angle(payload_text)
    config.remote_servo_updated_at = time.monotonic()


def on_mqtt_connect(client, userdata, flags, rc, properties=None) -> None:
    del userdata, flags, properties
    if rc != 0:
        print(f"MQTT: connect failed rc={rc}")
        config.mqtt_connected = False
        return
    config.mqtt_connected = True
    client.subscribe(config.MQTT_SERVO_TOPIC)
    print(f"MQTT: connected to {config.MQTT_BROKER_HOST}:{config.MQTT_BROKER_PORT} topic={config.MQTT_SERVO_TOPIC}")
    publish_status("online")


def on_mqtt_disconnect(client, userdata, rc, properties=None) -> None:
    del client, userdata, properties
    config.mqtt_connected = False
    if config.running:
        print(f"MQTT: disconnected rc={rc}")


def on_mqtt_message(client, userdata, message) -> None:
    del client, userdata
    try:
        payload_text = message.payload.decode("utf-8")
        handle_mqtt_servo_message(payload_text)
    except (UnicodeDecodeError, ValueError, json.JSONDecodeError) as exc:
        print(f"MQTT: invalid payload on {message.topic}: {exc}")


def setup_mqtt() -> None:
    mqtt = config.mqtt
    callback_api_version = getattr(mqtt, "CallbackAPIVersion", None)
    if callback_api_version is not None:
        client = mqtt.Client(callback_api_version=callback_api_version.VERSION1, client_id=config.MQTT_CLIENT_ID)
    else:
        client = mqtt.Client(client_id=config.MQTT_CLIENT_ID)

    if config.MQTT_USERNAME:
        client.username_pw_set(config.MQTT_USERNAME, config.MQTT_PASSWORD)

    client.on_connect = on_mqtt_connect
    client.on_disconnect = on_mqtt_disconnect
    client.on_message = on_mqtt_message
    client.reconnect_delay_set(min_delay=1, max_delay=5)
    client.connect_async(config.MQTT_BROKER_HOST, config.MQTT_BROKER_PORT, keepalive=config.MQTT_KEEPALIVE_S)
    client.loop_start()
    config.mqtt_client = client


def close_mqtt() -> None:
    if config.mqtt_client is None:
        return
    try:
        publish_status("offline")
    except Exception:
        pass
    try:
        config.mqtt_client.disconnect()
    except Exception:
        pass
    try:
        config.mqtt_client.loop_stop()
    except Exception:
        pass
    config.mqtt_client = None
    config.mqtt_connected = False
