"""MQTT client setup, callbacks, and status publishing (RPi actuator side)."""

from __future__ import annotations

import json
import time

from . import config
from .base import set_base
from .steering import apply_steering, resolve_remote_servo_angle


# ---------------------------------------------------------------------------
# publish status heartbeat
# ---------------------------------------------------------------------------

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


def publish_route_control(command: str) -> None:
    """Publish route control command (START/STOP) to MiniPC."""
    if config.mqtt_client is None or not config.mqtt_connected:
        return
    config.mqtt_client.publish("car/control/route", command, qos=1)
    print(f"MQTT: published route control: {command}")


def publish_mode(mode: str) -> None:
    """Publish control mode (AUTO/CRUISE/SQUARE/REMOTE_STEER) to MiniPC."""
    if config.mqtt_client is None or not config.mqtt_connected:
        return
    config.mqtt_client.publish("car/control/mode", mode, qos=1, retain=True)
    config.current_route_mode = mode
    print(f"MQTT: published mode: {mode}")


# ---------------------------------------------------------------------------
# message handlers (one per topic)
# ---------------------------------------------------------------------------

def handle_servo_message(payload_text: str) -> None:
    # Only accept MQTT servo commands while a dashboard route script is
    # active. Vision PID stream and stray publishers are ignored otherwise
    # so manual gamepad/keyboard control stays exclusive on the RPi.
    if not config.script_active:
        return
    angle = resolve_remote_servo_angle(payload_text)
    apply_steering(angle, "MQTT")


def handle_script_active_message(payload_text: str) -> None:
    cmd = payload_text.strip().upper()
    if cmd in ("ON", "1", "TRUE", "START"):
        config.script_active = True
        print("MQTT: script_active=ON (route script driving)")
    elif cmd in ("OFF", "0", "FALSE", "STOP"):
        config.script_active = False
        print("MQTT: script_active=OFF")
    else:
        print(f"MQTT: unknown script_active payload: {cmd}")


def handle_base_message(payload_text: str) -> None:
    cmd = payload_text.strip().upper()
    if cmd == "FORWARD":
        set_base(0, 1, 0, "MQTT-FORWARD")
    elif cmd == "BACKWARD":
        set_base(0, 0, 1, "MQTT-BACKWARD")
    elif cmd == "STOP":
        set_base(0, 0, 0, "MQTT-STOP")
    elif cmd == "LOCK":
        set_base(1, 0, 1, "MQTT-LOCK")
    elif cmd == "UNLOCK":
        set_base(1, 1, 0, "MQTT-UNLOCK")
    else:
        print(f"MQTT: unknown base command: {cmd}")


def handle_relay_message(payload_text: str) -> None:
    cmd = payload_text.strip().upper()
    if cmd == "ON":
        config.gpio.write(config.RELAY_PIN, 1)
        config.relay_on = True
        print("RELAY: ON (MQTT)")
    elif cmd == "OFF":
        config.gpio.write(config.RELAY_PIN, 0)
        config.relay_on = False
        print("RELAY: OFF (MQTT)")
    else:
        print(f"MQTT: unknown relay command: {cmd}")


# ---------------------------------------------------------------------------
# MQTT callbacks
# ---------------------------------------------------------------------------

def on_mqtt_connect(client, userdata, flags, rc, properties=None) -> None:
    del userdata, flags, properties
    if rc != 0:
        print(f"MQTT: connect failed rc={rc}")
        config.mqtt_connected = False
        return
    config.mqtt_connected = True
    client.subscribe(config.MQTT_SERVO_TOPIC)
    client.subscribe(config.MQTT_BASE_COMMAND_TOPIC)
    client.subscribe(config.MQTT_RELAY_TOPIC)
    client.subscribe("car/control/script_active")
    print(f"MQTT: connected to {config.MQTT_BROKER_HOST}:{config.MQTT_BROKER_PORT}")
    print(f"  topics: {config.MQTT_SERVO_TOPIC}, {config.MQTT_BASE_COMMAND_TOPIC}, {config.MQTT_RELAY_TOPIC}, car/control/script_active")
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
    except UnicodeDecodeError as exc:
        print(f"MQTT: invalid payload encoding on {message.topic}: {exc}")
        return

    topic = message.topic
    try:
        if topic == config.MQTT_SERVO_TOPIC:
            handle_servo_message(payload_text)
        elif topic == config.MQTT_BASE_COMMAND_TOPIC:
            handle_base_message(payload_text)
        elif topic == config.MQTT_RELAY_TOPIC:
            handle_relay_message(payload_text)
        elif topic == "car/control/script_active":
            handle_script_active_message(payload_text)
        else:
            print(f"MQTT: unhandled topic: {topic}")
    except (ValueError, json.JSONDecodeError) as exc:
        print(f"MQTT: invalid payload on {topic}: {exc}")


# ---------------------------------------------------------------------------
# setup / teardown
# ---------------------------------------------------------------------------

def setup_mqtt() -> None:
    mqtt = config.mqtt
    if mqtt is None:
        raise RuntimeError(
            "paho-mqtt is not available. Install requirements-rpi.txt in the runtime image/environment."
        )
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
