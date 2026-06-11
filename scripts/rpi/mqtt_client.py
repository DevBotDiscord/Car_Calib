"""MQTT client setup, callbacks, and status publishing (RPi actuator side)."""

from __future__ import annotations

import json
import time

from . import config
from .base import set_base
from .logging_utils import get_logger
from .steering import apply_steering, resolve_remote_servo_angle

logger = get_logger("mqtt")

BASE_STATES: dict[str, tuple[tuple[int, int, int], str]] = {
    "FORWARD": ((0, 1, 0), "FORWARD"),
    "BACKWARD": ((0, 0, 1), "BACKWARD"),
    "STOP": ((0, 0, 0), "STOP"),
    "LOCK": ((1, 0, 1), "LOCK"),
    "UNLOCK": ((1, 1, 0), "UNLOCK"),
    "TURN_LEFT": ((1, 0, 0), "TURN_LEFT"),
    "LEFT": ((1, 0, 0), "TURN_LEFT"),
    "TURN_RIGHT": ((0, 1, 1), "TURN_RIGHT"),
    "RIGHT": ((0, 1, 1), "TURN_RIGHT"),
}


# ---------------------------------------------------------------------------
# publish status heartbeat
# ---------------------------------------------------------------------------

def publish_status(state: str) -> None:
    """Publish detailed health telemetry retained on MQTT."""
    if config.mqtt_client is None or not config.mqtt_connected:
        return
    now = time.time()
    uptime_sec = max(0.0, now - config.bridge_started_at)
    pigpio_status = "online" if config.pigpio_connected else "offline"
    payload = json.dumps({
        "rpi_online": True,
        "state": state,
        "pigpio_connected": bool(config.pigpio_connected),
        "pigpio_status": pigpio_status,
        "mqtt_connected": True,
        "mqtt_status": "connected",
        "estop_active": bool(config.estop_active),
        "steer_angle": round(config.steer_angle, 2),
        "center_angle": round(config.CENTER_ANGLE, 2),
        "base_state": getattr(config, "last_base_label", "STOP"),
        "relay_on": bool(config.relay_on),
        "script_active": config.script_active,
        "current_route_mode": config.current_route_mode,
        "hostname": config.RPI_HOSTNAME,
        "uptime_sec": round(uptime_sec, 1),
        "uptime_s": round(uptime_sec, 1),
        "servo_pin": config.SERVO_PIN,
        "pigpio_host": config.PIGPIO_HOST,
        "ts": now,
    })
    config.mqtt_client.publish(config.MQTT_STATUS_TOPIC, payload, qos=1, retain=True)
    logger.debug(
        "[MQTT][TX][STATUS] state=%s pigpio=%s base=%s",
        state, pigpio_status, getattr(config, "last_base_label", "STOP"),
    )


def publish_estop(active: bool) -> None:
    """Publish retained E-stop latch state to MQTT_ESTOP_TOPIC."""
    if config.mqtt_client is None or not config.mqtt_connected:
        return
    payload = json.dumps({
        "active": bool(active),
        "latched_at": config.estop_latched_at,
        "ts": time.time(),
    })
    config.mqtt_client.publish(config.MQTT_ESTOP_TOPIC, payload, qos=1, retain=True)
    logger.warning("[MQTT][TX][ESTOP] active=%s topic=%s", active, config.MQTT_ESTOP_TOPIC)


def publish_route_control(command: str) -> None:
    """Publish route control command (START/STOP) to MiniPC."""
    if config.mqtt_client is None or not config.mqtt_connected:
        return
    config.mqtt_client.publish("car/control/route", command, qos=1)
    logger.info("[MQTT][TX][ROUTE] command=%s topic=car/control/route", command)


def publish_mode(mode: str) -> None:
    """Publish control mode (AUTO/CRUISE/SQUARE/REMOTE_STEER) to MiniPC."""
    if config.mqtt_client is None or not config.mqtt_connected:
        return
    config.mqtt_client.publish("car/control/mode", mode, qos=1, retain=True)
    config.current_route_mode = mode
    logger.info("[MQTT][TX][MODE] mode=%s topic=car/control/mode", mode)


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
    logger.info("[MQTT][RX][SERVO] apply angle=%.2f raw=%s", angle, payload_text)


def handle_script_active_message(payload_text: str) -> None:
    cmd = payload_text.strip().upper()
    if cmd in ("ON", "1", "TRUE", "START"):
        config.script_active = True
        logger.info("[MQTT][RX][SCRIPT] script_active=ON")
    elif cmd in ("OFF", "0", "FALSE", "STOP"):
        config.script_active = False
        logger.info("[MQTT][RX][SCRIPT] script_active=OFF")
    else:
        logger.warning("[MQTT][RX][SCRIPT] unknown payload=%s", cmd)


def handle_estop_reset_message(payload_text: str) -> None:
    """Attempt to clear the E-stop latch (hardware safety check still applies)."""
    del payload_text
    try:
        from .estop import try_reset

        ok = try_reset("dashboard")
        logger.warning("[MQTT][RX][ESTOP_RESET] try_reset -> %s", ok)
    except Exception as exc:  # noqa: BLE001
        logger.error("[MQTT][RX][ESTOP_RESET] failed: %s", exc)


def handle_base_message(payload_text: str) -> None:
    cmd = payload_text.strip().upper()
    entry = BASE_STATES.get(cmd)
    if entry is None:
        logger.warning("[MQTT][RX][BASE] unknown command=%s", cmd)
        return

    state, label = entry
    set_base(*state, f"MQTT-{label}")
    logger.info("[MQTT][RX][BASE] %s state=%d,%d,%d", label, *state)


def handle_relay_message(payload_text: str) -> None:
    cmd = payload_text.strip().upper()
    if cmd == "ON":
        config.gpio.write(config.RELAY_PIN, 1)
        config.relay_on = True
        logger.info("[MQTT][RX][RELAY] ON pin=%s", config.RELAY_PIN)
    elif cmd == "OFF":
        config.gpio.write(config.RELAY_PIN, 0)
        config.relay_on = False
        logger.info("[MQTT][RX][RELAY] OFF pin=%s", config.RELAY_PIN)
    else:
        logger.warning("[MQTT][RX][RELAY] unknown command=%s", cmd)


def handle_power_message(payload_text: str) -> None:
    cmd = payload_text.strip().upper()
    from .base import pulse_power

    if cmd == "ON":
        pulse_power(True)
        logger.info("[MQTT][RX][POWER] ON pin=%s", config.POWER_RELAY_PIN)
    elif cmd == "OFF":
        pulse_power(False)
        logger.info("[MQTT][RX][POWER] OFF pin=%s", config.POWER_RELAY_PIN)
    else:
        logger.warning("[MQTT][RX][POWER] unknown command=%s", cmd)


# ---------------------------------------------------------------------------
# MQTT callbacks
# ---------------------------------------------------------------------------

def on_mqtt_connect(client, userdata, flags, rc, properties=None) -> None:
    del userdata, flags, properties
    if rc != 0:
        logger.error("[MQTT][CONN] connect failed rc=%s", rc)
        config.mqtt_connected = False
        return
    config.mqtt_connected = True
    client.subscribe(config.MQTT_SERVO_TOPIC)
    client.subscribe(config.MQTT_BASE_COMMAND_TOPIC)
    client.subscribe(config.MQTT_RELAY_TOPIC)
    client.subscribe(config.MQTT_POWER_TOPIC)
    client.subscribe("car/control/script_active")
    client.subscribe("car/control/estop_reset")
    logger.info(
        "[MQTT][CONN] connected host=%s port=%s",
        config.MQTT_BROKER_HOST,
        config.MQTT_BROKER_PORT,
    )
    logger.info(
        "[MQTT][CONN] topics=%s,%s,%s,car/control/script_active",
        config.MQTT_SERVO_TOPIC,
        config.MQTT_BASE_COMMAND_TOPIC,
        config.MQTT_RELAY_TOPIC,
    )
    publish_status("online")


def on_mqtt_disconnect(client, userdata, rc, properties=None) -> None:
    del client, userdata, properties
    config.mqtt_connected = False
    if config.running:
        logger.warning("[MQTT][CONN] disconnected rc=%s", rc)


def on_mqtt_message(client, userdata, message) -> None:
    del client, userdata
    try:
        payload_text = message.payload.decode("utf-8")
    except UnicodeDecodeError as exc:
        logger.warning("[MQTT][RX] invalid encoding topic=%s error=%s", message.topic, exc)
        return

    topic = message.topic
    try:
        if topic == config.MQTT_SERVO_TOPIC:
            handle_servo_message(payload_text)
        elif topic == config.MQTT_BASE_COMMAND_TOPIC:
            handle_base_message(payload_text)
        elif topic == config.MQTT_RELAY_TOPIC:
            handle_relay_message(payload_text)
        elif topic == config.MQTT_POWER_TOPIC:
            handle_power_message(payload_text)
        elif topic == "car/control/script_active":
            handle_script_active_message(payload_text)
        elif topic == "car/control/estop_reset":
            handle_estop_reset_message(payload_text)
        else:
            logger.warning("[MQTT][RX] unhandled topic=%s", topic)
    except (ValueError, json.JSONDecodeError) as exc:
        logger.warning("[MQTT][RX] invalid payload topic=%s error=%s raw=%s", topic, exc, payload_text)


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

    # Last Will and Testament — broker republishes this retained payload
    # to MQTT_STATUS_TOPIC if the RPi disconnects ungracefully.
    lwt_payload = json.dumps({
        "rpi_online": False,
        "mqtt_connected": False,
        "mqtt_status": "disconnected",
        "reason": "mqtt_lwt",
    })
    client.will_set(config.MQTT_STATUS_TOPIC, lwt_payload, qos=1, retain=True)

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
