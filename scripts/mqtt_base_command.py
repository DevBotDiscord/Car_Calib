#!/usr/bin/env python3
"""Publish a base command to the MQTT broker for ESP32 testing."""

from __future__ import annotations

import argparse
import importlib

from config.settings import (
    MQTT_BASE_COMMAND_TOPIC,
    MQTT_BROKER_HOST,
    MQTT_BROKER_PORT,
    MQTT_KEEPALIVE_S,
    MQTT_PASSWORD,
    MQTT_USERNAME,
)

COMMANDS = ("STOP", "FORWARD", "BACKWARD", "LOCK", "UNLOCK")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Publish a base command to MQTT.")
    parser.add_argument("command", choices=COMMANDS, help="Base command to publish.")
    parser.add_argument("--host", default=MQTT_BROKER_HOST, help="MQTT broker host.")
    parser.add_argument("--port", type=int, default=MQTT_BROKER_PORT, help="MQTT broker port.")
    parser.add_argument("--topic", default=MQTT_BASE_COMMAND_TOPIC, help="MQTT topic for base command.")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    mqtt = importlib.import_module("paho.mqtt.client")

    callback_api_version = getattr(mqtt, "CallbackAPIVersion", None)
    if callback_api_version is not None:
        client = mqtt.Client(
            callback_api_version=callback_api_version.VERSION1,
            client_id="car-base-command",
        )
    else:
        client = mqtt.Client(client_id="car-base-command")

    if MQTT_USERNAME:
        client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

    client.connect(args.host, args.port, keepalive=MQTT_KEEPALIVE_S)
    result = client.publish(args.topic, args.command)
    wait_for_publish = getattr(result, "wait_for_publish", None)
    if callable(wait_for_publish):
        wait_for_publish()
    client.disconnect()

    print(f"Published {args.command} to {args.topic} via {args.host}:{args.port}")


if __name__ == "__main__":
    main()
