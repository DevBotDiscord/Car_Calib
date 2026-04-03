#!/usr/bin/env python3
"""Publish a servo angle to MQTT for Raspberry Pi or ESP32 bridge testing."""

from __future__ import annotations

import argparse
import importlib
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from config.settings import (
    MQTT_BROKER_HOST,
    MQTT_BROKER_PORT,
    MQTT_KEEPALIVE_S,
    MQTT_PASSWORD,
    MQTT_SERVO_TOPIC,
    MQTT_USERNAME,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Publish a servo angle to MQTT.")
    parser.add_argument("angle", type=float, help="Servo angle payload to publish.")
    parser.add_argument("--host", default=MQTT_BROKER_HOST, help="MQTT broker host.")
    parser.add_argument("--port", type=int, default=MQTT_BROKER_PORT, help="MQTT broker port.")
    parser.add_argument("--topic", default=MQTT_SERVO_TOPIC, help="MQTT topic for servo angle.")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    mqtt = importlib.import_module("paho.mqtt.client")

    callback_api_version = getattr(mqtt, "CallbackAPIVersion", None)
    if callback_api_version is not None:
        client = mqtt.Client(
            callback_api_version=callback_api_version.VERSION1,
            client_id="car-servo-command",
        )
    else:
        client = mqtt.Client(client_id="car-servo-command")

    if MQTT_USERNAME:
        client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

    client.connect(args.host, args.port, keepalive=MQTT_KEEPALIVE_S)
    result = client.publish(args.topic, f"{args.angle:.4f}")
    wait_for_publish = getattr(result, "wait_for_publish", None)
    if callable(wait_for_publish):
        wait_for_publish()
    client.disconnect()

    print(f"Published {args.angle:.4f} to {args.topic} via {args.host}:{args.port}")


if __name__ == "__main__":
    main()
