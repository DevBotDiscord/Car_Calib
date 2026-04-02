#!/usr/bin/env python3
"""TCP receiver that drives a steering servo on Raspberry Pi GPIO 19."""

from __future__ import annotations

import json
import os
import signal
import socket
import time

from gpiozero import AngularServo

HOST = os.getenv("SERVO_BRIDGE_HOST", "0.0.0.0")
PORT = int(os.getenv("SERVO_BRIDGE_PORT", "8765"))

SERVO_PIN = int(os.getenv("SERVO_PIN", "19"))
SERVO_CENTER_ANGLE = float(os.getenv("SERVO_CENTER_ANGLE", "-8"))
SERVO_MIN_ANGLE = -90
SERVO_MAX_ANGLE = 90
SERVO_MIN_PULSE = 0.0005
SERVO_MAX_PULSE = 0.0025
COMMAND_TIMEOUT = float(os.getenv("SERVO_COMMAND_TIMEOUT", "1.0"))
IDLE_SLEEP = 0.05

running = True


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def log(message: str) -> None:
    print(message)


class ServoBridgeServer:
    def __init__(self) -> None:
        self._servo = AngularServo(
            SERVO_PIN,
            min_angle=SERVO_MIN_ANGLE,
            max_angle=SERVO_MAX_ANGLE,
            min_pulse_width=SERVO_MIN_PULSE,
            max_pulse_width=SERVO_MAX_PULSE,
        )
        self._last_angle: float | None = None

    def apply_angle(self, angle: float) -> None:
        angle = clamp(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE)
        if angle == self._last_angle:
            return

        self._servo.angle = angle
        self._last_angle = angle
        log(f"SERVO: {angle} deg on GPIO {SERVO_PIN}")

    def center(self, angle: float = SERVO_CENTER_ANGLE) -> None:
        self.apply_angle(angle)

    def detach(self) -> None:
        self._servo.detach()
        self._last_angle = None
        log("SERVO: detached")

    def close(self) -> None:
        try:
            self.detach()
        except Exception:
            pass


servo_server = ServoBridgeServer()


def signal_handler(sig, frame) -> None:
    del sig, frame
    global running
    running = False


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


def handle_payload(payload: dict[str, float | str]) -> None:
    command = payload.get("type")

    if command == "angle":
        angle = float(payload["angle"])
        servo_server.apply_angle(angle)
    elif command == "center":
        servo_server.center(float(payload.get("angle", 0.0)))
    elif command == "detach":
        servo_server.detach()
    else:
        raise ValueError(f"Unsupported command: {command}")


def serve() -> None:
    global running

    print("=== RPI SERVO BRIDGE ===")
    print(f"Listening on {HOST}:{PORT}")
    print(f"Servo GPIO pin: {SERVO_PIN}")
    print(f"Servo center angle: {SERVO_CENTER_ANGLE}")
    print("")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((HOST, PORT))
        server_sock.listen(1)
        server_sock.settimeout(1.0)

        while running:
            try:
                conn, addr = server_sock.accept()
            except socket.timeout:
                continue

            log(f"Client connected: {addr[0]}:{addr[1]}")
            with conn:
                conn.settimeout(IDLE_SLEEP)
                buffer = ""
                last_command_at = time.monotonic()

                while running:
                    try:
                        data = conn.recv(4096)
                        if not data:
                            log("Client disconnected")
                            servo_server.center()
                            break

                        buffer += data.decode("utf-8")
                        while "\n" in buffer:
                            line, buffer = buffer.split("\n", 1)
                            if not line.strip():
                                continue

                            payload = json.loads(line)
                            handle_payload(payload)
                            last_command_at = time.monotonic()
                    except socket.timeout:
                        if time.monotonic() - last_command_at > COMMAND_TIMEOUT:
                            servo_server.center()
                            time.sleep(IDLE_SLEEP)
                    except (ValueError, json.JSONDecodeError) as exc:
                        log(f"Invalid payload: {exc}")
                    except OSError as exc:
                        log(f"Connection error: {exc}")
                        break


def main() -> None:
    try:
        serve()
    finally:
        print("Cleaning up...")
        servo_server.close()
        print("Done.")


if __name__ == "__main__":
    main()
