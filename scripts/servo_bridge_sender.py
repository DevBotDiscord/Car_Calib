#!/usr/bin/env python3
"""Keyboard sender that forwards steering commands to a Raspberry Pi servo bridge.

This script keeps the original keyboard-driven steering behaviour, but
replaces direct GPIO control with TCP messages sent to a remote receiver
running on the Raspberry Pi.
"""

from __future__ import annotations

import json
import os
import signal
import socket
import time

from evdev import InputDevice, ecodes

# =========================================================
# CONFIG
# =========================================================
KEYBOARD_DEVICE = os.getenv(
    "KEYBOARD_DEVICE",
    "/dev/input/by-id/usb-YJX_CHIP_WirelessDevice-event-kbd",
)
RPI_HOST = os.getenv("SERVO_BRIDGE_HOST", "192.168.1.50")
RPI_PORT = int(os.getenv("SERVO_BRIDGE_PORT", "8765"))

CENTER_ANGLE = -8
LEFT_LIMIT = -65
RIGHT_LIMIT = 60
STEP = 20
LOOP_DELAY = 0.01
SOCKET_TIMEOUT = 1.0
RECONNECT_DELAY = 1.0

# =========================================================
# GLOBAL STATE
# =========================================================
running = True
pressed_keys: set[str] = set()
steer_angle = CENTER_ANGLE


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def log(message: str) -> None:
    print(message)


class ServoBridgeClient:
    """Small TCP client that sends newline-delimited JSON commands."""

    def __init__(self, host: str, port: int) -> None:
        self._host = host
        self._port = port
        self._sock: socket.socket | None = None

    def connect(self) -> None:
        if self._sock is not None:
            return

        while running and self._sock is None:
            try:
                sock = socket.create_connection(
                    (self._host, self._port),
                    timeout=SOCKET_TIMEOUT,
                )
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self._sock = sock
                log(f"Connected to servo bridge at {self._host}:{self._port}")
            except OSError as exc:
                log(f"Bridge connect failed: {exc}. Retrying in {RECONNECT_DELAY:.1f}s")
                time.sleep(RECONNECT_DELAY)

    def send(self, payload: dict[str, float | str]) -> None:
        self.connect()
        if self._sock is None:
            return

        try:
            message = json.dumps(payload, separators=(",", ":")) + "\n"
            self._sock.sendall(message.encode("utf-8"))
        except OSError as exc:
            log(f"Bridge send failed: {exc}")
            self.close()

    def close(self) -> None:
        if self._sock is None:
            return

        try:
            self._sock.close()
        finally:
            self._sock = None


bridge = ServoBridgeClient(RPI_HOST, RPI_PORT)
keyboard = InputDevice(KEYBOARD_DEVICE)
last_sent_angle: float | None = None


def send_angle(angle: float, force: bool = False) -> None:
    global last_sent_angle

    angle = clamp(angle, LEFT_LIMIT, RIGHT_LIMIT)
    if not force and angle == last_sent_angle:
        return

    bridge.send({"type": "angle", "angle": angle})
    log(f"STEER: {angle} deg | CENTER: {CENTER_ANGLE} deg")
    last_sent_angle = angle


def steer_center(force: bool = False) -> None:
    global steer_angle
    steer_angle = CENTER_ANGLE
    send_angle(steer_angle, force=force)


def steer_left_step() -> None:
    global steer_angle
    steer_angle += STEP
    steer_angle = clamp(steer_angle, LEFT_LIMIT, RIGHT_LIMIT)
    send_angle(steer_angle)


def steer_right_step() -> None:
    global steer_angle
    steer_angle -= STEP
    steer_angle = clamp(steer_angle, LEFT_LIMIT, RIGHT_LIMIT)
    send_angle(steer_angle)


def cleanup() -> None:
    try:
        bridge.send({"type": "center", "angle": CENTER_ANGLE})
    except Exception:
        pass

    try:
        keyboard.ungrab()
    except Exception:
        pass

    bridge.close()


def signal_handler(sig, frame) -> None:
    del sig, frame
    global running
    running = False


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


def update_key_state(event) -> None:
    global CENTER_ANGLE
    if event.type != ecodes.EV_KEY:
        return

    keycode = ecodes.KEY.get(event.code)
    if not keycode:
        return

    keys = keycode if isinstance(keycode, list) else [keycode]

    if event.value in (1, 2):
        for key in keys:
            pressed_keys.add(key)
            if event.value == 1:
                if key == "KEY_1":
                    CENTER_ANGLE += 1
                    log(f"CENTER_ANGLE INCREASED: {CENTER_ANGLE}")
                    steer_center(force=True)
                elif key == "KEY_2":
                    CENTER_ANGLE -= 1
                    log(f"CENTER_ANGLE DECREASED: {CENTER_ANGLE}")
                    steer_center(force=True)
                elif key == "KEY_Q":
                    global running
                    running = False
    elif event.value == 0:
        for key in keys:
            pressed_keys.discard(key)


def process_controls() -> None:
    has_a = "KEY_A" in pressed_keys
    has_d = "KEY_D" in pressed_keys
    has_c = "KEY_C" in pressed_keys

    if has_c:
        steer_center()
        return

    if has_a and not has_d:
        steer_left_step()
    elif has_d and not has_a:
        steer_right_step()
    elif not has_a and not has_d:
        steer_center()


def main() -> None:
    global running

    print("=== SERVO BRIDGE SENDER ===")
    print(f"Keyboard: {keyboard.path}")
    print(f"Target bridge: {RPI_HOST}:{RPI_PORT}")
    print("A = steer left")
    print("D = steer right")
    print("C = center steering")
    print("1 = increase center angle")
    print("2 = decrease center angle")
    print("Q = quit")
    print("Ctrl+C = emergency exit")
    print("")
    print(
        f"Steering center={CENTER_ANGLE}, left={LEFT_LIMIT}, right={RIGHT_LIMIT}, step={STEP}"
    )
    print("")

    bridge.connect()
    steer_center(force=True)

    try:
        keyboard.grab()

        while running:
            try:
                for event in keyboard.read():
                    update_key_state(event)
            except (BlockingIOError, OSError):
                pass

            process_controls()
            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        print("\nEMERGENCY EXIT: Ctrl+C")
    finally:
        print("Cleaning up...")
        cleanup()
        print("Done.")


if __name__ == "__main__":
    main()
