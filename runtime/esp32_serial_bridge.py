"""MiniPC ↔ ESP32 serial actuator bridge.

The ESP32 runs firmware/esp32_mqtt_bridge as a USB-serial actuator (no WiFi).
This module:

* Scans serial ports (/dev/ttyUSB*, /dev/ttyACM*) and handshakes with "WHO".
  Only a port that replies "CARCALIB-ESP32 ..." is accepted — so a random
  serial device never gets mistaken for the controller.
* Pushes the live env-derived config down with "CFG {json}" on connect.
* Subscribes to MQTT (car/servo/angle, car/base/command, car/relay,
  car/control/estop_reset) and forwards each as a serial command line.
* Reads telemetry ("TEL {json}") and E-stop transitions ("ESTOP {json}")
  from the ESP32 and republishes them to MQTT (car/status, ugv/rpi/estop)
  so the existing dashboard pipeline works unchanged.

Resilience: the scan/connect loop never raises out of the worker thread.
If no ESP32 is found, or the link drops, it logs and keeps retrying — the
same "probe forever, never crash" behaviour as the camera acquisition loop.
"""

from __future__ import annotations

import glob
import json
import logging
import threading
import time
from typing import Any

try:
    import serial  # pyserial
except ImportError:  # pragma: no cover - optional dependency
    serial = None  # type: ignore[assignment]

try:
    import paho.mqtt.client as mqtt
except ImportError:  # pragma: no cover
    mqtt = None  # type: ignore[assignment]

logger = logging.getLogger(__name__)

HANDSHAKE_TOKEN = "CARCALIB-ESP32"
DEFAULT_PORT_GLOBS = ("/dev/ttyUSB*", "/dev/ttyACM*")


def _scan_ports(globs: tuple[str, ...]) -> list[str]:
    found: list[str] = []
    for pattern in globs:
        for path in sorted(glob.glob(pattern)):
            if path not in found:
                found.append(path)
    return found


class ESP32SerialBridge:
    """Owns the serial link, the MQTT client, and the background threads."""

    def __init__(
        self,
        *,
        mqtt_host: str,
        mqtt_port: int,
        mqtt_username: str = "",
        mqtt_password: str = "",
        servo_topic: str = "car/servo/angle",
        base_topic: str = "car/base/command",
        relay_topic: str = "car/relay",
        estop_reset_topic: str = "car/control/estop_reset",
        status_topic: str = "car/status",
        estop_topic: str = "ugv/rpi/estop",
        baud: int = 115200,
        port_globs: tuple[str, ...] = DEFAULT_PORT_GLOBS,
        device_config: dict[str, Any] | None = None,
        scan_timeout_s: float | None = None,
    ) -> None:
        self._mqtt_host = mqtt_host
        self._mqtt_port = mqtt_port
        self._mqtt_username = mqtt_username
        self._mqtt_password = mqtt_password
        self._servo_topic = servo_topic
        self._base_topic = base_topic
        self._relay_topic = relay_topic
        self._estop_reset_topic = estop_reset_topic
        self._status_topic = status_topic
        self._estop_topic = estop_topic
        self._baud = baud
        self._port_globs = port_globs
        self._device_config = device_config or {}
        self._scan_timeout_s = scan_timeout_s  # None = scan forever (esp32 mode)
        self._ever_connected = False

        self._ser: Any = None
        self._ser_lock = threading.Lock()
        self._mqtt_client: Any = None
        self._running = False
        self._conn_thread: threading.Thread | None = None
        self._read_thread: threading.Thread | None = None

    # ------------------------------------------------------------------ #
    # lifecycle
    # ------------------------------------------------------------------ #
    def start(self) -> None:
        if serial is None:
            logger.warning("pyserial not installed; ESP32 serial bridge disabled")
            return
        if mqtt is None:
            logger.warning("paho-mqtt not available; ESP32 serial bridge disabled")
            return
        self._running = True
        self._setup_mqtt()
        self._conn_thread = threading.Thread(target=self._connect_loop, daemon=True, name="esp32-connect")
        self._conn_thread.start()
        logger.info("ESP32 serial bridge started (scan globs=%s)", ",".join(self._port_globs))

    def close(self) -> None:
        self._running = False
        with self._ser_lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:  # noqa: BLE001
                    pass
                self._ser = None
        if self._mqtt_client is not None:
            try:
                self._mqtt_client.disconnect()
                self._mqtt_client.loop_stop()
            except Exception:  # noqa: BLE001
                pass

    # ------------------------------------------------------------------ #
    # MQTT
    # ------------------------------------------------------------------ #
    def _setup_mqtt(self) -> None:
        api = getattr(mqtt, "CallbackAPIVersion", None)
        if api is not None:
            client = mqtt.Client(callback_api_version=api.VERSION1, client_id="minipc-esp32-bridge")
        else:
            client = mqtt.Client(client_id="minipc-esp32-bridge")
        if self._mqtt_username:
            client.username_pw_set(self._mqtt_username, self._mqtt_password)
        client.on_connect = self._on_connect
        client.on_message = self._on_message
        client.reconnect_delay_set(min_delay=1, max_delay=5)
        client.connect_async(self._mqtt_host, self._mqtt_port, keepalive=60)
        client.loop_start()
        self._mqtt_client = client

    def _on_connect(self, client, userdata, flags, rc, properties=None) -> None:
        del userdata, flags, properties
        if rc != 0:
            logger.error("ESP32 bridge MQTT connect failed rc=%s", rc)
            return
        client.subscribe(self._servo_topic)
        client.subscribe(self._base_topic)
        client.subscribe(self._relay_topic)
        client.subscribe(self._estop_reset_topic)
        logger.info("ESP32 bridge MQTT connected; subscribed to actuator topics")

    def _on_message(self, client, userdata, message) -> None:
        del client, userdata
        try:
            payload = message.payload.decode("utf-8").strip()
        except UnicodeDecodeError:
            return
        topic = message.topic
        if topic == self._servo_topic:
            angle = self._parse_servo_angle(payload)
            if angle is not None:
                self._send(f"SERVO {angle:.2f}")
        elif topic == self._base_topic:
            self._send(f"BASE {payload.upper()}")
        elif topic == self._relay_topic:
            self._send(f"RELAY {payload.upper()}")
        elif topic == self._estop_reset_topic:
            self._send("ESTOP_RESET")

    @staticmethod
    def _parse_servo_angle(payload: str) -> float | None:
        payload = payload.strip()
        if not payload:
            return None
        try:
            if payload.startswith("{"):
                return float(json.loads(payload).get("angle"))
            return float(payload)
        except (ValueError, TypeError, json.JSONDecodeError):
            return None

    # ------------------------------------------------------------------ #
    # Serial connect/scan loop — probe forever, never crash
    # ------------------------------------------------------------------ #
    def _connect_loop(self) -> None:
        scan_started = time.monotonic()
        while self._running:
            try:
                port = self._scan_and_handshake()
                if port is None:
                    # auto mode: give up after scan_timeout_s if never connected,
                    # so the MQTT/RPi path stays the actuator. esp32 mode
                    # (scan_timeout_s is None) scans forever.
                    if (
                        self._scan_timeout_s is not None
                        and not self._ever_connected
                        and (time.monotonic() - scan_started) >= self._scan_timeout_s
                    ):
                        logger.warning(
                            "No ESP32 found within %.0fs; falling back to MQTT actuator path.",
                            self._scan_timeout_s,
                        )
                        self._running = False
                        return
                    logger.warning(
                        "No ESP32 found on %s; retrying.", ",".join(self._port_globs)
                    )
                    time.sleep(2.0)
                    continue
                logger.info("ESP32 connected on %s", port)
                self._ever_connected = True
                self._push_config()
                self._read_loop()  # blocks until link drops
            except Exception as exc:  # noqa: BLE001 — never let the thread die
                logger.warning("ESP32 bridge link error: %s; rescanning.", exc)
            finally:
                with self._ser_lock:
                    if self._ser is not None:
                        try:
                            self._ser.close()
                        except Exception:  # noqa: BLE001
                            pass
                        self._ser = None
            time.sleep(1.0)

    def _scan_and_handshake(self) -> str | None:
        for port in _scan_ports(self._port_globs):
            try:
                ser = serial.Serial(port, self._baud, timeout=1.0)
            except Exception as exc:  # noqa: BLE001
                logger.debug("ESP32 scan: cannot open %s: %s", port, exc)
                continue
            try:
                time.sleep(2.0)  # ESP32 resets on port open; wait for boot
                ser.reset_input_buffer()
                ser.write(b"WHO\n")
                ser.flush()
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline:
                    line = ser.readline().decode("utf-8", "ignore").strip()
                    if not line:
                        continue
                    if line.startswith(HANDSHAKE_TOKEN):
                        with self._ser_lock:
                            self._ser = ser
                        return port
                logger.debug("ESP32 scan: %s did not handshake", port)
            except Exception as exc:  # noqa: BLE001
                logger.debug("ESP32 scan: handshake error on %s: %s", port, exc)
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass
        return None

    def _push_config(self) -> None:
        self._send("CFG " + json.dumps(self._device_config, separators=(",", ":")))

    def _read_loop(self) -> None:
        while self._running:
            with self._ser_lock:
                ser = self._ser
            if ser is None:
                return
            try:
                raw = ser.readline()
            except Exception as exc:  # noqa: BLE001
                logger.warning("ESP32 read failed: %s", exc)
                return
            if raw == b"":
                continue  # timeout, keep waiting
            line = raw.decode("utf-8", "ignore").strip()
            if not line:
                continue
            self._handle_device_line(line)

    def _handle_device_line(self, line: str) -> None:
        if line.startswith("TEL "):
            self._republish(self._status_topic, line[4:], retain=True)
        elif line.startswith("ESTOP "):
            self._republish(self._estop_topic, line[6:], retain=True)
        elif line.startswith("LOG "):
            logger.info("ESP32: %s", line[4:])
        # WHO/CFGOK/PONG banners are ignored here

    def _republish(self, topic: str, json_text: str, retain: bool) -> None:
        if self._mqtt_client is None:
            return
        try:
            json.loads(json_text)  # validate before forwarding
        except json.JSONDecodeError:
            logger.debug("ESP32 bad json on %s: %s", topic, json_text)
            return
        try:
            self._mqtt_client.publish(topic, json_text, qos=1, retain=retain)
        except Exception as exc:  # noqa: BLE001
            logger.debug("ESP32 republish failed: %s", exc)

    # ------------------------------------------------------------------ #
    # serial write
    # ------------------------------------------------------------------ #
    def _send(self, line: str) -> None:
        with self._ser_lock:
            ser = self._ser
            if ser is None:
                return
            try:
                ser.write((line + "\n").encode("utf-8"))
                ser.flush()
            except Exception as exc:  # noqa: BLE001
                logger.warning("ESP32 write failed: %s", exc)
