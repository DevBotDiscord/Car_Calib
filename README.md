# car-calib v2 — Autonomous RC Car Vision + USB/MQTT Actuator Bridge

Autonomous RC car stack for lane/tile-gap vision steering, route recording, dashboard control, and actuator output through either Raspberry Pi MQTT bridge or USB serial ESP32/ESP8266 boards.

The MiniPC runs OpenCV vision at ~30 Hz, publishes steering commands over MQTT, serves an HTTPS dashboard/MJPEG stream, records route datasets, and can flash USB actuator firmware from the dashboard. The actuator side can be:

- Raspberry Pi bridge (`scripts/rpi/bridge.py`) driving GPIO through `pigpio`.
- ESP32 USB serial board running `firmware/esp32_mqtt_bridge_*`.
- ESP8266 NodeMCU USB serial board running `firmware/esp8266_serial_bridge`.

---

## Architecture

```text
[Camera]
  ↓
MiniPC / vision container
  ├─ main.py: OpenCV detector + SteeringController + route logging
  ├─ Mosquitto broker: MQTT port 1883
  ├─ HTTPS dashboard: stream/status/routes/tuning/firmware update
  └─ Optional USB serial actuator bridge
       ├─ ESP32 Dev Module / ESP32-S3
       └─ ESP8266 NodeMCU

MQTT actuator path:
MiniPC → car/servo/angle → Raspberry Pi bridge → pigpio → servo/base/relay

USB actuator path:
MiniPC MQTT callbacks → runtime/esp32_serial_bridge.py → USB serial line protocol → ESP32/ESP8266 pins
```

Steering convention across all paths:

```text
more negative angle = LEFT
more positive angle = RIGHT
```

---

## Repository map

```text
car-calibv2/
├── main.py                         # Vision entrypoint + dashboard/server wiring
├── process_video.py                # Offline video processor using same detector path
├── config/settings.py              # Environment-backed settings
├── control/                        # Steering/PID controllers
├── drivers/                        # Servo MQTT driver + MQTT control subscriber
├── models/                         # RobotState / FSM state
├── vision/                         # OpenCV LineDetector
├── runtime/
│   ├── https_stream.py             # FastAPI HTTPS MJPEG dashboard + APIs
│   ├── route_logging.py            # Route session summaries/acceptance
│   ├── route_script.py             # Dashboard script runner
│   ├── esp32_serial_bridge.py      # USB serial actuator bridge (ESP32/ESP8266 handshake)
│   ├── esp32_flasher.py            # Board detect + arduino-cli compile/upload
│   └── dashboard/                  # Static HTML/CSS/JS dashboard
├── scripts/rpi/                    # Raspberry Pi pigpio MQTT bridge
├── firmware/
│   ├── esp32_mqtt_bridge_esp32/    # ESP32 Dev Module sketch
│   ├── esp32_mqtt_bridge_esp32s3/  # ESP32-S3 sketch
│   ├── esp32_mqtt_bridge/          # Legacy/common ESP32 sketch copy
│   └── esp8266_serial_bridge/      # ESP8266 NodeMCU sketch
├── docker/
│   ├── vision/Dockerfile           # MiniPC image + arduino-cli ESP32/ESP8266 cores
│   ├── rpi/Dockerfile              # RPi bridge image + pigpio
│   └── mosquitto/mosquitto.conf
├── docker-compose.vision.yml       # MiniPC vision + MQTT stack
├── docker-compose.rpi.yml          # RPi bridge stack
├── deploy_production.sh            # Tar/SCP/docker compose deployment
├── .env.example
├── .env.production.example
└── tests/
```

---

## Hardware modes

### 1. Raspberry Pi MQTT bridge

Use when Raspberry Pi directly controls servo/base/relay through GPIO.

- MiniPC publishes `car/servo/angle`.
- RPi subscribes and applies steering through `pigpio`.
- RPi publishes telemetry to `car/status` and E-stop state to `ugv/rpi/estop`.
- Gamepad/keyboard override lives on RPi.

Production compose:

```bash
./deploy_production.sh minipc
./deploy_production.sh ras
```

### 2. ESP32 / ESP8266 USB serial actuator

Use when MiniPC has actuator board plugged by USB.

- Enable with `ESP32_SERIAL_ENABLED=true` and `ACTUATOR_MODE=auto` or `esp32`.
- `runtime/esp32_serial_bridge.py` scans `/dev/ttyUSB*` and `/dev/ttyACM*`.
- It sends `WHO`; accepted banners:
  - `CARCALIB-ESP32 v1`
  - `CARCALIB-ESP8266 v1`
- It pushes runtime config via `CFG {json}`.
- It forwards MQTT actuator commands as serial lines.
- It republishes board telemetry into the same dashboard MQTT pipeline.

`ACTUATOR_MODE`:

| Value | Behavior |
|---|---|
| `auto` | Try USB board, fall back to MQTT/RPi after `ESP32_SCAN_TIMEOUT_S`. |
| `esp32` | USB actuator only; scan forever. Name kept for backwards compatibility, supports ESP32 and ESP8266 banners. |
| `mqtt` | Never start USB bridge; use MQTT/RPi path only. |

---

## USB serial line protocol

MiniPC → board:

```text
WHO
CFG {json}
SERVO <angle_deg>
BASE FORWARD|BACKWARD|STOP|LOCK|UNLOCK|TURN_LEFT|TURN_RIGHT
RELAY ON|OFF
ESTOP_RESET
PING
```

Board → MiniPC:

```text
CARCALIB-ESP32 v1
CARCALIB-ESP8266 v1
CFGOK
PONG
TEL {json}
ESTOP {json}
LOG <message>
```

Telemetry is forwarded to MQTT `car/status`; E-stop events are forwarded to `ugv/rpi/estop`.

---

## Dashboard

Enable dashboard/stream:

```bash
MAIN_HTTPS_STREAM_ENABLED=true
MAIN_HTTPS_STREAM_PUBLIC=true
MAIN_HTTPS_STREAM_HOST=0.0.0.0
MAIN_HTTPS_STREAM_PORT=8443
python main.py --stream --public --host 0.0.0.0 --port 8443
```

Open:

```text
https://<minipc-ip>:8443/dashboard
```

Main endpoints:

| Endpoint | Purpose |
|---|---|
| `/dashboard` | Web UI. |
| `/stream.mjpg` | MJPEG stream. |
| `/snapshot.jpg` | Latest frame snapshot. |
| `/status` | Vision telemetry + RPi/USB actuator status. |
| `/route/script` | Submit route script. |
| `/route/script/status` | Route script runner status. |
| `/routes/list` | Recent route summaries. |
| `/control/params` | Live steering controller params. |
| `/control/presets` | Tuning preset CRUD. |
| `/esp32/status` | USB actuator flasher/connection/board status. |
| `/esp32/firmware` | Optional uploaded `.ino` override. |
| `/esp32/flash` | Detect board, compile matching sketch, upload. |

The firmware panel now auto-detects connected board through `arduino-cli board list --format json` and compiles the matching sketch:

| Detected board | FQBN | Sketch |
|---|---|---|
| ESP32 Dev Module | `esp32:esp32:esp32` | `firmware/esp32_mqtt_bridge_esp32/esp32_mqtt_bridge_esp32.ino` |
| ESP32-S3 Dev Module | `esp32:esp32:esp32s3` | `firmware/esp32_mqtt_bridge_esp32s3/esp32_mqtt_bridge_esp32s3.ino` |
| ESP8266 NodeMCU 1.0 | `esp8266:esp8266:nodemcuv2` | `firmware/esp8266_serial_bridge/esp8266_serial_bridge.ino` |

If detection fails, it falls back to `ESP32_FQBN`.

---

## Firmware flashing from dashboard

The MiniPC Docker image installs `arduino-cli`, `esp32:esp32`, and `esp8266:esp8266` cores when `INSTALL_ARDUINO_CLI=true`.

Manual compile examples:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/esp32_mqtt_bridge_esp32
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/esp32_mqtt_bridge_esp32s3
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 firmware/esp8266_serial_bridge
```

Manual upload examples:

```bash
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 firmware/esp32_mqtt_bridge_esp32
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32s3 firmware/esp32_mqtt_bridge_esp32s3
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp8266:esp8266:nodemcuv2 firmware/esp8266_serial_bridge
```

Dashboard flow:

1. Optional: drop `.ino` override.
2. Click `compile + update detected board`.
3. Backend detects port/board.
4. Backend compiles matching FQBN/sketch.
5. Serial bridge pauses to release the port.
6. Upload runs.
7. Serial bridge resumes.

---

## Configuration

Copy examples:

```bash
cp .env.example .env
cp .env.production.example .env.production
```

Important MiniPC settings:

| Variable | Default | Description |
|---|---:|---|
| `MAIN_TARGET_HZ` | `30.0` | Vision loop frequency. |
| `MAIN_CAMERA_INDEX` | `0` | Camera index. |
| `MAIN_HTTPS_STREAM_ENABLED` | `false` | Enable HTTPS dashboard/stream. |
| `MAIN_HTTPS_STREAM_PUBLIC` | `false` | Bind public host when using CLI `--public`. |
| `MAIN_HTTPS_TOKEN` | empty | Optional dashboard token. |
| `MQTT_BROKER_HOST` | `127.0.0.1` | MQTT broker host. |
| `MQTT_BROKER_PORT` | `1883` | MQTT broker port. |
| `MQTT_SERVO_TOPIC` | `car/servo/angle` | Steering topic. |
| `MQTT_BASE_COMMAND_TOPIC` | `car/base/command` | Base command topic. |
| `MQTT_STATUS_TOPIC` | `car/status` | Telemetry topic. |
| `DRIVER_SERVO_MQTT_ENABLED` | `false` | Publish servo angle over MQTT. |

USB actuator settings:

| Variable | Default | Description |
|---|---:|---|
| `ESP32_SERIAL_ENABLED` | `false` | Enable USB serial actuator bridge. |
| `ESP32_SERIAL_BAUD` | `115200` | Serial baud. |
| `ESP32_SERIAL_PORT_GLOBS` | `/dev/ttyUSB*,/dev/ttyACM*` | Port scan globs. |
| `ACTUATOR_MODE` | `auto` | `auto`, `esp32`, or `mqtt`. |
| `ESP32_SCAN_TIMEOUT_S` | `10.0` | Auto-mode USB scan timeout. |
| `ESP32_FQBN` | `esp32:esp32:esp32` | ESP32/fallback FQBN. |
| `ESP8266_FQBN` | `esp8266:esp8266:nodemcuv2` | ESP8266 FQBN. |

Actuator pin/config values sent to board via `CFG`:

| Variable | Default | Description |
|---|---:|---|
| `SERVO_PIN` | `12` | Servo pin for USB board config. ESP8266 firmware has NodeMCU-safe defaults but accepts override. |
| `SERVO_MIN_PULSE` | `0.0005` | Minimum servo pulse seconds. |
| `SERVO_MAX_PULSE` | `0.0025` | Maximum servo pulse seconds. |
| `SERVO_CENTER_ANGLE` | `-8` | Signed center angle for actuator path. |
| `SERVO_MAX_ANGLE_DEG` | `45` | Limit span around center. |
| `STEER_DEADBAND_DEG` | `1.0` | Ignore tiny steering changes. |
| `BASE_OUT1`, `BASE_OUT2`, `BASE_OUT3` | `17`, `27`, `22` | 3-pin base motor outputs. |
| `RELAY_PIN` | `5` | Relay output. |
| `ESTOP_GPIO` | `6` | E-stop input. |
| `ESTOP_ACTIVE_LOW` | `true` | Active-low E-stop input. |
| `TELEMETRY_INTERVAL_SEC` | `1.0` | Board telemetry interval. |

Vision/controller settings are in `config/settings.py` and `.env.example`.

---

## MQTT topics

| Topic | Direction | Payload | Retain |
|---|---|---|---|
| `car/servo/angle` | MiniPC → actuator | `{"angle": int}` or float string | no |
| `car/base/command` | dashboard/scripts → actuator | `FORWARD/BACKWARD/STOP/LOCK/UNLOCK/TURN_LEFT/TURN_RIGHT` | no |
| `car/relay` | dashboard/scripts → actuator | `ON/OFF` | no |
| `car/control/script_active` | dashboard → MiniPC | `ON/OFF/1/0/TRUE/FALSE/START/STOP` | no |
| `car/control/route` | RPi → MiniPC | `START/STOP` | no |
| `car/control/mode` | RPi → MiniPC | `AUTO/CRUISE/SQUARE/REMOTE_STEER/RECORD` | yes |
| `car/status` | actuator/RPi → dashboard | JSON telemetry | yes |
| `ugv/rpi/estop` | actuator/RPi → dashboard | JSON E-stop transition | yes |
| `car/control/estop_reset` | dashboard → actuator | reset request | no |

---

## Running locally

Install Python deps:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Run vision loop:

```bash
python main.py
```

Run with dashboard:

```bash
MAIN_HTTPS_STREAM_ENABLED=true python main.py --stream --public --host 0.0.0.0 --port 8443
```

Run offline video:

```bash
python process_video.py videos/6.mp4 --show-guidance-overlay --show-detector-debug
```

---

## Docker deployment

MiniPC:

```bash
docker compose -f docker-compose.vision.yml up --build -d
```

Raspberry Pi:

```bash
docker compose -f docker-compose.rpi.yml up --build -d
```

Production deploy:

```bash
./deploy_production.sh minipc
./deploy_production.sh ras
./deploy_production.sh all
```

The production deploy creates a tarball, copies to target, extracts under releases, updates `current`, and runs Docker Compose with stable project names.

---

## Route recording

Route sessions start/stop through `car/control/route` or dashboard script runner. Route outputs live under `ROUTE_LOG_ROOT` (`/data/routes` in production) and include:

- `route_frames.csv`
- `route.mp4`
- `route_summary.json`
- optional zip archive for download

Acceptance defaults:

| Variable | Default | Meaning |
|---|---:|---|
| `ROUTE_ACCEPT_MIN_FRAMES` | `60` | Minimum frames. |
| `ROUTE_ACCEPT_MAX_HW_ERRORS` | `0` | Maximum hardware errors. |
| `ROUTE_ACCEPT_MAX_GAP_RATIO` | `0.25` | Maximum vision gap ratio. |

---

## E-stop behavior

E-stop latches actuator into safe state:

- Base stops.
- Servo centers/releases.
- Relay blinks for visual warning.
- Dashboard health banner shows E-stop active.
- Reset only clears when physical button reads safe.

Both RPi bridge and USB board publish E-stop transitions to `ugv/rpi/estop`.

---

## Tests / verification

Run all tests:

```bash
python -m pytest tests/
```

Compile Python modules:

```bash
python -m compileall -q runtime main.py
```

Known note: current `tests/test_https_stream.py` may fail if tests expect `HttpsMjpegServer(restart_callback=...)` while runtime constructor does not expose that argument.

---

## License

See [`LICENSE`](LICENSE).
