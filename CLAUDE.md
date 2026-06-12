# car-calib — Autonomous RC Car with Vision Steering + Gamepad Override

Vision-based lane detection on MiniPC publishes steering over MQTT.
Raspberry Pi subscribes, drives servo + motor + relay via direct pigpio.
Gamepad/keyboard on RPi for manual override + cruise + square pattern.
Bidirectional MQTT: RPi publishes mode/route to MiniPC for route session tagging.

## Architecture

```
[Camera] → MiniPC (main.py, OpenCV lane detection, PID controller)
              ↓ MQTT car/servo/angle  (JSON {"angle": int} or float)
         [Mosquitto broker on MiniPC, host network, port 1883]
              ↑↓
[Raspberry Pi] → scripts/rpi/bridge.py → pigpio → Servo + Base + Relay
                  ↕ gamepad/keyboard (evdev) — local manual override
                  ↑ MQTT car/control/route + car/control/mode → MiniPC
```

- Vision loop: 30Hz, publishes `{"angle": int}` to MQTT
- RPi bridge loop: ~100Hz, polls input → decides → writes GPIO + publishes mode/route
- Telemetry: RPi publishes retained `car/status` every `TELEMETRY_INTERVAL_SEC` (1s default); MQTT LWT republishes on disconnect
- E-Stop: NC push-button on GPIO 6 latches the bridge into a safe state until `try_reset()` is called and the button reads safe; `base.py`/`steering.py` gate command writes while latched
- MiniPC subscribes to `car/control/route` (START/STOP), `car/control/mode`, `car/status`, and `ugv/rpi/estop` for route session lifecycle + dashboard telemetry
- Steering convention: more negative angle = LEFT, more positive = RIGHT
  Default (current code): CENTER_ANGLE=-8, SERVO_MAX_ANGLE_DEG=45 → LEFT_LIMIT=-53, RIGHT_LIMIT=+37
- IMU heading-hold: removed (mpu_home_detect.py kept as standalone test script, not wired into bridge)

## Directory Map

```
car-calib/
├── main.py                    # Vision entrypoint (MiniPC): camera → calibration → PID → MQTT
├── process_video.py           # Offline video processor, same pipeline as main.py
├── controlv8.py               # Standalone gamepad-to-pigpio reference script (bare metal RPi)
├── mpu_home_detect.py         # MPU6050 test script: gyro calibrate, yaw tracking, home detect
├── deploy_production.sh       # Production deploy: tar → SCP → docker compose up on target(s)
├── docker-compose.vision.yml  # MiniPC stack: vision + mosquitto
├── docker-compose.rpi.yml     # RPi stack: rpi-mqtt-bridge (host network, privileged)
├── requirements.txt           # Pip deps for vision (MiniPC)
├── requirements-rpi.txt       # Pip deps for bridge (RPi) — pigpio, evdev, paho-mqtt, python-dotenv
│
├── config/                    # Vision config (settings.py — env-based)
├── control/                   # PID controller (servo_pid.py)
├── drivers/                   # Servo driver (MQTT publish)
├── models/                    # RobotState FSM
├── runtime/                   # HTTPS MJPEG stream, dashboard, route logging
│   └── dashboard/             # Static HTML/JS/CSS dashboard (status bar + tables + event log)
├── vision/                    # Unified calibration vision package
├── visualization/             # PID simulation visualizer
├── firmware/                  # ESP32 MQTT bridge firmware
│
├── docker/
│   ├── vision/Dockerfile      # MiniPC image (python:3.11-slim + ffmpeg, libgl, opencv)
│   ├── rpi/Dockerfile         # RPi image (multi-stage: build pigpio from source + wheels)
│   │   └── entrypoint.sh      # Start pigpiod, then exec python bridge
│   └── mosquitto/mosquitto.conf  # MQTT broker config (listener 1883 0.0.0.0, anonymous)
│
├── scripts/
│   ├── rpi/                   # RPi bridge (modular, 9 files)
│   │   ├── bridge.py          # Main entrypoint: setup, main loop, cleanup, banner
│   │   ├── config.py          # All env vars + global state (~200 lines)
│   │   ├── base.py            # Motor GPIO: forward/backward/stop/lock/unlock/turn_left/turn_right + estop gate
│   │   ├── steering.py        # Servo: pulse calc, apply_steering, deadband, remote angle mapping + estop gate
│   │   ├── controls.py        # InputController: mode logic, cruise/square/relay triggers
│   │   ├── input_handler.py   # evdev reader: gamepad + keyboard state
│   │   ├── estop.py           # E-Stop GPIO 6 NC button: latch, debounce, blink relay
│   │   ├── logging_utils.py   # Compact stdout logger
│   │   └── mqtt_client.py     # MQTT connect, callbacks, publish_status/estop/mode/route_control + LWT
│   ├── servo_bridge_common.py # Shared helpers: clamp_angle, angle_within_limits
│   ├── input_device_helpers.py# Gamepad/keyboard detection helpers
│   ├── mqtt_servo_command.py  # CLI: publish servo angle to MQTT
│   ├── mqtt_base_command.py   # CLI: publish base command to MQTT
│   └── visualize_pid_simulation_standalone.py  # Plot CSV logs
│
├── docs/                      # Architecture docs (docker_images.md, servo_bridge.md, etc.)
├── tests/                     # Unit tests
└── .env.production.example    # Template for production env (copy to .env.production)
```

## Key Env Vars (.env / .env.production)

### Deployment
- `PROJECT_NAME=car-calib`
- `MINIPC_HOST`, `MINIPC_USER`, `MINIPC_PASSWORD` — MiniPC SSH credentials
- `RPI_HOST`, `RPI_USER`, `RPI_PASSWORD` — Raspberry Pi SSH credentials
- `MINIPC_COMPOSE_PROJECT_NAME=car-calib-minipc`
- `RPI_COMPOSE_PROJECT_NAME=car-calib-ras`
- `DEPLOY_KEEP_RELEASES=3`

### MQTT
- `MQTT_BROKER_HOST` — broker IP (RPi uses this to connect)
- `MQTT_BROKER_PORT=1883`
- `MQTT_SERVO_TOPIC=car/servo/angle`

### Servo / Steering
- `SERVO_CENTER_ANGLE=-8` — current code default center angle (RPi bridge config)
- `SERVO_MAX_ANGLE_DEG=45` — limits = CENTER ± this
- `SERVO_PIN=12` — GPIO pin for servo PWM
- `SERVO_MIN_PULSE=0.0005`, `SERVO_MAX_PULSE=0.0025` — pulse range (500-2500µs)
- `STEER_DEADBAND_DEG=1.0` — ignore tiny steering changes from same source
- `SERVO_RELEASE_IDLE=true` — release PWM when idle

### Base Motor
- `BASE_OUT1=17`, `BASE_OUT2=27`, `BASE_OUT3=22` — motor control pins
- Forward=(0,1,0), Backward=(0,0,1), Stop=(0,0,0), Lock=(1,0,1), Unlock=(1,1,0)

### Relay
- `RELAY_PIN=5`
- `RELAY_BLINK_INTERVAL_S=0.12`

### E-Stop (RPi)
- `ESTOP_GPIO=6` — NC push-button to GND
- `ESTOP_ACTIVE_LOW=true` — pull-up enabled, button press pulls pin LOW
- `ESTOP_DEBOUNCE_US=5000` — pigpio hardware glitch filter
- `ESTOP_LATCH_STABLE_S=0.02` — software stable-window after edge
- `ESTOP_BLINK_RELAY_S=5.0` — relay blink duration on latch (visual warning)

### Telemetry
- `TELEMETRY_INTERVAL_SEC=1.0` — heartbeat publish interval (was 10 s previously)
- `MQTT_ESTOP_TOPIC=ugv/rpi/estop` — retained E-stop transitions
- `MQTT_STATUS_TOPIC=car/status` — retained rich health JSON + LWT

### Cruise / Square Pattern
- `CRUISE_DURATION_S=30`
- `SQUARE_STRAIGHT_DURATION_S=5.0`
- `SQUARE_TURN_DURATION_S=1.0`

### Input shaping
- `GAMEPAD_STEER_DEADZONE=0.12`
- `GAMEPAD_DRIVE_DEADZONE=0.20`
- `MANUAL_STEER_HOLD=0.25`
- `SERVO_STEP=20.0` (keyboard steer step)

### Vision (MiniPC)
- `MAIN_TARGET_HZ=30.0`, `MAIN_CAMERA_INDEX=0`
- `PID_KP`, `PID_KI`, `PID_KD` — vision PID gains
- `DRIVER_SERVO_MQTT_ENABLED=true`
- `MAIN_SHOW_PREVIEW=false` — must be false in Docker (no GUI)

## RPi Bridge Decision Priority (`InputController.process`)

```
1. Cruise timeout               → stop cruise, publish AUTO + route STOP
2. Square pattern active        → force FORWARD + phase steering
3. Pending relay edge command   → emit ON/OFF command
4. RB hold (>0.3s) blink relay  → periodic ON/OFF toggle
5. Cruise active                → FORWARD + manual_steer=False (MQTT steer)
6. Keyboard steer               → C center, D right-step
7. Gamepad steer                → right-stick X (unless remote_steer_only)
8. Recenter pending             → one-shot center after release
9. Manual override hold window  → keep last manual steer angle
10. Idle/remote                 → manual_steer=False (MQTT path)
```

## Docker Build & Deploy

### Local
```bash
docker compose -f docker-compose.vision.yml up --build -d   # MiniPC
docker compose -f docker-compose.rpi.yml up --build -d      # Raspberry Pi
```

### Production
```bash
./deploy_production.sh minipc   # deploy vision + mqtt to MiniPC
./deploy_production.sh ras      # deploy bridge to Raspberry Pi
./deploy_production.sh all      # deploy both
```

Deploy flow: tar repo → SCP to target → extract to releases/<version>/ → symlink current → docker compose up --build

RPi Dockerfile is multi-stage:
1. Builder: apt install gcc/make/python3-dev/wget → pip wheel all deps → build pigpio from source (GitHub v78)
2. Final: apt install libevdev2/i2c-tools/python3-smbus → pip install from pre-built wheels → copy pigpiod/libs from builder

`PYTHONPATH=/usr/lib/python3/dist-packages` needed for smbus (installed via apt, not pip).

## IMU Status

- IMU heading-hold not integrated in current RPi bridge implementation.
- `scripts/rpi/bridge.py` + `scripts/rpi/controls.py` have no IMU control path.
- `mpu_home_detect.py` remains as standalone test utility.

## Gamepad Mapping (Xbox 360 / evdev)

| Button | Code | Action |
|--------|------|--------|
| A | BTN_SOUTH (304) | Stop base |
| B | BTN_EAST (305) | Toggle square pattern |
| X | BTN_WEST (306) | Toggle high-level route recording holder (RECORD mode when active alone) |
| Y | BTN_NORTH (307) | Toggle remote steer only |
| LB | BTN_TL (310) | Toggle cruise |
| RB | BTN_TR (311) | Relay tap/hold (tap=toggle, hold=blink) |
| START | BTN_START (315) | Quit |
| SELECT/BACK | BTN_SELECT (314) | (unbound) |
| Right stick X | ABS_RX (3) | Steering |
| Left stick Y | ABS_Y (1) | Drive (forward/backward) |
| D-pad up/down | ABS_HAT0Y (17) | HAT state tracked (no center adjust action in current controller) |

## Keyboard Mapping

| Key | Action |
|-----|--------|
| W | Forward |
| S | Backward |
| D | Steer right step |
| C | Center steering |
| X | Stop |
| L | Lock |
| U | Unlock |
| 1/2 | Adjust center angle ±1 |
| ENTER | Toggle cruise |
| Q | Quit |

Note: current controller does not bind `A` for steer-left in keyboard path.

## Steering Direction Convention (all paths synced)

```
More negative angle → STEER LEFT
More positive angle → STEER RIGHT
```

- Gamepad stick X axis maps inverted to angle: stick right → negative target, stick left → positive target (see `_gamepad_steer` formula in `controls.py`)
- Keyboard D → `steer_angle -= SERVO_STEP` (steer left by sign convention)
- Keyboard C → snap to `CENTER_ANGLE`
- MQTT vision angle treated as direct signed angle if within `[LEFT_LIMIT, RIGHT_LIMIT]`; legacy 60..120 input range mapped via `map_remote_angle()`

All paths gate through `apply_steering()` → clamp to [LEFT_LIMIT, RIGHT_LIMIT] → deadband check → `_write_servo()` → `gpio.set_servo_pulsewidth(pin, pulse_us)`

## Deploy Notes

- `<model>` in config files provides model hints — use them as-is
- deploy_production.sh uses project-name-based compose (-p) to avoid stacking containers per version
- RPi container uses `network_mode: host` and `privileged: true` — needs GPIO + /dev/input + pigpiod on host
- Legacy timestamp-based compose projects auto-cleaned up on first deploy with new project names
- mDNS (.local) hostnames don't resolve inside Docker containers — use IP addresses for MQTT_BROKER_HOST
- If host pigpiod runs on port 8888, the container's pigpiod start fails (port in use) — this is OK, bridge connects to host pigpiod

## MQTT Topics

| Topic | Direction | Payload | QoS | Retain |
|-------|-----------|---------|-----|--------|
| `car/servo/angle` | MiniPC → RPi | `{"angle": int}` JSON or float string | 0 | no |
| `car/base/command` | (any) → RPi | `FORWARD/BACKWARD/STOP/LOCK/UNLOCK/TURN_LEFT/TURN_RIGHT` | 1 | no |
| `car/relay` | (any) → RPi | `ON`/`OFF` | 0 | no |
| `car/control/script_active` | dashboard → RPi | `ON/OFF/1/0/TRUE/FALSE/START/STOP` | 0 | no |
| `car/control/route` | RPi → MiniPC | `START`/`STOP` — gates route logging session | 1 | no |
| `car/control/mode` | RPi → MiniPC | `AUTO/CRUISE/SQUARE/REMOTE_STEER/RECORD` | 1 | yes |
| `car/status` | RPi → all | Rich health JSON every `TELEMETRY_INTERVAL_SEC` (1.0s default); also LWT payload `{"rpi_online":false,"reason":"mqtt_lwt"}` if RPi disconnects | 1 | yes |
| `ugv/rpi/estop` (env `MQTT_ESTOP_TOPIC`) | RPi → all | `{"active":bool,"latched_at":float,"ts":float}` on E-stop transitions | 1 | yes |

MiniPC subscribes to `car/control/#` plus `car/status` and `ugv/rpi/estop` via
`drivers/mqtt_control_client.py`; the cached payload is exposed to the
dashboard through the `/status` endpoint as `rpi_status: {online, stale,
age_s, payload}` (stale=true once `age_s > 5.0`).

Vision-side gating: `handle_servo_message` ignores `car/servo/angle` until
the dashboard publishes `car/control/script_active=ON`, so manual
gamepad/keyboard control stays exclusive on the RPi by default.

## Modes (RPi → MiniPC tag)

Route record uses ref-counted holders (`cruise`, `square`, `manual`). START
publishes when first holder is acquired; STOP publishes only after the last
holder releases. Mode label resolved from active holders by priority
`cruise > square > manual > remote_steer > auto`.

- **AUTO** — vision PID drives servo, gamepad/keyboard drives base, no holder
- **REMOTE_STEER** — toggle Y; gamepad steer disabled, MQTT vision drives servo
- **CRUISE** — toggle LB or ENTER; auto-FORWARD `CRUISE_DURATION_S`, remote_steer_only forced ON, acquires `cruise` holder; release on cancel/timeout
- **SQUARE** — toggle B; loops `SQUARE_STRAIGHT_DURATION_S` forward then `SQUARE_TURN_DURATION_S` right at `RIGHT_LIMIT`; acquires `square` holder
- **RECORD** — toggle X; pure manual recording. Logs all frames, accepts cruise/square/manual driving inside same session. Acquires `manual` holder (released on second X press)

## Code Notes

- RPi bridge uses module-level global state (`scripts/rpi/config.py`) accessed via `from . import config` + `config.attr` — no circular imports
- All servo control is direct pigpio, no gpiozero caching/latency
- Vision preview (`cv2.imshow`) is headless-safe: try/except on cv2.error with auto-disable
- Mosquitto broker runs on MiniPC (docker-compose.vision.yml) on host network, port 1883
- MiniPC publishes integer-rounded angles via `ServoDriver._publish_mqtt_angle()` when `DRIVER_SERVO_MQTT_ENABLED=true`; otherwise calls local `_write_angle()` stub
- Route sessions accept ≥`ROUTE_ACCEPT_MIN_FRAMES` (60), `hw_errors ≤ ROUTE_ACCEPT_MAX_HW_ERRORS` (0), `gap_ratio ≤ ROUTE_ACCEPT_MAX_GAP_RATIO` (0.25)
