# car-calib — Autonomous RC Car with Vision Steering + IMU Heading-Hold

Vision-based lane detection on MiniPC publishes steering over MQTT.
Raspberry Pi subscribes, drives servo + motor via direct pigpio.
Gamepad/keyboard for manual override. Optional MPU6050 IMU heading-hold.

## Architecture

```
[Camera] → MiniPC (main.py, OpenCV lane detection, PID controller)
              ↓ MQTT car/servo/angle
         [Mosquitto broker]
              ↓
[Raspberry Pi] → scripts/rpi/bridge.py → pigpio → Servo PWM + Base GPIO
                  ↕ gamepad/keyboard (evdev)
                  ↕ MPU6050 IMU (I2C, gyro Z integration)
```

- Vision loop: 30Hz, publishes `{angle: float}` or plain float to MQTT
- RPi bridge loop: ~100Hz, processes all input sources in priority order
- Steering convention: more negative angle = LEFT, more positive = RIGHT
  Default: LEFT_LIMIT=-71, CENTER_ANGLE=-26, RIGHT_LIMIT=+19

## Directory Map

```
car-calib/
├── main.py                    # Vision entrypoint (MiniPC): camera → detector → PID → MQTT
├── process_video.py           # Offline video processor, same pipeline as main.py
├── controlv8.py               # Standalone gamepad-to-pigpio reference script (bare metal RPi)
├── mpu_home_detect.py         # MPU6050 test script: gyro calibrate, yaw tracking, home detect
├── deploy_production.sh       # Production deploy: tar → SCP → docker compose up on target(s)
├── docker-compose.vision.yml  # MiniPC stack: vision + mosquitto
├── docker-compose.rpi.yml     # RPi stack: rpi-mqtt-bridge (host network, privileged)
├── requirements.txt           # Pip deps for vision (MiniPC)
├── requirements-rpi.txt       # Pip deps for bridge (RPi) — pigpio, evdev, paho-mqtt, mpu6050-raspberrypi
│
├── config/                    # Vision config (settings.py — env-based)
├── control/                   # PID controller (servo_pid.py)
├── drivers/                   # Servo driver (MQTT publish)
├── models/                    # RobotState FSM
├── runtime/                   # HTTPS MJPEG stream, video helpers
├── vision/                    # LineDetector (OpenCV: CLAHE, Canny, Hough)
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
│   ├── rpi/                   # RPi bridge (modular, 8 files)
│   │   ├── bridge.py          # Main entrypoint: setup, main loop, cleanup, banner
│   │   ├── config.py          # All env vars + global state (~170 lines)
│   │   ├── base.py            # Motor GPIO: forward/backward/stop/lock/unlock
│   │   ├── steering.py        # Servo: pulse calc, apply_steering, deadband, remote angle mapping
│   │   ├── imu.py             # MPU6050: calibrate gyro bias, poll, yaw integration, set_home
│   │   ├── controls.py        # process_controls() + gamepad/keyboard input handlers
│   │   ├── cruise.py          # Auto-cruise: timed forward + MQTT steer
│   │   └── mqtt_client.py     # MQTT connect, callbacks, publish_status
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
- `SERVO_CENTER_ANGLE=-26` — mechanical center angle
- `SERVO_MAX_ANGLE_DEG=45` — limits = CENTER ± this (±45° from center)
- `SERVO_PIN=12` — GPIO pin for servo PWM
- `SERVO_MIN_PULSE=0.0005`, `SERVO_MAX_PULSE=0.0025` — MG996R pulse range (500-2500µs)
- `SERVO_RELEASE_IDLE=true` — release PWM when idle

### Base Motor
- `BASE_OUT1=17`, `BASE_OUT2=27`, `BASE_OUT3=22` — motor control pins
- Forward=(0,1,0), Backward=(0,0,1), Stop=(0,0,0), Lock=(1,0,1), Unlock=(1,1,0)

### IMU (optional)
- `IMU_ENABLED=true`
- `IMU_KP=0.12` — P-controller gain for heading correction
- `IMU_STRAIGHT_THRESHOLD_DEG=3.0` — tolerance for "straight"
- `IMU_GYRO_BIAS_SAMPLES=500` — calibration sample count

### Cruise
- `CRUISE_DURATION_S=30` — auto-cruise duration
- `CRUISE_STRAIGHT_FRAMES=5` — consecutive frames to trigger straight detection

### Vision (MiniPC)
- `MAIN_TARGET_HZ=30.0`, `MAIN_CAMERA_INDEX=0`
- `PID_KP`, `PID_KI`, `PID_KD` — vision PID gains
- `DRIVER_SERVO_MQTT_ENABLED=true`
- `MAIN_SHOW_PREVIEW=false` — must be false in Docker (no GUI)

## RPi Bridge Steering Priority (process_controls)

```
1. IMU mode (B toggle)          → heading-hold, target = home_steer + yaw_error
2. Cruise (SELECT toggle)       → pure MQTT/vision steer
3. Keyboard base (L/U/W/S/X)   → lock/unlock/forward/backward/stop
4. Gamepad base (axis Y, LB, RB, A=stop)
5. Keyboard steer (C/A/D)      → center/left_step/right_step
6. Gamepad steer (right stick X)
7. Remote steer (Y toggle ON)  → MQTT angle from vision
8. Idle                         → center or release PWM
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

## IMU (MPU6050) Details

- I2C addr 0x68, gyro Z integration for relative yaw
- Calibrate at startup: N samples, average gyro_z → bias
- Recalibrate on each IMU-mode activation (200 samples, ~1s)
- `set_home()`: lock yaw + current steer_angle as reference
- P-controller: `target = home_steer + yaw_error` (1:1 correction, clamped to limits)
- State labels: LEFT (drift left, error<0), RIGHT (drift right, error>0), STRAIGHT (|error|≤1°)
- Fallback: if IMU not detected → logs warning, bridge works normally (MQTT-only)

## Gamepad Mapping (Xbox 360)

| Button | Code | Action |
|--------|------|--------|
| A | BTN_SOUTH | Stop base |
| B | BTN_EAST | Toggle IMU heading-hold mode |
| X | BTN_WEST | Center angle -1 |
| Y | BTN_NORTH | Toggle remote-only steer (MQTT steer, local drive) |
| LB | BTN_TL | Lock base |
| RB | BTN_TR | Unlock base |
| START | BTN_START | Quit |
| SELECT/BACK | BTN_SELECT | Toggle auto-cruise (30s forward + MQTT steer) |
| Right stick X | ABS_RX | Steering |
| Left stick Y | ABS_Y | Drive (forward/backward) |
| D-pad up/down | ABS_HAT0Y | Adjust center angle ±1 |

## Keyboard Mapping

| Key | Action |
|-----|--------|
| W | Forward |
| S | Backward |
| A | Steer left |
| D | Steer right |
| C | Center steering |
| X | Stop |
| L | Lock |
| U | Unlock |
| 1/2 | Adjust center angle ±1 |
| ENTER | Toggle cruise |
| Q | Quit |

## Steering Direction Convention (all paths synced)

```
More negative angle → STEER LEFT
More positive angle → STEER RIGHT
```

- Gamepad stick left → more right (negative axis inverted via formula)
- Gamepad stick right → more left (positive axis inverted via formula)
- Keyboard A → left_step (+STEP) → steer left
- Keyboard D → right_step (-STEP) → steer right
- MQTT vision angle < 90° → more negative → steer left
- MQTT vision angle > 90° → more positive → steer right
- IMU drift right (error>0) → target = home_steer + error → more positive → steer right
  (IMU yaw sign may need per-car tuning — the convention above is the default)

All paths gate through `apply_steering()` → clamp to [LEFT_LIMIT, RIGHT_LIMIT] → `_write_servo()` → `gpio.set_servo_pulsewidth(pin, pulse_us)`

## Deploy Notes

- `<model>` in config files provides model hints — use them as-is
- deploy_production.sh uses project-name-based compose (-p) to avoid stacking containers per version
- RPi container uses `network_mode: host` and `privileged: true` — needs GPIO + /dev/input + pigpiod on host
- Legacy timestamp-based compose projects auto-cleaned up on first deploy with new project names
- mDNS (.local) hostnames don't resolve inside Docker containers — use IP addresses for MQTT_BROKER_HOST
- If host pigpiod runs on port 8888, the container's pigpiod start fails (port in use) — this is OK, bridge connects to host pigpiod

## Code Notes

- RPi bridge uses module-level global state (`config.py`) accessed via `import config` + `config.attr` pattern — no circular imports
- All servo control is direct pigpio, no gpiozero caching/latency
- Vision preview (`cv2.imshow`) is headless-safe: try/except on cv2.error with auto-disable
- Mosquitto broker runs on MiniPC (docker-compose.vision.yml) on host network, port 1883
