# UOG AIS Autobot Calibration

An autonomous robot **heading-hold** system for the **NVIDIA Jetson Nano**.  
The robot uses a downward-facing camera to detect floor tile-gap lines and applies a PID control loop to keep itself straight at **30 Hz**.

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Repository Structure](#repository-structure)
3. [System Architecture](#system-architecture)
4. [Hardware Requirements](#hardware-requirements)
5. [Software Requirements](#software-requirements)
6. [Installation](#installation)
7. [Running the System](#running-the-system)
8. [Configuration](#configuration)
9. [Module Documentation](#module-documentation)
10. [Running Tests](#running-tests)
11. [License](#license)

---

## Project Overview

`UOG_AIS_AUTOBOT_CALIBRATION` implements a closed-loop **heading stabilisation** system:

1. A camera captures frames at 30 Hz.
2. The [`vision`](docs/vision.md) module detects floor tile-gap lines and measures the angle θ relative to the horizontal axis.
3. The [`control`](docs/control.md) module runs a PID controller that converts the heading error `e = θ − 90°` into a servo steering command.
4. The [`drivers`](docs/drivers.md) module converts the servo angle command into PWM signals sent to the hardware.
5. The [`models`](docs/models.md) module holds shared state, PID constants, and the Finite State Machine (FSM) that governs transitions between operational modes.

---

## Repository Structure

```
UOG_AIS_AUTOBOT_CALIBRATION/
├── main.py                   # Entry point – 30 Hz control loop + CSV logging
├── process_video.py          # Offline video pipeline with overlays/debug panels
├── requirements.txt          # Python dependencies
├── config/                   # Centralized environment-backed settings
│   ├── __init__.py
│   └── settings.py
├── runtime/                  # Shared runtime helper utilities
│   ├── __init__.py
│   ├── https_stream.py       # HTTPS MJPEG stream + self-signed cert helpers
│   └── video_runtime_helpers.py
├── scripts/                  # Utility scripts (visualization/post-processing)
│   └── visualize_pid_simulation_standalone.py
│
├── control/                  # PID heading controllers
│   ├── __init__.py
│   ├── heading_controller.py # HeadingController (motor command output)
│   └── servo_pid.py          # ServoPID (servo angle output)
│
├── drivers/                  # Hardware abstraction layer
│   ├── __init__.py
│   ├── motors.py             # MotorDriver – differential PWM
│   └── servo_driver.py       # ServoDriver – servo angle → PWM
│
├── models/                   # Shared state and FSM definitions
│   ├── __init__.py
│   └── robot_state.py        # RobotState – single source of truth for all MVC layers
│
├── vision/                   # Computer-vision pipeline
│   ├── __init__.py
│   └── detector.py           # LineDetector – trapezoid ROI, grouping, tile-gap angle
│
├── tests/                    # Unit tests (pytest)
│   ├── test_detector.py
│   ├── test_heading_controller.py
│   ├── test_line_processor.py
│   ├── test_motors.py
│   ├── test_robot_state.py
│   ├── test_servo_driver.py
│   └── test_servo_pid.py
│
└── docs/                     # Per-module documentation
    ├── vision.md
    ├── control.md
    ├── drivers.md
    ├── models.md
    └── LOG_VISUALIZATION_GUIDE.md
```

---

## System Architecture

```
Camera
  │  (BGR frame)
  ▼
vision.LineDetector.get_reference_angle()
  │  θ (degrees) or None
  ▼
control.ServoPID.update()
  │  servo_angle (degrees)
  ▼
drivers.ServoDriver.send_angle()
  │  PWM pulse (µs)
  ▼
Servo Hardware
```

### Finite State Machine

All FSM states live in `models.robot_state`:

| State        | Description                                                        |
|--------------|--------------------------------------------------------------------|
| `SEARCHING`  | No valid tile-gap detected; waiting for a reference line.          |
| `LOCKED`     | Vision active; robot tracks the detected tile-gap angle.           |
| `GAPPING`    | Vision lost; robot holds the last known servo angle (~2 s gap).    |

**FSM transition diagram:**

```
           ┌──────────────────────────────────────────┐
           │  (vision returns None while in SEARCHING) │
           ▼                                           │
       SEARCHING ──── vision detected ────► LOCKED ───┘
           ▲                                  │
           │                                  │ vision lost
           │                                  ▼
           └────── vision restored ────── GAPPING
```

---

## Hardware Requirements

| Component | Specification |
|-----------|---------------|
| SBC | NVIDIA Jetson Nano (2 GB or 4 GB) |
| Camera | CSI or USB camera (index 0 by default) |
| Servo | Standard hobby servo (50 Hz, 1000–2000 µs pulse range) |
| Servo interface | PCA9685 I²C PWM board **or** Jetson Nano GPIO PWM pin |
| Motors (optional) | Differential-drive motors with H-bridge driver |

---

## Software Requirements

- Python 3.8+
- OpenCV ≥ 4.5 (`opencv-python-headless`)
- NumPy ≥ 1.21
- python-dotenv ≥ 1.0

See [`requirements.txt`](requirements.txt) for the full list.

---

## Installation

```bash
# 1. Clone the repository
git clone https://github.com/NhatNam041206/UOG_AIS_AUTOBOT_CALIBRATION.git
cd UOG_AIS_AUTOBOT_CALIBRATION

# 2. (Optional) Create and activate a virtual environment
python3 -m venv .venv
source .venv/bin/activate

# 3. Install dependencies
pip install -r requirements.txt
```

> **Jetson Nano note:** OpenCV is often pre-installed on JetPack images.  
> Use `pip install opencv-python-headless` only when building from a minimal image.

---

## Running the System

```bash
python main.py
```

Realtime debug mode with overlays, detector panel, CSV enrichment, and optional video write:

```bash
python main.py --debug --show-guidance-overlay --show-detector-debug --write-debug-video
```

Enable HTTPS MJPEG stream for another device on LAN:

```bash
python main.py --debug --stream --public --host 0.0.0.0 --port 8443 --show-preview
```

If token is configured, append `?token=...` when opening stream endpoints.

Stream endpoints:

- `https://<device-a-ip>:8443/stream.mjpg`
- `https://<device-a-ip>:8443/snapshot.jpg`
- `https://<device-a-ip>:8443/status`

TLS certificate and key are auto-generated (self-signed) when missing. For LAN testing, a browser trust warning is expected on first access.

The process will:
1. Open the camera at index `0`.
2. Start the 30 Hz heading-hold control loop.
3. Log each cycle's timestamp, detected angle θ, FSM state, and PID values to the console.
4. Append the same telemetry to `run_log.csv` in the working directory.
5. Centre the servo and release the camera on exit (`Ctrl+C`) or on a fatal error.

### Offline video processing

```bash
python process_video.py videos/6.mp4 --show-guidance-overlay --show-detector-debug
```

Enable 180° frame flip when needed:

```bash
python process_video.py videos/6.mp4 --flip-frame
```

Add a small frame delay when reviewing output slowly:

```bash
python process_video.py videos/6.mp4 --sleep-ms 10
```

### Changing the camera index

Set it in `.env`:

```bash
MAIN_CAMERA_INDEX=1
```

### CSV telemetry log

Every control cycle appends one row to `run_log.csv`:

| Column | Description |
|--------|-------------|
| `mono_timestamp` | `time.monotonic()` value at loop start |
| `utc_timestamp` | Wall-clock UTC timestamp (`ISO-8601`) |
| `loop_ms` | Loop duration (ms) |
| `loop_overrun_ms` | Positive overrun above target period |
| `fsm_state` | FSM state name (`SEARCHING`, `LOCKED`, or `GAPPING`) |
| `calibration_active` | Calibration gate status (`0`/`1`) |
| `theta` | Detected tile-gap angle θ (degrees), empty when `None` |
| `theta_source` | `live`, `stale`, or `none` |
| `theta_horizontal` | Selected horizontal-group angle before conversion |
| `reference_group_index` | Chosen line-group index from detector clustering |
| `selected_group_bbox` | Bounding box of chosen group (`x,y,w,h`) |
| `lines_count`, `groups_count` | Detector raw/grouped counts |
| `servo_angle` | Servo angle command sent to hardware (degrees) |
| `servo_offset` | Servo angle offset from center |
| `pid_p_term`, `pid_i_term`, `pid_d_term` | PID component snapshots |
| `pid_integral` | Current accumulated integral term |
| `pid_last_error` | Error value from the previous cycle |
| `hardware_send_latency_ms` | Servo send latency estimate |

---

## Configuration

All tunable parameters are now loaded from environment variables through `config/settings.py`.

1. Copy [.env.example](.env.example) to `.env`.
2. Edit values in `.env`.
3. Restart the process.

`.env` is intentionally git-ignored. Commit changes to `.env.example` when adding or renaming parameters.

### Common Runtime Settings

| Variable | Default | Description |
|----------|---------|-------------|
| `MAIN_TARGET_HZ` | `30.0` | Main loop frequency in Hz. |
| `MAIN_CAMERA_INDEX` | `0` | Camera index for `main.py`. |
| `MAIN_FLIP_FRAME` | `false` | Flip camera frame by 180° in `main.py`. |
| `MAIN_CSV_LOG_FILE` | `run_log.csv` | CSV path for live mode logging. |
| `MAIN_DEBUG_MODE` | `false` | Enable debug visuals/telemetry path in `main.py`. |
| `MAIN_SHOW_PREVIEW` | `false` | Show local OpenCV preview window. |
| `MAIN_SHOW_DETECTOR_DEBUG` | `false` | Include detector panel under the main frame. |
| `MAIN_WRITE_DEBUG_VIDEO` | `false` | Enable annotated live video output. |
| `MAIN_DEBUG_VIDEO_OUTPUT` | `main_debug.mp4` | Base path for live debug video output. |
| `MAIN_CAMERA_RETRY_LIMIT` | `3` | Camera init retry count before abort. |
| `MAIN_VIDEO_RETRY_LIMIT` | `5` | Consecutive video-write failures before abort. |
| `MAIN_HARDWARE_RETRY_LIMIT` | `5` | Consecutive servo-send failures before abort. |
| `MAIN_HTTPS_STREAM_ENABLED` | `false` | Enable HTTPS MJPEG stream server. |
| `MAIN_HTTPS_STREAM_HOST` | `127.0.0.1` | Default stream bind host. |
| `MAIN_HTTPS_STREAM_PORT` | `8443` | HTTPS stream port. |
| `MAIN_HTTPS_CERT_FILE` | `certs/main_stream_cert.pem` | TLS certificate file path. |
| `MAIN_HTTPS_KEY_FILE` | `certs/main_stream_key.pem` | TLS private key path. |

### process_video Settings

| Variable | Default | Description |
|----------|---------|-------------|
| `PROCESS_VIDEO_CSV_OUTPUT` | `video_log.csv` | Default CSV output path. |
| `PROCESS_VIDEO_OUTPUT` | `processed_video.mp4` | Default annotated video path. |
| `PROCESS_VIDEO_SHOW_GUIDANCE_OVERLAY` | `false` | Enable guidance overlay by default. |
| `PROCESS_VIDEO_SHOW_DETECTOR_DEBUG` | `false` | Enable detector debug panel by default. |
| `PROCESS_VIDEO_FLIP_FRAME` | `false` | Flip input frame by 180° before processing. |
| `PROCESS_VIDEO_FRAME_SLEEP_MS` | `0.0` | Extra delay after each processed frame (ms). |
| `PROCESS_VIDEO_START_CALIB_THRESHOLD_DEG` | `5.0` | Outer accepted range around 90°. |
| `PROCESS_VIDEO_STOP_CALIB_THRESHOLD_DEG` | `3.0` | Inner stop-calibrating range around 90°. |

### Vision and Control Settings

| Variable | Default | Description |
|----------|---------|-------------|
| `PID_KP`, `PID_KI`, `PID_KD` | `1.0`, `0.05`, `0.1` | PID gains. |
| `SERVO_CENTER_ANGLE` | `90.0` | Servo neutral position. |
| `MAX_STEERING_OFFSET` | `30.0` | Maximum steering correction. |
| `ROI_HEIGHT_PCT`, `ROI_TOP_WIDTH_PCT`, `ROI_BOTTOM_WIDTH_PCT` | `0.6`, `0.75`, `1.0` | ROI shape parameters. |
| `VISION_CLUSTER_ANGLE_BIAS_DEG` | `4.0` | Max normal-space angle gap for segment clustering. |
| `VISION_CLUSTER_RHO_BIAS_PX` | `25.0` | Max normal-space rho gap (px) for segment clustering. |
| `VISION_MIN_GROUP_TOTAL_LENGTH_PX` | `120.0` | Min total segment length to keep a horizontal candidate group. |
| `VISION_HORIZONTAL_MAX_ERROR_DEG` | `20.0` | Reject selected groups that are not horizontal enough. |
| `VISION_SANITY_MAX_DELTA_DEG` | `40.0` | Max inter-frame angle jump before rejection. |
| `CTRL_RELOCK_VALID_FRAMES` | `3` | Valid-frame debounce count before re-entering `LOCKED` from `GAPPING`. |

See [.env.example](.env.example) for the complete parameter list.

---

## Module Documentation

Detailed documentation for each module is in the [`docs/`](docs/) directory:

| Module | File | Documentation |
|--------|------|---------------|
| Vision | `vision/` | [docs/vision.md](docs/vision.md) |
| Control | `control/` | [docs/control.md](docs/control.md) |
| Drivers | `drivers/` | [docs/drivers.md](docs/drivers.md) |
| Models | `models/` | [docs/models.md](docs/models.md) |
| Logs and plots | `scripts/` | [docs/LOG_VISUALIZATION_GUIDE.md](docs/LOG_VISUALIZATION_GUIDE.md) |

---

## Running Tests

```bash
# Install pytest if not already installed
pip install pytest

# Run all tests
pytest tests/

# Run a specific test file
pytest tests/test_servo_pid.py -v
```

---

## License

This project is licensed under the terms in [LICENSE](LICENSE).
