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
├── requirements.txt          # Python dependencies
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
    └── models.md
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

The process will:
1. Open the camera at index `0`.
2. Start the 30 Hz heading-hold control loop.
3. Log each cycle's timestamp, detected angle θ, FSM state, and PID values to the console.
4. Append the same telemetry to `run_log.csv` in the working directory.
5. Centre the servo and release the camera on exit (`Ctrl+C`) or on a fatal error.

### Changing the camera index

Edit `_CAMERA_INDEX` at the top of `main.py`:

```python
_CAMERA_INDEX: int = 1   # use camera device /dev/video1
```

### CSV telemetry log

Every control cycle appends one row to `run_log.csv`:

| Column | Description |
|--------|-------------|
| `timestamp` | `time.monotonic()` value at loop start |
| `fsm_state` | FSM state name (`SEARCHING`, `LOCKED`, or `GAPPING`) |
| `theta` | Detected tile-gap angle θ (degrees), empty when `None` |
| `servo_angle` | Servo angle command sent to hardware (degrees) |
| `pid_integral` | Current accumulated integral term |
| `pid_last_error` | Error value from the previous cycle |

---

## Configuration

All tunable parameters are fields on `RobotState` or module-level constants in each source file.

### `RobotState` fields (`models/robot_state.py`)

| Field | Default | Description |
|-------|---------|-------------|
| `pid.kp` | `1.0` | Proportional gain |
| `pid.ki` | `0.05` | Integral gain |
| `pid.kd` | `0.1` | Derivative gain |
| `servo_center_angle` | `90.0°` | Neutral servo angle |
| `max_steering_offset` | `30.0°` | Maximum steering deviation from centre |
| `roi_height_pct` | `0.4` | Fraction of frame height used for the trapezoidal ROI |
| `roi_top_width_pct` | `0.6` | Top-edge width of the trapezoid as a fraction of frame width |
| `roi_bottom_width_pct` | `1.0` | Bottom-edge width of the trapezoid as a fraction of frame width |
| `debug_mode` | `False` | Save `debug_mask.jpg` once on first detection call |

### Vision pipeline constants (`vision/detector.py`)

| Constant | Default | Description |
|----------|---------|-------------|
| `_CLAHE_CLIP_LIMIT` | `2.0` | CLAHE contrast limit |
| `_CANNY_LOW` | `50` | Canny lower threshold |
| `_CANNY_HIGH` | `150` | Canny upper threshold |
| `_HOUGH_THRESHOLD` | `50` | Minimum Hough votes |
| `_HOUGH_MIN_LINE_LEN` | `30` px | Minimum accepted line length |
| `_ANGLE_THRESHOLD` | `3.0°` | Max angle difference to merge segments |
| `_SANITY_MAX_DELTA` | `20.0°` | Max inter-frame angle jump before rejection |

### Servo hardware constants (`drivers/servo_driver.py`)

| Constant | Default | Description |
|----------|---------|-------------|
| `_PULSE_MIN_US` | `1000` µs | Pulse width at 0° |
| `_PULSE_MAX_US` | `2000` µs | Pulse width at 180° |

---

## Module Documentation

Detailed documentation for each module is in the [`docs/`](docs/) directory:

| Module | File | Documentation |
|--------|------|---------------|
| Vision | `vision/` | [docs/vision.md](docs/vision.md) |
| Control | `control/` | [docs/control.md](docs/control.md) |
| Drivers | `drivers/` | [docs/drivers.md](docs/drivers.md) |
| Models | `models/` | [docs/models.md](docs/models.md) |

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
