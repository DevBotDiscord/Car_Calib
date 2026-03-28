# Models Module

The `models` package defines the **shared mutable state** and **Finite State Machine (FSM)** used across the vision, control, and driver layers.

All state lives in a single canonical file:

| File | `FSMState` values | Used by |
|------|-------------------|---------|
| `models/robot_state.py` | `SEARCHING`, `LOCKED`, `GAPPING` | `control.ServoPID`, `control.HeadingController`, `vision.LineDetector`, `main.py` |

---

## `models/robot_state.py`

### `FSMState` (enum)

```python
from models.robot_state import FSMState
```

| Member | Value | Description |
|--------|-------|-------------|
| `SEARCHING` | `auto()` | No valid tile-gap line detected; robot is looking for a reference. |
| `LOCKED` | `auto()` | Vision is active; robot is tracking a detected tile-gap line. |
| `GAPPING` | `auto()` | Vision lost; robot coasts on the last known servo angle (~2 s gap). |

### `PIDConstants` (dataclass)

```python
from models.robot_state import PIDConstants
pid = PIDConstants(kp=1.0, ki=0.05, kd=0.1)
```

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `kp` | `float` | `1.0` | Proportional gain. |
| `ki` | `float` | `0.05` | Integral gain. |
| `kd` | `float` | `0.1` | Derivative gain. |

### `RobotState` (dataclass)

`RobotState` is the single source of truth shared across all MVC layers.

```python
from models.robot_state import RobotState
state = RobotState()
```

#### Servo configuration

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `servo_center_angle` | `float` | `90.0` | Neutral servo angle in degrees. |
| `max_steering_offset` | `float` | `30.0` | Maximum allowed steering deviation from centre (degrees). Used as the PID output clamp in `ServoPID`. |

#### Vision / ROI parameters

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `roi_height_pct` | `float` | `0.6` | Height of the trapezoidal ROI as a fraction of frame height (bottom portion). |
| `roi_top_width_pct` | `float` | `0.75` | Width of the top edge of the trapezoid as a fraction of frame width. |
| `roi_bottom_width_pct` | `float` | `1.0` | Width of the bottom edge of the trapezoid as a fraction of frame width. |
| `debug_mode` | `bool` | `False` | When `True`, `LineDetector` saves `debug_mask.jpg` once on first call to verify the trapezoid ROI. |

#### Hold-state and history

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `last_valid_servo_angle` | `float` | `90.0` | Most recent servo angle issued while `LOCKED`; returned by `ServoPID` during `GAPPING`. |
| `last_valid_command` | `float` | `0.0` | Most recent motor-command output; returned by `HeadingController` during `GAPPING`. |

#### PID accumulators

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `pid` | `PIDConstants` | `PIDConstants()` | PID gain constants. |
| `fsm_state` | `FSMState` | `FSMState.SEARCHING` | Current FSM state. |
| `pid_integral` | `float` | `0.0` | Accumulated integral term. |
| `pid_last_error` | `float` | `0.0` | Previous error for derivative calculation. |

#### Methods

##### `transition_to(new_state: FSMState) → None`

Transition the FSM to `new_state`.  
Logs the transition at `INFO` level only when the state actually changes.

```python
state.transition_to(FSMState.LOCKED)
state.transition_to(FSMState.LOCKED)  # no-op – already LOCKED, nothing logged
state.transition_to(FSMState.GAPPING) # logs: "FSM transition: LOCKED -> GAPPING"
```

##### `reset_pid_integral() → None`

Reset `pid_integral` to `0.0`.  
Called by controllers when re-entering `LOCKED` from `GAPPING` to prevent integral windup accumulated during the blind period.

```python
state.reset_pid_integral()
```

---

## Adjusting Parameters at Runtime

All fields on `RobotState` are mutable and take effect immediately:

```python
state = RobotState()

# Tune PID gains
state.pid.kp = 1.5
state.pid.ki = 0.02
state.pid.kd = 0.2

# Widen the steering clamp
state.max_steering_offset = 45.0

# Enable debug mask output
state.debug_mode = True
```

Default values for `RobotState` are loaded from `.env` through `config/settings.py`.
Use `.env.example` for the full parameter list.

---

## FSM Transition Diagram

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
