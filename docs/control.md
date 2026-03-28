# Control Module

The `control` package provides the PID controllers that convert heading information from the vision pipeline into hardware commands.

Both controllers share the single `models.robot_state.RobotState` instance:

| Class | File | Output | FSM states used |
|-------|------|--------|-----------------|
| `HeadingController` | `control/heading_controller.py` | Motor command (float) | `SEARCHING`, `LOCKED`, `GAPPING` |
| `ServoPID` | `control/servo_pid.py` | Servo angle (degrees) | `SEARCHING`, `LOCKED`, `GAPPING` |

---

## `HeadingController` (`control/heading_controller.py`)

### Overview

`HeadingController` applies a PID algorithm to a **heading error** value and returns a **motor command**.  
It includes:

- **Hysteresis filter** — prevents jitter near zero error.
- **Sparse-signal support** — re-applies the last valid command when vision is lost.
- **Integral wind-up guard** — resets the integral on FSM re-entry from `GAPPING`.

### FSM States (from `models.robot_state`)

| State | Description |
|-------|-------------|
| `SEARCHING` | Initial state; no valid detection yet. |
| `LOCKED` | Vision active; PID running. |
| `GAPPING` | Vision lost; last valid command held. |

### Constructor

```python
HeadingController(state: RobotState)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `state` | `models.robot_state.RobotState` | Shared mutable state (PID constants, FSM, integral accumulators). |

### Public Method

#### `update(heading_error: Optional[float]) → float`

Compute and return the motor command for the current control cycle.

```python
controller = HeadingController(state)
command = controller.update(7.3)   # heading error in degrees
command = controller.update(None)  # vision lost → GAPPING, hold last command
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `heading_error` | `Optional[float]` | Heading error from the vision module in degrees, or `None` if no lines were detected. |

**Returns:** `float` — motor command derived from PID output (or `state.last_valid_command` during `GAPPING`).

### Hysteresis Logic

The controller uses two thresholds to avoid oscillation near zero:

| Threshold | Value | Meaning |
|-----------|-------|---------|
| `_HYSTERESIS_HIGH` | `5.0°` | Correction **activates** when `|e| > high`. |
| `_HYSTERESIS_LOW` | `3.0°` | Correction **deactivates** when `|e| < low`. |

These defaults are env-configurable via `CTRL_HYSTERESIS_HIGH` and `CTRL_HYSTERESIS_LOW`.

When `|e|` is between 3° and 5°, the previous correction state is maintained.

### Behaviour When Vision Is Lost

When `heading_error` is `None`:

1. The FSM transitions to `GAPPING`.
2. `state.last_valid_command` is returned unchanged.

When a valid signal is restored:

1. The integral term is reset to zero (prevents windup accumulated during the blind period).
2. The FSM transitions to `LOCKED`.

### PID Computation

```
P = Kp × e
I = Ki × Σ(e × dt)   [accumulated in state.pid_integral]
D = Kd × (e - e_prev) / dt

output = P + I + D
```

PID gains are read from `state.pid` (`PIDConstants`) and can be changed at runtime by modifying the dataclass fields.

PID defaults are env-configurable via `PID_KP`, `PID_KI`, and `PID_KD` in `.env`.

---

## `ServoPID` (`control/servo_pid.py`)

### Overview

`ServoPID` converts the raw tile-gap angle θ from the vision pipeline into a **servo angle command**.  
It includes:

- **Anti-windup** — clamps the integral accumulator to prevent the integral term from exceeding the steering clamp range.
- **Hold logic** — during vision loss (`GAPPING` state), the last valid servo angle is returned.
- **Integral reset** — on re-entry from `GAPPING`, the integral is zeroed.

### FSM States (from `models.robot_state`)

| State | Description |
|-------|-------------|
| `SEARCHING` | No valid tile-gap detected. |
| `LOCKED` | Vision active; PID computing servo commands. |
| `GAPPING` | Vision lost; servo angle held at `state.last_valid_servo_angle`. |

### Constructor

```python
ServoPID(state: RobotState)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `state` | `models.robot_state.RobotState` | Shared mutable state (PID constants, FSM, servo angles, steering clamp). |

### Public Method

#### `update(theta: Optional[float]) → float`

Compute and return the servo angle command for the current control cycle.

```python
pid = ServoPID(state)
servo_angle = pid.update(87.5)   # θ from vision
servo_angle = pid.update(None)   # vision lost → hold last angle
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `theta` | `Optional[float]` | Tile-gap angle from the vision module (degrees, relative to x-axis), or `None` if no line was detected. |

**Returns:** `float` — servo angle command in degrees.

### Servo Angle Calculation

```
error           = θ − 90°
max_offset      = state.max_steering_offset          # default 30°
steering_offset = clamp(PID(error), −max_offset, +max_offset)
servo_angle     = state.servo_center_angle + steering_offset
```

The steering offset is clamped to `±state.max_steering_offset` (default **30°**).  
The clamp is configured in `RobotState`, not as a module constant, so it can be changed at runtime:

```python
state.max_steering_offset = 45.0   # widen steering range
```

### Anti-Windup

When `ki ≠ 0`, the integral accumulator is clamped so the integral contribution (`ki × integral`) never exceeds the steering clamp range:

```
max_integral = state.max_steering_offset / |ki|
pid_integral = clamp(pid_integral, −max_integral, +max_integral)
```

### Behaviour When Vision Is Lost

When `theta` is `None`:

1. If the current state is `LOCKED`, the FSM transitions to `GAPPING`.
2. `state.last_valid_servo_angle` is returned.

When a valid signal is restored from `GAPPING`:

1. The integral term is reset (`state.reset_pid_integral()`).
2. The FSM transitions to `LOCKED`.
