# Models Module

The `models` package defines the **shared mutable state** and **Finite State Machine (FSM)** used across the vision, control, and driver layers.

There are two parallel implementations that serve different controller paths:

| File | `FSMState` values | Used by |
|------|-------------------|---------|
| `models/robot_state.py` | `SEARCHING`, `LOCKED`, `GAPPING` | `control.ServoPID` / `main.py` |
| `models/state.py` | `IDLE`, `CALIBRATING`, `DEAD_RECKONING` | `control.HeadingController` |

Both files follow the same structure: an `FSMState` enum, a `PIDConstants` dataclass, and a `RobotState` dataclass.

---

## `models/robot_state.py` — Heading-Hold State

### `FSMState` (enum)

```python
from models.robot_state import FSMState
```

| Member | Value | Description |
|--------|-------|-------------|
| `SEARCHING` | `auto()` | No valid tile-gap line detected; robot is looking for a reference. |
| `LOCKED` | `auto()` | Vision is active; robot is tracking a detected tile-gap line. |
| `GAPPING` | `auto()` | Vision lost; robot coasts on the last known servo angle. |

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

```python
from models.robot_state import RobotState
state = RobotState()
```

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `pid` | `PIDConstants` | `PIDConstants()` | PID gain constants. |
| `servo_center_angle` | `float` | `90.0` | Neutral servo angle in degrees. |
| `last_valid_servo_angle` | `float` | `90.0` | Most recent servo angle issued while `LOCKED`; used as fallback during `GAPPING`. |
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
Called by the controller when re-entering from `GAPPING` to prevent integral windup accumulated during the blind period.

```python
state.reset_pid_integral()
```

---

## `models/state.py` — Motor-Command State

### `FSMState` (enum)

```python
from models.state import FSMState
```

| Member | Value | Description |
|--------|-------|-------------|
| `IDLE` | `auto()` | Robot stationary, awaiting a start command. |
| `CALIBRATING` | `auto()` | Robot actively detecting floor tiles and computing heading error. |
| `DEAD_RECKONING` | `auto()` | Vision signal lost; robot coasts on the last valid command. |

### `PIDConstants` (dataclass)

Identical structure to `models.robot_state.PIDConstants`.

```python
from models.state import PIDConstants
pid = PIDConstants(kp=1.0, ki=0.05, kd=0.1)
```

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `kp` | `float` | `1.0` | Proportional gain. |
| `ki` | `float` | `0.05` | Integral gain. |
| `kd` | `float` | `0.1` | Derivative gain. |

### `RobotState` (dataclass)

```python
from models.state import RobotState
state = RobotState()
```

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `heading_error` | `float` | `0.0` | Current heading error `e = \|θ − 90°\|` in degrees. |
| `pid` | `PIDConstants` | `PIDConstants()` | PID gain constants. |
| `last_valid_command` | `float` | `0.0` | Most recent non-`None` PID output sent to motors; used as fallback during `DEAD_RECKONING`. |
| `fsm_state` | `FSMState` | `FSMState.IDLE` | Current FSM state. |
| `pid_integral` | `float` | `0.0` | Accumulated integral term. |
| `pid_last_error` | `float` | `0.0` | Previous error for derivative calculation. |

#### Methods

##### `transition_to(new_state: FSMState) → None`

Transition the FSM to `new_state`.  
Logs the transition at `INFO` level only when the state actually changes.

```python
state.transition_to(FSMState.CALIBRATING)
state.transition_to(FSMState.DEAD_RECKONING)
```

##### `reset_pid_integral() → None`

Reset `pid_integral` to `0.0`.

```python
state.reset_pid_integral()
```

---

## Adjusting PID Gains at Runtime

Both `RobotState` variants store gains in a `PIDConstants` dataclass, so they can be updated without restarting:

```python
state = RobotState()
state.pid.kp = 1.5
state.pid.ki = 0.02
state.pid.kd = 0.2
```

---

## FSM Transition Diagrams

### Heading-Hold FSM (`models/robot_state.py`)

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

### Motor-Command FSM (`models/state.py`)

```
         IDLE ──── start ────► CALIBRATING
                                   │   ▲
                        vision lost│   │vision restored
                                   ▼   │
                             DEAD_RECKONING
```
