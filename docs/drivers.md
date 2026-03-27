# Drivers Module

The `drivers` package provides the **hardware abstraction layer** between the control algorithms and the physical actuators.

| Class | File | Purpose |
|-------|------|---------|
| `MotorDriver` | `drivers/motors.py` | Converts PID output into differential PWM for left/right motors. |
| `ServoDriver` | `drivers/servo_driver.py` | Converts angle commands into PWM pulse widths for a servo motor. |

Both classes implement **stub** `_write_*` methods that must be overridden in a platform-specific subclass when deploying to real hardware (Jetson Nano GPIO, PCA9685, etc.).

---

## `MotorDriver` (`drivers/motors.py`)

### Overview

`MotorDriver` maps a signed PID output value to a **(left, right) PWM pair** that produces a gentle differential turn while maintaining forward motion.

**Mapping:**

```
pwm_left  = clamp(pwm_centre + pid_output, pwm_min, pwm_max)
pwm_right = clamp(pwm_centre − pid_output, pwm_min, pwm_max)
```

A positive PID output steers **right** (left motor faster); a negative value steers **left**.

### Constructor

```python
MotorDriver(
    pwm_centre: int = 128,
    pwm_min: int = 0,
    pwm_max: int = 255,
)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pwm_centre` | `int` | `128` | Baseline PWM for straight-ahead motion (mid of 0–255 range). |
| `pwm_min` | `int` | `0` | Minimum allowable PWM output. |
| `pwm_max` | `int` | `255` | Maximum allowable PWM output. |

### Public Methods

#### `set_pwm(pid_output: float) → tuple[int, int]`

Convert a PID output into a (left, right) PWM pair and apply it to the hardware.

```python
driver = MotorDriver()
left, right = driver.set_pwm(20.0)   # steer right slightly
left, right = driver.set_pwm(-20.0)  # steer left slightly
left, right = driver.set_pwm(0.0)    # straight ahead (128, 128)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `pid_output` | `float` | Signed PID controller output. Positive → steer right; negative → steer left. |

**Returns:** `tuple[int, int]` — `(pwm_left, pwm_right)` clamped integer PWM values.

**Raises:** `OSError` — if the underlying hardware interface fails (raised by subclass implementation of `_write_pwm`).

#### `stop() → None`

Issue an **emergency stop** by setting both motor PWM values to `0`.

```python
driver.stop()
```

### Extending for Real Hardware

Override `_write_pwm` in a subclass to send values to the actual hardware:

```python
from drivers.motors import MotorDriver
import Jetson.GPIO as GPIO

class JetsonMotorDriver(MotorDriver):
    def _write_pwm(self, pwm_left: int, pwm_right: int) -> None:
        GPIO.output(LEFT_PIN, pwm_left)
        GPIO.output(RIGHT_PIN, pwm_right)
```

---

## `ServoDriver` (`drivers/servo_driver.py`)

### Overview

`ServoDriver` translates a **degree value** into a **PWM pulse width (µs)** using linear interpolation, then sends the pulse to the servo hardware.

**Angle → pulse mapping:**

```
pulse_us = pulse_min + (angle / 180) × (pulse_max − pulse_min)
```

The result is clamped to `[0°, 180°]` before conversion.

### Constructor

```python
ServoDriver(
    channel: int = 0,
    center_angle: float = 90.0,
    pulse_min_us: int = 1000,
    pulse_max_us: int = 2000,
)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `channel` | `int` | `0` | PCA9685 channel (0–15) or Jetson Nano PWM pin number. |
| `center_angle` | `float` | `90.0°` | Neutral servo angle used by `center()`. |
| `pulse_min_us` | `int` | `1000` µs | Pulse width corresponding to 0°. |
| `pulse_max_us` | `int` | `2000` µs | Pulse width corresponding to 180°. |

### Public Methods

#### `send_angle(angle: float) → None`

Send a target angle to the servo hardware.

```python
driver = ServoDriver(channel=0)
driver.send_angle(90.0)    # centre – 1500 µs pulse
driver.send_angle(120.0)   # 30° right – 1667 µs pulse
driver.send_angle(60.0)    # 30° left – 1333 µs pulse
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `angle` | `float` | Target servo angle in degrees. Values outside `[0°, 180°]` are clamped. |

#### `center() → None`

Return the servo to its neutral (`center_angle`) position.  
Called during safe-state initialisation and emergency-stop procedures.

```python
driver.center()   # returns to 90° by default
```

### Extending for Real Hardware

Override `_write_angle` in a subclass to send the pulse to the actual hardware.

**PCA9685 example:**

```python
from drivers.servo_driver import ServoDriver
import board, busio
from adafruit_pca9685 import PCA9685

class PCA9685ServoDriver(ServoDriver):
    def __init__(self, channel: int = 0):
        super().__init__(channel=channel)
        i2c = busio.I2C(board.SCL, board.SDA)
        self._pca = PCA9685(i2c)
        self._pca.frequency = 50

    def _write_angle(self, angle: float, pulse_us: int) -> None:
        # 16-bit duty cycle for a 50 Hz (20 000 µs) period
        duty = int(pulse_us / 20_000 * 65_535)
        self._pca.channels[self._channel].duty_cycle = duty
```

**Jetson Nano GPIO example:**

```python
from drivers.servo_driver import ServoDriver
import Jetson.GPIO as GPIO

class JetsonServoDriver(ServoDriver):
    def __init__(self, pin: int):
        super().__init__(channel=pin)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(pin, GPIO.OUT)
        self._pwm = GPIO.PWM(pin, 50)
        self._pwm.start(0)

    def _write_angle(self, angle: float, pulse_us: int) -> None:
        duty = pulse_us / 20_000 * 100   # duty cycle in percent
        self._pwm.ChangeDutyCycle(duty)
```

### Pulse Width Reference

| Angle | Pulse Width |
|-------|-------------|
| 0° | 1000 µs |
| 45° | 1250 µs |
| 90° (centre) | 1500 µs |
| 135° | 1750 µs |
| 180° | 2000 µs |
