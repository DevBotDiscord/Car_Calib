# Servo Bridge Flow

The current flow keeps the keyboard plugged into the Raspberry Pi while the
vision controller sends servo angles down over TCP.

## Components

- `main.py` or `process_video.py` on the vision side:
  uses `drivers.servo_driver.ServoDriver` to send servo angles.
- `scripts/rpi_servo_bridge.py` on the Raspberry Pi:
  keeps the original keyboard and motor controls local, listens for remote
  servo angles, and applies them only when the user is not steering manually.

## Raspberry Pi side

Install dependencies:

```bash
pip install gpiozero evdev RPi.GPIO
```

Run:

```bash
SERVO_PIN=19 SERVO_BRIDGE_PORT=8765 python scripts/rpi_servo_bridge.py
```

Default remote input mapping expects the vision side to send angles around
`90` with a range of `60..120`. If your sender already uses the same angle
space as the keyboard script (`-65 .. -8 .. 60`), set:

```bash
REMOTE_INPUT_MIN_ANGLE=-65
REMOTE_INPUT_CENTER_ANGLE=-8
REMOTE_INPUT_MAX_ANGLE=60
```

Keyboard controls remain active:

- `W`: forward
- `S`: backward
- `A`: steer left
- `D`: steer right
- `C`: center steering
- `X`: stop
- `L`: lock
- `U`: unlock
- `1`: center angle +1
- `2`: center angle -1
- `Q`: quit

When `A`, `D`, or `C` is pressed, manual steering temporarily overrides the
remote angle stream. When the user stops steering, remote servo control
resumes automatically.

## Vision side

Enable bridge mode in `.env` or environment variables:

```bash
DRIVER_SERVO_BRIDGE_ENABLED=true
DRIVER_SERVO_BRIDGE_HOST=192.168.1.50
DRIVER_SERVO_BRIDGE_PORT=8765
DRIVER_SERVO_BRIDGE_MIN_SEND_INTERVAL_S=0.0
DRIVER_SERVO_BRIDGE_MIN_ANGLE_DELTA=0.0
```

If you want the vision side to send the same signed angles used by the
keyboard controller, also set:

```bash
SERVO_CENTER_ANGLE=-8
DRIVER_SERVO_ANGLE_MIN=-90
DRIVER_SERVO_ANGLE_MAX=90
```

Then run the existing application as usual:

```bash
python main.py
```

or

```bash
python process_video.py <video_path>
```

## Optional rate limiting

By default, bridge sending is now immediate. If you want to slow it down again,
set these manually:

```bash
DRIVER_SERVO_BRIDGE_MIN_SEND_INTERVAL_S=0.05
DRIVER_SERVO_BRIDGE_MIN_ANGLE_DELTA=1.0
```

That will send less often and ignore tiny angle changes.
