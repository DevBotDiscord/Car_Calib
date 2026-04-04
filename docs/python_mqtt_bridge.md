# Python MQTT Bridge

This flow keeps the keyboard connected to the Raspberry Pi while the
vision controller publishes steering angles over MQTT.

## Files

- `scripts/rpi_mqtt_bridge.py`: Raspberry Pi keyboard controller with MQTT servo subscribe
- `scripts/mqtt_servo_command.py`: test publisher for servo angle
- `drivers/servo_driver.py`: vision-side MQTT publisher

## Raspberry Pi Side

Install dependencies:

```bash
sudo apt install -y pigpio python3-pigpio python3-gpiozero
pip install evdev paho-mqtt python-dotenv
sudo systemctl enable --now pigpiod
```

Run:

```bash
MQTT_BROKER_HOST=192.168.1.50 SERVO_PIN=19 python scripts/rpi_mqtt_bridge.py
```

Optional controller tuning:

```bash
GAMEPAD_DEVICE=/dev/input/event5
GAMEPAD_STEER_DEADZONE=0.12
GAMEPAD_DRIVE_DEADZONE=0.20
INVERT_STEER_AXIS=false
INVERT_DRIVE_AXIS=false
```

If `pigpiod` is running on a different host, set:

```bash
PIGPIO_HOST=127.0.0.1
PIGPIO_PORT=8888
```

Keyboard controls remain local:

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

Controller controls are also supported when a gamepad is detected:

- right stick X: steering
- left stick Y: base forward/backward
- `A`: stop base
- `B`: center steering
- `LB`: lock
- `RB`: unlock
- `Y` / `X`: center angle `+1 / -1`
- D-pad up/down: center angle `+1 / -1`
- `START`: quit

When `A`, `D`, or `C` is pressed, local steering temporarily overrides the
MQTT angle stream. When the user stops steering, remote servo control
resumes automatically.

If the configured `KEYBOARD_DEVICE` does not exist, the bridge still starts
and remote MQTT steering continues to work; local keyboard control is simply
disabled for that session.

If a controller is present, it acts as another local manual override source.
When the stick/buttons return to neutral, remote MQTT steering resumes after
the normal `MANUAL_STEER_HOLD` window.

By default, the bridge now holds the last MQTT steering angle until a new
message arrives:

```bash
REMOTE_SERVO_HOLD_LAST=true
```

If you want the old timeout behavior back, set:

```bash
REMOTE_SERVO_HOLD_LAST=false
REMOTE_SERVO_TIMEOUT=0.6
```

## Accepted Servo Payload

The Python bridge accepts either:

- plain float payload, for example `90.0` or `-8.0`
- JSON payload, for example `{"type":"angle","angle":90}` or `{"type":"center"}`

If the payload already matches the local signed steering range
`-65 .. -8 .. 60`, it is used directly. Otherwise it is remapped from the
configured remote input range.

## Vision Side

Enable MQTT publishing in `.env`:

```bash
DRIVER_SERVO_MQTT_ENABLED=true
DRIVER_SERVO_BRIDGE_ENABLED=false
MQTT_BROKER_HOST=192.168.1.50
MQTT_BROKER_PORT=1883
MQTT_SERVO_TOPIC=car/servo/angle
```

If you want the vision side to publish the same signed steering angles used
on the Raspberry Pi, also set:

```bash
SERVO_CENTER_ANGLE=-8
DRIVER_SERVO_ANGLE_MIN=-90
DRIVER_SERVO_ANGLE_MAX=90
```

Then run the existing app as usual:

```bash
python main.py --debug
```

## Manual MQTT Test

Without running vision, you can publish a test angle directly:

```bash
python scripts/mqtt_servo_command.py -8
python scripts/mqtt_servo_command.py 90
```

## Notes

- `SERVO_PIN` defaults to `19`
- default base pins are `17`, `27`, `22`
- base output is driven through `pigpio`
- servo output now uses `gpiozero.AngularServo` with `PiGPIOFactory`
- gamepad auto-detect matches names like `edra`, `joystick`, `gamepad`, `controller`, `pad`
- servo MQTT angle is published as a retained message
- broker host and topic must match on both machines
