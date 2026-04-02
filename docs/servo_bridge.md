# Servo Bridge for Raspberry Pi

This setup splits keyboard steering and Raspberry Pi servo control into two
separate scripts:

- `scripts/servo_bridge_sender.py`: reads keyboard input and sends steering
  commands over TCP.
- `scripts/rpi_servo_bridge.py`: receives commands on the Raspberry Pi and
  drives the servo on GPIO `19`.

## Controls

- `A`: steer left
- `D`: steer right
- `C`: center steering
- `1`: increase center angle
- `2`: decrease center angle
- `Q`: quit

## Receiver on Raspberry Pi

Install dependencies:

```bash
pip install gpiozero
```

Run:

```bash
python scripts/rpi_servo_bridge.py
```

## Sender on keyboard machine

Install dependencies:

```bash
pip install evdev
```

Set the Raspberry Pi IP with `SERVO_BRIDGE_HOST`, then run:

```bash
export SERVO_BRIDGE_HOST=192.168.1.50
python scripts/servo_bridge_sender.py
```

## Notes

- The original servo pin `12` was changed to GPIO `19`.
- You can override `SERVO_PIN`, `SERVO_CENTER_ANGLE`, `SERVO_BRIDGE_HOST`, and
  `SERVO_BRIDGE_PORT` by environment variable without editing the scripts.
- The receiver will re-center the servo if no command is received for about
  one second.
- The sender keeps the original center-angle trim behaviour from `KEY_1` and
  `KEY_2`.
