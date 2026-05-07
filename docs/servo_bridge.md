# Servo Bridge Flow (MQTT)

Vision streams steering angles over MQTT. Raspberry Pi bridge subscribes,
drives servo + base via direct pigpio. Keyboard/gamepad stay local with
manual override priority.

## Components

- `main.py` on MiniPC: publishes steering to MQTT topic `car/servo/angle`
- `scripts/rpi_mqtt_bridge.py` on Raspberry Pi: subscribes, drives servo + base
- `scripts/servo_bridge_common.py`: shared angle helpers (clamp_angle, etc.)
- `scripts/input_device_helpers.py`: optional gamepad/keyboard detection

## Raspberry Pi side

### Docker (production)

```bash
docker compose -f docker-compose.rpi.yml up --build -d
```

### Bare metal (development)

```bash
sudo systemctl enable --now pigpiod
pip install -r requirements-rpi.txt
python scripts/rpi_mqtt_bridge.py
```

### Controls

- `W`: forward | `S`: backward | `X`: stop
- `A`: steer left | `D`: steer right | `C`: center
- `L`: lock | `U`: unlock
- `1`/`2`: adjust center angle
- `Y`: toggle remote-only steer (MQTT steer, local drive)
- `B`: toggle IMU heading-hold mode
- `SELECT`/`ENTER`: auto-cruise (30s forward + MQTT steer)
- `Q`: quit

## Vision side

Enabled via `.env`:

```bash
DRIVER_SERVO_MQTT_ENABLED=true
MQTT_BROKER_HOST=<rpi-or-broker-ip>
MQTT_BROKER_PORT=1883
```

## Remote input mapping

Vision sends angles in 0-180° space where 90° = straight.

Default mapping (`REMOTE_INPUT_*` env vars):
- 60° → full left
- 90° → center
- 120° → full right

If sender uses signed angles directly (within `[LEFT_LIMIT, RIGHT_LIMIT]`),
they pass through unchanged.

## Steer direction convention

```
More negative angle → steer LEFT
More positive angle → steer RIGHT
Range: LEFT_LIMIT .. RIGHT_LIMIT (default -71 .. +19, CENTER=-26)
```

All paths (gamepad, keyboard, MQTT, IMU) follow this single convention.
`apply_steering()` is the single gate → `_write_servo()` → `gpio.set_servo_pulsewidth()`.

## IMU heading-hold

Optional MPU6050. Toggle with `B` button on gamepad.

- Press B → quick gyro recalibrate → set_home(yaw=0, steer=current) → IMU steer active
- target = home_steer_angle + yaw_error (1:1 correction, clamped to limits)
- Press B again → back to normal steer
- IMU timeout/error → auto-disable, fallback to normal

Env config: `IMU_ENABLED`, `IMU_KP`, `IMU_STRAIGHT_THRESHOLD_DEG`.

## Notes

- `SERVO_PIN` defaults to 12
- Default base pins: OUT1=17, OUT2=27, OUT3=22
- Base output through direct `pigpio` (pid drive via container, host pigpiod)
- Servo via `gpio.set_servo_pulsewidth()` — no gpiozero, no caching
- Servo release (PWM off) on idle when `SERVO_RELEASE_IDLE=true`
