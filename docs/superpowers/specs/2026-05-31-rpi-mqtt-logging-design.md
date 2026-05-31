# RPi MQTT logging design

## Goal

Improve Raspberry Pi bridge logs for MQTT command handling. Logs must be compact, readable in Docker stdout, and useful when tailing live hardware behavior. Control behavior, MQTT topics, payload formats, GPIO output, and route/session semantics must not change.

## Selected approach

Use Python's standard `logging` module with compact one-line output.

Chosen scope:

- Add shared RPi logging setup helper.
- Migrate `scripts/rpi/mqtt_client.py` from `print()` to logging.
- Migrate key lifecycle/error prints in `scripts/rpi/bridge.py` to logging so startup/shutdown output matches MQTT logs.
- Keep remaining hardware/control behavior unchanged.

## Log format

Default stdout format:

```text
HH:MM:SS LEVEL [AREA][DIR][KIND] message key=value ...
```

Examples:

```text
10:22:11 INFO [MQTT][CONN] connected host=192.168.1.10 port=1883
10:22:12 INFO [MQTT][RX][BASE] TURN_LEFT state=1,0,0
10:22:13 INFO [MQTT][RX][SERVO] apply angle=-18.00 raw={"angle": -18}
10:22:14 WARNING [MQTT][RX][SERVO] ignored script_active=OFF raw={"angle": -12}
10:22:15 WARNING [MQTT][RX][BASE] unknown command=SPIN
```

`logging` level defaults to `INFO`. Optional env override may use `RPI_LOG_LEVEL` if added during implementation.

## Components

### `scripts/rpi/logging_utils.py`

Small helper module for RPi-side logging.

Responsibilities:

- Configure root or `car_calib.rpi` logger once.
- Attach stdout `StreamHandler`.
- Use compact formatter.
- Prevent duplicate handlers if setup runs more than once.
- Expose `setup_rpi_logging()` and optionally `get_logger(name)`.

This file must not write logs to disk.

### `scripts/rpi/mqtt_client.py`

Migrate MQTT-related `print()` calls to logger calls.

Logger name:

```python
car_calib.rpi.mqtt
```

Log groups:

- `[MQTT][CONN]` for connect/disconnect.
- `[MQTT][RX][SERVO]` for servo commands.
- `[MQTT][RX][BASE]` for base commands.
- `[MQTT][RX][RELAY]` for relay commands.
- `[MQTT][RX][SCRIPT]` for script-active commands.
- `[MQTT][TX][STATUS]`, `[MQTT][TX][MODE]`, `[MQTT][TX][ROUTE]` for outbound publishes.

Log valid command effects at `INFO`.

Log ignored or invalid input at `WARNING`:

- Servo payload ignored because `script_active=OFF`.
- Unknown base command.
- Unknown relay command.
- Unknown script-active payload.
- Unknown topic.
- Invalid UTF-8 or invalid payload values.

Log connection failure at `ERROR` or `WARNING` depending current style needed by implementation.

### `scripts/rpi/bridge.py`

Call logging setup early in `main()` before setup work emits messages.

Migrate lifecycle messages:

- Startup banner.
- Input device status.
- Input setup failure.
- Control loop errors.
- Exit/cleanup/done.

Keep log volume low. Do not log every 100 Hz loop iteration.

## Data flow

1. `bridge.py` calls `setup_rpi_logging()`.
2. `setup_mqtt()` starts MQTT client.
3. MQTT callbacks log inbound/outbound events through `car_calib.rpi.mqtt`.
4. Hardware functions still execute exactly as before.
5. Docker captures stdout, so logs remain visible through `docker logs` and deploy runtime logs.

## Behavior preservation

Implementation must not change:

- MQTT subscribe topics.
- MQTT publish topics.
- Payload parsing rules.
- `script_active` gate for servo commands.
- Base pin states:
  - `FORWARD` => `0,1,0`
  - `BACKWARD` => `0,0,1`
  - `STOP` => `0,0,0`
  - `LOCK` => `1,0,1`
  - `UNLOCK` => `1,1,0`
  - `TURN_LEFT`/`LEFT` => `1,0,0`
  - `TURN_RIGHT`/`RIGHT` => `0,1,1`
- Relay ON/OFF behavior.
- Servo angle resolution and clamping.
- Heartbeat cadence.

## Verification

Run static checks:

```bash
python -m compileall scripts/rpi
```

Manual or mocked runtime checks:

- Start RPi bridge or import callback handlers with mocked config/hardware.
- Publish or call handlers for:
  - valid base command
  - valid relay command
  - valid script-active command
  - valid servo command with `script_active=True`
  - servo command with `script_active=False`
  - invalid base command
  - invalid payload encoding path if feasible
- Confirm stdout/log capture has compact one-line format.
- Confirm no duplicate log lines after repeated setup.
- Confirm command side effects remain unchanged.

## Out of scope

- JSON structured logging.
- File-based log rotation.
- Migrating every RPi module to logging.
- Changing MQTT contracts.
- Changing dashboard route behavior.
- Changing servo/base/relay control logic.
