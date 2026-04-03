# ESP32 MQTT Bridge

This setup moves servo and base control onto an ESP32. The vision side
publishes the servo angle through MQTT, and the ESP32 subscribes to MQTT
topics to drive both the steering servo and the base output pins.

## Files

- `drivers/servo_driver.py`: optional MQTT publisher for servo angles
- `firmware/esp32_mqtt_bridge/esp32_mqtt_bridge.ino`: ESP32 subscriber firmware
- `scripts/mqtt_base_command.py`: simple base-command publisher for testing

## MQTT Topics

- `car/servo/angle`: payload is a plain floating-point angle, for example `95.25`
- `car/base/command`: payload is one of `STOP`, `FORWARD`, `BACKWARD`, `LOCK`, `UNLOCK`
- `car/status`: ESP32 publishes online status

## Vision Side

Install dependencies:

```bash
pip install -r requirements.txt
```

Enable MQTT publishing in `.env`:

```bash
DRIVER_SERVO_MQTT_ENABLED=true
DRIVER_SERVO_BRIDGE_ENABLED=false
MQTT_BROKER_HOST=192.168.1.50
MQTT_BROKER_PORT=1883
MQTT_SERVO_TOPIC=car/servo/angle
```

Then run the existing app as usual:

```bash
python main.py --debug
```

`main.py` will publish the servo angle directly from `ServoDriver`.

## ESP32 Side

Required Arduino libraries:

- `PubSubClient`
- `ESP32Servo`

Open and edit:

- `firmware/esp32_mqtt_bridge/esp32_mqtt_bridge.ino`

Update these values before flashing:

- `WIFI_SSID`
- `WIFI_PASSWORD`
- `MQTT_HOST`
- `MQTT_PORT`
- `MQTT_USERNAME`
- `MQTT_PASSWORD`

Default pin mapping in the firmware:

- Servo: `GPIO19`
- Base OUT1: `GPIO17`
- Base OUT2: `GPIO27`
- Base OUT3: `GPIO22`

## Servo Angle Mapping

By default, the ESP32 firmware expects the vision side to publish angles
in the `60 .. 90 .. 120` range, which matches the current `Car_Calib`
servo output style around center `90`.

If you want to publish the same signed angles used by your old keyboard
controller, change these constants in the `.ino` file:

```cpp
const float REMOTE_INPUT_MIN_ANGLE = -65.0f;
const float REMOTE_INPUT_CENTER_ANGLE = -8.0f;
const float REMOTE_INPUT_MAX_ANGLE = 60.0f;
```

## Base Testing

After the ESP32 is online, you can test base commands from the vision
machine:

```bash
python scripts/mqtt_base_command.py FORWARD
python scripts/mqtt_base_command.py STOP
```

## Broker

Any MQTT broker will work. A common local option is Mosquitto on the same
LAN as the vision machine and ESP32. The broker only needs normal TCP MQTT
access on port `1883` unless you choose a different port.
