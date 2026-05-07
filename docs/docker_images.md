# Docker Images

This repository now ships with two separate container targets:

- `vision`: runs `main.py` and publishes steering over MQTT
- `rpi-mqtt-bridge`: runs `scripts/rpi_mqtt_bridge.py` on the Raspberry Pi

They are meant to be built and deployed independently on their respective
machines.

## Files

- `docker/vision/Dockerfile`
- `docker/rpi/Dockerfile`
- `docker/rpi/entrypoint.sh`
- `docker-compose.vision.yml`
- `docker-compose.rpi.yml`
- `.dockerignore`
- `requirements-rpi.txt`

## Vision Image

Build:

```bash
docker build -f docker/vision/Dockerfile -t car-calib-vision:latest .
```

Run with Docker Compose:

```bash
docker compose -f docker-compose.vision.yml up --build -d
```

Notes:

- The compose file assumes a V4L2 camera such as `/dev/video0`
- Output files such as `run_log.csv` and generated HTTPS certs are stored in
  the Docker volume mounted at `/data`
- The stack now includes a local Mosquitto broker service (`mqtt`) on MiniPC
  with config at `docker/mosquitto/mosquitto.conf`
- `network_mode: host` is used so the container can expose the MJPEG stream
  directly and reach the MQTT broker without extra port mapping
- `MAIN_SHOW_PREVIEW=false` is the safe default for container use

If your vision host is Jetson Nano with CSI camera support, override the base
image during build with an L4T-compatible image:

```bash
docker build \
  --build-arg BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r35.4.1 \
  -f docker/vision/Dockerfile \
  -t car-calib-vision:jetson .
```

## Raspberry Pi MQTT Bridge Image

Build:

```bash
docker build -f docker/rpi/Dockerfile -t car-calib-rpi-mqtt-bridge:latest .
```

Run with Docker Compose:

```bash
docker compose -f docker-compose.rpi.yml up --build -d
```

Notes:

- The RPi container uses `privileged: true` because it needs GPIO and input
  device access for `pigpiod` and `/dev/input`
- `/dev/input` is mounted read-only so keyboard/controller devices remain
  visible inside the container
- `pigpiod` is started automatically by `docker/rpi/entrypoint.sh`
- The container joins host networking so `PIGPIO_HOST=127.0.0.1` and MQTT
  broker settings behave the same as bare metal

## Typical Deployment Flow

### Vision host

```bash
cp .env.example .env
# edit MQTT host, camera index, debug flags, etc.
docker compose -f docker-compose.vision.yml up --build -d
```

### Raspberry Pi host

```bash
cp .env.example .env
# edit MQTT host, SERVO_PIN, KEYBOARD_DEVICE, GAMEPAD_DEVICE, etc.
docker compose -f docker-compose.rpi.yml up --build -d
```

## Production Deployment Script

A production helper similar to your `RobotOS` deploy script now exists at:

- `deploy_production.sh`
- `deploy_production.bat`

Prepare:

```bash
cp .env.production.example .env.production
# fill in MINIPC_HOST / MINIPC_USER / RPI_HOST / RPI_USER / MQTT / camera / GPIO settings
chmod +x deploy_production.sh
```

Run:

```bash
./deploy_production.sh
./deploy_production.sh minipc
./deploy_production.sh ras
```

On Windows:

```bat
deploy_production.bat
deploy_production.bat minipc
deploy_production.bat ras
```

The `.bat` wrapper looks for `bash.exe` from Git for Windows and then runs
the same `deploy_production.sh`, so the deploy logic stays in one place.

SSH auth behavior:

- leave `MINIPC_PASSWORD` / `RPI_PASSWORD` blank to use SSH keys
- if a password is set, the script uses `sshpass` when available
- on Windows, password mode can also use PuTTY tools via `plink.exe` and `pscp.exe`
- key mode now uses SSH batch mode, so it fails fast instead of hanging on a password prompt
- when using PuTTY password mode for a host not yet trusted, set `MINIPC_SSH_HOST_KEY` or `RPI_SSH_HOST_KEY` to the host `SHA256:...` fingerprint
- release directory operations now have separate flags: `MINIPC_USE_SUDO_REMOTE` and `RPI_USE_SUDO_REMOTE`
- these default to `true` because the sample deploy paths use `/opt/...`, which usually needs elevated privileges
- `MINIPC_SUDO_PASSWORD` / `RPI_SUDO_PASSWORD` are optional; when blank, the script reuses the SSH password first, then falls back to passwordless sudo

What it does:

- creates a release tarball from the repository
- uploads it to the MiniPC and/or Raspberry Pi over SSH
- writes `.env.production` as remote `.env`
- switches `current` to the new release under `releases/<version>`
- runs `docker compose up --build -d` on the target host

## Important Runtime Assumptions

- Vision and Raspberry Pi usually run on different machines, so use the two
  compose files separately rather than starting both on one host
- The MQTT broker must still be reachable from both containers
- If you need GUI preview from the vision container, add an X11/Wayland
  passthrough setup; this repository does not enable that by default
- If the Raspberry Pi host already runs `pigpiod`, set `START_PIGPIOD=false`
  for the container
