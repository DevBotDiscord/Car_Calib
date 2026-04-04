# syntax=docker/dockerfile:1.7
ARG BASE_IMAGE=python:3.11-slim-bookworm
FROM ${BASE_IMAGE}

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1

WORKDIR /app

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        gcc \
        libevdev-dev \
        pigpio \
        python3-dev \
    && rm -rf /var/lib/apt/lists/*

COPY requirements-rpi.txt /tmp/requirements-rpi.txt

RUN python -m pip install --upgrade pip \
    && python -m pip install -r /tmp/requirements-rpi.txt

COPY . /app
COPY docker/rpi/entrypoint.sh /entrypoint.sh

RUN chmod +x /entrypoint.sh

ENV PIGPIO_HOST=127.0.0.1 \
    PIGPIO_PORT=8888 \
    START_PIGPIOD=true \
    PIGPIOD_STARTUP_DELAY_S=1

ENTRYPOINT ["/entrypoint.sh"]
CMD ["python", "scripts/rpi_mqtt_bridge.py"]
