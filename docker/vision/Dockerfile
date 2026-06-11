# syntax=docker/dockerfile:1.7
ARG BASE_IMAGE=python:3.11-slim-bookworm
FROM ${BASE_IMAGE}

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1

WORKDIR /app

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ffmpeg \
        libgl1 \
        libglib2.0-0 \
        libgomp1 \
        build-essential \
        linux-libc-dev \
        libnss-mdns \
        avahi-utils \
    && rm -rf /var/lib/apt/lists/* \
    && sed -i 's/^hosts:.*/hosts:          files mdns4_minimal [NOTFOUND=return] dns/' /etc/nsswitch.conf

# arduino-cli + ESP cores — needed for dashboard USB actuator firmware flashing.
# Placed BEFORE requirements.txt so editing Python deps does not invalidate
# this layer: the heavy ESP core download stays cached across deps changes.
# Skipped automatically at runtime if the binary is absent, so this layer
# is only required when ACTUATOR_MODE uses the USB actuator path.
ARG INSTALL_ARDUINO_CLI=true
ENV ARDUINO_DIRECTORIES_DATA=/opt/arduino \
    ARDUINO_DIRECTORIES_DOWNLOADS=/opt/arduino/staging \
    ARDUINO_DIRECTORIES_USER=/opt/arduino/user
RUN if [ "$INSTALL_ARDUINO_CLI" = "true" ]; then \
        apt-get update \
        && apt-get install -y --no-install-recommends curl ca-certificates \
        && curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=/usr/local/bin sh \
        && arduino-cli config init \
        && arduino-cli core update-index --additional-urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json,https://arduino.esp8266.com/stable/package_esp8266com_index.json \
        && arduino-cli core install esp32:esp32 --additional-urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json \
        && arduino-cli core install esp8266:esp8266 --additional-urls https://arduino.esp8266.com/stable/package_esp8266com_index.json \
        && rm -rf /var/lib/apt/lists/* /opt/arduino/staging ; \
    fi

COPY requirements.txt /tmp/requirements.txt

RUN python -m pip install --upgrade pip \
    && python -m pip install --retries 5 --timeout 120 -r /tmp/requirements.txt

COPY . /app

RUN mkdir -p /data/certs

ENV MAIN_CSV_LOG_FILE=/data/run_log.csv \
    MAIN_HTTPS_CERT_FILE=/data/certs/main_stream_cert.pem \
    MAIN_HTTPS_KEY_FILE=/data/certs/main_stream_key.pem \
    MAIN_SHOW_PREVIEW=false \
    MAIN_TERMINAL_LOG=true

ENTRYPOINT ["python", "main.py"]
