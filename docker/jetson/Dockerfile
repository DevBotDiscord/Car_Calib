# syntax=docker/dockerfile:1.7
# Jetson Nano (ARM64 / L4T) — self-calibration + direct GPIO
#
# Build:
#   docker build -f docker/jetson/Dockerfile -t car-calib-jetson .
#
# Run:
#   docker compose -f docker-compose.jetson.yml up -d

ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r36.4.0
FROM ${BASE_IMAGE}

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    DEBIAN_FRONTEND=noninteractive

WORKDIR /app

# ------------------------------------------------------------------ #
# System dependencies
# ------------------------------------------------------------------ #
RUN rm -f /etc/apt/apt.conf.d/docker-clean \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-dev \
        libopencv-dev \
        python3-opencv \
        libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------------ #
# Python dependencies
# ------------------------------------------------------------------ #
COPY requirements_jetson.txt /tmp/requirements.txt

RUN python3 -m pip install --no-cache-dir -r /tmp/requirements.txt

# ------------------------------------------------------------------ #
# App source
# ------------------------------------------------------------------ #
COPY . /app

RUN mkdir -p /data/logs /data/certs

ENV MAIN_CSV_LOG_FILE=/data/logs/run_log.csv \
    MAIN_TERMINAL_LOG=true \
    DASHBOARD_HOST=0.0.0.0 \
    DASHBOARD_PORT=8080

ENTRYPOINT ["python3", "main_jetson.py"]
