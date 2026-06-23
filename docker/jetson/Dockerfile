# syntax=docker/dockerfile:1.7
# Jetson Nano (ARM64 / L4T) — self-calibration + direct GPIO
#
# Cache strategy: slow layers first (apt, pip), source last.
# BuildKit cache mounts avoid re-downloading packages every build.
#
# Build:
#   DOCKER_BUILDKIT=1 docker build -f docker/jetson/Dockerfile -t car-calib-jetson .

ARG BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r36.4.0
FROM ${BASE_IMAGE}

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    DEBIAN_FRONTEND=noninteractive

WORKDIR /app

# ===================================================================== #
# Layer 1 – system packages (cached until Dockerfile or repo changes)
# ===================================================================== #
RUN rm -f /etc/apt/apt.conf.d/docker-clean \
    && --mount=type=cache,target=/var/cache/apt,sharing=locked \
       --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-dev \
        libopencv-dev \
        python3-opencv \
        libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# ===================================================================== #
# Layer 2 – Python deps (cached until requirements_jetson.txt changes)
# ===================================================================== #
COPY requirements_jetson.txt /tmp/requirements.txt

RUN --mount=type=cache,target=/root/.cache/pip \
    python3 -m pip install -r /tmp/requirements.txt

# ===================================================================== #
# Layer 3 – App source (invalidated on any source change)
# ===================================================================== #
COPY . /app

RUN mkdir -p /data/logs /data/certs

ENV MAIN_CSV_LOG_FILE=/data/logs/run_log.csv \
    MAIN_TERMINAL_LOG=true \
    DASHBOARD_HOST=0.0.0.0 \
    DASHBOARD_PORT=8080

ENTRYPOINT ["python3", "main_jetson.py"]
