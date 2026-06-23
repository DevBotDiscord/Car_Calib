# syntax=docker/dockerfile:1.7
# Jetson Nano (ARM64 / L4T) — self-calibration + direct GPIO
#
# dusty image already bundles OpenCV + Python3 + pip.
# Only pip layer can change between deploys → fast rebuilds.
#
# Build:
#   DOCKER_BUILDKIT=1 docker build -f docker/jetson/Dockerfile -t car-calib-jetson .

ARG BASE_IMAGE=dustynv/l4t-jetpack:r36.4.0
FROM ${BASE_IMAGE}

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

WORKDIR /app

# Verify base image has what we need
RUN python3 -c "import cv2; print('OpenCV', cv2.__version__)" \
    && python3 -c "import numpy; print('numpy', numpy.__version__)"

# Layer 2 – Python deps (cached unless requirements_jetson.txt changes)
COPY requirements_jetson.txt /tmp/requirements.txt
RUN --mount=type=cache,target=/root/.cache/pip \
    python3 -m pip install -r /tmp/requirements.txt

# Layer 3 – App source (invalidated on any .py change — seconds to rebuild)
COPY . /app
RUN mkdir -p /data/logs /data/certs

ENV MAIN_CSV_LOG_FILE=/data/logs/run_log.csv \
    MAIN_TERMINAL_LOG=true \
    DASHBOARD_HOST=0.0.0.0 \
    DASHBOARD_PORT=8080

ENTRYPOINT ["python3", "main_jetson.py"]
