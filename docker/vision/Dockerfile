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
