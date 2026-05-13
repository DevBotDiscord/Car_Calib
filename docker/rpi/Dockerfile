# syntax=docker/dockerfile:1.7
ARG BASE_IMAGE=python:3.11-slim-bookworm
ARG PIGPIO_VERSION=v78

FROM ${BASE_IMAGE} AS builder
ARG PIGPIO_VERSION

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1

WORKDIR /build

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        gcc \
        make \
        python3-dev \
        wget \
    && rm -rf /var/lib/apt/lists/*

COPY requirements-rpi.txt /tmp/requirements-rpi.txt

RUN python -m pip install --upgrade pip \
    && python -m pip wheel --wheel-dir /tmp/wheels -r /tmp/requirements-rpi.txt

RUN wget -O /tmp/pigpio.tar.gz "https://github.com/joan2937/pigpio/archive/refs/tags/${PIGPIO_VERSION}.tar.gz" \
    && tar -xzf /tmp/pigpio.tar.gz -C /tmp \
    && make -C "/tmp/pigpio-${PIGPIO_VERSION#v}" \
    && make -C "/tmp/pigpio-${PIGPIO_VERSION#v}" install DESTDIR=/tmp/pigpio-root

FROM ${BASE_IMAGE}

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1

ENV PYTHONPATH=/usr/lib/python3/dist-packages

WORKDIR /app

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

COPY requirements-rpi.txt /tmp/requirements-rpi.txt
COPY --from=builder /tmp/wheels /tmp/wheels
RUN python -m pip install --upgrade pip \
    && python -m pip install --no-index --find-links=/tmp/wheels --no-compile -r /tmp/requirements-rpi.txt \
    && rm -rf /tmp/wheels

COPY --from=builder /tmp/pigpio-root/usr/local/bin/pigpiod /usr/local/bin/pigpiod
COPY --from=builder /tmp/pigpio-root/usr/local/bin/pigs /usr/local/bin/pigs
COPY --from=builder /tmp/pigpio-root/usr/local/bin/pig2vcd /usr/local/bin/pig2vcd
COPY --from=builder /tmp/pigpio-root/usr/local/lib/libpigpio.so* /usr/local/lib/
COPY --from=builder /tmp/pigpio-root/usr/local/lib/libpigpiod_if.so* /usr/local/lib/
COPY --from=builder /tmp/pigpio-root/usr/local/lib/libpigpiod_if2.so* /usr/local/lib/
RUN ldconfig

COPY . /app
COPY docker/rpi/entrypoint.sh /entrypoint.sh

RUN chmod +x /entrypoint.sh

ENV PIGPIO_HOST=127.0.0.1 \
    PIGPIO_PORT=8888 \
    START_PIGPIOD=true \
    PIGPIOD_STARTUP_DELAY_S=1

ENTRYPOINT ["/entrypoint.sh"]
CMD ["python", "-m", "scripts.rpi.bridge"]
