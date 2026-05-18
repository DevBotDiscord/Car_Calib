#!/bin/sh
set -eu

# ---------------------------------------------------------------------------
# Start D-Bus + Avahi daemons so libnss-mdns can resolve *.local hostnames
# inside the container (e.g. car-brain.local for the MQTT broker).
# ---------------------------------------------------------------------------
case "${START_AVAHI:-true}" in
  1|true|TRUE|yes|YES|on|ON)
    if ! pidof dbus-daemon >/dev/null 2>&1; then
      mkdir -p /run/dbus
      dbus-daemon --system --fork || true
    fi
    if ! pidof avahi-daemon >/dev/null 2>&1; then
      mkdir -p /run/avahi-daemon
      avahi-daemon --no-drop-root --daemonize --no-chroot || true
      sleep "${AVAHI_STARTUP_DELAY_S:-1}"
    fi
    ;;
esac

case "${START_PIGPIOD:-true}" in
  1|true|TRUE|yes|YES|on|ON)
    if pigs t >/dev/null 2>&1; then
      echo "pigpiod already running; skip start"
    else
      pigpiod || true
      sleep "${PIGPIOD_STARTUP_DELAY_S:-1}"
    fi
    ;;
esac

exec "$@"
