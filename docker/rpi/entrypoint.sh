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
      # Clean stale pid file from previous crashed container start.
      if [ -f /run/dbus/pid ]; then
        rm -f /run/dbus/pid
      fi
      dbus-daemon --system --fork || true
    fi
    if ! pidof avahi-daemon >/dev/null 2>&1; then
      mkdir -p /run/avahi-daemon
      # Clean stale pid file from previous crashed container start.
      if [ -f /run/avahi-daemon/pid ]; then
        rm -f /run/avahi-daemon/pid
      fi
      avahi-daemon --no-drop-root --daemonize --no-chroot || true
      sleep "${AVAHI_STARTUP_DELAY_S:-1}"
    fi
    ;;
esac

case "${START_PIGPIOD:-true}" in
  1|true|TRUE|yes|YES|on|ON)
    # Always start a fresh pigpiod: kill any existing instance (host daemon or
    # leftover from a prior container start), wait for the socket to release,
    # then launch and verify it accepts connections.
    if pidof pigpiod >/dev/null 2>&1 || pigs t >/dev/null 2>&1; then
      echo "pigpiod already running; killing for fresh start"
      pkill -x pigpiod 2>/dev/null || true
      for _ in 1 2 3 4 5 6 7 8 9 10; do
        pidof pigpiod >/dev/null 2>&1 || break
        sleep 0.5
      done
    fi

    pigpiod
    sleep "${PIGPIOD_STARTUP_DELAY_S:-1}"

    pigpiod_ok=""
    for _ in 1 2 3 4 5 6 7 8 9 10; do
      if pigs t >/dev/null 2>&1; then
        pigpiod_ok="yes"
        break
      fi
      sleep 0.5
    done
    if [ -z "$pigpiod_ok" ]; then
      echo "ERROR: pigpiod failed to start / not accepting connections" >&2
      exit 1
    fi
    echo "pigpiod fresh start OK"
    ;;
esac

exec "$@"
