#!/bin/sh
set -eu

case "${START_AVAHI:-true}" in
  1|true|TRUE|yes|YES|on|ON)
    if ! pidof dbus-daemon >/dev/null 2>&1; then
      mkdir -p /run/dbus
      rm -f /run/dbus/pid 2>/dev/null || true
      dbus-daemon --system --fork || true
    fi
    if ! pidof avahi-daemon >/dev/null 2>&1; then
      mkdir -p /run/avahi-daemon
      rm -f /run/avahi-daemon/pid 2>/dev/null || true
      avahi-daemon --no-drop-root --daemonize --no-chroot || true
      sleep "${AVAHI_STARTUP_DELAY_S:-1}"
    fi
    ;;
esac

case "${START_PIGPIOD:-true}" in
  1|true|TRUE|yes|YES|on|ON)
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
