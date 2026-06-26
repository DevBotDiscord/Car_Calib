#!/bin/sh
set -eu

case "${START_PIGPIOD:-false}" in
  1|true|TRUE|yes|YES|on|ON)
    if ! pidof pigpiod >/dev/null 2>&1; then
      pigpiod
      sleep "${PIGPIOD_STARTUP_DELAY_S:-1}"
    fi
    if ! pigs t >/dev/null 2>&1; then
      echo "ERROR: pigpiod failed to start / not accepting connections" >&2
      exit 1
    fi
    echo "pigpiod OK"
    ;;
esac

exec "$@"
