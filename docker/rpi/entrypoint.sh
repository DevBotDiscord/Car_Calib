#!/bin/sh
set -eu

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
