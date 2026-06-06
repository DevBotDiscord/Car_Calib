"""Emergency Stop hardware handler for Raspberry Pi.

Connects a physical NC push-button to GPIO 6.  A hardware glitch filter
plus software stable-window debounce prevent spurious triggers.  When the
button opens (pin reads HIGH for active-low wiring), the module latches
into a safe state:

- Stops the base motor immediately.
- Returns the servo to center and releases PWM hold.
- Publishes ``{"active": true}`` retained on MQTT.
- Blinks the relay briefly for visual warning.
- All future base/servo writes are gated off in ``base.py`` / ``steering.py``.

The latch only clears when ``try_reset()`` is called *and* the button reads
its safe (closed) level — a deliberate two-step recovery.
"""

from __future__ import annotations

import threading
import time

from . import config
from .base import stop_base
from .logging_utils import get_logger
from .steering import release_servo, steer_center

logger = get_logger("estop")

_estop_lock = threading.Lock()


def _read_pin() -> int:
    if config.gpio is None:
        return 0
    return config.gpio.read(config.ESTOP_GPIO)


def _pin_is_active() -> bool:
    """Return True if the button is currently pressed (NC open / safety state)."""
    level = _read_pin()
    return level == 0 if config.ESTOP_ACTIVE_LOW else level == 1


def _pin_stays_at(expected_level: int, duration_s: float) -> bool:
    """Poll the pin and return True only if it stays at *expected_level* for the full window."""
    if config.gpio is None:
        return False
    deadline = time.monotonic() + max(0.0, duration_s)
    while time.monotonic() < deadline:
        if config.gpio.read(config.ESTOP_GPIO) != expected_level:
            return False
        time.sleep(0.005)
    return config.gpio.read(config.ESTOP_GPIO) == expected_level


# ------------------------------------------------------------------ #
# Latch / reset
# ------------------------------------------------------------------ #


def latch(reason: str = "gpio") -> bool:
    """Activate E-stop and force outputs safe.  Returns True only on the first call."""
    with _estop_lock:
        if config.estop_active:
            return False
        config.estop_active = True
        config.estop_latched_at = time.time()
        logger.error("[ESTOP][LATCH] reason=%s", reason)

    _safe_outputs()
    _publish_estop(True)
    _blink_relay_async()
    return True


def try_reset(reason: str = "manual") -> bool:
    """Reset the latch only if the physical button is currently in the safe (closed) state."""
    with _estop_lock:
        if not config.estop_active:
            return True

    if _pin_is_active():
        logger.warning("[ESTOP][RESET] rejected reason=%s — button still active", reason)
        _publish_estop(True)
        return False

    with _estop_lock:
        config.estop_active = False
        config.estop_latched_at = None
        logger.warning("[ESTOP][RESET] cleared reason=%s", reason)

    _publish_estop(False)
    return True


# ------------------------------------------------------------------ #
# Safe outputs / MQTT / relay blink
# ------------------------------------------------------------------ #


def _safe_outputs() -> None:
    try:
        stop_base(force=True)
    except Exception as exc:
        logger.error("[ESTOP][SAFE] stop_base failed: %s", exc)
    try:
        steer_center("ESTOP")
    except Exception as exc:
        logger.error("[ESTOP][SAFE] steer_center failed: %s", exc)
    try:
        release_servo("ESTOP")
    except Exception as exc:
        logger.error("[ESTOP][SAFE] release_servo failed: %s", exc)


def _publish_estop(active: bool) -> None:
    try:
        from .mqtt_client import publish_estop  # local import to avoid cycle

        publish_estop(active)
    except Exception as exc:
        logger.warning("[ESTOP][MQTT] publish failed: %s", exc)


def _blink_relay_async() -> None:
    threading.Thread(target=_blink_relay_loop, daemon=True, name="estop-relay-blink").start()


def _blink_relay_loop() -> None:
    if config.gpio is None:
        return
    end_at = time.monotonic() + config.ESTOP_BLINK_RELAY_S
    on = False
    interval = 0.12
    while config.estop_active and time.monotonic() < end_at:
        on = not on
        try:
            config.gpio.write(config.RELAY_PIN, 1 if on else 0)
        except Exception:
            break
        config.relay_on = on
        time.sleep(interval)
    try:
        config.gpio.write(config.RELAY_PIN, 0)
    except Exception:
        pass
    config.relay_on = False


# ------------------------------------------------------------------ #
# pigpio callback + setup
# ------------------------------------------------------------------ #


def _estop_callback(gpio: object, level: int, tick: int) -> None:
    del gpio, tick
    active_level = 0 if config.ESTOP_ACTIVE_LOW else 1
    if level == active_level:
        if _pin_stays_at(int(active_level), config.ESTOP_LATCH_STABLE_S):
            latch("gpio")
        else:
            logger.info("[ESTOP][DEBOUNCE] ignored unstable activation")
    else:
        logger.info("[ESTOP][RELEASE] button released — awaiting reset")


def setup() -> None:
    """Configure the E-stop pin: input + pull, glitch filter, edge callback."""
    if config.gpio is None:
        raise RuntimeError("pigpio is not initialized; cannot setup E-stop")

    pi = config.pigpio
    config.gpio.set_mode(config.ESTOP_GPIO, pi.INPUT)
    if config.ESTOP_ACTIVE_LOW:
        config.gpio.set_pull_up_down(config.ESTOP_GPIO, pi.PUD_UP)
    else:
        config.gpio.set_pull_up_down(config.ESTOP_GPIO, pi.PUD_DOWN)

    config.gpio.set_glitch_filter(config.ESTOP_GPIO, config.ESTOP_DEBOUNCE_US)
    config.gpio.callback(config.ESTOP_GPIO, pi.EITHER_EDGE, _estop_callback)

    logger.info(
        "[ESTOP][SETUP] pin=%s active_low=%s debounce_us=%s latch_stable_s=%.3f",
        config.ESTOP_GPIO,
        config.ESTOP_ACTIVE_LOW,
        config.ESTOP_DEBOUNCE_US,
        config.ESTOP_LATCH_STABLE_S,
    )

    if _pin_is_active():
        latch("boot")
