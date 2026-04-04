"""Helpers for optional local input devices on Raspberry Pi bridges."""

from __future__ import annotations

from typing import Any, Callable


def open_optional_input_device(
    device_path: str,
    *,
    log: Callable[[str], None],
    device_factory: Callable[[str], Any],
) -> Any | None:
    """Open an input device if present, otherwise log and skip it."""
    try:
        return device_factory(device_path)
    except FileNotFoundError:
        log(f"INPUT: device not found, skipping local input: {device_path}")
        return None
    except OSError as exc:
        log(f"INPUT: failed to open {device_path}, skipping local input: {exc}")
        return None


def find_optional_abs_input_device(
    device_path: str,
    *,
    log: Callable[[str], None],
    device_factory: Callable[[str], Any],
    list_devices_fn: Callable[[], list[str]],
    name_hints: tuple[str, ...],
    ev_abs_code: int,
) -> Any | None:
    """Find an EV_ABS-capable input device, optionally matching name hints."""
    if device_path:
        return open_optional_input_device(
            device_path,
            log=log,
            device_factory=device_factory,
        )

    candidates: list[str] = []
    normalized_hints = tuple(hint.strip().lower() for hint in name_hints if hint.strip())

    for path in list_devices_fn():
        device = None
        selected = False
        try:
            device = device_factory(path)
            capabilities = device.capabilities()
            if ev_abs_code not in capabilities:
                continue

            name = (getattr(device, "name", "") or "Unknown input").strip() or "Unknown input"
            candidates.append(f"{name} ({path})")
            lowered_name = name.lower()
            if not normalized_hints or any(hint in lowered_name for hint in normalized_hints):
                selected = True
                return device
        except OSError as exc:
            log(f"INPUT: failed to inspect {path}, skipping: {exc}")
        finally:
            if device is not None and not selected:
                close = getattr(device, "close", None)
                if callable(close):
                    close()

    if candidates:
        log(
            "INPUT: no matching controller found, skipping local controller. "
            f"Detected EV_ABS devices: {', '.join(candidates)}"
        )
        return None

    log("INPUT: no EV_ABS controller found, skipping local controller")
    return None
