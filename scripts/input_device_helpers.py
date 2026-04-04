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
