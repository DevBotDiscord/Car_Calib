"""Process-wide flag indicating the route-script runner is pinning the servo.

When a route script step pins the servo angle (left / right / backward),
the vision PID stream must not publish its own steering target or the
two streams will fight at different rates. The script runner toggles
this flag while a pinned step is active; the servo driver consults it
before publishing.
"""

from __future__ import annotations

import threading

_lock = threading.Lock()
_pinned: bool = False


def set_pinned(pinned: bool) -> None:
    """Mark the servo as script-pinned (or release it)."""
    global _pinned
    with _lock:
        _pinned = bool(pinned)


def is_pinned() -> bool:
    """Return True while the script runner is pinning the servo angle."""
    with _lock:
        return _pinned
