"""A bounded, shared JPEG producer for the embedded dashboard stream."""

from __future__ import annotations

import threading
import time
from typing import Any

import cv2
import numpy as np


class DashboardStreamBroker:
    """Encodes each dashboard frame at most once and shares it with all clients."""

    def __init__(self, *, max_clients: int, fps: float, jpeg_quality: int) -> None:
        self._max_clients = max(1, int(max_clients))
        self._interval_s = 1.0 / max(1.0, float(fps))
        self._jpeg_quality = max(1, min(100, int(jpeg_quality)))
        self._condition = threading.Condition()
        self._pending: np.ndarray | None = None
        self._jpeg: bytes | None = None
        self._sequence = 0
        self._clients = 0
        self._rejected_clients = 0
        self._closed = False
        self._last_submit = 0.0
        self._worker = threading.Thread(target=self._run, name="dashboard-jpeg", daemon=True)
        self._worker.start()

    def submit(self, frame: np.ndarray) -> None:
        now = time.monotonic()
        with self._condition:
            if self._closed or self._clients == 0 or now - self._last_submit < self._interval_s:
                return
            self._last_submit = now
            self._pending = frame.copy()
            self._condition.notify()

    def acquire_client(self) -> bool:
        with self._condition:
            if self._closed or self._clients >= self._max_clients:
                self._rejected_clients += 1
                return False
            self._clients += 1
            return True

    def release_client(self) -> None:
        with self._condition:
            self._clients = max(0, self._clients - 1)

    def wait_for_frame(self, sequence: int, timeout_s: float) -> tuple[bytes | None, int]:
        with self._condition:
            self._condition.wait_for(
                lambda: self._closed or (self._jpeg is not None and self._sequence != sequence),
                timeout=max(0.1, timeout_s),
            )
            return self._jpeg, self._sequence

    def health(self) -> dict[str, int]:
        with self._condition:
            return {
                "stream_clients": self._clients,
                "stream_max_clients": self._max_clients,
                "stream_rejected_clients": self._rejected_clients,
            }

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify_all()
        self._worker.join(timeout=1.0)

    def _run(self) -> None:
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._closed or self._pending is not None)
                if self._closed:
                    return
                frame = self._pending
                self._pending = None
            if frame is None:
                continue
            ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality])
            if not ok:
                continue
            with self._condition:
                self._jpeg = encoded.tobytes()
                self._sequence += 1
                self._condition.notify_all()
