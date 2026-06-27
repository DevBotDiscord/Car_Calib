"""Telemetry bridge: CSV logging, frame store, video output.

Wraps the scattered telemetry logic previously inlined in main.py into one class.
"""

from __future__ import annotations

import logging
from datetime import datetime, timezone
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)

_CSV_FIELDNAMES = [
    "route_id", "route_mode", "frame_num", "mono_timestamp", "utc_timestamp",
    "loop_ms", "loop_overrun_ms", "fsm_state", "calibration_active",
    "theta", "theta_source", "theta_for_overlay", "theta_horizontal",
    "reference_group_index", "selected_group_bbox", "lines_count",
    "groups_count", "horizontal_ok", "sanity_ok", "stale_output",
    "servo_angle", "servo_center_angle", "servo_offset",
    "pid_error", "pid_p_term", "pid_i_term", "pid_d_term",
    "pid_integral", "pid_last_error", "hardware_send_latency_ms",
    "stream_enabled", "stream_host", "stream_port",
]


def _format_bbox(bbox: tuple[int, int, int, int] | None) -> str:
    if bbox is None:
        return ""
    x, y, w, h = bbox
    return f"{x},{y},{w},{h}"


class TelemetryLogger:
    """Single sink for run-time CSV, frame snapshots, and debug video."""

    def __init__(
        self,
        csv_writer: Any,
        csv_file: Any,
        frame_store: Any | None = None,
        video_writer: Any | None = None,
        video_size: tuple[int, int] | None = None,
        stream_enabled: bool = False,
        stream_host: str = "",
        stream_port: int = 0,
    ) -> None:
        self._csv_writer = csv_writer
        self._csv_file = csv_file
        self._frame_store = frame_store
        self._video_writer = video_writer
        self._video_size = video_size
        self._stream_enabled = stream_enabled
        self._stream_host = stream_host
        self._stream_port = stream_port

    # ------------------------------------------------------------------ #
    # CSV
    # ------------------------------------------------------------------ #

    def log_state(
        self,
        *,
        frame_num: int,
        loop_start: float,
        overrun_ms: float,
        state: Any,
        theta: float | None,
        last_known_theta: float | None,
        theta_source: str,
        servo_angle: float,
        pid_error: float,
        pid_p_term: float,
        pid_i_term: float,
        pid_d_term: float,
        hardware_send_latency_ms: float,
        route_session: Any | None = None,
        detector_debug: dict[str, Any] | None = None,
        route_csv_writer: Any | None = None,
        route_csv_file: Any | None = None,
        current_route_mode: str = "",
    ) -> None:
        """Write one CSV row to the run log and optionally to the route log."""
        if self._csv_writer is None:
            return

        _LOOP_PERIOD_MS = (1.0 / 30.0) * 1000.0
        theta_str = f"{theta:.4f}" if theta is not None else ""
        mono_ts = f"{loop_start:.6f}"
        utc_ts = datetime.now(timezone.utc).isoformat()
        selected_group_bbox = detector_debug.get("selected_group_bbox") if detector_debug else None

        row: dict[str, Any] = {
            "route_id": route_session.route_id if route_session is not None else "",
            "route_mode": route_session.route_mode if route_session is not None else current_route_mode,
            "frame_num": frame_num,
            "mono_timestamp": mono_ts,
            "utc_timestamp": utc_ts,
            "loop_ms": f"{overrun_ms + _LOOP_PERIOD_MS:.4f}",
            "loop_overrun_ms": f"{overrun_ms:.4f}",
            "fsm_state": state.fsm_state.name,
            "calibration_active": int(state.calibration_active),
            "theta": theta_str,
            "theta_source": theta_source,
            "theta_for_overlay": f"{last_known_theta:.4f}" if last_known_theta is not None else "",
            "theta_horizontal": (
                f"{detector_debug.get('theta_horizontal'):.4f}"
                if detector_debug and detector_debug.get("theta_horizontal") is not None
                else ""
            ),
            "reference_group_index": detector_debug.get("reference_group_index", "") if detector_debug else "",
            "selected_group_bbox": _format_bbox(selected_group_bbox),
            "lines_count": detector_debug.get("lines_count", "") if detector_debug else "",
            "groups_count": detector_debug.get("groups_count", "") if detector_debug else "",
            "horizontal_ok": detector_debug.get("horizontal_ok", "") if detector_debug else "",
            "sanity_ok": detector_debug.get("sanity_ok", "") if detector_debug else "",
            "stale_output": detector_debug.get("stale_output", "") if detector_debug else "",
            "servo_angle": f"{servo_angle:.4f}",
            "servo_center_angle": f"{state.servo_center_angle:.4f}",
            "servo_offset": f"{(servo_angle - state.servo_center_angle):.4f}",
            "pid_error": f"{pid_error:.6f}",
            "pid_p_term": f"{pid_p_term:.6f}",
            "pid_i_term": f"{pid_i_term:.6f}",
            "pid_d_term": f"{pid_d_term:.6f}",
            "pid_integral": f"{state.pid_integral:.6f}",
            "pid_last_error": f"{state.pid_last_error:.6f}",
            "hardware_send_latency_ms": f"{hardware_send_latency_ms:.4f}",
            "stream_enabled": int(self._stream_enabled),
            "stream_host": self._stream_host,
            "stream_port": str(self._stream_port) if self._stream_enabled else "",
        }

        self._csv_writer.writerow(row)
        if self._csv_file is not None:
            self._csv_file.flush()

        if route_session is not None and route_csv_writer is not None:
            route_csv_writer.writerow(row)
            if route_csv_file is not None:
                route_csv_file.flush()

    # ------------------------------------------------------------------ #
    # Stream / video
    # ------------------------------------------------------------------ #

    def publish_frame(self, frame_bgr: np.ndarray, telemetry: dict[str, Any]) -> None:
        """Push frame + telemetry to the HTTP MJPEG frame store."""
        if self._frame_store is not None:
            self._frame_store.set_frame(frame_bgr, telemetry)

    def write_debug_frame(self, frame: np.ndarray) -> None:
        """Write one frame to the debug video writer."""
        if self._video_writer is None:
            return
        if self._video_size is not None:
            frame = cv2.resize(frame, self._video_size, interpolation=cv2.INTER_AREA)
        self._video_writer.write(frame)

    def close(self) -> None:
        """Release video writer resources (CSV file is closed externally)."""
        if self._video_writer is not None:
            try:
                self._video_writer.release()
            except Exception:  # noqa: BLE001
                pass
