"""Route session logging and dataset acceptance helpers."""

from __future__ import annotations

import json
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional
from uuid import uuid4

from config.settings import (
    ROUTE_ACCEPT_MAX_GAP_RATIO,
    ROUTE_ACCEPT_MAX_HW_ERRORS,
    ROUTE_ACCEPT_MIN_FRAMES,
    ROUTE_DIRECTION_EPS_DEG,
    ROUTE_LOG_ROOT,
)


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _safe_float(value: Optional[float]) -> Optional[float]:
    if value is None:
        return None
    return float(value)


@dataclass
class RouteFinalizeResult:
    route_id: str
    accepted: bool
    rejection_reason: str
    summary_path: str


class RouteSession:
    """Collects route-level metadata and writes a summary JSON on finalize."""

    def __init__(self) -> None:
        self.route_id = f"route-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}-{uuid4().hex[:8]}"
        self.started_at_utc = _utc_now_iso()
        self._start_monotonic: float | None = None
        self._end_monotonic: float | None = None
        self._ended_at_utc: str | None = None

        self.total_frames = 0
        self.frames_with_theta = 0
        self.hw_error_count = 0
        self.abstract_steps = 0

        self._pending_direction: str | None = None
        self._pending_count = 0
        self._stable_direction: str | None = None
        self._recent_directions: deque[str] = deque(maxlen=5)

    def start(self, mono_now: float) -> None:
        self._start_monotonic = mono_now

    def record_hw_error(self) -> None:
        self.hw_error_count += 1

    def update_frame(
        self,
        *,
        mono_now: float,
        theta: Optional[float],
        fsm_state: str,
        calibration_active: bool,
    ) -> tuple[Optional[float], str, str]:
        if self._start_monotonic is None:
            self.start(mono_now)

        self.total_frames += 1
        angle_diff: Optional[float] = None
        if theta is not None:
            self.frames_with_theta += 1
            angle_diff = abs(theta - 90.0)

        direction = self._direction_from_theta(theta)
        self._update_abstract_steps(direction)
        calib_status = self._calib_status(direction, fsm_state, calibration_active)
        return angle_diff, calib_status, direction

    def finalize(self, *, mono_now: float, status: str, explicit_rejection_reason: str = "") -> RouteFinalizeResult:
        self._end_monotonic = mono_now
        self._ended_at_utc = _utc_now_iso()

        elapsed_s = 0.0
        if self._start_monotonic is not None:
            elapsed_s = max(0.0, self._end_monotonic - self._start_monotonic)

        gap_frames = self.total_frames - self.frames_with_theta
        gap_ratio = (gap_frames / self.total_frames) if self.total_frames > 0 else 1.0

        accepted, rejection_reason = self._evaluate_acceptance(
            route_status=status,
            gap_ratio=gap_ratio,
            explicit_rejection_reason=explicit_rejection_reason,
        )

        payload = {
            "route_id": self.route_id,
            "start_timestamp_utc": self.started_at_utc,
            "end_timestamp_utc": self._ended_at_utc,
            "abstract_steps": self.abstract_steps,
            "total_elapsed_seconds": elapsed_s,
            "status": status,
            "accepted": accepted,
            "rejection_reason": rejection_reason,
            "total_frames": self.total_frames,
            "frames_with_theta": self.frames_with_theta,
            "gap_ratio": gap_ratio,
            "hardware_error_count": self.hw_error_count,
            "route_direction_eps_deg": ROUTE_DIRECTION_EPS_DEG,
        }

        root = Path(ROUTE_LOG_ROOT)
        try:
            root.mkdir(parents=True, exist_ok=True)
        except OSError:
            root = Path("logs/routes")
            root.mkdir(parents=True, exist_ok=True)
        summary_path = root / f"{self.route_id}.summary.json"
        summary_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

        return RouteFinalizeResult(
            route_id=self.route_id,
            accepted=accepted,
            rejection_reason=rejection_reason,
            summary_path=str(summary_path),
        )

    def _direction_from_theta(self, theta: Optional[float]) -> str:
        if theta is None:
            return "UNKNOWN"
        delta = theta - 90.0
        if abs(delta) <= ROUTE_DIRECTION_EPS_DEG:
            return "STRAIGHT"
        if delta > 0:
            return "RIGHT"
        return "LEFT"

    def _calib_status(self, direction: str, fsm_state: str, calibration_active: bool) -> str:
        if fsm_state == "GAPPING":
            return "GAPPING"
        if direction == "UNKNOWN":
            return "NO_REFERENCE"
        if not calibration_active:
            return "DRIVING_STRAIGHT"
        if direction == "LEFT":
            return "CALIBRATING_LEFT"
        if direction == "RIGHT":
            return "CALIBRATING_RIGHT"
        return "CALIBRATED"

    def _update_abstract_steps(self, direction: str) -> None:
        if direction == "UNKNOWN":
            return

        self._recent_directions.append(direction)
        if self._pending_direction == direction:
            self._pending_count += 1
        else:
            self._pending_direction = direction
            self._pending_count = 1

        # Require 3 consecutive frames to accept a direction segment.
        if self._pending_count < 3:
            return

        if self._stable_direction != self._pending_direction:
            self._stable_direction = self._pending_direction
            self.abstract_steps += 1

    def _evaluate_acceptance(
        self,
        *,
        route_status: str,
        gap_ratio: float,
        explicit_rejection_reason: str,
    ) -> tuple[bool, str]:
        if explicit_rejection_reason:
            return False, explicit_rejection_reason
        if route_status != "COMPLETED":
            return False, f"route_status={route_status}"
        if self.total_frames < ROUTE_ACCEPT_MIN_FRAMES:
            return False, f"insufficient_frames<{ROUTE_ACCEPT_MIN_FRAMES}"
        if self.hw_error_count > ROUTE_ACCEPT_MAX_HW_ERRORS:
            return False, f"hardware_errors>{ROUTE_ACCEPT_MAX_HW_ERRORS}"
        if gap_ratio > ROUTE_ACCEPT_MAX_GAP_RATIO:
            return False, f"gap_ratio>{ROUTE_ACCEPT_MAX_GAP_RATIO:.3f}"
        return True, ""
