"""Control module: V2 vanishing-point steering state machine.

Mirrors the SteeringController used by the offline UnifiedCalibrator so the
live loop and the offline processor share one calibration law. States:

* ``GAPPING``        — vision lost (no VP / no intercepts), hold center
* ``DANGER_LEFT``    — left intercept past margin, fixed nudge right
* ``DANGER_RIGHT``   — right intercept past margin, fixed nudge left
* ``TRACKING_COAST`` — error inside hysteresis dead-band, hold center
* ``TRACKING_PD``    — error past outer threshold, PD correction active

Angles are emitted in the configured servo convention. The VP error stays
90-centered, then output maps onto ``SERVO_CENTER_ANGLE`` ± max offset so
MiniPC MQTT commands match the RPi signed-physical angle convention.
"""

from __future__ import annotations

from models.robot_state import PIDConstants


class SteeringController:
    """State-machine governor implementing vision-lost, danger, and tracking stages."""

    def __init__(
        self,
        pid_constants: PIDConstants,
        danger_margin: int,
        nudge_deg: float,
        inner_thresh: float,
        outer_thresh: float,
        center_angle: float = 90.0,
        max_offset: float = 90.0,
    ) -> None:
        self._pid = pid_constants
        self._danger_margin = max(0, int(danger_margin))
        self._nudge_deg = float(nudge_deg)
        self._inner_thresh = abs(float(inner_thresh))
        self._outer_thresh = max(abs(float(outer_thresh)), self._inner_thresh)
        self._center = float(center_angle)
        self._max_offset = abs(float(max_offset))
        self._tracking_active = False
        self._last_error = 0.0

    def compute_steering(
        self,
        vp_angle: float | None,
        left_intercept: int | None,
        right_intercept: int | None,
        frame_width: int,
    ) -> tuple[float, str]:
        """Return steering angle and active state according to 3-stage logic."""
        center = self._center
        lo = center - self._max_offset
        hi = center + self._max_offset

        # Stage 1: Vision Lost (Gapping)
        if vp_angle is None or left_intercept is None or right_intercept is None:
            self._tracking_active = False
            self._last_error = 0.0
            return center, "GAPPING"

        # Stage 3: Danger Zone override (bypass PD)
        left_margin = self._danger_margin
        right_margin = max(0, int(frame_width) - self._danger_margin)
        if left_intercept > left_margin:
            self._tracking_active = False
            self._last_error = 0.0
            return center + self._nudge_deg, "DANGER_RIGHT"
        if right_intercept < right_margin:
            self._tracking_active = False
            self._last_error = 0.0
            return center - self._nudge_deg, "DANGER_LEFT"

        # Stage 2: Tracking with hysteresis
        error = float(vp_angle) - 90.0
        abs_error = abs(error)
        if abs_error <= self._inner_thresh:
            self._tracking_active = False
            self._last_error = 0.0
            return center, "TRACKING_COAST"

        if abs_error > self._outer_thresh:
            self._tracking_active = True

        if not self._tracking_active:
            return center, "TRACKING_COAST"

        pd_correction = self._apply_pd(error)
        steering_angle = max(lo, min(hi, center + pd_correction))
        return steering_angle, "TRACKING_PD"

    def _apply_pd(self, error: float) -> float:
        """Apply proportional-derivative smoothing and return steering correction."""
        derivative = error - self._last_error
        self._last_error = error
        return (self._pid.kp * error) + (self._pid.kd * derivative)

    # ------------------------------------------------------------------ #
    # Runtime parameter API (used by dashboard /control/params)
    # ------------------------------------------------------------------ #
    PARAM_BOUNDS: dict[str, tuple[float, float]] = {
        "kp": (0.0, 5.0),
        "ki": (0.0, 2.0),
        "kd": (0.0, 5.0),
        "danger_margin": (0.0, 400.0),
        "nudge_deg": (0.0, 45.0),
        "inner_thresh": (0.0, 30.0),
        "outer_thresh": (0.0, 60.0),
        "max_offset": (0.0, 90.0),
    }

    def get_params(self) -> dict[str, float]:
        return {
            "kp": float(self._pid.kp),
            "ki": float(self._pid.ki),
            "kd": float(self._pid.kd),
            "danger_margin": float(self._danger_margin),
            "nudge_deg": float(self._nudge_deg),
            "inner_thresh": float(self._inner_thresh),
            "outer_thresh": float(self._outer_thresh),
            "max_offset": float(self._max_offset),
        }

    def update_params(self, patch: dict[str, float]) -> dict[str, float]:
        """Apply a partial update; clamps every value to PARAM_BOUNDS. Returns new params."""
        if not isinstance(patch, dict):
            raise ValueError("params patch must be a JSON object")
        for key, raw in patch.items():
            if key not in self.PARAM_BOUNDS:
                raise ValueError(f"unknown param: {key}")
            try:
                value = float(raw)
            except (TypeError, ValueError) as exc:
                raise ValueError(f"param {key} must be numeric: {exc}") from exc
            lo, hi = self.PARAM_BOUNDS[key]
            value = max(lo, min(hi, value))
            if key == "kp":
                self._pid.kp = value
            elif key == "ki":
                self._pid.ki = value
            elif key == "kd":
                self._pid.kd = value
            elif key == "danger_margin":
                self._danger_margin = max(0, int(value))
            elif key == "nudge_deg":
                self._nudge_deg = value
            elif key == "inner_thresh":
                self._inner_thresh = abs(value)
            elif key == "outer_thresh":
                self._outer_thresh = abs(value)
            elif key == "max_offset":
                self._max_offset = abs(value)
        # keep outer_thresh >= inner_thresh after a partial update
        if self._outer_thresh < self._inner_thresh:
            self._outer_thresh = self._inner_thresh
        return self.get_params()
