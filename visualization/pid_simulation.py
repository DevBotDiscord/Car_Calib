"""PID/servo kinematic simulation visualisation for process_video output."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class SimulationState:
    """Current simulation values used for rendering."""

    predicted_theta: float
    measured_heading: float
    measured_velocity: float
    servo_heading: float
    servo_rate: float


def _wrap_180(theta: float) -> float:
    return theta % 180.0


def _shortest_delta(a: float, b: float) -> float:
    """Return shortest angular delta a-b in range [-90, 90]."""
    diff = (a - b + 90.0) % 180.0 - 90.0
    return diff


def _theta_to_x(theta_deg: float, x0: int, x1: int) -> int:
    theta_clamped = max(0.0, min(180.0, theta_deg))
    width = x1 - x0
    return x0 + int((theta_clamped / 180.0) * width)


class PIDSimulationVisualizer:
    """Two-panel kinematic visualiser tied to video timeline."""

    def __init__(
        self,
        speed_deg_per_sec: float,
        servo_center_angle: float,
        max_steering_offset: float,
    ) -> None:
        self._speed_deg_per_sec = max(1e-6, float(speed_deg_per_sec))
        self._servo_center_angle = float(servo_center_angle)
        self._max_steering_offset = max(1e-6, float(max_steering_offset))

        self._initialized = False
        self._predicted_theta: float | None = None
        self._measured_heading: float | None = None
        self._servo_heading: float | None = None
        self._last_detected_theta: float | None = None
        self._last_detected_velocity: float = 0.0
        self._measured_velocity: float = 0.0
        self._servo_rate: float = 0.0

    def is_initialized(self) -> bool:
        return self._initialized

    def update(self, theta_detected: float | None, servo_angle: float, dt: float) -> SimulationState | None:
        """Advance simulation by one frame duration."""
        dt = max(1e-6, float(dt))

        if theta_detected is not None:
            if self._last_detected_theta is not None:
                delta = _shortest_delta(theta_detected, self._last_detected_theta)
                self._last_detected_velocity = delta / dt
            self._last_detected_theta = theta_detected
            self._predicted_theta = theta_detected

            if not self._initialized:
                self._measured_heading = theta_detected
                self._servo_heading = theta_detected
                self._initialized = True

        elif self._predicted_theta is not None:
            self._predicted_theta = _wrap_180(
                self._predicted_theta + (self._last_detected_velocity * dt)
            )

        if (
            not self._initialized
            or self._predicted_theta is None
            or self._measured_heading is None
            or self._servo_heading is None
        ):
            return None

        max_step = self._speed_deg_per_sec * dt
        heading_before = float(self._measured_heading)
        delta_to_target = _shortest_delta(self._predicted_theta, heading_before)
        step = max(-max_step, min(max_step, delta_to_target))
        self._measured_heading = _wrap_180(heading_before + step)
        self._measured_velocity = step / dt

        servo_offset = float(servo_angle) - self._servo_center_angle
        normalized = max(-1.0, min(1.0, servo_offset / self._max_steering_offset))
        self._servo_rate = normalized * self._speed_deg_per_sec
        self._servo_heading = _wrap_180(float(self._servo_heading) + (self._servo_rate * dt))

        return SimulationState(
            predicted_theta=float(self._predicted_theta),
            measured_heading=float(self._measured_heading),
            measured_velocity=float(self._measured_velocity),
            servo_heading=float(self._servo_heading),
            servo_rate=float(self._servo_rate),
        )

    def render(
        self,
        width: int,
        height: int,
        frame_num: int,
        theta_detected: float | None,
        servo_angle: float,
        fsm_state: str,
        state: SimulationState | None,
    ) -> np.ndarray:
        """Render side-by-side simulation panels."""
        panel = np.zeros((height, width, 3), dtype=np.uint8)
        panel[:] = (14, 18, 24)

        mid = width // 2
        left = panel[:, :mid]
        right = panel[:, mid:]

        self._draw_detected_panel(
            left,
            frame_num=frame_num,
            theta_detected=theta_detected,
            state=state,
        )
        self._draw_servo_panel(
            right,
            servo_angle=servo_angle,
            fsm_state=fsm_state,
            state=state,
        )

        cv2.line(panel, (mid, 8), (mid, height - 8), (60, 60, 60), 1)
        return panel

    def _draw_detected_panel(
        self,
        canvas: np.ndarray,
        frame_num: int,
        theta_detected: float | None,
        state: SimulationState | None,
    ) -> None:
        h, w = canvas.shape[:2]
        cv2.putText(canvas, "Detected + Kinematic Speed Model", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (240, 220, 120), 2)

        gauge_x0 = 12
        gauge_x1 = w - 14
        gauge_y0 = max(44, h // 2 - 14)
        gauge_y1 = gauge_y0 + 26
        cv2.rectangle(canvas, (gauge_x0, gauge_y0), (gauge_x1, gauge_y1), (70, 70, 70), 1)

        center_x = _theta_to_x(90.0, gauge_x0, gauge_x1)
        cv2.line(canvas, (center_x, gauge_y0 - 4), (center_x, gauge_y1 + 4), (255, 255, 255), 1)

        if state is not None:
            target_x = _theta_to_x(state.predicted_theta, gauge_x0, gauge_x1)
            sim_x = _theta_to_x(state.measured_heading, gauge_x0, gauge_x1)
            cv2.line(canvas, (target_x, gauge_y0 - 5), (target_x, gauge_y1 + 5), (0, 220, 220), 2)
            cv2.line(canvas, (sim_x, gauge_y0 - 8), (sim_x, gauge_y1 + 8), (60, 120, 255), 3)

        if theta_detected is not None:
            detected_x = _theta_to_x(theta_detected, gauge_x0, gauge_x1)
            cv2.line(canvas, (detected_x, gauge_y0 - 10), (detected_x, gauge_y1 + 10), (0, 255, 255), 1)

        cv2.putText(canvas, "0", (gauge_x0 - 2, gauge_y0 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (190, 190, 190), 1)
        cv2.putText(canvas, "90", (center_x - 12, gauge_y0 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)
        cv2.putText(canvas, "180", (gauge_x1 - 28, gauge_y0 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (190, 190, 190), 1)

        y = gauge_y1 + 34
        cv2.putText(canvas, f"Frame: {frame_num}", (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (230, 230, 230), 1)
        y += 24
        cv2.putText(canvas, f"Detected theta: {theta_detected if theta_detected is not None else 'None'}", (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (140, 230, 230), 1)
        y += 24
        if state is not None:
            cv2.putText(canvas, f"Predicted theta: {state.predicted_theta:.2f} deg", (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (100, 220, 220), 1)
            y += 24
            cv2.putText(canvas, f"Sim heading: {state.measured_heading:.2f} deg", (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (80, 160, 255), 1)
            y += 24
            cv2.putText(canvas, f"Sim velocity: {state.measured_velocity:+.2f} deg/s", (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (80, 160, 255), 1)
        else:
            cv2.putText(canvas, "Waiting for first detected theta...", (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (180, 180, 180), 1)

    def _draw_servo_panel(
        self,
        canvas: np.ndarray,
        servo_angle: float,
        fsm_state: str,
        state: SimulationState | None,
    ) -> None:
        h, w = canvas.shape[:2]
        cv2.putText(canvas, "Servo/PID Integrated Response", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (120, 240, 160), 2)

        gauge_x0 = 12
        gauge_x1 = w - 14
        gauge_y0 = max(44, h // 2 - 14)
        gauge_y1 = gauge_y0 + 26
        cv2.rectangle(canvas, (gauge_x0, gauge_y0), (gauge_x1, gauge_y1), (70, 70, 70), 1)

        center_x = _theta_to_x(90.0, gauge_x0, gauge_x1)
        cv2.line(canvas, (center_x, gauge_y0 - 4), (center_x, gauge_y1 + 4), (255, 255, 255), 1)

        if state is not None:
            sim_x = _theta_to_x(state.servo_heading, gauge_x0, gauge_x1)
            cv2.line(canvas, (sim_x, gauge_y0 - 8), (sim_x, gauge_y1 + 8), (80, 255, 120), 3)

        cv2.putText(canvas, "0", (gauge_x0 - 2, gauge_y0 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (190, 190, 190), 1)
        cv2.putText(canvas, "90", (center_x - 12, gauge_y0 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)
        cv2.putText(canvas, "180", (gauge_x1 - 28, gauge_y0 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (190, 190, 190), 1)

        y = gauge_y1 + 34
        cv2.putText(canvas, f"FSM state: {fsm_state}", (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (230, 230, 230), 1)
        y += 24
        cv2.putText(canvas, f"Servo angle: {servo_angle:.2f} deg", (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (160, 240, 160), 1)
        y += 24
        cv2.putText(canvas, f"Servo offset: {servo_angle - self._servo_center_angle:+.2f} deg", (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (160, 240, 160), 1)
        y += 24
        if state is not None:
            cv2.putText(canvas, f"Sim heading: {state.servo_heading:.2f} deg", (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (100, 255, 140), 1)
            y += 24
            cv2.putText(canvas, f"Integrated rate: {state.servo_rate:+.2f} deg/s", (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (100, 255, 140), 1)
        else:
            cv2.putText(canvas, "Waiting for first detected theta...", (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (180, 180, 180), 1)
