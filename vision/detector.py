"""Lane reference angle detector utilities."""

from __future__ import annotations

from math import hypot
from typing import Any

import cv2
import numpy as np

from config.settings import (
    VISION_BLUR_KERNEL_H,
    VISION_BLUR_KERNEL_W,
    VISION_CANNY_HIGH,
    VISION_CANNY_LOW,
    VISION_HOUGH_MAX_LINE_GAP,
    VISION_HOUGH_MIN_LINE_LENGTH,
    VISION_HOUGH_THRESHOLD,
    VISION_MIN_ABS_SLOPE,
)
from models.robot_state import RobotState


def _angle_diff(a: float, b: float) -> float:
    """Return minimum angular difference in [0, 90] for [0,180) headings."""
    diff = abs((float(a) - float(b)) % 180.0)
    return min(diff, 180.0 - diff)


class LineDetector:
    """Detect lane-like line groups and produce reference heading angle."""

    def __init__(self, state: RobotState) -> None:
        self._state = state
        self._last_theta: float | None = None

    def get_reference_angle(self, frame: np.ndarray) -> float | None:
        """Run the pipeline and return a normalized heading angle or None."""
        theta, _ = self._process(frame)
        return theta

    def get_reference_angle_debug(
        self,
        frame: np.ndarray,
    ) -> tuple[float | None, dict[str, Any]]:
        """Run pipeline and return angle plus debug images/metadata."""
        return self._process(frame)

    def _process(self, frame: np.ndarray) -> tuple[float | None, dict[str, Any]]:
        if frame is None or frame.size == 0:
            return None, self._empty_debug()

        frame_h, frame_w = frame.shape[:2]
        roi_height = max(1, int(frame_h * max(0.0, min(1.0, self._state.roi_height_pct))))
        roi_bgr = frame[:roi_height, :]
        if roi_bgr.ndim == 2:
            gray = roi_bgr
            roi_bgr = cv2.cvtColor(roi_bgr, cv2.COLOR_GRAY2BGR)
        else:
            gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
        blur_w = VISION_BLUR_KERNEL_W if VISION_BLUR_KERNEL_W % 2 == 1 else VISION_BLUR_KERNEL_W + 1
        blur_h = VISION_BLUR_KERNEL_H if VISION_BLUR_KERNEL_H % 2 == 1 else VISION_BLUR_KERNEL_H + 1
        preprocessed = cv2.GaussianBlur(gray, (blur_w, blur_h), 0)
        edges = cv2.Canny(preprocessed, VISION_CANNY_LOW, VISION_CANNY_HIGH)

        raw_lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=VISION_HOUGH_THRESHOLD,
            minLineLength=VISION_HOUGH_MIN_LINE_LENGTH,
            maxLineGap=VISION_HOUGH_MAX_LINE_GAP,
        )

        lines = []
        if raw_lines is not None:
            lines = [tuple(map(int, item[0])) for item in raw_lines.tolist()]

        selected = self._select_opposite_slopes(lines)
        hough_vis = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        grouped_vis = roi_bgr.copy()
        for x1, y1, x2, y2 in lines:
            cv2.line(hough_vis, (x1, y1), (x2, y2), (80, 80, 255), 1)

        theta_output: float | None = None
        selected_group_bbox: tuple[int, int, int, int] | None = None
        reference_group_index: int | None = None
        theta_horizontal: float | None = None
        theta_candidate: float | None = None
        vp_x: int | None = None
        vp_y: int | None = None
        left_intercept: int | None = None
        right_intercept: int | None = None
        horizontal_ok = False
        sanity_ok = False

        if selected is not None:
            line_a, line_b = selected
            for idx, (x1, y1, x2, y2) in enumerate((line_a, line_b)):
                color = (0, 255, 0) if idx == 0 else (255, 0, 0)
                cv2.line(grouped_vis, (x1, y1), (x2, y2), color, 2)

            x_vals = [line_a[0], line_a[2], line_b[0], line_b[2]]
            y_vals = [line_a[1], line_a[3], line_b[1], line_b[3]]
            selected_group_bbox = (
                int(min(x_vals)),
                int(min(y_vals)),
                int(max(x_vals) - min(x_vals)),
                int(max(y_vals) - min(y_vals)),
            )
            reference_group_index = 0

            intercepts = [self._x_at_y(line_a, frame_h - 1), self._x_at_y(line_b, frame_h - 1)]
            if intercepts[0] is not None and intercepts[1] is not None:
                left_intercept, right_intercept = sorted((intercepts[0], intercepts[1]))
            vp = self._intersection(line_a, line_b)
            if vp is not None:
                vp_x, vp_y = vp
                cv2.circle(grouped_vis, (vp_x, vp_y), 4, (0, 255, 255), -1)
                theta_candidate = (180.0 / float(max(frame_w, 1))) * float(vp_x)
                theta_output = theta_candidate % 180.0
                theta_horizontal = theta_output
                horizontal_ok = _angle_diff(theta_output, 90.0) <= 90.0
                sanity_ok = 0.0 <= theta_output < 180.0
                self._last_theta = theta_output

        debug_data: dict[str, Any] = {
            "gray": gray,
            "roi": gray,
            "preprocessed": preprocessed,
            "edges": edges,
            "hough_vis": hough_vis,
            "grouped_vis": grouped_vis,
            "lines_count": len(lines),
            "groups_count": 2 if selected is not None else 0,
            "selected_lines": None if selected is None else [tuple(map(int, selected[0])), tuple(map(int, selected[1]))],
            "reference_group_index": reference_group_index,
            "selected_group_bbox": selected_group_bbox,
            "theta_horizontal": theta_horizontal,
            "theta_candidate": theta_candidate,
            "horizontal_ok": horizontal_ok,
            "sanity_ok": sanity_ok,
            "theta_output": theta_output,
            "vp_x": vp_x,
            "vp_y": vp_y,
            "left_intercept": left_intercept,
            "right_intercept": right_intercept,
            "raw_vp_angle": theta_output,
            "stale_output": False,
        }
        return theta_output, debug_data

    def _select_opposite_slopes(
        self,
        lines: list[tuple[int, int, int, int]],
    ) -> tuple[tuple[int, int, int, int], tuple[int, int, int, int]] | None:
        best_neg = None
        best_pos = None
        best_neg_len = 0.0
        best_pos_len = 0.0

        for x1, y1, x2, y2 in lines:
            dx = x2 - x1
            if dx == 0:
                continue
            slope = (y2 - y1) / dx
            if abs(slope) < VISION_MIN_ABS_SLOPE:
                continue
            length = hypot(dx, y2 - y1)
            if slope < 0 and length > best_neg_len:
                best_neg = (x1, y1, x2, y2)
                best_neg_len = length
            elif slope > 0 and length > best_pos_len:
                best_pos = (x1, y1, x2, y2)
                best_pos_len = length

        if best_neg is None or best_pos is None:
            return None
        return best_neg, best_pos

    @staticmethod
    def _intersection(
        line1: tuple[int, int, int, int],
        line2: tuple[int, int, int, int],
    ) -> tuple[int, int] | None:
        x1, y1, x2, y2 = line1
        x3, y3, x4, y4 = line2
        denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if denominator == 0:
            return None
        det1 = x1 * y2 - y1 * x2
        det2 = x3 * y4 - y3 * x4
        px = (det1 * (x3 - x4) - (x1 - x2) * det2) / denominator
        py = (det1 * (y3 - y4) - (y1 - y2) * det2) / denominator
        return int(round(px)), int(round(py))

    @staticmethod
    def _x_at_y(line: tuple[int, int, int, int], target_y: int) -> int | None:
        x1, y1, x2, y2 = line
        if y2 == y1:
            return int(x1)
        t = (float(target_y) - float(y1)) / float(y2 - y1)
        return int(round(float(x1) + t * float(x2 - x1)))

    @staticmethod
    def _empty_debug() -> dict[str, Any]:
        blank = np.zeros((1, 1), dtype=np.uint8)
        return {
            "gray": blank,
            "roi": blank,
            "preprocessed": blank,
            "edges": blank,
            "hough_vis": cv2.cvtColor(blank, cv2.COLOR_GRAY2BGR),
            "grouped_vis": cv2.cvtColor(blank, cv2.COLOR_GRAY2BGR),
            "lines_count": 0,
            "groups_count": 0,
            "reference_group_index": None,
            "selected_group_bbox": None,
            "theta_horizontal": None,
            "theta_candidate": None,
            "horizontal_ok": False,
            "sanity_ok": False,
            "theta_output": None,
            "vp_x": None,
            "vp_y": None,
            "left_intercept": None,
            "right_intercept": None,
            "raw_vp_angle": None,
            "selected_lines": None,
            "stale_output": False,
        }
