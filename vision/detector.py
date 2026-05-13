"""Vision module: tile-gap line detection for the heading-hold system.

Pipeline per frame:
1. ROI masking   – apply a trapezoidal mask centred on the lower region,
   defined by percentages stored in :class:`~models.robot_state.RobotState`.
2. Pre-processing – CLAHE equalisation followed by a 5×5 Gaussian blur.
3. Edge extraction – Canny edge detection.
4. Line detection  – Probabilistic Progressive Hough Transform (PPHT).
5. Angle calculation – ``θ = atan2(y2-y1, x2-x1) × 180/π``; a line
   parallel to the robot's forward path gives ``θ ≈ 90°`` (error ``e = 0``).
6. Line grouping   – cluster segments with configurable |Δθ| threshold.
7. Reference select – pick the most horizontal group (angle nearest 0°/180°).
8. Sanity check    – discard angles that shift by more than 20° in one frame.

Returns the angle θ (degrees, relative to the x-axis, in ``[0°, 180°)``) of
the reference tile-gap line, or ``None`` if no valid line is found.
"""

import logging
import math
from typing import Any, Optional

import cv2
import numpy as np

from models.robot_state import RobotState
from config.settings import (
    VISION_ANGLE_THRESHOLD_DEG,
    VISION_BLUR_KERNEL_H,
    VISION_BLUR_KERNEL_W,
    VISION_CANNY_HIGH,
    VISION_CANNY_LOW,
    VISION_CLAHE_CLIP_LIMIT,
    VISION_CLAHE_TILE_GRID_H,
    VISION_CLAHE_TILE_GRID_W,
    VISION_DEBUG_MASK_FILE,
    VISION_HORIZONTAL_MAX_ERROR_DEG,
    VISION_HOUGH_MAX_LINE_GAP,
    VISION_HOUGH_MIN_LINE_LEN,
    VISION_HOUGH_RHO,
    VISION_HOUGH_THETA_DEG,
    VISION_HOUGH_THRESHOLD,
    VISION_CLUSTER_ANGLE_BIAS_DEG,
    VISION_CLUSTER_RHO_BIAS_PX,
    VISION_MIDPOINT_THRESHOLD_PX,
    VISION_ROI_BORDER_BLACK_PX,
    VISION_ROI_BOTTOM_CLEAR_ROWS,
    VISION_ROI_EDGE_MARGIN_PX,
    VISION_SANITY_MAX_DELTA_DEG,
    VISION_MIN_GROUP_TOTAL_LENGTH_PX,
    VISION_LATERAL_PROBE_Y_PCT,
    VISION_VERTICAL_GROUP_MIN_ANGLE_DEG,
    VISION_VERTICAL_GROUP_MAX_ANGLE_DEG,
    VISION_LATERAL_MIN_TILE_WIDTH_PCT,
    VISION_LATERAL_MAX_TILE_WIDTH_PCT,
)

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------#
# Tunable constants
# ---------------------------------------------------------------------------#

# CLAHE parameters
_CLAHE_CLIP_LIMIT = VISION_CLAHE_CLIP_LIMIT
_CLAHE_TILE_GRID = (VISION_CLAHE_TILE_GRID_W, VISION_CLAHE_TILE_GRID_H)

# Gaussian blur kernel size (must be odd)
_BLUR_KERNEL = (VISION_BLUR_KERNEL_W, VISION_BLUR_KERNEL_H)

# Canny thresholds
_CANNY_LOW = VISION_CANNY_LOW
_CANNY_HIGH = VISION_CANNY_HIGH

# PPHT parameters
_HOUGH_RHO = VISION_HOUGH_RHO
_HOUGH_THETA = math.radians(VISION_HOUGH_THETA_DEG)
_HOUGH_THRESHOLD = VISION_HOUGH_THRESHOLD
_HOUGH_MIN_LINE_LEN = VISION_HOUGH_MIN_LINE_LEN
_HOUGH_MAX_LINE_GAP = VISION_HOUGH_MAX_LINE_GAP

# Grouping thresholds
_ANGLE_THRESHOLD: float = VISION_ANGLE_THRESHOLD_DEG
_MIDPOINT_THRESHOLD: float = VISION_MIDPOINT_THRESHOLD_PX
_CLUSTER_ANGLE_BIAS_DEG: float = VISION_CLUSTER_ANGLE_BIAS_DEG
_CLUSTER_RHO_BIAS_PX: float = VISION_CLUSTER_RHO_BIAS_PX

# Minimum combined segment length for a group to be considered valid.
_MIN_GROUP_TOTAL_LENGTH_PX: float = VISION_MIN_GROUP_TOTAL_LENGTH_PX

# Sanity check: discard frames where the angle shifts more than this amount
_SANITY_MAX_DELTA: float = VISION_SANITY_MAX_DELTA_DEG

# Hard cap for horizontal-line acceptance. If the selected line group angle is
# farther than this from 0°/180°, it is rejected.
_HORIZONTAL_MAX_ERROR_DEG: float = VISION_HORIZONTAL_MAX_ERROR_DEG

# Keep ROI boundary black so the mask edge is never detected as a line.
_ROI_BORDER_BLACK_PX: int = VISION_ROI_BORDER_BLACK_PX

# Keep a safety margin from ROI borders in edge space to avoid Canny response
# caused by the inside/outside intensity jump at the ROI boundary.
_ROI_EDGE_MARGIN_PX: int = VISION_ROI_EDGE_MARGIN_PX

# Clear a small bottom band in the edge map; Canny can place boundary response
# a couple of rows above the true image edge after blur.
_ROI_BOTTOM_CLEAR_ROWS: int = VISION_ROI_BOTTOM_CLEAR_ROWS

# Debug output filename
_DEBUG_MASK_FILE = VISION_DEBUG_MASK_FILE
_LATERAL_PROBE_Y_PCT = VISION_LATERAL_PROBE_Y_PCT
_VERTICAL_GROUP_MIN_ANGLE_DEG = VISION_VERTICAL_GROUP_MIN_ANGLE_DEG
_VERTICAL_GROUP_MAX_ANGLE_DEG = VISION_VERTICAL_GROUP_MAX_ANGLE_DEG
_LATERAL_MIN_TILE_WIDTH_PCT = VISION_LATERAL_MIN_TILE_WIDTH_PCT
_LATERAL_MAX_TILE_WIDTH_PCT = VISION_LATERAL_MAX_TILE_WIDTH_PCT
_LATERAL_MAX_STALE_WIDTH_FRAMES = 30


def _angle_diff(a: float, b: float) -> float:
    """Minimum angular difference in [0°, 90°] (handles the 0°/180° wrap).

    Args:
        a: First angle in degrees.
        b: Second angle in degrees.

    Returns:
        Absolute angular difference in degrees, clamped to [0°, 90°].
    """
    diff = abs(a - b) % 180.0
    return min(diff, 180.0 - diff)


class LineDetector:
    """Detects the reference tile-gap line angle from a camera frame.

    Uses a trapezoidal ROI mask whose geometry is defined by the percentages
    stored in the shared :class:`~models.robot_state.RobotState` instance.

    Args:
        state: Shared :class:`~models.robot_state.RobotState` instance
            supplying ROI parameters and the ``debug_mode`` flag.
    """

    def __init__(self, state: RobotState) -> None:
        self._state = state
        self._clahe = cv2.createCLAHE(
            clipLimit=_CLAHE_CLIP_LIMIT,
            tileGridSize=_CLAHE_TILE_GRID,
        )
        self._last_angle: Optional[float] = None
        self._last_lateral_offset_px: Optional[float] = None
        self._last_lateral_offset_norm: Optional[float] = None
        self._last_tile_width_px: Optional[float] = None
        self._last_tile_width_age: int = 0
        self._debug_saved: bool = False

    # ---------------------------------------------------------------------- #
    # Private pipeline helpers
    # ---------------------------------------------------------------------- #

    def _build_trapezoid_pts(self, h: int, w: int) -> np.ndarray:
        """Compute the 4 vertices of the trapezoidal ROI, centred horizontally.

        The trapezoid spans the bottom ``roi_height_pct`` of the frame.  Its
        top edge has width ``roi_top_width_pct × w`` and its bottom edge has
        width ``roi_bottom_width_pct × w``.

        Args:
            h: Frame height in pixels.
            w: Frame width in pixels.

        Returns:
            Array of shape ``(1, 4, 2)`` (int32) suitable for
            :func:`cv2.fillPoly`.
        """
        s = self._state
        roi_h = int(h * s.roi_height_pct)
        top_w = int(w * s.roi_top_width_pct)
        bot_w = int(w * s.roi_bottom_width_pct)
        cx = w // 2
        top_y = h - roi_h
        return np.array(
            [[
                [cx - top_w // 2, top_y],
                [cx + top_w // 2, top_y],
                [cx + bot_w // 2, h - 1],
                [cx - bot_w // 2, h - 1],
            ]],
            dtype=np.int32,
        )

    @staticmethod
    def _strip_roi_border_hits(mask: np.ndarray, roi_start_row: int) -> None:
        """Remove residual bright ROI-border pixels from a binary ROI mask.

        Steps:
        1. For each column, clear the first non-zero pixel encountered from
           ``roi_start_row`` downward.
        2. For each ROI row, clear the first and last non-zero pixels to remove
           side-border remnants.
        3. Clear the final image row to force-remove the ROI bottom border.

        Args:
            mask: Binary uint8 mask modified in place.
            roi_start_row: First row included in ROI processing.
        """
        h, w = mask.shape[:2]
        start_row = max(0, min(int(roi_start_row), h - 1))

        # Vertical pass: remove top-most white hit in each column.
        roi_view = mask[start_row:h, :]
        for x in range(w):
            ys = np.flatnonzero(roi_view[:, x] != 0)
            if ys.size > 0:
                mask[start_row + int(ys[0]), x] = 0

        # Horizontal pass: remove left-most and right-most white hits per row.
        for y in range(start_row, h):
            xs = np.flatnonzero(mask[y, :] != 0)
            if xs.size == 0:
                continue
            left_x = int(xs[0])
            right_x = int(xs[-1])
            mask[y, left_x] = 0
            if right_x != left_x:
                mask[y, right_x] = 0

        # Ensure the ROI bottom edge is fully removed.
        mask[h - 1, :] = 0

    def _apply_roi(self, frame: np.ndarray) -> np.ndarray:
        """Apply a trapezoidal mask to *frame*.

        Pixels outside the trapezoid are zeroed.  On the first call when
        ``state.debug_mode`` is ``True``, the binary mask is saved to
        ``debug_mask.jpg`` for visual inspection.

        Args:
            frame: Grayscale input image (2-D).

        Returns:
            Masked image of the same shape as *frame* (zero outside trapezoid).
        """
        h, w = frame.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        pts = self._build_trapezoid_pts(h, w)
        cv2.fillPoly(mask, [pts[0]], 255)
        cv2.polylines(
            mask,
            [pts[0]],
            isClosed=True,
            color=0,
            thickness=_ROI_BORDER_BLACK_PX,
        )
        roi_start_row = int(np.min(pts[0][:, 1]))
        self._strip_roi_border_hits(mask, roi_start_row)

        if self._state.debug_mode and not self._debug_saved:
            cv2.imwrite(_DEBUG_MASK_FILE, mask)
            self._debug_saved = True
            logger.info("Debug mask saved to %s", _DEBUG_MASK_FILE)

        return cv2.bitwise_and(frame, frame, mask=mask)

    def _preprocess(self, roi: np.ndarray) -> np.ndarray:
        """Apply CLAHE and Gaussian blur to a grayscale ROI.

        Args:
            roi: Grayscale image (may contain zero-padded areas outside mask).

        Returns:
            Blurred, contrast-enhanced grayscale image.
        """
        equalized = self._clahe.apply(roi)
        return cv2.GaussianBlur(equalized, _BLUR_KERNEL, 0)

    def _detect_edges(self, preprocessed: np.ndarray) -> np.ndarray:
        """Run Canny edge detection.

        Args:
            preprocessed: Pre-processed grayscale image.

        Returns:
            Binary edge map.
        """
        edges = cv2.Canny(preprocessed, _CANNY_LOW, _CANNY_HIGH)

        # Remove ROI boundary responses by keeping only an inner (eroded) ROI.
        h, w = edges.shape[:2]
        pts = self._build_trapezoid_pts(h, w)
        inner_roi_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(inner_roi_mask, [pts[0]], 255)
        if _ROI_EDGE_MARGIN_PX > 0:
            k = (2 * _ROI_EDGE_MARGIN_PX) + 1
            kernel = np.ones((k, k), dtype=np.uint8)
            inner_roi_mask = cv2.erode(inner_roi_mask, kernel, iterations=1)

        filtered = cv2.bitwise_and(edges, edges, mask=inner_roi_mask)
        if _ROI_BOTTOM_CLEAR_ROWS > 0:
            rows = min(_ROI_BOTTOM_CLEAR_ROWS, filtered.shape[0])
            filtered[-rows:, :] = 0
        return filtered

    def _detect_lines(self, edges: np.ndarray) -> Optional[np.ndarray]:
        """Run Probabilistic Hough Transform (PPHT) to find line segments.

        Args:
            edges: Binary edge map from Canny.

        Returns:
            Array of line segments shaped ``(N, 1, 4)`` or ``None``.
        """
        return cv2.HoughLinesP(
            edges,
            _HOUGH_RHO,
            _HOUGH_THETA,
            _HOUGH_THRESHOLD,
            minLineLength=_HOUGH_MIN_LINE_LEN,
            maxLineGap=_HOUGH_MAX_LINE_GAP,
        )

    @staticmethod
    def _segment_props(
        x1: int, y1: int, x2: int, y2: int
    ) -> tuple[float, float, float, float]:
        """Compute ``(angle_deg, length, mid_x, mid_y)`` for a segment.

        ``angle_deg = atan2(y2-y1, x2-x1) × 180/π`` (mod 180, range [0°, 180°)).
        A segment parallel to the robot's forward path (vertical in the image)
        gives ``angle_deg ≈ 90°``, yielding error ``e = angle_deg - 90° = 0``.

        Args:
            x1, y1, x2, y2: Segment endpoints.

        Returns:
            Tuple of ``(angle_deg, length, mid_x, mid_y)``.
        """
        angle_deg = math.degrees(math.atan2(float(y2 - y1), float(x2 - x1))) % 180.0
        # atan2 returns values in (-180°, 180°]; applying % 180.0 maps the
        # result to [0°, 180°) regardless of sign (Python modulo convention).
        length = math.hypot(x2 - x1, y2 - y1)
        mid_x = (x1 + x2) / 2.0
        mid_y = (y1 + y2) / 2.0
        return angle_deg, length, mid_x, mid_y

    def _group_lines(
        self, lines: np.ndarray
    ) -> list[list[tuple[int, int, int, int, float, float, float, float]]]:
        """Group segments by rho/theta proximity (normal form), PPHT-friendly.

        Each segment is represented as
        ``(x1, y1, x2, y2, angle_deg, length, mid_x, mid_y)``.

        Args:
            lines: Array of shape ``(N, 1, 4)`` from PPHT.

        Returns:
            List of groups; each group is a list of segment tuples.
        """
        segments = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle, length, mid_x, mid_y = self._segment_props(x1, y1, x2, y2)
            segments.append((x1, y1, x2, y2, angle, length, mid_x, mid_y))

        # Build Hough-like (rho, theta) descriptors from each segment so we can
        # cluster similarly to the classic Hough-lines approach.
        polar: list[tuple[float, float]] = []
        for seg in segments:
            angle = seg[4]
            mid_x = seg[6]
            mid_y = seg[7]
            theta_normal = (angle + 90.0) % 180.0
            theta_rad = math.radians(theta_normal)
            rho = (mid_x * math.cos(theta_rad)) + (mid_y * math.sin(theta_rad))
            polar.append((rho, theta_normal))

        assigned = [False] * len(segments)
        groups: list[list] = []

        for i, seg_i in enumerate(segments):
            if assigned[i]:
                continue
            group = [seg_i]
            assigned[i] = True
            for j in range(i + 1, len(segments)):
                if assigned[j]:
                    continue
                rho_i, theta_i = polar[i]
                rho_j, theta_j = polar[j]
                theta_diff = _angle_diff(theta_i, theta_j)
                rho_diff = abs(rho_i - rho_j)
                if theta_diff < _CLUSTER_ANGLE_BIAS_DEG and rho_diff < _CLUSTER_RHO_BIAS_PX:
                    seg_j = segments[j]
                    group.append(seg_j)
                    assigned[j] = True
            groups.append(group)

        return groups

    @staticmethod
    def _weighted_angle(
        group: list[tuple[int, int, int, int, float, float, float, float]],
    ) -> float:
        """Return the length-weighted average angle for *group*.

        ``θ_avg = Σ(θ_i · length_i) / Σ(length_i)``

        Args:
            group: List of segment tuples (x1, y1, x2, y2, angle, length, ...).

        Returns:
            Weighted average angle in degrees.
        """
        total_length = sum(seg[5] for seg in group)
        if total_length == 0.0:
            logger.warning(
                "All segments in group have zero length; using first segment angle."
            )
            return group[0][4]
        return sum(seg[4] * seg[5] for seg in group) / total_length

    @staticmethod
    def _group_fit_angle(
        group: list[tuple[int, int, int, int, float, float, float, float]],
    ) -> float:
        """Estimate group angle via line fitting to reduce segment-level jitter."""
        if len(group) == 1:
            return group[0][4]

        pts: list[tuple[float, float]] = []
        for seg in group:
            pts.append((float(seg[0]), float(seg[1])))
            pts.append((float(seg[2]), float(seg[3])))

        if len(pts) < 2:
            return group[0][4]

        arr = np.array(pts, dtype=np.float32).reshape((-1, 1, 2))
        line = cv2.fitLine(arr, cv2.DIST_L2, 0, 0.01, 0.01)
        vx_f = float(line[0][0])
        vy_f = float(line[1][0])
        if abs(vx_f) < 1e-9 and abs(vy_f) < 1e-9:
            return LineDetector._weighted_angle(group)

        return math.degrees(math.atan2(vy_f, vx_f)) % 180.0

    @staticmethod
    def _group_max_y(
        group: list[tuple[int, int, int, int, float, float, float, float]],
    ) -> float:
        """Return the highest y midpoint in *group* (lowest in image, nearest robot).

        Args:
            group: List of segment tuples.

        Returns:
            Maximum midpoint-y value across all segments in the group.
        """
        return max(seg[7] for seg in group)

    @staticmethod
    def _group_total_length(
        group: list[tuple[int, int, int, int, float, float, float, float]],
    ) -> float:
        """Return total segment length in pixels for *group*."""
        return sum(seg[5] for seg in group)

    def _group_horizontal_error(
        self,
        group: list[tuple[int, int, int, int, float, float, float, float]],
    ) -> float:
        """Return distance (degrees) from perfect horizontal (0°/180°)."""
        angle = self._group_fit_angle(group)
        return _angle_diff(angle, 0.0)

    @staticmethod
    def _group_points(
        group: list[tuple[int, int, int, int, float, float, float, float]],
    ) -> np.ndarray:
        pts: list[tuple[float, float]] = []
        for seg in group:
            pts.append((float(seg[0]), float(seg[1])))
            pts.append((float(seg[2]), float(seg[3])))
        return np.array(pts, dtype=np.float32).reshape((-1, 1, 2))

    @staticmethod
    def _fit_group_line(
        group: list[tuple[int, int, int, int, float, float, float, float]],
    ) -> tuple[float, float, float, float] | None:
        if not group:
            return None
        pts = LineDetector._group_points(group)
        if pts.shape[0] < 2:
            return None
        line = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
        vx = float(line[0][0])
        vy = float(line[1][0])
        x0 = float(line[2][0])
        y0 = float(line[3][0])
        if abs(vx) < 1e-9 and abs(vy) < 1e-9:
            return None
        return vx, vy, x0, y0

    @staticmethod
    def _line_x_at_y(
        line_params: tuple[float, float, float, float],
        y_query: float,
    ) -> float | None:
        vx, vy, x0, y0 = line_params
        if abs(vy) < 1e-9:
            return None
        t = (y_query - y0) / vy
        return x0 + (t * vx)

    @staticmethod
    def _is_vertical_candidate(angle: float) -> bool:
        return _VERTICAL_GROUP_MIN_ANGLE_DEG <= angle <= _VERTICAL_GROUP_MAX_ANGLE_DEG

    def _estimate_lateral_offset(
        self,
        groups: list[list[tuple[int, int, int, int, float, float, float, float]]],
        frame_width: int,
        frame_height: int,
    ) -> dict[str, float | int | str | None]:
        probe_y = float(max(0, min(frame_height - 1, int(frame_height * _LATERAL_PROBE_Y_PCT))))
        frame_center_x = frame_width / 2.0
        min_tile_width = frame_width * _LATERAL_MIN_TILE_WIDTH_PCT
        max_tile_width = frame_width * _LATERAL_MAX_TILE_WIDTH_PCT

        left_x: float | None = None
        right_x: float | None = None
        left_idx: int | None = None
        right_idx: int | None = None

        for idx, group in enumerate(groups):
            angle = self._group_fit_angle(group)
            if not self._is_vertical_candidate(angle):
                continue
            line_params = self._fit_group_line(group)
            if line_params is None:
                continue
            x_probe = self._line_x_at_y(line_params, probe_y)
            if x_probe is None or x_probe < 0.0 or x_probe > float(frame_width):
                continue

            if x_probe < frame_center_x and (left_x is None or x_probe > left_x):
                left_x = x_probe
                left_idx = idx
            if x_probe > frame_center_x and (right_x is None or x_probe < right_x):
                right_x = x_probe
                right_idx = idx

        tile_center_x: float | None = None
        offset_px: float | None = None
        offset_norm: float | None = None
        tile_width_px: float | None = None
        status = "unknown"

        if left_x is not None and right_x is not None:
            tile_width_px = right_x - left_x
            if min_tile_width <= tile_width_px <= max_tile_width:
                tile_center_x = (left_x + right_x) / 2.0
                offset_px = frame_center_x - tile_center_x
                half_width = max(1.0, tile_width_px / 2.0)
                offset_norm = max(-2.0, min(2.0, offset_px / half_width))
                abs_norm = abs(offset_norm)
                if abs_norm < 0.25:
                    status = "centered"
                elif abs_norm < 0.60:
                    status = "drift_right" if offset_norm > 0.0 else "drift_left"
                else:
                    status = "out_right" if offset_norm > 0.0 else "out_left"
                self._last_tile_width_px = tile_width_px
                self._last_tile_width_age = 0
            else:
                status = "invalid_tile_width"
        else:
            self._last_tile_width_age += 1

        can_use_stale_width = (
            self._last_tile_width_px is not None
            and self._last_tile_width_age <= _LATERAL_MAX_STALE_WIDTH_FRAMES
        )
        if status == "unknown" and can_use_stale_width:
            stale_width = float(self._last_tile_width_px)
            if left_x is not None and right_x is None:
                right_x = left_x + stale_width
                right_idx = -1
            elif right_x is not None and left_x is None:
                left_x = right_x - stale_width
                left_idx = -1

            if left_x is not None and right_x is not None:
                tile_width_px = right_x - left_x
                if tile_width_px > 1.0:
                    tile_center_x = (left_x + right_x) / 2.0
                    offset_px = frame_center_x - tile_center_x
                    half_width = max(1.0, tile_width_px / 2.0)
                    offset_norm = max(-2.0, min(2.0, offset_px / half_width))
                    abs_norm = abs(offset_norm)
                    if abs_norm < 0.25:
                        status = "centered_inferred"
                    elif abs_norm < 0.60:
                        status = "drift_right_inferred" if offset_norm > 0.0 else "drift_left_inferred"
                    else:
                        status = "out_right_inferred" if offset_norm > 0.0 else "out_left_inferred"

        return {
            "lateral_probe_y": probe_y,
            "lateral_left_x": left_x,
            "lateral_right_x": right_x,
            "lateral_left_group_index": left_idx,
            "lateral_right_group_index": right_idx,
            "lateral_tile_width_px": tile_width_px,
            "lateral_tile_center_x": tile_center_x,
            "lateral_offset_px": offset_px,
            "lateral_offset_norm": offset_norm,
            "lateral_status": status,
        }

    @staticmethod
    def _is_horizontal_candidate(angle: float) -> bool:
        """Return True when *angle* is within the horizontal acceptance cap."""
        return _angle_diff(angle, 0.0) <= _HORIZONTAL_MAX_ERROR_DEG

    @staticmethod
    def _horizontal_to_vertical_angle(horizontal_angle: float) -> float:
        """Map a horizontal-line angle to its vertical-direction equivalent.

        Examples:
        - 0° / 180° -> 90°
        - 20° -> 110°
        - 160° -> 70°
        """
        return (horizontal_angle + 90.0) % 180.0

    def _select_reference_group_index(
        self,
        groups: list[list[tuple[int, int, int, int, float, float, float, float]]],
    ) -> int:
        """Pick the most horizontal group; break ties by nearest robot."""
        return min(
            range(len(groups)),
            key=lambda idx: (
                self._group_horizontal_error(groups[idx]),
                -self._group_max_y(groups[idx]),
            ),
        )

    def _select_reference(
        self,
        groups: list[list[tuple[int, int, int, int, float, float, float, float]]],
    ) -> float:
        """Pick the reference group: the one most horizontal.

        Args:
            groups: Non-empty list of segment groups from :meth:`_group_lines`.

        Returns:
            Length-weighted average angle of the winning group in degrees.
        """
        best_group = groups[self._select_reference_group_index(groups)]
        return self._group_fit_angle(best_group)

    @staticmethod
    def _group_bounding_rect(
        group: list[tuple[int, int, int, int, float, float, float, float]],
    ) -> tuple[int, int, int, int]:
        """Return an axis-aligned bounding rectangle for all segments in *group*."""
        pts: list[tuple[int, int]] = []
        for seg in group:
            pts.append((int(seg[0]), int(seg[1])))
            pts.append((int(seg[2]), int(seg[3])))

        arr = np.array(pts, dtype=np.int32).reshape((-1, 1, 2))
        x, y, w, h = cv2.boundingRect(arr)
        return int(x), int(y), int(w), int(h)

    @staticmethod
    def _draw_raw_lines(
        canvas: np.ndarray,
        lines: Optional[np.ndarray],
    ) -> np.ndarray:
        """Draw raw Hough segments on *canvas* for debug visualisation."""
        vis = canvas.copy()
        if lines is None:
            return vis
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(vis, (x1, y1), (x2, y2), (0, 0, 255), 2)
        return vis

    @staticmethod
    def _draw_grouped_lines(
        canvas: np.ndarray,
        groups: list[list[tuple[int, int, int, int, float, float, float, float]]],
        reference_idx: Optional[int],
    ) -> np.ndarray:
        """Draw grouped segments with a distinct color per group."""
        vis = canvas.copy()
        palette = [
            (255, 120, 0),
            (0, 220, 220),
            (220, 0, 220),
            (0, 200, 80),
            (255, 200, 0),
            (180, 120, 255),
            (255, 255, 255),
        ]

        for idx, group in enumerate(groups):
            color = palette[idx % len(palette)]
            thickness = 4 if reference_idx == idx else 2
            for seg in group:
                x1, y1, x2, y2 = seg[0], seg[1], seg[2], seg[3]
                cv2.line(vis, (x1, y1), (x2, y2), color, thickness)
        return vis

    def _sanity_check(self, angle: float) -> bool:
        """Return ``True`` if *angle* passes the inter-frame sanity check.

        Discards any angle that would require a steering change of more than
        20° compared to the previous valid detection.

        Args:
            angle: Candidate angle in degrees.

        Returns:
            ``True`` if the angle is acceptable; ``False`` if it should be
            discarded.
        """
        if self._last_angle is None:
            return True
        delta = _angle_diff(angle, self._last_angle)
        if delta > _SANITY_MAX_DELTA:
            logger.warning(
                "Sanity check failed: Δθ=%.2f° > %.1f° (new=%.2f°, last=%.2f°)",
                delta,
                _SANITY_MAX_DELTA,
                angle,
                self._last_angle,
            )
            return False
        return True

    # ---------------------------------------------------------------------- #
    # Public API
    # ---------------------------------------------------------------------- #

    def get_reference_angle(self, frame: np.ndarray) -> Optional[float]:
        """Run the full pipeline and return the reference tile-gap angle θ.

        A line parallel to the robot's forward path gives ``θ ≈ 90°`` so that
        the heading error ``e = θ - 90° = 0``.

        Args:
            frame: Full BGR or grayscale camera frame.

        Returns:
            Angle θ in degrees relative to the x-axis (range ``[0°, 180°)``),
            or ``None`` if no valid tile-gap line is found.
        """
        if frame.ndim == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame

        roi = self._apply_roi(gray)
        preprocessed = self._preprocess(roi)
        edges = self._detect_edges(preprocessed)
        lines = self._detect_lines(edges)

        if lines is None:
            self._last_lateral_offset_px = None
            self._last_lateral_offset_norm = None
            logger.debug("No lines detected in frame.")
            return None

        groups = self._group_lines(lines)
        if not groups:
            self._last_lateral_offset_px = None
            self._last_lateral_offset_norm = None
            logger.debug("No line groups formed.")
            return None

        lateral_info = self._estimate_lateral_offset(groups, gray.shape[1], gray.shape[0])
        self._last_lateral_offset_px = lateral_info["lateral_offset_px"]
        self._last_lateral_offset_norm = lateral_info["lateral_offset_norm"]

        horizontal_groups = [
            group
            for group in groups
            if self._is_horizontal_candidate(self._group_fit_angle(group))
            and self._group_total_length(group) >= _MIN_GROUP_TOTAL_LENGTH_PX
        ]
        if not horizontal_groups:
            logger.debug(
                "No horizontal candidate groups found (max error=%.1f°).",
                _HORIZONTAL_MAX_ERROR_DEG,
            )
            return None

        theta_horizontal = self._select_reference(horizontal_groups)

        if not self._is_horizontal_candidate(theta_horizontal):
            logger.debug(
                "Rejected non-horizontal candidate θ=%.2f° (max error=%.1f°).",
                theta_horizontal,
                _HORIZONTAL_MAX_ERROR_DEG,
            )
            return None

        theta = self._horizontal_to_vertical_angle(theta_horizontal)

        if not self._sanity_check(theta):
            return None

        self._last_angle = theta
        logger.debug(
            "Reference angle θ=%.2f°  error=%.2f°  (groups=%d)",
            theta,
            theta - 90.0,
            len(groups),
        )
        return theta

    def get_reference_angle_debug(
        self,
        frame: np.ndarray,
    ) -> tuple[Optional[float], dict[str, Any]]:
        """Run full pipeline and return angle plus detailed debug artefacts.

        Returns a tuple ``(theta, debug_data)`` where ``theta`` follows the
        same semantics as :meth:`get_reference_angle`, and ``debug_data``
        contains intermediate images and stage metadata.
        """
        if frame.ndim == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            base = frame.copy()
        else:
            gray = frame
            base = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        roi = self._apply_roi(gray)
        preprocessed = self._preprocess(roi)
        edges = self._detect_edges(preprocessed)
        lines = self._detect_lines(edges)

        groups: list[list[tuple[int, int, int, int, float, float, float, float]]] = []
        reference_idx: Optional[int] = None
        theta_candidate: Optional[float] = None
        theta_horizontal: Optional[float] = None
        horizontal_ok = False
        sanity_ok = False
        stale_output = False
        theta_out: Optional[float] = None

        lateral_info: dict[str, float | int | str | None] = {
            "lateral_probe_y": None,
            "lateral_left_x": None,
            "lateral_right_x": None,
            "lateral_left_group_index": None,
            "lateral_right_group_index": None,
            "lateral_tile_width_px": None,
            "lateral_tile_center_x": None,
            "lateral_offset_px": None,
            "lateral_offset_norm": None,
            "lateral_status": "unknown",
        }

        if lines is not None:
            groups = self._group_lines(lines)
            lateral_info = self._estimate_lateral_offset(groups, gray.shape[1], gray.shape[0])
            self._last_lateral_offset_px = lateral_info["lateral_offset_px"]
            self._last_lateral_offset_norm = lateral_info["lateral_offset_norm"]
            if groups:
                horizontal_candidate_indices = [
                    idx
                    for idx, group in enumerate(groups)
                    if self._is_horizontal_candidate(self._group_fit_angle(group))
                    and self._group_total_length(group) >= _MIN_GROUP_TOTAL_LENGTH_PX
                ]
                horizontal_ok = len(horizontal_candidate_indices) > 0
                if horizontal_ok:
                    reference_idx = min(
                        horizontal_candidate_indices,
                        key=lambda idx: (
                            self._group_horizontal_error(groups[idx]),
                            -self._group_max_y(groups[idx]),
                        ),
                    )
                    theta_horizontal = self._group_fit_angle(groups[reference_idx])
                    theta_candidate = self._horizontal_to_vertical_angle(theta_horizontal)
                    sanity_ok = self._sanity_check(theta_candidate)
                    if sanity_ok:
                        self._last_angle = theta_candidate
                        theta_out = theta_candidate
                    else:
                        stale_output = self._last_angle is not None
                else:
                    stale_output = self._last_angle is not None
            else:
                stale_output = self._last_angle is not None
        elif self._last_angle is not None:
            stale_output = True
        else:
            self._last_lateral_offset_px = None
            self._last_lateral_offset_norm = None

        selected_group_bbox: Optional[tuple[int, int, int, int]] = None
        if reference_idx is not None:
            selected_group_bbox = self._group_bounding_rect(groups[reference_idx])

        hough_vis = self._draw_raw_lines(base, lines)
        grouped_vis = self._draw_grouped_lines(base, groups, reference_idx)
        if selected_group_bbox is not None:
            x, y, w, h = selected_group_bbox
            cv2.rectangle(grouped_vis, (x, y), (x + w, y + h), (255, 80, 80), 2)
            cv2.putText(
                grouped_vis,
                "Selected horizontal group",
                (x, max(18, y - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 80, 80),
                2,
            )
        elif stale_output:
            cv2.putText(
                grouped_vis,
                "STALE: no current selected group",
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (80, 80, 255),
                2,
            )

        probe_y = lateral_info["lateral_probe_y"]
        if probe_y is not None:
            y_probe_i = int(probe_y)
            cv2.line(grouped_vis, (0, y_probe_i), (grouped_vis.shape[1] - 1, y_probe_i), (80, 80, 80), 1)
        left_x = lateral_info["lateral_left_x"]
        right_x = lateral_info["lateral_right_x"]
        tile_center_x = lateral_info["lateral_tile_center_x"]
        if left_x is not None:
            x_i = int(left_x)
            cv2.line(grouped_vis, (x_i, 0), (x_i, grouped_vis.shape[0] - 1), (0, 160, 255), 1)
        if right_x is not None:
            x_i = int(right_x)
            cv2.line(grouped_vis, (x_i, 0), (x_i, grouped_vis.shape[0] - 1), (0, 160, 255), 1)
        if tile_center_x is not None:
            cx_i = int(tile_center_x)
            cv2.line(grouped_vis, (cx_i, 0), (cx_i, grouped_vis.shape[0] - 1), (80, 255, 120), 1)

        debug_data: dict[str, Any] = {
            "gray": gray,
            "roi": roi,
            "preprocessed": preprocessed,
            "edges": edges,
            "hough_vis": hough_vis,
            "grouped_vis": grouped_vis,
            "lines_count": 0 if lines is None else int(len(lines)),
            "groups_count": len(groups),
            "reference_group_index": reference_idx,
            "selected_group_bbox": selected_group_bbox,
            "theta_horizontal": theta_horizontal,
            "theta_candidate": theta_candidate,
            "horizontal_ok": horizontal_ok,
            "sanity_ok": sanity_ok,
            "theta_output": theta_out,
            "stale_output": stale_output,
            "horizontal_max_error": _HORIZONTAL_MAX_ERROR_DEG,
            "sanity_max_delta": _SANITY_MAX_DELTA,
            "angle_threshold": _ANGLE_THRESHOLD,
            "cluster_angle_bias_deg": _CLUSTER_ANGLE_BIAS_DEG,
            "cluster_rho_bias_px": _CLUSTER_RHO_BIAS_PX,
            "midpoint_threshold": _MIDPOINT_THRESHOLD,
            "min_group_total_length_px": _MIN_GROUP_TOTAL_LENGTH_PX,
            "hough_threshold": _HOUGH_THRESHOLD,
            "hough_min_line_len": _HOUGH_MIN_LINE_LEN,
            "hough_max_line_gap": _HOUGH_MAX_LINE_GAP,
            **lateral_info,
        }
        return theta_out, debug_data

    @property
    def last_lateral_offset_px(self) -> Optional[float]:
        return self._last_lateral_offset_px

    @property
    def last_lateral_offset_norm(self) -> Optional[float]:
        return self._last_lateral_offset_norm
