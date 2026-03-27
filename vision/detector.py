"""Vision module: tile-gap line detection for the heading-hold system.

Pipeline per frame:
1. ROI masking   – apply a trapezoidal mask centred on the lower region,
   defined by percentages stored in :class:`~models.robot_state.RobotState`.
2. Pre-processing – CLAHE equalisation followed by a 5×5 Gaussian blur.
3. Edge extraction – Canny edge detection.
4. Line detection  – Probabilistic Progressive Hough Transform (PPHT).
5. Angle calculation – ``θ = atan2(y2-y1, x2-x1) × 180/π``; a line
   parallel to the robot's forward path gives ``θ ≈ 90°`` (error ``e = 0``).
6. Line grouping   – cluster segments with |Δθ| < 3°.
7. Reference select – pick the group closest to the robot (highest y-mid).
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

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------#
# Tunable constants
# ---------------------------------------------------------------------------#

# CLAHE parameters
_CLAHE_CLIP_LIMIT = 2.0
_CLAHE_TILE_GRID = (8, 8)

# Gaussian blur kernel size (must be odd)
_BLUR_KERNEL = (5, 5)

# Canny thresholds
_CANNY_LOW = 50
_CANNY_HIGH = 150

# PPHT parameters
_HOUGH_RHO = 1                # distance resolution in pixels
_HOUGH_THETA = math.pi / 180  # angle resolution in radians
_HOUGH_THRESHOLD = 50         # minimum votes
_HOUGH_MIN_LINE_LEN = 30      # minimum segment length in pixels
_HOUGH_MAX_LINE_GAP = 10      # maximum gap between collinear segments

# Grouping thresholds
_ANGLE_THRESHOLD: float = 5.0    # degrees – maximum Δθ to merge two segments
_MIDPOINT_THRESHOLD: float = 30.0  # pixels – maximum midpoint distance to merge

# Sanity check: discard frames where the angle shifts more than this amount
_SANITY_MAX_DELTA: float = 40.0  # degrees

# Debug output filename
_DEBUG_MASK_FILE = "debug_mask.jpg"


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
        return cv2.Canny(preprocessed, _CANNY_LOW, _CANNY_HIGH)

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
        """Group segments with similar slopes (|Δθ| < 3°) and close midpoints.

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
                seg_j = segments[j]
                if (
                    _angle_diff(seg_i[4], seg_j[4]) < _ANGLE_THRESHOLD
                    and math.hypot(seg_i[6] - seg_j[6], seg_i[7] - seg_j[7])
                    < _MIDPOINT_THRESHOLD
                ):
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

    def _select_reference(
        self,
        groups: list[list[tuple[int, int, int, int, float, float, float, float]]],
    ) -> float:
        """Pick the reference group: the one lowest in the image (closest to robot).

        Args:
            groups: Non-empty list of segment groups from :meth:`_group_lines`.

        Returns:
            Length-weighted average angle of the winning group in degrees.
        """
        best_group = max(groups, key=self._group_max_y)
        return self._weighted_angle(best_group)

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
            logger.debug("No lines detected in frame.")
            return None

        groups = self._group_lines(lines)
        if not groups:
            logger.debug("No line groups formed.")
            return None

        theta = self._select_reference(groups)

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
        sanity_ok = False
        theta_out: Optional[float] = None

        if lines is not None:
            groups = self._group_lines(lines)
            if groups:
                reference_idx = max(range(len(groups)), key=lambda idx: self._group_max_y(groups[idx]))
                theta_candidate = self._weighted_angle(groups[reference_idx])
                sanity_ok = self._sanity_check(theta_candidate)
                if sanity_ok:
                    self._last_angle = theta_candidate
                    theta_out = theta_candidate

        hough_vis = self._draw_raw_lines(base, lines)
        grouped_vis = self._draw_grouped_lines(base, groups, reference_idx)

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
            "theta_candidate": theta_candidate,
            "sanity_ok": sanity_ok,
            "theta_output": theta_out,
            "sanity_max_delta": _SANITY_MAX_DELTA,
            "angle_threshold": _ANGLE_THRESHOLD,
            "midpoint_threshold": _MIDPOINT_THRESHOLD,
            "hough_threshold": _HOUGH_THRESHOLD,
            "hough_min_line_len": _HOUGH_MIN_LINE_LEN,
            "hough_max_line_gap": _HOUGH_MAX_LINE_GAP,
        }
        return theta_out, debug_data
