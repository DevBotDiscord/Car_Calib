"""Vision module: tile-gap line detection for the heading-hold system.

Pipeline per frame:
1. ROI masking   – discard the top 60 % of the frame (keep bottom 40 %).
2. Pre-processing – CLAHE equalisation followed by a 5×5 Gaussian blur.
3. Edge extraction – Canny edge detection.
4. Line detection  – Probabilistic Progressive Hough Transform (PPHT).
5. Line grouping   – cluster segments with |Δθ| < 3° and close midpoints.
6. Reference select – pick the group that is lowest in the image (nearest
   the robot) and compute its length-weighted average angle.
7. Sanity check    – discard angles that shift by more than 20° in one frame.

Returns the angle θ (degrees, relative to the x-axis) of the reference
tile-gap line, or ``None`` if no valid line is found.
"""

import logging
import math
from typing import Optional

import cv2
import numpy as np

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
_ANGLE_THRESHOLD: float = 3.0   # degrees – maximum Δθ to merge two segments
_MIDPOINT_THRESHOLD: float = 50.0  # pixels  – maximum midpoint distance to merge

# Sanity check: discard frames where the angle shifts more than this amount
_SANITY_MAX_DELTA: float = 20.0  # degrees


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


class LineProcessor:
    """Detects the reference tile-gap line angle from a camera frame.

    Args:
        roi_keep_fraction: Fraction of frame height to *keep* (from the
            bottom).  Default ``0.4`` discards the top 60 %.
    """

    def __init__(self, roi_keep_fraction: float = 0.4) -> None:
        self._roi_keep_fraction = roi_keep_fraction
        self._clahe = cv2.createCLAHE(
            clipLimit=_CLAHE_CLIP_LIMIT,
            tileGridSize=_CLAHE_TILE_GRID,
        )
        self._last_angle: Optional[float] = None

    # ---------------------------------------------------------------------- #
    # Private pipeline helpers
    # ---------------------------------------------------------------------- #

    def _apply_roi(self, frame: np.ndarray) -> np.ndarray:
        """Return only the bottom *roi_keep_fraction* of *frame*.

        Args:
            frame: Input BGR or grayscale frame.

        Returns:
            Cropped frame (bottom portion only).
        """
        height = frame.shape[0]
        start_row = int(height * (1.0 - self._roi_keep_fraction))
        return frame[start_row:, :]

    def _preprocess(self, roi: np.ndarray) -> np.ndarray:
        """Apply CLAHE and Gaussian blur to a grayscale ROI.

        Args:
            roi: Grayscale ROI image.

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
    ) -> tuple[float, float, float, float, float]:
        """Compute (angle_deg, length, mid_x, mid_y) for a segment.

        Args:
            x1, y1, x2, y2: Segment endpoints.

        Returns:
            Tuple of ``(angle_deg, length, mid_x, mid_y)``.
        """
        angle_deg = math.degrees(math.atan2(float(y2 - y1), float(x2 - x1))) % 180.0
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

        θ_avg = Σ(θ_i · length_i) / Σ(length_i)

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
        """Return the highest y midpoint in *group* (lowest in image).

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
        """Pick the reference group: the one lowest in the image.

        The lowest group (highest y-midpoint) represents the tile gap most
        recently passed by the robot and is the most relevant for control.

        Args:
            groups: Non-empty list of segment groups from :meth:`_group_lines`.

        Returns:
            Length-weighted average angle of the winning group in degrees.
        """
        best_group = max(groups, key=self._group_max_y)
        return self._weighted_angle(best_group)

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

        Args:
            frame: Full BGR or grayscale camera frame.

        Returns:
            Angle θ in degrees relative to the x-axis (range [0°, 180°)),
            or ``None`` if no valid tile-gap line is found.
        """
        # Convert to grayscale if needed
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
            "Reference angle θ=%.2f° (groups=%d)", theta, len(groups)
        )
        return theta
