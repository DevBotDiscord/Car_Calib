"""Vision module: heading-error detection via camera frames.

Pipeline per frame:
1. ROI masking – discard the top 60 % of the frame (keep bottom 40 %).
2. Pre-processing – CLAHE equalisation followed by a 5×5 Gaussian blur.
3. Edge extraction – Canny edge detection.
4. Line detection – Probabilistic Progressive Hough Transform (PPHT).
5. Angle calculation – ``e = |mean_theta - 90°|``.

Returns ``None`` when no lines are detected.
"""

import logging
import math
from typing import Optional

import cv2
import numpy as np

logger = logging.getLogger(__name__)

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
_HOUGH_THRESHOLD = 50         # minimum number of votes
_HOUGH_MIN_LINE_LEN = 30      # minimum line length in pixels
_HOUGH_MAX_LINE_GAP = 10      # maximum gap between line segments


class HeadingDetector:
    """Detects robot heading error from a camera frame.

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

    def _apply_roi(self, frame: np.ndarray) -> np.ndarray:
        """Crop the frame, keeping only the bottom *roi_keep_fraction*.

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
        blurred = cv2.GaussianBlur(equalized, _BLUR_KERNEL, 0)
        return blurred

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
    def _lines_to_angle(lines: np.ndarray) -> float:
        """Compute the mean orientation angle of detected line segments.

        Each segment is converted to an angle in [0°, 180°) using
        ``atan2(dy, dx)``, then the mean is taken.

        Args:
            lines: Array of line segments shaped ``(N, 1, 4)``.

        Returns:
            Mean angle in degrees in the range [0°, 180°).
        """
        angles: list[float] = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle_rad = math.atan2(float(y2 - y1), float(x2 - x1))
            angle_deg = math.degrees(angle_rad) % 180.0
            angles.append(angle_deg)
        return float(np.mean(angles))

    def compute_heading_error(self, frame: np.ndarray) -> Optional[float]:
        """Run the full detection pipeline and return the heading error *e*.

        ``e = |theta_robot - 90°|``

        Args:
            frame: Full BGR camera frame.

        Returns:
            Heading error ``e`` in degrees, or ``None`` if no lines are
            found.
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

        theta = self._lines_to_angle(lines)
        error = abs(theta - 90.0)
        logger.debug("Detected angle=%.2f°, heading_error=%.2f°", theta, error)
        return error
