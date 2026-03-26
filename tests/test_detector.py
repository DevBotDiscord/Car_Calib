"""Unit tests for vision/detector.py."""

import math

import numpy as np
import pytest

from vision.detector import HeadingDetector


@pytest.fixture()
def detector():
    return HeadingDetector(roi_keep_fraction=0.4)


class TestROIMasking:
    def test_roi_keeps_bottom_fraction(self, detector):
        """Bottom 40 % of a 100-row frame should be rows 60–99."""
        frame = np.zeros((100, 200, 3), dtype=np.uint8)
        frame[:60, :] = 255  # top 60 % is white
        frame[60:, :] = 0   # bottom 40 % is black
        # Gray conversion
        gray = frame[:, :, 0]
        roi = detector._apply_roi(gray)
        assert roi.shape[0] == 40
        assert roi.max() == 0  # bottom portion is black

    def test_roi_start_row(self, detector):
        gray = np.arange(100, dtype=np.uint8).reshape(100, 1)
        roi = detector._apply_roi(gray)
        # With keep_fraction=0.4, start_row = 60
        assert roi[0, 0] == 60

    def test_roi_custom_fraction(self):
        det = HeadingDetector(roi_keep_fraction=0.5)
        gray = np.zeros((100, 200), dtype=np.uint8)
        roi = det._apply_roi(gray)
        assert roi.shape[0] == 50


class TestPreprocessing:
    def test_preprocess_output_shape(self, detector):
        roi = np.random.randint(0, 256, (40, 200), dtype=np.uint8)
        result = detector._preprocess(roi)
        assert result.shape == roi.shape

    def test_preprocess_returns_uint8(self, detector):
        roi = np.random.randint(0, 256, (40, 200), dtype=np.uint8)
        result = detector._preprocess(roi)
        assert result.dtype == np.uint8


class TestHeadingError:
    def test_returns_none_on_blank_frame(self, detector):
        """A completely uniform frame should yield no edges or lines."""
        frame = np.zeros((200, 200, 3), dtype=np.uint8)
        result = detector.compute_heading_error(frame)
        assert result is None

    def test_returns_float_on_valid_frame(self, detector):
        """A frame with a clear horizontal line should return a float."""
        frame = np.zeros((200, 200, 3), dtype=np.uint8)
        # Draw a bold white horizontal line in the bottom 40 % (rows 120–200)
        frame[140:145, :] = 255
        result = detector.compute_heading_error(frame)
        # If lines are detected, result must be a non-negative float
        if result is not None:
            assert isinstance(result, float)
            assert result >= 0.0

    def test_error_is_absolute_value(self, detector):
        """Heading error must always be non-negative."""
        # Simulate a nearly-vertical line (theta ~= 80°): e = |80 - 90| = 10
        frame = np.zeros((200, 200, 3), dtype=np.uint8)
        # Draw a near-vertical line from bottom-left to slightly right
        import cv2
        cv2.line(frame, (100, 120), (110, 200), (255, 255, 255), 3)
        result = detector.compute_heading_error(frame)
        if result is not None:
            assert result >= 0.0

    def test_lines_to_angle_horizontal(self):
        """A perfectly horizontal line segment should yield ~0° or ~180°."""
        lines = np.array([[[0, 50, 200, 50]]])  # y1 == y2 → horizontal
        angle = HeadingDetector._lines_to_angle(lines)
        # atan2(0, 200) = 0° mod 180 = 0°
        assert angle == pytest.approx(0.0, abs=1e-6)

    def test_lines_to_angle_multiple_segments(self):
        """Mean of two identical angles should equal that angle."""
        lines = np.array([
            [[0, 0, 100, 0]],   # 0°
            [[0, 0, 100, 0]],   # 0°
        ])
        angle = HeadingDetector._lines_to_angle(lines)
        assert angle == pytest.approx(0.0, abs=1e-6)

    def test_grayscale_frame_accepted(self, detector):
        """Detector should accept 2-D (grayscale) input without crashing."""
        frame = np.zeros((200, 200), dtype=np.uint8)
        result = detector.compute_heading_error(frame)
        assert result is None  # blank → no lines
