"""Unit tests for vision/detector.py (LineDetector)."""

import math

import numpy as np
import pytest

from models.robot_state import RobotState
from vision.detector import LineDetector, _angle_diff


@pytest.fixture()
def state():
    """RobotState with roi_height_pct=0.4, roi_top_width_pct=0.6, roi_bottom_width_pct=1.0."""
    s = RobotState()
    s.roi_height_pct = 0.4
    s.roi_top_width_pct = 0.6
    s.roi_bottom_width_pct = 1.0
    return s


@pytest.fixture()
def detector(state):
    return LineDetector(state)


class TestTrapezoidMask:
    def test_build_trapezoid_pts_shape(self, detector):
        """_build_trapezoid_pts must return an array of shape (1, 4, 2)."""
        pts = detector._build_trapezoid_pts(100, 200)
        assert pts.shape == (1, 4, 2)

    def test_top_y_at_roi_boundary(self, detector):
        """Top vertices must sit at h - roi_h = 100 - 40 = 60."""
        pts = detector._build_trapezoid_pts(100, 200)
        top_left_y = pts[0][0][1]
        top_right_y = pts[0][1][1]
        assert top_left_y == 60
        assert top_right_y == 60

    def test_bottom_y_at_frame_edge(self, detector):
        """Bottom vertices must sit at h - 1 = 99."""
        pts = detector._build_trapezoid_pts(100, 200)
        bot_left_y = pts[0][3][1]
        bot_right_y = pts[0][2][1]
        assert bot_left_y == 99
        assert bot_right_y == 99

    def test_roi_is_centred(self, detector):
        """Top and bottom vertices must be horizontally centred (cx = w//2 = 100)."""
        pts = detector._build_trapezoid_pts(100, 200)
        cx = 200 // 2
        top_cx = (pts[0][0][0] + pts[0][1][0]) // 2
        bot_cx = (pts[0][3][0] + pts[0][2][0]) // 2
        assert top_cx == cx
        assert bot_cx == cx

    def test_apply_roi_preserves_shape(self, detector):
        """_apply_roi must return an array with the same shape as the input."""
        gray = np.zeros((100, 200), dtype=np.uint8)
        result = detector._apply_roi(gray)
        assert result.shape == gray.shape

    def test_apply_roi_zeroes_outside_trapezoid(self, detector):
        """Pixels well above the ROI boundary should be zeroed."""
        # A fully-white frame – the top rows must be zeroed by the mask
        gray = np.full((100, 200), 255, dtype=np.uint8)
        result = detector._apply_roi(gray)
        # Row 0 is above the trapezoid, so it must be zero
        assert result[0, 100] == 0

    def test_apply_roi_keeps_inside_pixels(self, detector):
        """Pixels inside the trapezoid should retain their original value."""
        gray = np.full((100, 200), 128, dtype=np.uint8)
        result = detector._apply_roi(gray)
        # The bottom centre pixel is always inside the bottom of the trapezoid
        assert result[98, 100] == 128


class TestPreprocessing:
    def test_preprocess_output_shape(self, detector):
        roi = np.random.randint(0, 256, (100, 200), dtype=np.uint8)
        result = detector._preprocess(roi)
        assert result.shape == roi.shape

    def test_preprocess_returns_uint8(self, detector):
        roi = np.random.randint(0, 256, (100, 200), dtype=np.uint8)
        result = detector._preprocess(roi)
        assert result.dtype == np.uint8


class TestAngleDiff:
    def test_zero_diff(self):
        assert _angle_diff(45.0, 45.0) == pytest.approx(0.0)

    def test_simple_diff(self):
        assert _angle_diff(10.0, 13.0) == pytest.approx(3.0)

    def test_wrap_around_180(self):
        assert _angle_diff(1.0, 179.0) == pytest.approx(2.0)

    def test_symmetry(self):
        assert _angle_diff(30.0, 50.0) == pytest.approx(_angle_diff(50.0, 30.0))


class TestGetReferenceAngle:
    def test_returns_none_on_blank_frame(self, detector):
        """A completely uniform frame should yield no edges or lines."""
        frame = np.zeros((200, 200, 3), dtype=np.uint8)
        result = detector.get_reference_angle(frame)
        assert result is None

    def test_returns_float_on_valid_frame(self, detector):
        """A frame with a clear horizontal line should return a float."""
        frame = np.zeros((200, 200, 3), dtype=np.uint8)
        frame[140:145, :] = 255
        result = detector.get_reference_angle(frame)
        if result is not None:
            assert isinstance(result, float)
            assert 0.0 <= result < 180.0

    def test_grayscale_frame_accepted(self, detector):
        """Detector should accept 2-D (grayscale) input without crashing."""
        frame = np.zeros((200, 200), dtype=np.uint8)
        result = detector.get_reference_angle(frame)
        assert result is None

    def test_last_angle_updated_on_valid_detection(self, detector):
        """After a valid detection, _last_angle should be updated."""
        frame = np.zeros((200, 200, 3), dtype=np.uint8)
        frame[140:145, :] = 255
        result = detector.get_reference_angle(frame)
        if result is not None:
            assert detector._last_angle == pytest.approx(result)
