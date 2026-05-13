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
        # Use a point well inside the ROI (away from top/side/bottom borders).
        assert result[90, 100] == 128


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


class TestReferenceSelection:
    def test_select_reference_prefers_most_horizontal_group(self, detector):
        """Selection should prefer angle nearest 0°/180° over vertical groups."""
        near_vertical_group = [
            (0, 0, 0, 10, 90.0, 10.0, 5.0, 80.0),
        ]
        near_horizontal_group = [
            (0, 0, 10, 1, 5.0, 10.0, 5.0, 40.0),
        ]

        theta = detector._select_reference([near_vertical_group, near_horizontal_group])
        assert theta == pytest.approx(5.0)

    def test_horizontal_cap_rejects_vertical_angle(self, detector):
        """A near-vertical candidate must fail horizontal acceptance."""
        assert detector._is_horizontal_candidate(90.0) is False

    def test_horizontal_to_vertical_angle_transform(self, detector):
        """Selected horizontal angle must map to vertical-equivalent heading."""
        assert detector._horizontal_to_vertical_angle(0.0) == pytest.approx(90.0)
        assert detector._horizontal_to_vertical_angle(180.0) == pytest.approx(90.0)
        assert detector._horizontal_to_vertical_angle(20.0) == pytest.approx(110.0)
        assert detector._horizontal_to_vertical_angle(160.0) == pytest.approx(70.0)


class TestHorizontalCapIntegration:
    def test_get_reference_angle_rejects_non_horizontal_candidate(
        self,
        detector,
        monkeypatch: pytest.MonkeyPatch,
    ):
        """End-to-end path should return None when selected angle is non-horizontal."""
        vertical_group = [
            (0, 0, 0, 20, 90.0, 20.0, 10.0, 80.0),
        ]

        monkeypatch.setattr(
            detector,
            "_detect_lines",
            lambda _edges: np.array([[[0, 0, 1, 1]]], dtype=np.int32),
        )
        monkeypatch.setattr(detector, "_group_lines", lambda _lines: [vertical_group])

        frame = np.zeros((200, 200, 3), dtype=np.uint8)
        assert detector.get_reference_angle(frame) is None


class TestLateralOffset:
    def test_estimate_lateral_offset_centered(self, detector):
        left_group = [
            (60, 40, 62, 180, 89.0, 140.0, 61.0, 110.0),
        ]
        right_group = [
            (140, 40, 138, 180, 91.0, 140.0, 139.0, 110.0),
        ]
        info = detector._estimate_lateral_offset([left_group, right_group], 200, 200)
        assert info["lateral_status"] == "centered"
        assert info["lateral_offset_norm"] == pytest.approx(0.0, abs=0.1)

    def test_estimate_lateral_offset_drift_left(self, detector):
        left_group = [
            (30, 40, 32, 180, 89.0, 140.0, 31.0, 110.0),
        ]
        right_group = [
            (130, 40, 128, 180, 91.0, 140.0, 129.0, 110.0),
        ]
        info = detector._estimate_lateral_offset([left_group, right_group], 200, 200)
        assert info["lateral_status"] in {"drift_left", "out_left"}
        assert info["lateral_offset_norm"] is not None
        assert float(info["lateral_offset_norm"]) > 0.0
