"""Unit tests for vision/detector.py (LineDetector) — V2 vanishing-point pipeline."""

import numpy as np
import pytest

from models.robot_state import RobotState
from vision.detector import LineDetector, _angle_diff


@pytest.fixture()
def state():
    s = RobotState()
    s.roi_height_pct = 0.4
    return s


@pytest.fixture()
def detector(state):
    return LineDetector(state)


class TestAngleDiff:
    def test_zero_diff(self):
        assert _angle_diff(45.0, 45.0) == pytest.approx(0.0)

    def test_simple_diff(self):
        assert _angle_diff(10.0, 13.0) == pytest.approx(3.0)

    def test_wrap_around_180(self):
        assert _angle_diff(1.0, 179.0) == pytest.approx(2.0)

    def test_symmetry(self):
        assert _angle_diff(30.0, 50.0) == pytest.approx(_angle_diff(50.0, 30.0))


class TestSelectOppositeSlopes:
    def test_returns_none_when_no_negative_line(self, detector):
        """Only positive-slope lines present → no opposite pair."""
        lines = [(0, 0, 10, 10), (0, 0, 20, 30)]  # both positive slope
        assert detector._select_opposite_slopes(lines) is None

    def test_returns_none_when_no_positive_line(self, detector):
        lines = [(0, 10, 10, 0), (0, 30, 20, 0)]  # both negative slope
        assert detector._select_opposite_slopes(lines) is None

    def test_filters_near_horizontal_lines(self, detector):
        """abs(slope) < VISION_MIN_ABS_SLOPE (0.3) is rejected."""
        lines = [(0, 0, 100, 5), (0, 5, 100, 0)]  # slope ±0.05, below threshold
        assert detector._select_opposite_slopes(lines) is None

    def test_picks_longest_per_sign(self, detector):
        """Longest negative + longest positive slope line are selected."""
        neg_short = (0, 10, 5, 0)      # slope -2, len ~11
        neg_long = (0, 100, 50, 0)     # slope -2, len ~111
        pos_short = (0, 0, 5, 10)      # slope +2, len ~11
        pos_long = (0, 0, 50, 100)     # slope +2, len ~111
        selected = detector._select_opposite_slopes(
            [neg_short, pos_short, neg_long, pos_long]
        )
        assert selected is not None
        best_neg, best_pos = selected
        assert best_neg == neg_long
        assert best_pos == pos_long

    def test_skips_vertical_lines(self, detector):
        """dx == 0 (vertical) is skipped without ZeroDivisionError."""
        lines = [(5, 0, 5, 100), (0, 10, 10, 0), (0, 0, 10, 10)]
        selected = detector._select_opposite_slopes(lines)
        assert selected is not None  # neg + pos pair still found


class TestIntersection:
    def test_crossing_lines_return_point(self, detector):
        line1 = (0, 0, 10, 10)   # y = x
        line2 = (0, 10, 10, 0)   # y = 10 - x
        vp = detector._intersection(line1, line2)
        assert vp == (5, 5)

    def test_parallel_lines_return_none(self, detector):
        line1 = (0, 0, 10, 10)
        line2 = (0, 5, 10, 15)  # same slope, parallel
        assert detector._intersection(line1, line2) is None


class TestXAtY:
    def test_interpolates_along_line(self, detector):
        line = (0, 0, 10, 100)  # x grows 0→10 as y grows 0→100
        assert detector._x_at_y(line, 50) == 5

    def test_horizontal_line_returns_x1(self, detector):
        line = (3, 20, 40, 20)  # y2 == y1
        assert detector._x_at_y(line, 99) == 3


class TestGetReferenceAngle:
    def test_returns_none_on_blank_frame(self, detector):
        frame = np.zeros((200, 200, 3), dtype=np.uint8)
        assert detector.get_reference_angle(frame) is None

    def test_grayscale_frame_accepted(self, detector):
        frame = np.zeros((200, 200), dtype=np.uint8)
        assert detector.get_reference_angle(frame) is None

    def test_returns_float_in_range_on_valid_frame(self, detector):
        """Two opposite-slope lanes converging → VP-derived theta in [0,180)."""
        frame = np.zeros((200, 200, 3), dtype=np.uint8)
        # Left lane (positive slope downward-left) + right lane (negative slope)
        import cv2

        cv2.line(frame, (40, 199), (90, 60), (255, 255, 255), 3)
        cv2.line(frame, (160, 199), (110, 60), (255, 255, 255), 3)
        result = detector.get_reference_angle(frame)
        if result is not None:
            assert isinstance(result, float)
            assert 0.0 <= result < 180.0

    def test_last_theta_updated_on_valid_detection(self, detector):
        import cv2

        frame = np.zeros((200, 200, 3), dtype=np.uint8)
        cv2.line(frame, (40, 199), (90, 60), (255, 255, 255), 3)
        cv2.line(frame, (160, 199), (110, 60), (255, 255, 255), 3)
        result = detector.get_reference_angle(frame)
        if result is not None:
            assert detector._last_theta == pytest.approx(result)


class TestDebugContract:
    def test_blank_frame_debug_has_v2_keys(self, detector):
        frame = np.zeros((200, 200, 3), dtype=np.uint8)
        theta, debug = detector.get_reference_angle_debug(frame)
        assert theta is None
        for key in (
            "selected_lines",
            "vp_x",
            "vp_y",
            "left_intercept",
            "right_intercept",
            "theta_output",
            "theta_horizontal",
            "lines_count",
            "groups_count",
        ):
            assert key in debug

    def test_blank_frame_debug_values_neutral(self, detector):
        frame = np.zeros((200, 200, 3), dtype=np.uint8)
        _theta, debug = detector.get_reference_angle_debug(frame)
        assert debug["selected_lines"] is None
        assert debug["vp_x"] is None
        assert debug["groups_count"] == 0
