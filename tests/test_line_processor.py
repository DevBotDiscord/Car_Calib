"""Unit tests for vision/line_processor.py."""

import math

import numpy as np
import pytest

from vision.line_processor import LineProcessor, _angle_diff


@pytest.fixture()
def processor():
    return LineProcessor(roi_keep_fraction=0.4)


class TestAngleDiff:
    def test_zero_diff(self):
        assert _angle_diff(45.0, 45.0) == pytest.approx(0.0)

    def test_simple_diff(self):
        assert _angle_diff(10.0, 13.0) == pytest.approx(3.0)

    def test_wrap_around_180(self):
        """Angles near 0° and near 180° should be close (mod-180 wrap)."""
        assert _angle_diff(1.0, 179.0) == pytest.approx(2.0)

    def test_symmetry(self):
        assert _angle_diff(30.0, 50.0) == pytest.approx(_angle_diff(50.0, 30.0))


class TestROIMasking:
    def test_roi_keeps_bottom_fraction(self, processor):
        """Bottom 40 % of a 100-row frame should be rows 60–99."""
        frame = np.zeros((100, 200, 3), dtype=np.uint8)
        frame[:60, :] = 255
        frame[60:, :] = 0
        gray = frame[:, :, 0]
        roi = processor._apply_roi(gray)
        assert roi.shape[0] == 40
        assert roi.max() == 0

    def test_roi_start_row(self, processor):
        gray = np.arange(100, dtype=np.uint8).reshape(100, 1)
        roi = processor._apply_roi(gray)
        assert roi[0, 0] == 60


class TestSegmentProps:
    def test_horizontal_segment(self):
        angle, length, mid_x, mid_y = LineProcessor._segment_props(0, 50, 100, 50)
        assert angle == pytest.approx(0.0)
        assert length == pytest.approx(100.0)
        assert mid_x == pytest.approx(50.0)
        assert mid_y == pytest.approx(50.0)

    def test_length_calculation(self):
        _, length, _, _ = LineProcessor._segment_props(0, 0, 3, 4)
        assert length == pytest.approx(5.0)


class TestGroupLines:
    def test_similar_angles_are_grouped(self, processor):
        """Two near-horizontal lines with close midpoints should form one group."""
        lines = np.array([
            [[0, 50, 100, 50]],   # 0°
            [[0, 55, 100, 55]],   # 0° (very close)
        ])
        groups = processor._group_lines(lines)
        # Both should be in the same group (Δθ=0° and midpoints ~5 px apart)
        assert len(groups) == 1
        assert len(groups[0]) == 2

    def test_different_angles_form_separate_groups(self, processor):
        """Lines with >3° angle difference should form separate groups."""
        lines = np.array([
            [[0, 50, 100, 50]],         # ~0°  (horizontal)
            [[50, 0, 50, 100]],         # ~90° (vertical)
        ])
        groups = processor._group_lines(lines)
        assert len(groups) == 2

    def test_far_apart_midpoints_not_merged(self, processor):
        """Same angle but distant midpoints should form separate groups."""
        # Both lines are horizontal, but midpoints are 200 px apart
        lines = np.array([
            [[0, 10, 100, 10]],    # midpoint y=10
            [[0, 210, 100, 210]],  # midpoint y=210
        ])
        groups = processor._group_lines(lines)
        assert len(groups) == 2


class TestWeightedAngle:
    def test_equal_lengths_give_mean(self):
        group = [
            (0, 0, 10, 0, 10.0, 10.0, 5.0, 0.0),   # angle=10, length=10
            (0, 0, 10, 0, 20.0, 10.0, 5.0, 0.0),   # angle=20, length=10
        ]
        result = LineProcessor._weighted_angle(group)
        assert result == pytest.approx(15.0)

    def test_longer_segment_dominates(self):
        group = [
            (0, 0, 10, 0, 10.0, 10.0, 5.0, 0.0),   # angle=10, length=10
            (0, 0, 10, 0, 40.0, 100.0, 5.0, 0.0),  # angle=40, length=100
        ]
        result = LineProcessor._weighted_angle(group)
        # Weighted: (10*10 + 40*100) / 110 ≈ 37.27
        assert result == pytest.approx((10 * 10 + 40 * 100) / 110, rel=1e-4)

    def test_zero_total_length(self):
        """Zero-length segments fall back to the first segment's angle."""
        group = [(0, 0, 0, 0, 45.0, 0.0, 0.0, 0.0)]
        result = LineProcessor._weighted_angle(group)
        assert result == pytest.approx(45.0)


class TestSelectReference:
    def test_lowest_group_selected(self, processor):
        """Group with higher y-midpoint (lower in image) should win."""
        group_high = [(0, 10, 100, 10, 0.0, 100.0, 50.0, 10.0)]   # y=10
        group_low = [(0, 100, 100, 100, 45.0, 100.0, 50.0, 100.0)]  # y=100
        result = processor._select_reference([group_high, group_low])
        assert result == pytest.approx(45.0)


class TestSanityCheck:
    def test_first_angle_always_accepted(self, processor):
        assert processor._sanity_check(45.0) is True

    def test_small_delta_accepted(self, processor):
        processor._last_angle = 45.0
        assert processor._sanity_check(50.0) is True

    def test_large_delta_rejected(self, processor):
        processor._last_angle = 45.0
        assert processor._sanity_check(90.0) is False

    def test_exactly_20_degrees_accepted(self, processor):
        processor._last_angle = 45.0
        # |65 - 45| = 20, not > 20, so accepted
        assert processor._sanity_check(65.0) is True

    def test_just_over_20_degrees_rejected(self, processor):
        processor._last_angle = 45.0
        assert processor._sanity_check(65.1) is False


class TestGetReferenceAngle:
    def test_returns_none_on_blank_frame(self, processor):
        """A uniform frame should have no edges, no lines, and return None."""
        frame = np.zeros((200, 200, 3), dtype=np.uint8)
        assert processor.get_reference_angle(frame) is None

    def test_returns_float_on_valid_frame(self, processor):
        """A frame with a clear line should return a float angle."""
        frame = np.zeros((200, 200, 3), dtype=np.uint8)
        frame[140:145, :] = 255  # horizontal line in bottom 40 %
        result = processor.get_reference_angle(frame)
        if result is not None:
            assert isinstance(result, float)
            assert 0.0 <= result < 180.0

    def test_last_angle_updated_on_valid_detection(self, processor):
        """After a valid detection, _last_angle should be set."""
        frame = np.zeros((200, 200, 3), dtype=np.uint8)
        frame[140:145, :] = 255
        result = processor.get_reference_angle(frame)
        if result is not None:
            assert processor._last_angle == pytest.approx(result)

    def test_accepts_grayscale_frame(self, processor):
        """Detector should accept 2-D (grayscale) frames."""
        frame = np.zeros((200, 200), dtype=np.uint8)
        result = processor.get_reference_angle(frame)
        assert result is None  # blank → no lines
