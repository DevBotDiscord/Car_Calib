"""Tests for unbounded VP and selected-line overlay diagnostics."""

from __future__ import annotations

import numpy as np

from runtime.overlay_drawer import OverlayDrawer


def test_offscreen_vp_draws_visible_boundary_indicator() -> None:
    frame = np.zeros((120, 200, 3), dtype=np.uint8)
    drawer = OverlayDrawer()

    drawer._draw_vp_indicator(frame, (100, -1000), "above")

    assert np.count_nonzero(frame) > 0
    assert np.count_nonzero(frame[:20]) > 0


def test_overlay_accepts_explicit_left_and_right_lines_with_unbounded_vp() -> None:
    frame = np.zeros((300, 600, 3), dtype=np.uint8)
    drawer = OverlayDrawer()

    output = drawer.draw(
        frame,
        {
            "state": "TRACKING_PD",
            "danger_boundary": "RIGHT",
            "recovery_direction": "LEFT",
            "danger_threshold_x": 540,
            "raw_vp_angle": 82.0,
            "vp_coord": (328, -1032),
            "vp_location": "above",
            "left_intercept_x": -250,
            "right_intercept_x": 449,
            "final_steering_cmd": 85.0,
            "lines": [(7, 252, 70, 0), (382, 1, 422, 299)],
            "left_line": (7, 252, 70, 0),
            "right_line": (382, 1, 422, 299),
        },
    )

    assert output.shape == frame.shape
    assert np.count_nonzero(output) > 0
