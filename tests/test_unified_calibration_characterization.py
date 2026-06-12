"""Characterization tests for the current unified calibration behavior.

These tests intentionally describe Phase 1 behavior. Later phases may update
them only when an approved phase deliberately changes the corresponding
contract.
"""

from __future__ import annotations

import csv
import io
import logging

import numpy as np
import pytest

from control.steering_controller import SteeringController
from models.robot_state import PIDConstants, RobotState
from unified_calibration_components import (
    GeometryCalculator,
    TelemetryLogger,
    UnifiedCalibrator,
    VisionProcessor,
)


class _DetectorStub:
    def __init__(self) -> None:
        self.calls = 0

    def get_reference_angle_debug(self, frame: np.ndarray):
        self.calls += 1
        return 90.0, {
            "theta_horizontal": 90.0,
            "reference_group_index": 0,
            "selected_group_bbox": (0, 0, 200, 100),
            "lines_count": 2,
            "groups_count": 2,
            "horizontal_ok": True,
            "sanity_ok": True,
            "stale_output": False,
            "selected_lines": [(0, 100, 100, 0), (100, 0, 200, 100)],
        }


class _VisionStub:
    def __init__(self) -> None:
        self.process_calls = 0
        self.filter_calls = 0

    def process_frame(self, frame: np.ndarray):
        self.process_calls += 1
        return [(0, 100, 100, 0), (100, 0, 200, 100)]

    def _apply_geometric_filter(self, lines):
        self.filter_calls += 1
        return lines[0], lines[1]


class _TelemetryStub:
    def __init__(self) -> None:
        self.logged: dict[str, object] | None = None

    def update_visuals(self, frame, telemetry_data, debug_data):
        return frame.copy()

    def log_state(self, frame_num, telemetry_data):
        self.logged = dict(telemetry_data)

    def write_video(self, frame):
        return None

    def publish_stream(self, frame, telemetry_data):
        return None


def _make_calibrator_for_characterization() -> UnifiedCalibrator:
    calibrator = UnifiedCalibrator.__new__(UnifiedCalibrator)
    calibrator._logger = logging.getLogger("test.unified")
    calibrator._robot_state = RobotState(
        pid=PIDConstants(kp=1.0, ki=0.05, kd=0.1),
        servo_center_angle=90.0,
        max_steering_offset=30.0,
    )
    calibrator._detector = _DetectorStub()
    calibrator._vision = _VisionStub()
    calibrator._geometry = GeometryCalculator()
    calibrator._steering = SteeringController(
        pid_constants=calibrator._robot_state.pid,
        danger_margin=0,
        nudge_deg=5.0,
        inner_thresh=3.0,
        outer_thresh=5.0,
        center_angle=90.0,
        max_offset=30.0,
    )
    calibrator._telemetry = _TelemetryStub()
    calibrator._target_hz = 0.0
    calibrator._terminal_log_enabled = False
    calibrator._last_terminal_log_time = 0.0
    calibrator._stream_configs = {}
    calibrator._stream_enabled = False
    calibrator._last_rendered_frame = None
    return calibrator


def test_vision_filter_selects_longest_opposite_slope_pair() -> None:
    processor = VisionProcessor(roi_height_pct=0.6)
    selected = processor._apply_geometric_filter(
        [
            (0, 20, 10, 0),
            (0, 100, 50, 0),
            (0, 0, 10, 20),
            (0, 0, 50, 100),
            (0, 0, 100, 1),
            (5, 0, 5, 100),
        ]
    )

    assert selected == ((0, 100, 50, 0), (0, 0, 50, 100))


def test_geometry_maps_intersection_intercepts_and_angle() -> None:
    geometry = GeometryCalculator()
    negative = (0, 100, 100, 0)
    positive = (100, 0, 200, 100)

    assert geometry.calculate_vanishing_point(negative, positive) == (100, 0)
    assert geometry.calculate_bottom_intercepts(negative, positive, 200) == (-100, 300)
    assert geometry.map_vp_to_angle(100, 200) == pytest.approx(90.0)
    assert geometry.map_vp_to_angle(50, 0) == pytest.approx(90.0)


@pytest.mark.parametrize(
    ("vp_angle", "left", "right", "expected_angle", "expected_state"),
    [
        (None, None, None, 90.0, "GAPPING"),
        (90.0, 101, 600, 95.0, "DANGER_RIGHT"),
        (90.0, 0, 539, 85.0, "DANGER_LEFT"),
        (92.0, 0, 640, 90.0, "TRACKING_COAST"),
        (100.0, 0, 640, 101.0, "TRACKING_PD"),
    ],
)
def test_steering_state_machine_current_outputs(
    vp_angle,
    left,
    right,
    expected_angle,
    expected_state,
) -> None:
    controller = SteeringController(
        pid_constants=PIDConstants(kp=1.0, ki=0.5, kd=0.1),
        danger_margin=100,
        nudge_deg=5.0,
        inner_thresh=3.0,
        outer_thresh=5.0,
        center_angle=90.0,
        max_offset=30.0,
    )

    angle, state = controller.compute_steering(vp_angle, left, right, 640)

    assert angle == pytest.approx(expected_angle)
    assert state == expected_state


def test_steering_hysteresis_stays_active_until_inner_threshold() -> None:
    controller = SteeringController(
        pid_constants=PIDConstants(kp=1.0, ki=0.0, kd=0.0),
        danger_margin=0,
        nudge_deg=5.0,
        inner_thresh=3.0,
        outer_thresh=5.0,
        center_angle=90.0,
        max_offset=30.0,
    )

    assert controller.compute_steering(96.0, 0, 640, 640) == (96.0, "TRACKING_PD")
    assert controller.compute_steering(94.0, 0, 640, 640) == (94.0, "TRACKING_PD")
    assert controller.compute_steering(93.0, 0, 640, 640) == (90.0, "TRACKING_COAST")


def test_unified_update_calls_both_current_vision_paths_once() -> None:
    calibrator = _make_calibrator_for_characterization()
    frame = np.zeros((200, 200, 3), dtype=np.uint8)

    angle = calibrator.update(frame, frame_num=7)

    assert angle == pytest.approx(90.0)
    assert calibrator._detector.calls == 1
    assert calibrator._vision.process_calls == 1
    assert calibrator._vision.filter_calls == 1
    assert calibrator._telemetry.logged is not None
    assert calibrator._telemetry.logged["frame_num"] == 7
    assert calibrator._telemetry.logged["vp_angle"] == pytest.approx(90.0)
    assert calibrator._telemetry.logged["fsm_state"] == "TRACKING_COAST"
    assert calibrator._telemetry.logged["calibration_active"] == 0


def test_telemetry_logger_projects_known_fields_and_formats_bbox() -> None:
    output = io.StringIO()
    logger = TelemetryLogger.__new__(TelemetryLogger)
    logger._csv_fieldnames = ["frame_num", "vp_angle", "selected_group_bbox", "optional"]
    logger._csv_writer = csv.DictWriter(output, fieldnames=logger._csv_fieldnames)
    logger._csv_file = None

    logger.log_state(
        3,
        {
            "vp_angle": 91.5,
            "selected_group_bbox": (1, 2, 3, 4),
            "ignored": "not projected",
        },
    )

    assert output.getvalue().strip() == "3,91.5,\"1,2,3,4\","
