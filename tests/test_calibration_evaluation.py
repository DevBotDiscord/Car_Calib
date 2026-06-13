"""Tests for notebook-friendly calibration evaluation tooling."""

from __future__ import annotations

import json

import cv2
import numpy as np

from calibration_evaluation import CalibrationEvaluator, build_review_panel, iter_video_frames
from unified_calibration_components import (
    CalibrationProcessingError,
    CalibrationResult,
)


class _SequenceCalibrator:
    def __init__(self) -> None:
        self.calls: list[int] = []

    def process_frame(self, frame: np.ndarray, frame_num: int) -> CalibrationResult:
        self.calls.append(frame_num)
        if frame_num == 2:
            raise CalibrationProcessingError(
                frame_num=frame_num,
                stage="geometry",
                process="calculate_vanishing_point",
                cause=ArithmeticError("synthetic failure"),
            )
        observation = 90.0 if frame_num == 0 else None
        return CalibrationResult(
            steering_angle=90.0,
            control_state="TRACKING_COAST" if observation is not None else "GAPPING",
            observation_angle=observation,
            calibration_active=False,
            telemetry={
                "lines_count": 2 if observation is not None else 0,
                "danger_boundary": None,
                "recovery_direction": None,
                "danger_threshold_x": None,
                "vp_x": 50 if observation is not None else None,
                "vp_y": 10 if observation is not None else None,
                "vp_location": "inside" if observation is not None else "missing",
                "left_intercept": 0 if observation is not None else None,
                "right_intercept": 100 if observation is not None else None,
            },
            debug_data={
                "vision_debug": {
                    "gray": np.zeros((20, 30), dtype=np.uint8),
                    "edges": np.zeros((20, 30), dtype=np.uint8),
                    "hough_vis": np.zeros((20, 30, 3), dtype=np.uint8),
                    "grouped_vis": np.zeros((20, 30, 3), dtype=np.uint8),
                    "selected_lines": [(0, 10, 10, 0), (10, 0, 20, 10)]
                    if observation is not None
                    else [],
                    "selected_left_line_info": {
                        "role": "left_negative",
                        "endpoints": [0, 10, 10, 0],
                        "slope": -1.0,
                        "length": 14.14,
                        "bottom_intercept": 0,
                    }
                    if observation is not None
                    else None,
                    "selected_right_line_info": {
                        "role": "right_positive",
                        "endpoints": [10, 0, 20, 10],
                        "slope": 1.0,
                        "length": 14.14,
                        "bottom_intercept": 100,
                    }
                    if observation is not None
                    else None,
                }
            },
        )


class _CaptureStub:
    def __init__(self, count: int) -> None:
        self.frames = [np.full((2, 3, 3), index, dtype=np.uint8) for index in range(count)]
        self.index = 0
        self.released = False

    def isOpened(self) -> bool:
        return not self.released and self.index < len(self.frames)

    def read(self):
        if not self.isOpened():
            return False, None
        frame = self.frames[self.index]
        self.index += 1
        return True, frame

    def get(self, prop: int) -> float:
        if prop == cv2.CAP_PROP_POS_MSEC:
            return float((self.index - 1) * 100)
        return 0.0

    def release(self) -> None:
        self.released = True


def test_evaluator_exports_records_summary_and_review_panels(tmp_path) -> None:
    calibrator = _SequenceCalibrator()
    evaluator = CalibrationEvaluator(
        calibrator,
        output_dir=tmp_path,
        review_every=0,
        review_missing=True,
        review_errors=True,
    )
    frames = [
        (0, 0.0, np.zeros((20, 30, 3), dtype=np.uint8)),
        (1, 0.1, np.zeros((20, 30, 3), dtype=np.uint8)),
        (2, 0.2, np.zeros((20, 30, 3), dtype=np.uint8)),
    ]

    records, summary = evaluator.evaluate_frames(frames)

    assert calibrator.calls == [0, 1, 2]
    assert [record.status for record in records] == [
        "valid_observation",
        "missing_observation",
        "processing_error",
    ]
    assert summary.as_dict() == {
        "total_frames": 3,
        "valid_observations": 1,
        "missing_observations": 1,
        "processing_errors": 1,
        "review_panels": 2,
        "control_states": {"GAPPING": 1, "TRACKING_COAST": 1},
        "failure_processes": {"calculate_vanishing_point": 1},
    }
    assert records[0].selected_lines == [[0, 10, 10, 0], [10, 0, 20, 10]]
    assert records[0].selected_left_line_info["role"] == "left_negative"
    assert records[0].selected_right_line_info["role"] == "right_positive"
    assert records[0].vp_location == "inside"
    assert records[1].review_panel is not None
    assert records[2].failure_stage == "geometry"
    assert records[2].review_panel is not None
    assert (tmp_path / records[1].review_panel).exists()
    assert (tmp_path / records[2].review_panel).exists()

    exported = [
        json.loads(line)
        for line in (tmp_path / "frames.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    assert [item["status"] for item in exported] == [
        "valid_observation",
        "missing_observation",
        "processing_error",
    ]
    assert json.loads((tmp_path / "summary.json").read_text(encoding="utf-8")) == summary.as_dict()


def test_evaluator_accepts_plain_notebook_frame_iterable() -> None:
    calibrator = _SequenceCalibrator()
    evaluator = CalibrationEvaluator(
        calibrator,
        review_missing=False,
        review_errors=False,
    )

    records, summary = evaluator.evaluate_frames(
        [np.zeros((20, 30, 3), dtype=np.uint8) for _ in range(2)]
    )

    assert [record.frame_num for record in records] == [0, 1]
    assert [record.timestamp_s for record in records] == [None, None]
    assert summary.total_frames == 2


def test_review_panel_contains_six_equal_tiles() -> None:
    calibrator = _SequenceCalibrator()
    frame = np.zeros((20, 30, 3), dtype=np.uint8)
    result = calibrator.process_frame(frame, 0)
    evaluator = CalibrationEvaluator(calibrator)
    record = evaluator.evaluate_frame(frame, 0)

    panel = build_review_panel(frame, record, result)

    assert panel.shape == (40, 90, 3)


def test_review_panel_limit_bounds_artifact_count(tmp_path) -> None:
    evaluator = CalibrationEvaluator(
        _SequenceCalibrator(),
        output_dir=tmp_path,
        review_every=1,
        max_review_panels=1,
    )

    records, summary = evaluator.evaluate_frames(
        [np.zeros((20, 30, 3), dtype=np.uint8) for _ in range(2)]
    )

    assert [record.review_panel is not None for record in records] == [True, False]
    assert summary.review_panels == 1


def test_video_iterator_applies_start_stride_and_limit(monkeypatch) -> None:
    capture = _CaptureStub(8)
    monkeypatch.setattr("calibration_evaluation.cv2.VideoCapture", lambda _path: capture)

    frames = list(iter_video_frames("route.mp4", start_frame=1, stride=2, max_frames=3))

    assert [frame_num for frame_num, _timestamp, _frame in frames] == [1, 3, 5]
    assert [timestamp for _frame_num, timestamp, _frame in frames] == [0.1, 0.3, 0.5]
    assert capture.released is True
