"""Notebook-friendly calibration evaluation and artifact export."""

from __future__ import annotations

import json
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable, Iterator

import cv2
import numpy as np

from runtime.overlay_drawer import OverlayDrawer
from unified_calibration_components import (
    CalibrationProcessingError,
    CalibrationResult,
    UnifiedCalibrator,
)


@dataclass(frozen=True)
class EvaluationRecord:
    """Serializable result for one evaluated frame."""

    frame_num: int
    timestamp_s: float | None
    status: str
    steering_angle: float | None
    control_state: str | None
    observation_angle: float | None
    calibration_active: bool
    lines_count: int | None
    selected_lines: list[list[int]]
    selected_left_line_info: dict[str, Any] | None
    selected_right_line_info: dict[str, Any] | None
    vp_x: int | None
    vp_y: int | None
    vp_location: str
    left_intercept: int | None
    right_intercept: int | None
    failure_stage: str | None
    failure_process: str | None
    failure_type: str | None
    failure_detail: str | None
    review_panel: str | None = None

    def as_dict(self) -> dict[str, Any]:
        """Return a JSON-serializable dictionary."""
        return asdict(self)


@dataclass(frozen=True)
class EvaluationSummary:
    """Aggregate counts produced by an evaluation run."""

    total_frames: int
    valid_observations: int
    missing_observations: int
    processing_errors: int
    review_panels: int
    control_states: dict[str, int]
    failure_processes: dict[str, int]

    def as_dict(self) -> dict[str, Any]:
        """Return a JSON-serializable dictionary."""
        return asdict(self)


def iter_video_frames(
    video_path: str | Path,
    *,
    start_frame: int = 0,
    max_frames: int | None = None,
    stride: int = 1,
) -> Iterator[tuple[int, float | None, np.ndarray]]:
    """Yield numbered video frames with timestamps for notebook or CLI use."""
    if max_frames is not None and int(max_frames) <= 0:
        return
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise RuntimeError(f"Unable to open video file: {video_path}")

    frame_num = 0
    yielded = 0
    stride = max(1, int(stride))
    try:
        while capture.isOpened():
            ok, frame = capture.read()
            if not ok or frame is None:
                break
            current = frame_num
            frame_num += 1
            if current < start_frame or (current - start_frame) % stride != 0:
                continue
            timestamp_ms = float(capture.get(cv2.CAP_PROP_POS_MSEC))
            timestamp_s = timestamp_ms / 1000.0 if timestamp_ms >= 0 else None
            yield current, timestamp_s, frame
            yielded += 1
            if max_frames is not None and yielded >= max(0, int(max_frames)):
                break
    finally:
        capture.release()


class CalibrationEvaluator:
    """Evaluate frames without hardware, streaming, or runtime side effects."""

    def __init__(
        self,
        calibrator: UnifiedCalibrator | None = None,
        *,
        output_dir: str | Path | None = None,
        review_every: int = 0,
        review_missing: bool = True,
        review_errors: bool = True,
        max_review_panels: int | None = 200,
    ) -> None:
        self._calibrator = calibrator or UnifiedCalibrator(telemetry_enabled=False)
        self._owns_calibrator = calibrator is None
        self._output_dir = None if output_dir is None else Path(output_dir)
        self._review_every = max(0, int(review_every))
        self._review_missing = bool(review_missing)
        self._review_errors = bool(review_errors)
        self._max_review_panels = (
            None if max_review_panels is None else max(0, int(max_review_panels))
        )
        self._saved_review_count = 0

    def evaluate_frames(
        self,
        frames: Iterable[np.ndarray | tuple[int, float | None, np.ndarray]],
    ) -> tuple[list[EvaluationRecord], EvaluationSummary]:
        """Evaluate an iterable and optionally persist review artifacts."""
        self._saved_review_count = 0
        records: list[EvaluationRecord] = []
        jsonl_file = None
        if self._output_dir is not None:
            self._output_dir.mkdir(parents=True, exist_ok=True)
            jsonl_file = (self._output_dir / "frames.jsonl").open("w", encoding="utf-8")

        try:
            for fallback_num, item in enumerate(frames):
                frame_num, timestamp_s, frame = self._normalize_item(fallback_num, item)
                record = self.evaluate_frame(frame, frame_num, timestamp_s)
                records.append(record)
                if jsonl_file is not None:
                    jsonl_file.write(json.dumps(record.as_dict(), sort_keys=True) + "\n")
                    jsonl_file.flush()
        finally:
            if jsonl_file is not None:
                jsonl_file.close()

        summary = self.summarize(records)
        if self._output_dir is not None:
            summary_path = self._output_dir / "summary.json"
            summary_path.write_text(
                json.dumps(summary.as_dict(), indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
        return records, summary

    def evaluate_frame(
        self,
        frame: np.ndarray,
        frame_num: int,
        timestamp_s: float | None = None,
    ) -> EvaluationRecord:
        """Evaluate one frame and save a review panel when configured."""
        result: CalibrationResult | None = None
        error: CalibrationProcessingError | None = None
        try:
            result = self._calibrator.process_frame(frame, frame_num)
            record = self._record_from_result(result, frame_num, timestamp_s)
        except CalibrationProcessingError as exc:
            error = exc
            diagnostic = exc.diagnostic
            record = EvaluationRecord(
                frame_num=frame_num,
                timestamp_s=timestamp_s,
                status="processing_error",
                steering_angle=None,
                control_state=None,
                observation_angle=None,
                calibration_active=False,
                lines_count=None,
                selected_lines=[],
                selected_left_line_info=None,
                selected_right_line_info=None,
                vp_x=None,
                vp_y=None,
                vp_location="missing",
                left_intercept=None,
                right_intercept=None,
                failure_stage=diagnostic.stage,
                failure_process=diagnostic.process,
                failure_type=diagnostic.error_type,
                failure_detail=diagnostic.detail,
            )

        if self._should_save_review(record):
            review_path = self._save_review_panel(frame, record, result, error)
            record = EvaluationRecord(**{**record.as_dict(), "review_panel": review_path})
        return record

    @staticmethod
    def summarize(records: Iterable[EvaluationRecord]) -> EvaluationSummary:
        """Aggregate records for quick comparison between calibration runs."""
        materialized = list(records)
        states = Counter(record.control_state for record in materialized if record.control_state)
        failures = Counter(
            record.failure_process for record in materialized if record.failure_process
        )
        return EvaluationSummary(
            total_frames=len(materialized),
            valid_observations=sum(record.status == "valid_observation" for record in materialized),
            missing_observations=sum(
                record.status == "missing_observation" for record in materialized
            ),
            processing_errors=sum(record.status == "processing_error" for record in materialized),
            review_panels=sum(record.review_panel is not None for record in materialized),
            control_states=dict(sorted(states.items())),
            failure_processes=dict(sorted(failures.items())),
        )

    def close(self) -> None:
        """Release the internally created calibrator."""
        if self._owns_calibrator:
            self._calibrator.close()

    @staticmethod
    def _normalize_item(
        fallback_num: int,
        item: np.ndarray | tuple[int, float | None, np.ndarray],
    ) -> tuple[int, float | None, np.ndarray]:
        if isinstance(item, tuple):
            return int(item[0]), item[1], item[2]
        return fallback_num, None, item

    @staticmethod
    def _record_from_result(
        result: CalibrationResult,
        frame_num: int,
        timestamp_s: float | None,
    ) -> EvaluationRecord:
        telemetry = result.telemetry
        vision_debug = result.debug_data.get("vision_debug", {})
        selected_lines = [
            [int(value) for value in line]
            for line in vision_debug.get("selected_lines", [])
        ]
        status = "valid_observation" if result.observation_angle is not None else "missing_observation"
        return EvaluationRecord(
            frame_num=frame_num,
            timestamp_s=timestamp_s,
            status=status,
            steering_angle=result.steering_angle,
            control_state=result.control_state,
            observation_angle=result.observation_angle,
            calibration_active=result.calibration_active,
            lines_count=int(telemetry.get("lines_count", 0)),
            selected_lines=selected_lines,
            selected_left_line_info=vision_debug.get("selected_left_line_info"),
            selected_right_line_info=vision_debug.get("selected_right_line_info"),
            vp_x=telemetry.get("vp_x"),
            vp_y=telemetry.get("vp_y"),
            vp_location=str(telemetry.get("vp_location", "missing")),
            left_intercept=telemetry.get("left_intercept"),
            right_intercept=telemetry.get("right_intercept"),
            failure_stage=None,
            failure_process=None,
            failure_type=None,
            failure_detail=None,
        )

    def _should_save_review(self, record: EvaluationRecord) -> bool:
        if self._output_dir is None:
            return False
        if self._review_every > 0 and record.frame_num % self._review_every == 0:
            selected = True
        elif self._review_missing and record.status == "missing_observation":
            selected = True
        else:
            selected = self._review_errors and record.status == "processing_error"
        if not selected:
            return False
        return (
            self._max_review_panels is None
            or self._saved_review_count < self._max_review_panels
        )

    def _save_review_panel(
        self,
        frame: np.ndarray,
        record: EvaluationRecord,
        result: CalibrationResult | None,
        error: CalibrationProcessingError | None,
    ) -> str:
        assert self._output_dir is not None
        review_dir = self._output_dir / "review_frames"
        review_dir.mkdir(parents=True, exist_ok=True)
        file_name = f"frame_{record.frame_num:06d}_{record.status}.jpg"
        path = review_dir / file_name
        panel = build_review_panel(frame, record, result, error)
        if not cv2.imwrite(str(path), panel):
            raise RuntimeError(f"Unable to write review panel: {path}")
        self._saved_review_count += 1
        return path.relative_to(self._output_dir).as_posix()


def build_review_panel(
    frame: np.ndarray,
    record: EvaluationRecord,
    result: CalibrationResult | None,
    error: CalibrationProcessingError | None = None,
) -> np.ndarray:
    """Build a labeled source/intermediate contact sheet for visual review."""
    source = _to_bgr(frame)
    height, width = source.shape[:2]
    debug = {} if result is None else result.debug_data.get("vision_debug", {})
    overlay = _build_diagnostic_overlay(source, record)
    tiles = [
        ("overlay", overlay),
        ("gray", debug.get("gray")),
        ("edges", debug.get("edges")),
        ("hough", debug.get("hough_vis")),
        ("selected", debug.get("grouped_vis")),
    ]
    rendered = [_labeled_tile(label, image, width, height) for label, image in tiles]
    info = np.zeros_like(source)
    lines = [
        f"frame={record.frame_num} status={record.status}",
        f"state={record.control_state or '-'} observation={record.observation_angle}",
        f"steering={record.steering_angle} lines={record.lines_count}",
        f"vp=({record.vp_x}, {record.vp_y}) [{record.vp_location}]",
        f"intercepts=({record.left_intercept}, {record.right_intercept})",
        f"L/NEG: {_format_line_info(record.selected_left_line_info)}",
        f"R/POS: {_format_line_info(record.selected_right_line_info)}",
    ]
    if error is not None:
        lines.extend(
            [
                f"failure={record.failure_stage}.{record.failure_process}",
                f"{record.failure_type}: {record.failure_detail}",
            ]
        )
    for index, text in enumerate(lines):
        cv2.putText(
            info,
            text[:110],
            (12, 30 + index * 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    rendered.append(_labeled_tile("result", info, width, height))
    return np.vstack((np.hstack(rendered[:3]), np.hstack(rendered[3:])))


def _labeled_tile(label: str, image: Any, width: int, height: int) -> np.ndarray:
    tile = _to_bgr(image)
    tile = cv2.resize(tile, (width, height), interpolation=cv2.INTER_AREA)
    cv2.rectangle(tile, (0, 0), (width, 34), (0, 0, 0), -1)
    cv2.putText(
        tile,
        label,
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return tile


def _to_bgr(image: Any) -> np.ndarray:
    if not isinstance(image, np.ndarray) or image.size == 0:
        return np.zeros((120, 160, 3), dtype=np.uint8)
    if image.ndim == 2:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if image.shape[2] == 4:
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image.copy()


def _format_line_info(info: dict[str, Any] | None) -> str:
    if not info:
        return "-"
    endpoints = info.get("endpoints")
    slope = info.get("slope")
    length = info.get("length")
    intercept = info.get("bottom_intercept")
    slope_text = "vertical" if slope is None else f"{float(slope):+.3f}"
    length_text = "-" if length is None else f"{float(length):.1f}"
    return f"{endpoints} m={slope_text} len={length_text} bottom_x={intercept}"


def _build_diagnostic_overlay(frame: np.ndarray, record: EvaluationRecord) -> np.ndarray:
    left_line = (
        None
        if not record.selected_left_line_info
        else record.selected_left_line_info.get("endpoints")
    )
    right_line = (
        None
        if not record.selected_right_line_info
        else record.selected_right_line_info.get("endpoints")
    )
    return OverlayDrawer().draw(
        frame,
        {
            "state": record.control_state or "VISION_LOST",
            "raw_vp_angle": record.observation_angle,
            "vp_coord": (
                None
                if record.vp_x is None or record.vp_y is None
                else (record.vp_x, record.vp_y)
            ),
            "vp_location": record.vp_location,
            "left_intercept_x": record.left_intercept,
            "right_intercept_x": record.right_intercept,
            "final_steering_cmd": record.steering_angle,
            "lines": record.selected_lines,
            "left_line": left_line,
            "right_line": right_line,
        },
    )
