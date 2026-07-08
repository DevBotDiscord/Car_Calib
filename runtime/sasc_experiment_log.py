"""SASC baseline experiment CSV writer."""

from __future__ import annotations

import csv
import os
import re
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, TextIO

SASC_FIELDNAMES = [
    "experiment_id",
    "run_id",
    "algorithm_version",
    "adaptive_enabled",
    "scene_type",
    "frame_id",
    "timestamp_ms",
    "frame_width",
    "frame_height",
    "roi_height",
    "edge_pixels",
    "total_hough_lines",
    "left_candidate_lines",
    "right_candidate_lines",
    "selected_pair_found",
    "left_bottom_intercept_x",
    "right_bottom_intercept_x",
    "lane_width_px",
    "width_error_px",
    "vp_x",
    "vp_y",
    "vp_angle",
    "servo_angle",
    "steering_offset",
    "steering_error",
    "derivative_error",
    "tracking_active",
    "danger_zone_active",
    "danger_margin_px",
    "danger_action",
    "vision_lost",
    "lost_frames",
    "recovery_active",
    "recovery_angle",
    "processing_time_ms",
    "fps",
    "cpu_usage_percent",
    "ram_usage_mb",
    "lane_detect_success",
    "frame_quality_note",
]


@dataclass
class ResourceSample:
    cpu_usage_percent: float | None
    ram_usage_mb: float | None


class ResourceSampler:
    def __init__(self) -> None:
        self._last_wall = time.perf_counter()
        self._last_cpu = time.process_time()

    def sample(self) -> ResourceSample:
        now_wall = time.perf_counter()
        now_cpu = time.process_time()
        wall_delta = max(0.0, now_wall - self._last_wall)
        cpu_delta = max(0.0, now_cpu - self._last_cpu)
        self._last_wall = now_wall
        self._last_cpu = now_cpu

        cpu_percent = None
        if wall_delta > 0:
            cpu_percent = min(100.0 * (os.cpu_count() or 1), (cpu_delta / wall_delta) * 100.0)
        return ResourceSample(cpu_usage_percent=cpu_percent, ram_usage_mb=_ram_usage_mb())


class SascExperimentLogger:
    def __init__(
        self,
        path: str | Path,
        *,
        run_id: str,
        scene_type: str | None = None,
        start_monotonic: float | None = None,
    ) -> None:
        self.path = Path(path)
        self.scene_type = _scene_type(scene_type)
        self.run_id = _compose_run_id(_env_str("SASC_RUN_ID", run_id), scene_type=self.scene_type)
        self.start_monotonic = time.monotonic() if start_monotonic is None else float(start_monotonic)
        self._sampler = ResourceSampler()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file: TextIO = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=SASC_FIELDNAMES, extrasaction="ignore")
        self._writer.writeheader()
        self._file.flush()

    def write_frame(self, telemetry: dict[str, Any], *, frame_id: int, mono_now: float) -> None:
        elapsed_ms = max(0, int(round((float(mono_now) - self.start_monotonic) * 1000.0)))
        resources = self._sampler.sample()
        row = build_sasc_row(
            telemetry,
            run_id=self.run_id,
            scene_type=self.scene_type,
            frame_id=frame_id,
            timestamp_ms=elapsed_ms,
            resources=resources,
        )
        self._writer.writerow(row)
        self._file.flush()

    def close(self) -> None:
        self._file.flush()
        self._file.close()


def build_sasc_row(
    telemetry: dict[str, Any],
    *,
    run_id: str,
    scene_type: str | None = None,
    frame_id: int,
    timestamp_ms: int,
    resources: ResourceSample | None = None,
) -> dict[str, Any]:
    loop_ms = _float_or_none(telemetry.get("loop_ms"))
    fps = _float_or_none(telemetry.get("fps"))
    if fps is None and loop_ms and loop_ms > 0:
        fps = 1000.0 / loop_ms
    selected_pair = _int_bool(telemetry.get("selected_pair_found"))
    vision_lost = _int_bool(telemetry.get("vision_lost"))
    danger_active = _int_bool(telemetry.get("danger_zone_active"))
    object_state = str(telemetry.get("object_state") or "none")
    resolved_scene_type = _scene_type(scene_type)

    return {
        "experiment_id": _env_str("SASC_EXPERIMENT_ID", "EXP01"),
        "run_id": run_id,
        "algorithm_version": _env_str("SASC_ALGORITHM_VERSION", "baseline_v1"),
        "adaptive_enabled": int(_env_bool("SASC_ADAPTIVE_ENABLED", False)),
        "scene_type": resolved_scene_type,
        "frame_id": frame_id,
        "timestamp_ms": timestamp_ms,
        "frame_width": _clean(telemetry.get("frame_width")),
        "frame_height": _clean(telemetry.get("frame_height")),
        "roi_height": _clean(telemetry.get("roi_height")),
        "edge_pixels": _clean(telemetry.get("edge_pixels")),
        "total_hough_lines": _clean(telemetry.get("total_hough_lines", telemetry.get("lines_count"))),
        "left_candidate_lines": _clean(telemetry.get("left_candidate_lines")),
        "right_candidate_lines": _clean(telemetry.get("right_candidate_lines")),
        "selected_pair_found": selected_pair,
        "left_bottom_intercept_x": _clean(telemetry.get("left_intercept")),
        "right_bottom_intercept_x": _clean(telemetry.get("right_intercept")),
        "lane_width_px": _clean(telemetry.get("lane_width_px")),
        "width_error_px": _clean(telemetry.get("width_error_px")),
        "vp_x": _clean(telemetry.get("vp_x")),
        "vp_y": _clean(telemetry.get("vp_y")),
        "vp_angle": _clean(telemetry.get("vp_angle")),
        "servo_angle": _clean(telemetry.get("servo_angle")),
        "steering_offset": _clean(telemetry.get("servo_offset")),
        "steering_error": _clean(telemetry.get("pid_error")),
        "derivative_error": _clean(telemetry.get("derivative_error")),
        "tracking_active": _clean(telemetry.get("tracking_active")),
        "danger_zone_active": danger_active,
        "danger_margin_px": _clean(telemetry.get("danger_margin_px")),
        "danger_action": _clean(telemetry.get("danger_action", "NONE")),
        "vision_lost": vision_lost,
        "lost_frames": _clean(telemetry.get("lost_frames")),
        "recovery_active": _clean(telemetry.get("recovery_active")),
        "recovery_angle": _clean(telemetry.get("recovery_angle")),
        "processing_time_ms": _clean(loop_ms),
        "fps": _clean(fps),
        "cpu_usage_percent": _clean(None if resources is None else resources.cpu_usage_percent),
        "ram_usage_mb": _clean(None if resources is None else resources.ram_usage_mb),
        "lane_detect_success": selected_pair,
        "frame_quality_note": _frame_quality_note(
            object_state=object_state,
            vision_lost=bool(vision_lost),
            danger_active=bool(danger_active),
        ),
    }


def _frame_quality_note(*, object_state: str, vision_lost: bool, danger_active: bool) -> str:
    if object_state == "near":
        return "object_near"
    if object_state == "far":
        return "object_far"
    if vision_lost:
        return "vision_lost"
    if danger_active:
        return "danger_zone"
    return "normal"


def _compose_run_id(base_run_id: str, *, scene_type: str | None = None) -> str:
    base = _id_part(base_run_id or time.strftime("RUN%Y%m%dT%H%M%SZ", time.gmtime()))
    scene = _id_part(_scene_type(scene_type))
    experiment = _id_part(_env_str("SASC_EXPERIMENT_ID", "EXP01"))
    return f"{base}_{scene}_{experiment}"


def _scene_type(value: str | None) -> str:
    if value is not None and str(value).strip():
        return str(value).strip()
    return _env_str("SASC_SCENE_TYPE", "unknown")


def _id_part(value: str) -> str:
    normalized = re.sub(r"[^A-Za-z0-9-]+", "-", str(value).strip())
    normalized = normalized.strip("-")
    return normalized or "unknown"


def _ram_usage_mb() -> float | None:
    try:
        import resource  # type: ignore[import-not-found]
    except ImportError:
        return None
    usage = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    if os.name == "posix":
        return float(usage) / 1024.0
    return float(usage) / (1024.0 * 1024.0)


def _clean(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, str):
        return value
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, float):
        return round(value, 4)
    return value


def _float_or_none(value: Any) -> float | None:
    if value is None or value == "":
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _int_bool(value: Any) -> int:
    if isinstance(value, str):
        return int(value.strip().lower() in {"1", "true", "yes", "on"})
    return int(bool(value))


def _env_bool(name: str, default: bool) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _env_str(name: str, default: str) -> str:
    value = os.getenv(name)
    if value is None or value == "":
        return default
    return value
