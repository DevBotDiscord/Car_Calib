"""Tests for SASC baseline experiment CSV mapping."""

from __future__ import annotations

from runtime.sasc_experiment_log import SASC_FIELDNAMES, ResourceSample, SascExperimentLogger, build_sasc_row


def test_sasc_row_matches_template_columns_and_maps_telemetry():
    telemetry = {
        "frame_width": 640,
        "frame_height": 480,
        "roi_height": 240,
        "edge_pixels": 3821,
        "total_hough_lines": 14,
        "left_candidate_lines": 6,
        "right_candidate_lines": 5,
        "selected_pair_found": 1,
        "left_intercept": 142,
        "right_intercept": 498,
        "lane_width_px": 356,
        "width_error_px": 24,
        "vp_x": 321,
        "vp_y": 88,
        "vp_angle": 90.28,
        "servo_angle": 91.4,
        "servo_offset": 1.4,
        "pid_error": "0.28",
        "derivative_error": "0.06",
        "tracking_active": 1,
        "danger_zone_active": 0,
        "danger_margin_px": 40,
        "danger_action": "NONE",
        "vision_lost": 0,
        "lost_frames": 0,
        "recovery_active": 0,
        "recovery_angle": 0,
        "loop_ms": 18.7,
    }

    row = build_sasc_row(
        telemetry,
        run_id="RUN01",
        frame_id=1,
        timestamp_ms=0,
        resources=ResourceSample(cpu_usage_percent=42.1, ram_usage_mb=612.0),
    )

    assert list(row.keys()) == SASC_FIELDNAMES
    assert row["run_id"] == "RUN01"
    assert row["lane_width_px"] == 356
    assert row["width_error_px"] == 24
    assert row["lane_detect_success"] == 1
    assert row["frame_quality_note"] == "normal"

def test_sasc_logger_composes_run_id_from_preset_scene_and_experiment(tmp_path, monkeypatch):
    monkeypatch.setenv("SASC_SCENE_TYPE", "env scene")
    monkeypatch.setenv("SASC_EXPERIMENT_ID", "EXP01")
    logger = SascExperimentLogger(
        tmp_path / "sasc.csv",
        run_id="route-abc",
        scene_type="straight road",
        start_monotonic=0,
    )
    try:
        assert logger.run_id == "route-abc_straight-road_EXP01"
        assert logger.scene_type == "straight road"
    finally:
        logger.close()
