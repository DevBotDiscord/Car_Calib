"""Runtime helpers for CSV, video, camera, overlays, and loop timing."""

from __future__ import annotations

import argparse
import csv
import logging
import time
from datetime import datetime
from pathlib import Path
from typing import Any, TextIO

import cv2
import numpy as np

from runtime.overlay_drawer import OverlayDrawer


def init_csv_logger(
    path: str,
    fieldnames: list[str],
    use_daily_layout: bool = True,
) -> tuple[csv.DictWriter, TextIO]:
    """Open CSV append logger with optional legacy daily/timestamp layout."""
    source = Path(path)
    if use_daily_layout:
        now = datetime.now()
        day_folder = f"{now.day}_{now.month}_{now.year}"
        timestamp = f"{now.hour}_{now.minute}"
        stem = source.stem or "run_logs"
        suffix = source.suffix or ".csv"
        base_dir = source.parent if str(source.parent) != "." else Path("logs")
        csv_path = base_dir / day_folder / f"{timestamp}_{stem}{suffix}"
    else:
        suffix = source.suffix or ".csv"
        file_name = source.name if source.name else f"run_logs{suffix}"
        base_dir = source.parent if str(source.parent) != "." else Path("logs")
        csv_path = base_dir / file_name

    csv_path.parent.mkdir(parents=True, exist_ok=True)
    existed = csv_path.exists() and csv_path.stat().st_size > 0
    fileobj = csv_path.open("a", newline="", encoding="utf-8")
    writer = csv.DictWriter(fileobj, fieldnames=fieldnames)
    if not existed:
        writer.writeheader()
        fileobj.flush()
    return writer, fileobj


def print_progress(frame_num: int, total_frames: int) -> None:
    """Print an in-place progress bar."""
    total = max(total_frames, 1)
    progress = min(max(frame_num / total, 0.0), 1.0)
    bar_len = 24
    filled = int(progress * bar_len)
    bar = "#" * filled + "-" * (bar_len - filled)
    print(f"\r[{bar}] {frame_num}/{total_frames}", end="", flush=True)


def init_video(path: str, logger: logging.Logger) -> tuple[cv2.VideoCapture, float, int, int, int]:
    """Open and validate a video capture from file path."""
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise RuntimeError(f"Unable to open video file: {path}")
    fps = float(cap.get(cv2.CAP_PROP_FPS) or 30.0)
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
    logger.info("Video initialized: %s (%sx%s @ %.2f fps)", path, width, height, fps)
    return cap, fps, frame_count, width, height


def init_video_writer(path: str, fps: float, width: int, height: int) -> cv2.VideoWriter:
    """Create MP4 video writer."""
    out_path = Path(path)
    if out_path.parent and str(out_path.parent) != ".":
        out_path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(out_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        float(fps),
        (int(width), int(height)),
    )
    if not writer.isOpened():
        raise RuntimeError(f"Unable to open video writer at: {path}")
    return writer


def init_live_video_writer(path: str, fps: float, width: int, height: int) -> tuple[cv2.VideoWriter, str]:
    """Create timestamped MP4 writer and return writer + resolved path."""
    src = Path(path)
    ts = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
    stem = src.stem or "debug"
    suffix = src.suffix or ".mp4"
    resolved = str(src.with_name(f"{stem}_{ts}{suffix}"))
    return init_video_writer(resolved, fps, width, height), resolved


def configure_terminal_logging(enabled: bool) -> None:
    """Adjust stream handler verbosity for root logger."""
    root = logging.getLogger()
    level = logging.INFO if enabled else logging.ERROR
    root.setLevel(level)
    for handler in root.handlers:
        if isinstance(handler, logging.StreamHandler):
            handler.setLevel(level)


def init_camera(index: int) -> cv2.VideoCapture:
    """Open and validate a camera device."""
    capture = cv2.VideoCapture(int(index))
    if not capture.isOpened():
        raise RuntimeError(f"Unable to open camera index {index}")
    return capture


def init_camera_with_retries(index: int, retries: int, logger: logging.Logger) -> cv2.VideoCapture:
    """Try opening camera with bounded retries."""
    attempts = max(1, int(retries))
    for attempt in range(1, attempts + 1):
        try:
            return init_camera(index)
        except RuntimeError:
            logger.warning("Camera init failed (%s/%s)", attempt, attempts)
            if attempt < attempts:
                time.sleep(0.25)
    raise RuntimeError(f"Unable to open camera index {index} after {attempts} retries")


def sleep_remainder(loop_start: float, loop_period: float, logger: logging.Logger) -> None:
    """Sleep to maintain loop period and log overruns."""
    elapsed = time.perf_counter() - float(loop_start)
    remaining = float(loop_period) - elapsed
    if remaining > 0:
        time.sleep(remaining)
    else:
        logger.debug("Loop overrun by %.2f ms", abs(remaining) * 1000.0)


def build_process_video_arg_parser(
    default_input: str = "",
    default_output: str = "processed_video.mp4",
    default_csv: str = "video_log.csv",
    default_send_to_servo: bool = True,
) -> argparse.ArgumentParser:
    """Build parser used by process-video entrypoint."""
    parser = argparse.ArgumentParser(description="Process calibration video")
    parser.add_argument("--input", default=default_input, help="Input video path")
    parser.add_argument("--output", default=default_output, help="Output video path")
    parser.add_argument("--csv-output", default=default_csv, help="CSV output path")
    parser.add_argument("--send-to-servo", action="store_true", default=default_send_to_servo)
    parser.add_argument("--no-send-to-servo", action="store_false", dest="send_to_servo")
    return parser


def build_main_arg_parser(
    default_camera_index: int = 0,
    default_target_hz: float = 30.0,
    default_csv_log_file: str = "run_logs.csv",
    **compat_defaults: Any,
) -> argparse.ArgumentParser:
    """Build parser used by main realtime entrypoint."""
    parser = argparse.ArgumentParser(description="Unified realtime calibration")
    parser.add_argument("--camera-index", type=int, default=default_camera_index)
    parser.add_argument("--target-hz", type=float, default=default_target_hz)
    parser.add_argument("--csv-log-file", default=default_csv_log_file)
    parser.add_argument("--csv-output", dest="csv_output", default=compat_defaults.get("csv_output_default", default_csv_log_file))
    parser.add_argument("--video-output", default=compat_defaults.get("debug_video_output_default", "main_debug.mp4"))
    parser.add_argument("--host", default=compat_defaults.get("stream_host_default", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=compat_defaults.get("stream_port_default", 8443))
    parser.add_argument("--stream-token", default=compat_defaults.get("stream_token_default", ""))
    parser.add_argument("--frame-scale", type=float, default=compat_defaults.get("frame_scale_default", 1.0))
    parser.add_argument("--overlay-scale", type=float, default=compat_defaults.get("overlay_scale_default", 1.0))
    parser.add_argument("--camera-retry-limit", type=int, default=compat_defaults.get("camera_retry_limit_default", 3))
    parser.add_argument("--video-retry-limit", type=int, default=compat_defaults.get("video_retry_limit_default", 5))
    parser.add_argument("--hardware-retry-limit", type=int, default=compat_defaults.get("hardware_retry_limit_default", 5))
    parser.set_defaults(
        debug_mode=compat_defaults.get("debug_mode_default", False),
        terminal_log=compat_defaults.get("terminal_log_default", True),
        show_preview=compat_defaults.get("show_preview_default", False),
        show_guidance_overlay=compat_defaults.get("show_guidance_overlay_default", True),
        show_detector_debug=compat_defaults.get("show_detector_debug_default", False),
        write_debug_video=compat_defaults.get("write_debug_video_default", False),
        flip_frame=compat_defaults.get("flip_frame_default", False),
        stream_enabled=compat_defaults.get("stream_enabled_default", False),
        stream_public=compat_defaults.get("stream_public_default", False),
    )
    parser.add_argument("--debug", action="store_true", dest="debug_mode")
    parser.add_argument("--no-debug", action="store_false", dest="debug_mode")
    parser.add_argument("--terminal-log", action="store_true", dest="terminal_log")
    parser.add_argument("--no-terminal-log", action="store_false", dest="terminal_log")
    parser.add_argument("--show-preview", action="store_true", dest="show_preview")
    parser.add_argument("--no-preview", action="store_false", dest="show_preview")
    parser.add_argument("--show-guidance-overlay", action="store_true", dest="show_guidance_overlay")
    parser.add_argument("--no-guidance-overlay", action="store_false", dest="show_guidance_overlay")
    parser.add_argument("--show-detector-debug", action="store_true", dest="show_detector_debug")
    parser.add_argument("--no-detector-debug", action="store_false", dest="show_detector_debug")
    parser.add_argument("--write-debug-video", action="store_true", dest="write_debug_video")
    parser.add_argument("--no-write-debug-video", action="store_false", dest="write_debug_video")
    parser.add_argument("--flip-frame", action="store_true", dest="flip_frame")
    parser.add_argument("--no-flip-frame", action="store_false", dest="flip_frame")
    parser.add_argument("--stream", action="store_true", dest="stream_enabled")
    parser.add_argument("--no-stream", action="store_false", dest="stream_enabled")
    parser.add_argument("--public", action="store_true", dest="stream_public")
    return parser


def maybe_flip_frame(frame: np.ndarray, flip_frame: bool) -> np.ndarray:
    """Conditionally flip frame horizontally."""
    if flip_frame:
        return cv2.flip(frame, 1)
    return frame


def _line_intersection(line1: Any, line2: Any) -> tuple[int, int] | None:
    try:
        x1, y1, x2, y2 = [float(v) for v in line1]
        x3, y3, x4, y4 = [float(v) for v in line2]
    except (TypeError, ValueError):
        return None
    denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if denominator == 0:
        return None
    det1 = x1 * y2 - y1 * x2
    det2 = x3 * y4 - y3 * x4
    px = (det1 * (x3 - x4) - (x1 - x2) * det2) / denominator
    py = (det1 * (y3 - y4) - (y1 - y2) * det2) / denominator
    return int(round(px)), int(round(py))


def _line_x_at_y(line: Any, target_y: int) -> int | None:
    try:
        x1, y1, x2, y2 = [float(v) for v in line]
    except (TypeError, ValueError):
        return None
    if y2 == y1:
        return int(round(x1))
    t = (float(target_y) - y1) / (y2 - y1)
    return int(round(x1 + t * (x2 - x1)))


def draw_overlay(
    frame: np.ndarray,
    frame_num: int,
    theta: float | None,
    theta_for_overlay: float | None,
    servo_angle: float,
    servo_center_angle: float,
    fsm_state: str,
    show_guidance_overlay: bool,
    start_calib_threshold_deg: float,
    stop_calib_threshold_deg: float,
    overlay_scale: float = 1.0,
    route_id: str | None = None,
    route_mode: str | None = None,
    detector_debug: dict[str, Any] | None = None,
) -> np.ndarray:
    """Render V2 HUD / telemetry overlay and return annotated frame."""
    debug = detector_debug or {}
    selected_lines = debug.get("selected_lines") or []
    vp_x = debug.get("vp_x")
    vp_y = debug.get("vp_y")
    if (vp_x is None or vp_y is None) and len(selected_lines) >= 2:
        vp = _line_intersection(selected_lines[0], selected_lines[1])
        if vp is not None:
            vp_x, vp_y = vp

    left_intercept = debug.get("left_intercept")
    right_intercept = debug.get("right_intercept")
    if (left_intercept is None or right_intercept is None) and len(selected_lines) >= 2:
        intercepts = [_line_x_at_y(line, frame.shape[0] - 1) for line in selected_lines[:2]]
        if all(value is not None for value in intercepts):
            left_intercept, right_intercept = sorted(intercepts)

    raw_angle = debug.get("raw_vp_angle")
    if raw_angle is None:
        raw_angle = theta_for_overlay if theta_for_overlay is not None else theta

    drawer = OverlayDrawer(
        inner_thresh=stop_calib_threshold_deg,
        outer_thresh=start_calib_threshold_deg,
    )
    output = drawer.draw(
        frame,
        {
            "state": fsm_state,
            "raw_vp_angle": raw_angle,
            "left_intercept_x": left_intercept,
            "right_intercept_x": right_intercept,
            "final_steering_cmd": servo_angle,
            "lines": selected_lines,
            "vp_coord": None if vp_x is None or vp_y is None else (vp_x, vp_y),
        },
    )

    if route_id or route_mode:
        cv2.putText(
            output,
            f"Route: {route_id or '-'}  Mode: {route_mode or '-'}",
            (10, 176),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (180, 255, 200),
            1,
            cv2.LINE_AA,
        )
    return output

def build_detector_debug_panel(frame_width: int, panel_height: int, detector_debug: dict[str, Any]) -> np.ndarray:
    """Build 2x2 detector debug panel from provided stage images."""
    panel_h = max(120, int(panel_height))
    panel_w = max(200, int(frame_width))
    tile_h = panel_h // 2
    tile_w = panel_w // 2

    def to_bgr(value: Any) -> np.ndarray:
        if isinstance(value, np.ndarray) and value.size > 0:
            image = value
        else:
            image = np.zeros((tile_h, tile_w), dtype=np.uint8)
        if image.ndim == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        if image.shape[2] == 4:
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        return cv2.resize(image, (tile_w, tile_h))

    gray = to_bgr(detector_debug.get("gray"))
    edges = to_bgr(detector_debug.get("edges"))
    hough = to_bgr(detector_debug.get("hough_vis"))
    grouped = to_bgr(detector_debug.get("grouped_vis"))

    top = np.hstack((gray, edges))
    bottom = np.hstack((hough, grouped))
    panel = np.vstack((top, bottom))

    lines_count = detector_debug.get("lines_count", "")
    cv2.putText(
        panel,
        f"lines={lines_count}",
        (8, panel.shape[0] - 8),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return panel
