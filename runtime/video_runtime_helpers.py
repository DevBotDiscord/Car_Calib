"""Reusable runtime and rendering helpers for camera/video pipelines."""

from __future__ import annotations

import argparse
import csv
import logging
import os
import time
from typing import Any, TextIO, cast

import cv2
import numpy as np


def maybe_flip_frame(frame: np.ndarray, flip_frame: bool) -> np.ndarray:
    """Return flipped frame when *flip_frame* is enabled (180 degree flip)."""
    if not flip_frame:
        return frame
    return cv2.flip(frame, -1)


def configure_terminal_logging(enabled: bool) -> None:
    """Configure whether INFO logs are shown in terminal output."""
    logging.getLogger().setLevel(logging.INFO if enabled else logging.ERROR)


def init_video(path: str, logger: logging.Logger) -> tuple[cv2.VideoCapture, float, int, int, int]:
    """Open a video file and return capture and metadata."""
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open video file: {path}")

    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps <= 0:
        fps = 30.0

    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    logger.info(
        "Video initialised: %s (FPS=%.2f, frames=%d)",
        path,
        fps,
        frame_count,
    )
    return cap, fps, frame_count, frame_width, frame_height


def init_video_writer(path: str, fps: float, width: int, height: int) -> cv2.VideoWriter:
    """Create an output video writer using MP4V codec."""
    fourcc_builder = getattr(cv2, "VideoWriter_fourcc", None)
    if callable(fourcc_builder):
        fourcc = int(cast(int, fourcc_builder(*"mp4v")))
    else:
        fourcc = int(cv2.VideoWriter.fourcc(*"mp4v"))
    writer = cv2.VideoWriter(path, fourcc, fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"Failed to create output video writer: {path}")
    return writer


def init_camera(index: int) -> cv2.VideoCapture:
    """Open a camera device and validate it."""
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open camera at index {index}.")
    return cap


def init_csv_logger(path: str, fieldnames: list[str]) -> tuple[csv.DictWriter, TextIO]:
    """Open (or create) *path* and return a ``(writer, file)`` pair."""
    file_exists = os.path.isfile(path)
    csv_file = open(path, "a", newline="")  # noqa: SIM115
    writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    if not file_exists:
        writer.writeheader()
        csv_file.flush()
    return writer, csv_file


def print_progress(frame_num: int, total_frames: int) -> None:
    """Print an in-place progress bar to the terminal."""
    if total_frames <= 0:
        print(f"\rProcessed frames: {frame_num}", end="", flush=True)
        return

    width = 32
    ratio = min(frame_num / total_frames, 1.0)
    filled = int(width * ratio)
    bar = "#" * filled + "-" * (width - filled)
    percent = ratio * 100.0
    print(f"\r[{bar}] {frame_num}/{total_frames} ({percent:5.1f}%)", end="", flush=True)


def sleep_remainder(loop_start: float, loop_period: float, logger: logging.Logger) -> None:
    """Sleep for the remainder of the target loop period."""
    elapsed = time.monotonic() - loop_start
    remaining = loop_period - elapsed
    if remaining > 0:
        time.sleep(remaining)
    else:
        logger.debug("Loop overrun by %.4f s.", -remaining)


def _to_bgr(image: np.ndarray) -> np.ndarray:
    if image.ndim == 2:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    return image


def _fit_to_tile(image: np.ndarray, tile_w: int, tile_h: int) -> np.ndarray:
    return cv2.resize(image, (tile_w, tile_h), interpolation=cv2.INTER_AREA)


def _draw_tile_label(tile: np.ndarray, label: str) -> None:
    cv2.rectangle(tile, (0, 0), (tile.shape[1], 24), (0, 0, 0), -1)
    cv2.putText(tile, label, (8, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 2)


def build_detector_debug_panel(
    frame_width: int,
    panel_height: int,
    detector_debug: dict[str, Any],
) -> np.ndarray:
    """Build a compact debug panel with key detector stages only."""
    panel = np.zeros((panel_height, frame_width, 3), dtype=np.uint8)
    rows = 2
    cols = 2
    tile_h = panel_height // rows
    tile_w = frame_width // cols

    gray = _fit_to_tile(_to_bgr(detector_debug["gray"]), tile_w, tile_h)
    roi = _fit_to_tile(_to_bgr(detector_debug["roi"]), tile_w, tile_h)
    hough_vis = _fit_to_tile(_to_bgr(detector_debug["hough_vis"]), tile_w, tile_h)
    grouped_vis = _fit_to_tile(_to_bgr(detector_debug["grouped_vis"]), tile_w, tile_h)

    tiles = [
        (gray, "Gray"),
        (roi, "ROI Masked"),
        (hough_vis, "Hough Lines"),
        (grouped_vis, "Grouped + Reference"),
    ]

    for idx, (tile, label) in enumerate(tiles):
        r = idx // cols
        c = idx % cols
        _draw_tile_label(tile, label)
        y0 = r * tile_h
        y1 = y0 + tile_h
        x0 = c * tile_w
        x1 = x0 + tile_w
        panel[y0:y1, x0:x1] = tile

    # Put metadata on top of bottom-right tile.
    text_x = tile_w + 8
    text_y = tile_h + 42
    meta_lines = [
        f"lines={detector_debug['lines_count']} groups={detector_debug['groups_count']}",
        f"ref_group={detector_debug['reference_group_index']}",
        f"theta_candidate={detector_debug['theta_candidate']}",
        f"horizontal_ok={detector_debug['horizontal_ok']} sanity_ok={detector_debug['sanity_ok']}",
        f"theta_out={detector_debug['theta_output']}",
        f"stale_output={detector_debug.get('stale_output', False)}",
        f"min_group_len_px={detector_debug.get('min_group_total_length_px')}",
    ]
    for line in meta_lines:
        cv2.putText(panel, line, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (245, 245, 245), 1)
        text_y += 20

    return panel


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
) -> np.ndarray:
    """Render pipeline values onto a frame before writing to output video."""
    cv2.rectangle(frame, (8, 8), (660, 270), (0, 0, 0), -1)
    cv2.addWeighted(frame, 0.65, frame, 0.35, 0, frame)

    cv2.putText(frame, f"Frame: {frame_num}", (16, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(
        frame,
        f"Theta: {theta:.2f} deg" if theta is not None else "Theta: None",
        (16, 62),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2,
    )
    cv2.putText(frame, f"Servo: {servo_angle:.2f} deg", (16, 89), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (80, 255, 80), 2)
    cv2.putText(frame, f"State: {fsm_state}", (16, 116), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 120), 2)

    if theta is None and theta_for_overlay is not None:
        cv2.putText(
            frame,
            f"Theta source: STALE ({theta_for_overlay:.2f} deg)",
            (16, 140),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (80, 80, 255),
            2,
        )

    if show_guidance_overlay:
        servo_offset = servo_angle - servo_center_angle
        if abs(servo_offset) < 1.0:
            direction = "STRAIGHT"
        elif servo_offset > 0:
            direction = "RIGHT"
        else:
            direction = "LEFT"

        direction_y = 168 if (theta is None and theta_for_overlay is not None) else 145
        cv2.putText(
            frame,
            f"Direction: {direction} ({servo_offset:+.2f} deg)",
            (16, direction_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (180, 220, 255),
            2,
        )
        cv2.putText(
            frame,
            f"Start calibrating threshold: +/-{start_calib_threshold_deg:.1f} deg",
            (16, direction_y + 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.56,
            (120, 230, 255),
            2,
        )
        cv2.putText(
            frame,
            f"Stop calibrating threshold: +/-{stop_calib_threshold_deg:.1f} deg",
            (16, direction_y + 47),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.56,
            (120, 255, 150),
            2,
        )

        gauge_x0 = 16
        gauge_x1 = 640
        gauge_y0 = direction_y + 60
        gauge_y1 = gauge_y0 + 23
        gauge_w = gauge_x1 - gauge_x0

        def theta_to_x(theta_deg: float) -> int:
            theta_clamped = max(0.0, min(180.0, theta_deg))
            return gauge_x0 + int((theta_clamped / 180.0) * gauge_w)

        cv2.rectangle(frame, (gauge_x0, gauge_y0), (gauge_x1, gauge_y1), (60, 60, 60), 1)

        accepted_l = theta_to_x(90.0 - start_calib_threshold_deg)
        accepted_r = theta_to_x(90.0 + start_calib_threshold_deg)
        stop_l = theta_to_x(90.0 - stop_calib_threshold_deg)
        stop_r = theta_to_x(90.0 + stop_calib_threshold_deg)

        cv2.rectangle(frame, (accepted_l, gauge_y0 + 1), (accepted_r, gauge_y1 - 1), (60, 160, 220), -1)
        cv2.rectangle(frame, (stop_l, gauge_y0 + 1), (stop_r, gauge_y1 - 1), (60, 220, 120), -1)

        center_x = theta_to_x(90.0)
        cv2.line(frame, (center_x, gauge_y0 - 3), (center_x, gauge_y1 + 3), (255, 255, 255), 1)
        if theta_for_overlay is not None:
            theta_x = theta_to_x(theta_for_overlay)
            cv2.line(frame, (theta_x, gauge_y0 - 5), (theta_x, gauge_y1 + 5), (0, 0, 255), 2)

        cv2.putText(frame, "0", (gauge_x0 - 2, gauge_y0 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 2)
        cv2.putText(frame, "90", (center_x - 12, gauge_y0 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 220, 220), 2)
        cv2.putText(frame, "180", (gauge_x1 - 22, gauge_y0 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 2)

        legend = "Blue=accepted region | Green=stop-calibrating region | Red=current theta"
        cv2.putText(frame, legend, (16, gauge_y1 + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (220, 220, 220), 2)

    return frame


def build_process_video_arg_parser(
    csv_output_default: str,
    video_output_default: str,
    send_to_servo_default: bool,
    terminal_log_default: bool,
    show_guidance_overlay_default: bool,
    show_detector_debug_default: bool,
    flip_frame_default: bool,
    start_calib_threshold_default: float,
    stop_calib_threshold_default: float,
    frame_sleep_ms_default: float,
) -> argparse.ArgumentParser:
    """Build CLI argument parser for process_video entry point."""
    parser = argparse.ArgumentParser(
        description="Process a video file through the heading-hold control pipeline."
    )
    parser.add_argument("video_path", help="Path to the input video file.")
    parser.add_argument(
        "--output",
        "-o",
        default=csv_output_default,
        help="Path for the output CSV log file (default: video_log.csv).",
    )
    parser.add_argument(
        "--video-output",
        "-v",
        default=video_output_default,
        help="Path for output annotated video (default: processed_video.mp4).",
    )

    parser.set_defaults(
        send_to_servo=send_to_servo_default,
        terminal_log=terminal_log_default,
        show_guidance_overlay=show_guidance_overlay_default,
        show_detector_debug=show_detector_debug_default,
        flip_frame=flip_frame_default,
    )

    parser.add_argument(
        "--no-servo",
        action="store_false",
        dest="send_to_servo",
        help="Do not send commands to the servo; only log computed values.",
    )
    parser.add_argument(
        "--send-servo",
        action="store_true",
        dest="send_to_servo",
        help="Send computed angles to servo hardware.",
    )
    parser.add_argument(
        "--terminal-log",
        action="store_true",
        dest="terminal_log",
        help="Show INFO logs in terminal while processing.",
    )
    parser.add_argument(
        "--no-terminal-log",
        action="store_false",
        dest="terminal_log",
        help="Hide INFO logs in terminal while processing.",
    )
    parser.add_argument(
        "--show-guidance-overlay",
        action="store_true",
        dest="show_guidance_overlay",
        help="Show guidance overlays in output video.",
    )
    parser.add_argument(
        "--no-guidance-overlay",
        action="store_false",
        dest="show_guidance_overlay",
        help="Disable guidance overlays in output video.",
    )
    parser.add_argument(
        "--show-detector-debug",
        action="store_true",
        dest="show_detector_debug",
        help="Render compact detector debug panel.",
    )
    parser.add_argument(
        "--no-detector-debug",
        action="store_false",
        dest="show_detector_debug",
        help="Disable detector debug panel output.",
    )
    parser.add_argument(
        "--flip-frame",
        action="store_true",
        dest="flip_frame",
        help="Flip each frame by 180 degrees before processing.",
    )
    parser.add_argument(
        "--no-flip-frame",
        action="store_false",
        dest="flip_frame",
        help="Disable frame flipping.",
    )

    parser.add_argument(
        "--sleep-ms",
        type=float,
        default=frame_sleep_ms_default,
        help="Extra delay in milliseconds added after each frame (default: 0).",
    )

    parser.add_argument(
        "--start-calib-threshold",
        type=float,
        default=start_calib_threshold_default,
        help="Start calibrating threshold in degrees around 90.",
    )
    parser.add_argument(
        "--stop-calib-threshold",
        type=float,
        default=stop_calib_threshold_default,
        help="Stop calibrating threshold in degrees around 90.",
    )
    return parser
