"""Process a video file through the heading-hold control pipeline.

Similar to main.py, but reads frames from a video file instead of a live camera.
Frame timing is based on the video's FPS rather than strict 30 Hz pacing.

The pipeline for each frame:
1. Detect the reference tile-gap angle via :class:`~vision.detector.LineDetector`.
2. Compute the servo angle via :class:`~control.servo_pid.ServoPID`.
3. Log frame data to a CSV file (``video_log.csv``) without sending to servo.

Usage:
    python process_video.py <path_to_video> [--output <csv_path>]
    [--video-output <video_path>] [--no-servo] [--terminal-log]
    [--show-guidance-overlay] [--show-detector-debug]
"""

import argparse
import csv
import logging
import os
import sys
import time
from typing import Any, TextIO

import cv2
import numpy as np

from control.servo_pid import ServoPID
from drivers.servo_driver import ServoDriver
from models.robot_state import RobotState
from vision.detector import LineDetector

# --------------------------------------------------------------------------- #
# Logging configuration
# --------------------------------------------------------------------------- #
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%Y-%m-%dT%H:%M:%S",
)
logger = logging.getLogger(__name__)

# --------------------------------------------------------------------------- #
# Constants
# --------------------------------------------------------------------------- #
_CSV_FIELDNAMES = ["frame_num", "timestamp", "fsm_state", "theta", "servo_angle",
                   "pid_integral", "pid_last_error"]
_DEFAULT_START_CALIB_THRESHOLD_DEG = 5.0
_DEFAULT_STOP_CALIB_THRESHOLD_DEG = 3.0


def _configure_terminal_logging(enabled: bool) -> None:
    """Configure whether INFO logs are shown in terminal output."""
    logging.getLogger().setLevel(logging.INFO if enabled else logging.ERROR)


def _init_video(path: str) -> tuple[cv2.VideoCapture, float, int, int, int]:
    """Open the video file and extract metadata.

    Args:
        path: Filesystem path to the video file.

    Returns:
        Tuple of (capture, fps, frame_count, frame_width, frame_height).

    Raises:
        RuntimeError: If the video cannot be opened.
    """
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
        path, fps, frame_count
    )
    return cap, fps, frame_count, frame_width, frame_height


def _init_video_writer(path: str, fps: float, width: int, height: int) -> cv2.VideoWriter:
    """Create an output video writer using MP4V codec."""
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(path, fourcc, fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"Failed to create output video writer: {path}")
    return writer


def _to_bgr(image: np.ndarray) -> np.ndarray:
    """Convert grayscale image to BGR for tiled debug rendering."""
    if image.ndim == 2:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    return image


def _fit_to_tile(image: np.ndarray, tile_w: int, tile_h: int) -> np.ndarray:
    """Resize *image* to tile dimensions."""
    return cv2.resize(image, (tile_w, tile_h), interpolation=cv2.INTER_AREA)


def _draw_tile_label(tile: np.ndarray, label: str) -> None:
    """Draw a compact stage label on a tile."""
    cv2.rectangle(tile, (0, 0), (tile.shape[1], 24), (0, 0, 0), -1)
    cv2.putText(
        tile,
        label,
        (8, 17),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (230, 230, 230),
        1,
    )


def _build_detector_debug_panel(
    frame_width: int,
    panel_height: int,
    detector_debug: dict[str, Any],
) -> np.ndarray:
    """Build a dense debug panel with intermediate detector stages."""
    panel = np.zeros((panel_height, frame_width, 3), dtype=np.uint8)
    rows = 2
    cols = 3
    tile_h = panel_height // rows
    tile_w = frame_width // cols

    gray = _fit_to_tile(_to_bgr(detector_debug["gray"]), tile_w, tile_h)
    roi = _fit_to_tile(_to_bgr(detector_debug["roi"]), tile_w, tile_h)
    preprocessed = _fit_to_tile(_to_bgr(detector_debug["preprocessed"]), tile_w, tile_h)
    edges = _fit_to_tile(_to_bgr(detector_debug["edges"]), tile_w, tile_h)
    hough_vis = _fit_to_tile(_to_bgr(detector_debug["hough_vis"]), tile_w, tile_h)
    grouped_vis = _fit_to_tile(_to_bgr(detector_debug["grouped_vis"]), tile_w, tile_h)

    tiles = [
        (gray, "Gray"),
        (roi, "ROI Masked"),
        (preprocessed, "Preprocessed (CLAHE+Blur)"),
        (edges, "Canny Edges"),
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

    # Metadata block over the grouped tile for maximum context density.
    text_x = 2 * tile_w + 8
    text_y = tile_h + 42
    meta_lines = [
        f"lines={detector_debug['lines_count']}  groups={detector_debug['groups_count']}",
        f"ref_group={detector_debug['reference_group_index']}",
        f"theta_candidate={detector_debug['theta_candidate']}",
        f"sanity_ok={detector_debug['sanity_ok']}  theta_out={detector_debug['theta_output']}",
        (
            "Hough(thr/minLen/maxGap)="
            f"{detector_debug['hough_threshold']}/"
            f"{detector_debug['hough_min_line_len']}/"
            f"{detector_debug['hough_max_line_gap']}"
        ),
        (
            "Group(dAngle/midPx)/Sanity="
            f"{detector_debug['angle_threshold']}/"
            f"{detector_debug['midpoint_threshold']}/"
            f"{detector_debug['sanity_max_delta']}"
        ),
    ]
    for line in meta_lines:
        cv2.putText(
            panel,
            line,
            (text_x, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (245, 245, 245),
            1,
        )
        text_y += 18

    return panel


def _draw_overlay(
    frame: np.ndarray,
    frame_num: int,
    theta: float | None,
    servo_angle: float,
    servo_center_angle: float,
    fsm_state: str,
    show_guidance_overlay: bool,
    start_calib_threshold_deg: float,
    stop_calib_threshold_deg: float,
) -> np.ndarray:

    """Render pipeline values onto a frame before writing to output video."""
    cv2.rectangle(frame, (8, 8), (660, 240), (0, 0, 0), -1)
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

    if show_guidance_overlay:
        servo_offset = servo_angle - servo_center_angle
        if abs(servo_offset) < 1.0:
            direction = "STRAIGHT"
        elif servo_offset > 0:
            direction = "RIGHT"
        else:
            direction = "LEFT"

        cv2.putText(
            frame,
            f"Direction: {direction} ({servo_offset:+.2f} deg)",
            (16, 145),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (180, 220, 255),
            2,
        )
        cv2.putText(
            frame,
            f"Start calibrating threshold: +/-{start_calib_threshold_deg:.1f} deg",
            (16, 170),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.56,
            (120, 230, 255),
            1,
        )
        cv2.putText(
            frame,
            f"Stop calibrating threshold: +/-{stop_calib_threshold_deg:.1f} deg",
            (16, 192),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.56,
            (120, 255, 150),
            1,
        )

        # Draw a theta gauge [0, 180] with accepted and stop-calibrating zones.
        gauge_x0 = 16
        gauge_x1 = 640
        gauge_y0 = 205
        gauge_y1 = 228
        gauge_w = gauge_x1 - gauge_x0

        def theta_to_x(theta_deg: float) -> int:
            theta_clamped = max(0.0, min(180.0, theta_deg))
            return gauge_x0 + int((theta_clamped / 180.0) * gauge_w)

        cv2.rectangle(frame, (gauge_x0, gauge_y0), (gauge_x1, gauge_y1), (60, 60, 60), 1)

        accepted_l = theta_to_x(90.0 - start_calib_threshold_deg)
        accepted_r = theta_to_x(90.0 + start_calib_threshold_deg)
        stop_l = theta_to_x(90.0 - stop_calib_threshold_deg)
        stop_r = theta_to_x(90.0 + stop_calib_threshold_deg)

        # Accepted region (outer band)
        cv2.rectangle(frame, (accepted_l, gauge_y0 + 1), (accepted_r, gauge_y1 - 1), (60, 160, 220), -1)
        # Stop-calibrating region (inner band)
        cv2.rectangle(frame, (stop_l, gauge_y0 + 1), (stop_r, gauge_y1 - 1), (60, 220, 120), -1)

        # Center and current theta markers
        center_x = theta_to_x(90.0)
        cv2.line(frame, (center_x, gauge_y0 - 3), (center_x, gauge_y1 + 3), (255, 255, 255), 1)
        if theta is not None:
            theta_x = theta_to_x(theta)
            cv2.line(frame, (theta_x, gauge_y0 - 5), (theta_x, gauge_y1 + 5), (0, 0, 255), 2)

        cv2.putText(frame, "0", (gauge_x0 - 2, gauge_y0 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)
        cv2.putText(frame, "90", (center_x - 12, gauge_y0 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 220, 220), 1)
        cv2.putText(frame, "180", (gauge_x1 - 22, gauge_y0 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)

        legend = "Blue=accepted region | Green=stop-calibrating region | Red=current theta"
        cv2.putText(frame, legend, (16, 238), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (220, 220, 220), 1)

    return frame


def _print_progress(frame_num: int, total_frames: int) -> None:
    """Print an in-place progress bar to the terminal."""
    if total_frames <= 0:
        print(f"\rProcessed frames: {frame_num}", end="", flush=True)
        return
    width = 32
    ratio = min(frame_num / total_frames, 1.0)
    filled = int(width * ratio)
    bar = "#" * filled + "-" * (width - filled)
    percent = ratio * 100.0
    print(
        f"\r[{bar}] {frame_num}/{total_frames} ({percent:5.1f}%)",
        end="",
        flush=True,
    )


def _init_csv_logger(path: str) -> tuple[csv.DictWriter, TextIO]:
    """Open (or create) *path* and return a ``(writer, file)`` pair.

    Writes the CSV header row if the file does not yet exist.

    Args:
        path: Filesystem path for the CSV log file.

    Returns:
        Tuple of ``(DictWriter, file_object)`` so the caller can close the
        file on shutdown.
    """
    file_exists = os.path.isfile(path)
    csv_file = open(path, "a", newline="")  # noqa: SIM115
    writer = csv.DictWriter(csv_file, fieldnames=_CSV_FIELDNAMES)
    if not file_exists:
        writer.writeheader()
        csv_file.flush()
    return writer, csv_file


def process_video(
    video_path: str,
    csv_output: str = "video_log.csv",
    video_output: str = "processed_video.mp4",
    send_to_servo: bool = True,
    terminal_log: bool = False,
    show_guidance_overlay: bool = False,
    show_detector_debug: bool = False,
    start_calib_threshold_deg: float = _DEFAULT_START_CALIB_THRESHOLD_DEG,
    stop_calib_threshold_deg: float = _DEFAULT_STOP_CALIB_THRESHOLD_DEG,
) -> None:
    """Process a video file through the heading-hold control pipeline.

    Args:
        video_path: Path to the video file.
        csv_output: Path for the output CSV log file.
        video_output: Path for output annotated video.
        send_to_servo: If True, send computed servo angles to the servo.
                      If False, only log the computed values.
        terminal_log: If True, show INFO logs in terminal.
        show_guidance_overlay: If True, draw direction and threshold regions.
        show_detector_debug: If True, render detailed detector stage panels.
        start_calib_threshold_deg: Outer threshold (accepted region) around 90°.
        stop_calib_threshold_deg: Inner threshold (stop-calibrating region).
    """
    if start_calib_threshold_deg <= 0 or stop_calib_threshold_deg <= 0:
        raise ValueError("Calibration thresholds must be positive.")
    if stop_calib_threshold_deg > start_calib_threshold_deg:
        raise ValueError(
            "stop_calib_threshold_deg must be <= start_calib_threshold_deg."
        )

    _configure_terminal_logging(terminal_log)

    # ---------------------------------------------------------------------- #
    # Initialise shared state and subsystems
    # ---------------------------------------------------------------------- #
    state = RobotState()
    detector = LineDetector(state)
    controller = ServoPID(state)
    servo = ServoDriver() if send_to_servo else None
    csv_writer, csv_file = _init_csv_logger(csv_output)

    # ---------------------------------------------------------------------- #
    # Video initialisation
    # ---------------------------------------------------------------------- #
    try:
        cap, fps, total_frames, frame_width, frame_height = _init_video(video_path)
        debug_panel_height = 0
        if show_detector_debug:
            debug_panel_height = max(frame_height // 2, 240)
            if debug_panel_height % 2 != 0:
                debug_panel_height += 1
        out_height = frame_height + debug_panel_height
        video_writer = _init_video_writer(video_output, fps, frame_width, out_height)
    except RuntimeError as exc:
        logger.critical("Video initialisation failed: %s", exc)
        if servo:
            servo.center()
        sys.exit(1)

    # ---------------------------------------------------------------------- #
    # Main processing loop
    # ---------------------------------------------------------------------- #
    logger.info(
        "Starting video processing pipeline (csv=%s, video=%s, detector_debug=%s).",
        csv_output,
        video_output,
        show_detector_debug,
    )
    frame_num = 0
    try:
        while True:
            loop_start = time.monotonic()

            # --- 1. Capture frame ----------------------------------------- #
            ret, frame = cap.read()
            if not ret:
                logger.info("End of video reached.")
                break
            frame_num += 1

            # --- 2. Vision: detect reference tile-gap angle --------------- #
            detector_debug: dict[str, Any] | None = None
            if show_detector_debug:
                theta, detector_debug = detector.get_reference_angle_debug(frame)
            else:
                theta = detector.get_reference_angle(frame)
            logger.info(
                "frame=%d  theta=%s  state=%s",
                frame_num,
                f"{theta:.2f}°" if theta is not None else "None",
                state.fsm_state.name,
            )

            # --- 3. Control: compute servo angle -------------------------- #
            try:
                servo_angle = controller.update(theta)
            except Exception as ctrl_exc:  # noqa: BLE001
                logger.error(
                    "Controller error on frame %d: %s – stopping.",
                    frame_num, ctrl_exc,
                )
                break

            # --- 4. Servo: send angle command (optional) ------------------- #
            if servo:
                try:
                    servo.send_angle(servo_angle)
                except OSError as hw_exc:
                    logger.error(
                        "Servo hardware error on frame %d: %s – stopping.",
                        frame_num, hw_exc,
                    )
                    break

            # --- 5. CSV log ----------------------------------------------- #
            csv_writer.writerow({
                "frame_num": frame_num,
                "timestamp": f"{loop_start:.6f}",
                "fsm_state": state.fsm_state.name,
                "theta": f"{theta:.4f}" if theta is not None else "",
                "servo_angle": f"{servo_angle:.4f}",
                "pid_integral": f"{state.pid_integral:.6f}",
                "pid_last_error": f"{state.pid_last_error:.6f}",
            })
            csv_file.flush()

            # --- 6. Annotated video output ------------------------------- #
            annotated = _draw_overlay(
                frame=frame,
                frame_num=frame_num,
                theta=theta,
                servo_angle=servo_angle,
                servo_center_angle=state.servo_center_angle,
                fsm_state=state.fsm_state.name,
                show_guidance_overlay=show_guidance_overlay,
                start_calib_threshold_deg=start_calib_threshold_deg,
                stop_calib_threshold_deg=stop_calib_threshold_deg,
            )
            output_frame = annotated
            if show_detector_debug and detector_debug is not None:
                panel = _build_detector_debug_panel(
                    frame_width=frame_width,
                    panel_height=debug_panel_height,
                    detector_debug=detector_debug,
                )
                output_frame = cv2.vconcat([annotated, panel])

            video_writer.write(output_frame)

            # --- 7. Progress bar ----------------------------------------- #
            _print_progress(frame_num, total_frames)

    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received – shutting down.")
    except Exception as fatal_exc:  # noqa: BLE001
        logger.critical(
            "Fatal error on frame %d: %s", frame_num, fatal_exc
        )
        if servo:
            try:
                servo.center()
            except Exception as center_exc:  # noqa: BLE001
                logger.error("Failed to center servo during shutdown: %s", center_exc)
        raise
    finally:
        print()
        if servo:
            servo.center()
        cap.release()
        video_writer.release()
        csv_file.close()
        logger.info("Processing complete. Resources released.")


def main() -> None:
    """Entry point with command-line argument parsing."""
    parser = argparse.ArgumentParser(
        description="Process a video file through the heading-hold control pipeline."
    )
    parser.add_argument(
        "video_path",
        help="Path to the input video file.",
    )
    parser.add_argument(
        "--output", "-o",
        default="video_log.csv",
        help="Path for the output CSV log file (default: video_log.csv).",
    )
    parser.add_argument(
        "--video-output", "-v",
        default="processed_video.mp4",
        help="Path for output annotated video (default: processed_video.mp4).",
    )
    parser.add_argument(
        "--no-servo",
        action="store_true",
        help="Do not send commands to the servo; only log computed values.",
    )
    parser.add_argument(
        "--terminal-log",
        action="store_true",
        help="Show INFO logs in terminal while processing.",
    )
    parser.add_argument(
        "--show-guidance-overlay",
        action="store_true",
        help=(
            "Show car direction, start/stop calibrating thresholds, and "
            "accepted/stop regions on output video."
        ),
    )
    parser.add_argument(
        "--show-detector-debug",
        action="store_true",
        help=(
            "Render detailed detector stages: gray, ROI, preprocessing, "
            "edges, Hough lines, grouped lines, and stage metadata."
        ),
    )
    parser.add_argument(
        "--start-calib-threshold",
        type=float,
        default=_DEFAULT_START_CALIB_THRESHOLD_DEG,
        help=(
            "Start calibrating threshold in degrees around 90 (outer accepted "
            "region, default: 5.0)."
        ),
    )
    parser.add_argument(
        "--stop-calib-threshold",
        type=float,
        default=_DEFAULT_STOP_CALIB_THRESHOLD_DEG,
        help=(
            "Stop calibrating threshold in degrees around 90 (inner stop "
            "region, default: 3.0)."
        ),
    )

    args = parser.parse_args()

    if not os.path.isfile(args.video_path):
        logger.error("Video file not found: %s", args.video_path)
        sys.exit(1)

    process_video(
        video_path=args.video_path,
        csv_output=args.output,
        video_output=args.video_output,
        send_to_servo=not args.no_servo,
        terminal_log=args.terminal_log,
        show_guidance_overlay=args.show_guidance_overlay,
        show_detector_debug=args.show_detector_debug,
        start_calib_threshold_deg=args.start_calib_threshold,
        stop_calib_threshold_deg=args.stop_calib_threshold,
    )


if __name__ == "__main__":
    main()
