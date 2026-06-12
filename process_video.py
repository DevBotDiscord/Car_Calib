"""Offline video processing entry point for calibration runs."""

from __future__ import annotations

import logging
import os
import sys
import time
from pathlib import Path

from config.settings import (
    PROCESS_VIDEO_CSV_OUTPUT,
    PROCESS_VIDEO_FRAME_SLEEP_MS,
    PROCESS_VIDEO_OUTPUT,
    PROCESS_VIDEO_SHOW_VISION_DEBUG,
    PROCESS_VIDEO_SHOW_GUIDANCE_OVERLAY,
    PROCESS_VIDEO_TERMINAL_LOG,
)
from runtime.video_runtime_helpers import (
    build_process_video_arg_parser,
    init_video,
    print_progress,
)


def _set_env(name: str, value: object) -> None:
    """Apply an environment override used by the shared runtime."""
    os.environ[name] = str(value)


def _configure_shared_runtime(
    input_width: int,
    input_height: int,
    show_vision_debug: bool,
    show_guidance_overlay: bool,
    terminal_log: bool,
    output_name: str,
    csv_name: str,
) -> None:
    """Map process-video settings onto the shared unified runtime settings."""
    _set_env("MAIN_DEBUG_MODE", "true")
    _set_env("MAIN_DEBUG_VISUALIZER", "video")
    _set_env("MAIN_SHOW_PREVIEW", "false")
    _set_env("MAIN_WRITE_DEBUG_VIDEO", "true")
    _set_env("MAIN_DEBUG_VIDEO_OUTPUT", output_name)
    _set_env("MAIN_CSV_LOG_FILE", csv_name)
    _set_env("MAIN_SHOW_GUIDANCE_OVERLAY", str(show_guidance_overlay).lower())
    _set_env("MAIN_SHOW_VISION_DEBUG", str(show_vision_debug).lower())
    _set_env("MAIN_TERMINAL_LOG", str(terminal_log).lower())
    _set_env("MAIN_TARGET_HZ", "0.0")
    _set_env("MAIN_FLIP_FRAME", "false")
    _set_env("MAIN_CAMERA_INDEX", "0")
    _set_env("MAIN_HTTPS_STREAM_ENABLED", "false")
    _set_env("MAIN_FRAME_WIDTH", str(input_width))
    _set_env("MAIN_FRAME_HEIGHT", str(input_height + (240 if show_vision_debug else 0)))


def main() -> None:
    """Process a video file frame-by-frame and save run artifacts."""
    logging.basicConfig(
        level=logging.INFO if PROCESS_VIDEO_TERMINAL_LOG else logging.ERROR,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )

    parser = build_process_video_arg_parser(
        default_input="",
        default_output=PROCESS_VIDEO_OUTPUT,
        default_csv=PROCESS_VIDEO_CSV_OUTPUT,
    )
    args = parser.parse_args()

    if not args.input:
        parser.error("--input is required for process-video mode")

    logger = logging.getLogger(__name__)
    capture, _fps, total_frames, frame_width, frame_height = init_video(args.input, logger)
    output_name = Path(args.output).name
    csv_name = Path(args.csv_output).name

    _configure_shared_runtime(
        frame_width,
        frame_height,
        bool(PROCESS_VIDEO_SHOW_VISION_DEBUG),
        bool(PROCESS_VIDEO_SHOW_GUIDANCE_OVERLAY),
        bool(PROCESS_VIDEO_TERMINAL_LOG),
        output_name,
        csv_name,
    )

    from unified_calibration_components import CalibrationProcessingError, UnifiedCalibrator

    calibrator = UnifiedCalibrator()
    frame_num = 0
    try:
        while capture.isOpened():
            ok, frame = capture.read()
            if not ok or frame is None:
                break

            try:
                calibrator.update(frame, frame_num)
            except CalibrationProcessingError as exc:
                diagnostic = exc.diagnostic
                logger.exception(
                    "Offline calibration failure frame=%d stage=%s process=%s "
                    "error_type=%s detail=%s",
                    diagnostic.frame_num,
                    diagnostic.stage,
                    diagnostic.process,
                    diagnostic.error_type,
                    diagnostic.detail,
                )
                raise
            if PROCESS_VIDEO_FRAME_SLEEP_MS > 0:
                time.sleep(float(PROCESS_VIDEO_FRAME_SLEEP_MS) / 1000.0)

            frame_num += 1
            print_progress(frame_num, total_frames)
    finally:
        capture.release()
        calibrator.close()
        print()
        logger.info("Processed %s frame(s) from %s", frame_num, args.input)


if __name__ == "__main__":
    sys.exit(main())
