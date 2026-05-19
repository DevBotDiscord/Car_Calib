"""Process a video file through the heading-hold control pipeline."""

import logging
import os
import sys
import time
from typing import Any

import cv2

from config.settings import (
    PROCESS_VIDEO_CSV_OUTPUT,
    PROCESS_VIDEO_FLIP_FRAME,
    PROCESS_VIDEO_OUTPUT,
    PROCESS_VIDEO_SEND_TO_SERVO,
    PROCESS_VIDEO_SHOW_DETECTOR_DEBUG,
    PROCESS_VIDEO_SHOW_GUIDANCE_OVERLAY,
    PROCESS_VIDEO_START_CALIB_THRESHOLD_DEG,
    PROCESS_VIDEO_STOP_CALIB_THRESHOLD_DEG,
    PROCESS_VIDEO_TERMINAL_LOG,
    PROCESS_VIDEO_FRAME_SLEEP_MS,
)
from control.servo_pid import ServoPID
from drivers.servo_driver import ServoDriver
from models.robot_state import RobotState
from runtime.video_runtime_helpers import (
    build_detector_debug_panel,
    build_process_video_arg_parser,
    configure_terminal_logging,
    draw_overlay,
    init_csv_logger,
    init_video,
    init_video_writer,
    maybe_flip_frame,
    print_progress,
)
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
_CSV_FIELDNAMES = ["frame_num", "timestamp", "fsm_state", "theta", "servo_angle", "pid_integral", "pid_last_error"]



def process_video(
    video_path: str,
    csv_output: str = PROCESS_VIDEO_CSV_OUTPUT,
    video_output: str = PROCESS_VIDEO_OUTPUT,
    send_to_servo: bool = PROCESS_VIDEO_SEND_TO_SERVO,
    terminal_log: bool = PROCESS_VIDEO_TERMINAL_LOG,
    show_guidance_overlay: bool = PROCESS_VIDEO_SHOW_GUIDANCE_OVERLAY,
    show_detector_debug: bool = PROCESS_VIDEO_SHOW_DETECTOR_DEBUG,
    start_calib_threshold_deg: float = PROCESS_VIDEO_START_CALIB_THRESHOLD_DEG,
    stop_calib_threshold_deg: float = PROCESS_VIDEO_STOP_CALIB_THRESHOLD_DEG,
    flip_frame: bool = PROCESS_VIDEO_FLIP_FRAME,
    frame_sleep_ms: float = PROCESS_VIDEO_FRAME_SLEEP_MS,
) -> None:
    """Process a video file through the heading-hold control pipeline."""
    if start_calib_threshold_deg <= 0 or stop_calib_threshold_deg <= 0:
        raise ValueError("Calibration thresholds must be positive.")
    if stop_calib_threshold_deg > start_calib_threshold_deg:
        raise ValueError("stop_calib_threshold_deg must be <= start_calib_threshold_deg.")
    if frame_sleep_ms < 0:
        raise ValueError("frame_sleep_ms must be >= 0.")

    frame_sleep_seconds = frame_sleep_ms / 1000.0

    configure_terminal_logging(terminal_log)

    state = RobotState()
    detector = LineDetector(state)
    controller = ServoPID(
        state,
        start_calib_threshold_deg=start_calib_threshold_deg,
        stop_calib_threshold_deg=stop_calib_threshold_deg,
    )
    servo = ServoDriver() if send_to_servo else None
    csv_writer, csv_file = init_csv_logger(csv_output, _CSV_FIELDNAMES)

    try:
        cap, fps, total_frames, frame_width, frame_height = init_video(video_path, logger)

        debug_panel_height = 0
        if show_detector_debug:
            debug_panel_height = max(frame_height // 3, 220)
            if debug_panel_height % 2 != 0:
                debug_panel_height += 1

        top_height = frame_height
        out_height = top_height + debug_panel_height
        video_writer = init_video_writer(video_output, fps, frame_width, out_height)
    except RuntimeError as exc:
        logger.critical("Video initialisation failed: %s", exc)
        if servo:
            servo.center()
        sys.exit(1)

    logger.info(
        "Starting video processing pipeline (csv=%s, video=%s, detector_debug=%s, frame_sleep_ms=%.1f).",
        csv_output,
        video_output,
        show_detector_debug,
        frame_sleep_ms,
    )
    frame_num = 0
    last_known_theta: float | None = None

    try:
        while True:
            loop_start = time.monotonic()

            ret, frame = cap.read()
            if not ret:
                logger.info("End of video reached.")
                break

            frame_num += 1
            frame = maybe_flip_frame(frame, flip_frame)

            detector_debug: dict[str, Any] | None = None
            if show_detector_debug:
                theta, detector_debug = detector.get_reference_angle_debug(frame)
            else:
                theta = detector.get_reference_angle(frame)

            if theta is not None:
                last_known_theta = theta

            logger.info(
                "frame=%d  theta=%s  state=%s",
                frame_num,
                f"{theta:.2f} deg" if theta is not None else "None",
                state.fsm_state.name,
            )

            try:
                servo_angle = controller.update(theta)
            except Exception as ctrl_exc:  # noqa: BLE001
                logger.error("Controller error on frame %d: %s - stopping.", frame_num, ctrl_exc)
                break

            if servo:
                try:
                    servo.send_angle(servo_angle)
                except OSError as hw_exc:
                    logger.error("Servo hardware error on frame %d: %s - stopping.", frame_num, hw_exc)
                    break

            csv_writer.writerow(
                {
                    "frame_num": frame_num,
                    "timestamp": f"{loop_start:.6f}",
                    "fsm_state": state.fsm_state.name,
                    "theta": f"{theta:.4f}" if theta is not None else "",
                    "servo_angle": f"{servo_angle:.4f}",
                    "pid_integral": f"{state.pid_integral:.6f}",
                    "pid_last_error": f"{state.pid_last_error:.6f}",
                }
            )
            csv_file.flush()

            annotated = draw_overlay(
                frame=frame,
                frame_num=frame_num,
                theta=theta,
                theta_for_overlay=last_known_theta,
                servo_angle=servo_angle,
                servo_center_angle=state.servo_center_angle,
                fsm_state=state.fsm_state.name,
                show_guidance_overlay=show_guidance_overlay,
                start_calib_threshold_deg=start_calib_threshold_deg,
                stop_calib_threshold_deg=stop_calib_threshold_deg,
                route_id=None,
                route_mode=None,
            )

            top_frame = annotated
            if top_height != frame_height:
                top_frame = cv2.resize(annotated, (frame_width, top_height), interpolation=cv2.INTER_AREA)

            frame_stack = [top_frame]

            if show_detector_debug and detector_debug is not None:
                detector_panel = build_detector_debug_panel(
                    frame_width=frame_width,
                    panel_height=debug_panel_height,
                    detector_debug=detector_debug,
                )
                frame_stack.append(detector_panel)

            output_frame = cv2.vconcat(frame_stack)
            video_writer.write(output_frame)

            print_progress(frame_num, total_frames)
            if frame_sleep_seconds > 0:
                time.sleep(frame_sleep_seconds)

    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received - shutting down.")
    except Exception as fatal_exc:  # noqa: BLE001
        logger.critical("Fatal error on frame %d: %s", frame_num, fatal_exc)
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
            servo.close()
        cap.release()
        video_writer.release()
        csv_file.close()
        logger.info("Processing complete. Resources released.")



def main() -> None:
    """Entry point with command-line argument parsing."""
    parser = build_process_video_arg_parser(
        csv_output_default=PROCESS_VIDEO_CSV_OUTPUT,
        video_output_default=PROCESS_VIDEO_OUTPUT,
        send_to_servo_default=PROCESS_VIDEO_SEND_TO_SERVO,
        terminal_log_default=PROCESS_VIDEO_TERMINAL_LOG,
        show_guidance_overlay_default=PROCESS_VIDEO_SHOW_GUIDANCE_OVERLAY,
        show_detector_debug_default=PROCESS_VIDEO_SHOW_DETECTOR_DEBUG,
        flip_frame_default=PROCESS_VIDEO_FLIP_FRAME,
        start_calib_threshold_default=PROCESS_VIDEO_START_CALIB_THRESHOLD_DEG,
        stop_calib_threshold_default=PROCESS_VIDEO_STOP_CALIB_THRESHOLD_DEG,
        frame_sleep_ms_default=PROCESS_VIDEO_FRAME_SLEEP_MS,
    )

    args = parser.parse_args()

    if not os.path.isfile(args.video_path):
        logger.error("Video file not found: %s", args.video_path)
        sys.exit(1)

    process_video(
        video_path=args.video_path,
        csv_output=args.output,
        video_output=args.video_output,
        send_to_servo=args.send_to_servo,
        terminal_log=args.terminal_log,
        show_guidance_overlay=args.show_guidance_overlay,
        show_detector_debug=args.show_detector_debug,
        start_calib_threshold_deg=args.start_calib_threshold,
        stop_calib_threshold_deg=args.stop_calib_threshold,
        flip_frame=args.flip_frame,
        frame_sleep_ms=args.sleep_ms,
    )


if __name__ == "__main__":
    main()
