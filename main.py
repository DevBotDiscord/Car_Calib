"""Entry point for the autonomous robot heading-hold system.

Orchestrates the 30 Hz control loop:

1. Capture a frame from the camera.
2. Detect the reference tile-gap angle via
   :class:`~vision.detector.LineDetector`.
3. Compute the servo angle via :class:`~control.servo_pid.ServoPID`.
4. Send the angle to the servo via :class:`~drivers.servo_driver.ServoDriver`.

Error handling:
- Camera initialisation failure triggers an immediate emergency stop and exit.
- Any critical failure in the main loop centres the servo and exits cleanly.
- Per-frame hardware errors are logged but do not terminate the loop.

State changes and PID values are logged to a CSV file (``run_log.csv``) in
addition to the standard text log.
"""

import logging
import sys
import time
from datetime import datetime, timezone
from typing import Any

import cv2

from config.settings import (
    MAIN_CAMERA_INDEX,
    MAIN_CAMERA_RETRY_LIMIT,
    MAIN_CSV_LOG_FILE,
    CTRL_HYSTERESIS_HIGH,
    CTRL_HYSTERESIS_LOW,
    MAIN_DEBUG_MODE,
    MAIN_DEBUG_FRAME_SCALE,
    MAIN_DEBUG_OVERLAY_SCALE,
    MAIN_DEBUG_VIDEO_OUTPUT,
    MAIN_FLIP_FRAME,
    MAIN_HARDWARE_RETRY_LIMIT,
    MAIN_HTTPS_CERT_FILE,
    MAIN_HTTPS_KEY_FILE,
    MAIN_HTTPS_SELF_SIGNED_DAYS,
    MAIN_HTTPS_SNAPSHOT_PATH,
    MAIN_HTTPS_STATUS_PATH,
    MAIN_HTTPS_STREAM_ENABLED,
    MAIN_HTTPS_STREAM_HOST,
    MAIN_HTTPS_STREAM_PATH,
    MAIN_HTTPS_STREAM_PORT,
    MAIN_HTTPS_STREAM_PUBLIC,
    MAIN_HTTPS_TOKEN,
    MAIN_SHOW_DETECTOR_DEBUG,
    MAIN_SHOW_GUIDANCE_OVERLAY,
    MAIN_SHOW_PREVIEW,
    MAIN_TERMINAL_LOG,
    MAIN_TARGET_HZ,
    MAIN_VIDEO_OUTPUT_FPS,
    MAIN_VIDEO_RETRY_LIMIT,
    MAIN_WRITE_DEBUG_VIDEO,
)
from control.servo_pid import ServoPID
from drivers.base_driver import BaseDriver
from drivers.relay_driver import RelayDriver
from drivers.servo_driver import ServoDriver
from models.robot_state import RobotState
from runtime.https_stream import HttpsMjpegServer, SharedFrameStore, ensure_self_signed_cert
from runtime.route_logging import RouteSession
from runtime.video_runtime_helpers import (
    build_detector_debug_panel,
    build_main_arg_parser,
    configure_terminal_logging,
    draw_overlay,
    init_camera_with_retries,
    init_csv_logger,
    init_video_writer,
    maybe_flip_frame,
    resolve_show_preview,
    sleep_remainder,
)
from vision.detector import LineDetector

# Input device + control (optional, graceful fallback)
try:
    from control.input_controller import InputController
    from runtime.gamepad_handler import InputDeviceHandler
    _INPUT_ENABLED = True
except ImportError:
    InputController = None  # type: ignore[assignment]
    InputDeviceHandler = None  # type: ignore[assignment]
    _INPUT_ENABLED = False

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
_TARGET_HZ: float = MAIN_TARGET_HZ
_LOOP_PERIOD: float = 1.0 / _TARGET_HZ
_CAMERA_INDEX: int = MAIN_CAMERA_INDEX
_CSV_LOG_FILE: str = MAIN_CSV_LOG_FILE
_FLIP_FRAME: bool = MAIN_FLIP_FRAME
_CSV_FIELDNAMES = [
    "route_id",
    "route_mode",
    "frame_num",
    "mono_timestamp",
    "utc_timestamp",
    "loop_ms",
    "loop_overrun_ms",
    "fsm_state",
    "calibration_active",
    "theta",
    "theta_source",
    "theta_for_overlay",
    "theta_horizontal",
    "reference_group_index",
    "selected_group_bbox",
    "lateral_probe_y",
    "lateral_left_x",
    "lateral_right_x",
    "lateral_tile_center_x",
    "lateral_tile_width_px",
    "lateral_offset_px",
    "lateral_offset_norm",
    "lateral_status",
    "lines_count",
    "groups_count",
    "horizontal_ok",
    "sanity_ok",
    "stale_output",
    "servo_angle",
    "servo_center_angle",
    "servo_offset",
    "pid_error",
    "angle_diff",
    "calib_status",
    "direction",
    "pid_p_term",
    "pid_i_term",
    "pid_d_term",
    "pid_integral",
    "pid_last_error",
    "hardware_send_latency_ms",
    "stream_enabled",
    "stream_host",
    "stream_port",
]


def _format_bbox(bbox: tuple[int, int, int, int] | None) -> str:
    if bbox is None:
        return ""
    x, y, w, h = bbox
    return f"{x},{y},{w},{h}"


def main() -> None:
    """Run the 30 Hz heading-hold control loop."""
    parser = build_main_arg_parser(
        csv_output_default=_CSV_LOG_FILE,
        debug_mode_default=MAIN_DEBUG_MODE,
        terminal_log_default=MAIN_TERMINAL_LOG,
        show_preview_default=MAIN_SHOW_PREVIEW,
        show_guidance_overlay_default=MAIN_SHOW_GUIDANCE_OVERLAY,
        show_detector_debug_default=MAIN_SHOW_DETECTOR_DEBUG,
        write_debug_video_default=MAIN_WRITE_DEBUG_VIDEO,
        debug_video_output_default=MAIN_DEBUG_VIDEO_OUTPUT,
        flip_frame_default=_FLIP_FRAME,
        stream_enabled_default=MAIN_HTTPS_STREAM_ENABLED,
        stream_host_default=MAIN_HTTPS_STREAM_HOST,
        stream_port_default=MAIN_HTTPS_STREAM_PORT,
        stream_public_default=MAIN_HTTPS_STREAM_PUBLIC,
        stream_token_default=MAIN_HTTPS_TOKEN,
        frame_scale_default=MAIN_DEBUG_FRAME_SCALE,
        overlay_scale_default=MAIN_DEBUG_OVERLAY_SCALE,
        camera_retry_limit_default=MAIN_CAMERA_RETRY_LIMIT,
        video_retry_limit_default=MAIN_VIDEO_RETRY_LIMIT,
        hardware_retry_limit_default=MAIN_HARDWARE_RETRY_LIMIT,
    )
    args = parser.parse_args()
    args.show_preview = resolve_show_preview(
        show_preview=args.show_preview,
        debug_mode=args.debug_mode,
        argv=sys.argv[1:],
    )
    configure_terminal_logging(args.terminal_log)

    state = RobotState()
    detector = LineDetector(state)
    controller = ServoPID(state)
    servo = ServoDriver()
    csv_writer, csv_file = init_csv_logger(args.csv_output, _CSV_FIELDNAMES)

    # Input device + control setup (optional, graceful fallback)
    input_handler: Any | None = None
    input_controller: Any | None = None
    base_driver = BaseDriver()
    relay_driver = RelayDriver()
    if _INPUT_ENABLED and InputDeviceHandler is not None:
        from config.settings import (
            GAMEPAD_DEVICE,
            GAMEPAD_NAME_HINTS,
            KEYBOARD_DEVICE,
        )
        input_handler = InputDeviceHandler(
            keyboard_device_path=KEYBOARD_DEVICE,
            gamepad_device_path=GAMEPAD_DEVICE,
            gamepad_name_hints=tuple(GAMEPAD_NAME_HINTS.split(",")),
        )
        input_handler.setup()
        logger.info(
            "INPUT: devices detected gamepad=%s keyboard=%s",
            input_handler.has_gamepad,
            input_handler.has_keyboard,
        )
        if InputController is not None and (input_handler.has_gamepad or input_handler.has_keyboard):
            input_controller = InputController(input_handler)
            logger.info("INPUT: controller online")
        else:
            logger.info("INPUT: no input devices found, gamepad control disabled")

    cap = None
    video_writer = None
    stream_server = None
    frame_store = SharedFrameStore()
    frame_num = 0
    last_known_theta: float | None = None
    consecutive_hw_errors = 0
    consecutive_video_errors = 0
    stream_host = ""
    preview_available = args.show_preview
    route_session: RouteSession | None = None
    route_csv_path = None
    route_video_path = None
    route_csv_writer = None
    route_csv_file = None
    final_status = "COMPLETED"
    rejection_reason = ""
    current_route_mode = "AUTO"

    def _detect_route_mode() -> str:
        if input_controller is None:
            return "AUTO"
        if input_controller.cruise_active:
            return "CRUISE"
        if input_controller.square_pattern_active:
            return "SQUARE"
        if input_controller.controller_remote_steer_only:
            return "REMOTE_STEER"
        return "AUTO"

    def start_route_session() -> None:
        nonlocal route_session, route_csv_path, route_video_path, route_csv_writer, route_csv_file, current_route_mode
        if route_session is not None:
            return
        current_route_mode = _detect_route_mode()
        route_session = RouteSession(route_mode=current_route_mode)
        route_csv_path = route_session.route_dir / "route_frames.csv"
        route_video_path = route_session.route_dir / "route_video.mp4"
        route_csv_writer, route_csv_file = init_csv_logger(str(route_csv_path), _CSV_FIELDNAMES)
        logger.info("Route session started: id=%s mode=%s dir=%s", route_session.route_id, route_session.route_mode, route_session.route_dir)

    def finalize_route_session(status: str, reason: str = "") -> None:
        nonlocal route_session, route_csv_writer, route_csv_file, route_csv_path, route_video_path, video_writer
        if route_session is None:
            return
        if video_writer is not None:
            video_writer.release()
            video_writer = None
        if route_csv_file is not None:
            route_csv_file.close()
        summary = route_session.finalize(
            mono_now=time.monotonic(),
            status=status,
            explicit_rejection_reason=reason,
        )
        logger.info(
            "Route summary saved: id=%s mode=%s status=%s accepted=%s reason=%s path=%s",
            summary.route_id,
            summary.route_mode,
            status,
            summary.accepted,
            summary.rejection_reason or "-",
            summary.summary_path,
        )
        if route_csv_path is not None:
            logger.info("Route frame CSV saved: %s", route_csv_path)
        if args.write_debug_video and route_video_path is not None:
            logger.info("Route video saved: %s", route_video_path)
        route_session = None
        route_csv_writer = None
        route_csv_file = None
        route_csv_path = None
        route_video_path = None

    if args.stream_enabled:
        stream_host = "0.0.0.0" if args.stream_public else args.host
        ensure_self_signed_cert(
            cert_file=MAIN_HTTPS_CERT_FILE,
            key_file=MAIN_HTTPS_KEY_FILE,
            host=stream_host,
            valid_days=MAIN_HTTPS_SELF_SIGNED_DAYS,
        )
        stream_server = HttpsMjpegServer(
            host=stream_host,
            port=args.port,
            stream_path=MAIN_HTTPS_STREAM_PATH,
            snapshot_path=MAIN_HTTPS_SNAPSHOT_PATH,
            status_path=MAIN_HTTPS_STATUS_PATH,
            token=args.stream_token,
            cert_file=MAIN_HTTPS_CERT_FILE,
            key_file=MAIN_HTTPS_KEY_FILE,
            frame_store=frame_store,
        )
        stream_server.start()
        logger.info("HTTPS MJPEG stream online: %s", stream_server.stream_url())

    camera_candidates: list[int] = []
    for idx in [_CAMERA_INDEX, 0, 1, 2, 3, 4, 5]:
        if idx not in camera_candidates:
            camera_candidates.append(idx)

    opened_index: int | None = None
    last_camera_error: RuntimeError | None = None
    for candidate in camera_candidates:
        try:
            cap = init_camera_with_retries(
                candidate,
                retries=max(0, args.camera_retry_limit),
                logger=logger,
            )
            opened_index = candidate
            break
        except RuntimeError as exc:
            last_camera_error = exc
            logger.warning("Camera probe failed at index=%d: %s", candidate, exc)

    if cap is None or opened_index is None:
        logger.critical("Camera auto-probe failed. Last error: %s", last_camera_error)
        servo.center()
        sys.exit(1)

    logger.info("Camera initialised (index=%d).", opened_index)

    logger.info("Starting heading-hold control loop at %.0f Hz.", _TARGET_HZ)
    try:
        while True:
            loop_start = time.monotonic()
            frame_num += 1

            ret, frame = cap.read()
            if not ret:
                logger.warning("Frame capture failed; skipping cycle.")
                sleep_remainder(loop_start, _LOOP_PERIOD, logger)
                continue

            frame = maybe_flip_frame(frame, args.flip_frame)

            detector_debug: dict[str, Any] | None = None
            if args.debug_mode:
                theta, detector_debug = detector.get_reference_angle_debug(frame)
            else:
                theta = detector.get_reference_angle(frame)
            lateral_offset_norm = detector.last_lateral_offset_norm
            lateral_offset_px = detector.last_lateral_offset_px

            theta_source = "live" if theta is not None else "none"
            if theta is not None:
                last_known_theta = theta
            elif last_known_theta is not None:
                theta_source = "stale"

            logger.info(
                "frame=%d timestamp=%.3f  theta=%s  state=%s",
                frame_num,
                loop_start,
                f"{theta:.2f} deg" if theta is not None else "None",
                state.fsm_state.name,
            )

            pid_error = (theta - 90.0) if theta is not None else state.pid_last_error
            dt_est = max(1e-6, time.monotonic() - loop_start)
            pid_p_term = state.pid.kp * pid_error
            pid_i_term = state.pid.ki * state.pid_integral
            pid_d_term = 0.0 if theta is None else state.pid.kd * ((pid_error - state.pid_last_error) / dt_est)
            angle_diff = abs(theta - 90.0) if theta is not None else None
            if theta is None:
                direction = "UNKNOWN"
            elif theta > 90.0:
                direction = "RIGHT"
            elif theta < 90.0:
                direction = "LEFT"
            else:
                direction = "STRAIGHT"
            calib_status = "MANUAL_OR_IDLE"

            try:
                servo_angle = controller.update(theta, lateral_offset_norm=lateral_offset_norm)
            except Exception as ctrl_exc:  # noqa: BLE001
                logger.error(
                    "Controller error: %s - centering servo and stopping.",
                    ctrl_exc,
                )
                final_status = "FAILED_CONTROL"
                rejection_reason = f"controller_error:{type(ctrl_exc).__name__}"
                servo.center()
                break

            # --- input controller override ---
            if input_controller is not None:
                try:
                    input_handler.poll()
                except Exception:
                    pass
                decision = input_controller.process(time.monotonic())

                # base motor
                if decision.base_command:
                    cmd = decision.base_command
                    if cmd == "FORWARD":
                        base_driver.forward()
                    elif cmd == "BACKWARD":
                        base_driver.backward()
                    elif cmd == "STOP":
                        base_driver.stop()
                    elif cmd == "LOCK":
                        base_driver.lock_base()
                    elif cmd == "UNLOCK":
                        base_driver.unlock_base()

                # relay
                if decision.relay_command == "ON":
                    relay_driver.on()
                elif decision.relay_command == "OFF":
                    relay_driver.off()

                # manual steer override
                if decision.steer_angle is not None:
                    servo_angle = decision.steer_angle

                auto_route_active = bool(
                    input_controller.controller_remote_steer_only
                    or input_controller.cruise_active
                    or input_controller.square_pattern_active
                )
                if auto_route_active and route_session is None:
                    start_route_session()
                if (not auto_route_active) and route_session is not None:
                    finalize_route_session(status="COMPLETED")

            if route_session is not None:
                angle_diff, calib_status, direction = route_session.update_frame(
                    mono_now=loop_start,
                    theta=theta,
                    fsm_state=state.fsm_state.name,
                    calibration_active=state.calibration_active,
                )

            try:
                send_start = time.monotonic()
                servo.send_angle(servo_angle)
                hardware_send_latency_ms = (time.monotonic() - send_start) * 1000.0
                consecutive_hw_errors = 0
            except OSError as hw_exc:
                hardware_send_latency_ms = -1.0
                consecutive_hw_errors += 1
                logger.error(
                    "Servo hardware error (%d/%d): %s",
                    consecutive_hw_errors,
                    max(1, args.hardware_retry_limit),
                    hw_exc,
                )
                if route_session is not None:
                    route_session.record_hw_error()
                if consecutive_hw_errors >= max(1, args.hardware_retry_limit):
                    logger.error("Hardware retry limit reached. Abandoning session.")
                    final_status = "INTERRUPTED_HARDWARE"
                    rejection_reason = "critical_hardware_error"
                    servo.center()
                    break
                sleep_remainder(loop_start, _LOOP_PERIOD, logger)
                continue

            mono_now = time.monotonic()
            elapsed_ms = (mono_now - loop_start) * 1000.0
            overrun_ms = max(0.0, elapsed_ms - (_LOOP_PERIOD * 1000.0))
            utc_timestamp = datetime.now(timezone.utc).isoformat()

            selected_group_bbox = detector_debug.get("selected_group_bbox") if detector_debug else None

            csv_writer.writerow(
                {
                    "route_id": route_session.route_id if route_session is not None else "",
                    "route_mode": route_session.route_mode if route_session is not None else "",
                    "frame_num": frame_num,
                    "mono_timestamp": f"{loop_start:.6f}",
                    "utc_timestamp": utc_timestamp,
                    "loop_ms": f"{elapsed_ms:.4f}",
                    "loop_overrun_ms": f"{overrun_ms:.4f}",
                    "fsm_state": state.fsm_state.name,
                    "calibration_active": int(state.calibration_active),
                    "theta": f"{theta:.4f}" if theta is not None else "",
                    "theta_source": theta_source,
                    "theta_for_overlay": f"{last_known_theta:.4f}" if last_known_theta is not None else "",
                    "theta_horizontal": (
                        f"{detector_debug.get('theta_horizontal'):.4f}"
                        if detector_debug and detector_debug.get("theta_horizontal") is not None
                        else ""
                    ),
                    "reference_group_index": (
                        detector_debug.get("reference_group_index", "") if detector_debug else ""
                    ),
                    "selected_group_bbox": _format_bbox(selected_group_bbox),
                    "lateral_probe_y": detector_debug.get("lateral_probe_y", "") if detector_debug else "",
                    "lateral_left_x": detector_debug.get("lateral_left_x", "") if detector_debug else "",
                    "lateral_right_x": detector_debug.get("lateral_right_x", "") if detector_debug else "",
                    "lateral_tile_center_x": detector_debug.get("lateral_tile_center_x", "") if detector_debug else "",
                    "lateral_tile_width_px": detector_debug.get("lateral_tile_width_px", "") if detector_debug else "",
                    "lateral_offset_px": (
                        detector_debug.get("lateral_offset_px", "")
                        if detector_debug
                        else (f"{lateral_offset_px:.4f}" if lateral_offset_px is not None else "")
                    ),
                    "lateral_offset_norm": (
                        detector_debug.get("lateral_offset_norm", "")
                        if detector_debug
                        else (f"{lateral_offset_norm:.6f}" if lateral_offset_norm is not None else "")
                    ),
                    "lateral_status": detector_debug.get("lateral_status", "") if detector_debug else "",
                    "lines_count": detector_debug.get("lines_count", "") if detector_debug else "",
                    "groups_count": detector_debug.get("groups_count", "") if detector_debug else "",
                    "horizontal_ok": detector_debug.get("horizontal_ok", "") if detector_debug else "",
                    "sanity_ok": detector_debug.get("sanity_ok", "") if detector_debug else "",
                    "stale_output": detector_debug.get("stale_output", "") if detector_debug else "",
                    "servo_angle": f"{servo_angle:.4f}",
                    "servo_center_angle": f"{state.servo_center_angle:.4f}",
                    "servo_offset": f"{(servo_angle - state.servo_center_angle):.4f}",
                    "pid_error": f"{pid_error:.6f}",
                    "angle_diff": f"{angle_diff:.6f}" if angle_diff is not None else "",
                    "calib_status": calib_status,
                    "direction": direction,
                    "pid_p_term": f"{pid_p_term:.6f}",
                    "pid_i_term": f"{pid_i_term:.6f}",
                    "pid_d_term": f"{pid_d_term:.6f}",
                    "pid_integral": f"{state.pid_integral:.6f}",
                    "pid_last_error": f"{state.pid_last_error:.6f}",
                    "hardware_send_latency_ms": f"{hardware_send_latency_ms:.4f}",
                    "stream_enabled": int(args.stream_enabled),
                    "stream_host": stream_host,
                    "stream_port": args.port if args.stream_enabled else "",
                }
            )
            if route_session is not None and route_csv_writer is not None:
                route_csv_writer.writerow(
                    {
                        "route_id": route_session.route_id,
                        "route_mode": route_session.route_mode,
                        "frame_num": frame_num,
                        "mono_timestamp": f"{loop_start:.6f}",
                        "utc_timestamp": utc_timestamp,
                        "loop_ms": f"{elapsed_ms:.4f}",
                        "loop_overrun_ms": f"{overrun_ms:.4f}",
                        "fsm_state": state.fsm_state.name,
                        "calibration_active": int(state.calibration_active),
                        "theta": f"{theta:.4f}" if theta is not None else "",
                        "theta_source": theta_source,
                        "theta_for_overlay": f"{last_known_theta:.4f}" if last_known_theta is not None else "",
                        "theta_horizontal": (
                            f"{detector_debug.get('theta_horizontal'):.4f}"
                            if detector_debug and detector_debug.get("theta_horizontal") is not None
                            else ""
                        ),
                        "reference_group_index": (
                            detector_debug.get("reference_group_index", "") if detector_debug else ""
                        ),
                        "selected_group_bbox": _format_bbox(selected_group_bbox),
                        "lateral_probe_y": detector_debug.get("lateral_probe_y", "") if detector_debug else "",
                        "lateral_left_x": detector_debug.get("lateral_left_x", "") if detector_debug else "",
                        "lateral_right_x": detector_debug.get("lateral_right_x", "") if detector_debug else "",
                        "lateral_tile_center_x": detector_debug.get("lateral_tile_center_x", "") if detector_debug else "",
                        "lateral_tile_width_px": detector_debug.get("lateral_tile_width_px", "") if detector_debug else "",
                        "lateral_offset_px": (
                            detector_debug.get("lateral_offset_px", "")
                            if detector_debug
                            else (f"{lateral_offset_px:.4f}" if lateral_offset_px is not None else "")
                        ),
                        "lateral_offset_norm": (
                            detector_debug.get("lateral_offset_norm", "")
                            if detector_debug
                            else (f"{lateral_offset_norm:.6f}" if lateral_offset_norm is not None else "")
                        ),
                        "lateral_status": detector_debug.get("lateral_status", "") if detector_debug else "",
                        "lines_count": detector_debug.get("lines_count", "") if detector_debug else "",
                        "groups_count": detector_debug.get("groups_count", "") if detector_debug else "",
                        "horizontal_ok": detector_debug.get("horizontal_ok", "") if detector_debug else "",
                        "sanity_ok": detector_debug.get("sanity_ok", "") if detector_debug else "",
                        "stale_output": detector_debug.get("stale_output", "") if detector_debug else "",
                        "servo_angle": f"{servo_angle:.4f}",
                        "servo_center_angle": f"{state.servo_center_angle:.4f}",
                        "servo_offset": f"{(servo_angle - state.servo_center_angle):.4f}",
                        "pid_error": f"{pid_error:.6f}",
                        "angle_diff": f"{angle_diff:.6f}" if angle_diff is not None else "",
                        "calib_status": calib_status,
                        "direction": direction,
                        "pid_p_term": f"{pid_p_term:.6f}",
                        "pid_i_term": f"{pid_i_term:.6f}",
                        "pid_d_term": f"{pid_d_term:.6f}",
                        "pid_integral": f"{state.pid_integral:.6f}",
                        "pid_last_error": f"{state.pid_last_error:.6f}",
                        "hardware_send_latency_ms": f"{hardware_send_latency_ms:.4f}",
                        "stream_enabled": int(args.stream_enabled),
                        "stream_host": stream_host,
                        "stream_port": args.port if args.stream_enabled else "",
                    }
                )
            csv_file.flush()
            if route_csv_file is not None:
                route_csv_file.flush()

            output_frame = frame
            if args.debug_mode:
                annotated = draw_overlay(
                    frame=frame.copy(),
                    frame_num=frame_num,
                    theta=theta,
                    theta_for_overlay=last_known_theta,
                    servo_angle=servo_angle,
                    servo_center_angle=state.servo_center_angle,
                    fsm_state=state.fsm_state.name,
                    show_guidance_overlay=args.show_guidance_overlay,
                    start_calib_threshold_deg=CTRL_HYSTERESIS_HIGH,
                    stop_calib_threshold_deg=CTRL_HYSTERESIS_LOW,
                    lateral_offset_norm=lateral_offset_norm,
                    lateral_status=(detector_debug.get("lateral_status") if detector_debug else None),
                    overlay_scale=args.overlay_scale,
                )
                if args.show_detector_debug and detector_debug is not None:
                    frame_height, frame_width = annotated.shape[:2]
                    debug_panel_height = max(frame_height // 3, 220)
                    if debug_panel_height % 2 != 0:
                        debug_panel_height += 1
                    detector_panel = build_detector_debug_panel(
                        frame_width=frame_width,
                        panel_height=debug_panel_height,
                        detector_debug=detector_debug,
                    )
                    output_frame = cv2.vconcat([annotated, detector_panel])
                else:
                    output_frame = annotated

            if args.frame_scale > 1.0:
                output_frame = cv2.resize(
                    output_frame,
                    dsize=None,
                    fx=args.frame_scale,
                    fy=args.frame_scale,
                    interpolation=cv2.INTER_CUBIC,
                )

            if args.write_debug_video and route_session is not None and route_video_path is not None:
                if video_writer is None:
                    h, w = output_frame.shape[:2]
                    video_writer = init_video_writer(
                        str(route_video_path),
                        MAIN_VIDEO_OUTPUT_FPS,
                        w,
                        h,
                    )
                    logger.info("Route video writer active: %s", route_video_path)

                try:
                    video_writer.write(output_frame)
                    consecutive_video_errors = 0
                except Exception as video_exc:  # noqa: BLE001
                    consecutive_video_errors += 1
                    logger.warning(
                        "Debug video write error (%d/%d): %s",
                        consecutive_video_errors,
                        max(1, args.video_retry_limit),
                        video_exc,
                    )
                    if consecutive_video_errors >= max(1, args.video_retry_limit):
                        logger.error("Video writer retry limit reached. Abandoning session.")
                        final_status = "FAILED_VIDEO_OUTPUT"
                        rejection_reason = "video_writer_retry_limit"
                        break

            if args.stream_enabled:
                telemetry = {
                    "frame_num": frame_num,
                    "theta": theta,
                    "theta_source": theta_source,
                    "fsm_state": state.fsm_state.name,
                    "servo_angle": servo_angle,
                    "reference_group_index": detector_debug.get("reference_group_index") if detector_debug else None,
                    "selected_group_bbox": selected_group_bbox,
                    "lateral_offset_norm": lateral_offset_norm,
                    "lateral_status": detector_debug.get("lateral_status") if detector_debug else None,
                }
                frame_store.set_frame(output_frame, telemetry)

            if preview_available:
                try:
                    cv2.imshow("main_debug", output_frame)
                    if (cv2.waitKey(1) & 0xFF) == ord("q"):
                        logger.info("Quit requested from preview window.")
                        final_status = "INTERRUPTED_MANUAL"
                        rejection_reason = "preview_quit"
                        break
                except cv2.error as preview_exc:
                    preview_available = False
                    logger.warning("Preview disabled: %s", preview_exc)

            sleep_remainder(loop_start, _LOOP_PERIOD, logger)

    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received - shutting down.")
        final_status = "INTERRUPTED_MANUAL"
        rejection_reason = "manual_interrupt"
    except Exception as fatal_exc:  # noqa: BLE001
        logger.critical("Fatal error: %s - centering servo and stopping.", fatal_exc)
        final_status = "FAILED"
        rejection_reason = f"fatal_error:{type(fatal_exc).__name__}"
        try:
            servo.center()
        except Exception as center_exc:  # noqa: BLE001
            logger.error("Failed to center servo during shutdown: %s", center_exc)
        raise
    finally:
        servo.center()
        servo.close()
        try:
            base_driver.stop()
            base_driver.close()
        except Exception:
            pass
        try:
            relay_driver.off()
            relay_driver.close()
        except Exception:
            pass
        if input_handler is not None:
            try:
                input_handler.close()
            except Exception:
                pass
        if cap is not None:
            cap.release()
        if video_writer is not None:
            video_writer.release()
            video_writer = None
        if stream_server is not None:
            stream_server.stop()
        if preview_available:
            try:
                cv2.destroyAllWindows()
            except cv2.error as close_preview_exc:
                logger.debug("Preview window cleanup skipped: %s", close_preview_exc)
        csv_file.close()
        finalize_route_session(status=final_status, reason=rejection_reason)
        logger.info("Resources released. Goodbye.")


if __name__ == "__main__":
    main()
