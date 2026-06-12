"""Entry point for the autonomous robot heading-hold system.

Orchestrates the 30 Hz control loop:

1. Capture a frame from the camera.
2. Compute the unified lane-pair geometry and steering result.
3. Read the resulting steering command and calibration telemetry.
4. Send the angle to the servo via :class:`~drivers.servo_driver.ServoDriver`.
   The driver publishes the angle as an integer over MQTT to the RPi bridge.

The MiniPC also subscribes to ``car/control/route`` and ``car/control/mode``
from the RPi gamepad/keyboard controller; those signals drive the route
logging session lifecycle (start/stop) and tag the captured dataset.

Error handling:
- Camera initialisation failure triggers an immediate emergency stop and exit.
- Any critical failure in the main loop centres the servo and exits cleanly.
- Per-frame hardware errors are logged but do not terminate the loop.

State changes and PID values are logged to a CSV file (``run_log.csv``) in
addition to the standard text log. When a route session is active a parallel
``route_frames.csv`` is written inside the route directory.
"""

from __future__ import annotations

import logging
import os
import time
from datetime import datetime, timezone
from typing import Any

import cv2
import paho.mqtt.client as mqtt

from config.settings import (
    CTRL_HYSTERESIS_HIGH,
    CTRL_HYSTERESIS_LOW,
    MAIN_CAMERA_INDEX,
    MAIN_CAMERA_RETRY_LIMIT,
    MAIN_CSV_LOG_FILE,
    MAIN_DEBUG_FRAME_SCALE,
    MAIN_DEBUG_MODE,
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
    MAIN_SHOW_VISION_DEBUG,
    MAIN_SHOW_GUIDANCE_OVERLAY,
    MAIN_SHOW_PREVIEW,
    MAIN_TARGET_HZ,
    MAIN_TERMINAL_LOG,
    MAIN_VIDEO_OUTPUT_FPS,
    MAIN_VIDEO_RETRY_LIMIT,
    MAIN_WRITE_DEBUG_VIDEO,
    MQTT_BASE_COMMAND_TOPIC,
    MQTT_BROKER_HOST,
    MQTT_BROKER_PORT,
    MQTT_CLIENT_ID_PREFIX,
    MQTT_KEEPALIVE_S,
    MQTT_PASSWORD,
    MQTT_USERNAME,
    ESP32_SERIAL_ENABLED,
    ESP32_SERIAL_BAUD,
    ESP32_SERIAL_PORT_GLOBS,
    ACTUATOR_MODE,
    ESP32_SCAN_TIMEOUT_S,
    ESP32_FQBN,
)
from drivers.mqtt_control_client import MQTTControlClient
from drivers.servo_driver import ServoDriver
from runtime.https_stream import HttpsMjpegServer, SharedFrameStore, ensure_self_signed_cert
from runtime.route_logging import RouteSession
from runtime.video_runtime_helpers import (
    build_vision_debug_panel,
    build_main_arg_parser,
    configure_terminal_logging,
    draw_overlay,
    init_camera_with_retries,
    init_csv_logger,
    init_live_video_writer,
    init_video_writer,
    maybe_flip_frame,
    sleep_remainder,
)
from unified_calibration_components import UnifiedCalibrator

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
    "lines_count",
    "servo_angle",
    "servo_center_angle",
    "servo_offset",
    "pid_error",
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
def main() -> None:
    """Run the 30 Hz heading-hold control loop."""
    parser = build_main_arg_parser(
        csv_output_default=_CSV_LOG_FILE,
        debug_mode_default=MAIN_DEBUG_MODE,
        terminal_log_default=MAIN_TERMINAL_LOG,
        show_preview_default=MAIN_SHOW_PREVIEW,
        show_guidance_overlay_default=MAIN_SHOW_GUIDANCE_OVERLAY,
        show_vision_debug_default=MAIN_SHOW_VISION_DEBUG,
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
    configure_terminal_logging(args.terminal_log)

    calibrator = UnifiedCalibrator(telemetry_enabled=False)
    state = calibrator.robot_state
    controller = calibrator.steering_controller
    servo = ServoDriver()
    csv_writer, csv_file = init_csv_logger(args.csv_output, _CSV_FIELDNAMES)

    cap = None
    video_writer = None
    stream_server = None
    frame_store = SharedFrameStore()
    frame_num = 0
    base_stop_client: mqtt.Client | None = None
    last_known_theta: float | None = None
    consecutive_hw_errors = 0
    consecutive_video_errors = 0
    stream_host = ""

    # ----------------------------------------------------------------- #
    # Route logging + MQTT control subscription
    # ----------------------------------------------------------------- #
    route_session: RouteSession | None = None
    route_csv_writer = None
    route_csv_file = None
    route_video_writer = None
    route_video_size: tuple[int, int] | None = None
    current_route_mode = "AUTO"
    final_status = "COMPLETED"
    rejection_reason = ""

    def start_route_session() -> None:
        nonlocal route_session, route_csv_writer, route_csv_file
        nonlocal route_video_writer, route_video_size
        if route_session is not None:
            return
        route_session = RouteSession(route_mode=current_route_mode)
        route_csv_path = route_session.route_dir / "route_frames.csv"
        route_csv_writer, route_csv_file = init_csv_logger(
            str(route_csv_path), _CSV_FIELDNAMES
        )
        route_video_writer = None
        route_video_size = None
        # Attach optional script-runner metadata (preset name, steps, ...).
        if script_runner is not None:
            try:
                meta = script_runner.consume_pending_meta()
                if meta:
                    route_session.attach_meta("script", meta)
            except Exception as exc:  # noqa: BLE001
                logger.warning("Failed to attach script meta: %s", exc)
        logger.info(
            "Route session started: id=%s mode=%s dir=%s",
            route_session.route_id,
            route_session.route_mode,
            route_session.route_dir,
        )

    def finalize_route_session(status: str, reason: str = "") -> None:
        nonlocal route_session, route_csv_writer, route_csv_file
        nonlocal route_video_writer, route_video_size
        if route_session is None:
            return
        if route_csv_file is not None:
            route_csv_file.close()
        if route_video_writer is not None:
            try:
                route_video_writer.release()
            except Exception:  # noqa: BLE001
                pass
        route_video_writer = None
        route_video_size = None
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
        route_session = None
        route_csv_writer = None
        route_csv_file = None

    def on_route_control(payload: str) -> None:
        if payload == "START":
            if route_session is None:
                start_route_session()
        elif payload == "STOP":
            if route_session is not None:
                finalize_route_session(status="COMPLETED")

    def on_mode_control(payload: str) -> None:
        nonlocal current_route_mode
        current_route_mode = payload
        logger.info("MODE: received from RPi: %s", payload)

    def setup_base_stop_client() -> mqtt.Client | None:
        client_id = f"{MQTT_CLIENT_ID_PREFIX}-camera-watchdog"
        callback_api_version = getattr(mqtt, "CallbackAPIVersion", None)
        if callback_api_version is not None:
            client = mqtt.Client(
                callback_api_version=callback_api_version.VERSION1,
                client_id=client_id,
            )
        else:
            client = mqtt.Client(client_id=client_id)
        if MQTT_USERNAME:
            client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        client.reconnect_delay_set(min_delay=1, max_delay=5)
        client.connect_async(MQTT_BROKER_HOST, MQTT_BROKER_PORT, keepalive=MQTT_KEEPALIVE_S)
        client.loop_start()
        return client

    def publish_base_stop(reason: str) -> None:
        if base_stop_client is None:
            logger.warning("Cannot publish base STOP (%s): MQTT base client unavailable", reason)
            return
        try:
            base_stop_client.publish(MQTT_BASE_COMMAND_TOPIC, "STOP", qos=0)
            logger.warning("Published base STOP: %s", reason)
        except Exception as exc:  # noqa: BLE001
            logger.error("Base STOP publish failed (%s): %s", reason, exc)

    mqtt_control_client: MQTTControlClient | None = None
    try:
        mqtt_control_client = MQTTControlClient(
            on_route=on_route_control,
            on_mode=on_mode_control,
        )
        mqtt_control_client.setup()
    except Exception as exc:  # noqa: BLE001
        logger.warning("MQTT control client setup failed: %s", exc)
        mqtt_control_client = None

    def _rpi_status_provider() -> dict[str, Any] | None:
        if mqtt_control_client is None:
            return None
        try:
            return mqtt_control_client.get_rpi_status()
        except Exception:  # noqa: BLE001
            return None

    esp32_bridge = None
    # Actuator mode: "mqtt" never starts the ESP32 bridge; "esp32" scans
    # forever (exclusive); "auto" tries the ESP32 then falls back to MQTT.
    _esp32_wanted = ESP32_SERIAL_ENABLED and ACTUATOR_MODE in ("auto", "esp32")
    if ACTUATOR_MODE == "mqtt":
        logger.info("ACTUATOR_MODE=mqtt — ESP32 serial bridge disabled, MQTT/RPi path only")
    if _esp32_wanted:
        try:
            from runtime.esp32_serial_bridge import ESP32SerialBridge
            esp32_bridge = ESP32SerialBridge(
                mqtt_host=MQTT_BROKER_HOST,
                mqtt_port=MQTT_BROKER_PORT,
                mqtt_username=MQTT_USERNAME,
                mqtt_password=MQTT_PASSWORD,
                servo_topic=MQTT_SERVO_TOPIC,
                base_topic=MQTT_BASE_COMMAND_TOPIC,
                relay_topic=MQTT_RELAY_TOPIC,
                status_topic=MQTT_STATUS_TOPIC,
                baud=ESP32_SERIAL_BAUD,
                port_globs=tuple(g for g in ESP32_SERIAL_PORT_GLOBS.split(",") if g),
                scan_timeout_s=(None if ACTUATOR_MODE == "esp32" else ESP32_SCAN_TIMEOUT_S),
                device_config={
                    "servo_pin": int(os.getenv("SERVO_PIN", "12")),
                    "min_pulse_us": int(round(float(os.getenv("SERVO_MIN_PULSE", "0.0005")) * 1_000_000)),
                    "max_pulse_us": int(round(float(os.getenv("SERVO_MAX_PULSE", "0.0025")) * 1_000_000)),
                    "center_angle": float(os.getenv("SERVO_CENTER_ANGLE", "-8")),
                    "max_angle_deg": float(os.getenv("SERVO_MAX_ANGLE_DEG", "45")),
                    "deadband_deg": float(os.getenv("STEER_DEADBAND_DEG", "1.0")),
                    "out1": int(os.getenv("BASE_OUT1", "17")),
                    "out2": int(os.getenv("BASE_OUT2", "27")),
                    "out3": int(os.getenv("BASE_OUT3", "22")),
                    "relay_pin": int(os.getenv("RELAY_PIN", "5")),
                    "estop_pin": int(os.getenv("ESTOP_GPIO", "6")),
                    "estop_active_low": os.getenv("ESTOP_ACTIVE_LOW", "true").strip().lower() in {"1", "true", "t", "yes", "y", "on"},
                    "telemetry_ms": int(float(os.getenv("TELEMETRY_INTERVAL_SEC", "1.0")) * 1000),
                },
            )
            esp32_bridge.start()
        except Exception as exc:  # noqa: BLE001
            logger.warning("ESP32 serial bridge setup failed: %s", exc)
            esp32_bridge = None

    esp32_flasher = None
    if ACTUATOR_MODE in ("auto", "esp32"):
        try:
            from runtime.esp32_flasher import ESP32Flasher
            esp32_flasher = ESP32Flasher(
                fqbn=ESP32_FQBN,
                bridge=esp32_bridge,
                port_globs=tuple(g for g in ESP32_SERIAL_PORT_GLOBS.split(",") if g),
            )
        except Exception as exc:  # noqa: BLE001
            logger.warning("ESP32 flasher setup failed: %s", exc)
            esp32_flasher = None

    try:
        base_stop_client = setup_base_stop_client()
    except Exception as exc:  # noqa: BLE001
        logger.warning("MQTT base STOP client setup failed: %s", exc)
        base_stop_client = None

    script_runner = None
    if args.stream_enabled:
        stream_host = "0.0.0.0" if args.stream_public else args.host
        ensure_self_signed_cert(
            cert_file=MAIN_HTTPS_CERT_FILE,
            key_file=MAIN_HTTPS_KEY_FILE,
            host=stream_host,
            valid_days=MAIN_HTTPS_SELF_SIGNED_DAYS,
        )
        try:
            from runtime.route_script import RouteScriptRunner
            script_runner = RouteScriptRunner()
        except Exception as exc:  # noqa: BLE001
            logger.warning("Route script runner disabled: %s", exc)
            script_runner = None
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
            script_runner=script_runner,
            rpi_status_provider=_rpi_status_provider,
            steering_controller=controller,
            esp32_bridge=esp32_bridge,
            esp32_flasher=esp32_flasher,
        )
        stream_server.start()
        logger.info("HTTPS MJPEG stream online: %s", stream_server.stream_url())
        logger.info("Dashboard URL: https://%s:%d/dashboard", stream_host, args.port)

    camera_candidates: list[int] = []
    for idx in (_CAMERA_INDEX, 0, 1, 2, 3, 4, 5):
        if idx not in camera_candidates:
            camera_candidates.append(idx)

    def acquire_camera_blocking(reason: str) -> tuple[cv2.VideoCapture, int]:
        publish_base_stop(reason)
        while True:
            last_camera_error: RuntimeError | None = None
            for candidate in camera_candidates:
                try:
                    found_cap = init_camera_with_retries(
                        candidate,
                        retries=max(0, args.camera_retry_limit),
                        logger=logger,
                    )
                    logger.info("Camera initialised (index=%d).", candidate)
                    return found_cap, candidate
                except RuntimeError as exc:
                    last_camera_error = exc
                    logger.warning("Camera probe failed at index=%d: %s", candidate, exc)
            logger.error(
                "Camera unavailable across %s. Last error: %s. Retrying; base remains STOP.",
                camera_candidates,
                last_camera_error,
            )
            publish_base_stop("camera_unavailable_retry")
            time.sleep(1.0)

    cap, opened_index = acquire_camera_blocking("camera_startup_unavailable")

    logger.info("Starting heading-hold control loop at %.0f Hz.", _TARGET_HZ)
    try:
        while True:
            loop_start = time.monotonic()
            frame_num += 1

            ret, frame = cap.read()
            if not ret or frame is None:
                logger.warning("Frame capture failed; publishing base STOP and re-scanning cameras.")
                publish_base_stop("camera_capture_failed")
                try:
                    cap.release()
                except Exception:  # noqa: BLE001
                    pass
                cap, opened_index = acquire_camera_blocking("camera_capture_failed_reacquire")
                sleep_remainder(loop_start, _LOOP_PERIOD, logger)
                continue

            frame = maybe_flip_frame(frame, args.flip_frame)

            try:
                calibration = calibrator.process_frame(frame, frame_num)
            except Exception as ctrl_exc:  # noqa: BLE001
                logger.error(
                    "Unified calibration error: %s - centering servo and stopping.",
                    ctrl_exc,
                )
                final_status = "FAILED_CONTROL"
                rejection_reason = f"controller_error:{type(ctrl_exc).__name__}"
                servo.center()
                break

            theta = calibration.observation_angle
            vision_debug = calibration.debug_data.get("vision_debug")
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
                calibration.control_state,
            )

            servo_angle = calibration.steering_angle
            fsm_state_str = calibration.control_state
            pid_error = calibration.telemetry["pid_error"]
            pid_p_term = calibration.telemetry["pid_p_term"]
            pid_i_term = calibration.telemetry["pid_i_term"]
            pid_d_term = calibration.telemetry["pid_d_term"]

            if route_session is not None:
                route_session.update_frame(
                    mono_now=loop_start,
                    theta=theta,
                    fsm_state=calibration.control_state,
                    calibration_active=calibration.calibration_active,
                )

            try:
                send_start = time.monotonic()
                # Suppress recenter publishes while a route is active so the
                # servo holds its last steered angle instead of flip-flopping
                # to CENTER on every GAPPING/COAST frame. Final recenter is
                # emitted by servo.center() in the finally/finalize block.
                suppress_send = route_session is not None and fsm_state_str in (
                    "GAPPING",
                    "TRACKING_COAST",
                )
                if not suppress_send:
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

            csv_row = {
                "route_id": route_session.route_id if route_session is not None else "",
                "route_mode": route_session.route_mode if route_session is not None else "",
                "frame_num": frame_num,
                "mono_timestamp": f"{loop_start:.6f}",
                "utc_timestamp": utc_timestamp,
                "loop_ms": f"{elapsed_ms:.4f}",
                "loop_overrun_ms": f"{overrun_ms:.4f}",
                "fsm_state": calibration.control_state,
                "calibration_active": int(calibration.calibration_active),
                "theta": f"{theta:.4f}" if theta is not None else "",
                "theta_source": theta_source,
                "theta_for_overlay": f"{last_known_theta:.4f}" if last_known_theta is not None else "",
                "lines_count": vision_debug.get("lines_count", "") if vision_debug else "",
                "servo_angle": f"{servo_angle:.4f}",
                "servo_center_angle": f"{state.servo_center_angle:.4f}",
                "servo_offset": f"{(servo_angle - state.servo_center_angle):.4f}",
                "pid_error": pid_error,
                "pid_p_term": pid_p_term,
                "pid_i_term": pid_i_term,
                "pid_d_term": pid_d_term,
                "pid_integral": calibration.telemetry["pid_integral"],
                "pid_last_error": calibration.telemetry["pid_last_error"],
                "hardware_send_latency_ms": f"{hardware_send_latency_ms:.4f}",
                "stream_enabled": int(args.stream_enabled),
                "stream_host": stream_host,
                "stream_port": args.port if args.stream_enabled else "",
            }

            csv_writer.writerow(csv_row)
            csv_file.flush()
            if route_session is not None and route_csv_writer is not None:
                route_csv_writer.writerow(csv_row)
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
                    fsm_state=calibration.control_state,
                    show_guidance_overlay=args.show_guidance_overlay,
                    start_calib_threshold_deg=CTRL_HYSTERESIS_HIGH,
                    stop_calib_threshold_deg=CTRL_HYSTERESIS_LOW,
                    overlay_scale=args.overlay_scale,
                    route_id=route_session.route_id if route_session is not None else None,
                    route_mode=current_route_mode,
                    vision_debug=vision_debug,
                )
                if args.show_vision_debug and vision_debug is not None:
                    frame_height, frame_width = annotated.shape[:2]
                    debug_panel_height = max(frame_height // 3, 220)
                    if debug_panel_height % 2 != 0:
                        debug_panel_height += 1
                    vision_panel = build_vision_debug_panel(
                        frame_width=frame_width,
                        panel_height=debug_panel_height,
                        vision_debug=vision_debug,
                    )
                    output_frame = cv2.vconcat([annotated, vision_panel])
                else:
                    output_frame = annotated
            elif args.stream_enabled or args.write_debug_video or route_session is not None:
                output_frame = draw_overlay(
                    frame=frame.copy(),
                    frame_num=frame_num,
                    theta=theta,
                    theta_for_overlay=last_known_theta,
                    servo_angle=servo_angle,
                    servo_center_angle=state.servo_center_angle,
                    fsm_state=calibration.control_state,
                    show_guidance_overlay=False,
                    start_calib_threshold_deg=CTRL_HYSTERESIS_HIGH,
                    stop_calib_threshold_deg=CTRL_HYSTERESIS_LOW,
                    overlay_scale=args.overlay_scale,
                    route_id=route_session.route_id if route_session is not None else None,
                    route_mode=current_route_mode,
                    vision_debug=vision_debug,
                )

            if args.frame_scale > 1.0:
                output_frame = cv2.resize(
                    output_frame,
                    dsize=None,
                    fx=args.frame_scale,
                    fy=args.frame_scale,
                    interpolation=cv2.INTER_CUBIC,
                )

            if args.write_debug_video:
                if video_writer is None:
                    h, w = output_frame.shape[:2]
                    video_writer, resolved_output_path = init_live_video_writer(
                        args.video_output,
                        MAIN_VIDEO_OUTPUT_FPS,
                        w,
                        h,
                    )
                    logger.info("Debug video writer active: %s", resolved_output_path)

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

            if route_session is not None:
                if route_video_writer is None:
                    h, w = output_frame.shape[:2]
                    route_video_path = route_session.route_dir / "route.mp4"
                    try:
                        route_video_writer = init_video_writer(
                            str(route_video_path),
                            MAIN_VIDEO_OUTPUT_FPS,
                            w,
                            h,
                        )
                        route_video_size = (w, h)
                        logger.info("Route video writer active: %s", route_video_path)
                    except RuntimeError as route_video_exc:
                        logger.error("Route video writer init failed: %s", route_video_exc)
                        route_video_writer = None
                if route_video_writer is not None:
                    h, w = output_frame.shape[:2]
                    if route_video_size != (w, h):
                        write_frame = cv2.resize(output_frame, route_video_size, interpolation=cv2.INTER_AREA)
                    else:
                        write_frame = output_frame
                    try:
                        route_video_writer.write(write_frame)
                    except Exception as route_video_exc:  # noqa: BLE001
                        logger.warning("Route video write error: %s", route_video_exc)

            if args.stream_enabled:
                telemetry = {
                    "frame_num": frame_num,
                    "theta": theta,
                    "theta_source": theta_source,
                    "fsm_state": calibration.control_state,
                    "servo_angle": servo_angle,
                    "route_id": route_session.route_id if route_session is not None else None,
                    "route_mode": current_route_mode,
                    "lines_count": vision_debug.get("lines_count") if vision_debug else None,
                }
                frame_store.set_frame(output_frame, telemetry)

            if args.show_preview:
                cv2.imshow("main_debug", output_frame)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    logger.info("Quit requested from preview window.")
                    final_status = "INTERRUPTED_MANUAL"
                    rejection_reason = "preview_quit"
                    break

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
        try:
            servo.center()
        except Exception:  # noqa: BLE001
            pass
        try:
            servo.close()
        except Exception:  # noqa: BLE001
            pass
        if mqtt_control_client is not None:
            try:
                mqtt_control_client.close()
            except Exception:  # noqa: BLE001
                pass
        if esp32_bridge is not None:
            try:
                esp32_bridge.close()
            except Exception:  # noqa: BLE001
                pass
        if base_stop_client is not None:
            try:
                base_stop_client.disconnect()
            except Exception:  # noqa: BLE001
                pass
            try:
                base_stop_client.loop_stop()
            except Exception:  # noqa: BLE001
                pass
        if cap is not None:
            cap.release()
        if video_writer is not None:
            video_writer.release()
        if stream_server is not None:
            stream_server.stop()
        if script_runner is not None:
            try:
                script_runner.close()
            except Exception:  # noqa: BLE001
                pass
        calibrator.close()
        if args.show_preview:
            try:
                cv2.destroyAllWindows()
            except cv2.error as close_preview_exc:
                logger.debug("Preview window cleanup skipped: %s", close_preview_exc)
        csv_file.close()
        finalize_route_session(status=final_status, reason=rejection_reason)
        logger.info("Resources released. Goodbye.")


if __name__ == "__main__":
    main()
