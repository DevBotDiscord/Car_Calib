#!/usr/bin/env python3
"""Jetson Nano direct calibration + dashboard + base/relay control.

Same UnifiedCalibrator core, writes servo PWM directly via Jetson.GPIO.
Plus embedded HTTP dashboard with MJPEG stream, telemetry, and controls.

Usage:
    python3 main_jetson.py [--camera 0] [--hz 30] [--port 8080]

Env vars:
    SERVO_CENTER_ANGLE  (default: -35)
    MAX_STEERING_OFFSET (default: 60)
    SERVO_PIN           (default: 33)
    BASE_BIT2_PIN       (default: 15)
    BASE_BIT1_PIN       (default: 13)
    BASE_BIT0_PIN       (default: 11)
    RELAY_PIN           (default: 18)
    POWER_PIN           (default: 16)
"""

from __future__ import annotations

import argparse
import csv
import json
import logging
import os
import sys
import time
import threading
from pathlib import Path
from typing import Any, Callable

import cv2
import numpy as np

from config.settings import (
    DANGER_MARGIN_PX,
    DANGER_NUDGE_DEG,
    CTRL_HYSTERESIS_LOW,
    CTRL_HYSTERESIS_HIGH,
    MAIN_CSV_LOG_FILE,
    MAIN_TARGET_HZ,
    MAIN_CAMERA_INDEX,
)
from drivers.jetson_base import JetsonBaseDriver
from drivers.jetson_relay import JetsonRelayDriver
from drivers.jetson_servo import JetsonServoDriver
from drivers.pigpio_base import PigpioBaseDriver
from drivers.pigpio_relay import PigpioRelayDriver
from drivers.pigpio_servo import PigpioServoDriver
from models.robot_state import RobotState, FSMState
from runtime.jetson_http import JetsonHttpServer
from runtime.route_logging import RouteSession
from runtime.jetson_script_runner import JetsonScriptRunner
from runtime.calib_tuning import CalibTuneManager
from runtime.manual_override import ManualOverrideController
from runtime.object_detection import ObjectDetectionStatus, ObjectDetector, draw_object_boxes
from runtime.dashboard_stream import DashboardStreamBroker
from runtime.resource_limits import ScriptValidationError, StorageManager, validate_route_steps
from runtime.ultrasonic_safety import SonarConfig, UltrasonicSafety
from unified_calibration_components import UnifiedCalibrator, CalibrationProcessingError
from runtime.sasc_experiment_log import SascExperimentLogger

logger = logging.getLogger("jetson")

def _env_int(names: tuple[str, ...], default: int) -> int:
    for name in names:
        value = os.getenv(name)
        if value is not None:
            return int(value)
    return default

def _env_float(names: tuple[str, ...], default: float) -> float:
    for name in names:
        value = os.getenv(name)
        if value is not None:
            return float(value)
    return default

def _env_bool(name: str, default: bool) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in ("1", "true", "yes", "on")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Jetson Nano calibration + dashboard")
    p.add_argument("--camera", type=int, default=MAIN_CAMERA_INDEX)
    p.add_argument("--hz", type=float, default=MAIN_TARGET_HZ)
    p.add_argument("--csv", type=str, default=MAIN_CSV_LOG_FILE)
    p.add_argument("--flip", action="store_true", default=False)
    p.add_argument("--port", type=int, default=int(os.getenv("DASHBOARD_PORT", "8080")))
    p.add_argument("--host", type=str, default=os.getenv("DASHBOARD_HOST", "0.0.0.0"))
    hardware_default = os.getenv("CONTROL_HARDWARE", "jetson")
    servo_pin_default = "12" if hardware_default == "pigpio" else "33"
    p.add_argument("--hardware", choices=("jetson", "pigpio"), default=hardware_default)
    p.add_argument("--servo-pin", type=int, default=int(os.getenv("SERVO_PIN", servo_pin_default)))
    p.add_argument("--no-dashboard", action="store_true", default=False)
    return p


def _csv_fieldnames() -> list[str]:
    return [
        "frame_num", "mono_timestamp", "utc_timestamp",
        "loop_ms", "fsm_state", "calibration_active",
        "theta", "theta_source",
        "servo_angle", "servo_center_angle", "servo_offset",
        "pid_error", "pid_p_term", "pid_i_term", "pid_d_term",
        "base_command", "relay_on",
    ]

# --------------------------------------------------------------------------- #
# In-process route script runner (no MQTT)
# --------------------------------------------------------------------------- #
def constrain(value, min_value, max_value):
    return max(min_value, min(value, max_value))


def map_calibrated_servo(
    input_cmd,
    home_offset_deg=-8,
    limit_deg=60,
    reverse=False
):
    """
    input_cmd: lệnh logic từ 0 -> 180
            90 là home logic

    home_offset_deg: physical offset của home
                    ví dụ home lệch -8 độ thì servo physical home = 90 + (-8) = 82

    limit_deg: giới hạn vật lý mỗi bên
            ví dụ 60 nghĩa là chỉ chạy từ -60 -> +60 quanh home

    reverse: đảo chiều servo nếu cần
    """

    # Giới hạn input logic
    input_cmd = constrain(input_cmd, 0, 180)

    # Tính home servo thật sau calibration
    home_cmd = 90 + home_offset_deg

    # Đổi input 0..180 thành ratio -1..1
    ratio = (input_cmd - 90) / 90

    # Map ratio ra góc servo thật
    calib_deg = input_cmd - 90
    # Cap error nếu vượt quá limit
    if abs(calib_deg) > limit_deg:
        calib_deg = limit_deg if calib_deg > 0 else -limit_deg
        
    servo_cmd = home_cmd + calib_deg

    # Due to hardware limitation of servo, we need to add some offset to avoid hitting the physical limit
    if servo_cmd < home_cmd: #increase power of right side
        servo_cmd -= 10
    elif servo_cmd > home_cmd: #increase power of left side
        servo_cmd += -5
    
    # Giới hạn an toàn servo 0..180
    servo_cmd = constrain(servo_cmd, 0, 180)

    return servo_cmd

def _scan_routes() -> list[dict[str, Any]]:
    """Scan ROUTE_LOG_ROOT for completed route directories."""
    import json as _json
    from pathlib import Path as _Path
    root = _Path(os.getenv("ROUTE_LOG_ROOT", "/data/routes"))
    if not root.is_dir():
        root = _Path("run_logs/routes")
    if not root.is_dir():
        return []
    routes: list[dict[str, Any]] = []
    for d in sorted(root.iterdir(), reverse=True):
        if not d.is_dir() or not d.name.startswith("route-"):
            continue
        summary_file = d / "route_summary.json"
        info: dict[str, Any] = {
            "route_id": d.name,
            "route_mode": "",
            "preset": "",
            "status": "not_recorded",
            "frames": 0,
            "elapsed": 0.0,
            "zip_size": None,
            "has_zip": False,
            "ended_utc": "",
        }
        if summary_file.is_file():
            try:
                s = _json.loads(summary_file.read_text())
                info.update({
                    "route_id": s.get("route_id", d.name),
                    "route_mode": s.get("route_mode", ""),
                    "preset": s.get("preset_name", ""),
                    "status": s.get("status", "not_recorded"),
                    "frames": s.get("total_frames", 0),
                    "elapsed": float(s.get("total_elapsed_seconds", 0.0)),
                    "accepted": s.get("accepted"),
                    "ended_utc": s.get("end_timestamp_utc", ""),
                })
            except Exception:
                pass
        # Check for zip
        zip_path = d.with_suffix(".zip")
        if zip_path.is_file():
            info["has_zip"] = True
            info["zip_size"] = zip_path.stat().st_size
        routes.append(info)
    return routes


def main() -> None:
    args = build_parser().parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

    # --- debugpy remote debugging (set DEBUGPY_ENABLED=true) ---
    if os.getenv("DEBUGPY_ENABLED", "").strip().lower() in ("1", "true", "yes", "on"):
        import debugpy
        debugpy.listen(("0.0.0.0", int(os.getenv("DEBUGPY_PORT", "5678"))))
        if os.getenv("DEBUGPY_WAIT", "").strip().lower() in ("1", "true", "yes", "on"):
            logger.info("debugpy: waiting for client on port %s ...", os.getenv("DEBUGPY_PORT", "5678"))
            debugpy.wait_for_client()
            logger.info("debugpy: client attached")
        else:
            logger.info("debugpy: listening on port %s (non-blocking)", os.getenv("DEBUGPY_PORT", "5678"))

    # ------------------------------------------------------------------ #
    # Core algorithm
    # ------------------------------------------------------------------ #
    calibrator = UnifiedCalibrator(telemetry_enabled=True)
    state: RobotState = calibrator.robot_state
    controller = calibrator.steering_controller
    # SERVO_CENTER_ANGLE is physical home offset in direct-control mode.
    # Keep calibration output centered at logic 90 so mapping remains 0..180.
    state.servo_center_angle = 90.0
    state.max_steering_offset = 90.0
    state.last_valid_servo_angle = 90.0
    state.last_valid_command = 90.0
    controller._center = 90.0
    controller._max_offset = 90.0

    # ------------------------------------------------------------------ #
    # Direct hardware drivers
    # ------------------------------------------------------------------ #
    if args.hardware == "pigpio":
        pigpio_host = os.getenv("PIGPIO_HOST", "127.0.0.1")
        pigpio_port = int(os.getenv("PIGPIO_PORT", "8888"))
        servo = PigpioServoDriver(
            pin=args.servo_pin,
            host=pigpio_host,
            port=pigpio_port,
        )
        base = PigpioBaseDriver(
            out1=_env_int(("BASE_OUT1", "BASE_BIT2_PIN"), 17),
            out2=_env_int(("BASE_OUT2", "BASE_BIT1_PIN"), 27),
            out3=_env_int(("BASE_OUT3", "BASE_BIT0_PIN"), 22),
            host=pigpio_host,
            port=pigpio_port,
        )
        relay = PigpioRelayDriver(
            relay_pin=_env_int(("RELAY_PIN",), 13),
            power_relay_pin=_env_int(("POWER_RELAY_PIN", "POWER_PIN"), 5),
            relay_active_low=_env_bool("RELAY_ACTIVE_LOW", False),
            power_active_low=_env_bool("POWER_RELAY_ACTIVE_LOW", False),
            power_on_pulse_ms=_env_int(("POWER_ON_PULSE_MS",), 100),
            power_off_pulse_ms=_env_int(("POWER_OFF_PULSE_MS",), 3000),
            host=pigpio_host,
            port=pigpio_port,
        )
        hardware_source = "control-direct"
    else:
        servo = JetsonServoDriver(pin=args.servo_pin)
        base = JetsonBaseDriver(
            pin_bit2=_env_int(("BASE_BIT2_PIN",), 15),
            pin_bit1=_env_int(("BASE_BIT1_PIN",), 13),
            pin_bit0=_env_int(("BASE_BIT0_PIN",), 11),
        )
        relay = JetsonRelayDriver(
            relay_pin=_env_int(("RELAY_PIN",), 18),
            power_relay_pin=_env_int(("POWER_PIN", "POWER_RELAY_PIN"), 16),
            relay_active_low=_env_bool("RELAY_ACTIVE_LOW", True),
            power_active_low=_env_bool("POWER_RELAY_ACTIVE_LOW", True),
            power_on_pulse_ms=_env_int(("POWER_ON_PULSE_MS",), 100),
            power_off_pulse_ms=_env_int(("POWER_OFF_PULSE_MS",), 3000),
        )
        hardware_source = "jetson"

    # HC-SR04 is only available on the pigpio/Raspberry Pi direct-control path.
    # Its Echo timing is callback-based; update() below never waits for a pulse.
    if args.hardware == "pigpio":
        sonar = UltrasonicSafety.from_env(
            host=os.getenv("PIGPIO_HOST", "127.0.0.1"),
            port=int(os.getenv("PIGPIO_PORT", "8888")),
        )
    else:
        sonar = UltrasonicSafety(SonarConfig(enabled=False))

    # ------------------------------------------------------------------ #
    # Shared telemetry
    # ------------------------------------------------------------------ #
    telemetry_lock = threading.Lock()
    log_root = Path(args.csv).parent if Path(args.csv).is_absolute() else Path("logs")
    route_root = Path(os.getenv("ROUTE_LOG_ROOT", "/data/routes"))
    storage = StorageManager(log_root, route_root)
    stream_broker = DashboardStreamBroker(
        max_clients=_env_int(("DASHBOARD_STREAM_MAX_CLIENTS",), 2),
        fps=_env_float(("DASHBOARD_STREAM_FPS",), 8.0),
        jpeg_quality=_env_int(("DASHBOARD_STREAM_JPEG_QUALITY",), 60),
    )
    shared_telemetry: dict[str, Any] = {
        "frame": 0, "fsm": "GAPPING", "calib_active": False,
        "theta": None, "theta_src": "none",
        "servo": 0.0, "loop_ms": 0.0,
        "base": "STOP", "relay_on": False, "power_pulsing": False,
        "object_state": "none", "object_pause_active": False,
        "object_count": 0, "object_label": None, "object_conf": None,
        "object_boxes": [], "object_detector_error": None,
        "sonar_enabled": False, "sonar_available": False,
        "sonar_blocked": False, "sonar_distance_cm": None,
        "sonar_age_s": None, "sonar_reason": "",
        "manual_override_active": False, "manual_override_age_s": None,
        "manual_drive": 0.0, "manual_steer": 0.0,
        "manual_blocked_reason": "",
    }
    last_base_cmd = "STOP"
    last_manual_servo_angle: float | None = None

    def _update_shared(frame: np.ndarray, tel: dict[str, Any]) -> None:
        nonlocal shared_telemetry
        stream_broker.submit(frame)
        with telemetry_lock:
            shared_telemetry = dict(tel)

    def _status_getter() -> dict[str, Any]:
        with telemetry_lock:
            tel = dict(shared_telemetry)
        storage_health = storage.health()
        telemetry_health = (
            calibrator._telemetry.logging_health()
            if getattr(calibrator, "_telemetry", None) is not None
            else {"telemetry_logging_enabled": False, "telemetry_logging_error": "telemetry unavailable"}
        )
        return {
            "telemetry": {
                "frame_num": tel.get("frame"),
                "fsm_state": tel.get("fsm"),
                "calibration_active": tel.get("calib_active"),
                "theta": tel.get("theta"),
                "theta_source": tel.get("theta_src"),
                "servo_angle": tel.get("servo"),
                "loop_ms": tel.get("loop_ms"),
                "route_mode": tel.get("current_route_mode"),
                "object_state": tel.get("object_state"),
                "object_pause_active": tel.get("object_pause_active"),
                "object_count": tel.get("object_count"),
                "object_label": tel.get("object_label"),
                "object_conf": tel.get("object_conf"),
                "object_boxes": tel.get("object_boxes"),
                "object_detector_error": tel.get("object_detector_error"),
                "sonar_enabled": tel.get("sonar_enabled"),
                "sonar_available": tel.get("sonar_available"),
                "sonar_blocked": tel.get("sonar_blocked"),
                "sonar_distance_cm": tel.get("sonar_distance_cm"),
                "sonar_age_s": tel.get("sonar_age_s"),
                "sonar_reason": tel.get("sonar_reason"),
                "manual_override_active": tel.get("manual_override_active"),
                "manual_override_age_s": tel.get("manual_override_age_s"),
                "manual_drive": tel.get("manual_drive"),
                "manual_steer": tel.get("manual_steer"),
                "manual_blocked_reason": tel.get("manual_blocked_reason"),
            },
            "rpi_status": {
                "online": True,
                "stale": False,
                "age_s": 0.0,
                "payload": tel,
            },
            "actuator": {
                "online": True,
                "source": hardware_source,
            },
            "resource_health": {
                "storage_writable": storage_health.writable,
                "storage_reason": storage_health.reason,
                "disk_free_bytes": storage_health.free_bytes,
                "managed_data_bytes": storage_health.managed_bytes,
                "recording_active": route_session is not None,
                **telemetry_health,
                **stream_broker.health(),
            },
        }

    def _base_handler(cmd: str) -> None:
        nonlocal last_base_cmd
        base.command(cmd)
        last_base_cmd = cmd.upper()

    def _relay_handler(state: str) -> None:
        if state == "ON":
            relay.relay_on()
        elif state == "OFF":
            relay.relay_off()

    def _power_handler(state: str) -> None:
        if state == "ON":
            relay.power_on()
        elif state == "OFF":
            relay.power_off()

    # ------------------------------------------------------------------ #
    # HTTP Dashboard
    # ------------------------------------------------------------------ #
    http: JetsonHttpServer | None = None
    script_runner: JetsonScriptRunner | None = None
    route_session: RouteSession | None = None
    route_video_writer: cv2.VideoWriter | None = None
    route_video_disabled = False
    route_csv_file: Any | None = None
    route_csv_writer: csv.DictWriter | None = None
    sasc_logger: SascExperimentLogger | None = None
    current_sasc_scene_type = ""

    def _refresh_storage_active_paths() -> None:
        active: list[Path] = []
        telemetry_logger = getattr(calibrator, "_telemetry", None)
        if telemetry_logger is not None:
            active.extend(telemetry_logger.active_storage_paths())
        if route_session is not None:
            active.append(route_session.route_dir)
        storage.set_active_paths(active)

    _refresh_storage_active_paths()
    storage.retain()

    def _tune_idle_state() -> tuple[bool, str]:
        if script_runner is not None and script_runner.is_running():
            return False, "blocked: script running"
        if last_base_cmd.upper() != "STOP":
            return False, "blocked: base not STOP"
        return True, "idle"

    tune_file = os.getenv("CALIB_TUNE_FILE") or str(
        Path(os.getenv("ROUTE_LOG_ROOT", "/data/routes")) / "calib_tune.json"
    )
    tune_manager = CalibTuneManager(
        calibrator,
        tune_file,
        idle_getter=_tune_idle_state,
    )
    object_detector = ObjectDetector.from_env()
    object_detection_script_only = _env_bool("OBJECT_DETECTION_SCRIPT_ONLY", False)
    manual_override = ManualOverrideController.from_env(
        center_angle=90.0 + _env_float(("SERVO_CENTER_ANGLE",), -8.0),
        max_steer=_env_float(("MAX_STEERING_OFFSET",), 60.0),
    )

    if not args.no_dashboard:
        http = JetsonHttpServer(host=args.host, port=args.port)
        http.set_stream_broker(stream_broker)
        http.set_status_getter(_status_getter)
        http.set_base_handler(_base_handler)
        http.set_relay_handler(_relay_handler)
        http.set_power_handler(_power_handler)
        http.set_manual_override_handler(lambda body: manual_override.submit(body))
        # Route script runner + presets
        presets_path = Path(os.getenv("ROUTE_LOG_ROOT", "/data/routes")) / "presets.json"
        try:
            _presets: dict[str, list[dict[str, Any]]] = json.loads(
                presets_path.read_text(encoding="utf-8")
            )
        except Exception:
            _presets = {}

        def _save_presets() -> None:
            try:
                presets_path.parent.mkdir(parents=True, exist_ok=True)
                presets_path.write_text(json.dumps(_presets, indent=2), encoding="utf-8")
            except OSError as exc:
                storage.mark_error(exc)
                raise ScriptValidationError("preset storage is unavailable") from exc

        def _set_preset(body: str) -> None:
            data = json.loads(body)
            name = str(data.get("name", "untitled")).strip()[:80] or "untitled"
            _presets[name] = validate_route_steps(data.get("steps", []))
            _save_presets()

        def _delete_preset(name: str) -> None:
            _presets.pop(name, None)
            _save_presets()

        _steps: list[dict[str, Any]] = []
        script_runner = JetsonScriptRunner()
        script_runner.set_handlers(_base_handler, servo.send_angle, _relay_handler)

        def _submit_script(body: str) -> bool:
            nonlocal route_session, route_video_writer, route_video_disabled, route_csv_file, route_csv_writer, sasc_logger, current_sasc_scene_type
            payload = json.loads(body)
            steps = validate_route_steps(payload.get("steps", []))
            preset_name = str(payload.get("preset_name") or "").strip()
            ok = script_runner.submit(steps)
            if ok:
                if route_video_writer is not None:
                    try:
                        route_video_writer.release()
                    except OSError as exc:
                        storage.mark_error(exc)
                    route_video_writer = None
                route_video_disabled = False
                if route_csv_file is not None:
                    try:
                        route_csv_file.close()
                    except OSError as exc:
                        storage.mark_error(exc)
                    route_csv_file = None
                    route_csv_writer = None
                if sasc_logger is not None:
                    sasc_logger.close()
                    sasc_logger = None
                storage.retain()
                if storage.recover_if_possible():
                    try:
                        route_session = RouteSession(route_mode="SCRIPT")
                        _refresh_storage_active_paths()
                        route_session.attach_meta("script_steps", steps)
                        route_session.attach_meta("source", "dashboard_direct")
                        route_session.attach_meta("preset_name", preset_name)
                        route_session.attach_meta("video_file", "")
                        route_session.attach_meta("csv_file", "route_frames.csv")
                        route_session.attach_meta("sasc_file", "sasc_baseline_log.csv")
                        route_session.start(time.monotonic())
                        current_sasc_scene_type = preset_name
                        logger.info("Route recording started: %s", route_session.route_id)
                    except OSError as exc:
                        route_session = None
                        _refresh_storage_active_paths()
                        storage.mark_error(exc)
                        logger.warning("Route recording disabled: %s", exc)
                else:
                    logger.warning("Route recording disabled: %s", storage.health().reason)
            return ok

        http.set_script_runner(lambda: script_runner.status())
        http.set_script_stopper(script_runner.stop)
        http.set_script_submitter(_submit_script)
        http.set_steps_getter(lambda: _steps)
        http.set_steps_setter(lambda body: _steps.extend(validate_route_steps(json.loads(body).get("steps", []))))
        http.set_presets_getter(lambda: [{"name": k, "steps": v, "steps_count": len(v)} for k, v in _presets.items()])
        http.set_presets_setter(_set_preset)
        http.set_preset_deleter(_delete_preset)
        http.set_routes_getter(_scan_routes)
        http.set_tune_handlers(
            getter=tune_manager.status,
            applier=tune_manager.apply,
            saver=tune_manager.save,
            resetter=tune_manager.reset,
        )

        http.start()

    def _finalize_route(status: str) -> None:
        nonlocal route_session, route_video_writer, route_video_disabled, route_csv_file, route_csv_writer, sasc_logger, current_sasc_scene_type
        if route_video_writer is not None:
            try:
                route_video_writer.release()
            except OSError as exc:
                storage.mark_error(exc)
            route_video_writer = None
        if route_csv_file is not None:
            try:
                route_csv_file.flush()
                route_csv_file.close()
            except OSError as exc:
                storage.mark_error(exc)
            route_csv_file = None
            route_csv_writer = None
        if sasc_logger is not None:
            sasc_logger.close()
            sasc_logger = None
        if route_session is None:
            return
        try:
            result = route_session.finalize(mono_now=time.monotonic(), status=status)
            logger.info(
                "Route recording finalized: %s accepted=%s reason=%s",
                result.route_id,
                result.accepted,
                result.rejection_reason,
            )
        except OSError as exc:
            storage.mark_error(exc)
            logger.warning("Route summary could not be written: %s", exc)
        route_session = None
        route_video_disabled = False
        _refresh_storage_active_paths()
        storage.retain()
        current_sasc_scene_type = ""

    def _open_route_video_writer(frame_w: int, frame_h: int) -> cv2.VideoWriter | None:
        if route_session is None:
            return None
        candidates = [
            ("route.avi", "MJPG"),
            ("route.mp4", "mp4v"),
        ]
        for filename, codec in candidates:
            path = route_session.route_dir / filename
            writer = cv2.VideoWriter(
                str(path),
                cv2.VideoWriter_fourcc(*codec),
                float(args.hz),
                (frame_w, frame_h),
            )
            if writer.isOpened():
                route_session.attach_meta("video_file", filename)
                logger.info("Route video recording to %s codec=%s", path, codec)
                return writer
            writer.release()
            logger.warning("Route video writer unavailable: %s codec=%s", path, codec)
        route_session.attach_meta("video_file", "")
        return None

    # ------------------------------------------------------------------ #
    # Camera
    # ------------------------------------------------------------------ #
    # Camera auto-detect: keep trying until we get one
    def acquire_camera() -> cv2.VideoCapture:
        while True:
            candidates = [args.camera] + [i for i in range(10) if i != args.camera]
            for idx in candidates:
                test = cv2.VideoCapture(idx)
                if test.isOpened():
                    logger.info("Camera opened at index %d", idx)
                    test.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    test.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    return test
                test.release()
            logger.warning("No camera found across %s, retrying in 0.5s...", candidates)
            time.sleep(0.5)

    cap = acquire_camera()

    # ------------------------------------------------------------------ #
    # Main loop
    # ------------------------------------------------------------------ #
    target_period = 1.0 / max(0.1, args.hz)
    frame_num = 0
    last_known_theta: float | None = None

    logger.info("Starting Calibration control loop at %.0f Hz", args.hz)
    try:
        while True:
            loop_start = time.monotonic()
            frame_num += 1

            ret, frame = cap.read()
            if not ret or frame is None:
                logger.warning("Frame %d capture failed, re-acquiring camera...", frame_num)
                try:
                    cap.release()
                except Exception:
                    pass
                base.stop()
                cap = acquire_camera()
                time.sleep(0.1)
                continue

            if args.flip:
                frame = cv2.flip(frame, -1)

            # --- calibration ---
            try:
                calibration = calibrator.process_frame(frame, frame_num)
                display_frame = calibrator.render_frame(frame, calibration)
            except CalibrationProcessingError as exc:
                logger.exception("Calibration failure: %s", exc)
                servo.center()
                base.stop()
                break

            theta = calibration.observation_angle
            servo_angle = calibration.steering_angle
            fsm_state = calibration.control_state

            if theta is not None:
                last_known_theta = theta

            now = time.monotonic()
            script_running = script_runner is not None and script_runner.is_running()
            manual_wants_control = manual_override.wants_control(now=now)
            if object_detection_script_only and not script_running and not manual_wants_control:
                object_status = ObjectDetectionStatus("none", False, 0, None, None, ())
            else:
                object_status = object_detector.process(frame, now=now)
            display_frame = draw_object_boxes(display_frame, object_status.object_boxes)
            object_pause_active = object_status.object_pause_active
            sonar_status = sonar.update(now=now)
            sonar_pause_active = sonar_status.blocked
            safety_pause_active = object_pause_active or sonar_pause_active

            manual_decision = manual_override.evaluate(
                now=now,
                object_near=safety_pause_active or object_status.object_state == "near",
                estop_active=False,
            )
            if manual_decision.release_servo and not manual_decision.active:
                if last_base_cmd.upper() != "STOP":
                    _base_handler("STOP")
                release_servo = getattr(servo, "release", None)
                if callable(release_servo):
                    release_servo()
                last_manual_servo_angle = None

            if script_runner is not None:
                if manual_decision.pause_script:
                    script_runner.set_paused(True, "manual_override")
                elif sonar_pause_active:
                    script_runner.set_paused(True, f"sonar_{sonar_status.reason or 'blocked'}")
                else:
                    script_runner.set_paused(
                        object_pause_active,
                        "object_detected" if object_pause_active else "",
                    )

            if safety_pause_active and not manual_decision.active:
                if last_base_cmd.upper() != "STOP":
                    _base_handler("STOP")
                release_servo = getattr(servo, "release", None)
                if callable(release_servo):
                    release_servo()
            elif manual_decision.active:
                if manual_decision.base_command != last_base_cmd.upper():
                    _base_handler(manual_decision.base_command)
                if manual_decision.release_servo:
                    release_servo = getattr(servo, "release", None)
                    if callable(release_servo):
                        release_servo()
                if manual_decision.servo_angle is not None and (
                    last_manual_servo_angle is None
                    or abs(last_manual_servo_angle - manual_decision.servo_angle) >= 0.5
                ):
                    servo.send_angle(manual_decision.servo_angle)
                    last_manual_servo_angle = manual_decision.servo_angle
            else:
                last_manual_servo_angle = None

            # --- servo ---
            output_angle = map_calibrated_servo(
                servo_angle,
                home_offset_deg=_env_float(("SERVO_CENTER_ANGLE",), -8.0),
                limit_deg=_env_int(("MAX_STEERING_OFFSET",), 60),
                reverse=_env_bool("SERVO_REVERSE", False),
            )
            if (
                not safety_pause_active
                and not manual_decision.active
                and script_runner is not None
                and script_running
                and script_runner.vision_pid_active()
            ):
                servo.send_angle(output_angle)
            final_angle = (
                manual_decision.servo_angle
                if manual_decision.active and manual_decision.servo_angle is not None
                else output_angle
            )

            # --- telemetry ---
            loop_ms = (time.monotonic() - loop_start) * 1000.0
            if frame_num % 10 == 1:
                logger.info(
                    "frame=%d state=%s vp=%.1f° steer=%.1f° loop=%.0fms",
                    frame_num, fsm_state,
                    theta if theta is not None else float('nan'),
                    final_angle, loop_ms,
                )
            pid_error = 0.0 if theta is None else float(theta) - 90.0
            current_route_id = route_session.route_id if route_session is not None else None
            if route_session is not None:
                if not storage.health().writable:
                    _finalize_route("STORAGE_ERROR")
                else:
                    try:
                        if route_video_writer is None and not route_video_disabled:
                            frame_h, frame_w = display_frame.shape[:2]
                            route_video_writer = _open_route_video_writer(frame_w, frame_h)
                            route_video_disabled = route_video_writer is None
                        if route_video_writer is not None:
                            route_video_writer.write(display_frame)
                        route_session.update_frame(
                            mono_now=time.monotonic(),
                            theta=theta,
                            fsm_state=fsm_state,
                            calibration_active=calibration.calibration_active,
                        )
                    except OSError as exc:
                        storage.mark_error(exc)
                        _finalize_route("STORAGE_ERROR")
                    if route_session is not None and (script_runner is None or not script_runner.is_running()):
                        _finalize_route("COMPLETED")

            # Build dashboard telemetry, merging calibrator output with hardware state
            tel = dict(calibration.telemetry)
            tel.update({
                "source": hardware_source,
                "rpi_online": True,
                "mqtt_connected": True,
                "estop_active": False,
                "steer_angle": f"{final_angle:.1f}",
                "current_route_mode": "AUTO",
                "route_id": current_route_id,
                "centered": fsm_state == "GAPPING",
                "relay_on": relay.relay_state,
                "servo_feedback_enabled": False,
                "servo_feedback_angle": f"{servo_angle:.1f}",
                "servo_feedback_error": "0.0",
                "servo_feedback_ok": True,
                "servo_feedback_raw": 0,
                "frame": frame_num,
                "fsm": fsm_state,
                "calib_active": calibration.calibration_active,
                "theta": f"{theta:.2f}" if theta is not None else None,
                "theta_src": "live" if theta is not None else ("stale" if last_known_theta is not None else "none"),
                "servo": f"{servo_angle:.2f}",
                "loop_ms": f"{loop_ms:.1f}",
                "base": last_base_cmd,
                "power_pulsing": relay.power_pulsing,
            })
            tel.update(object_status.telemetry())
            tel.update(sonar_status.telemetry())
            tel["safety_pause_active"] = safety_pause_active
            tel.update(manual_decision.telemetry())
            if route_session is not None:
                mono_now = time.monotonic()
                try:
                    if route_csv_writer is None:
                        route_csv_file = (route_session.route_dir / "route_frames.csv").open(
                            "w",
                            newline="",
                            encoding="utf-8",
                        )
                        fieldnames = sorted(tel.keys())
                        route_csv_writer = csv.DictWriter(
                            route_csv_file,
                            fieldnames=fieldnames,
                            extrasaction="ignore",
                        )
                        route_csv_writer.writeheader()
                        logger.info("Route CSV recording to %s", route_session.route_dir / "route_frames.csv")
                    route_csv_writer.writerow(tel)
                    if route_csv_file is not None:
                        route_csv_file.flush()
                    if sasc_logger is None:
                        sasc_logger = SascExperimentLogger(
                            route_session.route_dir / "sasc_baseline_log.csv",
                            run_id=route_session.route_id,
                            scene_type=current_sasc_scene_type,
                            start_monotonic=getattr(route_session, "_start_monotonic", None),
                        )
                        logger.info("SASC CSV recording to %s", sasc_logger.path)
                    if not sasc_logger.write_frame(tel, frame_id=route_session.total_frames, mono_now=mono_now):
                        raise OSError(sasc_logger.error or "SASC logger disabled")
                except OSError as exc:
                    storage.mark_error(exc)
                    _finalize_route("STORAGE_ERROR")
            _update_shared(display_frame, tel)

            # --- CSV telemetry (delegated to calibrator) ---
            calibration.telemetry.update({"final_servo_angle": final_angle})
            if calibrator._telemetry is not None:
                calibrator._telemetry.log_state(frame_num, calibration.telemetry)
                _refresh_storage_active_paths()
                logging_error = calibrator._telemetry.logging_health().get("telemetry_logging_error", "")
                if logging_error:
                    storage.mark_error(str(logging_error))

            # --- sleep remainder ---
            remaining = target_period - (time.monotonic() - loop_start)
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    finally:
        _finalize_route("INTERRUPTED")
        object_detector.close()
        sonar.close()
        servo.center()
        time.sleep(0.3)
        base.stop()
        servo.close()
        base.close()
        relay.close()
        cap.release()
        if http is not None:
            http.stop()
        stream_broker.close()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        logger.info("Shutdown complete")


if __name__ == "__main__":
    main()
