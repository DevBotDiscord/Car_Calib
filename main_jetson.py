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
import logging
import os
import sys
import time
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

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
from models.robot_state import RobotState, FSMState
from runtime.jetson_http import JetsonHttpServer
from unified_calibration_components import UnifiedCalibrator, CalibrationProcessingError

logger = logging.getLogger("jetson")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Jetson Nano calibration + dashboard")
    p.add_argument("--camera", type=int, default=MAIN_CAMERA_INDEX)
    p.add_argument("--hz", type=float, default=MAIN_TARGET_HZ)
    p.add_argument("--csv", type=str, default=MAIN_CSV_LOG_FILE)
    p.add_argument("--flip", action="store_true", default=False)
    p.add_argument("--port", type=int, default=int(os.getenv("DASHBOARD_PORT", "8080")))
    p.add_argument("--host", type=str, default=os.getenv("DASHBOARD_HOST", "0.0.0.0"))
    p.add_argument("--servo-pin", type=int, default=int(os.getenv("SERVO_PIN", "33")))
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


def main() -> None:
    args = build_parser().parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

    # ------------------------------------------------------------------ #
    # Core algorithm
    # ------------------------------------------------------------------ #
    calibrator = UnifiedCalibrator(telemetry_enabled=False)
    state: RobotState = calibrator.robot_state
    controller = calibrator.steering_controller

    # ------------------------------------------------------------------ #
    # Jetson GPIO drivers
    # ------------------------------------------------------------------ #
    servo = JetsonServoDriver(pin=args.servo_pin)
    base = JetsonBaseDriver(
        pin_bit2=int(os.getenv("BASE_BIT2_PIN", "15")),
        pin_bit1=int(os.getenv("BASE_BIT1_PIN", "13")),
        pin_bit0=int(os.getenv("BASE_BIT0_PIN", "11")),
    )
    relay = JetsonRelayDriver(
        relay_pin=int(os.getenv("RELAY_PIN", "18")),
        power_relay_pin=int(os.getenv("POWER_PIN", "16")),
    )

    # ------------------------------------------------------------------ #
    # Shared telemetry
    # ------------------------------------------------------------------ #
    telemetry_lock = threading.Lock()
    shared_frame: np.ndarray | None = None
    shared_telemetry: dict[str, Any] = {
        "frame": 0, "fsm": "GAPPING", "calib_active": False,
        "theta": None, "theta_src": "none",
        "servo": 0.0, "loop_ms": 0.0,
        "base": "STOP", "relay_on": False, "power_pulsing": False,
    }
    last_base_cmd = "STOP"

    def _update_shared(frame: np.ndarray, tel: dict[str, Any]) -> None:
        nonlocal shared_frame, shared_telemetry
        with telemetry_lock:
            shared_frame = frame.copy() if frame is not None else None
            shared_telemetry = dict(tel)

    def _frame_getter() -> np.ndarray | None:
        with telemetry_lock:
            return shared_frame.copy() if shared_frame is not None else None

    def _status_getter() -> dict[str, Any]:
        with telemetry_lock:
            return dict(shared_telemetry)

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
    if not args.no_dashboard:
        http = JetsonHttpServer(host=args.host, port=args.port)
        http.set_frame_getter(_frame_getter)
        http.set_status_getter(_status_getter)
        http.set_base_handler(_base_handler)
        http.set_relay_handler(_relay_handler)
        http.set_power_handler(_power_handler)
        http.start()

    # ------------------------------------------------------------------ #
    # Camera
    # ------------------------------------------------------------------ #
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        logger.error("Camera index %d failed", args.camera)
        servo.close()
        base.close()
        relay.close()
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    logger.info("Camera %d opened", args.camera)

    # ------------------------------------------------------------------ #
    # CSV logging
    # ------------------------------------------------------------------ #
    csv_path = Path(args.csv)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    existed = csv_path.exists() and csv_path.stat().st_size > 0
    csv_fields = _csv_fieldnames()
    csv_fh = csv_path.open("a", newline="", encoding="utf-8")
    csv_writer = csv.DictWriter(csv_fh, fieldnames=csv_fields)
    if not existed:
        csv_writer.writeheader()
        csv_fh.flush()

    # ------------------------------------------------------------------ #
    # Main loop
    # ------------------------------------------------------------------ #
    target_period = 1.0 / max(0.1, args.hz)
    frame_num = 0
    last_known_theta: float | None = None

    logger.info("Starting Jetson Nano loop at %.0f Hz", args.hz)
    try:
        while True:
            loop_start = time.monotonic()
            frame_num += 1

            ret, frame = cap.read()
            if not ret or frame is None:
                logger.warning("frame %d capture failed", frame_num)
                time.sleep(0.01)
                continue

            if args.flip:
                frame = cv2.flip(frame, -1)

            # --- calibration ---
            try:
                calibration = calibrator.process_frame(frame, frame_num)
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

            # --- servo ---
            servo.send_angle(servo_angle)

            # --- telemetry ---
            loop_ms = (time.monotonic() - loop_start) * 1000.0
            pid_error = 0.0 if theta is None else float(theta) - 90.0

            tel = {
                "frame": frame_num,
                "fsm": fsm_state,
                "calib_active": calibration.calibration_active,
                "theta": f"{theta:.2f}" if theta is not None else None,
                "theta_src": "live" if theta is not None else ("stale" if last_known_theta is not None else "none"),
                "servo": f"{servo_angle:.2f}",
                "loop_ms": f"{loop_ms:.1f}",
                "base": last_base_cmd,
                "relay_on": relay.relay_state,
                "power_pulsing": relay.power_pulsing,
            }
            _update_shared(frame, tel)

            # --- CSV ---
            csv_writer.writerow({
                "frame_num": frame_num,
                "mono_timestamp": f"{loop_start:.6f}",
                "utc_timestamp": datetime.now(timezone.utc).isoformat(),
                "loop_ms": f"{loop_ms:.4f}",
                "fsm_state": fsm_state,
                "calibration_active": int(calibration.calibration_active),
                "theta": f"{theta:.4f}" if theta is not None else "",
                "theta_source": "live" if theta is not None else ("stale" if last_known_theta is not None else "none"),
                "servo_angle": f"{servo_angle:.4f}",
                "servo_center_angle": f"{state.servo_center_angle:.4f}",
                "servo_offset": f"{(servo_angle - state.servo_center_angle):.4f}",
                "pid_error": f"{pid_error:.6f}",
                "pid_p_term": f"{state.pid.kp * pid_error:.6f}",
                "pid_i_term": f"{state.pid.ki * state.pid_integral:.6f}",
                "pid_d_term": "0.000000",
                "base_command": last_base_cmd,
                "relay_on": int(relay.relay_state),
            })

            if frame_num % 30 == 0:
                csv_fh.flush()

            # --- sleep remainder ---
            remaining = target_period - (time.monotonic() - loop_start)
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    finally:
        servo.center()
        time.sleep(0.3)
        base.stop()
        servo.close()
        base.close()
        relay.close()
        cap.release()
        csv_fh.close()
        if http is not None:
            http.stop()
        cv2.destroyAllWindows()
        logger.info("Shutdown complete")


if __name__ == "__main__":
    main()
