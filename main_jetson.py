#!/usr/bin/env python3
"""Jetson Nano direct calibration loop.

Same UnifiedCalibrator core algorithm (vision → geometry → steering),
but writes servo PWM directly via Jetson.GPIO — no MQTT, no ESP, no network.

Usage:
    python3 main_jetson.py [--camera 0] [--hz 30] [--csv logs/run.csv]

Env vars (override in .env or export):
    SERVO_CENTER_ANGLE  (default: -35)
    MAX_STEERING_OFFSET (default: 60)
    SERVO_PIN           (default: 33, BOARD pin 33)

Flow:
    1. Camera → cv2.VideoCapture
    2. UnifiedCalibrator.process_frame(frame, num)
    3. CalibrationResult.steering_angle
    4. JetsonServoDriver.send_angle(angle)
"""

from __future__ import annotations

import argparse
import csv
import logging
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2

from config.settings import (
    DANGER_MARGIN_PX,
    DANGER_NUDGE_DEG,
    CTRL_HYSTERESIS_LOW,
    CTRL_HYSTERESIS_HIGH,
    MAIN_CSV_LOG_FILE,
    MAIN_TARGET_HZ,
    MAIN_CAMERA_INDEX,
)
from drivers.jetson_servo import JetsonServoDriver
from models.robot_state import RobotState, FSMState
from runtime.overlay_drawer import OverlayDrawer
from unified_calibration_components import UnifiedCalibrator, CalibrationProcessingError
from vision.detector import LineDetector
from control.steering_controller import SteeringController

logger = logging.getLogger(__name__)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Jetson Nano direct calibration loop (self-calib + direct PWM)"
    )
    p.add_argument("--camera", type=int, default=MAIN_CAMERA_INDEX)
    p.add_argument("--hz", type=float, default=MAIN_TARGET_HZ)
    p.add_argument("--csv", type=str, default=MAIN_CSV_LOG_FILE)
    p.add_argument("--flip", action="store_true", default=False)
    p.add_argument("--servo-pin", type=int, default=int(os.getenv("SERVO_PIN", "33")))
    p.add_argument("--show-preview", action="store_true", default=False)
    return p


def _csv_fieldnames() -> list[str]:
    return [
        "frame_num", "mono_timestamp", "utc_timestamp",
        "loop_ms", "fsm_state", "calibration_active",
        "theta", "theta_source",
        "servo_angle", "servo_center_angle", "servo_offset",
        "pid_error", "pid_p_term", "pid_i_term", "pid_d_term",
    ]


def main() -> None:
    args = build_parser().parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

    # ------------------------------------------------------------------ #
    # Core algorithm
    # ------------------------------------------------------------------ #
    calibrator = UnifiedCalibrator(telemetry_enabled=False)
    state: RobotState = calibrator.robot_state
    controller: SteeringController = calibrator.steering_controller

    # ------------------------------------------------------------------ #
    # Jetson servo driver (direct PWM)
    # ------------------------------------------------------------------ #
    servo = JetsonServoDriver(pin=args.servo_pin)

    # ------------------------------------------------------------------ #
    # Camera
    # ------------------------------------------------------------------ #
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        logger.error("Camera index %d failed", args.camera)
        servo.close()
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
    csv_fh = csv_path.open("a", newline="", encoding="utf-8")
    csv_fields = _csv_fieldnames()
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
                break

            theta = calibration.observation_angle
            servo_angle = calibration.steering_angle
            fsm_state = calibration.control_state

            if theta is not None:
                last_known_theta = theta

            # --- servo ---
            servo.send_angle(servo_angle)

            # --- CSV ---
            elapsed_ms = (time.monotonic() - loop_start) * 1000.0
            pid_error = 0.0 if theta is None else float(theta) - 90.0
            csv_writer.writerow({
                "frame_num": frame_num,
                "mono_timestamp": f"{loop_start:.6f}",
                "utc_timestamp": datetime.now(timezone.utc).isoformat(),
                "loop_ms": f"{elapsed_ms:.4f}",
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
            })

            if frame_num % 30 == 0:
                csv_fh.flush()

            # --- preview ---
            if args.show_preview:
                rendered = calibration.debug_data.get("vision_debug", {}).get("grouped_vis")
                if rendered is not None:
                    cv2.imshow("Jetson Nano Calib", rendered)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break

            # --- sleep remainder ---
            remaining = target_period - (time.monotonic() - loop_start)
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    finally:
        servo.center()
        time.sleep(0.3)
        servo.close()
        cap.release()
        csv_fh.close()
        cv2.destroyAllWindows()
        logger.info("Shutdown complete")


if __name__ == "__main__":
    main()
