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
import json
import logging
import os
import sys
import time
import threading
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
from unified_calibration_components import UnifiedCalibrator, CalibrationProcessingError

logger = logging.getLogger("jetson")

def _env_int(names: tuple[str, ...], default: int) -> int:
    for name in names:
        value = os.getenv(name)
        if value is not None:
            return int(value)
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
class JetsonScriptRunner:
    """Runs route steps in a background thread using direct GPIO."""

    def __init__(self) -> None:
        self._thread: threading.Thread | None = None
        self._running = False
        self._steps: list[dict[str, Any]] = []
        self._current_step_idx: int = -1
        self._current_step: dict[str, Any] | None = None
        self._step_started_at: float | None = None
        self._last_error: str | None = None
        self._lock = threading.Lock()
        # callbacks set after init
        self._base_cb: Callable[[str], None] | None = None
        self._servo_cb: Callable[[float], None] | None = None
        self._relay_cb: Callable[[bool], None] | None = None
        self._center_angle = float(os.getenv("SERVO_CENTER_ANGLE", "-35"))
        self._max_steer = float(os.getenv("MAX_STEERING_OFFSET", "60"))
        self._republish_period_s = 1.0 / max(
            0.1,
            float(os.getenv("ROUTE_SCRIPT_REPUBLISH_HZ", "10")),
        )

    def set_handlers(
        self,
        base_cb: Callable[[str], None],
        servo_cb: Callable[[float], None],
        relay_cb: Callable[[bool], None],
    ) -> None:
        self._base_cb = base_cb
        self._servo_cb = servo_cb
        self._relay_cb = relay_cb

    @property
    def is_running(self) -> bool:
        return self._running

    def status(self) -> dict[str, Any]:
        with self._lock:
            elapsed = time.time() - self._step_started_at if self._step_started_at else 0.0
            return {
                "running": self._running,
                "steps": list(self._steps),
                "current": self._current_step,
                "current_idx": self._current_step_idx,
                "current_step": self._current_step_idx + 1 if self._running else 0,
                "total": len(self._steps),
                "step": self._current_step,
                "step_elapsed_s": elapsed,
                "last_error": self._last_error,
            }

    def submit(self, steps: list[dict[str, Any]]) -> bool:
        if not steps:
            return False
        with self._lock:
            self._steps = list(steps)
        self.stop()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        self._running = False
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2)

    def _run(self) -> None:
        self._running = True
        self._last_error = None
        try:
            for idx, step in enumerate(self._steps):
                if not self._running:
                    break
                with self._lock:
                    self._current_step_idx = idx
                    self._current_step = dict(step)
                    self._step_started_at = time.time()
                self._execute_step(step)
        except Exception as exc:
            self._last_error = str(exc)
            logger.exception("Route script failed")
        finally:
            with self._lock:
                self._running = False
                self._current_step = None
                self._current_step_idx = -1
                self._step_started_at = None
            # Stop base
            if self._base_cb:
                self._base_cb("STOP")
            # Center servo
            if self._servo_cb:
                self._servo_cb(float(os.getenv("SERVO_CENTER_ANGLE", "-35")))

    def _execute_step(self, step: dict[str, Any]) -> None:
        action = str(step.get("action", "stop")).lower().replace(" ", "_")
        duration = max(0.0, float(step.get("duration_s", step.get("duration", 0))))
        logger.info("Script step %d: %s (%.1fs)", self._current_step_idx, action, duration)

        base_cmd = "STOP"
        servo_angle = self._center_angle

        if action in ("forward", "straight"):
            base_cmd = "FORWARD"
        elif action in ("backward",):
            base_cmd = "BACKWARD"
        elif action in ("left",):
            base_cmd = "FORWARD"
            servo_angle = self._center_angle + self._max_steer
        elif action in ("right",):
            base_cmd = "FORWARD"
            servo_angle = self._center_angle - self._max_steer
        elif action in ("turn_left",):
            base_cmd = "TURN_LEFT"
            servo_angle = None
        elif action in ("turn_right",):
            base_cmd = "TURN_RIGHT"
            servo_angle = None
        elif action in ("stop", "pause"):
            base_cmd = "STOP"

        if self._base_cb:
            self._base_cb(base_cmd)
        if servo_angle is not None and self._servo_cb:
            self._servo_cb(servo_angle)

        if duration > 0:
            # Sleep in small chunks to allow stop
            deadline = time.time() + duration
            while self._running and time.time() < deadline:
                if servo_angle is not None and self._servo_cb:
                    self._servo_cb(servo_angle)
                time.sleep(self._republish_period_s)


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

    home_offset_deg: offset calibration của home
                    ví dụ home lệch -8 độ thì servo home = 90 + (-8) = 82

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

    # Đảo chiều nếu cần
    if reverse:
        ratio = -ratio

    # Map ratio ra góc servo thật
    servo_cmd = home_cmd + ratio * limit_deg

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
        summary_file = d / "summary.json"
        info: dict[str, Any] = {
            "route_id": d.name,
            "route_mode": "",
            "preset": "",
            "status": "not_recorded",
            "frames": 0,
            "elapsed": "0s",
            "zip_size": "—",
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
                    "elapsed": s.get("elapsed", "0s"),
                    "ended_utc": s.get("ended_utc", ""),
                })
            except Exception:
                pass
        # Check for zip
        zip_path = d / f"{d.name}.zip"
        if zip_path.is_file():
            sz = zip_path.stat().st_size
            info["zip_size"] = f"{sz/1024:.0f} KB" if sz < 1024*1024 else f"{sz/1024/1024:.1f} MB"
        routes.append(info)
    return routes


def main() -> None:
    args = build_parser().parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

    # ------------------------------------------------------------------ #
    # Core algorithm
    # ------------------------------------------------------------------ #
    calibrator = UnifiedCalibrator(telemetry_enabled=True)
    state: RobotState = calibrator.robot_state
    controller = calibrator.steering_controller

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
            tel = dict(shared_telemetry)
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
    if not args.no_dashboard:
        http = JetsonHttpServer(host=args.host, port=args.port)
        http.set_frame_getter(_frame_getter)
        http.set_status_getter(_status_getter)
        http.set_base_handler(_base_handler)
        http.set_relay_handler(_relay_handler)
        http.set_power_handler(_power_handler)
        # Route script runner + presets
        _presets: dict[str, list[dict[str, Any]]] = {}
        _steps: list[dict[str, Any]] = []
        script_runner = JetsonScriptRunner()
        script_runner.set_handlers(_base_handler, servo.send_angle, _relay_handler)

        http.set_script_runner(lambda: script_runner.status())
        http.set_script_stopper(script_runner.stop)
        http.set_script_submitter(lambda body: script_runner.submit(json.loads(body).get("steps", [])))
        http.set_steps_getter(lambda: _steps)
        http.set_steps_setter(lambda body: _steps.extend(json.loads(body).get("steps", [])))
        http.set_presets_getter(lambda: [{"name": k, "steps": v, "steps_count": len(v)} for k, v in _presets.items()])
        http.set_presets_setter(lambda body: (d := json.loads(body), _presets.update({d.get("name", "untitled"): d.get("steps", [])})))
        http.set_preset_deleter(lambda name: _presets.pop(name, None))
        http.set_routes_getter(_scan_routes)

        http.start()

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
    # Telemetry (CSV + stream) handled by UnifiedCalibrator internals
    # ------------------------------------------------------------------ #

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

            # --- servo ---
            output_angle = map_calibrated_servo(
                servo_angle,
                home_offset_deg=state.servo_center_angle,
                limit_deg=_env_int(("MAX_STEERING_OFFSET",), 60),
                reverse=_env_bool("SERVO_REVERSE", False),
            )
            if script_runner is None or not script_runner.is_running:
                servo.send_angle(output_angle)
            final_angle = output_angle
            if frame_num % 10 == 1:
                logger.info(
                    "frame=%d state=%s vp=%.1f° steer=%.1f° loop=%.0fms",
                    frame_num, fsm_state,
                    theta if theta is not None else float('nan'),
                    final_angle, loop_ms,
                )
            
            # --- telemetry ---
            loop_ms = (time.monotonic() - loop_start) * 1000.0
            pid_error = 0.0 if theta is None else float(theta) - 90.0

            # Build dashboard telemetry, merging calibrator output with hardware state
            tel = dict(calibration.telemetry)
            tel.update({
                "source": hardware_source,
                "rpi_online": True,
                "mqtt_connected": True,
                "estop_active": False,
                "steer_angle": f"{final_angle:.1f}",
                "current_route_mode": "AUTO",
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
            _update_shared(display_frame, tel)

            # --- CSV telemetry (delegated to calibrator) ---
            calibration.telemetry.update({"final_servo_angle": final_angle})
            if calibrator._telemetry is not None:
                calibrator._telemetry.log_state(frame_num, calibration.telemetry)

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
        if http is not None:
            http.stop()
        cv2.destroyAllWindows()
        logger.info("Shutdown complete")


if __name__ == "__main__":
    main()
