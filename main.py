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

from config.settings import (
    MAIN_CAMERA_INDEX,
    MAIN_CSV_LOG_FILE,
    MAIN_FLIP_FRAME,
    MAIN_TARGET_HZ,
)
from control.servo_pid import ServoPID
from drivers.servo_driver import ServoDriver
from models.robot_state import RobotState
from runtime.video_runtime_helpers import (
    init_camera,
    init_csv_logger,
    maybe_flip_frame,
    sleep_remainder,
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
_TARGET_HZ: float = MAIN_TARGET_HZ
_LOOP_PERIOD: float = 1.0 / _TARGET_HZ
_CAMERA_INDEX: int = MAIN_CAMERA_INDEX
_CSV_LOG_FILE: str = MAIN_CSV_LOG_FILE
_FLIP_FRAME: bool = MAIN_FLIP_FRAME
_CSV_FIELDNAMES = ["timestamp", "fsm_state", "theta", "servo_angle",
                   "pid_integral", "pid_last_error"]


def main() -> None:
    """Run the 30 Hz heading-hold control loop."""
    state = RobotState()
    detector = LineDetector(state)
    controller = ServoPID(state)
    servo = ServoDriver()
    csv_writer, csv_file = init_csv_logger(_CSV_LOG_FILE, _CSV_FIELDNAMES)

    try:
        cap = init_camera(_CAMERA_INDEX)
        logger.info("Camera initialised (index=%d).", _CAMERA_INDEX)
    except RuntimeError as exc:
        logger.critical("Camera initialisation failed: %s", exc)
        servo.center()
        sys.exit(1)

    logger.info("Starting heading-hold control loop at %.0f Hz.", _TARGET_HZ)
    try:
        while True:
            loop_start = time.monotonic()

            ret, frame = cap.read()
            if not ret:
                logger.warning("Frame capture failed; skipping cycle.")
                sleep_remainder(loop_start, _LOOP_PERIOD, logger)
                continue

            frame = maybe_flip_frame(frame, _FLIP_FRAME)

            theta = detector.get_reference_angle(frame)
            logger.info(
                "timestamp=%.3f  theta=%s  state=%s",
                loop_start,
                f"{theta:.2f} deg" if theta is not None else "None",
                state.fsm_state.name,
            )

            try:
                servo_angle = controller.update(theta)
            except Exception as ctrl_exc:  # noqa: BLE001
                logger.error(
                    "Controller error: %s - centering servo and stopping.",
                    ctrl_exc,
                )
                servo.center()
                break

            try:
                servo.send_angle(servo_angle)
            except OSError as hw_exc:
                logger.error(
                    "Servo hardware error: %s - centering servo and stopping.",
                    hw_exc,
                )
                servo.center()
                break

            csv_writer.writerow({
                "timestamp": f"{loop_start:.6f}",
                "fsm_state": state.fsm_state.name,
                "theta": f"{theta:.4f}" if theta is not None else "",
                "servo_angle": f"{servo_angle:.4f}",
                "pid_integral": f"{state.pid_integral:.6f}",
                "pid_last_error": f"{state.pid_last_error:.6f}",
            })

            sleep_remainder(loop_start, _LOOP_PERIOD, logger)

    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received - shutting down.")
    except Exception as fatal_exc:  # noqa: BLE001
        logger.critical("Fatal error: %s - centering servo and stopping.", fatal_exc)
        try:
            servo.center()
        except Exception as center_exc:  # noqa: BLE001
            logger.error("Failed to center servo during shutdown: %s", center_exc)
        raise
    finally:
        servo.center()
        cap.release()
        csv_file.close()
        logger.info("Resources released. Goodbye.")


if __name__ == "__main__":
    main()
