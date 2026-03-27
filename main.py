"""Entry point for the autonomous robot heading-hold system.

Orchestrates the 30 Hz control loop:

1. Capture a frame from the camera.
2. Detect the reference tile-gap angle via
   :class:`~vision.line_processor.LineProcessor`.
3. Compute the servo angle via :class:`~control.servo_pid.ServoPID`.
4. Send the angle to the servo via :class:`~drivers.servo_driver.ServoDriver`.

Error handling:
- Camera initialisation failure triggers an immediate emergency stop and exit.
- Any critical failure in the main loop centres the servo and exits cleanly.
- Per-frame hardware errors are logged but do not terminate the loop.
"""

import logging
import sys
import time

import cv2

from control.servo_pid import ServoPID
from drivers.servo_driver import ServoDriver
from models.robot_state import RobotState
from vision.line_processor import LineProcessor

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
_TARGET_HZ: float = 30.0
_LOOP_PERIOD: float = 1.0 / _TARGET_HZ
_CAMERA_INDEX: int = 0  # Default camera device index


def _init_camera(index: int) -> cv2.VideoCapture:
    """Open the camera device and validate it.

    Args:
        index: Camera device index (typically 0 on Jetson Nano).

    Returns:
        Opened :class:`cv2.VideoCapture` object.

    Raises:
        RuntimeError: If the camera cannot be opened.
    """
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open camera at index {index}.")
    logger.info("Camera initialised (index=%d).", index)
    return cap


def main() -> None:
    """Run the 30 Hz heading-hold control loop."""
    # ---------------------------------------------------------------------- #
    # Initialise shared state and subsystems
    # ---------------------------------------------------------------------- #
    state = RobotState()
    processor = LineProcessor()
    controller = ServoPID(state)
    servo = ServoDriver()

    # ---------------------------------------------------------------------- #
    # Camera initialisation – critical; abort on failure
    # ---------------------------------------------------------------------- #
    try:
        cap = _init_camera(_CAMERA_INDEX)
    except RuntimeError as exc:
        logger.critical("Camera initialisation failed: %s", exc)
        servo.center()
        sys.exit(1)

    # ---------------------------------------------------------------------- #
    # Main control loop at 30 Hz
    # ---------------------------------------------------------------------- #
    logger.info("Starting heading-hold control loop at %.0f Hz.", _TARGET_HZ)
    try:
        while True:
            loop_start = time.monotonic()

            # --- 1. Capture frame ----------------------------------------- #
            ret, frame = cap.read()
            if not ret:
                logger.warning("Frame capture failed; skipping cycle.")
                _sleep_remainder(loop_start)
                continue

            # --- 2. Vision: detect reference tile-gap angle --------------- #
            theta = processor.get_reference_angle(frame)
            logger.info(
                "timestamp=%.3f  theta=%s  state=%s",
                loop_start,
                f"{theta:.2f}°" if theta is not None else "None",
                state.fsm_state.name,
            )

            # --- 3. Control: compute servo angle -------------------------- #
            try:
                servo_angle = controller.update(theta)
            except Exception as ctrl_exc:  # noqa: BLE001
                logger.error(
                    "Controller error: %s – centering servo and stopping.",
                    ctrl_exc,
                )
                servo.center()
                break

            # --- 4. Servo: send angle command ----------------------------- #
            try:
                servo.send_angle(servo_angle)
            except OSError as hw_exc:
                logger.error(
                    "Servo hardware error: %s – centering servo and stopping.",
                    hw_exc,
                )
                servo.center()
                break

            # --- 5. Pace the loop ----------------------------------------- #
            _sleep_remainder(loop_start)

    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received – shutting down.")
    except Exception as fatal_exc:  # noqa: BLE001
        logger.critical("Fatal error: %s – centering servo and stopping.", fatal_exc)
        try:
            servo.center()
        except Exception as center_exc:  # noqa: BLE001
            logger.error("Failed to center servo during shutdown: %s", center_exc)
        raise
    finally:
        servo.center()
        cap.release()
        logger.info("Resources released. Goodbye.")


def _sleep_remainder(loop_start: float) -> None:
    """Sleep for the remainder of the target loop period.

    Args:
        loop_start: :func:`time.monotonic` timestamp at the start of the
            current loop iteration.
    """
    elapsed = time.monotonic() - loop_start
    remaining = _LOOP_PERIOD - elapsed
    if remaining > 0:
        time.sleep(remaining)
    else:
        logger.debug("Loop overrun by %.4f s.", -remaining)


if __name__ == "__main__":
    main()
