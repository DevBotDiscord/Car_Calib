"""Entry point for the autonomous robot heading-stability system.

Orchestrates the 30 Hz control loop:

1. Capture a frame from the camera.
2. Compute heading error via :class:`~vision.detector.HeadingDetector`.
3. Issue a motor command via :class:`~control.heading_controller.HeadingController`.
4. Translate the command to PWM signals via :class:`~drivers.motors.MotorDriver`.

Error handling:
- Camera initialisation failure triggers an immediate emergency stop and exit.
- Per-frame hardware errors are logged but do not terminate the loop.
"""

import logging
import sys
import time

import cv2

from control.heading_controller import HeadingController
from drivers.motors import MotorDriver
from models.state import FSMState, RobotState
from vision.detector import HeadingDetector

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
    """Run the 30 Hz heading-stability control loop."""
    # ---------------------------------------------------------------------- #
    # Initialise shared state and subsystems
    # ---------------------------------------------------------------------- #
    state = RobotState()
    detector = HeadingDetector()
    controller = HeadingController(state)
    motors = MotorDriver()

    # ---------------------------------------------------------------------- #
    # Camera initialisation – critical; abort on failure
    # ---------------------------------------------------------------------- #
    try:
        cap = _init_camera(_CAMERA_INDEX)
    except RuntimeError as exc:
        logger.critical("Camera initialisation failed: %s", exc)
        motors.stop()
        sys.exit(1)

    state.transition_to(FSMState.CALIBRATING)

    # ---------------------------------------------------------------------- #
    # Main control loop at 30 Hz
    # ---------------------------------------------------------------------- #
    logger.info("Starting control loop at %.0f Hz.", _TARGET_HZ)
    try:
        while True:
            loop_start = time.monotonic()

            # --- 1. Capture frame ----------------------------------------- #
            ret, frame = cap.read()
            if not ret:
                logger.warning("Frame capture failed; skipping cycle.")
                _sleep_remainder(loop_start)
                continue

            # --- 2. Vision: compute heading error -------------------------- #
            heading_error = detector.compute_heading_error(frame)
            logger.info(
                "timestamp=%.3f  heading_error=%s  state=%s",
                loop_start,
                f"{heading_error:.2f}°" if heading_error is not None else "None",
                state.fsm_state.name,
            )

            # --- 3. Control: compute PID output ---------------------------- #
            try:
                pid_output = controller.update(heading_error)
            except Exception as ctrl_exc:  # noqa: BLE001
                logger.error("Controller error: %s – applying emergency stop.", ctrl_exc)
                motors.stop()
                break

            logger.info(
                "pid_output=%.4f  last_valid_command=%.4f",
                pid_output,
                state.last_valid_command,
            )

            # --- 4. Motors: translate to PWM ------------------------------- #
            try:
                motors.set_pwm(pid_output)
            except OSError as hw_exc:
                logger.error("Motor hardware error: %s – applying emergency stop.", hw_exc)
                motors.stop()
                break

            # --- 5. Pace the loop ----------------------------------------- #
            _sleep_remainder(loop_start)

    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received – shutting down.")
    finally:
        motors.stop()
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
