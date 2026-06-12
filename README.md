# Car Calibration

Vision-based steering calibration for an autonomous RC car. A MiniPC processes
camera frames, computes a steering command, and publishes it to the vehicle
control path. The Raspberry Pi bridge, MQTT transport, servo communication,
and deployment stack are outside the current core-architecture refinement.

## Current Integrated Calibration Method

The active calibration method is a lane-pair and vanishing-point controller:

1. Crop the configured top portion of the BGR camera frame.
2. Convert the ROI to grayscale, apply Gaussian blur, and run Canny.
3. Extract line segments with probabilistic Hough transform.
4. Select the longest negative-slope and positive-slope segments.
5. Project both lines to the bottom of the frame and calculate their
   intersection as the vanishing point.
6. Map the vanishing-point x-coordinate linearly into an angle centered at
   `90` degrees.
7. Apply danger-zone overrides, tracking hysteresis, and a PD steering
   correction.

Current steering states are:

| State | Current behavior |
| --- | --- |
| `GAPPING` | Required vision geometry is missing; return center steering. |
| `DANGER_RIGHT` | Left bottom intercept crossed its margin; nudge right. |
| `DANGER_LEFT` | Right bottom intercept crossed its margin; nudge left. |
| `TRACKING_COAST` | Error is inside hysteresis or tracking is not active; return center. |
| `TRACKING_PD` | Apply the current proportional-derivative correction. |

`PID_KI` is exposed by configuration but is not used by the current PD
controller. True PID control is planned for a later approved refinement phase.

## Current Runtime Paths

The repository is transitional:

- `main.py` is the live MiniPC runtime. It currently calls
  `vision.detector.LineDetector` and `control.steering_controller.SteeringController`
  directly.
- `process_video.py` is the offline entrypoint. It uses
  `UnifiedCalibrator` from `unified_calibration_components.py`.
- `UnifiedCalibrator` currently runs both `LineDetector` and `VisionProcessor`
  on every frame. This overlapping behavior is intentionally characterized in
  Phase 1 and will be removed only in a later approved phase.

The target architecture and delivery rules are documented in
[Core Architecture Governance](docs/architecture_governance.md).

## Core Repository Map

```text
main.py                              Live camera/runtime orchestration
process_video.py                     Offline video entrypoint
unified_calibration_components.py    Current unified facade and mixed utilities
control/steering_controller.py       Current danger/hysteresis/PD controller
vision/detector.py                   Overlapping detector scheduled for removal
models/robot_state.py                Current control state and PID constants
runtime/overlay_drawer.py            Calibration visualization
runtime/video_runtime_helpers.py     CSV, video, camera, CLI, and timing helpers
tests/                               Pytest tests and characterization coverage
```

## Running

Install dependencies:

```bash
pip install -r requirements.txt
pip install -r requirements-dev.txt
```

Live runtime:

```bash
python main.py
```

Offline video processing:

```bash
python process_video.py --input videos/example.mp4 --output processed_video.mp4
```

Useful offline options:

```text
--input PATH
--output PATH
--csv-output PATH
--send-to-servo
--no-send-to-servo
```

Runtime and calibration defaults are environment-backed through
`config/settings.py`; see `.env.example`.

## Current Telemetry

The current implementation writes CSV telemetry containing runtime timing,
detector/debug fields, vanishing-point geometry, steering state, steering
command, and PID snapshots. The current schema is fixed in both `main.py` and
`TelemetryLogger`.

Known current limitations:

- Live and offline runtime paths are not yet unified.
- `UnifiedCalibrator` processes overlapping detector paths.
- Some detector quality fields are diagnostic rather than effective gates.
- Telemetry, visualization, streaming, and loop timing are mixed in
  `TelemetryLogger`.
- CSV output is not yet a projection of fully dynamic telemetry.

These limitations are documented rather than changed in Phase 1.

## Tests

Run the full suite:

```bash
python -m pytest tests -q
```

Run Phase 1 unified-calibration characterization tests:

```bash
python -m pytest tests/test_unified_calibration_characterization.py -q
```

The refinement program requires implementation, tests, documentation, one
focused commit, and user verification before the next phase starts.
