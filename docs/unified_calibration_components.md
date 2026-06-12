# Unified Calibration Components: Phase 1 Baseline

`unified_calibration_components.py` currently combines calibration
computation, runtime orchestration, visualization, logging, streaming, and
loop timing. Phase 1 documents and characterizes this behavior without
changing it.

## Current Components

- `ConfigManager` dynamically reads grouped runtime settings.
- `VisionProcessor` crops the top ROI, applies grayscale/blur/Canny/Hough, and
  selects the longest line from each slope sign.
- `GeometryCalculator` calculates line intersection, bottom intercepts, and a
  linear vanishing-point angle.
- `TelemetryLogger` owns CSV, overlay, debug panel, video, HTTPS frame
  publishing, run artifact layout, and loop sleep.
- `UnifiedCalibrator` runs the detector paths, geometry, steering, telemetry,
  preview, camera capture, and timing.

## Current `UnifiedCalibrator.update`

For each frame, `update(frame, frame_num)`:

1. Calls `LineDetector.get_reference_angle_debug`.
2. Calls `VisionProcessor.process_frame`.
3. Calls the private `VisionProcessor._apply_geometric_filter`.
4. Calculates bottom intercepts and vanishing point when a pair exists.
5. Falls back to the `LineDetector` angle only when geometry has no VP angle.
6. Calls `SteeringController.compute_steering`.
7. Builds a fixed telemetry dictionary.
8. Renders, logs, writes video, publishes a frame, and emits terminal status.
9. Returns only the final steering angle.

This overlapping detector behavior is locked by Phase 1 characterization
tests and is scheduled for removal in Phase 3.

## Current Inputs And Outputs

Input:

- BGR or grayscale NumPy frame.
- Integer frame number.

Output:

- Final steering angle as `float`.

Side effects:

- CSV logging.
- Optional debug video and HTTPS frame publishing.
- Periodic terminal logging.
- Updates the internal last PID error and rendered frame.

## Target Direction

Later approved phases will split the classes into computation-only stages,
return typed calibration results, remove `LineDetector`, add plugin/capability
contracts, and separate telemetry sinks. See
[architecture_governance.md](architecture_governance.md).
