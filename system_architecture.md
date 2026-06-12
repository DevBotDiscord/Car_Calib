# Current Core Architecture

This document records the behavior present at Phase 1. It is a
characterization baseline, not the final modular architecture.

## Live Runtime

```text
camera frame
  -> vision.detector.LineDetector
  -> raw VP angle + intercept/debug dictionary
  -> control.steering_controller.SteeringController
  -> servo command
  -> runtime logging / overlay / stream / route integration
```

`main.py` owns camera recovery, control subscriptions, route sessions,
actuator integration, streaming, CSV/video output, and shutdown handling. The
core-refinement program does not change these behaviors during Phase 1.

## Offline Unified Runtime

```text
camera/video frame
  -> LineDetector.get_reference_angle_debug()
  -> VisionProcessor.process_frame()
  -> VisionProcessor._apply_geometric_filter()
  -> GeometryCalculator
  -> SteeringController
  -> TelemetryLogger
```

The current `UnifiedCalibrator` runs overlapping detector paths. Geometry and
steering use the `VisionProcessor` pair when available; `LineDetector` supplies
a fallback angle and legacy debug fields. The steering controller still
requires bottom intercepts, so an angle-only fallback results in `GAPPING`.

## Current Calibration Behavior

`VisionProcessor` crops the top `ROI_HEIGHT_PCT` of the frame, converts it to
grayscale, blurs it, runs Canny, and extracts probabilistic Hough segments. Its
geometric filter selects the longest positive-slope and negative-slope lines.

`GeometryCalculator` projects those lines to the frame bottom, calculates
their infinite-line intersection, and maps the vanishing-point x-coordinate
to a linear `0..180` angle proxy.

`SteeringController` applies this priority:

1. Missing angle/intercepts -> `GAPPING`, center output.
2. Left intercept inside the danger margin -> `DANGER_RIGHT`, fixed right nudge.
3. Right intercept inside the danger margin -> `DANGER_LEFT`, fixed left nudge.
4. Error within inner threshold -> `TRACKING_COAST`, center output.
5. Error past outer threshold -> enable `TRACKING_PD`.
6. While tracking remains active -> proportional plus frame-to-frame derivative.

The current controller does not use elapsed time or the configured integral
gain. That behavior is intentionally preserved in Phase 1.

## Current Responsibility Problems

- `UnifiedCalibrator` owns orchestration, rendering, logging, streaming, and
  loop timing.
- `TelemetryLogger` owns CSV, overlays, video, streaming, artifacts, and sleep.
- Live and offline processing use different orchestration paths.
- Geometry and detector behavior are duplicated.
- Components communicate through broad dictionaries.

The governing rules and target boundaries are in
[docs/architecture_governance.md](docs/architecture_governance.md).
