# Unified Calibration Components

`UnifiedCalibrator` is the only calibration facade used by live and offline
entrypoints. A frame is processed once by the shared vision path.

## Current Components

- `ConfigManager`: reads grouped runtime settings.
- `VisionProcessor`: performs ROI, grayscale, blur, Canny, Hough extraction,
  and current opposite-sign pair selection.
- `GeometryCalculator`: calculates line intersection, bottom intercepts, and a
  linear vanishing-point angle.
- `SteeringController`: applies danger overrides, hysteresis, and current PD
  steering.
- `TelemetryLogger`: owns current CSV, overlay, debug panel, video, HTTPS frame
  publishing, artifacts, and loop sleep.
- `UnifiedCalibrator`: orchestrates one calibration computation.

## Public Calibration API

```python
result = calibrator.process_frame(frame, frame_num)
```

`process_frame()` processes vision once, computes geometry and steering once,
and returns a frozen `CalibrationResult`:

- `steering_angle: float`
- `control_state: str`
- `observation_angle: float | None`
- `calibration_active: bool`
- `telemetry: dict[str, Any]`
- `debug_data: dict[str, Any]`

The `vision_debug` entry inside `debug_data` contains unified vision
intermediates consumed by existing HUD helpers.

## Offline Compatibility API

```python
angle = calibrator.update(frame, frame_num)
```

`update()` calls `process_frame()` and then applies visualization, CSV, video,
stream, terminal-log, and timing side effects. It returns the steering angle
for compatibility with `process_video.py`.

## Current Pair Selection

The current filter rejects vertical and near-horizontal candidates, then
selects the longest negative-slope and positive-slope segments. The approved
next refinement will select the most opposite valid slopes and add explicit
geometry rejection reasons.

## Remaining Work

Later approved gates will split computation stages into focused modules,
replace broad dictionaries with stronger contracts, add plugins/capabilities,
refine validation and PID control, and separate telemetry sinks. See
[architecture_governance.md](architecture_governance.md).
