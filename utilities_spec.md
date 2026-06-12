# Core Utility Contracts

This document records the core utility contracts after calibration
centralization. Transport, Docker, MQTT, and servo utilities are outside this
refinement gate.

## Unified Calibration

Module: `unified_calibration_components`

### `CalibrationResult`

Frozen result returned by `UnifiedCalibrator.process_frame(frame, frame_num)`:

- `steering_angle: float`
- `control_state: str`
- `observation_angle: float | None`
- `calibration_active: bool`
- `telemetry: dict[str, Any]`
- `debug_data: dict[str, Any]`

Live and offline callers must consume this result rather than independently
recomputing calibration geometry or steering.

### `VisionProcessor.process_frame_debug`

Input: BGR or grayscale NumPy frame.

Output: `(lines, debug)` where `lines` is a list of integer `(x1, y1, x2, y2)`
segments and `debug` contains current visualization intermediates:

- `gray`
- `roi`
- `preprocessed`
- `edges`
- `hough_vis`
- `grouped_vis`
- `lines_count`
- `selected_lines`

The outer compatibility key used by existing HUD helpers is still named
`detector_debug`; it does not refer to a separate detector implementation.

### `VisionProcessor._apply_geometric_filter`

Current private behavior:

- rejects vertical lines;
- rejects lines below `VISION_MIN_ABS_SLOPE`;
- selects the longest negative-slope and positive-slope candidate;
- returns `None` unless both signs are available.

The approved next geometry refinement will select the most opposite valid
slopes and expose explicit rejection reasons.

### `GeometryCalculator`

- `calculate_vanishing_point(line1, line2) -> tuple[int, int] | None`
- `calculate_bottom_intercepts(line1, line2, frame_height) -> tuple[int, int]`
- `map_vp_to_angle(vp_x, frame_width) -> float`

These functions perform geometry only and do not select lines or steering
states.

### `UnifiedCalibrator.update`

Offline compatibility wrapper. It calls `process_frame()` exactly once, then
applies the current telemetry, visualization, video, streaming, and terminal
side effects. It returns only the final steering angle.

## Runtime Helpers

Module: `runtime.video_runtime_helpers`

The current helper module owns CLI builders, camera/video initialization, CSV
writers, overlays, the compatibility debug panel, and loop sleeping. These
responsibilities remain unchanged in this gate and are scheduled for later
separation.

Consumers of debug dictionaries must use `.get()` for optional values. Removed
legacy detector-quality fields are not part of new output.

## Telemetry

The current telemetry dictionary includes frame timing, line count, geometry,
state, steering, PID snapshots, and stream information. CSV consumers must
tolerate missing optional fields. Dynamic namespaced telemetry and JSONL are
planned for the dedicated telemetry gate.
